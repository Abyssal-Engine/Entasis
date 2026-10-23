package large_world_precision

import "core:fmt"
import "core:math"
import entasis "entasis:entasis"
import util "entasis:entasis_utilities"
import support "../benchmark_support"

Replay_Group :: struct
{
	world: entasis.World,
	frame: entasis.Origin_Frame,
	policy: entasis.Default_Narrow_Policy,
	state: Owner_State,
	query: entasis.Query_Context,
	query_state: Owner_State,
	bodies: [DISTRIBUTED_BODY_COUNT]entasis.Body_Handle,
	collidables: [DISTRIBUTED_BODY_COUNT]entasis.Collidable_Reference,
	body_index_by_handle: [DISTRIBUTED_BODY_COUNT]i32,
	initial_positions: [DISTRIBUTED_BODY_COUNT]entasis.Global_Position,
	exported_positions: [DISTRIBUTED_BODY_COUNT]entasis.Global_Position,
	bounds_min: [3]f64,
	bounds_max: [3]f64,
}

Replay_Owner :: struct
{
	regions: []Replay_Group,
	pool: entasis.Buffer_Pool,
	pool_state: Owner_State,
	workers: entasis.Thread_Pool,
	worker_state: Owner_State,
	dispatcher: ^entasis.Dispatcher,
	worker_interface: entasis.Dispatcher_Interface,
	options: support.Options,
	precision_case: Precision_Case,
	export_checksum: f64,
	frame_median_ms: f64,
	frame_max_ms: f64,
	sampling_mode: Sampling_Mode,
	memory: ^Precision_Memory,
	ray_scratch: [DISTRIBUTED_BODY_COUNT]entasis.Ray_Hit,
}

replay_world_description :: proc(
	region: ^Replay_Group,
	dispatcher: ^entasis.Dispatcher,
) -> entasis.World_Description
{
	description := entasis.world_description_default();
	description.gravity = {0, -10, 0};
	description.damping = {};
	description.capacity = {
		bodies=DISTRIBUTED_BODY_COUNT,
		statics=1,
		inactive_body_sets=1,
		shapes_per_type=2,
		constraints=6144,
		initial_constraints_per_type_batch=64,
		minimum_constraints_per_body=8,
		broad_phase_candidates=6144,
		pairs=6144,
		collision_child_pairs=i32(dispatcher.worker_count),
		inactive_pairs=1,
		pending_pairs_per_worker=4096,
	};
	description.solve = {velocity_iterations=1, substeps=4, fallback_batch_threshold=64};
	description.threading = {external_dispatcher=dispatcher};
	description.narrow_callbacks = entasis.narrow_policy_default(&region.policy);
	return description;
}

replay_world_create :: proc(
	region: ^Replay_Group,
	origin: entasis.Global_Position,
	pool: ^entasis.Buffer_Pool,
	dispatcher: ^entasis.Dispatcher,
	memory: ^Precision_Memory = nil,
) -> entasis.Status
{
	status: entasis.Status;
	region.frame, status = entasis.origin_frame_at(origin);
	if status != .Ok
	{
		return status;
	}
	region.policy = {material=entasis.contact_material(0.5, 2, entasis.spring_settings(30, 1))};
	status = precision_world_initialize(&region.world, replay_world_description(region, dispatcher), pool, memory);
	if status != .Ok
	{
		return status;
	}
	region.state = .Ready;
	floor_shape, floor_status := entasis.shape_add(&region.world, entasis.box(48, 1, 24));
	if floor_status != .Ok
	{
		return floor_status;
	}
	_, status = entasis.static_add(&region.world, entasis.static_body(floor_shape, entasis.pose({0, -0.5, 0})));
	if status != .Ok
	{
		return status;
	}
	shape, shape_status := entasis.shape_add(&region.world, entasis.box(1, 1, 1));
	if shape_status != .Ok
	{
		return shape_status;
	}
	inertia, inertia_status := entasis.shape_registered_inertia(&region.world, shape, 1);
	if inertia_status != .Ok
	{
		return inertia_status;
	}
	description := entasis.body_dynamic(shape, inertia, entasis.pose(), {}, entasis.body_activity(-1, 32));
	description.collidable.maximum_speculative_margin = f32(math.F32_MAX);
	// these bounds enclose the entire immutable query fixture, including its floor
	// simulation never uses them to prune moving bodies
	region.bounds_min = {-24, -1, -12};
	region.bounds_max = {24, 0, 12};
	for index in 0 ..< DISTRIBUTED_BODY_COUNT
	{
		global := distributed_fixture_position(region.frame, index);
		local: entasis.Vector3;
		local, status = entasis.origin_frame_to_local(region.frame, global);
		if status != .Ok
		{
			return status;
		}
		description.pose.position = local;
		region.initial_positions[index] = global;
		region.bodies[index], status = entasis.body_add(&region.world, description);
		if status != .Ok
		{
			return status;
		}
		handle := region.bodies[index];
		// the fresh, deletion-free fixture has a dense handle range. admit that
		// range once before using the table to validate unordered query output
		if handle.value < 0 || handle.value >= DISTRIBUTED_BODY_COUNT
		{
			return .Invalid_Description;
		}
		region.body_index_by_handle[handle.value] = i32(index);
		region.collidables[index], status = entasis.body_collidable_reference(&region.world, handle);
		if status != .Ok
		{
			return status;
		}
		region.bounds_max[1] = max(region.bounds_max[1], f64(local.y)+0.5);
		reconstructed: entasis.Global_Position;
		reconstructed, status = entasis.origin_frame_to_global(region.frame, local);
		if status != .Ok || precision_global_close(reconstructed, global, POSITION_TOLERANCE) != .Accepted
		{
			return .Invalid_Description;
		}
	}
	return .Ok;
}

replay_worlds_create :: proc(owner: ^Replay_Owner) -> support.Benchmark_Result
{
	for &region, index in owner.regions
	{
		origin := entasis.global_position(DISTRIBUTED_OFFSET+f64(index)*DISTRIBUTED_SEPARATION, 0, 0);
		status := replay_world_create(&region, origin, &owner.pool, owner.dispatcher, owner.memory);
		if status != .Ok
		{
			return {status=.Fixture_Failed, physics_status=status};
		}
	}
	return {status=.Ok};
}

replay_world_destroy :: proc(region: ^Replay_Group) -> entasis.Status
{
	status := entasis.Status.Ok;
	if region.query_state == .Ready
	{
		status = entasis.query_context_destroy(&region.query);
	}
	if region.state == .Ready
	{
		world_status := entasis.world_destroy(&region.world);
		if status == .Ok
		{
			status = world_status;
		}
	}
	region^ = {};
	return status;
}

replay_worlds_destroy :: proc(owner: ^Replay_Owner) -> entasis.Status
{
	status := entasis.Status.Ok;
	for &region in owner.regions
	{
		world_status := replay_world_destroy(&region);
		if status == .Ok
		{
			status = world_status;
		}
	}
	return status;
}

replay_owner_initialize :: proc(
	owner: ^Replay_Owner,
	regions: []Replay_Group,
	options: support.Options,
	precision_case: Precision_Case,
	memory: ^Precision_Memory = nil,
) -> support.Benchmark_Result
{
	owner.regions = regions;
	owner.options = options;
	owner.memory = memory;
	owner.precision_case = precision_case;
	status := precision_pool_initialize(&owner.pool, memory);
	if status != .Ok
	{
		return {status=.Allocation_Failed, physics_status=status};
	}
	owner.pool_state = .Ready;
	worker_status := precision_workers_initialize(&owner.workers, options.worker_count, memory);
	if worker_status != .Ok
	{
		fmt.eprintfln("replay_worker_initialization_failed status=%v", worker_status);
		return {status=.Creation_Failed};
	}
	owner.worker_state = .Ready;
	owner.worker_interface = entasis.dispatcher_from_thread_pool(&owner.workers);
	owner.dispatcher = util.thread_dispatcher_boundary(&owner.workers);
	return replay_worlds_create(owner);
}

replay_owner_destroy :: proc(owner: ^Replay_Owner) -> entasis.Status
{
	status := replay_worlds_destroy(owner);
	if owner.worker_state == .Ready
	{
		worker_status := entasis.thread_pool_destroy(&owner.workers);
		if worker_status != .Ok
		{
			fmt.eprintfln("replay_worker_release_failed status=%v", worker_status);
			if status == .Ok
			{
				status = .Invalid_Description;
			}
		}
	}
	if owner.pool_state == .Ready
	{
		pool_status := entasis.buffer_pool_destroy(&owner.pool);
		if status == .Ok
		{
			status = pool_status;
		}
	}
	owner^ = {};
	return status;
}

