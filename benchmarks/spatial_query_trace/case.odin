package spatial_query_trace

import "base:runtime"
import "core:time"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

CASE_ID :: "spatial_query_trace";
STATIC_GRID_X :: 25;
STATIC_GRID_Y :: 16;
STATIC_GRID_Z :: 25;
STATIC_BODY_COUNT :: STATIC_GRID_X * STATIC_GRID_Y * STATIC_GRID_Z;
RAY_QUERY_COUNT :: 50_000;
SPHERE_CAST_QUERY_COUNT :: 25_000;
OVERLAP_QUERY_COUNT :: 25_000;
QUERY_COUNT_PER_BATCH :: RAY_QUERY_COUNT + SPHERE_CAST_QUERY_COUNT + OVERLAP_QUERY_COUNT;
WARMUP_BATCH_COUNT :: 10;
MEASURED_BATCH_COUNT :: 100;
STATIC_HALF_EXTENT :: f32(0.5);
STATIC_SPACING :: f32(2.0);
STATIC_BASE_CENTER_X :: f32(0.0);
STATIC_BASE_CENTER_Y :: f32(0.0);
STATIC_BASE_CENTER_Z :: f32(0.0);
SPHERE_CAST_RADIUS :: f32(0.5);
OVERLAP_HALF_EXTENT :: f32(0.25);
QUERY_DISTANCE :: f32(64.0);
MISS_OFFSET :: f32(4.0);
POOL_MINIMUM_BLOCK_SIZE :: 65_536;
WORKER_POOL_BLOCK_SIZE :: 65_536;
SHAPES_PER_TYPE :: 4;
EXPECTED_RAY_HITS :: RAY_QUERY_COUNT / 2;
EXPECTED_SPHERE_CAST_HITS :: SPHERE_CAST_QUERY_COUNT / 2;
EXPECTED_OVERLAP_HITS :: OVERLAP_QUERY_COUNT / 2;
MAXIMUM_WORKER_COUNT :: entasis.MAXIMUM_WORKER_COUNT;

#assert(STATIC_BODY_COUNT == 10_000);
#assert(QUERY_COUNT_PER_BATCH == 100_000);
#assert(RAY_QUERY_COUNT % 2 == 0);
#assert(SPHERE_CAST_QUERY_COUNT % 2 == 0);
#assert(OVERLAP_QUERY_COUNT % 2 == 0);

Case_Status :: enum u8
{
	Ok,
	Invalid_Argument,
	Allocation_Failed,
	Creation_Failed,
	Fixture_Failed,
	Query_Failed,
	Validation_Failed,
	Release_Failed,
}

Owner_State :: enum u8
{
	Empty,
	Ready,
	Released,
}

Query_Family :: enum u8
{
	Ray,
	Sphere_Cast,
	Overlap,
}

Execution_Mode :: enum u8
{
	Timed,
	Validate,
}

Ray_Input :: struct
{
	ray: physics.Tree_Ray,
}

Sphere_Cast_Input :: struct
{
	pose:     physics.Rigid_Pose,
	velocity: physics.Body_Velocity,
}

Overlap_Input :: struct
{
	bounds: util.Bounding_Box,
}

Lane_Result :: struct #align(128)
{
	hit_count: u64,
	checksum:  u64,
	status:    physics.Physics_Status,
}

#assert(align_of(Lane_Result) == 128);
#assert(size_of(Lane_Result) == 128);

Family_Result :: struct
{
	hit_count: u64,
	checksum:  u64,
	status:    physics.Physics_Status,
}

Batch_Result :: struct
{
	ray_hits:            u64,
	sphere_cast_hits:    u64,
	overlap_hits:        u64,
	ray_checksum:        u64,
	sphere_cast_checksum: u64,
	overlap_checksum:    u64,
	ray_nanoseconds:     u64,
	sphere_nanoseconds:  u64,
	overlap_nanoseconds: u64,
	status:              Case_Status,
	physics_status:      physics.Physics_Status,
}

Benchmark_Result :: struct
{
	status:               Case_Status,
	physics_status:       physics.Physics_Status,
	ray_hits:             u64,
	sphere_cast_hits:     u64,
	overlap_hits:         u64,
	ray_checksum:         u64,
	sphere_cast_checksum: u64,
	overlap_checksum:     u64,
	ray_nanoseconds:      u64,
	sphere_nanoseconds:   u64,
	overlap_nanoseconds:  u64,
}

Benchmark_Owner :: struct
{
	world:              entasis.World,
	state:              Owner_State,
	worker_count:       int,
	simulation:         ^physics.Simulation,
	owner_pool:         ^util.Buffer_Pool,
	dispatcher:         ^util.Thread_Dispatcher_Boundary,
	sphere_shape:       physics.Typed_Index,
	ray_inputs:         []Ray_Input,
	sphere_cast_inputs: []Sphere_Cast_Input,
	overlap_inputs:     []Overlap_Input,
	lanes:              [MAXIMUM_WORKER_COUNT]Lane_Result,
	observations:       Query_Observations,
}

Dispatch_Context :: struct
{
	owner:  ^Benchmark_Owner,
	family: Query_Family,
	mode:   Execution_Mode,
}

centered_grid_coordinate :: #force_inline proc "contextless" (
	base_center, spacing: f32,
	coordinate, count: int,
) -> f32
{
	return base_center + spacing * (f32(coordinate) - 0.5 * f32(count - 1));
}

uncentered_grid_coordinate :: #force_inline proc "contextless" (
	base_center, spacing: f32,
	coordinate: int,
) -> f32
{
	return base_center + spacing * f32(coordinate);
}

vector_component :: #force_inline proc "contextless" (
	value: util.Vector3,
	component: int,
) -> f32
{
	if component == 0
	{
		return value.x;
	}
	if component == 1
	{
		return value.y;
	}
	return value.z;
}

vector_set_component :: #force_inline proc "contextless" (
	value: util.Vector3,
	component: int,
	component_value: f32,
) -> util.Vector3
{
	result := value;
	if component == 0
	{
		result.x = component_value;
	}
	else if component == 1
	{
		result.y = component_value;
	}
	else
	{
		result.z = component_value;
	}
	return result;
}

query_generate :: proc "contextless" (global_index: int) -> (util.Vector3, util.Vector3)
{
	family_index := global_index;
	if global_index >= RAY_QUERY_COUNT + SPHERE_CAST_QUERY_COUNT
	{
		family_index -= RAY_QUERY_COUNT + SPHERE_CAST_QUERY_COUNT;
	}
	else if global_index >= RAY_QUERY_COUNT
	{
		family_index -= RAY_QUERY_COUNT;
	}
	sample := family_index / 2;
	intended_hit := (family_index & 1) == 0;
	slot := sample % STATIC_BODY_COUNT;
	ix := slot % STATIC_GRID_X;
	iz := (slot / STATIC_GRID_X) % STATIC_GRID_Z;
	iy := slot / (STATIC_GRID_X * STATIC_GRID_Z);
	center := util.Vector3{
		centered_grid_coordinate(STATIC_BASE_CENTER_X, STATIC_SPACING, ix, STATIC_GRID_X),
		uncentered_grid_coordinate(STATIC_BASE_CENTER_Y, STATIC_SPACING, iy),
		centered_grid_coordinate(STATIC_BASE_CENTER_Z, STATIC_SPACING, iz, STATIC_GRID_Z),
	};
	scene_minimum := util.Vector3{
		centered_grid_coordinate(STATIC_BASE_CENTER_X, STATIC_SPACING, 0, STATIC_GRID_X) - STATIC_HALF_EXTENT,
		STATIC_BASE_CENTER_Y - STATIC_HALF_EXTENT,
		centered_grid_coordinate(STATIC_BASE_CENTER_Z, STATIC_SPACING, 0, STATIC_GRID_Z) - STATIC_HALF_EXTENT,
	};
	scene_maximum := util.Vector3{
		centered_grid_coordinate(STATIC_BASE_CENTER_X, STATIC_SPACING, STATIC_GRID_X - 1, STATIC_GRID_X) + STATIC_HALF_EXTENT,
		uncentered_grid_coordinate(STATIC_BASE_CENTER_Y, STATIC_SPACING, STATIC_GRID_Y - 1) + STATIC_HALF_EXTENT,
		centered_grid_coordinate(STATIC_BASE_CENTER_Z, STATIC_SPACING, STATIC_GRID_Z - 1, STATIC_GRID_Z) + STATIC_HALF_EXTENT,
	};
	face := sample % 6;
	axis := face / 2;
	if global_index >= RAY_QUERY_COUNT + SPHERE_CAST_QUERY_COUNT
	{
		overlap_center := center;
		if !intended_hit
		{
			overlap_center = vector_set_component(
				overlap_center,
				axis,
				vector_component(scene_maximum, axis) + MISS_OFFSET,
			);
		}
		return overlap_center, {};
	}
	face_sign := -1;
	if (face & 1) != 0
	{
		face_sign = 1;
	}
	first_transverse := 0;
	if axis == 0
	{
		first_transverse = 1;
	}
	origin, direction: util.Vector3;
	for component in 0 ..< 3
	{
		if component == axis
		{
			coordinate := vector_component(scene_minimum, component) - 5;
			direction_component := f32(1);
			if face_sign > 0
			{
				coordinate = vector_component(scene_maximum, component) + 5;
				direction_component = -1;
			}
			origin = vector_set_component(origin, component, coordinate);
			direction = vector_set_component(direction, component, direction_component);
		}
		else
		{
			coordinate := vector_component(center, component);
			if !intended_hit && component == first_transverse
			{
				coordinate = vector_component(scene_maximum, component) + MISS_OFFSET;
			}
			origin = vector_set_component(origin, component, coordinate);
		}
	}
	return origin, direction;
}

query_inputs_create :: proc(owner: ^Benchmark_Owner) -> Case_Status
{
	if owner == nil
	{
		return .Invalid_Argument;
	}
	owner.ray_inputs = make([]Ray_Input, RAY_QUERY_COUNT);
	owner.sphere_cast_inputs = make([]Sphere_Cast_Input, SPHERE_CAST_QUERY_COUNT);
	owner.overlap_inputs = make([]Overlap_Input, OVERLAP_QUERY_COUNT);
	if owner.ray_inputs == nil || owner.sphere_cast_inputs == nil || owner.overlap_inputs == nil
	{
		return .Allocation_Failed;
	}
	for index in 0 ..< RAY_QUERY_COUNT
	{
		origin, direction := query_generate(index);
		owner.ray_inputs[index] = {
			ray={origin=origin, direction=direction, maximum_t=QUERY_DISTANCE},
		};
	}
	for index in 0 ..< SPHERE_CAST_QUERY_COUNT
	{
		origin, direction := query_generate(RAY_QUERY_COUNT + index);
		owner.sphere_cast_inputs[index] = {
			pose=entasis.pose(origin),
			velocity=entasis.velocity(direction),
		};
	}
	half_extent := util.Vector3{OVERLAP_HALF_EXTENT, OVERLAP_HALF_EXTENT, OVERLAP_HALF_EXTENT};
	for index in 0 ..< OVERLAP_QUERY_COUNT
	{
		center, _ := query_generate(RAY_QUERY_COUNT + SPHERE_CAST_QUERY_COUNT + index);
		owner.overlap_inputs[index] = {
			bounds={
				min=util.vector3_subtract(center, half_extent),
				max=util.vector3_add(center, half_extent),
			},
		};
	}
	return .Ok;
}

query_inputs_destroy :: proc(owner: ^Benchmark_Owner)
{
	if owner == nil
	{
		return;
	}
	if owner.ray_inputs != nil
	{
		delete(owner.ray_inputs);
	}
	if owner.sphere_cast_inputs != nil
	{
		delete(owner.sphere_cast_inputs);
	}
	if owner.overlap_inputs != nil
	{
		delete(owner.overlap_inputs);
	}
	owner.ray_inputs = nil;
	owner.sphere_cast_inputs = nil;
	owner.overlap_inputs = nil;
}

benchmark_world_description :: proc(worker_count: int) -> entasis.World_Description
{
	description := entasis.world_description_default();
	description.gravity = {};
	description.damping = {};
	description.capacity.bodies = 1;
	description.capacity.statics = STATIC_BODY_COUNT;
	description.capacity.shapes_per_type = SHAPES_PER_TYPE;
	description.threading = {
		worker_count=i32(worker_count),
		worker_pool_block_size=WORKER_POOL_BLOCK_SIZE,
	};
	return description;
}

benchmark_build_fixture :: proc(owner: ^Benchmark_Owner) -> Case_Status
{
	if owner == nil || owner.state != .Ready
	{
		return .Invalid_Argument;
	}
	box_shape, box_status := entasis.shape_add(
		&owner.world,
		entasis.box_half_extents(STATIC_HALF_EXTENT, STATIC_HALF_EXTENT, STATIC_HALF_EXTENT),
	);
	if box_status != .Ok
	{
		return .Fixture_Failed;
	}
	owner.sphere_shape, box_status = entasis.shape_add(
		&owner.world,
		entasis.sphere(SPHERE_CAST_RADIUS),
	);
	if box_status != .Ok
	{
		return .Fixture_Failed;
	}
	for iy in 0 ..< STATIC_GRID_Y
	{
		for iz in 0 ..< STATIC_GRID_Z
		{
			for ix in 0 ..< STATIC_GRID_X
			{
				position := entasis.Vector3{
					centered_grid_coordinate(STATIC_BASE_CENTER_X, STATIC_SPACING, ix, STATIC_GRID_X),
					uncentered_grid_coordinate(STATIC_BASE_CENTER_Y, STATIC_SPACING, iy),
					centered_grid_coordinate(STATIC_BASE_CENTER_Z, STATIC_SPACING, iz, STATIC_GRID_Z),
				};
				_, status := entasis.static_add(
					&owner.world,
					entasis.static_body(box_shape, entasis.pose(position)),
					.None,
				);
				if status != .Ok
				{
					return .Fixture_Failed;
				}
			}
		}
	}
	stats, stats_status := entasis.world_stats(&owner.world);
	if stats_status != .Ok || stats.statics != STATIC_BODY_COUNT ||
		stats.active_bodies != 0 || stats.sleeping_bodies != 0
	{
		return .Fixture_Failed;
	}
	return .Ok;
}

benchmark_owner_create :: proc(
	owner: ^Benchmark_Owner,
	pool: ^entasis.Buffer_Pool,
	worker_count: int,
) -> Case_Status
{
	if owner == nil || pool == nil || owner.state != .Empty ||
		worker_count <= 0 || worker_count > MAXIMUM_WORKER_COUNT
	{
		return .Invalid_Argument;
	}
	owner.worker_count = worker_count;
	if entasis.world_init_with_pool(
		&owner.world,
		benchmark_world_description(worker_count),
		pool,
	) != .Ok
	{
		return .Creation_Failed;
	}
	owner.state = .Ready;
	input_status := query_inputs_create(owner);
	if input_status != .Ok
	{
		_ = benchmark_owner_destroy(owner);
		return input_status;
	}
	fixture_status := benchmark_build_fixture(owner);
	if fixture_status != .Ok
	{
		_ = benchmark_owner_destroy(owner);
		return fixture_status;
	}
	borrow_status: entasis.Status;
	owner.simulation, borrow_status = entasis.world_borrow_simulation(&owner.world);
	if borrow_status != .Ok || owner.simulation == nil
	{
		_ = benchmark_owner_destroy(owner);
		return .Creation_Failed;
	}
	owner.owner_pool, borrow_status = entasis.world_borrow_pool(&owner.world);
	if borrow_status != .Ok || owner.owner_pool == nil
	{
		_ = benchmark_owner_destroy(owner);
		return .Creation_Failed;
	}
	owner.dispatcher, borrow_status = entasis.world_borrow_dispatcher(&owner.world);
	if borrow_status != .Ok ||
		(worker_count == 1 && owner.dispatcher != nil) ||
		(worker_count > 1 && (owner.dispatcher == nil || owner.dispatcher.worker_count != worker_count))
	{
		_ = benchmark_owner_destroy(owner);
		return .Creation_Failed;
	}
	return .Ok;
}

benchmark_owner_destroy :: proc(owner: ^Benchmark_Owner) -> Case_Status
{
	if owner == nil
	{
		return .Invalid_Argument;
	}
	if owner.state == .Released
	{
		return .Release_Failed;
	}
	query_inputs_destroy(owner);
	delete(owner.observations.rays);
	delete(owner.observations.sweeps);
	delete(owner.observations.overlaps);
	owner.observations = {};
	owner.simulation = nil;
	owner.owner_pool = nil;
	owner.dispatcher = nil;
	if owner.state == .Ready
	{
		if entasis.world_destroy(&owner.world) != .Ok
		{
			owner.state = .Released;
			return .Release_Failed;
		}
	}
	owner.state = .Released;
	return .Ok;
}

caller_buffer :: #force_inline proc "contextless" (
	$T: typeid,
	value: ^T,
) -> util.Buffer(T)
{
	return {
		memory=([^]T)(rawptr(value)),
		length=1,
		id=util.BUFFER_CALLER_OWNED_ID,
	};
}

checksum_mix :: #force_inline proc "contextless" (value: u64) -> u64
{
	mixed := value;
	mixed ~= mixed >> 30;
	mixed *= 0xbf58_476d_1ce4_e5b9;
	mixed ~= mixed >> 27;
	mixed *= 0x94d0_49bb_1331_11eb;
	mixed ~= mixed >> 31;
	return mixed;
}

ray_execute :: #force_inline proc "contextless" (
	simulation: ^physics.Simulation,
	pool: ^util.Buffer_Pool,
	input: Ray_Input,
) -> (bool, physics.Ray_Query_Hit, physics.Physics_Status)
{
	hit_storage: physics.Ray_Query_Hit;
	collector: physics.Ray_Query_Collector;
	status := physics.ray_query_collector_initialize(
		&collector,
		caller_buffer(physics.Ray_Query_Hit, &hit_storage),
		.Earliest,
	);
	if status != .Ok
	{
		return false, {}, status;
	}
	status = physics.simulation_ray_query(simulation, input.ray, &collector, pool);
	return collector.count > 0, hit_storage, status;
}

sphere_cast_execute :: #force_inline proc "contextless" (
	simulation: ^physics.Simulation,
	pool: ^util.Buffer_Pool,
	shape: physics.Typed_Index,
	input: Sphere_Cast_Input,
) -> (bool, physics.Sweep_Query_Hit, physics.Physics_Status)
{
	hit_storage: physics.Sweep_Query_Hit;
	collector := physics.Sweep_Query_Collector{
		hits=caller_buffer(physics.Sweep_Query_Hit, &hit_storage),
		mode=.Earliest,
	};
	status := physics.simulation_sweep_query(
		simulation,
		shape,
		input.pose,
		input.velocity,
		QUERY_DISTANCE,
		0.05,
		0.000005,
		25,
		&collector,
		pool,
	);
	return collector.count > 0, hit_storage, status;
}

overlap_execute :: #force_inline proc "contextless" (
	simulation: ^physics.Simulation,
	pool: ^util.Buffer_Pool,
	input: Overlap_Input,
) -> (bool, physics.Volume_Query_Hit, physics.Physics_Status)
{
	_ = pool;
	if simulation == nil || simulation.state != .Ready && simulation.state != .Stepping
	{
		return false, {}, .Disposed;
	}
	reference, state, status := physics.broad_phase_volume_any_query(
		&simulation.broad_phase, input.bounds,
	);
	return state == .Present, {target_id=i32(reference.packed)}, status;
}

ray_lane_timed :: proc "contextless" (
	owner: ^Benchmark_Owner,
	worker_index: int,
	pool: ^util.Buffer_Pool,
) -> Lane_Result
{
	begin := len(owner.ray_inputs) * worker_index / owner.worker_count;
	end := len(owner.ray_inputs) * (worker_index + 1) / owner.worker_count;
	hit_count := u64(0);
	for index in begin ..< end
	{
		hit, _, status := ray_execute(owner.simulation, pool, owner.ray_inputs[index]);
		if status != .Ok
		{
			return {status=status};
		}
		if hit
		{
			hit_count += 1;
		}
	}
	return {hit_count=hit_count, status=.Ok};
}

ray_lane_validate :: proc "contextless" (
	owner: ^Benchmark_Owner,
	worker_index: int,
	pool: ^util.Buffer_Pool,
) -> Lane_Result
{
	begin := len(owner.ray_inputs) * worker_index / owner.worker_count;
	end := len(owner.ray_inputs) * (worker_index + 1) / owner.worker_count;
	result := Lane_Result{status=.Ok};
	for index in begin ..< end
	{
		hit, value, status := ray_execute(owner.simulation, pool, owner.ray_inputs[index]);
		if status != .Ok
		{
			return {status=status};
		}
		token := u64(index + 1) * 0x9e37_79b9_7f4a_7c15;
		if len(owner.observations.rays) > 0
		{
			owner.observations.rays[index] = {state=.Hit if hit else .Miss, hit=transmute(entasis.Ray_Hit)value};
		}
		if hit
		{
			result.hit_count += 1;
			token ~= u64(u32(value.target_id)) << 32;
			token ~= u64(transmute(u32)value.t);
			token ~= 0x7261_795f_6869_7401;
		}
		else
		{
			token ~= 0x7261_795f_6d69_7373;
		}
		result.checksum ~= checksum_mix(token);
	}
	return result;
}

sphere_cast_lane_timed :: proc "contextless" (
	owner: ^Benchmark_Owner,
	worker_index: int,
	pool: ^util.Buffer_Pool,
) -> Lane_Result
{
	begin := len(owner.sphere_cast_inputs) * worker_index / owner.worker_count;
	end := len(owner.sphere_cast_inputs) * (worker_index + 1) / owner.worker_count;
	hit_count := u64(0);
	for index in begin ..< end
	{
		hit, _, status := sphere_cast_execute(
			owner.simulation,
			pool,
			owner.sphere_shape,
			owner.sphere_cast_inputs[index],
		);
		if status != .Ok
		{
			return {status=status};
		}
		if hit
		{
			hit_count += 1;
		}
	}
	return {hit_count=hit_count, status=.Ok};
}

sphere_cast_lane_validate :: proc "contextless" (
	owner: ^Benchmark_Owner,
	worker_index: int,
	pool: ^util.Buffer_Pool,
) -> Lane_Result
{
	begin := len(owner.sphere_cast_inputs) * worker_index / owner.worker_count;
	end := len(owner.sphere_cast_inputs) * (worker_index + 1) / owner.worker_count;
	result := Lane_Result{status=.Ok};
	for index in begin ..< end
	{
		hit, value, status := sphere_cast_execute(
			owner.simulation,
			pool,
			owner.sphere_shape,
			owner.sphere_cast_inputs[index],
		);
		if status != .Ok
		{
			return {status=status};
		}
		token := u64(index + 1) * 0xd6e8_feb8_6659_fd93;
		if len(owner.observations.sweeps) > 0
		{
			owner.observations.sweeps[index] = {state=.Hit if hit else .Miss, hit=transmute(entasis.Sweep_Hit)value};
		}
		if hit
		{
			result.hit_count += 1;
			token ~= u64(u32(value.target_id)) << 32;
			token ~= u64(transmute(u32)value.sweep.t0);
			token ~= 0x7377_6565_705f_6869;
		}
		else
		{
			token ~= 0x7377_6565_705f_6d73;
		}
		result.checksum ~= checksum_mix(token);
	}
	return result;
}

overlap_lane_timed :: proc "contextless" (
	owner: ^Benchmark_Owner,
	worker_index: int,
	pool: ^util.Buffer_Pool,
) -> Lane_Result
{
	begin := len(owner.overlap_inputs) * worker_index / owner.worker_count;
	end := len(owner.overlap_inputs) * (worker_index + 1) / owner.worker_count;
	hit_count := u64(0);
	for index in begin ..< end
	{
		hit, _, status := overlap_execute(owner.simulation, pool, owner.overlap_inputs[index]);
		if status != .Ok
		{
			return {status=status};
		}
		if hit
		{
			hit_count += 1;
		}
	}
	return {hit_count=hit_count, status=.Ok};
}

overlap_lane_validate :: proc "contextless" (
	owner: ^Benchmark_Owner,
	worker_index: int,
	pool: ^util.Buffer_Pool,
) -> Lane_Result
{
	begin := len(owner.overlap_inputs) * worker_index / owner.worker_count;
	end := len(owner.overlap_inputs) * (worker_index + 1) / owner.worker_count;
	result := Lane_Result{status=.Ok};
	for index in begin ..< end
	{
		hit, value, status := overlap_execute(owner.simulation, pool, owner.overlap_inputs[index]);
		if status != .Ok
		{
			return {status=status};
		}
		token := u64(index + 1) * 0xa076_1d64_78bd_642f;
		if len(owner.observations.overlaps) > 0
		{
			owner.observations.overlaps[index] = {state=.Hit if hit else .Miss, target={packed=u32(value.target_id)}};
		}
		if hit
		{
			result.hit_count += 1;
			token ~= u64(u32(value.target_id)) << 32;
			token ~= 0x6f76_6572_6c61_705f;
		}
		else
		{
			token ~= 0x6f76_6572_5f6d_6973;
		}
		result.checksum ~= checksum_mix(token);
	}
	return result;
}

query_worker :: proc "contextless" (
	worker_index: int,
	dispatcher: ^util.Thread_Dispatcher_Boundary,
)
{
	if dispatcher == nil || dispatcher.unmanaged_context == nil || dispatcher.worker_pool == nil
	{
		return;
	}
	ctx := (^Dispatch_Context)(dispatcher.unmanaged_context);
	if ctx == nil || ctx.owner == nil || worker_index < 0 || worker_index >= ctx.owner.worker_count
	{
		return;
	}
	context = runtime.default_context();
	pool, pool_status := dispatcher.worker_pool(dispatcher, worker_index);
	if pool_status != .Ok || pool == nil
	{
		ctx.owner.lanes[worker_index] = {status=.Invalid_Argument};
		return;
	}
	result: Lane_Result;
	switch ctx.family
	{
		case .Ray:
			if ctx.mode == .Validate
			{
				result = ray_lane_validate(ctx.owner, worker_index, pool);
			}
			else
			{
				result = ray_lane_timed(ctx.owner, worker_index, pool);
			}
		case .Sphere_Cast:
			if ctx.mode == .Validate
			{
				result = sphere_cast_lane_validate(ctx.owner, worker_index, pool);
			}
			else
			{
				result = sphere_cast_lane_timed(ctx.owner, worker_index, pool);
			}
		case .Overlap:
			if ctx.mode == .Validate
			{
				result = overlap_lane_validate(ctx.owner, worker_index, pool);
			}
			else
			{
				result = overlap_lane_timed(ctx.owner, worker_index, pool);
			}
	}
	ctx.owner.lanes[worker_index] = result;
}

family_execute :: proc(
	owner: ^Benchmark_Owner,
	family: Query_Family,
	mode: Execution_Mode,
) -> Family_Result
{
	if owner == nil || owner.state != .Ready || owner.simulation == nil || owner.owner_pool == nil
	{
		return {status=.Invalid_Argument};
	}
	for worker_index in 0 ..< owner.worker_count
	{
		owner.lanes[worker_index] = {status=.Invalid_Argument};
	}
	ctx := Dispatch_Context{owner=owner, family=family, mode=mode};
	if owner.worker_count == 1
	{
		switch family
		{
			case .Ray:
				if mode == .Validate
				{
					owner.lanes[0] = ray_lane_validate(owner, 0, owner.owner_pool);
				}
				else
				{
					owner.lanes[0] = ray_lane_timed(owner, 0, owner.owner_pool);
				}
			case .Sphere_Cast:
				if mode == .Validate
				{
					owner.lanes[0] = sphere_cast_lane_validate(owner, 0, owner.owner_pool);
				}
				else
				{
					owner.lanes[0] = sphere_cast_lane_timed(owner, 0, owner.owner_pool);
				}
			case .Overlap:
				if mode == .Validate
				{
					owner.lanes[0] = overlap_lane_validate(owner, 0, owner.owner_pool);
				}
				else
				{
					owner.lanes[0] = overlap_lane_timed(owner, 0, owner.owner_pool);
				}
		}
	}
	else
	{
		if owner.dispatcher == nil || owner.dispatcher.dispatch == nil
		{
			return {status=.Invalid_Argument};
		}
		dispatch_status := owner.dispatcher.dispatch(
			owner.dispatcher,
			query_worker,
			owner.worker_count,
			&ctx,
		);
		if dispatch_status != .Ok
		{
			return {status=.Invalid_Argument};
		}
	}
	result := Family_Result{status=.Ok};
	for worker_index in 0 ..< owner.worker_count
	{
		lane := owner.lanes[worker_index];
		if lane.status != .Ok
		{
			return {status=lane.status};
		}
		result.hit_count += lane.hit_count;
		result.checksum ~= lane.checksum;
	}
	return result;
}

batch_execute :: proc(
	owner: ^Benchmark_Owner,
	mode: Execution_Mode,
) -> Batch_Result
{
	result := Batch_Result{status=.Ok, physics_status=.Ok};
	start := time.tick_now();
	ray_result := family_execute(owner, .Ray, mode);
	end := time.tick_now();
	if ray_result.status != .Ok
	{
		return {status=.Query_Failed, physics_status=ray_result.status};
	}
	result.ray_nanoseconds = u64(time.tick_diff(start, end));
	result.ray_hits = ray_result.hit_count;
	result.ray_checksum = ray_result.checksum;

	start = time.tick_now();
	sphere_result := family_execute(owner, .Sphere_Cast, mode);
	end = time.tick_now();
	if sphere_result.status != .Ok
	{
		return {status=.Query_Failed, physics_status=sphere_result.status};
	}
	result.sphere_nanoseconds = u64(time.tick_diff(start, end));
	result.sphere_cast_hits = sphere_result.hit_count;
	result.sphere_cast_checksum = sphere_result.checksum;

	start = time.tick_now();
	overlap_result := family_execute(owner, .Overlap, mode);
	end = time.tick_now();
	if overlap_result.status != .Ok
	{
		return {status=.Query_Failed, physics_status=overlap_result.status};
	}
	result.overlap_nanoseconds = u64(time.tick_diff(start, end));
	result.overlap_hits = overlap_result.hit_count;
	result.overlap_checksum = overlap_result.checksum;
	return result;
}

batch_hits_valid :: #force_inline proc "contextless" (result: Batch_Result) -> bool
{
	return result.ray_hits == EXPECTED_RAY_HITS &&
		result.sphere_cast_hits == EXPECTED_SPHERE_CAST_HITS &&
		result.overlap_hits == EXPECTED_OVERLAP_HITS;
}

benchmark_execute :: proc(owner: ^Benchmark_Owner) -> Benchmark_Result
{
	validation := batch_execute(owner, .Validate);
	if validation.status != .Ok
	{
		return {status=validation.status, physics_status=validation.physics_status};
	}
	if !batch_hits_valid(validation) || validation.ray_checksum == 0 ||
		validation.sphere_cast_checksum == 0 || validation.overlap_checksum == 0
	{
		return {status=.Validation_Failed};
	}
	for _ in 0 ..< WARMUP_BATCH_COUNT
	{
		warmup := batch_execute(owner, .Timed);
		if warmup.status != .Ok
		{
			return {status=warmup.status, physics_status=warmup.physics_status};
		}
		if !batch_hits_valid(warmup)
		{
			return {status=.Validation_Failed};
		}
	}
	result := Benchmark_Result{
		status=.Ok,
		physics_status=.Ok,
		ray_checksum=validation.ray_checksum,
		sphere_cast_checksum=validation.sphere_cast_checksum,
		overlap_checksum=validation.overlap_checksum,
	};
	for _ in 0 ..< MEASURED_BATCH_COUNT
	{
		batch := batch_execute(owner, .Timed);
		if batch.status != .Ok
		{
			return {status=batch.status, physics_status=batch.physics_status};
		}
		if !batch_hits_valid(batch)
		{
			return {status=.Validation_Failed};
		}
		result.ray_hits = batch.ray_hits;
		result.sphere_cast_hits = batch.sphere_cast_hits;
		result.overlap_hits = batch.overlap_hits;
		result.ray_nanoseconds += batch.ray_nanoseconds;
		result.sphere_nanoseconds += batch.sphere_nanoseconds;
		result.overlap_nanoseconds += batch.overlap_nanoseconds;
	}
	if result.ray_nanoseconds == 0 || result.sphere_nanoseconds == 0 ||
		result.overlap_nanoseconds == 0
	{
		return {status=.Validation_Failed};
	}
	return result;
}
