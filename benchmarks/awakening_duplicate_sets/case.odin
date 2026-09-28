package awakening_duplicate_sets

import "core:math"
import "base:runtime"
import entasis "entasis:entasis"

SET_COUNT :: 4;
BODIES_PER_SET :: 256;
SLEEPING_BODY_COUNT :: SET_COUNT * BODIES_PER_SET;
ACTIVE_BODY_COUNT :: SLEEPING_BODY_COUNT;
TOTAL_BODY_COUNT :: SLEEPING_BODY_COUNT + ACTIVE_BODY_COUNT;
EXPECTED_PAIR_COUNT :: SLEEPING_BODY_COUNT;
MAXIMUM_WORKER_COUNT :: entasis.MAXIMUM_WORKER_COUNT;
WORKER_POOL_BLOCK_SIZE :: 65536;
PINNED_FALLBACK_BATCH_INDEX :: 64;
TIMESTEP_DURATION :: f32(1.0 / 60.0);

Case_Status :: enum u8
{
	Ok,
	Invalid_Argument,
	Allocation_Failed,
	Creation_Failed,
	Fixture_Failed,
	Step_Failed,
	Validation_Failed,
	Release_Failed,
}

Owner_State :: enum u8
{
	Empty,
	Ready,
	Released,
}

Owner :: struct
{
	world: entasis.World,
	state: Owner_State,
	bodies: []entasis.Body_Handle,
}

Run_Result :: struct
{
	status:                  Case_Status,
	physics_status:          entasis.Status,
	active_body_count:       int,
	inactive_set_count:      int,
	pair_count:              int,
	invalid_transform_count: int,
	elapsed_ms:              f64,
}

world_description :: proc(worker_count: int) -> entasis.World_Description
{
	description := entasis.world_description_default();
	description.gravity = {};
	description.damping = {};
	description.capacity = {
		bodies=TOTAL_BODY_COUNT,
		statics=1,
		inactive_body_sets=SET_COUNT,
		shapes_per_type=2,
		constraints=4096,
		initial_constraints_per_type_batch=64,
		minimum_constraints_per_body=8,
		broad_phase_candidates=4096,
		pairs=4096,
		collision_child_pairs=i32(worker_count),
		inactive_pairs=1,
		pending_pairs_per_worker=2048,
	};
	description.solve = {
		velocity_iterations=2,
		substeps=1,
		fallback_batch_threshold=PINNED_FALLBACK_BATCH_INDEX,
	};
	description.threading = {
		worker_count=i32(worker_count),
		worker_pool_block_size=WORKER_POOL_BLOCK_SIZE,
	};
	return description;
}

body_position :: proc "contextless" (set_index, body_index: int) -> entasis.Vector3
{
	return {
		f32(set_index) * 80 + f32(body_index % 16) * 2,
		0,
		f32(body_index / 16) * 2,
	};
}

build_fixture :: proc(owner: ^Owner) -> Case_Status
{
	if owner == nil || owner.state != .Ready
	{
		return .Invalid_Argument;
	}
	box := entasis.box_half_extents(0.45, 0.45, 0.45);
	shape, shape_status := entasis.shape_add(&owner.world, box);
	if shape_status != .Ok
	{
		return .Fixture_Failed;
	}
	inertia, inertia_status := entasis.shape_inertia(box, 1);
	if inertia_status != .Ok
	{
		return .Fixture_Failed;
	}
	description := entasis.body_dynamic(
		shape,
		inertia,
		entasis.pose(),
		{},
		entasis.body_activity(-1, 255),
	);
	description.collidable.maximum_speculative_margin = f32(math.F32_MAX);

	for set_index in 0 ..< SET_COUNT
	{
		for body_index in 0 ..< BODIES_PER_SET
		{
			description.pose.position = body_position(set_index, body_index);
			handle, status := entasis.body_add(&owner.world, description);
			if status != .Ok
			{
				return .Fixture_Failed;
			}
			owner.bodies[set_index * BODIES_PER_SET + body_index] = handle;
		}
	}
	for set_index in 0 ..< SET_COUNT
	{
		for body_index in 0 ..< BODIES_PER_SET
		{
			description.pose.position = body_position(set_index, body_index);
			handle, status := entasis.body_add(&owner.world, description);
			if status != .Ok
			{
				return .Fixture_Failed;
			}
			owner.bodies[SLEEPING_BODY_COUNT + set_index * BODIES_PER_SET + body_index] = handle;
		}
	}
	for set_index in 0 ..< SET_COUNT
	{
		first := set_index * BODIES_PER_SET;
		last := first + BODIES_PER_SET;
		status := entasis.bodies_sleep_group(&owner.world, owner.bodies[first:last]);
		if status != .Ok
		{
			return .Fixture_Failed;
		}
	}
	stats, stats_status := entasis.world_stats(&owner.world);
	if stats_status != .Ok || stats.active_bodies != ACTIVE_BODY_COUNT ||
		stats.sleeping_bodies != SLEEPING_BODY_COUNT || stats.sleeping_islands != SET_COUNT
	{
		return .Fixture_Failed;
	}
	return .Ok;
}

owner_create :: proc(
	owner: ^Owner,
	pool: ^entasis.Buffer_Pool,
	worker_count: int,
) -> (Case_Status, entasis.Status)
{
	if owner == nil || pool == nil || owner.state != .Empty || worker_count <= 0 ||
		worker_count > MAXIMUM_WORKER_COUNT
	{
		return .Invalid_Argument, .Invalid_Argument;
	}
	status := entasis.world_init_with_pool(
		&owner.world,
		world_description(worker_count),
		pool,
	);
	if status != .Ok
	{
		return .Creation_Failed, status;
	}
	owner.state = .Ready;
	allocation_error: runtime.Allocator_Error;
	owner.bodies, allocation_error = make([]entasis.Body_Handle, TOTAL_BODY_COUNT);
	if allocation_error != nil
	{
		owner_destroy(owner);
		return .Allocation_Failed, .Capacity_Missing;
	}
	fixture_status := build_fixture(owner);
	if fixture_status != .Ok
	{
		_ = owner_destroy(owner);
		return fixture_status, .Invalid_Description;
	}
	return .Ok, .Ok;
}

finite_f32_status :: proc "contextless" (value: f32) -> Case_Status
{
	return .Ok if transmute(u32)value & 0x7f80_0000 != 0x7f80_0000 else .Validation_Failed;
}

owner_step :: #force_inline proc(owner: ^Owner) -> entasis.Status
{
	if owner == nil || owner.state != .Ready
	{
		return .Invalid_Argument;
	}
	return entasis.world_step(&owner.world, TIMESTEP_DURATION);
}

owner_validate :: proc(owner: ^Owner) -> Run_Result
{
	if owner == nil || owner.state != .Ready
	{
		return {status=.Invalid_Argument, physics_status=.Invalid_Argument};
	}
	stats, stats_status := entasis.world_stats(&owner.world);
	if stats_status != .Ok
	{
		return {status=.Validation_Failed, physics_status=stats_status};
	}
	view, view_status := entasis.active_body_view(&owner.world);
	if view_status != .Ok
	{
		return {status=.Validation_Failed, physics_status=view_status};
	}
	invalid_transform_count := 0;
	for body_index in 0 ..< view.count
	{
		pose := view.dynamics[body_index].motion.pose;
		if finite_f32_status(pose.position.x) != .Ok || finite_f32_status(pose.position.y) != .Ok ||
			finite_f32_status(pose.position.z) != .Ok || finite_f32_status(pose.orientation.x) != .Ok ||
			finite_f32_status(pose.orientation.y) != .Ok || finite_f32_status(pose.orientation.z) != .Ok ||
			finite_f32_status(pose.orientation.w) != .Ok
		{
			invalid_transform_count += 1;
		}
	}
	result := Run_Result{
		status=.Ok,
		physics_status=.Ok,
		active_body_count=stats.active_bodies,
		inactive_set_count=stats.sleeping_islands,
		pair_count=stats.active_pairs,
		invalid_transform_count=invalid_transform_count,
	};
	if result.active_body_count != TOTAL_BODY_COUNT || result.inactive_set_count != 0 ||
		result.pair_count != EXPECTED_PAIR_COUNT || result.invalid_transform_count != 0
	{
		result.status = .Validation_Failed;
	}
	return result;
}

owner_destroy :: proc(owner: ^Owner) -> Case_Status
{
	if owner == nil || owner.state != .Ready
	{
		return .Invalid_Argument;
	}
	status := entasis.world_destroy(&owner.world);
	delete(owner.bodies);
	owner^ = {state=.Released};
	if status != .Ok
	{
		return .Release_Failed;
	}
	return .Ok;
}
