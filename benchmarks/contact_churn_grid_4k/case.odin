package contact_churn_grid_4k

import entasis "entasis:entasis"

CASE_ID :: "contact_churn_grid_4k";
GRID_X :: 64;
GRID_Z :: 64;
DYNAMIC_BODY_COUNT :: GRID_X * GRID_Z;
STABLE_BODY_COUNT :: DYNAMIC_BODY_COUNT / 2;
TOGGLED_BODY_COUNT :: DYNAMIC_BODY_COUNT - STABLE_BODY_COUNT;
STATIC_BODY_COUNT :: DYNAMIC_BODY_COUNT;
PROTOCOL_BODY_COUNT :: DYNAMIC_BODY_COUNT + STATIC_BODY_COUNT;
PROTOCOL_SHAPE_COUNT :: PROTOCOL_BODY_COUNT;
WARMUP_STEP_COUNT :: 8;
MEASURED_STEP_COUNT :: 1200;
TIMESTEP_DURATION :: f32(1.0 / 60.0);
HALF_EXTENT :: f32(0.5);
SPACING :: f32(3.0);
CONTACT_Y :: f32(0.95);
SEPARATED_Y :: f32(3.0);
ORIGIN_X :: -f32(GRID_X - 1) * SPACING * 0.5;
ORIGIN_Z :: -f32(GRID_Z - 1) * SPACING * 0.5;
INNER_HALF_WIDTH :: f32(GRID_X) * SPACING * 0.5 + 1;
CEILING_Y :: f32(4);
MAXIMUM_WORKER_COUNT :: entasis.MAXIMUM_WORKER_COUNT;
WORKER_POOL_BLOCK_SIZE :: 65536;
POOL_MINIMUM_BLOCK_SIZE :: 65536;
PINNED_FALLBACK_BATCH_INDEX :: 64;
INITIAL_CONSTRAINTS_PER_TYPE_BATCH :: 64;
MINIMUM_CONSTRAINTS_PER_BODY :: 8;
INACTIVE_BODY_SET_COUNT :: 1;
SHAPES_PER_TYPE :: 4;
INACTIVE_PAIR_CAPACITY :: 1;
CONSTRAINT_CAPACITY :: 96 * 1024;
BROAD_PHASE_TRANSACTION_CAPACITY :: 96 * 1024;
PAIR_CACHE_CAPACITY :: 96 * 1024;
PENDING_PAIR_CAPACITY_PER_WORKER :: 1 << 12;

#assert(CONSTRAINT_CAPACITY >= DYNAMIC_BODY_COUNT * 2);
#assert(BROAD_PHASE_TRANSACTION_CAPACITY >= DYNAMIC_BODY_COUNT * 2);
#assert(PAIR_CACHE_CAPACITY >= DYNAMIC_BODY_COUNT * 2);

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

Benchmark_Step_Phase :: enum u8
{
	None,
	Warmup,
	Measured,
}

Benchmark_Run_Result :: struct
{
	status:                    Case_Status,
	physics_status:            entasis.Status,
	step_phase:                Benchmark_Step_Phase,
	failed_step:               int,
	exhausted_pool_power_mask: u32,
}

Owner_State :: enum u8
{
	Empty,
	Ready,
	Released,
}

Benchmark_Simulation_Owner :: struct
{
	world:         entasis.World,
	narrow_policy: entasis.Default_Narrow_Policy,
	state:         Owner_State,
}

Stability_Counters :: struct
{
	invalid_transform_count: int,
	below_floor_count:       int,
	out_of_bounds_count:     int,
	dynamic_body_count:      int,
}

benchmark_world_description :: proc(
	owner: ^Benchmark_Simulation_Owner,
	worker_count: int,
) -> entasis.World_Description
{
	description := entasis.world_description_default();
	description.gravity = {};
	description.damping = {};
	description.capacity = {
		bodies=DYNAMIC_BODY_COUNT,
		statics=STATIC_BODY_COUNT,
		inactive_body_sets=INACTIVE_BODY_SET_COUNT,
		shapes_per_type=SHAPES_PER_TYPE,
		constraints=CONSTRAINT_CAPACITY,
		initial_constraints_per_type_batch=INITIAL_CONSTRAINTS_PER_TYPE_BATCH,
		minimum_constraints_per_body=MINIMUM_CONSTRAINTS_PER_BODY,
		broad_phase_candidates=BROAD_PHASE_TRANSACTION_CAPACITY,
		pairs=PAIR_CACHE_CAPACITY,
		collision_child_pairs=i32(worker_count),
		inactive_pairs=INACTIVE_PAIR_CAPACITY,
		pending_pairs_per_worker=PENDING_PAIR_CAPACITY_PER_WORKER,
	};
	description.solve = {
		velocity_iterations=4,
		substeps=1,
		fallback_batch_threshold=PINNED_FALLBACK_BATCH_INDEX,
	};
	description.threading = {
		worker_count=i32(worker_count),
		worker_pool_block_size=WORKER_POOL_BLOCK_SIZE,
	};
	description.narrow_callbacks = entasis.narrow_policy_default(&owner.narrow_policy);
	return description;
}

benchmark_build_fixture :: proc(owner: ^Benchmark_Simulation_Owner) -> Case_Status
{
	if owner == nil || owner.state != .Ready
	{
		return .Invalid_Argument;
	}
	box_value := entasis.box_half_extents(HALF_EXTENT, HALF_EXTENT, HALF_EXTENT);
	box_shape, shape_status := entasis.shape_add(&owner.world, box_value);
	if shape_status != .Ok
	{
		return .Fixture_Failed;
	}

	for z in 0 ..< GRID_Z
	{
		for x in 0 ..< GRID_X
		{
			position := entasis.Vector3{
				ORIGIN_X + f32(x) * SPACING,
				0,
				ORIGIN_Z + f32(z) * SPACING,
			};
			_, status := entasis.static_add(
				&owner.world,
				entasis.static_body(box_shape, entasis.pose(position)),
			);
			if status != .Ok
			{
				return .Fixture_Failed;
			}
		}
	}

	inertia, inertia_status := entasis.shape_inertia(box_value, 1);
	if inertia_status != .Ok
	{
		return .Fixture_Failed;
	}
	body_description := entasis.body_dynamic(
		box_shape,
		inertia,
		entasis.pose(),
		{},
		entasis.body_activity(-1, 32),
	);
	body_description.collidable.maximum_speculative_margin = 0;
	for z in 0 ..< GRID_Z
	{
		for x in 0 ..< GRID_X
		{
			body_description.pose.position = {
				ORIGIN_X + f32(x) * SPACING,
				SEPARATED_Y,
				ORIGIN_Z + f32(z) * SPACING,
			};
			_, body_status := entasis.body_add(&owner.world, body_description);
			if body_status != .Ok
			{
				return .Fixture_Failed;
			}
		}
	}
	stats, stats_status := entasis.world_stats(&owner.world);
	if stats_status != .Ok || stats.active_bodies != DYNAMIC_BODY_COUNT ||
		stats.sleeping_bodies != 0 || stats.statics != STATIC_BODY_COUNT
	{
		return .Fixture_Failed;
	}
	return .Ok;
}

benchmark_owner_create :: proc(
	owner: ^Benchmark_Simulation_Owner,
	pool: ^entasis.Buffer_Pool,
	worker_count: int,
) -> Case_Status
{
	if owner == nil || pool == nil || owner.state != .Empty ||
		worker_count <= 0 || worker_count > MAXIMUM_WORKER_COUNT
	{
		return .Invalid_Argument;
	}
	owner.narrow_policy = {
		material=entasis.contact_material(
			0.5,
			2,
			entasis.spring_settings(30, 1),
		),
	};
	status := entasis.world_init_with_pool(
		&owner.world,
		benchmark_world_description(owner, worker_count),
		pool,
	);
	if status != .Ok
	{
		return .Creation_Failed;
	}
	owner.state = .Ready;
	solver_stats, solver_status := entasis.world_solver_stats(&owner.world);
	if solver_status != .Ok || solver_stats.fallback_batch_index != PINNED_FALLBACK_BATCH_INDEX
	{
		_ = benchmark_owner_destroy(owner);
		return .Creation_Failed;
	}
	fixture_status := benchmark_build_fixture(owner);
	if fixture_status != .Ok
	{
		_ = benchmark_owner_destroy(owner);
		return fixture_status;
	}
	return .Ok;
}

Contact_State :: enum u8
{
	Separated, Contact,
}

benchmark_set_contact_state :: #force_inline proc "contextless" (
	owner: ^Benchmark_Simulation_Owner,
	contact_state: Contact_State,
) -> entasis.Status
{
	view, status := entasis.active_body_view(&owner.world);
	if status != .Ok || view.count != DYNAMIC_BODY_COUNT
	{
		if status != .Ok
		{
			return status;
		}
		return .Invalid_Argument;
	}
	toggled_y := SEPARATED_Y;
	if contact_state == .Contact
	{
		toggled_y = CONTACT_Y;
	}
	for body_index in 0 ..< view.count
	{
		target_y := CONTACT_Y;
		if body_index >= STABLE_BODY_COUNT
		{
			target_y = toggled_y;
		}
		view.dynamics[body_index].motion.pose.position.y = target_y;
		view.dynamics[body_index].motion.velocity = {};
	}
	return .Ok;
}

benchmark_owner_step_index :: #force_inline proc(owner: ^Benchmark_Simulation_Owner, step_index: int) -> entasis.Status
{
	state: Contact_State = .Contact if step_index & 1 == 0 else .Separated;
	status: entasis.Status = benchmark_set_contact_state(owner, state);
	if status != .Ok
	{
		return status;
	}
	return entasis.world_step(&owner.world, TIMESTEP_DURATION);
}

benchmark_owner_step :: #force_inline proc(
	owner: ^Benchmark_Simulation_Owner,
	step_count: int,
	phase: Benchmark_Step_Phase,
) -> Benchmark_Run_Result
{
	if owner == nil || owner.state != .Ready || step_count <= 0 || phase == .None
	{
		return {status=.Invalid_Argument, physics_status=.Invalid_Argument, step_phase=phase};
	}
	for step_index in 0 ..< step_count
	{
		step_status: entasis.Status = benchmark_owner_step_index(owner, step_index);
		if step_status != .Ok
		{
			return {
				status=.Step_Failed,
				physics_status=step_status,
				step_phase=phase,
				failed_step=step_index + 1,
			};
		}
	}
	return {status=.Ok, physics_status=.Ok, step_phase=phase};
}

benchmark_owner_step_verified :: proc(
	owner: ^Benchmark_Simulation_Owner,
	step_count: int,
	phase: Benchmark_Step_Phase,
) -> Benchmark_Run_Result
{
	if owner == nil || owner.state != .Ready || step_count <= 0 || phase == .None
	{
		return {status=.Invalid_Argument, physics_status=.Invalid_Argument, step_phase=phase};
	}
	for step_index in 0 ..< step_count
	{
		step_status: entasis.Status = benchmark_owner_step_index(owner, step_index);
		if step_status != .Ok
		{
			return {
				status=.Step_Failed,
				physics_status=step_status,
				step_phase=phase,
				failed_step=step_index + 1,
			};
		}
		expected_pair_count := STABLE_BODY_COUNT;
		if step_index & 1 == 0
		{
			expected_pair_count += TOGGLED_BODY_COUNT;
		}
		stats, stats_status := entasis.world_stats(&owner.world);
		if stats_status != .Ok || stats.active_pairs != expected_pair_count
		{
			return {
				status=.Validation_Failed,
				physics_status=stats_status,
				step_phase=phase,
				failed_step=step_index + 1,
			};
		}
	}
	return {status=.Ok, physics_status=.Ok, step_phase=phase};
}

benchmark_owner_destroy :: proc(owner: ^Benchmark_Simulation_Owner) -> Case_Status
{
	if owner == nil || owner.state != .Ready
	{
		return .Invalid_Argument;
	}
	status := entasis.world_destroy(&owner.world);
	owner^ = {state=.Released};
	if status != .Ok
	{
		return .Release_Failed;
	}
	return .Ok;
}

benchmark_f32_validation_status :: #force_inline proc "contextless" (value: f32) -> Case_Status
{
	if transmute(u32)value & 0x7f80_0000 != 0x7f80_0000
	{
		return .Ok;
	}
	return .Validation_Failed;
}

benchmark_count_stability :: proc(
	owner: ^Benchmark_Simulation_Owner,
	counters: ^Stability_Counters,
) -> Case_Status
{
	if owner == nil || owner.state != .Ready || counters == nil
	{
		return .Invalid_Argument;
	}
	counters^ = {};
	view, view_status := entasis.active_body_view(&owner.world);
	if view_status != .Ok
	{
		return .Validation_Failed;
	}
	for body_index in 0 ..< view.count
	{
		pose := view.dynamics[body_index].motion.pose;
		position := pose.position;
		orientation := pose.orientation;
		counters.dynamic_body_count += 1;
		if benchmark_f32_validation_status(position.x) != .Ok ||
			benchmark_f32_validation_status(position.y) != .Ok ||
			benchmark_f32_validation_status(position.z) != .Ok ||
			benchmark_f32_validation_status(orientation.x) != .Ok ||
			benchmark_f32_validation_status(orientation.y) != .Ok ||
			benchmark_f32_validation_status(orientation.z) != .Ok ||
			benchmark_f32_validation_status(orientation.w) != .Ok
		{
			counters.invalid_transform_count += 1;
		}
		if position.y < 0
		{
			counters.below_floor_count += 1;
		}
		if position.x < -INNER_HALF_WIDTH || position.x > INNER_HALF_WIDTH ||
			position.z < -INNER_HALF_WIDTH || position.z > INNER_HALF_WIDTH ||
			position.y < 0 || position.y > CEILING_Y
		{
			counters.out_of_bounds_count += 1;
		}
	}
	stats, stats_status := entasis.world_stats(&owner.world);
	if stats_status != .Ok || counters.dynamic_body_count != DYNAMIC_BODY_COUNT ||
		stats.sleeping_bodies != 0 || stats.active_pairs != STABLE_BODY_COUNT ||
		counters.invalid_transform_count != 0 || counters.below_floor_count != 0
	{
		return .Validation_Failed;
	}
	return .Ok;
}
