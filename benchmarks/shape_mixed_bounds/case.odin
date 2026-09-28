package shape_mixed_bounds

import "core:math"
import entasis "entasis:entasis"
import cooking "entasis:entasis_cooking"

BODY_COUNT :: 9 * 1024;
SHAPE_TYPE_COUNT :: 9;
WARMUP_STEP_COUNT :: 30;
MEASURED_STEP_COUNT :: 3000;
TIMESTEP_DURATION :: f32(1.0 / 60.0);
MAXIMUM_WORKER_COUNT :: entasis.MAXIMUM_WORKER_COUNT;
WORKER_POOL_BLOCK_SIZE :: 65536;
POOL_MINIMUM_BLOCK_SIZE :: 65536;
BOUND_TOLERANCE :: f32(2e-4);
PINNED_FALLBACK_BATCH_INDEX :: 64;

SPHERE_SLOT :: 0;
CAPSULE_SLOT :: 1;
BOX_SLOT :: 2;
TRIANGLE_SLOT :: 3;
CYLINDER_SLOT :: 4;
CONVEX_HULL_SLOT :: 5;
COMPOUND_SLOT :: 6;
BIG_COMPOUND_SLOT :: 7;
MESH_SLOT :: 8;

#assert(SHAPE_TYPE_COUNT == 9);
#assert(MESH_SLOT + 1 == SHAPE_TYPE_COUNT);

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
	status:         Case_Status,
	physics_status: entasis.Status,
	step_phase:     Benchmark_Step_Phase,
	failed_step:    int,
}

Owner_State :: enum u8
{
	Empty,
	Ready,
	Released,
}

Benchmark_Simulation_Owner :: struct
{
	world:  entasis.World,
	shapes: [SHAPE_TYPE_COUNT]entasis.Shape_Handle,
	state:  Owner_State,
}

Bounds_Counters :: struct
{
	body_count:              int,
	shape_type_mask:         u32,
	invalid_bounds_count:    int,
	mismatched_bounds_count: int,
}

benchmark_vector_length :: #force_inline proc "contextless" (value: entasis.Vector3) -> f32
{
	return math.sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

benchmark_vector_normalize :: #force_inline proc "contextless" (
	value: entasis.Vector3,
) -> entasis.Vector3
{
	inverse_length := 1 / benchmark_vector_length(value);
	return {
		value.x * inverse_length,
		value.y * inverse_length,
		value.z * inverse_length,
	};
}

benchmark_quaternion_axis_angle :: #force_inline proc "contextless" (
	axis: entasis.Vector3,
	angle: f32,
) -> entasis.Quaternion
{
	half_angle := f64(angle) * 0.5;
	sine := math.sin(half_angle);
	return {
		f32(f64(axis.x) * sine),
		f32(f64(axis.y) * sine),
		f32(f64(axis.z) * sine),
		f32(math.cos(half_angle)),
	};
}

benchmark_angular_bounds_expansion :: #force_inline proc "contextless" (
	angular_speed, dt, maximum_radius, maximum_angular_expansion: f32,
) -> f32
{
	angle := min(angular_speed * dt, f32(math.PI / 3));
	angle_squared := angle * angle;
	angle_fourth := angle_squared * angle_squared;
	angle_sixth := angle_fourth * angle_squared;
	cosine_minus_one :=
		angle_squared * (-1.0 / 2.0) +
		angle_fourth * (1.0 / 24.0) -
		angle_sixth * (1.0 / 720.0);
	return min(
		maximum_angular_expansion,
		math.sqrt(max(0, -2 * maximum_radius * maximum_radius * cosine_minus_one)),
	);
}

benchmark_motion_bounds_expansion :: #force_inline proc "contextless" (
	linear_velocity, angular_velocity: entasis.Vector3,
	dt, maximum_radius, maximum_angular_expansion, maximum_allowed_expansion: f32,
) -> (minimum, maximum: entasis.Vector3)
{
	linear_displacement := entasis.Vector3{
		linear_velocity.x * dt,
		linear_velocity.y * dt,
		linear_velocity.z * dt,
	};
	angular_expansion := benchmark_angular_bounds_expansion(
		benchmark_vector_length(angular_velocity),
		dt,
		maximum_radius,
		maximum_angular_expansion,
	);
	minimum = {
		max(-maximum_allowed_expansion, min(f32(0), linear_displacement.x) - angular_expansion),
		max(-maximum_allowed_expansion, min(f32(0), linear_displacement.y) - angular_expansion),
		max(-maximum_allowed_expansion, min(f32(0), linear_displacement.z) - angular_expansion),
	};
	maximum = {
		min(maximum_allowed_expansion, max(f32(0), linear_displacement.x) + angular_expansion),
		min(maximum_allowed_expansion, max(f32(0), linear_displacement.y) + angular_expansion),
		min(maximum_allowed_expansion, max(f32(0), linear_displacement.z) + angular_expansion),
	};
	return;
}

benchmark_world_description :: proc(worker_count: int) -> entasis.World_Description
{
	description := entasis.world_description_default();
	description.gravity = {};
	description.damping = {};
	description.capacity = {
		bodies=BODY_COUNT,
		statics=1,
		inactive_body_sets=1,
		shapes_per_type=16,
		constraints=1024,
		initial_constraints_per_type_batch=16,
		minimum_constraints_per_body=8,
		broad_phase_candidates=BODY_COUNT * 2,
		pairs=BODY_COUNT * 2,
		collision_child_pairs=BODY_COUNT * 2,
		inactive_pairs=1,
		pending_pairs_per_worker=64,
	};
	description.solve = {
		velocity_iterations=8,
		substeps=1,
		fallback_batch_threshold=PINNED_FALLBACK_BATCH_INDEX,
	};
	description.threading = {
		worker_count=i32(worker_count),
		worker_pool_block_size=WORKER_POOL_BLOCK_SIZE,
	};
	return description;
}

benchmark_create_shapes :: proc(
	owner: ^Benchmark_Simulation_Owner,
) -> ([SHAPE_TYPE_COUNT]entasis.Shape_Handle, Case_Status)
{
	if owner == nil || owner.state != .Ready
	{
		return {}, .Invalid_Argument;
	}
	shapes: [SHAPE_TYPE_COUNT]entasis.Shape_Handle;
	status: entasis.Status;

	shapes[SPHERE_SLOT], status = entasis.shape_add(&owner.world, entasis.sphere(0.55));
	if status != .Ok
	{
		return {}, .Fixture_Failed;
	}
	shapes[CAPSULE_SLOT], status = entasis.shape_add(
		&owner.world,
		entasis.capsule_half_length(0.35, 0.65),
	);
	if status != .Ok
	{
		return {}, .Fixture_Failed;
	}
	shapes[BOX_SLOT], status = entasis.shape_add(
		&owner.world,
		entasis.box_half_extents(0.55, 0.4, 0.7),
	);
	if status != .Ok
	{
		return {}, .Fixture_Failed;
	}
	shapes[TRIANGLE_SLOT], status = entasis.shape_add(
		&owner.world,
		entasis.triangle({-0.65, -0.35, 0}, {0.7, -0.3, 0.15}, {0.05, 0.75, -0.1}),
	);
	if status != .Ok
	{
		return {}, .Fixture_Failed;
	}
	shapes[CYLINDER_SLOT], status = entasis.shape_add(
		&owner.world,
		entasis.cylinder_half_length(0.45, 0.6),
	);
	if status != .Ok
	{
		return {}, .Fixture_Failed;
	}

	cooking_context: cooking.Cooking_Context;
	if cooking.cooking_context_init(&cooking_context, POOL_MINIMUM_BLOCK_SIZE) != .Ok
	{
		return {}, .Allocation_Failed;
	}
	defer
	{
		_ = cooking.cooking_context_destroy(&cooking_context);
	}

	hull_points := [8]entasis.Vector3{
		{-0.55, -0.45, -0.65}, {0.55, -0.45, -0.65},
		{-0.55, 0.45, -0.65}, {0.55, 0.45, -0.65},
		{-0.55, -0.45, 0.65}, {0.55, -0.45, 0.65},
		{-0.55, 0.45, 0.65}, {0.55, 0.45, 0.65},
	};
	cooked_hull, hull_status := cooking.cook_hull(&cooking_context, hull_points[:]);
	if hull_status != .Ok
	{
		return {}, .Fixture_Failed;
	}
	shapes[CONVEX_HULL_SLOT], status = cooking.cooked_hull_import(&owner.world, &cooked_hull);
	hull_release_status := cooking.cooked_hull_destroy(&cooked_hull);
	if status != .Ok || hull_release_status != .Ok
	{
		return {}, .Fixture_Failed;
	}

	children := [3]entasis.Compound_Child{
		entasis.compound_child(shapes[SPHERE_SLOT], entasis.pose({-0.55, 0, 0})),
		entasis.compound_child(
			shapes[BOX_SLOT],
			entasis.pose({0.5, 0.1, 0}, benchmark_quaternion_axis_angle({0, 1, 0}, 0.4)),
		),
		entasis.compound_child(
			shapes[CAPSULE_SLOT],
			entasis.pose({0, 0.45, 0.25}, benchmark_quaternion_axis_angle({1, 0, 0}, -0.3)),
		),
	};
	shapes[COMPOUND_SLOT], status = entasis.shape_import_compound(&owner.world, children[:]);
	if status != .Ok
	{
		return {}, .Fixture_Failed;
	}

	big_children := [5]entasis.Compound_Child{
		entasis.compound_child(shapes[SPHERE_SLOT], entasis.pose({-0.7, 0, 0})),
		entasis.compound_child(shapes[CYLINDER_SLOT], entasis.pose({0.7, 0, 0})),
		entasis.compound_child(
			shapes[BOX_SLOT],
			entasis.pose({0, -0.55, 0}, benchmark_quaternion_axis_angle({0, 0, 1}, 0.25)),
		),
		entasis.compound_child(
			shapes[CAPSULE_SLOT],
			entasis.pose({0, 0.55, 0}, benchmark_quaternion_axis_angle({1, 0, 0}, 0.35)),
		),
		entasis.compound_child(shapes[TRIANGLE_SLOT], entasis.pose({0, 0, 0.65})),
	};
	shapes[BIG_COMPOUND_SLOT], status = entasis.shape_import_big_compound(
		&owner.world,
		big_children[:],
	);
	if status != .Ok
	{
		return {}, .Fixture_Failed;
	}

	mesh_triangles := [12]entasis.Triangle{
		entasis.triangle({-0.6, -0.5, -0.7}, {0.6, -0.5, -0.7}, {0.6, 0.5, -0.7}),
		entasis.triangle({-0.6, -0.5, -0.7}, {0.6, 0.5, -0.7}, {-0.6, 0.5, -0.7}),
		entasis.triangle({-0.6, -0.5, 0.7}, {0.6, 0.5, 0.7}, {0.6, -0.5, 0.7}),
		entasis.triangle({-0.6, -0.5, 0.7}, {-0.6, 0.5, 0.7}, {0.6, 0.5, 0.7}),
		entasis.triangle({-0.6, -0.5, -0.7}, {0.6, -0.5, 0.7}, {0.6, -0.5, -0.7}),
		entasis.triangle({-0.6, -0.5, -0.7}, {-0.6, -0.5, 0.7}, {0.6, -0.5, 0.7}),
		entasis.triangle({-0.6, 0.5, -0.7}, {0.6, 0.5, -0.7}, {0.6, 0.5, 0.7}),
		entasis.triangle({-0.6, 0.5, -0.7}, {0.6, 0.5, 0.7}, {-0.6, 0.5, 0.7}),
		entasis.triangle({-0.6, -0.5, -0.7}, {-0.6, 0.5, -0.7}, {-0.6, 0.5, 0.7}),
		entasis.triangle({-0.6, -0.5, -0.7}, {-0.6, 0.5, 0.7}, {-0.6, -0.5, 0.7}),
		entasis.triangle({0.6, -0.5, -0.7}, {0.6, -0.5, 0.7}, {0.6, 0.5, 0.7}),
		entasis.triangle({0.6, -0.5, -0.7}, {0.6, 0.5, 0.7}, {0.6, 0.5, -0.7}),
	};
	cooked_mesh, mesh_status := cooking.cook_mesh(
		&cooking_context,
		mesh_triangles[:],
		{1, 1, 1},
	);
	if mesh_status != .Ok
	{
		return {}, .Fixture_Failed;
	}
	shapes[MESH_SLOT], status = cooking.cooked_mesh_import(&owner.world, &cooked_mesh);
	mesh_release_status := cooking.cooked_mesh_destroy(&cooked_mesh);
	if status != .Ok || mesh_release_status != .Ok
	{
		return {}, .Fixture_Failed;
	}

	return shapes, .Ok;
}

benchmark_build_fixture :: proc(owner: ^Benchmark_Simulation_Owner) -> Case_Status
{
	if owner == nil || owner.state != .Ready
	{
		return .Invalid_Argument;
	}
	shapes, status := benchmark_create_shapes(owner);
	if status != .Ok
	{
		return status;
	}
	owner.shapes = shapes;

	inertia, inertia_status := entasis.shape_inertia(
		entasis.box_half_extents(0.55, 0.45, 0.65),
		1,
	);
	if inertia_status != .Ok
	{
		return .Fixture_Failed;
	}
	body := entasis.body_dynamic(
		shapes[SPHERE_SLOT],
		inertia,
		entasis.pose(),
		{},
		entasis.body_activity(-1, 255),
	);
	body.collidable.maximum_speculative_margin = 4;
	for body_index in 0 ..< BODY_COUNT
	{
		type_id := body_index % SHAPE_TYPE_COUNT;
		grid_x := body_index % 128;
		grid_z := (body_index / 128) % 72;
		axis := benchmark_vector_normalize({
				0.3 + f32((body_index * 3) % 11) * 0.07,
				0.5 + f32((body_index * 5) % 13) * 0.05,
				0.7 + f32((body_index * 7) % 17) * 0.03,
			});
		body.collidable.shape = shapes[type_id];
		body.pose = entasis.pose(
			{f32(grid_x) * 3.5, f32(type_id) * 2.25, f32(grid_z) * 3.5},
			benchmark_quaternion_axis_angle(axis, f32(body_index % 127) * 0.013),
		);
		body.velocity = entasis.velocity(
			{
				f32((body_index % 9) - 4) * 0.35,
				f32(((body_index / 9) % 7) - 3) * 0.2,
				f32(((body_index / 63) % 11) - 5) * 0.25,
			},
			{
				0.1 + f32(body_index % 5) * 0.08,
				0.2 + f32(body_index % 7) * 0.06,
				0.15 + f32(body_index % 3) * 0.11,
			},
		);
		_, body_status := entasis.body_add(&owner.world, body);
		if body_status != .Ok
		{
			return .Fixture_Failed;
		}
	}
	stats, stats_status := entasis.world_stats(&owner.world);
	if stats_status != .Ok || stats.active_bodies != BODY_COUNT ||
		stats.sleeping_bodies != 0 || stats.registered_shapes != SHAPE_TYPE_COUNT ||
		stats.registered_shape_types != SHAPE_TYPE_COUNT
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
	status := entasis.world_init_with_pool(
		&owner.world,
		benchmark_world_description(worker_count),
		pool,
	);
	if status != .Ok
	{
		return .Creation_Failed;
	}
	owner.state = .Ready;
	fixture_status := benchmark_build_fixture(owner);
	if fixture_status != .Ok
	{
		_ = benchmark_owner_destroy(owner);
		return fixture_status;
	}
	return .Ok;
}

benchmark_owner_predict :: #force_inline proc(
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
		step_status := entasis.world_stage_predict_bounds(&owner.world, TIMESTEP_DURATION);
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

benchmark_f32_finite :: #force_inline proc "contextless" (value: f32) -> bool
{
	return transmute(u32)value & 0x7f80_0000 != 0x7f80_0000;
}

benchmark_close :: #force_inline proc "contextless" (a, b: f32) -> bool
{
	return abs(a - b) <= BOUND_TOLERANCE * max(f32(1), max(abs(a), abs(b)));
}

benchmark_validate_bounds :: proc(
	owner: ^Benchmark_Simulation_Owner,
	counters: ^Bounds_Counters,
) -> Case_Status
{
	if owner == nil || owner.state != .Ready || counters == nil
	{
		return .Invalid_Argument;
	}
	counters^ = {};
	view, view_status := entasis.active_body_view(&owner.world);
	if view_status != .Ok || view.count != BODY_COUNT
	{
		return .Validation_Failed;
	}
	for body_index in 0 ..< view.count
	{
		type_id := body_index % SHAPE_TYPE_COUNT;
		collidable := &view.collidables[body_index];
		if collidable.shape != owner.shapes[type_id]
		{
			counters.mismatched_bounds_count += 1;
			continue;
		}
		counters.shape_type_mask |= u32(1) << u32(type_id);
		actual, actual_status := entasis.body_bounds(&owner.world, view.handles[body_index]);
		if actual_status != .Ok
		{
			return .Validation_Failed;
		}
		if !benchmark_f32_finite(actual.min.x) || !benchmark_f32_finite(actual.min.y) ||
			!benchmark_f32_finite(actual.min.z) || !benchmark_f32_finite(actual.max.x) ||
			!benchmark_f32_finite(actual.max.y) || !benchmark_f32_finite(actual.max.z) ||
			actual.min.x > actual.max.x || actual.min.y > actual.max.y ||
			actual.min.z > actual.max.z
		{
			counters.invalid_bounds_count += 1;
			continue;
		}

		motion := view.dynamics[body_index].motion;
		bounds, bounds_status := entasis.shape_bounds(
			&owner.world,
			collidable.shape,
			motion.pose.orientation,
		);
		if bounds_status != .Ok
		{
			return .Validation_Failed;
		}
		angular_expansion := benchmark_angular_bounds_expansion(
			benchmark_vector_length(motion.velocity.angular),
			TIMESTEP_DURATION,
			bounds.maximum_radius,
			bounds.maximum_angular_expansion,
		);
		margin := clamp(
			benchmark_vector_length(motion.velocity.linear) * TIMESTEP_DURATION + angular_expansion,
			collidable.minimum_speculative_margin,
			collidable.maximum_speculative_margin,
		);
		maximum_allowed_expansion := margin;
		if collidable.continuity.mode != .Discrete
		{
			maximum_allowed_expansion = f32(math.F32_MAX);
		}
		minimum_expansion, maximum_expansion := benchmark_motion_bounds_expansion(
			motion.velocity.linear,
			motion.velocity.angular,
			TIMESTEP_DURATION,
			bounds.maximum_radius,
			bounds.maximum_angular_expansion,
			maximum_allowed_expansion,
		);
		expected_min := entasis.Vector3{
			motion.pose.position.x + bounds.min.x + minimum_expansion.x,
			motion.pose.position.y + bounds.min.y + minimum_expansion.y,
			motion.pose.position.z + bounds.min.z + minimum_expansion.z,
		};
		expected_max := entasis.Vector3{
			motion.pose.position.x + bounds.max.x + maximum_expansion.x,
			motion.pose.position.y + bounds.max.y + maximum_expansion.y,
			motion.pose.position.z + bounds.max.z + maximum_expansion.z,
		};
		if !benchmark_close(actual.min.x, expected_min.x) ||
			!benchmark_close(actual.min.y, expected_min.y) ||
			!benchmark_close(actual.min.z, expected_min.z) ||
			!benchmark_close(actual.max.x, expected_max.x) ||
			!benchmark_close(actual.max.y, expected_max.y) ||
			!benchmark_close(actual.max.z, expected_max.z) ||
			!benchmark_close(collidable.speculative_margin, margin)
		{
			counters.mismatched_bounds_count += 1;
		}
		counters.body_count += 1;
	}
	if counters.body_count != BODY_COUNT || counters.invalid_bounds_count != 0 ||
		counters.mismatched_bounds_count != 0 ||
		counters.shape_type_mask != (u32(1) << SHAPE_TYPE_COUNT) - 1
	{
		return .Validation_Failed;
	}
	return .Ok;
}
