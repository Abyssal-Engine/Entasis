package large_world_precision

import "core:fmt"
import "core:math"
import "core:time"
import entasis "entasis:entasis"
import support "../benchmark_support"

PRECISION_CANDIDATE :: #config(ENTASIS_PRECISION_CANDIDATE, "baseline");
PRECISION_NATIVE_GLOBAL :: PRECISION_CANDIDATE == "mixed64";
#assert(PRECISION_CANDIDATE == "baseline" || PRECISION_CANDIDATE == "origin" || PRECISION_NATIVE_GLOBAL);

POOL_MINIMUM_BLOCK_SIZE :: 65_536;
BODY_CAPACITY :: 10_000;
CONSTRAINT_CAPACITY :: 50;
COMPACT_BODY_COUNT :: 10_000;
COMPACT_STATIC_COUNT :: 5;
CROSSING_BODY_COUNT :: 1_000;
CROSSING_STATIC_COUNT :: 1;
CROSSING_JOINT_COUNT :: 50;
QUERY_STATIC_COUNT :: 3;
QUERY_REGISTERED_SHAPE_COUNT :: 5;
MEASURED_STEPS :: 512;
WARMUP_STEPS :: 30;
POSITION_TOLERANCE :: 0.001;
MAINTENANCE_SHIFT :: entasis.Vector3{256, 0, 0};

Precision_Admission :: enum u8
{
	Rejected,
	Accepted,
}

Precision_Case :: enum u8
{
	Compact_Near,
	Compact_10km,
	Compact_1000km,
	Compact_1gm,
	Shift_Sparse,
	Shift_Stress,
	Distributed,
	Crossing,
	Queries_Local,
	Queries_Spanning,
	Holdout_Small,
	Holdout_Mixed,
}

Precision_Fixture :: enum u8
{
	Compact,
	Crossing,
	Query_Local,
	Distributed,
	Query_Spanning,
}

Precision_Profile :: struct
{
	fixture: Precision_Fixture,
	body_count: int,
	static_count: int,
	joint_count: int,
	shape_count: int,
	offset_x: f64,
}

precision_profile :: #force_inline proc "contextless" (
	value: Precision_Case,
) -> Precision_Profile
{
	switch value
	{
		case .Compact_Near:
			return {.Compact, COMPACT_BODY_COUNT, COMPACT_STATIC_COUNT, 0, 4, 0};
		case .Compact_10km:
			return {.Compact, COMPACT_BODY_COUNT, COMPACT_STATIC_COUNT, 0, 4, 10_000};
		case .Compact_1000km:
			return {.Compact, COMPACT_BODY_COUNT, COMPACT_STATIC_COUNT, 0, 4, 1_000_000};
		case .Compact_1gm, .Shift_Sparse, .Shift_Stress:
			return {.Compact, COMPACT_BODY_COUNT, COMPACT_STATIC_COUNT, 0, 4, 1_000_000_000};
		case .Crossing:
			return {
				.Crossing,
				CROSSING_BODY_COUNT,
				CROSSING_STATIC_COUNT,
				CROSSING_JOINT_COUNT,
				2,
				1_000_000_512,
			};
		case .Queries_Local:
			return {.Query_Local, 0, QUERY_STATIC_COUNT, 0, QUERY_REGISTERED_SHAPE_COUNT, 1_000_000_000};
		case .Distributed, .Queries_Spanning:
			shapes: int = 2 * DISTRIBUTED_WORLD_COUNT;
			when PRECISION_NATIVE_GLOBAL
			{
				shapes = 2;
			}
			fixture: Precision_Fixture = .Distributed;
			if value == .Queries_Spanning
			{
				fixture = .Query_Spanning;
			}
			return {fixture, DISTRIBUTED_WORLD_COUNT*DISTRIBUTED_BODY_COUNT, DISTRIBUTED_WORLD_COUNT, 0, shapes, DISTRIBUTED_OFFSET};
		case .Holdout_Small:
			return {.Compact, 1_000, 5, 0, 4, 1_000_000};
		case .Holdout_Mixed:
			return {.Compact, 1_000, 5, 0, 8, 1_000_000};
	}
	unreachable();
}

precision_fixture_text :: #force_inline proc "contextless" (
	value: Precision_Case,
) -> string
{
	profile := precision_profile(value);
	if value == .Holdout_Mixed
	{
		return "mixed-shape-holdout";
	}
	if value == .Holdout_Small
	{
		return "small-compact-holdout";
	}
	switch profile.fixture
	{
		case .Compact: return "compact-container";
		case .Crossing: return "crossing-pairs";
		case .Query_Local: return "local-query-probes";
		case .Distributed:
			when PRECISION_NATIVE_GLOBAL
			{
				return "single-world-distributed";
			}
			return "distributed-single-world-groups";
		case .Query_Spanning:
			when PRECISION_NATIVE_GLOBAL
			{
				return "single-world-spanning-rays";
			}
			return "distributed-spanning-rays";
	}
	unreachable();
}

precision_case_parse :: proc(value: string) -> (Precision_Case, Precision_Admission)
{
	switch value
	{
		case "compact-near":     return .Compact_Near, .Accepted;
		case "compact-10km":     return .Compact_10km, .Accepted;
		case "compact-1000km":   return .Compact_1000km, .Accepted;
		case "compact-1gm":      return .Compact_1gm, .Accepted;
		case "shift-sparse":     return .Shift_Sparse, .Accepted;
		case "shift-stress":     return .Shift_Stress, .Accepted;
		case "distributed":      return .Distributed, .Accepted;
		case "crossing":         return .Crossing, .Accepted;
		case "queries-local":    return .Queries_Local, .Accepted;
		case "queries-spanning": return .Queries_Spanning, .Accepted;
		case "holdout-small":    return .Holdout_Small, .Accepted;
		case "holdout-mixed":    return .Holdout_Mixed, .Accepted;
	}
	return {}, .Rejected;
}

precision_case_text :: proc(value: Precision_Case) -> string
{
	switch value
	{
		case .Compact_Near: return "compact-near";
		case .Compact_10km: return "compact-10km";
		case .Compact_1000km: return "compact-1000km";
		case .Compact_1gm: return "compact-1gm";
		case .Shift_Sparse: return "shift-sparse";
		case .Shift_Stress: return "shift-stress";
		case .Distributed: return "distributed";
		case .Crossing: return "crossing";
		case .Queries_Local: return "queries-local";
		case .Queries_Spanning: return "queries-spanning";
		case .Holdout_Small: return "holdout-small";
		case .Holdout_Mixed: return "holdout-mixed";
	}
	unreachable();
}

precision_case_supported :: #force_inline proc "contextless" (
	value: Precision_Case,
) -> Precision_Admission
{
	
	when PRECISION_NATIVE_GLOBAL
	{
		switch value
		{
			case .Compact_Near, .Compact_10km, .Compact_1000km, .Compact_1gm,
				.Shift_Sparse, .Shift_Stress, .Distributed, .Crossing, .Queries_Local, .Queries_Spanning,
				.Holdout_Small, .Holdout_Mixed:
				return .Accepted;
		}
	}
	else when PRECISION_CANDIDATE == "origin"
	{
		switch value
		{
			case .Compact_Near, .Compact_10km, .Compact_1000km, .Compact_1gm,
				.Shift_Sparse, .Shift_Stress, .Crossing, .Queries_Local, .Holdout_Small, .Holdout_Mixed:
				return .Accepted;
			case .Distributed, .Queries_Spanning:
				return .Rejected;
		}
	}
	else
	{
		switch value
		{
			case .Compact_Near, .Compact_10km, .Compact_1000km, .Compact_1gm:
				return .Accepted;
			case .Shift_Sparse, .Shift_Stress, .Distributed, .Crossing,
				.Queries_Local, .Queries_Spanning, .Holdout_Small, .Holdout_Mixed:
				return .Rejected;
		}
	}
	unreachable();
}

precision_case_offset :: #force_inline proc "contextless" (value: Precision_Case) -> f64
{
	return precision_profile(value).offset_x;
}

precision_case_maintenance_interval :: #force_inline proc "contextless" (
	value: Precision_Case,
) -> int
{
	// absolute-position candidates track the same application frame changes
	// without translating their live world or charging fictional maintenance
	when PRECISION_NATIVE_GLOBAL
	{
		return 0;
	}
	switch value
	{
		case .Shift_Sparse: return 256;
		case .Shift_Stress: return 16;
		case .Compact_Near, .Compact_10km, .Compact_1000km, .Compact_1gm,
			.Distributed, .Crossing, .Queries_Local, .Queries_Spanning,
			.Holdout_Small, .Holdout_Mixed:
			return 0;
	}
	return 0;
}

Owner_State :: enum u8
{
	Empty,
	Ready,
}

Benchmark_Storage :: struct
{
	bodies: [BODY_CAPACITY]entasis.Body_Handle,
	constraints: [CONSTRAINT_CAPACITY]entasis.Constraint_Handle,
	expected_global_positions: [BODY_CAPACITY]entasis.Global_Position,
	exported_global_positions: [BODY_CAPACITY]entasis.Global_Position,
	frame_nanoseconds: [MEASURED_STEPS]i64,
	sorted_frame_nanoseconds: [MEASURED_STEPS]i64,
	maintenance_nanoseconds: [MEASURED_STEPS]i64,
}

benchmark_storage: Benchmark_Storage;

Benchmark_Owner :: struct
{
	world: entasis.World,
	narrow_policy: entasis.Default_Narrow_Policy,
	bodies: []entasis.Body_Handle,
	constraints: []entasis.Constraint_Handle,
	expected_global_positions: []entasis.Global_Position,
	exported_global_positions: []entasis.Global_Position,
	frame_nanoseconds: []i64,
	sorted_frame_nanoseconds: []i64,
	maintenance_nanoseconds: []i64,
	state: Owner_State,
	options: support.Options,
	precision_case: Precision_Case,
	frame: entasis.Origin_Frame,
	registered_shapes: int,
	shapes: support.Shape_Cache,
	export_checksum: f64,
	frame_median_ms: f64,
	frame_max_ms: f64,
	maintenance_max_ms: f64,
	maintenance_count: int,
	sampling_mode: Sampling_Mode,
	memory: ^Precision_Memory,
	query: Query_Local_Owner,
	spanning: Global_Spanning_Owner,
}

precision_options :: proc(
	worker_count: int,
	output: string,
	precision_case: Precision_Case,
) -> support.Options
{
	profile := precision_profile(precision_case);
	if profile.fixture == .Distributed || profile.fixture == .Query_Spanning
	{
		return distributed_options(worker_count, output, precision_case);
	}
	if profile.fixture == .Query_Local
	{
		return {
			workload=.Container,
			shape=.Sphere,
			static_shape=.Box,
			shape_size={0.5, 0.5, 0.5},
			density=1,
			layout_scale=1,
			steps=QUERY_PASS_COUNT,
			timestep_hz=60,
			warmup_steps=QUERY_WARMUP_COUNT,
			velocity_iterations=1,
			substeps=4,
			sleep=.Disabled,
			worker_count=worker_count,
			output=output,
			population_count=profile.static_count,
			body_count=0,
			static_count=profile.static_count,
		};
	}
	if profile.fixture == .Crossing
	{
		return {
			workload=.Container,
			shape=.Box,
			static_shape=.Box,
			shape_size={1, 1, 1},
			density=1,
			layout_scale=1,
			steps=MEASURED_STEPS,
			timestep_hz=60,
			warmup_steps=WARMUP_STEPS,
			velocity_iterations=1,
			substeps=4,
			sleep=.Disabled,
			grid={2, 10, 50},
			spacing={1.2, 1.5, 1.5},
			spawn_height=0,
			container_size={32, 32, 80},
			worker_count=worker_count,
			output=output,
			population_count=profile.body_count + profile.static_count,
			body_count=profile.body_count,
			static_count=profile.static_count,
		};
	}
	if precision_case == .Holdout_Small || precision_case == .Holdout_Mixed
	{
		return holdout_options(worker_count, output, precision_case);
	}
	return {
		workload=.Container,
		shape=.Box,
		static_shape=.Box,
		shape_size={1, 1, 1},
		density=1,
		layout_scale=1,
		steps=MEASURED_STEPS,
		timestep_hz=60,
		warmup_steps=WARMUP_STEPS,
		velocity_iterations=1,
		substeps=4,
		sleep=.Disabled,
		grid={25, 16, 25},
		spacing={1.02, 1.02, 1.02},
		spawn_height=24.51,
		container_size={29, 24.5, 29},
		worker_count=worker_count,
		output=output,
		population_count=profile.body_count + profile.static_count,
		body_count=profile.body_count,
		static_count=profile.static_count,
	};
}

benchmark_world_description :: proc(owner: ^Benchmark_Owner) -> entasis.World_Description
{
	profile := precision_profile(owner.precision_case);
	pairs := max(1, (owner.options.body_count * 96 * 1024 + 9999) / 10000);
	description := entasis.world_description_default();
	description.gravity = {0, -10, 0};
	if profile.fixture != .Compact && profile.fixture != .Distributed
	{
		description.gravity = {};
	}
	description.damping = {};
	description.capacity = {
		bodies=i32(max(1, owner.options.body_count)),
		statics=i32(max(1, owner.options.static_count)),
		inactive_body_sets=1,
		shapes_per_type=4,
		constraints=i32(pairs + profile.joint_count),
		initial_constraints_per_type_batch=64,
		minimum_constraints_per_body=8,
		broad_phase_candidates=i32(pairs),
		pairs=i32(pairs),
		collision_child_pairs=i32(owner.options.worker_count),
		inactive_pairs=1,
		pending_pairs_per_worker=4096,
	};
	description.solve = {
		velocity_iterations=i32(owner.options.velocity_iterations),
		substeps=i32(owner.options.substeps),
		fallback_batch_threshold=64,
	};
	description.threading = {
		worker_count=i32(owner.options.worker_count),
		worker_pool_block_size=POOL_MINIMUM_BLOCK_SIZE,
	};
	// primitive fixtures reserve one child slot per worker. compound/mesh
	// holdouts need the existing public automatic child-pair capacity policy
	if owner.precision_case == .Holdout_Mixed
	{
		description.capacity.collision_child_pairs = 0;
	}
	description.narrow_callbacks = entasis.narrow_policy_default(&owner.narrow_policy);
	return description;
}

global_from_local_fixture :: #force_inline proc(
	owner: ^Benchmark_Owner,
	local: entasis.Vector3,
) -> entasis.Global_Position
{
	return entasis.global_position(
		owner.frame.origin.x + f64(local.x),
		owner.frame.origin.y + f64(local.y),
		owner.frame.origin.z + f64(local.z),
	);
}

add_static_fixture :: proc(
	owner: ^Benchmark_Owner,
	shape: entasis.Shape_Handle,
	logical_position: entasis.Vector3,
) -> entasis.Status
{
	global: entasis.Global_Position = global_from_local_fixture(owner, logical_position);
	_, status := candidate_static_add(owner, shape, global, .Overlaps);
	return status;
}

compact_build_fixture :: proc(owner: ^Benchmark_Owner) -> entasis.Status
{
	o := owner.options;
	t := o.layout_scale;
	floor_shape, status := support.register_shape(
		&owner.world,
		&owner.shapes,
		o.static_shape,
		{o.container_size[0], t, o.container_size[2]},
	);
	if status != .Ok
	{
		return status;
	}
	side_shape, side_status := support.register_shape(
		&owner.world,
		&owner.shapes,
		o.static_shape,
		{t, o.container_size[1], o.container_size[2]},
	);
	if side_status != .Ok
	{
		return side_status;
	}
	front_shape, front_status := support.register_shape(
		&owner.world,
		&owner.shapes,
		o.static_shape,
		{o.container_size[0], o.container_size[1], t},
	);
	if front_status != .Ok
	{
		return front_status;
	}
	shape, inertia, shape_status := support.register_dynamic(
		&owner.world,
		&owner.shapes,
		o.shape,
		o.shape_size,
		o.density,
	);
	if shape_status != .Ok
	{
		return shape_status;
	}
	shapes := [5]entasis.Shape_Handle{floor_shape, side_shape, side_shape, front_shape, front_shape};
	x, y, z := o.container_size[0]/2, (o.container_size[1]-t)/2, o.container_size[2]/2;
	positions := [5]entasis.Vector3{{0, -t/2, 0}, {-x, y, 0}, {x, y, 0}, {0, y, -z}, {0, y, z}};
	for position, index in positions
	{
		status = add_static_fixture(owner, shapes[index], position);
		if status != .Ok
		{
			return status;
		}
	}
	description := entasis.body_dynamic(
		shape,
		inertia,
		entasis.pose(),
		{},
		entasis.body_activity(-1, 32),
	);
	description.collidable.maximum_speculative_margin = f32(math.F32_MAX);
	origin_x := -0.5*f32(o.grid[0]-1)*o.spacing[0];
	origin_z := -0.5*f32(o.grid[2]-1)*o.spacing[2];
	index := 0;
	for y_index in 0 ..< o.grid[1]
	{
		for z_index in 0 ..< o.grid[2]
		{
			for x_index in 0 ..< o.grid[0]
			{
				logical := entasis.Vector3{
					origin_x + f32(x_index)*o.spacing[0],
					o.spawn_height + f32(y_index)*o.spacing[1],
					origin_z + f32(z_index)*o.spacing[2],
				};
				global := global_from_local_fixture(owner, logical);
				owner.expected_global_positions[index] = global;
				owner.bodies[index], status = candidate_body_add(owner, description, global);
				if status != .Ok
				{
					return status;
				}
				index += 1;
			}
		}
	}
	return .Ok;
}

benchmark_build_fixture :: proc(owner: ^Benchmark_Owner) -> entasis.Status
{
	profile := precision_profile(owner.precision_case);
	switch profile.fixture
	{
		case .Compact:
			if owner.precision_case == .Holdout_Mixed
			{
				return holdout_build_mixed(owner);
			}
			return compact_build_fixture(owner);
		case .Crossing: return crossing_build_fixture(owner);
		case .Query_Local: return query_local_build_fixture(owner);
		case .Distributed, .Query_Spanning:
			when PRECISION_NATIVE_GLOBAL
			{
				return distributed_global_build_fixture(owner);
			}
			else
			{
				unreachable();
			}
	}
	unreachable();
}

precision_close :: #force_inline proc "contextless" (
	a, b, tolerance: f64,
) -> Precision_Admission
{
	if math.abs(a-b) <= tolerance
	{
		return .Accepted;
	}
	return .Rejected;
}

precision_global_close :: #force_inline proc "contextless" (
	a, b: entasis.Global_Position,
	tolerance: f64,
) -> Precision_Admission
{
	if precision_close(a.x, b.x, tolerance) == .Accepted &&
		precision_close(a.y, b.y, tolerance) == .Accepted &&
		precision_close(a.z, b.z, tolerance) == .Accepted
	{
		return .Accepted;
	}
	return .Rejected;
}

precision_vector_close :: #force_inline proc "contextless" (
	a, b: entasis.Vector3,
	tolerance: f64,
) -> Precision_Admission
{
	if precision_close(f64(a.x), f64(b.x), tolerance) == .Accepted &&
		precision_close(f64(a.y), f64(b.y), tolerance) == .Accepted &&
		precision_close(f64(a.z), f64(b.z), tolerance) == .Accepted
	{
		return .Accepted;
	}
	return .Rejected;
}

validate_initial_placement :: proc(owner: ^Benchmark_Owner) -> support.Benchmark_Result
{
	for handle, index in owner.bodies
	{
		global, global_status := candidate_body_global_position(owner, handle);
		if global_status != .Ok
		{
			return {status=.Fixture_Failed, physics_status=global_status};
		}
		expected := owner.expected_global_positions[index];
		if precision_close(global.x, expected.x, POSITION_TOLERANCE) != .Accepted ||
			precision_close(global.y, expected.y, POSITION_TOLERANCE) != .Accepted ||
			precision_close(global.z, expected.z, POSITION_TOLERANCE) != .Accepted
		{
			return {status=.Fixture_Failed};
		}
	}
	return {status=.Ok};
}

benchmark_owner_create :: proc(
	owner: ^Benchmark_Owner,
	pool: ^entasis.Buffer_Pool,
	options: support.Options,
	precision_case: Precision_Case,
	memory: ^Precision_Memory = nil,
) -> support.Benchmark_Result
{
	owner.options = options;
	owner.memory = memory;
	owner.precision_case = precision_case;
	profile := precision_profile(precision_case);
	origin := entasis.global_position(precision_case_offset(precision_case), 0, 0);
	frame_status: entasis.Status;
	owner.frame, frame_status = entasis.origin_frame_at(origin);
	if frame_status != .Ok
	{
		return {status=.Fixture_Failed, physics_status=frame_status};
	}
	benchmark_storage = {};
	owner.bodies = benchmark_storage.bodies[:profile.body_count];
	owner.constraints = benchmark_storage.constraints[:profile.joint_count];
	owner.expected_global_positions = benchmark_storage.expected_global_positions[:profile.body_count];
	owner.exported_global_positions = benchmark_storage.exported_global_positions[:profile.body_count];
	owner.frame_nanoseconds = benchmark_storage.frame_nanoseconds[:];
	owner.sorted_frame_nanoseconds = benchmark_storage.sorted_frame_nanoseconds[:];
	owner.maintenance_nanoseconds = benchmark_storage.maintenance_nanoseconds[:];
	owner.narrow_policy = {
		material=entasis.contact_material(0.5, 2, entasis.spring_settings(30, 1)),
	};
	status := precision_world_initialize(&owner.world, benchmark_world_description(owner), pool, memory);
	if status != .Ok
	{
		return {status=.Creation_Failed, physics_status=status};
	}
	owner.state = .Ready;
	status = benchmark_build_fixture(owner);
	if status != .Ok
	{
		return {status=.Fixture_Failed, physics_status=status};
	}
	stats, stats_status := entasis.world_stats(&owner.world);
	if stats_status != .Ok
	{
		return {status=.Fixture_Failed, physics_status=stats_status};
	}
	owner.registered_shapes = profile.shape_count;
	if stats.active_bodies != profile.body_count || stats.sleeping_bodies != 0 ||
		stats.statics != profile.static_count ||
		stats.registered_shapes != owner.registered_shapes ||
		stats.active_constraints != profile.joint_count
	{
		return {status=.Fixture_Failed};
	}
	if profile.fixture == .Query_Local
	{
		return query_local_prepare(owner);
	}
	result: support.Benchmark_Result = validate_initial_placement(owner);
	when PRECISION_NATIVE_GLOBAL
	{
		if result.status == .Ok && profile.fixture == .Query_Spanning
		{
			return global_spanning_prepare(owner);
		}
	}
	return result;
}

benchmark_owner_destroy :: proc(owner: ^Benchmark_Owner) -> entasis.Status
{
	status := entasis.Status.Ok;
	when PRECISION_NATIVE_GLOBAL
	{
		if owner.spanning.state == .Ready
		{
			status = entasis.query_context_destroy(&owner.spanning.ctx);
		}
	}
	if owner.query.state == .Ready
	{
		status = query_local_destroy(owner);
	}
	if owner.state == .Ready
	{
		world_status := entasis.world_destroy(&owner.world);
		if status == .Ok
		{
			status = world_status;
		}
	}
	owner^ = {};
	return status;
}

step_world :: proc(
	owner: ^Benchmark_Owner,
	step_count: int,
	phase: support.Step_Phase,
) -> support.Benchmark_Result
{
	for step_index in 0 ..< step_count
	{
		status := entasis.world_step(&owner.world, 1/f32(owner.options.timestep_hz));
		if status != .Ok
		{
			return {
				status=.Step_Failed,
				physics_status=status,
				phase=phase,
				failed_step=step_index+1,
			};
		}
	}
	return {status=.Ok, physics_status=.Ok, phase=phase};
}

export_global_positions :: #force_inline proc(owner: ^Benchmark_Owner) -> support.Benchmark_Result
{
	checksum: f64;
	for handle, index in owner.bodies
	{
		global, global_status := candidate_body_global_position(owner, handle);
		if global_status != .Ok
		{
			return {status=.Validation_Failed, physics_status=global_status, phase=.Measured};
		}
		owner.exported_global_positions[index] = global;
		checksum += global.x*0.5 + global.y*0.25 + global.z*0.125;
	}
	owner.export_checksum += checksum;
	return {status=.Ok, physics_status=.Ok, phase=.Measured};
}

sort_frame_durations :: proc(values: []i64)
{
	for index in 1 ..< len(values)
	{
		value := values[index];
		cursor := index;
		for cursor > 0 && values[cursor-1] > value
		{
			values[cursor] = values[cursor-1];
			cursor -= 1;
		}
		values[cursor] = value;
	}
}

compact_validate_positions :: proc(
	owner: ^Benchmark_Owner,
	sample: ^support.Benchmark_Sample,
) -> support.Benchmark_Result
{
	o := owner.options;
	radius := math.sqrt(
		o.shape_size[0]*o.shape_size[0] +
		o.shape_size[1]*o.shape_size[1] +
		o.shape_size[2]*o.shape_size[2],
	)/2;
	x_limit := max(
		o.container_size[0]/2+o.layout_scale/2,
		f32(o.grid[0]-1)*o.spacing[0]/2,
	) + radius;
	z_limit := max(
		o.container_size[2]/2+o.layout_scale/2,
		f32(o.grid[2]-1)*o.spacing[2]/2,
	) + radius;
	y_limit := 2*max(
		o.container_size[1],
		o.spawn_height+f32(o.grid[1]-1)*o.spacing[1],
	) + radius;
	for global in owner.exported_global_positions
	{
		x := global.x-owner.frame.origin.x;
		y := global.y-owner.frame.origin.y;
		z := global.z-owner.frame.origin.z;
		if math.is_nan(x) || math.is_inf(x, 0) ||
			math.is_nan(y) || math.is_inf(y, 0) ||
			math.is_nan(z) || math.is_inf(z, 0)
		{
			sample.invalid_transform_count += 1;
			continue;
		}
		if y < min(f64(0), f64(o.spawn_height-o.shape_size[1]))
		{
			sample.below_floor_count += 1;
		}
		if math.abs(x) > f64(x_limit) || math.abs(z) > f64(z_limit) || y > f64(y_limit)
		{
			sample.out_of_bounds_count += 1;
		}
	}
	if sample.invalid_transform_count != 0 || sample.below_floor_count != 0
	{
		return {status=.Validation_Failed};
	}
	return {status=.Ok};
}

benchmark_validate :: proc(
	owner: ^Benchmark_Owner,
	sample: ^support.Benchmark_Sample,
) -> support.Benchmark_Result
{
	stats, status := entasis.world_stats(&owner.world);
	if status != .Ok
	{
		return {status=.Validation_Failed, physics_status=status};
	}
	sample.stats = stats;
	profile := precision_profile(owner.precision_case);
	expected_step_index := u64(MEASURED_STEPS);
	if profile.fixture == .Query_Local || profile.fixture == .Query_Spanning
	{
		expected_step_index = 0;
	}
	if stats.step_index != expected_step_index ||
		stats.active_bodies+stats.sleeping_bodies != profile.body_count ||
		stats.statics != profile.static_count ||
		stats.registered_shapes != owner.registered_shapes
	{
		return {status=.Validation_Failed};
	}
	maintenance_interval := precision_case_maintenance_interval(owner.precision_case);
	expected_maintenance_count := 0;
	if maintenance_interval != 0
	{
		expected_maintenance_count = MEASURED_STEPS / maintenance_interval;
	}
	if owner.maintenance_count != expected_maintenance_count
	{
		return {status=.Validation_Failed};
	}
	switch profile.fixture
	{
		case .Compact: return compact_validate_positions(owner, sample);
		case .Crossing: return crossing_validate(owner, sample);
		case .Query_Local: return query_local_validate(owner, sample);
		case .Distributed, .Query_Spanning:
			when PRECISION_NATIVE_GLOBAL
			{
				if profile.fixture == .Distributed
				{
					return distributed_global_validate(owner, sample);
				}
				return global_spanning_validate(owner);
			}
			else
			{
				unreachable();
			}
	}
	unreachable();
}

measure_world :: proc(owner: ^Benchmark_Owner) -> (support.Benchmark_Result, support.Benchmark_Sample)
{
	maintenance_interval := precision_case_maintenance_interval(owner.precision_case);
	maintenance_index: int;
	mode := owner.sampling_mode;
	if mode == .Memory
	{
		status := precision_memory_world(&owner.world, &owner.memory.initial);
		if status != .Ok
		{
			return {status=.Validation_Failed, physics_status=status}, {};
		}
		precision_memory_begin(owner.memory);
	}
	whole_start := time.tick_now();
	for step_index in 0 ..< MEASURED_STEPS
	{
		frame_start: time.Tick;
		memory_frame: Memory_Frame;
		if mode == .Instrumented
		{
			frame_start = time.tick_now();
		}
		if maintenance_interval != 0 && (step_index+1) % maintenance_interval == 0
		{
			shift := MAINTENANCE_SHIFT;
			if maintenance_index % 2 != 0
			{
				shift = {-shift.x, -shift.y, -shift.z};
			}
			maintenance_start: time.Tick;
			maintenance_allocations: i64;
			if mode == .Instrumented
			{
				maintenance_start = time.tick_now();
			}
			if mode == .Memory
			{
				maintenance_allocations = owner.memory.tracker.total_allocation_count;
			}
			status := entasis.world_shift_origin(&owner.world, &owner.frame, shift);
			if mode == .Instrumented
			{
				owner.maintenance_nanoseconds[step_index] = i64(time.tick_diff(maintenance_start, time.tick_now()));
			}
			if mode == .Memory
			{
				memory_frame.maintenance_allocations = owner.memory.tracker.total_allocation_count-maintenance_allocations;
			}
			if status != .Ok
			{
				return {status=.Step_Failed, physics_status=status, phase=.Measured, failed_step=step_index+1}, {};
			}
			owner.maintenance_count += 1;
			maintenance_index += 1;
		}
		if mode == .Engine_Only
		{
			frame_start = time.tick_now();
		}
		status := entasis.world_step(&owner.world, 1/f32(owner.options.timestep_hz));
		if mode == .Engine_Only
		{
			owner.frame_nanoseconds[step_index] = i64(time.tick_diff(frame_start, time.tick_now()));
		}
		if status != .Ok
		{
			return {status=.Step_Failed, physics_status=status, phase=.Measured, failed_step=step_index+1}, {};
		}
		export_result := export_global_positions(owner);
		if export_result.status != .Ok
		{
			return export_result, {};
		}
		if mode == .Instrumented
		{
			owner.frame_nanoseconds[step_index] = i64(time.tick_diff(frame_start, time.tick_now()));
		}
		if mode == .Memory
		{
			memory_status := precision_memory_world(&owner.world, &memory_frame);
			if memory_status != .Ok
			{
				return {status=.Validation_Failed, physics_status=memory_status}, {};
			}
			precision_memory_record(owner.memory, memory_frame);
		}
	}
	whole_end := time.tick_now();
	sample := support.Benchmark_Sample{elapsed_ms=f64(time.tick_diff(whole_start, whole_end))/1e6};
	if mode == .Instrumented || mode == .Engine_Only
	{
		sampling_summarize(owner, MEASURED_STEPS);
		maintenance_max: i64;
		engine_total: i64;
		for duration, index in owner.frame_nanoseconds
		{
			engine_total += duration;
			maintenance_max = max(maintenance_max, owner.maintenance_nanoseconds[index]);
		}
		owner.maintenance_max_ms = f64(maintenance_max)/1e6;
		if mode == .Engine_Only
		{
			sample.elapsed_ms = f64(engine_total)/1e6;
		}
	}
	if sample.elapsed_ms <= 0 || math.is_nan(sample.elapsed_ms) || math.is_inf(sample.elapsed_ms, 0)
	{
		return {status=.Validation_Failed, phase=.Measured}, {};
	}
	result := benchmark_validate(owner, &sample);
	if result.status == .Ok && (mode == .Instrumented || mode == .Engine_Only)
	{
		for duration, frame_index in owner.frame_nanoseconds
		{
			maintenance_duration := owner.maintenance_nanoseconds[frame_index];
			fmt.printfln("precision_frame candidate=%s case=%s sampling=%s frame=%d elapsed_ms=%.9f maintenance=%d maintenance_elapsed_ms=%.9f",
				PRECISION_CANDIDATE, precision_case_text(owner.precision_case), sampling_mode_text(mode),
				frame_index+1, f64(duration)/1e6,
				1 if maintenance_interval != 0 && (frame_index+1)%maintenance_interval == 0 else 0,
				f64(maintenance_duration)/1e6);
		}
	}
	if mode == .Memory
	{
		precision_memory_emit(owner.memory, size_of(Benchmark_Storage)+size_of(Benchmark_Owner));
	}
	return result, sample;
}
