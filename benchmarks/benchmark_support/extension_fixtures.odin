package benchmark_support

import "core:fmt"
import "core:math"
import "core:os"
import "core:strconv"
import "core:strings"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

BENCHMARK_COMPONENTS :: #config(ENTASIS_BENCHMARK_COMPONENTS, "all");
#assert(BENCHMARK_COMPONENTS == "common" || BENCHMARK_COMPONENTS == "all");
Extension_CSV_State :: enum u8
{
	Existing,
	Empty,
}

// frozen public-Odin parity fixtures. helpers do not instrument normal engine paths
Extension_Arguments :: struct
{
	workers: int,
	output: string,
}

extension_arguments :: proc(args: []string) -> (Extension_Arguments, Admission)
{
	result: Extension_Arguments;
	if len(args) != 3
	{
		return {}, .Invalid;
	}
	for arg in args[1:]
	{
		if strings.has_prefix(arg, "--worker-count=")
		{
			if result.workers != 0
			{
				return {}, .Invalid;
			}
			n: int;
			ok: bool;
			n, ok = strconv.parse_int(arg[len("--worker-count="):]);
			if !ok || n <= 0 || n > entasis.MAXIMUM_WORKER_COUNT
			{
				return {}, .Invalid;
			}
			result.workers = n;
		}
		else if strings.has_prefix(arg, "--output=")
		{
			if len(result.output) != 0
			{
				return {}, .Invalid;
			}
			result.output = arg[len("--output="):];
		}
		else
		{
			return {}, .Invalid;
		}
	}
	if result.workers == 0 || len(result.output) == 0
	{
		return {}, .Invalid;
	}
	return result, .Ok;
}

extension_require :: proc(status: entasis.Status, operation: string)
{
	if status != .Ok
	{
		fmt.eprintfln("extension_fixture_failed operation=%s status=%v", operation, status);
		os.exit(1);
	}
}

extension_csv_open :: proc(path: string) -> (^os.File, Extension_CSV_State)
{
	info: os.File_Info;
	stat_error: os.Error;
	info, stat_error = os.stat(path, context.temp_allocator);
	if stat_error != nil && stat_error != os.Error(os.General_Error.Not_Exist)
	{
		panic("cannot stat CSV output");
	}
	state: Extension_CSV_State = .Existing;
	if stat_error != nil || info.size == 0
	{
		state = .Empty;
	}
	file: ^os.File;
	err: os.Error;
	file, err = os.open(path, {.Write, .Append, .Create}, os.Permissions_Default_File);
	if err != nil
	{
		panic("cannot open CSV output");
	}
	return file, state;
}

extension_csv_close :: proc(file: ^os.File)
{
	if os.close(file) != nil
	{
		panic("cannot close CSV output");
	}
}

Custom_Sphere :: struct
{
	radius: f32,
}

custom_sphere_value :: #force_inline proc "contextless" (shape: rawptr) -> physics.Sphere
{
	return {radius=(^Custom_Sphere)(shape).radius};
}

custom_sphere_bounds :: proc "contextless" (
shape: rawptr, orientation: util.Quaternion, registry: ^physics.Shape_Registry,
) -> (physics.Shape_Bounds, physics.Physics_Status)
{
	_ = registry;
	value: physics.Sphere = custom_sphere_value(shape);
	if physics.sphere_validate(value) != .Ok
	{
		return {}, .Invalid_Description;
	}
	return physics.sphere_bounds(value, orientation), .Ok;
}

custom_sphere_inertia :: proc "contextless" (
shape: rawptr, registry: ^physics.Shape_Registry, mass: f32,
) -> (physics.Body_Inertia, physics.Physics_Status)
{
	_ = registry;
	return physics.sphere_inertia(custom_sphere_value(shape), mass);
}

custom_sphere_ray :: proc "contextless" (
shape: rawptr, pose_value: physics.Rigid_Pose, ray_value: physics.Tree_Ray,
registry: ^physics.Shape_Registry,
) -> (physics.Shape_Ray_Hit, physics.Physics_Status)
{
	_ = registry;
	return physics.sphere_ray_test(custom_sphere_value(shape), pose_value, ray_value);
}

custom_sphere_support :: proc "contextless" (
shape: rawptr, direction: util.Vector3, registry: ^physics.Shape_Registry,
) -> (util.Vector3, physics.Physics_Status)
{
	_ = registry;
	return physics.sphere_support(custom_sphere_value(shape), direction);
}

custom_sphere_pair :: proc "contextless" (
shape_a, shape_b: rawptr,
pose_a, pose_b: physics.Rigid_Pose,
speculative_margin: f32,
shapes: ^physics.Shape_Registry,
) -> (physics.Convex_Contact_Manifold, physics.Physics_Status)
{
	a: physics.Sphere = custom_sphere_value(shape_a);
	b: physics.Sphere = custom_sphere_value(shape_b);
	return physics.sphere_pair_test(&a, &b, pose_a, pose_b, speculative_margin, shapes);
}

custom_sphere_builtin_pair :: proc "contextless" (
shape_a, shape_b: rawptr,
pose_a, pose_b: physics.Rigid_Pose,
speculative_margin: f32,
shapes: ^physics.Shape_Registry,
) -> (physics.Convex_Contact_Manifold, physics.Physics_Status)
{
	a: physics.Sphere = custom_sphere_value(shape_a);
	return physics.sphere_pair_test(&a, shape_b, pose_a, pose_b, speculative_margin, shapes);
}

custom_sweep_radius :: #force_inline proc "contextless" (shape: rawptr, type_id: int) -> f32
{
	if type_id == physics.SPHERE_TYPE_ID
	{
		return (^physics.Sphere)(shape).radius;
	}
	return (^Custom_Sphere)(shape).radius;
}

custom_sphere_sweep :: proc "contextless" (
shape_a, shape_b: rawptr,
type_a, type_b: int,
pose_a, pose_b: physics.Rigid_Pose,
velocity_a, velocity_b: physics.Body_Velocity,
maximum_t, minimum_progression, convergence_threshold: f32,
maximum_iteration_count: int,
shapes: ^physics.Shape_Registry,
collision_tasks: ^physics.Collision_Task_Registry,
filter: physics.Collision_Child_Filter_Proc,
user_context: rawptr,
) -> (physics.Sweep_Result, physics.Physics_Status)
{
	_, _, _, _, _ = minimum_progression, convergence_threshold, maximum_iteration_count, shapes, collision_tasks;
	_, _ = filter, user_context;
	radius: f32 = custom_sweep_radius(shape_a, type_a) + custom_sweep_radius(shape_b, type_b);
	position: util.Vector3 = util.vector3_subtract(pose_a.position, pose_b.position);
	velocity_value: util.Vector3 = util.vector3_subtract(velocity_a.linear, velocity_b.linear);
	c: f32 = util.vector3_dot(position, position) - radius * radius;
	t: f32 = f32(0);
	if c > 0
	{
		a: f32 = util.vector3_dot(velocity_value, velocity_value);
		if a <= 1e-20
		{
			return {state=.Miss, t0=maximum_t, t1=maximum_t, child_a=-1, child_b=-1}, .Ok;
		}
		b: f32 = util.vector3_dot(position, velocity_value);
		discriminant: f32 = b * b - a * c;
		if discriminant < 0
		{
			return {state=.Miss, t0=maximum_t, t1=maximum_t, child_a=-1, child_b=-1}, .Ok;
		}
		t = (-b - math.sqrt(discriminant)) / a;
		if t < 0 || t > maximum_t
		{
			return {state=.Miss, t0=maximum_t, t1=maximum_t, child_a=-1, child_b=-1}, .Ok;
		}
	}
	center_a: util.Vector3 = util.vector3_add(pose_a.position, util.vector3_scale(velocity_a.linear, t));
	center_b: util.Vector3 = util.vector3_add(pose_b.position, util.vector3_scale(velocity_b.linear, t));
	offset: util.Vector3 = util.vector3_subtract(center_a, center_b);
	distance: f32 = util.vector3_length(offset);
	normal: util.Vector3 = util.Vector3{1, 0, 0};
	if distance > 1e-20
	{
		normal = util.vector3_scale(offset, 1 / distance);
	}
	return {
		state=.Hit, t0=t, t1=t,
		location=util.vector3_add(center_b, util.vector3_scale(normal, custom_sweep_radius(shape_b, type_b))),
		normal=normal, child_a=-1, child_b=-1,
	}, .Ok;
}

custom_sphere_sweep_child :: proc "contextless" (
shape_a, shape_b: rawptr,
type_a, type_b: int,
parent_pose_a, parent_pose_b: physics.Rigid_Pose,
local_pose_a, local_pose_b: physics.Rigid_Pose,
velocity_a, velocity_b: physics.Body_Velocity,
maximum_t, minimum_progression, convergence_threshold: f32,
maximum_iteration_count: int,
shapes: ^physics.Shape_Registry,
collision_tasks: ^physics.Collision_Task_Registry,
) -> (physics.Sweep_Result, physics.Physics_Status)
{
	return custom_sphere_sweep(
	shape_a, shape_b, type_a, type_b,
	physics.rigid_pose_concatenate(local_pose_a, parent_pose_a),
	physics.rigid_pose_concatenate(local_pose_b, parent_pose_b),
	velocity_a, velocity_b, maximum_t, minimum_progression,
	convergence_threshold, maximum_iteration_count,
	shapes, collision_tasks, nil, nil,
	);
}

custom_sphere_box_pair :: proc "contextless" (
a, b: rawptr, pa, pb: physics.Rigid_Pose, margin: f32, shapes: ^physics.Shape_Registry,
) -> (physics.Convex_Contact_Manifold, physics.Physics_Status)
{
	sphere: physics.Sphere = custom_sphere_value(a);
	return physics.sphere_box_test(&sphere, b, pa, pb, margin, shapes);
}

// includes the otherwise-missing custom-versus-built-in floor route
extension_register_sphere :: proc(world: ^entasis.World) -> (entasis.Shape_Type_ID, entasis.Status)
{
	next: entasis.Shape_Type_ID;
	next_status: entasis.Status;
	next, next_status = entasis.custom_shape_next_type_id(world);
	if next_status != .Ok
	{
		return {}, next_status;
	}
	id: entasis.Shape_Type_ID;
	status: entasis.Status;
	id, status = entasis.custom_shape_register(world, entasis.custom_shape_registration(
	Custom_Sphere, .Convex, custom_sphere_bounds, custom_sphere_inertia,
	custom_sphere_ray, custom_sphere_support, expected_type_id=next,
	));
	if status != .Ok
	{
		return {}, status;
	}
	_, status = entasis.collision_task_register(world, entasis.collision_task_convex(
	id, id, 16, .Sphere, custom_sphere_pair, custom_sphere_pair_wide,
	));
	if status != .Ok
	{
		return {}, status;
	}
	_, status = entasis.collision_task_register(world, entasis.collision_task_convex(
	id, entasis.SHAPE_TYPE_SPHERE, 16, .Sphere, custom_sphere_builtin_pair,
	custom_sphere_pair_wide,
	));
	if status != .Ok
	{
		return {}, status;
	}
	_, status = entasis.collision_task_register(world, entasis.collision_task_convex(
	id, entasis.SHAPE_TYPE_BOX, 16, .Sphere_Including, custom_sphere_box_pair,
	custom_sphere_box_wide,
	));
	if status != .Ok
	{
		return {}, status;
	}
	_, status = entasis.sweep_task_register(world, entasis.sweep_task_registration(
	id, id, custom_sphere_sweep, custom_sphere_sweep_child,
	));
	if status != .Ok
	{
		return {}, status;
	}
	_, status = entasis.sweep_task_register(world, entasis.sweep_task_registration(
	entasis.SHAPE_TYPE_SPHERE, id, custom_sphere_sweep, custom_sphere_sweep_child,
	));
	if status != .Ok
	{
		return {}, status;
	}
	return id, .Ok;
}

// the built-in gather only fills typed payloads for built-in type IDs. a native
// custom task must pack its own raw shape pointers before invoking a wide kernel.
// reusing example 22's built-in callback directly would silently test empty data
custom_sphere_pair_wide :: proc "contextless" (
bundle: ^physics.Collision_Convex_Wide_Bundle, _: ^physics.Shape_Registry,
) -> (physics.Collision_Wide_Manifold_Result, physics.Physics_Status)
{
	a, b: physics.Sphere_Wide;
	for lane in 0..<util.PRODUCTION_LANE_COUNT
	{
		physics.sphere_wide_write_slot_trusted(&a, lane, custom_sphere_value(bundle.shape_a[lane]));
		physics.sphere_wide_write_slot_trusted(&b, lane, custom_sphere_value(bundle.shape_b[lane]));
	}
	wide: physics.Convex_1_Contact_Manifold_Wide;
	status: physics.Physics_Status;
	wide, status = physics.sphere_pair_test_wide(a, b, bundle.speculative_margin, bundle.offset_b, bundle.count);
	return {kind=.One_Contact, one=wide}, status;
}

custom_sphere_box_wide :: proc "contextless" (
bundle: ^physics.Collision_Convex_Wide_Bundle, _: ^physics.Shape_Registry,
) -> (physics.Collision_Wide_Manifold_Result, physics.Physics_Status)
{
	a: physics.Sphere_Wide;
	for lane in 0..<util.PRODUCTION_LANE_COUNT
	{
		physics.sphere_wide_write_slot_trusted(&a, lane, custom_sphere_value(bundle.shape_a[lane]));
	}
	wide: physics.Convex_1_Contact_Manifold_Wide;
	status: physics.Physics_Status;
	wide, status = physics.sphere_box_test_wide(a, bundle.b.box, bundle.speculative_margin, bundle.offset_b, bundle.orientation_b, bundle.count);
	return {kind=.One_Contact, one=wide}, status;
}
