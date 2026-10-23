package precision_broadphase_tests

import "core:mem"
import "core:os"
import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Scene :: struct
{
	pool: util.Buffer_Pool,
	tracker: mem.Tracking_Allocator,
	simulation: ^physics.Simulation,
	storage: util.Buffer(physics.Simulation),
	shapes: [10]physics.Typed_Index,
}

Custom_Sphere :: struct
{
	radius: f32,
}

custom_bounds :: proc "contextless" (
	shape: rawptr, orientation: util.Quaternion, registry: ^physics.Shape_Registry,
) -> (physics.Shape_Bounds, physics.Physics_Status)
{
	_ = registry;
	return physics.sphere_bounds({(^Custom_Sphere)(shape).radius}, orientation), .Ok;
}

custom_inertia :: proc "contextless" (
	shape: rawptr, registry: ^physics.Shape_Registry, mass: f32,
) -> (physics.Body_Inertia, physics.Physics_Status)
{
	_ = registry;
	return physics.sphere_inertia({(^Custom_Sphere)(shape).radius}, mass);
}

custom_ray :: proc "contextless" (
	shape: rawptr, pose: physics.Rigid_Pose, ray: physics.Tree_Ray, registry: ^physics.Shape_Registry,
) -> (physics.Shape_Ray_Hit, physics.Physics_Status)
{
	_ = registry;
	return physics.sphere_ray_test({(^Custom_Sphere)(shape).radius}, pose, ray);
}

custom_support :: proc "contextless" (
	shape: rawptr, direction: util.Vector3, registry: ^physics.Shape_Registry,
) -> (util.Vector3, physics.Physics_Status)
{
	_ = registry;
	return physics.sphere_support({(^Custom_Sphere)(shape).radius}, direction);
}

scene_initialize :: proc(t: ^testing.T, scene: ^Scene, capacity: i32 = 128, workers: i32 = 4)
{
	mem.tracking_allocator_init(&scene.tracker, os.heap_allocator(), os.heap_allocator());
	testing.expect_value(t, util.buffer_pool_initialize_with_allocator(
		&scene.pool, mem.tracking_allocator(&scene.tracker), 4096,
	), util.Memory_Status.Ok);
	memory_status: util.Memory_Status;
	scene.storage, memory_status = util.buffer_pool_take_at_least(&scene.pool, physics.Simulation, 1);
	testing.expect_value(t, memory_status, util.Memory_Status.Ok);
	scene.simulation = &scene.storage.memory[0];
	scene.simulation^ = {};
	description: physics.Simulation_Create_Description = physics.simulation_create_description_default(&scene.pool);
	description.allocation_sizes = {
		bodies=capacity, statics=64, inactive_body_sets=capacity, shapes_per_type=8,
		constraints=capacity * 4, constraint_batches=8, initial_constraints_per_type_batch=16,
		minimum_constraints_per_body=4, broad_phase_candidates=capacity * 16, pairs=capacity * 16,
		inactive_pairs=capacity * 16, pending_pairs_per_worker=capacity * 4, workers=workers,
	};
	description.default_pose_context = {};
	testing.expect_value(t, physics.simulation_create(scene.simulation, &description).status, physics.Physics_Status.Ok);

	registry: ^physics.Shape_Registry = &scene.simulation.shapes;
	status: physics.Physics_Status;
	sphere: physics.Sphere = {0.0625};
	capsule: physics.Capsule = {0.0625, 0.125};
	box: physics.Box = {0.0625, 0.125, 0.25};
	triangle: physics.Triangle = {{-0.125, 0, -0.125}, {0.125, 0, -0.125}, {0, 0, 0.125}};
	cylinder: physics.Cylinder = {0.0625, 0.125};
	scene.shapes[0], status = physics.shape_registry_add(registry, physics.SPHERE_TYPE_ID, &sphere);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	scene.shapes[1], status = physics.shape_registry_add(registry, physics.CAPSULE_TYPE_ID, &capsule);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	scene.shapes[2], status = physics.shape_registry_add(registry, physics.BOX_TYPE_ID, &box);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	scene.shapes[3], status = physics.shape_registry_add(registry, physics.TRIANGLE_TYPE_ID, &triangle);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	scene.shapes[4], status = physics.shape_registry_add(registry, physics.CYLINDER_TYPE_ID, &cylinder);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	points: [8]util.Vector3 = {
		{-0.125, -0.125, -0.125}, {0.125, -0.125, -0.125}, {0.125, 0.125, -0.125}, {-0.125, 0.125, -0.125},
		{-0.125, -0.125, 0.125}, {0.125, -0.125, 0.125}, {0.125, 0.125, 0.125}, {-0.125, 0.125, 0.125},
	};
	starts: [6]i32 = {0, 4, 8, 12, 16, 20};
	indices: [24]i32 = {0, 3, 2, 1, 4, 5, 6, 7, 0, 4, 7, 3, 1, 2, 6, 5, 0, 1, 5, 4, 3, 7, 6, 2};
	hull: physics.Convex_Hull;
	testing.expect_value(t, physics.convex_hull_create(
		&hull, &points[0], len(points), &starts[0], len(starts), &indices[0], len(indices), &scene.pool,
	), physics.Physics_Status.Ok);
	scene.shapes[5], status = physics.shape_registry_add(registry, physics.CONVEX_HULL_TYPE_ID, &hull);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	children: [2]physics.Compound_Child = {
		{local_position={-0.25, 0, 0}, local_orientation=util.quaternion_identity(), shape_index=scene.shapes[0]},
		{local_position={0.25, 0, 0}, local_orientation=util.quaternion_identity(), shape_index=scene.shapes[2]},
	};
	compound: physics.Compound;
	testing.expect_value(t, physics.compound_create(&compound, &children[0], len(children), &scene.pool), physics.Physics_Status.Ok);
	scene.shapes[6], status = physics.shape_registry_add(registry, physics.COMPOUND_TYPE_ID, &compound);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	big_compound: physics.Big_Compound;
	testing.expect_value(t, physics.big_compound_create(&big_compound, &children[0], len(children), registry, &scene.pool), physics.Physics_Status.Ok);
	scene.shapes[7], status = physics.shape_registry_add(registry, physics.BIG_COMPOUND_TYPE_ID, &big_compound);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	mesh: physics.Mesh;
	testing.expect_value(t, physics.mesh_create(&mesh, &triangle, 1, {1, 1, 1}, &scene.pool), physics.Physics_Status.Ok);
	scene.shapes[8], status = physics.shape_registry_add(registry, physics.MESH_TYPE_ID, &mesh);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	registration: physics.Shape_Type_Registration = {
		size=size_of(Custom_Sphere), alignment=align_of(Custom_Sphere), batch_type=.Convex,
		bounds=custom_bounds, inertia=custom_inertia, ray=custom_ray, support=custom_support,
		sweep_support=custom_support, dispose=physics.shape_no_dispose,
	};
	custom_type: i32;
	custom_type, status = physics.shape_registry_register_custom(registry, registration);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	custom: Custom_Sphere = {0.0625};
	scene.shapes[9], status = physics.shape_registry_add(registry, int(custom_type), &custom);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
}

scene_dispose :: proc(t: ^testing.T, scene: ^Scene)
{
	testing.expect_value(t, physics.simulation_destroy(scene.simulation), physics.Physics_Status.Ok);
	testing.expect_value(t, util.buffer_pool_return(&scene.pool, &scene.storage), util.Memory_Status.Ok);
	testing.expect_value(t, util.buffer_pool_dispose(&scene.pool), util.Memory_Status.Ok);
	testing.expect_value(t, scene.tracker.current_memory_allocated, i64(0));
	mem.tracking_allocator_destroy(&scene.tracker);
}

body_description :: proc "contextless" (shape: physics.Typed_Index) -> physics.Body_Description
{
	return {
		pose={orientation=util.quaternion_identity()},
		local_inertia={inverse_mass=1, inverse_inertia_tensor={1, 0, 1, 0, 0, 1}},
		collidable={shape=shape, minimum_speculative_margin=0.0625, maximum_speculative_margin=0.0625},
		activity={sleep_threshold=0, minimum_timestep_count_under_threshold=255},
	};
}

add_body :: proc(t: ^testing.T, scene: ^Scene, shape: physics.Typed_Index, position: physics.Global_Position) -> physics.Body_Handle
{
	description: physics.Body_Description = body_description(shape);
	handle: physics.Body_Handle;
	status: physics.Physics_Status;
	handle, status = physics.bodies_add(&scene.simulation.bodies, &description);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	testing.expect_value(t, physics.bodies_set_global_position(&scene.simulation.bodies, handle, position), physics.Physics_Status.Ok);
	if physics.typed_index_state(shape) == .Present
	{
		testing.expect_value(t, physics.broad_phase_add_body(&scene.simulation.broad_phase, handle), physics.Physics_Status.Ok);
	}
	return handle;
}

add_static :: proc(t: ^testing.T, scene: ^Scene, shape: physics.Typed_Index, position: physics.Global_Position) -> physics.Static_Handle
{
	description: physics.Static_Description = {pose={orientation=util.quaternion_identity()}, shape=shape};
	handle: physics.Static_Handle;
	status: physics.Physics_Status;
	handle, status = physics.statics_add(&scene.simulation.statics, &description);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	testing.expect_value(t, physics.statics_set_global_position(&scene.simulation.statics, handle, position), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.broad_phase_add_static(&scene.simulation.broad_phase, handle), physics.Physics_Status.Ok);
	return handle;
}

body_bounds :: proc(t: ^testing.T, scene: ^Scene, handle: physics.Body_Handle) -> physics.Global_Bounds
{
	location: physics.Body_Memory_Location;
	status: physics.Physics_Status;
	location, status = physics.bodies_resolve(&scene.simulation.bodies, handle);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	set: ^physics.Body_Set = &scene.simulation.bodies.sets.memory[location.set_index];
	tree: ^physics.Tree = &scene.simulation.broad_phase.active_tree;
	if location.set_index != 0
	{
		tree = &scene.simulation.broad_phase.static_tree;
	}
	bounds: physics.Global_Bounds;
	bounds, status = physics.tree_get_leaf_bounds(tree, int(set.collidables.memory[location.index].broad_phase_index));
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	return bounds;
}

static_bounds :: proc(t: ^testing.T, scene: ^Scene, handle: physics.Static_Handle) -> physics.Global_Bounds
{
	index: int;
	status: physics.Physics_Status;
	index, status = physics.statics_resolve(&scene.simulation.statics, handle);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	bounds: physics.Global_Bounds;
	bounds, status = physics.tree_get_leaf_bounds(
		&scene.simulation.broad_phase.static_tree, int(scene.simulation.statics.statics.memory[index].broad_phase_index),
	);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	return bounds;
}

// all oracle positions/extents in these tests are within 1 Gm and exactly
// representable in f64. this does not reuse the production outward converter
expect_enclosure :: proc(t: ^testing.T, actual: physics.Global_Bounds, position: physics.Global_Position, local: $Bounds)
{
	testing.expect(t, f64(actual.min.x) <= position.x + f64(local.min.x));
	testing.expect(t, f64(actual.min.y) <= position.y + f64(local.min.y));
	testing.expect(t, f64(actual.min.z) <= position.z + f64(local.min.z));
	testing.expect(t, f64(actual.max.x) >= position.x + f64(local.max.x));
	testing.expect(t, f64(actual.max.y) >= position.y + f64(local.max.y));
	testing.expect(t, f64(actual.max.z) >= position.z + f64(local.max.z));
}

local_bounds :: proc(t: ^testing.T, scene: ^Scene, shape: physics.Typed_Index, orientation: util.Quaternion) -> physics.Shape_Bounds
{
	bounds: physics.Shape_Bounds;
	status: physics.Physics_Status;
	bounds, status = physics.shape_registry_compute_bounds(&scene.simulation.shapes, shape, orientation);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	return bounds;
}

expect_body_geometry :: proc(t: ^testing.T, scene: ^Scene, handle: physics.Body_Handle)
{
	location: physics.Body_Memory_Location;
	location, _ = physics.bodies_resolve(&scene.simulation.bodies, handle);
	set: ^physics.Body_Set = &scene.simulation.bodies.sets.memory[location.set_index];
	local: physics.Shape_Bounds = local_bounds(t, scene, set.collidables.memory[location.index].shape,
		set.dynamics_state.memory[location.index].motion.pose.orientation);
	expect_enclosure(t, body_bounds(t, scene, handle), set.global_positions.memory[location.index], util.Bounding_Box{min=local.min, max=local.max});
	expect_precise_enclosure(t, body_precise_bounds(t, scene, handle), set.global_positions.memory[location.index], util.Bounding_Box{min=local.min, max=local.max});
}

sleep_body :: proc(t: ^testing.T, scene: ^Scene, handle: physics.Body_Handle, dispatcher: ^util.Thread_Dispatcher_Boundary = nil)
{
	location: physics.Body_Memory_Location;
	location, _ = physics.bodies_resolve(&scene.simulation.bodies, handle);
	scene.simulation.bodies.sets.memory[location.set_index].activity.memory[location.index].sleep_candidate = .Candidate;
	sleeper: ^physics.Island_Sleeper = &scene.simulation.sleeper;
	testing.expect_value(t, physics.island_scaffold_clear(&sleeper.scaffold), physics.Physics_Status.Ok);
	bodies, constraints: int;
	state: physics.Reference_State;
	status: physics.Physics_Status;
	bodies, constraints, state, status = physics.island_sleeper_build_island(sleeper, &sleeper.scaffold, handle);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	testing.expect_value(t, state, physics.Reference_State.Present);
	testing.expect_value(t, physics.island_sleeper_sleep_island(sleeper, &sleeper.scaffold, bodies, constraints, dispatcher), physics.Physics_Status.Ok);
}
