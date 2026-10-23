package precision_collisions_tests

import "core:mem"
import "core:os"
import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Completion :: enum u8
{
	Stored,
	Materialized,
}

Shape_Kind :: enum u8
{
	Sphere,
	Capsule,
	Box,
	Triangle,
	Cylinder,
	Hull,
	Compound,
	Big_Compound,
	Mesh,
}

Scene :: struct
{
	tracker: mem.Tracking_Allocator,
	pool: util.Buffer_Pool,
	storage: util.Buffer(physics.Simulation),
	simulation: ^physics.Simulation,
}

Contact_Capture :: struct
{
	view: physics.Contact_Constraint_Data_View,
	count: int,
}

capture_contact :: proc "contextless" (user_context: rawptr, view: ^physics.Contact_Constraint_Data_View)
{
	capture: ^Contact_Capture = (^Contact_Capture)(user_context);
	capture.view = view^;
	capture.count += 1;
}

scene_initialize :: proc(t: ^testing.T, scene: ^Scene, completion: Completion = .Stored, capacity: i32 = 128)
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
		bodies=capacity, statics=capacity, inactive_body_sets=capacity, shapes_per_type=8,
		constraints=capacity * 4, constraint_batches=8, initial_constraints_per_type_batch=32,
		minimum_constraints_per_body=8, broad_phase_candidates=capacity * 32, pairs=capacity * 32,
		collision_child_pairs=capacity * 32, inactive_pairs=capacity * 32, pending_pairs_per_worker=capacity * 8, workers=4,
	};
	description.default_pose_context = {};
	testing.expect_value(t, physics.simulation_create(scene.simulation, &description).status, physics.Physics_Status.Ok);
	if completion == .Stored
	{
		testing.expect_value(t, physics.narrow_phase_bind_default_stored_completion(&scene.simulation.narrow_phase, {
			friction_coefficient=1, spring_settings=physics.spring_settings_create(30, 1), maximum_recovery_velocity=2,
		}), physics.Physics_Status.Ok);
	}
}

scene_dispose :: proc(t: ^testing.T, scene: ^Scene)
{
	testing.expect_value(t, physics.simulation_destroy(scene.simulation), physics.Physics_Status.Ok);
	testing.expect_value(t, util.buffer_pool_return(&scene.pool, &scene.storage), util.Memory_Status.Ok);
	testing.expect_value(t, util.buffer_pool_dispose(&scene.pool), util.Memory_Status.Ok);
	testing.expect_value(t, scene.tracker.current_memory_allocated, i64(0));
	mem.tracking_allocator_destroy(&scene.tracker);
}

add_shape :: proc(t: ^testing.T, scene: ^Scene, kind: Shape_Kind) -> physics.Typed_Index
{
	registry: ^physics.Shape_Registry = &scene.simulation.shapes;
	shape: physics.Typed_Index;
	status: physics.Physics_Status;
	switch kind
	{
	case .Sphere:
		value: physics.Sphere = {0.5};
		shape, status = physics.shape_registry_add(registry, physics.SPHERE_TYPE_ID, &value);
	case .Capsule:
		value: physics.Capsule = {0.5, 0.25};
		shape, status = physics.shape_registry_add(registry, physics.CAPSULE_TYPE_ID, &value);
	case .Box:
		value: physics.Box = {0.5, 0.5, 0.5};
		shape, status = physics.shape_registry_add(registry, physics.BOX_TYPE_ID, &value);
	case .Triangle:
		value: physics.Triangle = {{-1, 0, -1}, {1, 0, -1}, {-1, 0, 1}};
		shape, status = physics.shape_registry_add(registry, physics.TRIANGLE_TYPE_ID, &value);
	case .Cylinder:
		value: physics.Cylinder = {0.5, 0.5};
		shape, status = physics.shape_registry_add(registry, physics.CYLINDER_TYPE_ID, &value);
	case .Hull:
		points: [8]util.Vector3 = {
			{-0.5, -0.5, -0.5}, {0.5, -0.5, -0.5}, {0.5, 0.5, -0.5}, {-0.5, 0.5, -0.5},
			{-0.5, -0.5, 0.5}, {0.5, -0.5, 0.5}, {0.5, 0.5, 0.5}, {-0.5, 0.5, 0.5},
		};
		starts: [6]i32 = {0, 4, 8, 12, 16, 20};
		indices: [24]i32 = {0, 3, 2, 1, 4, 5, 6, 7, 0, 4, 7, 3, 1, 2, 6, 5, 0, 1, 5, 4, 3, 7, 6, 2};
		value: physics.Convex_Hull;
		testing.expect_value(t, physics.convex_hull_create(
			&value, &points[0], len(points), &starts[0], len(starts), &indices[0], len(indices), &scene.pool,
		), physics.Physics_Status.Ok);
		shape, status = physics.shape_registry_add(registry, physics.CONVEX_HULL_TYPE_ID, &value);
	case .Compound, .Big_Compound:
		leaf: physics.Typed_Index = add_shape(t, scene, .Sphere);
		children: [2]physics.Compound_Child = {
			{local_position={-0.5, 0, 0}, local_orientation=util.quaternion_identity(), shape_index=leaf},
			{local_position={0.5, 0, 0}, local_orientation=util.quaternion_identity(), shape_index=leaf},
		};
		if kind == .Compound
		{
			value: physics.Compound;
			testing.expect_value(t, physics.compound_create(&value, &children[0], len(children), &scene.pool), physics.Physics_Status.Ok);
			shape, status = physics.shape_registry_add(registry, physics.COMPOUND_TYPE_ID, &value);
		}
		else
		{
			value: physics.Big_Compound;
			testing.expect_value(t, physics.big_compound_create(&value, &children[0], len(children), registry, &scene.pool), physics.Physics_Status.Ok);
			shape, status = physics.shape_registry_add(registry, physics.BIG_COMPOUND_TYPE_ID, &value);
		}
	case .Mesh:
		triangles: [2]physics.Triangle = {
			{{-1, 0, -1}, {1, 0, -1}, {-1, 0, 1}},
			{{1, 0, -1}, {1, 0, 1}, {-1, 0, 1}},
		};
		value: physics.Mesh;
		testing.expect_value(t, physics.mesh_create(&value, &triangles[0], len(triangles), {1, 1, 1}, &scene.pool), physics.Physics_Status.Ok);
		shape, status = physics.shape_registry_add(registry, physics.MESH_TYPE_ID, &value);
	}
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	return shape;
}

add_body :: proc(
	t: ^testing.T, scene: ^Scene, shape: physics.Typed_Index, position: physics.Global_Position,
	mobility: physics.Body_Mobility = .Dynamic,
) -> physics.Body_Handle
{
	description: physics.Body_Description = {
		pose={orientation=util.quaternion_identity()},
		local_inertia={inverse_mass=1, inverse_inertia_tensor={1, 0, 1, 0, 0, 1}},
		collidable={shape=shape, minimum_speculative_margin=0, maximum_speculative_margin=0},
		activity={sleep_threshold=-1, minimum_timestep_count_under_threshold=255},
	};
	if mobility == .Kinematic
	{
		description.local_inertia = {};
	}
	handle: physics.Body_Handle;
	status: physics.Physics_Status;
	handle, status = physics.bodies_add(&scene.simulation.bodies, &description);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	testing.expect_value(t, physics.bodies_set_global_position(&scene.simulation.bodies, handle, position), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.broad_phase_add_body(&scene.simulation.broad_phase, handle), physics.Physics_Status.Ok);
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

contact_view :: proc(t: ^testing.T, scene: ^Scene, pair_index: int) -> physics.Contact_Constraint_Data_View
{
	cache: ^physics.Pair_Cache = &scene.simulation.narrow_phase.pair_cache;
	capture: Contact_Capture;
	testing.expect_value(t, physics.narrow_phase_try_extract_solver_contact_data(
		&scene.simulation.narrow_phase, cache.mapping.values.memory[pair_index].constraint_handle,
		capture_contact, &capture,
	), physics.Physics_Status.Ok);
	testing.expect_value(t, capture.count, 1);
	return capture.view;
}

expect_contact_sets_equal :: proc(t: ^testing.T, a, b: ^Scene)
{
	cache_a: ^physics.Pair_Cache = &a.simulation.narrow_phase.pair_cache;
	cache_b: ^physics.Pair_Cache = &b.simulation.narrow_phase.pair_cache;
	testing.expect_value(t, cache_a.mapping.count, cache_b.mapping.count);
	for index in 0 ..< cache_a.mapping.count
	{
		other: int = physics.pair_cache_index_of(cache_b, cache_a.mapping.keys.memory[index]);
		testing.expect(t, other >= 0);
		if other < 0
		{
			continue;
		}
		testing.expect_value(t, cache_a.mapping.values.memory[index].feature_ids, cache_b.mapping.values.memory[other].feature_ids);
		testing.expect_value(t, contact_view(t, a, index), contact_view(t, b, other));
	}
}

// borrowed until the next scene mutation
@(private="package")
body_motion :: proc(t: ^testing.T, scene: ^Scene, handle: physics.Body_Handle) -> ^physics.Motion_State
{
	location: physics.Body_Memory_Location;
	status: physics.Physics_Status;
	location, status = physics.bodies_resolve(&scene.simulation.bodies, handle);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	return &scene.simulation.bodies.sets.memory[location.set_index].dynamics_state.memory[location.index].motion;
}
