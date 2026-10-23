package precision_queries_tests

import "core:mem"
import "core:os"
import "core:testing"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

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
	world: entasis.World,
	query: entasis.Query_Context,
	simulation: ^physics.Simulation,
}

scene_initialize :: proc(t: ^testing.T, scene: ^Scene, capacity: i32 = 128)
{
	mem.tracking_allocator_init(&scene.tracker, os.heap_allocator(), os.heap_allocator());
	testing.expect_value(t, util.buffer_pool_initialize_with_allocator(
		&scene.pool, mem.tracking_allocator(&scene.tracker), 4096,
	), util.Memory_Status.Ok);
	description: entasis.World_Description = entasis.world_description_default();
	description.gravity = {};
	description.allocator = mem.tracking_allocator(&scene.tracker);
	description.capacity = {
		bodies=capacity, statics=capacity, inactive_body_sets=capacity, shapes_per_type=16,
		constraints=capacity * 4, initial_constraints_per_type_batch=16, minimum_constraints_per_body=8,
		broad_phase_candidates=capacity * 8, pairs=capacity * 8, inactive_pairs=capacity * 8,
		pending_pairs_per_worker=capacity * 4,
	};
	testing.expect_value(t, entasis.world_init_with_pool_and_allocation_scope(&scene.world, description, &scene.pool, .All_Owned), entasis.Status.Ok);
	status: physics.Physics_Status;
	scene.simulation, status = entasis.world_borrow_simulation(&scene.world);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	testing.expect_value(t, entasis.query_context_init(&scene.query, &scene.world), entasis.Status.Ok);
}

scene_dispose :: proc(t: ^testing.T, scene: ^Scene)
{
	testing.expect_value(t, entasis.query_context_destroy(&scene.query), entasis.Status.Ok);
	testing.expect_value(t, entasis.world_destroy(&scene.world), entasis.Status.Ok);
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

add_body :: proc(t: ^testing.T, scene: ^Scene, shape: physics.Typed_Index, position: physics.Global_Position,
	mobility: physics.Body_Mobility = .Dynamic) -> physics.Body_Handle
{
	description: physics.Body_Description = {
		pose={orientation=util.quaternion_identity()},
		local_inertia={inverse_mass=1, inverse_inertia_tensor={1, 0, 1, 0, 0, 1}},
		collidable={shape=shape},
		activity={sleep_threshold=0, minimum_timestep_count_under_threshold=255},
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

expect_position :: proc(t: ^testing.T, actual, expected: physics.Global_Position, tolerance: f64 = 0.000002)
{
	testing.expectf(t, abs(actual.x - expected.x) <= tolerance && abs(actual.y - expected.y) <= tolerance && abs(actual.z - expected.z) <= tolerance,
		"actual %v expected %v tolerance %v", actual, expected, tolerance);
}

translated_segment :: proc "contextless" (segment: physics.World_Precision_Segment, offset: physics.Global_Position) -> physics.World_Precision_Segment
{
	return {
		start={segment.start.x + offset.x, segment.start.y + offset.y, segment.start.z + offset.z},
		end={segment.end.x + offset.x, segment.end.y + offset.y, segment.end.z + offset.z},
	};
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
