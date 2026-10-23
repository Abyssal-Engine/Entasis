package precision_targets_tests

import "core:mem"
import "core:os"
import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Scene :: struct
{
	tracker: mem.Tracking_Allocator,
	pool: util.Buffer_Pool,
	storage: util.Buffer(physics.Simulation),
	simulation: ^physics.Simulation,
}

scene_initialize :: proc(t: ^testing.T, scene: ^Scene, capacity: i32 = 128, substeps: i32 = 1, sequential_threshold: i32 = 4)
{
	mem.tracking_allocator_init(&scene.tracker, os.heap_allocator(), os.heap_allocator());
	testing.expect_value(t, util.buffer_pool_initialize_with_allocator(
		&scene.pool, mem.tracking_allocator(&scene.tracker), 4096,
	), util.Memory_Status.Ok);
	status: util.Memory_Status;
	scene.storage, status = util.buffer_pool_take_at_least(&scene.pool, physics.Simulation, 1);
	testing.expect_value(t, status, util.Memory_Status.Ok);
	scene.simulation = &scene.storage.memory[0];
	scene.simulation^ = {};
	description: physics.Simulation_Create_Description = physics.simulation_create_description_default(&scene.pool);
	description.allocation_sizes = {
		bodies=capacity, statics=8, inactive_body_sets=32, shapes_per_type=8,
		constraints=capacity * 4, constraint_batches=sequential_threshold + 2, initial_constraints_per_type_batch=32,
		minimum_constraints_per_body=8, broad_phase_candidates=64, pairs=64,
		collision_child_pairs=64, inactive_pairs=64, pending_pairs_per_worker=64, workers=4,
	};
	description.solve_description = {velocity_iteration_count=4, substep_count=substeps, fallback_batch_threshold=sequential_threshold};
	description.default_pose_context = {};
	testing.expect_value(t, physics.simulation_create(scene.simulation, &description).status, physics.Physics_Status.Ok);
}

scene_dispose :: proc(t: ^testing.T, scene: ^Scene)
{
	testing.expect_value(t, physics.simulation_destroy(scene.simulation), physics.Physics_Status.Ok);
	testing.expect_value(t, util.buffer_pool_return(&scene.pool, &scene.storage), util.Memory_Status.Ok);
	testing.expect_value(t, util.buffer_pool_dispose(&scene.pool), util.Memory_Status.Ok);
	testing.expect_value(t, scene.tracker.current_memory_allocated, i64(0));
	mem.tracking_allocator_destroy(&scene.tracker);
}

add_body :: proc(t: ^testing.T, scene: ^Scene, position: physics.Global_Position, mobility: physics.Body_Mobility = .Dynamic) -> physics.Body_Handle
{
	description: physics.Body_Description = {
		pose={orientation=util.quaternion_identity()},
		local_inertia={inverse_mass=1, inverse_inertia_tensor={1, 0, 1, 0, 0, 1}},
		collidable={maximum_speculative_margin=0.5},
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
	// direct solver tests bypass pose preparation and use identity orientations
	index: i32 = scene.simulation.bodies.handle_to_location.memory[handle.value].index;
	dynamics: ^physics.Body_Dynamics = &scene.simulation.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX].dynamics_state.memory[index];
	dynamics.inertia.world = dynamics.inertia.local;
	return handle;
}

body_position :: proc(t: ^testing.T, scene: ^Scene, handle: physics.Body_Handle) -> physics.Global_Position
{
	position: physics.Global_Position;
	status: physics.Physics_Status;
	position, status = physics.bodies_get_global_position(&scene.simulation.bodies, handle);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	return position;
}

// borrowed until the next body storage mutation in this fixture
@(private="package")
body_motion :: proc(t: ^testing.T, scene: ^Scene, handle: physics.Body_Handle) -> ^physics.Motion_State
{
	location: physics.Body_Memory_Location;
	status: physics.Physics_Status;
	location, status = physics.bodies_resolve(&scene.simulation.bodies, handle);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	return &scene.simulation.bodies.sets.memory[location.set_index].dynamics_state.memory[location.index].motion;
}

// the caller owns the scene and does not mutate constraint storage while borrowed
@(private="package")
constraint_batch :: proc(t: ^testing.T, scene: ^Scene, handle: physics.Constraint_Handle) -> ^physics.Type_Batch
{
	location: physics.Constraint_Location;
	status: physics.Physics_Status;
	location, status = physics.solver_resolve(&scene.simulation.solver, handle);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	batch: ^physics.Constraint_Batch = &scene.simulation.solver.active_set.batches.memory[location.batch_index];
	return &batch.type_batches.memory[batch.type_id_to_batch_index[location.type_id]];
}

expect_vector :: proc(t: ^testing.T, actual, expected: util.Vector3, tolerance: f32 = 0.00002)
{
	testing.expectf(t, abs(actual.x - expected.x) <= tolerance && abs(actual.y - expected.y) <= tolerance && abs(actual.z - expected.z) <= tolerance,
		"actual %v expected %v tolerance %v", actual, expected, tolerance);
}
