package precision_callbacks_tests

import "core:mem"
import "core:os"
import "core:testing"
import e "entasis:entasis"
import p "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Scene :: struct
{
	tracker: mem.Tracking_Allocator,
	pool: util.Buffer_Pool,
	world: e.World,
	simulation: ^p.Simulation,
	sphere, box: e.Shape_Handle,
}

scene_initialize :: proc(t: ^testing.T, scene: ^Scene, capacity: i32 = 32, substeps: i32 = 1, workers: i32 = 1,
	callbacks: e.Pose_Callbacks = {}, threshold: i32 = 4)
{
	mem.tracking_allocator_init(&scene.tracker, os.heap_allocator(), os.heap_allocator());
	backing: mem.Allocator = mem.tracking_allocator(&scene.tracker);
	testing.expect_value(t, util.buffer_pool_initialize_with_allocator(
		&scene.pool, backing, 4096,
	), util.Memory_Status.Ok);
	description: e.World_Description = e.world_description_default();
	description.pose_callbacks = callbacks;
	description.gravity = {};
	description.damping = {};
	description.allocator = backing;
	description.capacity = {
		bodies=capacity, statics=capacity, inactive_body_sets=capacity, shapes_per_type=16,
		constraints=capacity * 4, initial_constraints_per_type_batch=16, minimum_constraints_per_body=8,
		broad_phase_candidates=capacity * 8, pairs=capacity * 8, inactive_pairs=capacity * 8,
		pending_pairs_per_worker=capacity * 4,
	};
	description.solve.substeps = substeps;
	description.solve.fallback_batch_threshold = threshold;
	description.threading.worker_count = workers;
	testing.expect_value(t, e.world_init_with_pool_and_allocation_scope(&scene.world, description, &scene.pool, .All_Owned), e.Status.Ok);
	status: e.Status;
	scene.simulation, status = e.world_borrow_simulation(&scene.world);
	testing.expect_value(t, status, e.Status.Ok);
	scene.sphere, status = e.shape_add(&scene.world, e.sphere(0.5));
	testing.expect_value(t, status, e.Status.Ok);
	scene.box, status = e.shape_add(&scene.world, e.box(1, 1, 1));
	testing.expect_value(t, status, e.Status.Ok);
}

scene_dispose :: proc(t: ^testing.T, scene: ^Scene)
{
	testing.expect_value(t, e.world_destroy(&scene.world), e.Status.Ok);
	testing.expect_value(t, util.buffer_pool_dispose(&scene.pool), util.Memory_Status.Ok);
	testing.expect_value(t, scene.tracker.current_memory_allocated, i64(0));
	mem.tracking_allocator_destroy(&scene.tracker);
}

body_description :: proc(shape: e.Shape_Handle) -> e.Body_Description
{
	return e.body_dynamic(shape, {inverse_mass=1, inverse_inertia_tensor={1, 0, 1, 0, 0, 1}},
		e.pose(), {}, e.body_activity(-1, 255));
}

add_body :: proc(t: ^testing.T, scene: ^Scene, position: e.Global_Position, shape: e.Shape_Handle) -> e.Body_Handle
{
	handle: e.Body_Handle;
	status: e.Status;
	handle, status = e.body_add_global(&scene.world, body_description(shape), position);
	testing.expect_value(t, status, e.Status.Ok);
	return handle;
}

expect_body_position :: proc(t: ^testing.T, scene: ^Scene, handle: e.Body_Handle, expected: e.Global_Position)
{
	pose: e.Global_Pose;
	status: e.Status;
	pose, status = e.body_get_global_pose(&scene.world, handle);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, pose.position, expected);
	state: e.Body_State;
	state, status = e.body_get(&scene.world, handle);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, state.pose.position, e.Vector3{f32(expected.x), f32(expected.y), f32(expected.z)});
}

sleep_body :: proc(t: ^testing.T, scene: ^Scene, body: e.Body_Handle)
{
	handles: [1]e.Body_Handle = {body};
	testing.expect_value(t, e.bodies_sleep_group(&scene.world, handles[:]), e.Status.Ok);
	testing.expect(t, scene.simulation.bodies.handle_to_location.memory[body.value].set_index > 0);
}
