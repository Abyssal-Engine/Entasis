package precision_snapshots_tests

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
	allocation: mem.Allocator = {}, block_size: int = 4096)
{
	mem.tracking_allocator_init(&scene.tracker, os.heap_allocator(), os.heap_allocator());
	backing: mem.Allocator = allocation;
	if backing.procedure == nil
	{
		backing = mem.tracking_allocator(&scene.tracker);
	}
	testing.expect_value(t, util.buffer_pool_initialize_with_allocator(
		&scene.pool, backing, block_size,
	), util.Memory_Status.Ok);
	description: e.World_Description = e.world_description_default();
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

add_static :: proc(t: ^testing.T, scene: ^Scene, position: e.Global_Position) -> e.Static_Handle
{
	handle: e.Static_Handle;
	status: e.Status;
	handle, status = e.static_add_global(&scene.world, e.static_body(scene.sphere, e.pose()), position, .None);
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

expect_static_position :: proc(t: ^testing.T, scene: ^Scene, handle: e.Static_Handle, expected: e.Global_Position)
{
	pose: e.Global_Pose;
	status: e.Status;
	pose, status = e.static_get_global_pose(&scene.world, handle);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, pose.position, expected);
	state: e.Static_State;
	state, status = e.static_get(&scene.world, handle);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, state.pose.position, e.Vector3{f32(expected.x), f32(expected.y), f32(expected.z)});
}

sleep_body :: proc(t: ^testing.T, scene: ^Scene, body: e.Body_Handle)
{
	index: i32 = scene.simulation.bodies.handle_to_location.memory[body.value].index;
	scene.simulation.bodies.sets.memory[0].activity.memory[index].sleep_candidate = .Candidate;
	sleeper: ^p.Island_Sleeper = &scene.simulation.sleeper;
	testing.expect_value(t, p.island_scaffold_clear(&sleeper.scaffold), e.Status.Ok);
	body_count, constraint_count: int;
	can_sleep: p.Reference_State;
	status: e.Status;
	body_count, constraint_count, can_sleep, status = p.island_sleeper_build_island(sleeper, &sleeper.scaffold, body);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, can_sleep, p.Reference_State.Present);
	testing.expect_value(t, p.island_sleeper_sleep_island(sleeper, &sleeper.scaffold, body_count, constraint_count), e.Status.Ok);
	testing.expect(t, scene.simulation.bodies.handle_to_location.memory[body.value].set_index > 0);
}

add_kinematic :: proc(t: ^testing.T, scene: ^Scene, position: e.Global_Position) -> e.Body_Handle
{
	handle: e.Body_Handle;
	status: e.Status;
	handle, status = e.body_add_global(&scene.world,
		e.body_kinematic(e.Shape_Handle{}, e.pose(), {}, e.body_activity(-1, 255)), position);
	testing.expect_value(t, status, e.Status.Ok);
	return handle;
}

add_servo :: proc(t: ^testing.T, scene: ^Scene, body: e.Body_Handle, target: e.Global_Position) -> e.Constraint_Handle
{
	description: e.One_Body_Linear_Servo = {
		target={f32(target.x), f32(target.y), f32(target.z)},
		spring_settings=e.spring_settings(10, 1), servo_settings=e.servo_settings(100, 0, 1e6),
	};
	handle: e.Constraint_Handle;
	status: e.Status;
	handle, status = e.constraint_add_1(&scene.world, body, description);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, p.simulation_set_constraint_global_target(scene.simulation, handle, target), e.Status.Ok);
	return handle;
}

add_lock :: proc(t: ^testing.T, scene: ^Scene, body: e.Body_Handle, target: e.Global_Position) -> e.Constraint_Handle
{
	value: e.Body_Axis_Lock = e.body_axis_lock_default(e.pose());
	value.linear_axes = {.X, .Y, .Z};
	testing.expect_value(t, e.body_set_axis_lock(&scene.world, body, value), e.Status.Ok);
	handle: e.Constraint_Handle;
	status: e.Status;
	handle, status = p.body_axis_lock_find(scene.simulation.body_control, body);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, p.simulation_set_constraint_global_target(scene.simulation, handle, target), e.Status.Ok);
	return handle;
}

expect_position_bits :: proc(t: ^testing.T, actual, expected: e.Global_Position)
{
	testing.expect_value(t, transmute([3]u64)actual, transmute([3]u64)expected);
}
