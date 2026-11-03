package simulation_tests

import "core:math"
import "core:simd"
import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Elision_Kinematic_Placement :: enum u8
{
	Absent,
	Synchronized,
	Fallback,
	Fallback_Full,
}

Elision_Callback_State :: struct
{
	visits: [20]i32,
	valid: physics.Reference_State,
}

elision_prepare :: proc "contextless" (user_context: rawptr, dt: f32) -> physics.Physics_Status
{
	_ = user_context;
	_ = dt;
	return .Ok;
}

elision_integrate :: proc "contextless" (
	user_context: rawptr, indices: util.I32x8, position: util.Vector3_Wide,
	orientation: util.Quaternion_Wide, inertia: physics.Body_Inertia_Wide,
	mask: util.I32x8, worker: int, dt: util.F32x8, velocity: ^physics.Body_Velocity_Wide,
)
{
	_ = position;
	_ = orientation;
	_ = inertia;
	_ = worker;
	_ = dt;
	_ = velocity;
	state: ^Elision_Callback_State = (^Elision_Callback_State)(user_context);
	for lane in 0 ..< 8
	{
		if simd.extract(mask, lane) != 0
		{
			index: i32 = simd.extract(indices, lane);
			if index < 0 || index >= len(state.visits)
			{
				state.valid = .Missing;
				continue;
			}
			state.visits[index] += 1;
		}
	}
}

@(test)
empty_kinematic_stages_preserve_live_and_fallback_body_integration :: proc(t: ^testing.T)
{
	placements: [4]Elision_Kinematic_Placement = {.Absent, .Synchronized, .Fallback, .Fallback_Full};
	body_counts: [2]int = {4, 20};
	worker_counts: [2]int = {1, 2};
	for worker_count in worker_counts
	{
		for body_count in body_counts
		{
			for placement in placements
			{
				if placement == .Fallback_Full && body_count == 4
				{
					continue;
				}
				kinematic_body: int = -1;
				switch placement
				{
				case .Absent:
				case .Synchronized: kinematic_body = 1;
				case .Fallback: kinematic_body = body_count-1;
				case .Fallback_Full: kinematic_body = 3;
				}
				pool: util.Buffer_Pool;
				prepare_pool(t, &pool);
				defer util.buffer_pool_dispose(&pool);
				world: ^physics.Simulation = allocate_simulation(t);
				if world == nil
				{
					return;
				}
				defer free_simulation(t, world);
				description: physics.Simulation_Create_Description = small_description(&pool);
				description.allocation_sizes.workers = i32(worker_count);
				description.solve_description = {velocity_iteration_count=1, substep_count=4, fallback_batch_threshold=2};
				testing.expect_value(t, physics.simulation_create(world, &description).status, physics.Physics_Status.Ok);
				defer physics.simulation_destroy(world);
				state: Elision_Callback_State = {valid=.Present};
				world.integrator.callbacks.prepare_for_integration = elision_prepare;
				world.integrator.callbacks.integrate_velocity = elision_integrate;
				world.integrator.callbacks.integrate_kinematic_velocity = .Enabled;
				world.integrator.callbacks.user_context = &state;
				handles: [20]physics.Body_Handle;
				for index in 0 ..< body_count
				{
					body: physics.Body_Description = dynamic_body();
					body.pose.position = {};
					body.velocity = {linear={1, 0, 0}};
					body.activity.sleep_threshold = -1;
					if index == kinematic_body
					{
						body.local_inertia = {};
					}
					status: physics.Physics_Status;
					handles[index], status = physics.simulation_add_body(world, &body);
					testing.expect_value(t, status, physics.Physics_Status.Ok);
				}
				for index in 1 ..< body_count
				{
					connected: [4]physics.Body_Handle = {handles[0], handles[index], {}, {}};
					joint: physics.Ball_Socket = {spring_settings={angular_frequency=10, twice_damping_ratio=2}};
					constraint: physics.Constraint_Handle;
					status: physics.Physics_Status;
					constraint, status = physics.simulation_add_constraint(world, &connected, &joint);
					_ = constraint;
					testing.expect_value(t, status, physics.Physics_Status.Ok);
				}
				testing.expect_value(t, world.solver.active_set.batch_count, i32(3));
				for batch_index in 0 ..< int(world.solver.active_set.batch_count)
				{
					batch: ^physics.Constraint_Batch = &world.solver.active_set.batches.memory[batch_index];
					for type_index in 0 ..< int(batch.type_batch_count)
					{
						type_batch: ^physics.Type_Batch = &batch.type_batches.memory[type_index];
						tail: int = int(type_batch.count)%8;
						if tail == 0
						{
							continue;
						}
						references: [^]util.I32x8 = physics.type_batch_body_bundle(type_batch, int(type_batch.count)-tail);
						for role in 0 ..< int(type_batch.body_count)
						{
							for lane in tail ..< 8
							{
								references[role] = simd.replace(references[role], lane, i32(physics.BODY_REFERENCE_KINEMATIC_MASK));
							}
						}
					}
				}
				dispatcher: util.Thread_Dispatcher;
				boundary: ^util.Thread_Dispatcher_Boundary;
				if worker_count > 1
				{
					if !testing.expect_value(t, util.thread_dispatcher_initialize(&dispatcher, worker_count), util.Threading_Status.Ok)
					{
						return;
					}
					boundary = util.thread_dispatcher_boundary(&dispatcher);
				}
				defer
				{
					if worker_count > 1
					{
						testing.expect_value(t, util.thread_dispatcher_shutdown(&dispatcher), util.Threading_Status.Ok);
					}
				}
				testing.expect_value(t, physics.simulation_timestep(world, 0.1, boundary), physics.Physics_Status.Ok);
				testing.expect_value(t, state.valid, physics.Reference_State.Present);
				for handle, index in handles[:body_count]
				{
					body: physics.Body_Description;
					status: physics.Physics_Status;
					body, status = physics.bodies_get_description(&world.bodies, handle);
					testing.expect_value(t, status, physics.Physics_Status.Ok);
					testing.expect(t, math.abs(body.pose.position.x-0.1) < 1e-6);
					testing.expect_value(t, body.pose.position.y, f32(0));
					testing.expect_value(t, body.velocity.linear, util.Vector3{1, 0, 0});
					testing.expect_value(t, state.visits[index], i32(5));
				}
			}
		}
	}
}
