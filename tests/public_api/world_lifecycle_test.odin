package public_api_tests

import "core:testing"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

small_world_description :: proc() -> entasis.World_Description
{
	description := entasis.world_description_default();
	description.capacity = {
		bodies=4,
		statics=4,
		inactive_body_sets=2,
		shapes_per_type=2,
		constraints=16,
		initial_constraints_per_type_batch=4,
		minimum_constraints_per_body=4,
		broad_phase_candidates=16,
		pairs=16,
		collision_child_pairs=0,
		inactive_pairs=8,
		pending_pairs_per_worker=8,
	};
	description.solve = {
		velocity_iterations=4,
		substeps=1,
		fallback_batch_threshold=8,
	};
	return description;
}

lifecycle_body_description :: proc() -> entasis.Body_Description
{
	return {
		pose={orientation=util.quaternion_identity(), position={0, 1, 0}},
		local_inertia={inverse_inertia_tensor={xx=1, yy=1, zz=1}, inverse_mass=1},
		activity={sleep_threshold=-1, minimum_timestep_count_under_threshold=255},
	};
}

@(test)
world_defaults_map_to_qualified_low_level_defaults :: proc(t: ^testing.T)
{
	description := entasis.world_description_default();
	low_capacity := physics.simulation_allocation_sizes_default();
	low_solve := physics.solve_description_default();
	testing.expect_value(t, description.gravity, entasis.Vector3{0, -9.81, 0});
	testing.expect_value(t, description.damping, entasis.Damping{linear=0.01, angular=0.01});
	testing.expect_value(t, description.capacity.bodies, low_capacity.bodies);
	testing.expect_value(t, description.capacity.statics, low_capacity.statics);
	testing.expect_value(t, description.capacity.constraints, low_capacity.constraints);
	testing.expect_value(t, description.capacity.pairs, low_capacity.pairs);
	testing.expect_value(t, description.solve.velocity_iterations, low_solve.velocity_iteration_count);
	testing.expect_value(t, description.solve.substeps, low_solve.substep_count);
	testing.expect_value(t, description.solve.fallback_batch_threshold, low_solve.fallback_batch_threshold);
	testing.expect_value(t, description.threading.worker_count, i32(1));
	testing.expect_value(t, description.threading.external_dispatcher, (^entasis.Dispatcher)(nil));
	testing.expect(t, description.allocator.procedure != nil);
}

@(test)
owned_world_can_clear_grow_destroy_and_reinitialize :: proc(t: ^testing.T)
{
	world: entasis.World;
	for iteration in 0 ..< 2
	{
		description := small_world_description();
		description.gravity = {0, -10, 0};
		description.damping = {linear=0.02, angular=0.03};
		status := entasis.world_init(&world, description);
		if !testing.expect_value(t, status, entasis.Status.Ok)
		{
			return;
		}
		first_body, body_status := entasis.body_add(&world, lifecycle_body_description());
		testing.expect_value(t, body_status, entasis.Status.Ok);
		testing.expect(t, entasis.body_handle_is_valid(first_body));
		grown := description.capacity;
		grown.bodies = 32;
		grown.statics = 16;
		grown.constraints = 64;
		grown.pairs = 64;
		grown.broad_phase_candidates = 64;
		grown.collision_child_pairs = 0;
		testing.expect_value(t, entasis.world_ensure_capacity(&world, grown), entasis.Status.Ok);
		for _ in 0 ..< 20
		{
			_, add_status := entasis.body_add(&world, lifecycle_body_description());
			testing.expect_value(t, add_status, entasis.Status.Ok);
		}
		testing.expect_value(t, entasis.world_clear(&world), entasis.Status.Ok);
		after_clear, after_clear_status := entasis.body_add(&world, lifecycle_body_description());
		testing.expect_value(t, after_clear_status, entasis.Status.Ok);
		testing.expect_value(t, after_clear, first_body);
		testing.expect_value(t, entasis.world_step(&world, 1.0 / 60.0), entasis.Status.Ok);
		testing.expect_value(t, entasis.world_destroy(&world), entasis.Status.Ok);
		testing.expect_value(t, entasis.world_destroy(&world), entasis.Status.Disposed);
		_ = iteration;
	}
}

@(test)
external_pool_and_dispatcher_remain_caller_owned :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	if !testing.expect_value(
		t, util.buffer_pool_initialize(&pool, 4096, 4), util.Memory_Status.Ok,
	)
	{
		return;
	}
	dispatcher_owner: util.Thread_Dispatcher;
	if !testing.expect_value(
		t, util.thread_dispatcher_initialize(&dispatcher_owner, 2, 4096), util.Threading_Status.Ok,
	)
	{
		_ = util.buffer_pool_dispose(&pool);
		return;
	}
	world: entasis.World;
	description := small_world_description();
	description.threading.external_dispatcher = util.thread_dispatcher_boundary(&dispatcher_owner);
	description.threading.worker_count = 1;
	status := entasis.world_init_with_pool(&world, description, &pool);
	if !testing.expect_value(t, status, entasis.Status.Ok)
	{
		_ = util.thread_dispatcher_shutdown(&dispatcher_owner);
		_ = util.buffer_pool_dispose(&pool);
		return;
	}
	testing.expect_value(t, entasis.world_step(&world, 1.0 / 60.0), entasis.Status.Ok);
	testing.expect_value(t, entasis.world_destroy(&world), entasis.Status.Ok);
	// these succeed only if world_destroy left externally owned resources intact
	testing.expect_value(
		t, util.thread_dispatcher_shutdown(&dispatcher_owner), util.Threading_Status.Ok,
	);
	testing.expect_value(t, util.buffer_pool_dispose(&pool), util.Memory_Status.Ok);
}

@(test)
failed_initialization_does_not_poison_world :: proc(t: ^testing.T)
{
	world: entasis.World;
	uninitialized_pool: util.Buffer_Pool;
	description := small_world_description();
	testing.expect_value(
		t,
		entasis.world_init_with_pool(&world, description, &uninitialized_pool),
		entasis.Status.Invalid_Argument,
	);
	description.damping.linear = 2;
	testing.expect_value(
		t, entasis.world_init(&world, description), entasis.Status.Invalid_Description,
	);
	description = small_world_description();
	testing.expect_value(t, entasis.world_init(&world, description), entasis.Status.Ok);
	testing.expect_value(t, entasis.world_destroy(&world), entasis.Status.Ok);
}

@(test)
owned_dispatcher_is_used_by_default_step :: proc(t: ^testing.T)
{
	world: entasis.World;
	description := small_world_description();
	description.threading.worker_count = 2;
	description.threading.worker_pool_block_size = 4096;
	if !testing.expect_value(t, entasis.world_init(&world, description), entasis.Status.Ok)
	{
		return;
	}
	testing.expect_value(t, entasis.world_step(&world, 1.0 / 60.0), entasis.Status.Ok);
	testing.expect_value(t, entasis.world_destroy(&world), entasis.Status.Ok);
}

// regression: moving the grown maintenance workspace used to leave its task
// procedure table pointing into the constructor's expired stack storage
@(test)
world_grown_threaded_maintenance_rebinds_its_task_table :: proc(t: ^testing.T)
{
	for scope in ([2]entasis.Allocation_Scope{.Legacy, .All_Owned})
	{
		for workers in ([2]i32{2, 4})
		{
			world: entasis.World;
			description := entasis.world_description_default();
			description.threading.worker_count = workers;
			description.capacity.bodies = 128;
			description.capacity.constraints = 128;
			description.capacity.initial_constraints_per_type_batch = 128;
			if !testing.expect_value(t, entasis.world_init_with_allocation_scope(&world, description, scope), entasis.Status.Ok)
			{
				return;
			}
			defer testing.expect_value(t, entasis.world_destroy(&world), entasis.Status.Ok);
			value := entasis.box(1, 1, 1);
			shape, shape_status := entasis.shape_add(&world, value);
			inertia, inertia_status := entasis.shape_inertia(value, 1);
			testing.expect_value(t, shape_status, entasis.Status.Ok);
			testing.expect_value(t, inertia_status, entasis.Status.Ok);
			motor := entasis.One_Body_Linear_Motor{target_velocity={1, 0, 0}, settings={maximum_force=10, damping=1}};
			for i in 0..<2048
			{
				body, status := entasis.body_add(&world, entasis.body_dynamic(shape, inertia,
						entasis.pose({f32(i%32)*2, 2, f32(i/32)*2}), {}, entasis.body_activity(-1, 255)));
				if !testing.expect_value(t, status, entasis.Status.Ok)
				{
					return;
				}
				_, status = entasis.constraint_add(&world, []entasis.Body_Handle{body}, motor);
				if !testing.expect_value(t, status, entasis.Status.Ok)
				{
					return;
				}
			}
			if !testing.expect_value(t, entasis.world_step(&world, 1.0/60.0), entasis.Status.Ok)
			{
				return;
			}
			simulation, status := entasis.world_borrow_simulation(&world);
			testing.expect_value(t, status, entasis.Status.Ok);
			workspace := &simulation.broad_phase.maintenance_workspace;
			testing.expect(t, workspace.task_stack.procedures.memory == &workspace.task_procedures[0]);
			testing.expect_value(t, physics.broad_phase_resize(&simulation.broad_phase, 2048,
					int(description.capacity.statics), int(description.capacity.broad_phase_candidates)), physics.Physics_Status.Ok);
			testing.expect(t, workspace.task_stack.procedures.memory == &workspace.task_procedures[0]);
			testing.expect_value(t, entasis.world_step(&world, 1.0/60.0), entasis.Status.Ok);
		}
	}
}
