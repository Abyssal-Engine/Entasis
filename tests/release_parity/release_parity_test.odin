package release_parity_tests

import "core:mem"
import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

prepare_pool :: proc(t: ^testing.T, pool: ^util.Buffer_Pool) -> bool
{
	if !testing.expect_value(
		t, util.buffer_pool_initialize(pool, 4096, 2), util.Memory_Status.Ok,
	)
	{
		return false;
	}
	for power in 0 ..= 22
	{
		if !testing.expect_value(
			t, util.buffer_pool_ensure_capacity_for_power(pool, 1 << 20, power),
			util.Memory_Status.Ok,
		)
		{
			return false;
		}
	}
	return true;
}

allocate_simulation :: proc(t: ^testing.T) -> ^physics.Simulation
{
	memory, allocation_error := mem.alloc(
		size_of(physics.Simulation), align_of(physics.Simulation),
	);
	if !testing.expect(t, allocation_error == nil && memory != nil)
	{
		return nil;
	}
	simulation := (^physics.Simulation)(memory);
	simulation^ = {};
	return simulation;
}

release_simulation :: proc(t: ^testing.T, simulation: ^physics.Simulation)
{
	if simulation == nil
	{
		return;
	}
	if simulation.state == .Ready
	{
		testing.expect_value(
			t, physics.simulation_destroy(simulation), physics.Physics_Status.Ok,
		);
	}
	testing.expect_value(t, mem.free(simulation), mem.Allocator_Error.None);
}

small_default_description :: proc(
	pool: ^util.Buffer_Pool,
) -> physics.Simulation_Create_Description
{
	description := physics.simulation_create_description_default(pool);
	description.allocation_sizes = {
		bodies=32,
		statics=16,
		inactive_body_sets=4,
		shapes_per_type=4,
		constraints=64,
		constraint_batches=1,
		initial_constraints_per_type_batch=8,
		minimum_constraints_per_body=8,
		broad_phase_candidates=64,
		pairs=64,
		collision_child_pairs=64,
		inactive_pairs=32,
		pending_pairs_per_worker=32,
		workers=1,
	};
	return description;
}

dynamic_body :: proc(position: util.Vector3 = {}) -> physics.Body_Description
{
	return {
		pose={position=position, orientation=util.quaternion_identity()},
		local_inertia={
			inverse_inertia_tensor={xx=1, yy=1, zz=1},
			inverse_mass=1,
		},
		activity={sleep_threshold=-1, minimum_timestep_count_under_threshold=255},
	};
}

@(test)
default_fallback_threshold_is_not_clamped_by_allocation_hint :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	if !prepare_pool(t, &pool)
	{
		return;
	}
	defer util.buffer_pool_dispose(&pool);
	simulation := allocate_simulation(t);
	if simulation == nil
	{
		return;
	}
	defer release_simulation(t, simulation);
	description := small_default_description(&pool);
	if !testing.expect_value(
		t, physics.simulation_create(simulation, &description).status,
		physics.Physics_Status.Ok,
	)
	{
		return;
	}

	testing.expect_value(
		t, simulation.solver.fallback_batch_threshold,
		i32(physics.DEFAULT_FALLBACK_BATCH_THRESHOLD),
	);
	testing.expect_value(
		t, simulation.solver.fallback_batch_index,
		i32(physics.DEFAULT_FALLBACK_BATCH_THRESHOLD),
	);
	testing.expect_value(
		t, simulation.allocation_sizes.constraint_batches,
		i32(physics.DEFAULT_FALLBACK_BATCH_THRESHOLD + 1),
	);
	testing.expect(
		t, int(simulation.solver.active_set.batches.length) >=
		physics.DEFAULT_FALLBACK_BATCH_THRESHOLD + 1,
	);

	center_description := dynamic_body();
	center, center_status := physics.simulation_add_body(
		simulation, &center_description,
	);
	if !testing.expect_value(t, center_status, physics.Physics_Status.Ok)
	{
		return;
	}
	constraint_description := physics.Ball_Socket{
		spring_settings=physics.spring_settings_create(30, 1),
	};
	for constraint_index in 0 ..< 9
	{
		partner_description := dynamic_body({f32(constraint_index + 1), 0, 0});
		partner, partner_status := physics.simulation_add_body(
			simulation, &partner_description,
		);
		if !testing.expect_value(t, partner_status, physics.Physics_Status.Ok)
		{
			return;
		}
		handles := [4]physics.Body_Handle{center, partner, {}, {}};
		constraint, constraint_status := physics.simulation_add_constraint(
			simulation, &handles, &constraint_description,
		);
		if !testing.expect_value(
			t, constraint_status, physics.Physics_Status.Ok,
		)
		{
			return;
		}
		location := simulation.solver.handle_to_constraint.memory[constraint.value];
		testing.expect_value(t, location.batch_index, i32(constraint_index));
	}
	testing.expect_value(
		t, simulation.solver.sequential_batch.constraint_count, i32(0),
	);
}

@(test)
default_simulation_grows_for_later_multithreaded_dispatcher :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	if !prepare_pool(t, &pool)
	{
		return;
	}
	defer util.buffer_pool_dispose(&pool);
	simulation := allocate_simulation(t);
	if simulation == nil
	{
		return;
	}
	defer release_simulation(t, simulation);
	description := small_default_description(&pool);
	if !testing.expect_value(
		t, physics.simulation_create(simulation, &description).status,
		physics.Physics_Status.Ok,
	)
	{
		return;
	}

	dispatcher: util.Thread_Dispatcher;
	if !testing.expect_value(
		t, util.thread_dispatcher_initialize(&dispatcher, 4, 65536),
		util.Threading_Status.Ok,
	)
	{
		return;
	}
	defer util.thread_dispatcher_shutdown(&dispatcher);
	boundary := util.thread_dispatcher_boundary(&dispatcher);
	for _ in 0 ..< 2
	{
		if !testing.expect_value(
			t, physics.simulation_timestep(simulation, 1.0 / 60.0, boundary),
			physics.Physics_Status.Ok,
		)
		{
			return;
		}
	}
	testing.expect_value(t, simulation.allocation_sizes.workers, i32(4));
	testing.expect(t, simulation.broad_phase.query_workspace.worker_count >= 4);
	testing.expect(t, simulation.broad_phase.maintenance_workspace.worker_count >= 4);
	testing.expect(t, simulation.narrow_phase.active_worker_count >= 4);
	testing.expect(t, simulation.sleeper.worker_count >= 4);
}

@(test)
ensure_and_resize_use_logical_allocation_sizes :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	if !prepare_pool(t, &pool)
	{
		return;
	}
	defer util.buffer_pool_dispose(&pool);
	simulation := allocate_simulation(t);
	if simulation == nil
	{
		return;
	}
	defer release_simulation(t, simulation);
	description := small_default_description(&pool);
	if !testing.expect_value(
		t, physics.simulation_create(simulation, &description).status,
		physics.Physics_Status.Ok,
	)
	{
		return;
	}

	own_sizes := simulation.allocation_sizes;
	testing.expect_value(
		t, physics.simulation_ensure_capacity(simulation, own_sizes),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(
		t, physics.simulation_resize(simulation, own_sizes),
		physics.Physics_Status.Ok,
	);

	legacy_hint := own_sizes;
	legacy_hint.constraint_batches = 1;
	testing.expect_value(
		t, physics.simulation_ensure_capacity(simulation, legacy_hint),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(
		t, physics.simulation_resize(simulation, legacy_hint),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(
		t, simulation.solver.fallback_batch_index,
		i32(physics.DEFAULT_FALLBACK_BATCH_THRESHOLD),
	);
	testing.expect_value(
		t, simulation.allocation_sizes.constraint_batches,
		i32(physics.DEFAULT_FALLBACK_BATCH_THRESHOLD + 1),
	);
}
