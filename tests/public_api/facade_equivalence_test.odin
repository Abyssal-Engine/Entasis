package public_api_tests

import "core:mem"
import "core:testing"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Direct_World :: struct
{
	simulation: ^physics.Simulation,
}

equivalence_description :: proc() -> entasis.World_Description
{
	description := entasis.world_description_default();
	description.capacity = {
		bodies=8,
		statics=4,
		inactive_body_sets=2,
		shapes_per_type=2,
		constraints=16,
		initial_constraints_per_type_batch=4,
		minimum_constraints_per_body=4,
		broad_phase_candidates=16,
		pairs=16,
		collision_child_pairs=16,
		inactive_pairs=8,
		pending_pairs_per_worker=8,
	};
	return description;
}

prepare_facade_world :: proc(
	t: ^testing.T, world: ^entasis.World, pool: ^util.Buffer_Pool,
) -> bool
{
	if !testing.expect_value(
		t, util.buffer_pool_initialize(pool, 4096, 2), util.Memory_Status.Ok,
	)
	{
		return false;
	}
	return testing.expect_value(
		t,
		entasis.world_init_with_pool(world, equivalence_description(), pool),
		entasis.Status.Ok,
	);
}

release_facade_world :: proc(t: ^testing.T, world: ^entasis.World, pool: ^util.Buffer_Pool)
{
	status := entasis.world_destroy(world);
	testing.expect(t, status == .Ok || status == .Disposed);
	testing.expect_value(t, util.buffer_pool_dispose(pool), util.Memory_Status.Ok);
}

prepare_direct_world :: proc(
	t: ^testing.T, world: ^Direct_World, pool: ^util.Buffer_Pool,
) -> bool
{
	memory, allocation_error := mem.alloc(
		size_of(physics.Simulation), align_of(physics.Simulation),
	);
	if !testing.expect(t, allocation_error == nil && memory != nil)
	{
		return false;
	}
	world.simulation = (^physics.Simulation)(memory);
	world.simulation^ = {};
	if !testing.expect_value(
		t, util.buffer_pool_initialize(pool, 4096, 2), util.Memory_Status.Ok,
	)
	{
		return false;
	}
	public_description := equivalence_description();
	description := physics.simulation_create_description_default(pool);
	description.allocation_sizes = {
		bodies=public_description.capacity.bodies,
		statics=public_description.capacity.statics,
		inactive_body_sets=public_description.capacity.inactive_body_sets,
		shapes_per_type=public_description.capacity.shapes_per_type,
		constraints=public_description.capacity.constraints,
		constraint_batches=public_description.solve.fallback_batch_threshold + 1,
		initial_constraints_per_type_batch=public_description.capacity.initial_constraints_per_type_batch,
		minimum_constraints_per_body=public_description.capacity.minimum_constraints_per_body,
		broad_phase_candidates=public_description.capacity.broad_phase_candidates,
		pairs=public_description.capacity.pairs,
		collision_child_pairs=public_description.capacity.collision_child_pairs,
		inactive_pairs=public_description.capacity.inactive_pairs,
		pending_pairs_per_worker=public_description.capacity.pending_pairs_per_worker,
		workers=1,
	};
	result := physics.simulation_create(world.simulation, &description);
	return testing.expect_value(t, result.status, physics.Physics_Status.Ok);
}

release_direct_world :: proc(t: ^testing.T, world: ^Direct_World, pool: ^util.Buffer_Pool)
{
	if world.simulation != nil
	{
		if world.simulation.state == .Ready
		{
			testing.expect_value(
				t, physics.simulation_destroy(world.simulation), physics.Physics_Status.Ok,
			);
		}
		testing.expect_value(t, mem.free(world.simulation), mem.Allocator_Error.None);
		world.simulation = nil;
	}
	testing.expect_value(t, util.buffer_pool_dispose(pool), util.Memory_Status.Ok);
}

body_description :: proc() -> entasis.Body_Description
{
	return {
		pose={orientation=util.quaternion_identity(), position={0, 1, 0}},
		local_inertia={inverse_inertia_tensor={xx=1, yy=1, zz=1}, inverse_mass=1},
		activity={sleep_threshold=-1, minimum_timestep_count_under_threshold=255},
	};
}

@(test)
facade_add_step_and_query_match_low_level_paths :: proc(t: ^testing.T)
{
	facade_world: entasis.World;
	direct_world: Direct_World;
	facade_pool, direct_pool: util.Buffer_Pool;
	if !prepare_facade_world(t, &facade_world, &facade_pool)
	{
		return;
	}
	defer release_facade_world(t, &facade_world, &facade_pool);
	if !prepare_direct_world(t, &direct_world, &direct_pool)
	{
		return;
	}
	defer release_direct_world(t, &direct_world, &direct_pool);

	description := body_description();
	facade_handle, facade_status := entasis.body_add(&facade_world, description);
	direct_handle, direct_status := physics.simulation_add_body(
		direct_world.simulation, &description,
	);
	testing.expect_value(t, facade_status, direct_status);
	testing.expect_value(t, facade_handle, direct_handle);

	facade_step := entasis.world_step(&facade_world, 1.0 / 60.0);
	direct_step := physics.simulation_timestep(direct_world.simulation, 1.0 / 60.0);
	testing.expect_value(t, facade_step, direct_step);

	facade_hits, direct_hits: [1]entasis.Ray_Query_Hit;
	facade_collector, direct_collector: entasis.Ray_Query_Collector;
	facade_storage := util.Buffer(entasis.Ray_Query_Hit){memory=&facade_hits[0], length=1, id=-1};
	direct_storage := util.Buffer(physics.Ray_Query_Hit){memory=&direct_hits[0], length=1, id=-1};
	testing.expect_value(
		t, physics.ray_query_collector_initialize(&facade_collector, facade_storage, .Earliest),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(
		t, physics.ray_query_collector_initialize(&direct_collector, direct_storage, .Earliest),
		physics.Physics_Status.Ok,
	);
	ray := entasis.Ray{origin={0, 10, 0}, direction={0, -1, 0}, maximum_t=100};
	facade_query := entasis.ray_query(&facade_world, ray, &facade_collector, &facade_pool);
	direct_query := physics.simulation_ray_query(
		direct_world.simulation, ray, &direct_collector, &direct_pool,
	);
	testing.expect_value(t, facade_query, direct_query);
	testing.expect_value(t, facade_collector.count, direct_collector.count);
	if facade_collector.count > 0
	{
		testing.expect_value(t, facade_hits[0], direct_hits[0]);
	}
}
