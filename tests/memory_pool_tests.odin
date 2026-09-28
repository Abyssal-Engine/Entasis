package tests

import "core:testing"
import "core:math"
import "core:mem"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

@(test)
buffer_pool_grows_without_manual_preallocation :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	status := util.buffer_pool_initialize(&pool, 1024, 2);
	if !testing.expect_value(t, status, util.Memory_Status.Ok)
	{
		return;
	}
	defer util.buffer_pool_dispose(&pool);

	buffers: [80]util.Buffer(u8);
	for index in 0 ..< len(buffers)
	{
		buffer, take_status := util.buffer_pool_take(&pool, u8, 64);
		if !testing.expect_value(t, take_status, util.Memory_Status.Ok)
		{
			return;
		}
		buffers[index] = buffer;
		buffers[index].memory[0] = u8(index);
	}

	power := int(buffers[0].id) >> util.BUFFER_POOL_ID_POWER_SHIFT;
	power_pool := &pool.pools[power];
	testing.expect_value(t, power_pool.block_count, 5);
	testing.expect(t, power_pool.block_capacity >= power_pool.block_count);
	for index in 0 ..< len(buffers)
	{
		testing.expect_value(t, buffers[index].memory[0], u8(index));
		testing.expect_value(t, util.buffer_pool_return(&pool, &buffers[index]), util.Memory_Status.Ok);
	}
	testing.expect_value(t, power_pool.free_count, len(buffers));
	testing.expect(t, power_pool.slot_capacity >= len(buffers));

	for _ in 0 ..< len(buffers)
	{
		buffer, take_status := util.buffer_pool_take(&pool, u8, 64);
		if !testing.expect_value(t, take_status, util.Memory_Status.Ok)
		{
			return;
		}
		testing.expect_value(t, util.buffer_pool_return(&pool, &buffer), util.Memory_Status.Ok);
	}
}

@(test)
buffer_pool_clear_retains_metadata_and_remains_usable :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	status := util.buffer_pool_initialize(&pool, 1024, 2);
	if !testing.expect_value(t, status, util.Memory_Status.Ok)
	{
		return;
	}
	defer util.buffer_pool_dispose(&pool);

	buffers: [40]util.Buffer(u8);
	for index in 0 ..< len(buffers)
	{
		buffer, take_status := util.buffer_pool_take(&pool, u8, 64);
		if !testing.expect_value(t, take_status, util.Memory_Status.Ok)
		{
			return;
		}
		buffers[index] = buffer;
	}
	power := int(buffers[0].id) >> util.BUFFER_POOL_ID_POWER_SHIFT;
	block_capacity := pool.pools[power].block_capacity;
	slot_capacity := pool.pools[power].slot_capacity;
	testing.expect(t, pool.pools[power].block_count > 0);

	testing.expect_value(t, util.buffer_pool_clear(&pool), util.Memory_Status.Ok);
	testing.expect_value(t, pool.pools[power].block_count, 0);
	testing.expect_value(t, pool.pools[power].next_slot, 0);
	testing.expect_value(t, pool.pools[power].free_count, 0);
	testing.expect_value(t, pool.pools[power].block_capacity, block_capacity);
	testing.expect_value(t, pool.pools[power].slot_capacity, slot_capacity);

	buffer, take_status := util.buffer_pool_take(&pool, u8, 64);
	if !testing.expect_value(t, take_status, util.Memory_Status.Ok)
	{
		return;
	}
	buffer.memory[0] = 77;
	testing.expect_value(t, buffer.memory[0], u8(77));
	testing.expect_value(t, util.buffer_pool_return(&pool, &buffer), util.Memory_Status.Ok);
}

@(test)
buffer_pool_resize_preserves_data :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	status := util.buffer_pool_initialize(&pool, 1024);
	if !testing.expect_value(t, status, util.Memory_Status.Ok)
	{
		return;
	}
	defer util.buffer_pool_dispose(&pool);

	buffer, take_status := util.buffer_pool_take(&pool, i32, 3);
	if !testing.expect_value(t, take_status, util.Memory_Status.Ok)
	{
		return;
	}
	defer if buffer.memory != nil
	{
		util.buffer_pool_return(&pool, &buffer);
	}
	buffer.memory[0] = 11;
	buffer.memory[1] = 22;
	buffer.memory[2] = 33;

	testing.expect_value(
		t, util.buffer_pool_resize_to_at_least(&pool, &buffer, 100, 3), util.Memory_Status.Ok,
	);
	testing.expect(t, int(buffer.length) >= 100);
	testing.expect_value(t, buffer.memory[0], i32(11));
	testing.expect_value(t, buffer.memory[1], i32(22));
	testing.expect_value(t, buffer.memory[2], i32(33));
}

@(test)
buffer_pool_optional_prewarm_remains_supported :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	status := util.buffer_pool_initialize(&pool, 1024);
	if !testing.expect_value(t, status, util.Memory_Status.Ok)
	{
		return;
	}
	defer util.buffer_pool_dispose(&pool);

	power, power_status := util.buffer_pool_power_for_count(u8, 64);
	if !testing.expect_value(t, power_status, util.Memory_Status.Ok)
	{
		return;
	}
	testing.expect_value(
		t, util.buffer_pool_ensure_capacity_for_power(&pool, 100 * 64, power), util.Memory_Status.Ok,
	);
	block_count := pool.pools[power].block_count;
	available, available_status := util.buffer_pool_available_slot_count(&pool, power);
	testing.expect_value(t, available_status, util.Memory_Status.Ok);
	testing.expect(t, available >= 100);

	buffers: [100]util.Buffer(u8);
	for index in 0 ..< len(buffers)
	{
		buffer, take_status := util.buffer_pool_take(&pool, u8, 64);
		if !testing.expect_value(t, take_status, util.Memory_Status.Ok)
		{
			return;
		}
		buffers[index] = buffer;
	}
	testing.expect_value(t, pool.pools[power].block_count, block_count);
	for index in 0 ..< len(buffers)
	{
		testing.expect_value(t, util.buffer_pool_return(&pool, &buffers[index]), util.Memory_Status.Ok);
	}
}

@(test)
id_pool_return_grows_automatically :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	status := util.buffer_pool_initialize(&pool, 1024);
	if !testing.expect_value(t, status, util.Memory_Status.Ok)
	{
		return;
	}
	defer util.buffer_pool_dispose(&pool);

	ids: util.Id_Pool;
	if !testing.expect_value(t, util.id_pool_initialize(&ids, 1, &pool), util.Memory_Status.Ok)
	{
		return;
	}
	defer util.id_pool_dispose(&ids, &pool);

	claimed: [64]i32;
	for index in 0 ..< len(claimed)
	{
		id, take_status := util.id_pool_take(&ids);
		if !testing.expect_value(t, take_status, util.Memory_Status.Ok)
		{
			return;
		}
		claimed[index] = id;
		testing.expect_value(t, id, i32(index));
	}
	for id in claimed
	{
		if !testing.expect_value(t, util.id_pool_return(&ids, id, &pool), util.Memory_Status.Ok)
		{
			return;
		}
	}
	testing.expect_value(t, ids.available_id_count, len(claimed));
	testing.expect(t, int(ids.available_ids.length) >= len(claimed));

	seen: [64]bool;
	for _ in 0 ..< len(claimed)
	{
		id, take_status := util.id_pool_take(&ids);
		if !testing.expect_value(t, take_status, util.Memory_Status.Ok)
		{
			return;
		}
		testing.expect(t, id >= 0 && id < i32(len(seen)));
		if id >= 0 && id < i32(len(seen))
		{
			testing.expect(t, !seen[id]);
			seen[id] = true;
		}
	}
	for present in seen
	{
		testing.expect(t, present);
	}
}

@(test)
quick_list_grows_through_buffer_pool :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	status := util.buffer_pool_initialize(&pool, 1024);
	if !testing.expect_value(t, status, util.Memory_Status.Ok)
	{
		return;
	}
	defer util.buffer_pool_dispose(&pool);

	list: util.Quick_List(i32);
	if !testing.expect_value(
		t, util.quick_list_initialize(&list, 1, &pool), util.Collection_Status.Ok,
	)
	{
		return;
	}
	defer util.quick_list_dispose(&list, &pool);

	for index in 0 ..< 1000
	{
		if !testing.expect_value(
			t, util.quick_list_add(&list, i32(index * 3), &pool), util.Collection_Status.Ok,
		)
		{
			return;
		}
	}
	testing.expect_value(t, list.count, 1000);
	testing.expect(t, int(list.span.length) >= 1000);
	for index in 0 ..< list.count
	{
		testing.expect_value(t, list.span.memory[index], i32(index * 3));
	}
}

@(test)
worker_buffer_pools_clear_and_reuse :: proc(t: ^testing.T)
{
	workers: util.Worker_Buffer_Pools;
	status := util.worker_buffer_pools_initialize(&workers, 3, 1024);
	if !testing.expect_value(t, status, util.Memory_Status.Ok)
	{
		return;
	}
	defer util.worker_buffer_pools_dispose(&workers);

	for worker_index in 0 ..< workers.worker_count
	{
		pool, get_status := util.worker_buffer_pool_get(&workers, worker_index);
		if !testing.expect_value(t, get_status, util.Memory_Status.Ok)
		{
			return;
		}
		for allocation_index in 0 ..< 32
		{
			buffer, take_status := util.buffer_pool_take(pool, u8, 64);
			if !testing.expect_value(t, take_status, util.Memory_Status.Ok)
			{
				return;
			}
			buffer.memory[0] = u8(worker_index + allocation_index);
		}
	}
	testing.expect(t, util.worker_buffer_pools_total_allocated_byte_count(&workers) > 0);
	testing.expect_value(t, util.worker_buffer_pools_clear(&workers), util.Memory_Status.Ok);
	testing.expect_value(t, util.worker_buffer_pools_total_allocated_byte_count(&workers), u64(0));

	for worker_index in 0 ..< workers.worker_count
	{
		pool, get_status := util.worker_buffer_pool_get(&workers, worker_index);
		if !testing.expect_value(t, get_status, util.Memory_Status.Ok)
		{
			return;
		}
		buffer, take_status := util.buffer_pool_take(pool, u8, 64);
		if !testing.expect_value(t, take_status, util.Memory_Status.Ok)
		{
			return;
		}
		testing.expect_value(t, util.buffer_pool_return(pool, &buffer), util.Memory_Status.Ok);
	}
}

@(test)
simulation_grows_from_small_initial_capacities :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	if !testing.expect_value(
		t, util.buffer_pool_initialize(&pool, 4096, 2), util.Memory_Status.Ok,
	)
	{
		return;
	}
	defer util.buffer_pool_dispose(&pool);

	description := physics.simulation_create_description_default(&pool);
	description.allocation_sizes = {
		bodies=2,
		statics=2,
		inactive_body_sets=1,
		shapes_per_type=1,
		constraints=8,
		constraint_batches=2,
		initial_constraints_per_type_batch=1,
		minimum_constraints_per_body=1,
		broad_phase_candidates=8,
		pairs=8,
		collision_child_pairs=16,
		inactive_pairs=1,
		pending_pairs_per_worker=8,
		workers=1,
	};

	simulation_memory, allocation_error := mem.alloc(
		size_of(physics.Simulation), align_of(physics.Simulation),
	);
	if !testing.expect(t, allocation_error == nil && simulation_memory != nil)
	{
		return;
	}
	defer mem.free(simulation_memory);
	simulation := (^physics.Simulation)(simulation_memory);
	simulation^ = {};
	create_result := physics.simulation_create(simulation, &description);
	if !testing.expect_value(t, create_result.status, physics.Physics_Status.Ok)
	{
		return;
	}
	defer physics.simulation_destroy(simulation);

	shapes: [64]physics.Typed_Index;
	for index in 0 ..< len(shapes)
	{
		box := physics.Box{
			0.4 + f32(index % 3) * 0.01,
			0.4 + f32(index % 5) * 0.01,
			0.4 + f32(index % 7) * 0.01,
		};
		shape, shape_status := physics.shape_registry_add(
			&simulation.shapes, physics.BOX_TYPE_ID, &box,
		);
		if !testing.expect_value(t, shape_status, physics.Physics_Status.Ok)
		{
			return;
		}
		shapes[index] = shape;
	}
	for index in 0 ..< len(shapes)
	{
		shape, _, resolve_status := physics.shape_registry_resolve(
			&simulation.shapes, shapes[index],
		);
		if !testing.expect_value(t, resolve_status, physics.Physics_Status.Ok)
		{
			return;
		}
		box := (^physics.Box)(shape)^;
		testing.expect_value(t, box.half_width, 0.4 + f32(index % 3) * 0.01);
		testing.expect_value(t, box.half_height, 0.4 + f32(index % 5) * 0.01);
		testing.expect_value(t, box.half_length, 0.4 + f32(index % 7) * 0.01);
	}

	inertia, inertia_status := physics.box_inertia(physics.Box{0.4, 0.4, 0.4}, 1);
	if !testing.expect_value(t, inertia_status, physics.Physics_Status.Ok)
	{
		return;
	}
	body_description := physics.Body_Description{
		pose={orientation=util.quaternion_identity()},
		local_inertia=inertia,
		collidable={
			continuity=physics.continuous_detection_passive(),
			maximum_speculative_margin=f32(math.F32_MAX),
		},
		activity={sleep_threshold=-1, minimum_timestep_count_under_threshold=32},
	};
	body_handles: [128]physics.Body_Handle;
	for index in 0 ..< len(body_handles)
	{
		body_description.pose.position = {f32(index) * 4, 100, 0};
		body_description.collidable.shape = shapes[index % len(shapes)];
		handle, body_status := physics.simulation_add_body(simulation, &body_description);
		if !testing.expectf(t, body_status == .Ok, "body %d failed with %v", index, body_status)
		{
			return;
		}
		body_handles[index] = handle;
	}
	for index in 0 ..< len(body_handles)
	{
		description, body_status := physics.bodies_get_description(
			&simulation.bodies, body_handles[index],
		);
		if !testing.expect_value(t, body_status, physics.Physics_Status.Ok)
		{
			return;
		}
		testing.expect_value(t, description.pose.position.x, f32(index) * 4);
		testing.expect_value(t, description.pose.position.y, f32(100));
		testing.expect_value(t, description.collidable.shape.packed, shapes[index % len(shapes)].packed);
	}

	static_handles: [64]physics.Static_Handle;
	for index in 0 ..< len(static_handles)
	{
		static_description := physics.Static_Description{
			pose={
				orientation=util.quaternion_identity(),
				position={f32(index) * 4, -100, 0},
			},
			shape=shapes[index],
		};
		handle, static_status := physics.simulation_add_static(simulation, &static_description);
		if !testing.expect_value(t, static_status, physics.Physics_Status.Ok)
		{
			return;
		}
		static_handles[index] = handle;
	}
	for index in 0 ..< len(static_handles)
	{
		description, static_status := physics.statics_get_description(
			&simulation.statics, static_handles[index],
		);
		if !testing.expect_value(t, static_status, physics.Physics_Status.Ok)
		{
			return;
		}
		testing.expect_value(t, description.pose.position.x, f32(index) * 4);
		testing.expect_value(t, description.pose.position.y, f32(-100));
		testing.expect_value(t, description.shape.packed, shapes[index].packed);
	}

	testing.expect_value(
		t,
		physics.simulation_timestep(simulation, f32(1.0 / 60.0)),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(
		t,
		simulation.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX].count,
		len(body_handles),
	);
	testing.expect_value(t, simulation.statics.count, len(static_handles));
	testing.expect(t, int(simulation.bodies.handle_to_location.length) >= len(body_handles));
	testing.expect(t, int(simulation.statics.handle_to_index.length) >= len(static_handles));

	for handle in body_handles
	{
		if !testing.expect_value(
			t, physics.simulation_remove_body(simulation, handle), physics.Physics_Status.Ok,
		)
		{
			return;
		}
	}
	for handle in static_handles
	{
		if !testing.expect_value(
			t, physics.simulation_remove_static(simulation, handle), physics.Physics_Status.Ok,
		)
		{
			return;
		}
	}
	testing.expect_value(
		t,
		simulation.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX].count,
		0,
	);
	testing.expect_value(t, simulation.statics.count, 0);
	testing.expect(t, simulation.bodies.handle_pool.available_id_count >= len(body_handles));
	testing.expect(t, simulation.statics.handle_pool.available_id_count >= len(static_handles));

	box_batch := &simulation.shapes.batches[physics.BOX_TYPE_ID];
	for shape in shapes
	{
		if !testing.expect_value(
			t, physics.shape_registry_remove(&simulation.shapes, shape), physics.Physics_Status.Ok,
		)
		{
			return;
		}
	}
	testing.expect_value(t, box_batch.active_count, 0);
	testing.expect(t, box_batch.ids.available_id_count >= len(shapes));
	for index in 0 ..< len(shapes)
	{
		box := physics.Box{
			0.5 + f32(index % 3) * 0.01,
			0.5 + f32(index % 5) * 0.01,
			0.5 + f32(index % 7) * 0.01,
		};
		shape, shape_status := physics.shape_registry_add(
			&simulation.shapes, physics.BOX_TYPE_ID, &box,
		);
		if !testing.expect_value(t, shape_status, physics.Physics_Status.Ok)
		{
			return;
		}
		shapes[index] = shape;
	}
	testing.expect_value(t, box_batch.active_count, len(shapes));
}
