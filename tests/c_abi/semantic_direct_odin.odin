package main

import "core:fmt"
import entasis "entasis:entasis"

emit_world :: proc(worker_count: i32)
{
	world: entasis.World;
	description := entasis.world_description_default();
	description.threading.worker_count = worker_count;
	if entasis.world_init(&world, description) != .Ok
	{
		panic("world_init");
	}
	for _ in 0 ..< 8
	{
		if entasis.world_step(&world, 1.0 / 120.0) != .Ok
		{
			panic("world_step");
		}
	}
	stats, status := entasis.world_stats(&world);
	if status != .Ok
	{
		panic("world_stats");
	}
	fmt.printf(
		"workers=%d step=%d active=%d sleeping=%d islands=%d statics=%d active_constraints=%d sleeping_constraints=%d active_pairs=%d inactive_pairs=%d shapes=%d shape_types=%d\n",
		worker_count,
		stats.step_index,
		stats.active_bodies,
		stats.sleeping_bodies,
		stats.sleeping_islands,
		stats.statics,
		stats.active_constraints,
		stats.sleeping_constraints,
		stats.active_pairs,
		stats.inactive_pairs,
		stats.registered_shapes,
		stats.registered_shape_types,
	);
	if entasis.world_clear(&world) != .Ok
	{
		panic("world_clear");
	}
	cleared, clear_status := entasis.world_stats(&world);
	if clear_status != .Ok
	{
		panic("world_stats clear");
	}
	fmt.printf(
		"workers=%d cleared_step=%d active=%d sleeping=%d statics=%d shapes=%d shape_types=%d\n",
		worker_count,
		cleared.step_index,
		cleared.active_bodies,
		cleared.sleeping_bodies,
		cleared.statics,
		cleared.registered_shapes,
		cleared.registered_shape_types,
	);
	if entasis.world_destroy(&world) != .Ok
	{
		panic("world_destroy");
	}
}

main :: proc()
{
	emit_world(1);
	emit_world(4);
}
