package main

import "core:fmt"
import entasis "entasis:entasis"

BODY_COUNT :: 64;
STATIC_COUNT :: 32;

check :: proc(status: entasis.Status, label: string)
{
	if status != .Ok
	{
		panic(label);
	}
}

main :: proc()
{
	description := entasis.world_description_default();
	description.gravity = {};
	description.capacity.bodies = 128;
	description.capacity.statics = 64;
	description.capacity.shapes_per_type = 16;
	description.capacity.broad_phase_candidates = 512;
	description.capacity.pairs = 512;
	description.capacity.collision_child_pairs = 512;

	world: entasis.World;
	check(entasis.world_init(&world, description), "world_init");

	sphere_value := entasis.sphere(0.5);
	box_value := entasis.box(1, 1, 1);
	sphere, sphere_status := entasis.shape_add(&world, sphere_value);
	check(sphere_status, "sphere");
	box, box_status := entasis.shape_add(&world, box_value);
	check(box_status, "box");
	inertia, inertia_status := entasis.shape_inertia(sphere_value, 1);
	check(inertia_status, "inertia");

	body_descriptions: [BODY_COUNT]entasis.Body_Description;
	body_handles: [BODY_COUNT]entasis.Body_Handle;
	for index in 0 ..< BODY_COUNT
	{
		body_descriptions[index] = entasis.body_dynamic(
			sphere,
			inertia,
			entasis.pose({f32(index), 1, 0}),
			entasis.velocity({f32(index + 1), 0, 0}),
			entasis.body_activity(-1, 255),
		);
	}
	body_written, body_status := entasis.body_add_batch(
		&world, body_descriptions[:], body_handles[:],
	);
	check(body_status, "body_add_batch");
	if body_written != BODY_COUNT
	{
		panic("body count");
	}

	static_descriptions: [STATIC_COUNT]entasis.Static_Description;
	static_handles: [STATIC_COUNT]entasis.Static_Handle;
	for index in 0 ..< STATIC_COUNT
	{
		static_descriptions[index] = entasis.static_body(
			box, entasis.pose({f32(index), -1, 0}),
		);
	}
	static_written, static_status := entasis.static_add_batch(
		&world, static_descriptions[:], static_handles[:], .None,
	);
	check(static_status, "static_add_batch");
	if static_written != STATIC_COUNT
	{
		panic("static count");
	}

	for index in 0 ..< BODY_COUNT
	{
		body_descriptions[index].pose.position = {
			f32(index * 2), f32(index % 7), -f32(index),
		};
		body_descriptions[index].velocity.linear = {
			f32(100 + index), f32(index * 3), 0,
		};
	}
	body_applied, apply_status := entasis.body_apply_batch(
		&world, body_handles[:], body_descriptions[:],
	);
	check(apply_status, "body_apply_batch");
	if body_applied != BODY_COUNT
	{
		panic("body apply count");
	}

	for index in 0 ..< STATIC_COUNT
	{
		static_descriptions[index].pose.position = {
			f32(index * 3), -2, f32(index),
		};
	}
	static_applied, static_apply_status := entasis.static_apply_batch(
		&world, static_handles[:], static_descriptions[:], .None,
	);
	check(static_apply_status, "static_apply_batch");
	if static_applied != STATIC_COUNT
	{
		panic("static apply count");
	}

	body_position_sum: i64;
	body_velocity_sum: i64;
	for handle in body_handles
	{
		state, status := entasis.body_get(&world, handle);
		check(status, "body_get");
		body_position_sum += i64(state.pose.position.x);
		body_position_sum += i64(state.pose.position.y);
		body_position_sum += i64(state.pose.position.z);
		body_velocity_sum += i64(state.velocity.linear.x);
		body_velocity_sum += i64(state.velocity.linear.y);
	}
	static_position_sum: i64;
	for handle in static_handles
	{
		state, status := entasis.static_get(&world, handle);
		check(status, "static_get");
		static_position_sum += i64(state.pose.position.x);
		static_position_sum += i64(state.pose.position.y);
		static_position_sum += i64(state.pose.position.z);
	}

	sphere_info, sphere_info_status := entasis.shape_inspect(&world, sphere);
	check(sphere_info_status, "sphere_info");
	box_info, box_info_status := entasis.shape_inspect(&world, box);
	check(box_info_status, "box_info");
	stats, stats_status := entasis.world_stats(&world);
	check(stats_status, "world_stats");
	fmt.printf(
		"bodies=%d statics=%d body_position_sum=%d body_velocity_sum=%d static_position_sum=%d sphere_refs=%d box_refs=%d active=%d sleeping=%d registered_shapes=%d first_body=%d last_body=%d first_static=%d last_static=%d\n",
		body_written,
		static_written,
		body_position_sum,
		body_velocity_sum,
		static_position_sum,
		sphere_info.reference_count,
		box_info.reference_count,
		stats.active_bodies,
		stats.sleeping_bodies,
		stats.registered_shapes,
		body_handles[0].value,
		body_handles[BODY_COUNT - 1].value,
		static_handles[0].value,
		static_handles[STATIC_COUNT - 1].value,
	);

	body_removed, body_remove_status := entasis.body_remove_batch(&world, body_handles[:]);
	check(body_remove_status, "body_remove_batch");
	static_removed, static_remove_status := entasis.static_remove_batch(
		&world, static_handles[:], .None,
	);
	check(static_remove_status, "static_remove_batch");
	check(entasis.shape_remove(&world, sphere), "sphere_remove");
	check(entasis.shape_remove(&world, box), "box_remove");
	cleared, cleared_status := entasis.world_stats(&world);
	check(cleared_status, "cleared_stats");
	fmt.printf(
		"removed_bodies=%d removed_statics=%d active=%d sleeping=%d statics=%d registered_shapes=%d\n",
		body_removed,
		static_removed,
		cleared.active_bodies,
		cleared.sleeping_bodies,
		cleared.statics,
		cleared.registered_shapes,
	);
	check(entasis.world_destroy(&world), "world_destroy");
}
