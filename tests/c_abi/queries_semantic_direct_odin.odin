package main

import "core:fmt"
import "core:os"
import entasis "entasis:entasis"

check :: proc(status: entasis.Status, label: string)
{
	if status != .Ok
	{
		panic(label);
	}
}

bool_int :: #force_inline proc(value: bool) -> int
{
	if value
	{
		return 1;
	}
	return 0;
}

float_bits :: #force_inline proc(value: f32) -> u32
{
	return transmute(u32)value;
}

world_description :: proc() -> entasis.World_Description
{
	description := entasis.world_description_default();
	description.gravity = {};
	description.profiling = true;
	description.threading.worker_count = 1;
	description.capacity.bodies = 32;
	description.capacity.statics = 32;
	description.capacity.shapes_per_type = 16;
	description.capacity.constraints = 64;
	description.capacity.broad_phase_candidates = 256;
	description.capacity.pairs = 256;
	description.capacity.collision_child_pairs = 256;
	return description;
}

Query_Order :: struct
{
	results: []entasis.Query_Result, calls, failures: int,
}

query_order_allow :: proc "contextless" (user: rawptr, collidable: entasis.Collidable_Reference) -> bool
{
	state := (^Query_Order)(user);
	state.calls += 1;
	if state.results[9].count != 1
	{
		state.failures += 1;
	}
	return entasis.collidable_mobility(collidable) == .Static;
}

main :: proc()
{
	if len(os.args) > 2 || (len(os.args) == 2 && os.args[1] != "--queries-only")
	{
		os.exit(2);
	}
	world: entasis.World;
	check(entasis.world_init(&world, world_description()), "world_init");
	sphere_value := entasis.sphere(1);
	box_value := entasis.box(2, 2, 2);
	sphere_shape, status := entasis.shape_add(&world, sphere_value);
	check(status, "sphere_shape");
	box_shape, box_status := entasis.shape_add(&world, box_value);
	check(box_status, "box_shape");
	inertia, inertia_status := entasis.shape_inertia(sphere_value, 1);
	check(inertia_status, "inertia");
	dynamic_body, dynamic_status := entasis.body_add(
		&world,
		entasis.body_dynamic(
			sphere_shape,
			inertia,
			entasis.pose({0, 0, 0}),
			entasis.velocity(),
			entasis.body_activity(-1, 255),
		),
	);
	check(dynamic_status, "dynamic_body");
	_, kinematic_status := entasis.body_add(
		&world,
		entasis.body_kinematic(
			sphere_shape,
			entasis.pose({8, 0, 0}),
			entasis.velocity(),
			entasis.body_activity(-1, 255),
		),
	);
	check(kinematic_status, "kinematic_body");
	static_body, static_status := entasis.static_add(
		&world,
		entasis.static_body(box_shape, entasis.pose({5, 0, 0})),
		.None,
	);
	check(static_status, "static_body");
	ray_value := entasis.ray({-10, 0, 0}, {1, 0, 0}, 30);
	any_hit, any_status := entasis.ray_cast_any(&world, ray_value);
	check(any_status, "ray_any");
	closest, closest_status := entasis.ray_cast_closest(&world, ray_value);
	check(closest_status, "ray_closest");
	hits: [8]entasis.Ray_Hit;
	hit_count, hit_status := entasis.ray_cast_all(&world, ray_value, hits[:]);
	check(hit_status, "ray_all");
	mobility_mask: u32;
	for hit in hits[:hit_count]
	{
		mobility_mask |= u32(1) << u32(entasis.collidable_mobility(hit.collidable));
	}
	bounds := entasis.Bounding_Box{min={4.5, -1, -1}, max={5.5, 1, 1}};
	queries := [4]entasis.Query{
		entasis.query_ray_any(ray_value),
		entasis.query_ray_closest(ray_value),
		entasis.query_ray_all(ray_value, entasis.query_output(0, 8)),
		entasis.query_volume_all(bounds, entasis.query_output(0, 8)),
	};
	results: [4]entasis.Query_Result;
	batch_ray_hits: [8]entasis.Ray_Hit;
	batch_volume_hits: [8]entasis.Volume_Hit;
	scratch := entasis.query_scratch(batch_ray_hits[:], nil, batch_volume_hits[:]);
	check(entasis.query_batch(&world, queries[:], results[:], &scratch), "query_batch");
	packet: [23]entasis.Query;
	packet_results: [23]entasis.Query_Result;
	for &query in packet
	{
		query = entasis.query_ray_closest(ray_value);
	}
	check(entasis.query_batch(&world, packet[:], packet_results[:], nil), "packet batch");
	for result in packet_results
	{
		if result.ray_hit != closest || result.count != 1
		{
			panic("packet parity");
		}
	}
	order := Query_Order{results=packet_results[:]};
	packet[10] = entasis.query_ray_closest(ray_value, {allow=query_order_allow, user_context=&order});
	packet[19] = entasis.query_ray_closest(entasis.ray({}, {}, 30));
	packet[22] = entasis.query_ray_all(ray_value, entasis.query_output(0, 8));
	packet_results = {};
	if entasis.query_batch(&world, packet[:], packet_results[:], &scratch) != .Invalid_Argument
	{
		panic("mixed status");
	}
	for i in 0 ..< 22
	{
		if i == 10
		{
			if packet_results[i].ray_hit.t != 14
			{
				panic("callback result");
			}
		}
		else if i == 19
		{
			if packet_results[i].count != 0
			{
				panic("invalid result");
			}
		}
		else if packet_results[i].ray_hit != closest
		{
			panic("mixed packet parity");
		}
	}
	if order.calls != 3 || order.failures != 0 || packet_results[22].count != 3
	{
		panic("mixed order");
	}
	fmt.printf("packet count=%d callbacks=%d invalid=%d tail=%d\n", len(packet), order.calls, int(packet_results[19].status), packet_results[21].count);
	_, collision_status := entasis.collision_query(
		&world,
		sphere_shape,
		entasis.pose({0, 0, 0}),
		sphere_shape,
		entasis.pose({1.5, 0, 0}),
	);
	check(collision_status, "collision_query");
	fmt.printf(
		"query any=%d closest=%d closest_mobility=%d all=%d mask=%d batch=%d,%d,%d,%d collision=%d\n",
		bool_int(any_hit),
		float_bits(closest.t),
		int(entasis.collidable_mobility(closest.collidable)),
		hit_count,
		mobility_mask,
		results[0].count,
		results[1].count,
		results[2].count,
		results[3].count,
		bool_int(collision_status == .Ok),
	);
	check(entasis.distance_query_reserve(&world), "distance reserve");
	distance_queries: [5]entasis.Distance_Query;
	distance_results: [5]entasis.Distance_Query_Result;
	for &query, index in distance_queries
	{
		query.kind = entasis.Distance_Query_Kind(index if index<4 else 255);
		query.shape_a = sphere_shape;
		query.shape_b = sphere_shape;
		query.pose_a = entasis.pose();
		query.pose_b = entasis.pose({4 if index==1 else 1, 0, 0});
		query.point = {4, 0, 0};
		query.settings = entasis.distance_query_settings_default();
	}
	if entasis.distance_query_batch(&world, distance_queries[:], distance_results[:]) != .Invalid_Argument
	{
		panic("distance batch status");
	}
	for &value, index in distance_results
	{
		fmt.printf("distance item=%d status=%d state=%d distance=%d depth=%d pa=%d,%d,%d pb=%d,%d,%d normal=%d,%d,%d children=%d,%d correction=%d,%d,%d,%d\n",
			index, u8(value.status), u8(value.shape.geometry.state),
			float_bits(value.shape.geometry.distance), float_bits(value.shape.geometry.depth),
			float_bits(value.shape.geometry.point_a.x), float_bits(value.shape.geometry.point_a.y), float_bits(value.shape.geometry.point_a.z),
			float_bits(value.shape.geometry.point_b.x), float_bits(value.shape.geometry.point_b.y), float_bits(value.shape.geometry.point_b.z),
			float_bits(value.shape.geometry.normal.x), float_bits(value.shape.geometry.normal.y), float_bits(value.shape.geometry.normal.z),
			value.shape.child_a, value.shape.child_b, u8(value.correction.state),
			float_bits(value.correction.translation.x), float_bits(value.correction.translation.y), float_bits(value.correction.translation.z));
	}
	geometric_any, geometric_status := entasis.overlap_any(&world, sphere_shape, entasis.pose());
	check(geometric_status, "geometric overlap any");
	fmt.printf("geometric_any state=%d\n", u8(geometric_any));
	if len(os.args) == 2
	{
		check(entasis.world_destroy(&world), "world_destroy");
		return;
	}
	body_view, body_view_status := entasis.active_body_view(&world);
	check(body_view_status, "body_view");
	static_values, static_view_status := entasis.static_view(&world);
	check(static_view_status, "static_view");
	_, body_row_ok := entasis.active_body_row(body_view, 0);
	_, static_row_ok := entasis.static_view_row(static_values, 0);
	fmt.printf(
		"views bodies=%d statics=%d body_valid=%d static_valid=%d rows=%d,%d\n",
		body_view.count,
		static_values.count,
		bool_int(entasis.body_view_valid(&world, body_view)),
		bool_int(entasis.static_view_valid(&world, static_values)),
		bool_int(body_row_ok),
		bool_int(static_row_ok),
	);
	properties: entasis.Body_Property_Table(u64);
	check(entasis.body_property_init(&properties, 8), "property_init");
	check(entasis.body_property_set(&properties, dynamic_body, 777), "property_set");
	property_value, property_get_status := entasis.body_property_get(&properties, dynamic_body);
	check(property_get_status, "property_get");
	property_key, property_key_status := entasis.body_property_key(&properties, dynamic_body);
	check(property_key_status, "property_key");
	property_copy := property_value^;
	check(entasis.body_property_remove(&properties, dynamic_body), "property_remove");
	_, stale_status := entasis.body_property_get_key(&properties, property_key);
	fmt.printf(
		"property value=%d stale=%d capacity=%d\n",
		property_copy,
		bool_int(stale_status == .Not_Found),
		bool_int(entasis.body_property_capacity(&properties) >= 8),
	);
	check(entasis.body_property_destroy(&properties), "property_destroy");
	users: entasis.Contact_User_Table;
	tracker: entasis.Contact_Tracker;
	check(entasis.contact_user_table_init(&users, 8, 8), "users_init");
	check(entasis.contact_user_set_body(&users, dynamic_body, 111), "user_body");
	check(entasis.contact_user_set_static(&users, static_body, 222), "user_static");
	check(entasis.contact_tracker_init(&tracker, 8), "tracker_init");
	check(entasis.contact_tracker_bind(&tracker, &world, &users), "tracker_bind");
	zero_velocity := entasis.velocity();
	contact_pose := entasis.pose({3.5, 0, 0});
	check(entasis.body_set_pose(&world, dynamic_body, contact_pose), "contact_pose");
	check(entasis.body_set_velocity(&world, dynamic_body, zero_velocity), "contact_velocity");
	check(entasis.world_step(&world, 1.0 / 60.0), "step_begin");
	events: [4]entasis.Contact_Event;
	begin_count, _, begin_status := entasis.contact_events_drain(&tracker, events[:]);
	check(begin_status, "drain_begin");
	begin_kind := int(events[0].kind);
	begin_users := bool_int(
		(events[0].user_a == 111 || events[0].user_b == 111) &&
		(events[0].user_a == 222 || events[0].user_b == 222),
	);
	check(entasis.body_set_pose(&world, dynamic_body, contact_pose), "persist_pose");
	check(entasis.body_set_velocity(&world, dynamic_body, zero_velocity), "persist_velocity");
	check(entasis.world_step(&world, 1.0 / 60.0), "step_persist");
	persist_count, _, persist_status := entasis.contact_events_drain(&tracker, events[:]);
	check(persist_status, "drain_persist");
	persist_kind := int(events[0].kind);
	check(entasis.body_set_pose(&world, dynamic_body, entasis.pose({-10, 0, 0})), "end_pose");
	check(entasis.body_set_velocity(&world, dynamic_body, zero_velocity), "end_velocity");
	check(entasis.world_step(&world, 1.0 / 60.0), "step_end");
	end_count, _, end_status := entasis.contact_events_drain(&tracker, events[:]);
	check(end_status, "drain_end");
	end_kind := int(events[0].kind);
	fmt.printf(
		"events counts=%d,%d,%d kinds=%d,%d,%d users=%d\n",
		begin_count,
		persist_count,
		end_count,
		begin_kind,
		persist_kind,
		end_kind,
		begin_users,
	);
	profile_enabled, profile_enabled_status := entasis.world_profile_enabled(&world);
	check(profile_enabled_status, "profile_enabled");
	profile, profile_status := entasis.world_profile_snapshot(&world);
	check(profile_status, "profile");
	stats, stats_status := entasis.world_stats(&world);
	check(stats_status, "stats");
	solver_stats, solver_status := entasis.world_solver_stats(&world);
	check(solver_status, "solver_stats");
	fmt.printf(
		"profile enabled=%d step=%d trace=%d active=%d statics=%d batches_nonnegative=%d\n",
		bool_int(profile_enabled),
		profile.step_index,
		bool_int(profile.trace_count > 0),
		stats.active_bodies,
		stats.statics,
		bool_int(solver_stats.active_batches >= 0),
	);
	check(entasis.contact_tracker_unbind(&tracker), "tracker_unbind");
	check(entasis.contact_tracker_destroy(&tracker), "tracker_destroy");
	check(entasis.contact_user_table_destroy(&users), "users_destroy");
	check(entasis.world_destroy(&world), "world_destroy");
}
