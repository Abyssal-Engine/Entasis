package public_api_tests

import "core:testing"
import entasis "entasis:entasis"

Contact_Event_Test_Scene :: struct
{
	world:       entasis.World,
	user_ids:    entasis.Contact_User_Table,
	tracker:     entasis.Contact_Tracker,
	shape:       entasis.Shape_Handle,
	body:        entasis.Body_Handle,
	static_body: entasis.Static_Handle,
	inertia:     entasis.Body_Inertia,
}

contact_event_test_scene_init :: proc(
	t: ^testing.T,
	scene: ^Contact_Event_Test_Scene,
	worker_count: int = 1,
	tracker_capacity: int = 4,
) -> bool
{
	description := small_world_description();
	description.gravity = {};
	description.damping = {};
	description.threading.worker_count = i32(worker_count);
	if !testing.expect_value(
		t, entasis.world_init(&scene.world, description), entasis.Status.Ok,
	)
	{
		return false;
	}
	if !testing.expect_value(
		t, entasis.contact_user_table_init(&scene.user_ids, 4, 4), entasis.Status.Ok,
	)
	{
		return false;
	}
	if !testing.expect_value(
		t, entasis.contact_tracker_init(&scene.tracker, tracker_capacity), entasis.Status.Ok,
	)
	{
		return false;
	}
	if !testing.expect_value(
		t, entasis.contact_tracker_bind(&scene.tracker, &scene.world, &scene.user_ids),
		entasis.Status.Ok,
	)
	{
		return false;
	}

	shape_value := entasis.sphere(1);
	shape_status: entasis.Status;
	scene.shape, shape_status = entasis.shape_add(&scene.world, shape_value);
	if !testing.expect_value(t, shape_status, entasis.Status.Ok)
	{
		return false;
	}
	inertia_status: entasis.Status;
	scene.inertia, inertia_status = entasis.shape_inertia(shape_value, 1);
	if !testing.expect_value(t, inertia_status, entasis.Status.Ok)
	{
		return false;
	}

	body_status: entasis.Status;
	scene.body, body_status = entasis.body_add(
		&scene.world,
		entasis.body_dynamic(
		scene.shape,
		scene.inertia,
		entasis.pose({0, 0, 0}),
		{},
		entasis.body_activity(-1, 255),
	),
	);
	if !testing.expect_value(t, body_status, entasis.Status.Ok)
	{
		return false;
	}
	static_status: entasis.Status;
	scene.static_body, static_status = entasis.static_add(
		&scene.world,
		entasis.static_body(scene.shape, entasis.pose({1.5, 0, 0})),
		.None,
	);
	if !testing.expect_value(t, static_status, entasis.Status.Ok)
	{
		return false;
	}
	if !testing.expect_value(
		t, entasis.contact_user_set_body(&scene.user_ids, scene.body, 111), entasis.Status.Ok,
	)
	{
		return false;
	}
	if !testing.expect_value(
		t, entasis.contact_user_set_static(&scene.user_ids, scene.static_body, 222),
		entasis.Status.Ok,
	)
	{
		return false;
	}
	return true;
}

contact_event_test_scene_destroy :: proc(scene: ^Contact_Event_Test_Scene)
{
	_ = entasis.contact_tracker_destroy(&scene.tracker);
	_ = entasis.contact_user_table_destroy(&scene.user_ids);
	_ = entasis.world_destroy(&scene.world);
}

contact_event_reset_body :: proc(
	scene: ^Contact_Event_Test_Scene,
	position: entasis.Vector3,
) -> entasis.Status
{
	status := entasis.body_set_pose(&scene.world, scene.body, entasis.pose(position));
	if status != .Ok
	{
		return status;
	}
	return entasis.body_set_velocity(&scene.world, scene.body, {});
}

contact_event_step_and_drain_one :: proc(
	t: ^testing.T,
	scene: ^Contact_Event_Test_Scene,
) -> (entasis.Contact_Event, bool)
{
	if !testing.expect_value(
		t, entasis.world_step(&scene.world, 1.0 / 60.0), entasis.Status.Ok,
	)
	{
		return {}, false;
	}
	events: [1]entasis.Contact_Event;
	written, required, status := entasis.contact_events_drain(&scene.tracker, events[:]);
	if !testing.expect_value(t, status, entasis.Status.Ok) ||
		!testing.expect_value(t, written, 1) ||
		!testing.expect_value(t, required, 1)
	{
		return {}, false;
	}
	return events[0], true;
}

@(test)
contact_tracker_reports_begin_persist_and_end_with_user_ids :: proc(t: ^testing.T)
{
	scene: Contact_Event_Test_Scene;
	if !contact_event_test_scene_init(t, &scene)
	{
		return;
	}
	defer contact_event_test_scene_destroy(&scene);

	begin, begin_ok := contact_event_step_and_drain_one(t, &scene);
	if !begin_ok
	{
		return;
	}
	testing.expect_value(t, begin.kind, entasis.Contact_Event_Kind.Begin);
	testing.expect_value(t, begin.user_a, u64(111));
	testing.expect_value(t, begin.user_b, u64(222));
	testing.expect(t, .User_A_Available in begin.flags);
	testing.expect(t, .User_B_Available in begin.flags);
	testing.expect(t, .Normal_Available in begin.flags);
	testing.expect(t, .Solver_Data_Available in begin.flags);
	testing.expect(t, begin.contact_count >= 1);
	testing.expect(t, begin.contact_data.contact_count >= 1);
	testing.expect_value(t, begin.contact_data.constraint, begin.constraint);
	testing.expect(t, begin.contact_data.contacts[0].feature_id != -1);

	if !testing.expect_value(
		t, contact_event_reset_body(&scene, {0, 0, 0}), entasis.Status.Ok,
	)
	{
		return;
	}
	persist, persist_ok := contact_event_step_and_drain_one(t, &scene);
	if !persist_ok
	{
		return;
	}
	testing.expect_value(t, persist.kind, entasis.Contact_Event_Kind.Persist);
	testing.expect_value(t, persist.a, begin.a);
	testing.expect_value(t, persist.b, begin.b);
	testing.expect(t, persist.contact_count >= 1);

	if !testing.expect_value(
		t, contact_event_reset_body(&scene, {10, 0, 0}), entasis.Status.Ok,
	)
	{
		return;
	}
	ended, end_ok := contact_event_step_and_drain_one(t, &scene);
	if !end_ok
	{
		return;
	}
	testing.expect_value(t, ended.kind, entasis.Contact_Event_Kind.End);
	testing.expect_value(t, ended.a, begin.a);
	testing.expect_value(t, ended.b, begin.b);
	testing.expect_value(t, ended.contact_count, u8(0));
	testing.expect_value(t, ended.user_a, u64(111));
	testing.expect_value(t, ended.user_b, u64(222));
}

@(test)
contact_event_capacity_failures_are_retryable_without_committing_history :: proc(t: ^testing.T)
{
	scene: Contact_Event_Test_Scene;
	if !contact_event_test_scene_init(t, &scene, 1, 0)
	{
		return;
	}
	defer contact_event_test_scene_destroy(&scene);
	if !testing.expect_value(
		t, entasis.world_step(&scene.world, 1.0 / 60.0), entasis.Status.Ok,
	)
	{
		return;
	}

	empty: [0]entasis.Contact_Event;
	written, required, status := entasis.contact_events_drain(&scene.tracker, empty[:]);
	testing.expect_value(t, written, 0);
	testing.expect_value(t, required, 1);
	testing.expect_value(t, status, entasis.Status.Capacity_Missing);
	if !testing.expect_value(
		t, entasis.contact_tracker_ensure_capacity(&scene.tracker, required), entasis.Status.Ok,
	)
	{
		return;
	}

	written, required, status = entasis.contact_events_drain(&scene.tracker, empty[:]);
	testing.expect_value(t, written, 0);
	testing.expect_value(t, required, 1);
	testing.expect_value(t, status, entasis.Status.Capacity_Missing);

	events: [1]entasis.Contact_Event;
	written, required, status = entasis.contact_events_drain(&scene.tracker, events[:]);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect_value(t, written, 1);
	testing.expect_value(t, required, 1);
	testing.expect_value(t, events[0].kind, entasis.Contact_Event_Kind.Begin);
}

contact_event_trace :: proc(
	t: ^testing.T,
	worker_count: int,
	output: []entasis.Contact_Event,
) -> bool
{
	scene: Contact_Event_Test_Scene;
	if !contact_event_test_scene_init(t, &scene, worker_count)
	{
		return false;
	}
	defer contact_event_test_scene_destroy(&scene);
	if len(output) < 3
	{
		return false;
	}
	for step in 0 ..< 3
	{
		if step == 1 && contact_event_reset_body(&scene, {0, 0, 0}) != .Ok
		{
			return false;
		}
		if step == 2 && contact_event_reset_body(&scene, {10, 0, 0}) != .Ok
		{
			return false;
		}
		if entasis.world_step(&scene.world, 1.0 / 60.0) != .Ok
		{
			return false;
		}
		written, required, status := entasis.contact_events_drain(
			&scene.tracker, output[step:step + 1],
		);
		if status != .Ok || written != 1 || required != 1
		{
			return false;
		}
	}
	return true;
}

@(test)
contact_event_trace_preserves_semantics_across_worker_counts :: proc(t: ^testing.T)
{
	one: [3]entasis.Contact_Event;
	two: [3]entasis.Contact_Event;
	if !testing.expect(t, contact_event_trace(t, 1, one[:]))
	{
		return;
	}
	if !testing.expect(t, contact_event_trace(t, 2, two[:]))
	{
		return;
	}
	for index in 0 ..< len(one)
	{
		testing.expect_value(t, one[index].kind, two[index].kind);
		testing.expect_value(t, one[index].contact_count, two[index].contact_count);
		testing.expect_value(t, one[index].user_a, two[index].user_a);
		testing.expect_value(t, one[index].user_b, two[index].user_b);
	}
}

@(test)
user_id_generation_distinguishes_reused_physics_handles :: proc(t: ^testing.T)
{
	scene: Contact_Event_Test_Scene;
	if !contact_event_test_scene_init(t, &scene)
	{
		return;
	}
	defer contact_event_test_scene_destroy(&scene);
	_, ok := contact_event_step_and_drain_one(t, &scene);
	if !ok
	{
		return;
	}

	old_static := scene.static_body;
	if !testing.expect_value(
		t, entasis.static_remove(&scene.world, old_static, .None), entasis.Status.Ok,
	)
	{
		return;
	}
	if !testing.expect_value(
		t, entasis.contact_user_remove_static(&scene.user_ids, old_static), entasis.Status.Ok,
	)
	{
		return;
	}
	new_static, add_status := entasis.static_add(
		&scene.world,
		entasis.static_body(scene.shape, entasis.pose({1.5, 0, 0})),
		.None,
	);
	if !testing.expect_value(t, add_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect_value(t, new_static, old_static);
	scene.static_body = new_static;
	if !testing.expect_value(
		t, entasis.contact_user_set_static(&scene.user_ids, new_static, 222), entasis.Status.Ok,
	)
	{
		return;
	}
	if contact_event_reset_body(&scene, {0, 0, 0}) != .Ok
	{
		return;
	}
	if !testing.expect_value(
		t, entasis.world_step(&scene.world, 1.0 / 60.0), entasis.Status.Ok,
	)
	{
		return;
	}

	events: [2]entasis.Contact_Event;
	written, required, status := entasis.contact_events_drain(&scene.tracker, events[:]);
	if !testing.expect_value(t, status, entasis.Status.Ok) ||
		!testing.expect_value(t, written, 2) ||
		!testing.expect_value(t, required, 2)
	{
		return;
	}
	begin_count := 0;
	end_count := 0;
	for event in events[:written]
	{
		testing.expect_value(t, event.user_b, u64(222));
		switch event.kind
		{
			case .Begin:
				begin_count += 1;
			case .End:
				end_count += 1;
			case .Persist:
				testing.expect(t, false);
		}
	}
	testing.expect_value(t, begin_count, 1);
	testing.expect_value(t, end_count, 1);
}

@(test)
contact_events_can_be_discarded_without_breaking_pair_history :: proc(t: ^testing.T)
{
	scene: Contact_Event_Test_Scene;
	if !contact_event_test_scene_init(t, &scene)
	{
		return;
	}
	defer contact_event_test_scene_destroy(&scene);
	if !testing.expect_value(
		t, entasis.world_step(&scene.world, 1.0 / 60.0), entasis.Status.Ok,
	)
	{
		return;
	}
	required, status := entasis.contact_events_discard(&scene.tracker);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect_value(t, required, 1);
	if contact_event_reset_body(&scene, {0, 0, 0}) != .Ok
	{
		return;
	}
	persist, ok := contact_event_step_and_drain_one(t, &scene);
	if !ok
	{
		return;
	}
	testing.expect_value(t, persist.kind, entasis.Contact_Event_Kind.Persist);
}

@(test)
contact_event_resource_helpers_cover_explicit_lifecycle :: proc(t: ^testing.T)
{
	users: entasis.Contact_User_Table;
	if !testing.expect_value(
		t, entasis.contact_user_table_init(&users, 0, 0), entasis.Status.Ok,
	)
	{
		return;
	}
	if !testing.expect_value(
		t, entasis.contact_user_table_ensure_capacity(&users, 2, 2), entasis.Status.Ok,
	)
	{
		return;
	}
	body := entasis.Body_Handle{value=0};
	static_body := entasis.Static_Handle{value=0};
	if !testing.expect_value(
		t, entasis.contact_user_set_body(&users, body, 11), entasis.Status.Ok,
	)
	{
		return;
	}
	if !testing.expect_value(
		t, entasis.contact_user_set_static(&users, static_body, 22), entasis.Status.Ok,
	)
	{
		return;
	}
	if !testing.expect_value(
		t, entasis.contact_user_remove_body(&users, body), entasis.Status.Ok,
	)
	{
		return;
	}
	if !testing.expect_value(
		t, entasis.contact_user_remove_static(&users, static_body), entasis.Status.Ok,
	)
	{
		return;
	}
	if !testing.expect_value(
		t, entasis.contact_user_table_clear(&users), entasis.Status.Ok,
	)
	{
		return;
	}
	if !testing.expect_value(
		t, entasis.contact_user_table_destroy(&users), entasis.Status.Ok,
	)
	{
		return;
	}

	world: entasis.World;
	if !testing.expect_value(
		t, entasis.world_init(&world, small_world_description()), entasis.Status.Ok,
	)
	{
		return;
	}
	defer entasis.world_destroy(&world);
	tracker: entasis.Contact_Tracker;
	if !testing.expect_value(
		t, entasis.contact_tracker_init(&tracker, 0), entasis.Status.Ok,
	)
	{
		return;
	}
	testing.expect_value(t, entasis.contact_tracker_capacity(&tracker), 0);
	if !testing.expect_value(
		t, entasis.contact_tracker_ensure_capacity(&tracker, 4), entasis.Status.Ok,
	)
	{
		return;
	}
	testing.expect(t, entasis.contact_tracker_capacity(&tracker) >= 4);
	if !testing.expect_value(
		t, entasis.contact_tracker_bind(&tracker, &world), entasis.Status.Ok,
	)
	{
		return;
	}
	if !testing.expect_value(
		t, entasis.contact_tracker_clear(&tracker), entasis.Status.Ok,
	)
	{
		return;
	}
	if !testing.expect_value(
		t, entasis.contact_tracker_unbind(&tracker), entasis.Status.Ok,
	)
	{
		return;
	}
	if !testing.expect_value(
		t, entasis.contact_tracker_bind(&tracker, &world), entasis.Status.Ok,
	)
	{
		return;
	}
	if !testing.expect_value(
		t, entasis.contact_tracker_unbind(&tracker), entasis.Status.Ok,
	)
	{
		return;
	}
	testing.expect_value(t, entasis.contact_tracker_destroy(&tracker), entasis.Status.Ok);
}

contact_event_set_sample :: proc(
	t: ^testing.T,
	worker_count: int,
	output: []entasis.Contact_Event,
) -> int
{
	scene: Contact_Event_Test_Scene;
	if !contact_event_test_scene_init(t, &scene, worker_count, 8)
	{
		return 0;
	}
	defer contact_event_test_scene_destroy(&scene);

	for index in 1 ..= 2
	{
		x := f32(index * 10);
		body, body_status := entasis.body_add(
			&scene.world,
			entasis.body_dynamic(
			scene.shape,
			scene.inertia,
			entasis.pose({x, 0, 0}),
			{},
			entasis.body_activity(-1, 255),
		),
		);
		if !testing.expect_value(t, body_status, entasis.Status.Ok)
		{
			return 0;
		}
		static_body, static_status := entasis.static_add(
			&scene.world,
			entasis.static_body(scene.shape, entasis.pose({x + 1.5, 0, 0})),
			.None,
		);
		if !testing.expect_value(t, static_status, entasis.Status.Ok)
		{
			return 0;
		}
		if !testing.expect_value(
			t, entasis.contact_user_set_body(&scene.user_ids, body, u64(100 + index)),
			entasis.Status.Ok,
		)
		{
			return 0;
		}
		if !testing.expect_value(
			t, entasis.contact_user_set_static(
			&scene.user_ids, static_body, u64(200 + index),
		),
			entasis.Status.Ok,
		)
		{
			return 0;
		}
	}
	if !testing.expect_value(
		t, entasis.world_step(&scene.world, 1.0 / 60.0), entasis.Status.Ok,
	)
	{
		return 0;
	}
	written, required, status := entasis.contact_events_drain(&scene.tracker, output);
	if !testing.expect_value(t, status, entasis.Status.Ok)
	{
		return 0;
	}
	if !testing.expect_value(t, required, 3)
	{
		return 0;
	}
	return written;
}

contact_event_user_pair_mask :: proc(
	t: ^testing.T,
	events: []entasis.Contact_Event,
) -> u8
{
	mask: u8;
	for event in events
	{
		testing.expect_value(t, event.kind, entasis.Contact_Event_Kind.Begin);
		testing.expect_value(t, event.contact_count, u8(1));
		switch
		{
			case event.user_a == 111 && event.user_b == 222:
				mask |= 1 << 0;
			case event.user_a == 101 && event.user_b == 201:
				mask |= 1 << 1;
			case event.user_a == 102 && event.user_b == 202:
				mask |= 1 << 2;
			case:
				testing.expect(t, false);
		}
	}
	return mask;
}

@(test)
contact_event_contents_match_across_worker_counts_without_order_contract :: proc(t: ^testing.T)
{
	one_worker: [3]entasis.Contact_Event;
	two_workers: [3]entasis.Contact_Event;
	one_count := contact_event_set_sample(t, 1, one_worker[:]);
	two_count := contact_event_set_sample(t, 2, two_workers[:]);
	if !testing.expect_value(t, one_count, 3) ||
		!testing.expect_value(t, two_count, one_count)
	{
		return;
	}
	expected_mask := u8(0b111);
	testing.expect_value(
		t, contact_event_user_pair_mask(t, one_worker[:one_count]), expected_mask,
	);
	testing.expect_value(
		t, contact_event_user_pair_mask(t, two_workers[:two_count]), expected_mask,
	);
}

@(test)
contact_events_preserve_pair_lifetimes_without_collection_order_contract :: proc(t: ^testing.T)
{
	scene: Contact_Event_Test_Scene;
	if !contact_event_test_scene_init(t, &scene, 2)
	{
		return;
	}
	defer contact_event_test_scene_destroy(&scene);

	begin, begin_ok := contact_event_step_and_drain_one(t, &scene);
	if !begin_ok
	{
		return;
	}
	testing.expect_value(t, begin.kind, entasis.Contact_Event_Kind.Begin);
	if contact_event_reset_body(&scene, {0, 0, 0}) != .Ok
	{
		return;
	}
	persist, persist_ok := contact_event_step_and_drain_one(t, &scene);
	if !persist_ok
	{
		return;
	}
	testing.expect_value(t, persist.kind, entasis.Contact_Event_Kind.Persist);
	if contact_event_reset_body(&scene, {10, 0, 0}) != .Ok
	{
		return;
	}
	ended, end_ok := contact_event_step_and_drain_one(t, &scene);
	if !end_ok
	{
		return;
	}
	testing.expect_value(t, ended.kind, entasis.Contact_Event_Kind.End);
}
