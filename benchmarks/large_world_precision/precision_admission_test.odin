package large_world_precision

@(require) import "core:math"
@(require) import "core:testing"
@(require) import entasis "entasis:entasis"

when PRECISION_NATIVE_GLOBAL
{
	@(test)
	mixed64_contact_oracle_is_order_independent_but_not_geometry_blind :: proc(t: ^testing.T)
	{
		a: entasis.Solver_Contact_Data = {body_count=2, contact_count=2, offset_b={1, 0, 0}};
		a.contacts[0] = {offset_a={0.5, 0, 0}, normal={0, 1, 0}, depth=0.002, feature_id=7};
		a.contacts[1] = {offset_a={0.5, 0, 1}, normal={0, 1, 0}, depth=0.003, feature_id=7};
		b: entasis.Solver_Contact_Data = a;
		b.contacts[0], b.contacts[1] = b.contacts[1], b.contacts[0];
		b.impulses[0] = 123;
		b.contacts[0].normal_impulse = 42;
		errors: Precision_Contact_Errors;
		testing.expect_value(t, precision_contact_geometry_matches(&a, &b, &errors), Precision_Admission.Accepted);
		testing.expect_value(t, errors.contacts, 2);
		testing.expect_value(t, errors.depth, f64(0));
		testing.expect_value(t, errors.offset, f64(0));
		b.contacts[0] = b.contacts[1];
		testing.expect_value(t, precision_contact_geometry_matches(&a, &b, &errors), Precision_Admission.Rejected);
		for mutation in 0 ..< 7
		{
			b = a;
			switch mutation
			{
			case 0: b.contacts[0].depth += 0.002;
			case 1: b.contacts[0].offset_a.x += 0.002;
			case 2: b.contacts[0].normal.x += 0.0002;
			case 3: b.contacts[0].feature_id += 1;
			case 4: b.offset_b.y += 0.002;
			case 5: b.contact_count -= 1;
			case 6: b.contacts[0].depth = math.nan_f32();
			}
			testing.expectf(t, precision_contact_geometry_matches(&a, &b, &errors) == .Rejected, "mutation=%d", mutation);
		}
	}

	@(test)
	mixed64_contact_oracle_requires_logical_body_and_static_identity :: proc(t: ^testing.T)
	{
		near: entasis.Solver_Contact_Data = {body_count=2, body_handles={{1}, {2}, {}, {}}};
		far: entasis.Solver_Contact_Data = {body_count=2, body_handles={{626}, {627}, {}, {}}};
		testing.expect_value(t, precision_contact_pair_matches(&far, &near, 625, {}, {}), Precision_Admission.Accepted);
		far.body_handles[1].value += 1;
		testing.expect_value(t, precision_contact_pair_matches(&far, &near, 625, {}, {}), Precision_Admission.Rejected);
		near.body_count = 1;
		far.body_count = 1;
		near.pair_b = {packed=0x80000000};
		far.pair_b = {packed=0x80000001};
		testing.expect_value(t, precision_contact_pair_matches(&far, &near, 625, far.pair_b, near.pair_b), Precision_Admission.Accepted);
		testing.expect_value(t, precision_contact_pair_matches(&far, &near, 625, near.pair_b, near.pair_b), Precision_Admission.Rejected);
	}

	@(test)
	mixed64_orientation_oracle_resolves_small_angles_and_quaternion_sign :: proc(t: ^testing.T)
	{
		identity: entasis.Quaternion = {0, 0, 0, 1};
		testing.expect_value(t, precision_orientation_error(identity, {0, 0, 0, -2}), f64(0));
		for angle in ([4]f64{1e-7, 1e-5, 0.0002, 0.2})
		{
			rotation: entasis.Quaternion = {f32(math.sin(angle * 0.5)), 0, 0, f32(math.cos(angle * 0.5))};
			testing.expectf(t, abs(precision_orientation_error(identity, rotation) - angle) <= angle * 1e-6, "angle=%.17g", angle);
		}
		for rotation in ([3]entasis.Quaternion{{}, {math.nan_f32(), 0, 0, 1}, {math.inf_f32(1), 0, 0, 1}})
		{
			testing.expect_value(t, precision_close(precision_orientation_error(identity, rotation), 0, 0.0001), Precision_Admission.Rejected);
		}
	}

	@(test)
	mixed64_controlled_oracle_rejects_position_velocity_and_orientation_corruption :: proc(t: ^testing.T)
	{
		pool: entasis.Buffer_Pool;
		testing.expect_value(t, entasis.buffer_pool_init(&pool, POOL_MINIMUM_BLOCK_SIZE), entasis.Status.Ok);
		defer testing.expect_value(t, entasis.buffer_pool_destroy(&pool), entasis.Status.Ok);
		worlds: [2]Precision_Control_World;
		defer
		{
			for &owner in worlds
			{
				testing.expect_value(t, entasis.world_destroy(&owner.world), entasis.Status.Ok);
			}
		}
		testing.expect_value(t, precision_control_create(&worlds[0], &pool, {}, 1), entasis.Status.Ok);
		testing.expect_value(t, precision_control_create(&worlds[1], &pool, {1e9, 0, 0}, 1), entasis.Status.Ok);
		errors: Precision_Control_Errors;
		testing.expect_value(t, precision_control_compare(&worlds[0], &worlds[1], &errors), entasis.Status.Ok);
		handle: entasis.Body_Handle = worlds[1].bodies[0];
		description, read_status := entasis.body_get(&worlds[1].world, handle);
		pose, pose_status := entasis.body_get_global_pose(&worlds[1].world, handle);
		testing.expect_value(t, read_status, entasis.Status.Ok);
		testing.expect_value(t, pose_status, entasis.Status.Ok);
		for mutation in 0 ..< 4
		{
			changed: entasis.Body_Description = description;
			position: entasis.Global_Position = pose.position;
			switch mutation
			{
			case 0: position.x += 0.002;
			case 1: changed.velocity.linear.x = 0.0002;
			case 2: changed.velocity.angular.z = 0.0002;
			case 3: changed.pose.orientation = {0.0001, 0, 0, 1};
			}
			testing.expect_value(t, entasis.body_apply_global(&worlds[1].world, handle, changed, position), entasis.Status.Ok);
			testing.expect_value(t, precision_control_compare(&worlds[0], &worlds[1], &errors), entasis.Status.Invalid_Description);
		}
		testing.expect_value(t, entasis.body_apply_global(&worlds[1].world, handle, description, pose.position), entasis.Status.Ok);
		testing.expect_value(t, precision_control_compare(&worlds[0], &worlds[1], &errors), entasis.Status.Ok);
	}
}
