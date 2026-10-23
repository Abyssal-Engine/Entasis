package precision_callbacks_tests

import "core:math"
import "core:testing"
import e "entasis:entasis"
import p "entasis:entasis_physics"
import util "entasis:entasis_utilities"

@(test)
global_planetary_gravity_has_independent_one_axis_oracle :: proc(t: ^testing.T)
{
	for origin in ([4]f64{0, 1e3, 1e6, 1e9})
	{
		policy: e.Global_Planetary_Gravity_Policy = e.planetary_gravity_policy_global({origin + 0.125, -origin + 0.25, origin - 0.125}, 32);
		testing.expect_value(t, e.planetary_gravity_global_initialize(&policy, nil), e.Status.Ok);
		testing.expect_value(t, e.planetary_gravity_global_prepare(&policy, 0.015625), e.Status.Ok);
		for axis in 0 ..< 3
		{
			positions: e.Global_Position_Bundle;
			radii: [8]f64 = {0, 0.001953125, -0.001953125, 0.5, 1, 2.001953125, -3.001953125, 6.4e6};
			for lane in 0 ..< 8
			{
				positions[lane] = policy.center;
				switch axis
				{
					case 0: positions[lane].x += radii[lane];
					case 1: positions[lane].y += radii[lane];
					case 2: positions[lane].z += radii[lane];
				}
			}
			velocity: p.Body_Velocity_Wide;
			e.planetary_gravity_global_velocity(&policy, {}, &positions, {}, {}, util.I32x8(-1), 0, util.F32x8(0.015625), &velocity);
			for lane in 0 ..< 8
			{
				r: f64 = radii[lane];
				expected: f64 = -0.5 * r;
				if abs(r) > 1
				{
					expected = -0.5 / (r * r);
					if r < 0
					{
						expected = -expected;
					}
				}
				actual: util.Vector3 = util.vector3_wide_read_slot(velocity.linear, lane);
				components: [3]f32 = {actual.x, actual.y, actual.z};
				testing.expectf(t, abs(f64(components[axis]) - expected) <= max(1e-20, abs(expected) * 1e-6),
					"origin %v axis %v radius %v actual %v expected %v", origin, axis, r, components[axis], expected);
				for other in 0 ..< 3
				{
					if other != axis
					{
						testing.expect_value(t, components[other], f32(0));
					}
				}
			}
		}
	}
}

@(test)
global_planetary_gravity_masks_and_extreme_finite_coordinates :: proc(t: ^testing.T)
{
	limit: f64 = f64(f32(math.F32_MAX));
	policy: e.Global_Planetary_Gravity_Policy = e.planetary_gravity_policy_global({-limit, -limit, -limit}, 1);
	testing.expect_value(t, e.planetary_gravity_global_initialize(&policy, nil), e.Status.Ok);
	testing.expect_value(t, e.planetary_gravity_global_prepare(&policy, 1), e.Status.Ok);
	for bits in 0 ..< 256
	{
		positions: e.Global_Position_Bundle;
		masks: [8]i32;
		for lane in 0 ..< 8
		{
			positions[lane] = {math.nan_f64(), math.nan_f64(), math.nan_f64()};
			if bits & (1 << uint(lane)) != 0
			{
				positions[lane] = {limit, limit, limit};
				masks[lane] = -1;
			}
		}
		velocity: p.Body_Velocity_Wide = {linear=util.vector3_wide_broadcast({1, 2, 3}), angular=util.vector3_wide_broadcast({4, 5, 6})};
		prior: p.Body_Velocity_Wide = velocity;
		e.planetary_gravity_global_velocity(&policy, {}, &positions, {}, {}, transmute(util.I32x8)masks, 0, util.F32x8(1), &velocity);
		for lane in 0 ..< 8
		{
			testing.expect_value(t, util.vector3_wide_read_slot(velocity.linear, lane), util.vector3_wide_read_slot(prior.linear, lane));
			testing.expect_value(t, util.vector3_wide_read_slot(velocity.angular, lane), util.vector3_wide_read_slot(prior.angular, lane));
		}
	}
}

@(test)
global_planetary_steps_match_near_world_across_workers_and_origin_shift :: proc(t: ^testing.T)
{
	for workers in ([2]i32{1, 4})
	{
		for substeps in ([2]i32{1, 4})
		{
			policies: [2]e.Global_Planetary_Gravity_Policy;
			scenes: [2]Scene;
			bodies: [2][17]e.Body_Handle;
			for variant in 0 ..< 2
			{
				origin: f64 = f64(variant) * 1e9;
				policies[variant] = e.planetary_gravity_policy_global({origin + 0.125, -origin + 0.25, origin - 0.125}, 1, 0.02, 0.01);
				callbacks: e.Pose_Callbacks = e.pose_policy_planetary_global(&policies[variant]);
				callbacks.allow_substeps_for_unconstrained = .Enabled;
				scene_initialize(t, &scenes[variant], substeps=substeps, workers=workers, callbacks=callbacks);
				for index in 0 ..< 17
				{
					bodies[variant][index] = add_body(t, &scenes[variant], {origin + 2.126953125 + f64(index) / 8, -origin + 0.25, origin - 0.125}, {});
				}
			}
			for frame in 0 ..< 128
			{
				if frame == 64
				{
					testing.expect_value(t, p.simulation_translate_origin(scenes[1].simulation, {1e9, -1e9, 1e9}), e.Status.Ok);
					policies[1].center.x -= 1e9;
					policies[1].center.y += 1e9;
					policies[1].center.z -= 1e9;
				}
				for variant in 0 ..< 2
				{
					testing.expect_value(t, e.world_step(&scenes[variant].world, 0.00390625), e.Status.Ok);
				}
				for index in 0 ..< 17
				{
					a, b: e.Global_Pose;
					va, vb: e.Body_Description;
					status: e.Status;
					a, status = e.body_get_global_pose(&scenes[0].world, bodies[0][index]);
					testing.expect_value(t, status, e.Status.Ok);
					b, status = e.body_get_global_pose(&scenes[1].world, bodies[1][index]);
					testing.expect_value(t, status, e.Status.Ok);
					va, status = e.body_get(&scenes[0].world, bodies[0][index]);
					testing.expect_value(t, status, e.Status.Ok);
					vb, status = e.body_get(&scenes[1].world, bodies[1][index]);
					testing.expect_value(t, status, e.Status.Ok);
					testing.expect(t, abs((a.position.x - policies[0].center.x) - (b.position.x - policies[1].center.x)) < 1e-4);
					testing.expect(t, abs(va.velocity.linear.x - vb.velocity.linear.x) < 1e-5);
					testing.expect_value(t, a.position.y - policies[0].center.y, b.position.y - policies[1].center.y);
					testing.expect_value(t, a.position.z - policies[0].center.z, b.position.z - policies[1].center.z);
				}
			}
			for variant in 0 ..< 2
			{
				scene_dispose(t, &scenes[variant]);
			}
		}
	}
}
