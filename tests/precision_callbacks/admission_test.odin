package precision_callbacks_tests

import "core:testing"
import "core:math"
import e "entasis:entasis"
import p "entasis:entasis_physics"

@(test)
global_callback_table_requires_exactly_one_velocity_contract :: proc(t: ^testing.T)
{
	capture: Capture;
	callbacks: e.Pose_Callbacks = capture_callbacks(&capture);
	testing.expect_value(t, p.pose_integrator_callbacks_validate(callbacks), e.Status.Ok);
	callbacks.integrate_velocity = capture_legacy;
	testing.expect_value(t, p.pose_integrator_callbacks_validate(callbacks), e.Status.Invalid_Argument);
	callbacks.integrate_velocity_global = nil;
	testing.expect_value(t, p.pose_integrator_callbacks_validate(callbacks), e.Status.Ok);
	callbacks.integrate_velocity = nil;
	testing.expect_value(t, p.pose_integrator_callbacks_validate(callbacks), e.Status.Invalid_Argument);
	for kind in 0 ..< 3
	{
		description: e.World_Description = e.world_description_default();
		description.pose_callbacks.integrate_velocity_global = capture_global;
		if kind == 1
		{
			description.pose_callbacks = capture_callbacks(&capture);
			description.pose_callbacks.integrate_velocity = capture_legacy;
		}
		else if kind == 2
		{
			description.pose_callbacks = capture_callbacks(&capture);
			description.pose_callbacks.integrate_velocity_global = nil;
		}
		world: e.World;
		testing.expect_value(t, e.world_init(&world, description), e.Status.Invalid_Description);
	}
	testing.expect_value(t, capture.initializations, 0);
	scene: Scene;
	scene_initialize(t, &scene, callbacks=capture_callbacks(&capture));
	testing.expect_value(t, capture.initializations, 1);
	scene_dispose(t, &scene);
	testing.expect_value(t, capture.disposals, 1);
	testing.expect_value(t, size_of(e.Global_Position_Bundle), 192);
}

@(test)
global_planetary_policy_rejects_nonfinite_inputs_before_activation :: proc(t: ^testing.T)
{
	for kind in 0 ..< 8
	{
		policy: e.Global_Planetary_Gravity_Policy = e.planetary_gravity_policy_global();
		switch kind
		{
			case 0: policy.center.x = math.inf_f64(1);
			case 1: policy.center.y = math.nan_f64();
			case 2: policy.gravity = -1;
			case 3: policy.gravity = math.inf_f32(1);
			case 4: policy.gravity = math.nan_f32();
			case 5: policy.linear_damping = math.nan_f32();
			case 6: policy.angular_damping = 2;
			case 7: policy.center.z = f64(f32(math.F32_MAX)) * 2;
		}
		testing.expect_value(t, e.planetary_gravity_global_initialize(&policy, nil), e.Status.Invalid_Description);
	}
	policy: e.Global_Planetary_Gravity_Policy = e.planetary_gravity_policy_global();
	for dt in ([4]f32{0, -1, math.inf_f32(1), math.nan_f32()})
	{
		testing.expect_value(t, e.planetary_gravity_global_prepare(&policy, dt), e.Status.Invalid_Argument);
	}
	testing.expect_value(t, e.planetary_gravity_global_initialize(nil, nil), e.Status.Invalid_Argument);
}

@(test)
global_callback_initialization_failure_does_not_activate_or_poison_world :: proc(t: ^testing.T)
{
	capture: Capture = {initialize_status=.Capacity_Missing};
	world: e.World;
	description: e.World_Description = e.world_description_default();
	description.pose_callbacks = capture_callbacks(&capture);
	description.capacity.bodies = 8;
	description.capacity.statics = 8;
	description.capacity.constraints = 16;
	testing.expect_value(t, e.world_init(&world, description), e.Status.Capacity_Missing);
	testing.expect_value(t, capture.initializations, 1);
	testing.expect_value(t, capture.disposals, 0);
	capture.initialize_status = .Ok;
	testing.expect_value(t, e.world_init(&world, description), e.Status.Ok);
	testing.expect_value(t, capture.initializations, 2);
	testing.expect_value(t, e.world_destroy(&world), e.Status.Ok);
	testing.expect_value(t, capture.disposals, 1);
}
