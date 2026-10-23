package benchmark_support

import "core:simd"
import "core:sync"
import p "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Counted_Gravity_Retention :: enum u8
{
	Unchanged,
	Half,
}

Counted_Gravity :: struct
{
	counts: [4096]i32,
	gravity: f32,
	retention: Counted_Gravity_Retention,
}

counted_gravity_initialize :: proc "contextless" (user_context: rawptr, simulation: ^p.Simulation) -> p.Physics_Status
{
	return .Ok;
}

counted_gravity_prepare :: proc "contextless" (user_context: rawptr, dt: f32) -> p.Physics_Status
{
	return .Ok;
}

counted_gravity_dispose :: proc "contextless" (user_context: rawptr)
{
}

counted_gravity_velocity :: proc "contextless" (
	user_context: rawptr, indices: util.I32x8, position: util.Vector3_Wide,
	orientation: util.Quaternion_Wide, inertia: p.Body_Inertia_Wide,
	mask: util.I32x8, worker_index: int, dt: util.F32x8, velocity: ^p.Body_Velocity_Wide,
)
{
	state: ^Counted_Gravity = (^Counted_Gravity)(user_context);
	for lane in 0 ..< util.PRODUCTION_LANE_COUNT
	{
		if simd.extract(mask, lane) == 0
		{
			continue;
		}
		index: i32 = simd.extract(indices, lane);
		if index >= 0
		{
			_ = sync.atomic_add_explicit(&state.counts[index], 1, .Relaxed);
		}
	}
	velocity.linear.y = simd.add(velocity.linear.y, simd.mul(util.F32x8(state.gravity), dt));
	if state.retention == .Half
	{
		linear_velocity_half(user_context, indices, position, orientation, inertia, mask, worker_index, dt, velocity);
	}
}


linear_velocity_half :: proc "contextless" (
	raw: rawptr, indices: util.I32x8, position: util.Vector3_Wide, orientation: util.Quaternion_Wide,
	inertia: p.Body_Inertia_Wide, mask: util.I32x8, worker: int, dt: util.F32x8, velocity: ^p.Body_Velocity_Wide,
)
{
	_, _, _, _, _, _, _, _ = raw, indices, position, orientation, inertia, mask, worker, dt;
	velocity.linear = util.vector3_wide_scale(velocity.linear, util.F32x8(0.5));
}

