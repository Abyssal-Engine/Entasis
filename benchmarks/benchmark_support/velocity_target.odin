package benchmark_support

import "core:simd"
import entasis "entasis:entasis"
import util "entasis:entasis_utilities"

Velocity_Target_Prestep :: struct
{
	target_speed: entasis.F32x8,
}

Velocity_Target_Impulses :: struct
{
	accumulated: entasis.F32x8,
}

velocity_target_kernel :: proc "contextless" (
	prestep_raw: rawptr,
	bodies: ^[4]entasis.Constraint_Kernel_Body_Wide,
	dt, inverse_dt: f32,
	impulses_raw: rawptr,
	active_mask: entasis.I32x8,
	phase: entasis.Constraint_Kernel_Phase,
)
{
	_, _ = dt, inverse_dt;
	prestep := (^Velocity_Target_Prestep)(prestep_raw);
	impulses := (^Velocity_Target_Impulses)(impulses_raw);
	body := &bodies[0];
	switch phase
	{
		case .Prestep, .Incremental_Update:
			return;
		case .Warmstart:
			updated := simd.add(
				body.linear_velocity.x,
				simd.mul(impulses.accumulated, body.inverse_mass),
			);
			body.linear_velocity.x = util.wide_select_f32(
				active_mask, updated, body.linear_velocity.x,
			);
		case .Solve:
			delta_velocity := simd.sub(prestep.target_speed, body.linear_velocity.x);
			inverse_mass := util.wide_select_f32(active_mask, body.inverse_mass, entasis.F32x8(1));
			impulse_delta := simd.div(delta_velocity, inverse_mass);
			updated_velocity := simd.add(
				body.linear_velocity.x,
				simd.mul(impulse_delta, body.inverse_mass),
			);
			body.linear_velocity.x = util.wide_select_f32(
				active_mask, updated_velocity, body.linear_velocity.x,
			);
			updated_impulse := simd.add(impulses.accumulated, impulse_delta);
			impulses.accumulated = util.wide_select_f32(
				active_mask, updated_impulse, impulses.accumulated,
			);
	}
}

