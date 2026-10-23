package precision_constraints_tests

import "core:simd"
import "core:testing"
import p "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Relative_Probe_Description :: struct
{
	body_count, initial_positions, solve_positions: f32,
}

Relative_Probe_Prestep :: struct
{
	body_count, initial_positions, solve_positions: util.F32x8,
}

Relative_Probe_Impulses :: struct
{
	positions: [4]util.Vector3_Wide,
	marker: util.F32x8,
}

Relative_Probe_Context :: struct
{
	marker: f32,
}

relative_probe_validate :: proc "contextless" (type_id: i32, description: rawptr) -> p.Physics_Status
{
	_ = type_id;
	value: ^Relative_Probe_Description = (^Relative_Probe_Description)(description);
	if value.body_count < 1 || value.body_count > 4
	{
		return .Invalid_Description;
	}
	return .Ok;
}

relative_probe_validate_contextual :: proc "contextless" (user: rawptr, type_id: i32, description: rawptr) -> p.Physics_Status
{
	_ = user;
	return relative_probe_validate(type_id, description);
}

relative_probe_execute :: proc "contextless" (
	prestep: rawptr, bodies: ^[4]p.Constraint_Kernel_Body_Wide, impulses: rawptr,
	mask: util.I32x8, phase: p.Constraint_Kernel_Phase, marker: f32,
)
{
	data: ^Relative_Probe_Prestep = (^Relative_Probe_Prestep)(prestep);
	output: ^Relative_Probe_Impulses = (^Relative_Probe_Impulses)(impulses);
	for lane in 0 ..< 8
	{
		if simd.extract(mask, lane) == 0
		{
			continue;
		}
		count: int = int(simd.extract(data.body_count, lane));
		requested: u32 = u32(simd.extract(data.solve_positions if phase == .Solve else data.initial_positions, lane));
		for body in 0 ..< count
		{
			if requested & (u32(1) << u32(body)) == 0
			{
				continue;
			}
			position: util.Vector3 = util.vector3_wide_read_slot(bodies[body].position, lane);
			util.vector3_wide_write_slot(&output.positions[body], lane, position);
			if phase == .Solve
			{
				util.vector3_wide_write_slot(&bodies[body].linear_velocity, lane, position);
			}
		}
		output.marker = simd.replace(output.marker, lane, marker);
	}
}

relative_probe_kernel :: proc "contextless" (
	prestep: rawptr, bodies: ^[4]p.Constraint_Kernel_Body_Wide, dt, inverse_dt: f32,
	impulses: rawptr, mask: util.I32x8, phase: p.Constraint_Kernel_Phase,
)
{
	_, _ = dt, inverse_dt;
	relative_probe_execute(prestep, bodies, impulses, mask, phase, 0);
}

relative_probe_kernel_contextual :: proc "contextless" (
	user, prestep: rawptr, bodies: ^[4]p.Constraint_Kernel_Body_Wide, dt, inverse_dt: f32,
	impulses: rawptr, mask: ^util.I32x8, phase: p.Constraint_Kernel_Phase,
)
{
	_, _ = dt, inverse_dt;
	value: ^Relative_Probe_Context = (^Relative_Probe_Context)(user);
	relative_probe_execute(prestep, bodies, impulses, mask^, phase, value.marker);
}

relative_probe_record :: proc "contextless" (
	count: int, initial_positions, solve_positions: u8,
	space: p.Constraint_Position_Space = .Body_A_Relative,
) -> p.Constraint_Type_Record
{
	record: p.Constraint_Type_Record = {
		type_id=p.FIRST_CALLER_CONSTRAINT_TYPE_ID, body_count=i32(count),
		description_size=size_of(Relative_Probe_Description), prestep_bundle_size=size_of(Relative_Probe_Prestep),
		impulse_bundle_size=size_of(Relative_Probe_Impulses), position_space=space,
		apply_description=p.constraint_description_transfer, build_description=p.constraint_description_build,
		validate_description=relative_probe_validate, prestep_warmstart_solve=relative_probe_kernel,
		incrementally_update=relative_probe_kernel, move_record=p.constraint_storage_move_lane, remove_record=p.constraint_storage_remove_lane,
	};
	for body in 0 ..< count
	{
		record.initial_access[body] = p.BODY_ACCESS_ALL if initial_positions & (u8(1) << u32(body)) != 0 else p.BODY_ACCESS_NO_POSITION;
		record.solve_access[body] = p.BODY_ACCESS_ALL if solve_positions & (u8(1) << u32(body)) != 0 else p.BODY_ACCESS_NO_POSITION;
	}
	return record;
}

relative_probe_binding :: proc "contextless" (probe_context: ^Relative_Probe_Context) -> p.Contextual_Constraint_Binding
{
	return {user_context=probe_context, validate_description=relative_probe_validate_contextual,
		kernel=relative_probe_kernel_contextual, incremental_kernel=relative_probe_kernel_contextual};
}

relative_probe_populate :: proc(t: ^testing.T, scene: ^Scene, record: p.Constraint_Type_Record,
	fixture: ^Joint_Fixture, origin: f64, count: int = 17)
{
	description: Relative_Probe_Description = {body_count=f32(record.body_count)};
	for body in 0 ..< int(record.body_count)
	{
		if .Position in record.initial_access[body]
		{
			description.initial_positions += f32(u8(1) << u32(body));
		}
		if .Position in record.solve_access[body]
		{
			description.solve_positions += f32(u8(1) << u32(body));
		}
	}
	for group in 0 ..< count
	{
		for body in 0 ..< int(record.body_count)
		{
			position: p.Global_Position = {origin + f64(group) * 512 + f64(body) / 512,
				-origin + 0.125 - f64(body) / 256, origin + 0.25 + f64(body) / 128};
			fixture.bodies[group][body] = add_body(t, scene, position);
		}
		status: p.Physics_Status;
		fixture.joints[group], status = p.solver_add_raw(&scene.simulation.solver, record.type_id, &fixture.bodies[group], &description);
		testing.expect_value(t, status, p.Physics_Status.Ok);
	}
}

relative_probe_dispatch :: proc(t: ^testing.T, scene: ^Scene, record: ^p.Constraint_Type_Record,
	anchor: p.Constraint_Handle, stage: p.Solver_Execution_Stage, route: Execution_Route,
	$dispatch: p.Constraint_Callback_Dispatch)
{
	batch: ^p.Type_Batch = constraint_batch(t, scene, anchor);
	if route == .Registered_Bundle
	{
		for bundle in 0 ..< (int(batch.count) + 7) / 8
		{
			testing.expect_value(t, p.solver_execute_type_bundle_dispatch_integration(&scene.simulation.solver,
				batch, record, 0, bundle, 1.0 / 64, 64, .Continuation, stage, 0, dispatch, .Never), p.Physics_Status.Ok);
		}
	}
	else
	{
		for index in 0 ..< int(batch.count)
		{
			testing.expect_value(t, p.solver_execute_type_lane_dispatch_integration(&scene.simulation.solver,
				batch, record, 0, index, 1.0 / 64, 64, .Continuation, stage, 0, dispatch, .Never), p.Physics_Status.Ok);
		}
	}
}
