package precision_callbacks_tests

import e "entasis:entasis"
import p "entasis:entasis_physics"
import util "entasis:entasis_utilities"
import "core:simd"

Capture_Mode :: enum u8
{
	Observe,
	Position_Acceleration,
}

Capture :: struct
{
	simulation: ^p.Simulation,
	origin: e.Global_Position,
	mode: Capture_Mode,
	initialize_status, prepare_status: e.Status,
	initializations, preparations, disposals: int,
	counts: [256]int,
	positions: [256][8]e.Global_Position,
	last: [256]e.Global_Position,
	worker_calls: [64]int,
	invalid_lanes: [64]int,
}

capture_initialize :: proc "contextless" (user_context: rawptr, simulation: ^p.Simulation) -> e.Status
{
	value: ^Capture = (^Capture)(user_context);
	value.simulation = simulation;
	value.initializations += 1;
	return value.initialize_status;
}

capture_prepare :: proc "contextless" (user_context: rawptr, dt: f32) -> e.Status
{
	_ = dt;
	value: ^Capture = (^Capture)(user_context);
	value.preparations += 1;
	return value.prepare_status;
}

capture_dispose :: proc "contextless" (user_context: rawptr)
{
	value: ^Capture = (^Capture)(user_context);
	value.disposals += 1;
	value.simulation = nil;
}

capture_global :: proc "contextless" (
	user_context: rawptr, indices: util.I32x8, positions: ^e.Global_Position_Bundle,
	orientation: util.Quaternion_Wide, inertia: p.Body_Inertia_Wide,
	mask: util.I32x8, worker: int, dt: util.F32x8, velocity: ^p.Body_Velocity_Wide,
)
{
	_, _ = orientation, inertia;
	value: ^Capture = (^Capture)(user_context);
	value.worker_calls[worker] += 1;
	active: ^p.Body_Set = &value.simulation.bodies.sets.memory[p.BODIES_ACTIVE_SET_INDEX];
	for lane in 0 ..< 8
	{
		if simd.extract(mask, lane) == 0
		{
			continue;
		}
		index: i32 = simd.extract(indices, lane);
		if index < 0 || int(index) >= active.count
		{
			value.invalid_lanes[worker] += 1;
			continue;
		}
		handle: i32 = active.index_to_handle.memory[index].value;
		count: int = value.counts[handle];
		if count < 8
		{
			value.positions[handle][count] = positions[lane];
		}
		value.counts[handle] += 1;
		value.last[handle] = positions[lane];
		if value.mode == .Position_Acceleration
		{
			linear: util.Vector3 = util.vector3_wide_read_slot(velocity.linear, lane);
			linear.x += f32(positions[lane].x - value.origin.x) * simd.extract(dt, lane);
			util.vector3_wide_write_slot(&velocity.linear, lane, linear);
		}
	}
}

capture_legacy :: proc "contextless" (
	user_context: rawptr, indices: util.I32x8, positions: util.Vector3_Wide,
	orientation: util.Quaternion_Wide, inertia: p.Body_Inertia_Wide,
	mask: util.I32x8, worker: int, dt: util.F32x8, velocity: ^p.Body_Velocity_Wide,
)
{
	bundle: e.Global_Position_Bundle;
	for lane in 0 ..< 8
	{
		bundle[lane] = p.world_precision_position_from_float(util.vector3_wide_read_slot(positions, lane));
	}
	capture_global(user_context, indices, &bundle, orientation, inertia, mask, worker, dt, velocity);
}

capture_callbacks :: proc "contextless" (value: ^Capture) -> e.Pose_Callbacks
{
	return {
		initialize=capture_initialize, prepare_for_integration=capture_prepare,
		integrate_velocity_global=capture_global, dispose=capture_dispose,
		allow_substeps_for_unconstrained=.Enabled, user_context=value,
	};
}
