package entasis

import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Precision_Mode :: util.Precision_Mode;
PRECISION_MODE :: util.PRECISION_MODE;
PRECISION_NAME :: util.PRECISION_NAME;

// frame conversion is available in both compile-time precision profiles
Global_Position       :: physics.Global_Position;
Origin_Frame :: physics.Origin_Frame;

// global_position constructs an application-space f64 position
global_position :: #force_inline proc "contextless" (x, y, z: f64) -> Global_Position
{
	return {x, y, z};
}

// origin_frame_at constructs an origin frame after finite validation
origin_frame_at :: #force_inline proc "contextless" (
	origin: Global_Position = {},
) -> (Origin_Frame, Status)
{
	return physics.origin_frame_create(origin);
}

// origin_frame_to_local subtracts the f64 frame origin before narrowing to
// the engine local f32 coordinate type
origin_frame_to_local :: #force_inline proc "contextless" (
	frame: Origin_Frame,
	global: Global_Position,
) -> (Vector3, Status)
{
	return physics.origin_frame_to_local(frame, global);
}

// origin_frame_to_global reconstructs application coordinates in f64
origin_frame_to_global :: #force_inline proc "contextless" (
	frame: Origin_Frame,
	local: Vector3,
) -> (Global_Position, Status)
{
	return physics.origin_frame_to_global(frame, local);
}

// global_position_delta forms a local displacement in f64 before narrowing
global_position_delta :: #force_inline proc "contextless" (
	from, to: Global_Position,
) -> (Vector3, Status)
{
	return physics.global_position_delta(from, to);
}

// origin_frame_translate advances caller-owned application state
// without mutating a world
origin_frame_translate :: #force_inline proc "contextless" (
	frame: ^Origin_Frame,
	delta: Global_Position,
) -> Status
{
	return physics.origin_frame_translate(frame, delta);
}

// world_shift_origin subtracts one finite local delta from every live
// absolute world-space value and advances the caller-owned f64 origin by the
// same representable delta after the simulation commit succeeds
world_shift_origin :: proc "contextless" (
	world: ^World,
	frame: ^Origin_Frame,
	delta: Vector3,
) -> Status
{
	data: ^world_data = world_data_get(world);
	if data == nil
	{
		return .Disposed;
	}
	if data.simulation.state != .Ready
	{
		return .Invalid_Argument;
	}
	if frame == nil
	{
		return .Invalid_Argument;
	}
	next_frame: Origin_Frame = frame^;
	status: Status = physics.origin_frame_translate(
		&next_frame,
		{x=f64(delta.x), y=f64(delta.y), z=f64(delta.z)},
	);
	if status != .Ok
	{
		return status;
	}
	status = physics.simulation_translate_origin(&data.simulation, delta);
	if status != .Ok
	{
		return status;
	}
	if delta != {}
	{
		frame^ = next_frame;
		world_invalidate_views_data(data);
	}
	return .Ok;
}
