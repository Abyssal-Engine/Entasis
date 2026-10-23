package entasis_physics

import "core:math"
import util "entasis:entasis_utilities"

// Global_Position stores a position in the current simulation frame
// application positions add the caller-owned Origin_Frame origin
Global_Position :: struct
{
	x: f64,
	y: f64,
	z: f64,
}

// origin_frame maps local simulation coordinates to application-owned
// global coordinates without simulation pointers or allocation
Origin_Frame :: struct
{
	origin: Global_Position,
}

global_position_finite :: #force_inline proc "contextless" (value: Global_Position) -> Reference_State
{
	if math.is_nan(value.x) || math.is_inf(value.x, 0) ||
		math.is_nan(value.y) || math.is_inf(value.y, 0) ||
		math.is_nan(value.z) || math.is_inf(value.z, 0)
	{
		return .Missing;
	}
	return .Present;
}

origin_frame_create :: #force_inline proc "contextless" (
	origin: Global_Position = {},
) -> (Origin_Frame, Physics_Status)
{
	if global_position_finite(origin) == .Missing
	{
		return {}, .Invalid_Argument;
	}
	return {origin=origin}, .Ok;
}

origin_frame_to_local :: #force_inline proc "contextless" (
	frame: Origin_Frame,
	global: Global_Position,
) -> (util.Vector3, Physics_Status)
{
	if global_position_finite(frame.origin) == .Missing ||
		global_position_finite(global) == .Missing
	{
		return {}, .Invalid_Argument;
	}
	x: f64 = global.x - frame.origin.x;
	y: f64 = global.y - frame.origin.y;
	z: f64 = global.z - frame.origin.z;
	limit: f64 = f64(max(f32));
	if math.is_nan(x) || math.is_inf(x, 0) || math.abs(x) > limit ||
		math.is_nan(y) || math.is_inf(y, 0) || math.abs(y) > limit ||
		math.is_nan(z) || math.is_inf(z, 0) || math.abs(z) > limit
	{
		return {}, .Invalid_Argument;
	}
	return {f32(x), f32(y), f32(z)}, .Ok;
}

origin_frame_to_global :: #force_inline proc "contextless" (
	frame: Origin_Frame,
	local: util.Vector3,
) -> (Global_Position, Physics_Status)
{
	if global_position_finite(frame.origin) == .Missing ||
		math.is_nan(local.x) || math.is_inf(local.x, 0) ||
		math.is_nan(local.y) || math.is_inf(local.y, 0) ||
		math.is_nan(local.z) || math.is_inf(local.z, 0)
	{
		return {}, .Invalid_Argument;
	}
	result: Global_Position = Global_Position{
		x=frame.origin.x + f64(local.x),
		y=frame.origin.y + f64(local.y),
		z=frame.origin.z + f64(local.z),
	};
	if global_position_finite(result) == .Missing
	{
		return {}, .Invalid_Argument;
	}
	return result, .Ok;
}

global_position_delta :: #force_inline proc "contextless" (
	from, to: Global_Position,
) -> (util.Vector3, Physics_Status)
{
	return origin_frame_to_local({origin=from}, to);
}

origin_frame_translate :: #force_inline proc "contextless" (
	frame: ^Origin_Frame,
	delta: Global_Position,
) -> Physics_Status
{
	if frame == nil || global_position_finite(frame.origin) == .Missing ||
		global_position_finite(delta) == .Missing
	{
		return .Invalid_Argument;
	}
	next: Global_Position = Global_Position{
		x=frame.origin.x + delta.x,
		y=frame.origin.y + delta.y,
		z=frame.origin.z + delta.z,
	};
	if global_position_finite(next) == .Missing
	{
		return .Invalid_Argument;
	}
	frame.origin = next;
	return .Ok;
}

#assert(size_of(Global_Position) == 24);
#assert(size_of(Origin_Frame) == 24);

world_precision_vector_finite :: #force_inline proc "contextless" (value: util.Vector3) -> Reference_State
{
	if math.is_nan(value.x) || math.is_inf(value.x, 0) ||
		math.is_nan(value.y) || math.is_inf(value.y, 0) ||
		math.is_nan(value.z) || math.is_inf(value.z, 0)
	{
		return .Missing;
	}
	return .Present;
}

world_precision_translate_vector :: #force_inline proc "contextless" (
	value, delta: util.Vector3,
) -> (util.Vector3, Physics_Status)
{
	if world_precision_vector_finite(value) == .Missing ||
		world_precision_vector_finite(delta) == .Missing
	{
		return {}, .Invalid_Argument;
	}
	x := f64(value.x) - f64(delta.x);
	y := f64(value.y) - f64(delta.y);
	z := f64(value.z) - f64(delta.z);
	limit := f64(math.F32_MAX);
	if math.abs(x) > limit || math.abs(y) > limit || math.abs(z) > limit
	{
		return {}, .Invalid_Argument;
	}
	return {f32(x), f32(y), f32(z)}, .Ok;
}

world_precision_next_f32_down :: #force_inline proc "contextless" (value: f32) -> f32
{
	if value == 0
	{
		return transmute(f32)u32(0x8000_0001);
	}
	bits := transmute(u32)value;
	if value > 0
	{
		bits -= 1;
	}
	else
	{
		bits += 1;
	}
	return transmute(f32)bits;
}

world_precision_next_f32_up :: #force_inline proc "contextless" (value: f32) -> f32
{
	if value == 0
	{
		return transmute(f32)u32(1);
	}
	bits := transmute(u32)value;
	if value > 0
	{
		bits += 1;
	}
	else
	{
		bits -= 1;
	}
	return transmute(f32)bits;
}

world_precision_translate_bound_scalar :: #force_inline proc "contextless" (
	value, delta: f32,
	lower: Reference_State,
) -> (f32, Physics_Status)
{
	if math.is_nan(value) || math.is_inf(value, 0) ||
		math.is_nan(delta) || math.is_inf(delta, 0)
	{
		return 0, .Invalid_Argument;
	}
	exact := f64(value) - f64(delta);
	if math.abs(exact) > f64(math.F32_MAX)
	{
		return 0, .Invalid_Argument;
	}
	translated := f32(exact);
	if lower == .Present && f64(translated) > exact
	{
		translated = world_precision_next_f32_down(translated);
	}
	else if lower == .Missing && f64(translated) < exact
	{
		translated = world_precision_next_f32_up(translated);
	}
	if math.is_nan(translated) || math.is_inf(translated, 0)
	{
		return 0, .Invalid_Argument;
	}
	return translated, .Ok;
}

world_precision_translate_bounds :: #force_inline proc "contextless" (
	bounds: util.Bounding_Box,
	delta: util.Vector3,
) -> (util.Bounding_Box, Physics_Status)
{
	min_x, min_x_status := world_precision_translate_bound_scalar(bounds.min.x, delta.x, .Present);
	if min_x_status != .Ok
	{
		return {}, min_x_status;
	}
	min_y, min_y_status := world_precision_translate_bound_scalar(bounds.min.y, delta.y, .Present);
	if min_y_status != .Ok
	{
		return {}, min_y_status;
	}
	min_z, min_z_status := world_precision_translate_bound_scalar(bounds.min.z, delta.z, .Present);
	if min_z_status != .Ok
	{
		return {}, min_z_status;
	}
	max_x, max_x_status := world_precision_translate_bound_scalar(bounds.max.x, delta.x, .Missing);
	if max_x_status != .Ok
	{
		return {}, max_x_status;
	}
	max_y, max_y_status := world_precision_translate_bound_scalar(bounds.max.y, delta.y, .Missing);
	if max_y_status != .Ok
	{
		return {}, max_y_status;
	}
	max_z, max_z_status := world_precision_translate_bound_scalar(bounds.max.z, delta.z, .Missing);
	if max_z_status != .Ok
	{
		return {}, max_z_status;
	}
	return {
		min={min_x, min_y, min_z},
		max={max_x, max_y, max_z},
	}, .Ok;
}

