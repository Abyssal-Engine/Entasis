#+vet !unused-imports

package entasis_physics

import "core:math"
import util "entasis:entasis_utilities"

when util.PRECISION_MODE == .Double_Coordinates
{
	// closed finite segment in the simulation's current coordinate frame. the
	// dominant direction component is exactly +/-1 in the local float ray, its
	// parameter is distance along that axis, not a rounded global fraction
	World_Precision_Ray :: struct
	{
		segment: World_Precision_Prepared_Segment,
		direction: [3]f64,
		length: f64,
		axis: int,
	}

	World_Precision_Ray_Origin :: enum u8
	{
		Clipped,
		Segment_Start,
	}

	World_Precision_Local_Ray :: struct
	{
		ray: Tree_Ray,
		position: Global_Position,
		start_axis: f64,
		distance: f64,
		axis: int,
		sign: f64,
		origin_source: World_Precision_Ray_Origin,
	}

	world_precision_ray_prepare :: proc "contextless" (
		segment: World_Precision_Segment,
	) -> (World_Precision_Ray, Physics_Status)
	{
		prepared: World_Precision_Prepared_Segment;
		status: Physics_Status;
		prepared, status = world_precision_segment_prepare(segment);
		if status != .Ok
		{
			return {}, status;
		}
		result: World_Precision_Ray = {segment=prepared};
		for axis in 0 ..< 3
		{
			result.direction[axis] = prepared.end[axis] - prepared.start[axis];
			if abs(result.direction[axis]) > result.length
			{
				result.length = abs(result.direction[axis]);
				result.axis = axis;
			}
		}
		if result.length == 0
		{
			return {}, .Invalid_Argument;
		}
		for axis in 0 ..< 3
		{
			result.direction[axis] /= result.length;
		}
		return result, .Ok;
	}

	// first move the line's reference near the target in f64, then clip against
	// the target's rotated local AABB. a 1 Gm ray is never sent to an f32 kernel
	// numerical mode: tolerant, IEEE f64 setup followed by existing local f32
	// geometry. no fast-math reassociation or absolute-position-scaled epsilon
	world_precision_ray_localize :: proc "contextless" (
		prepared: ^World_Precision_Ray, position: Global_Position, bounds: Shape_Bounds,
	) -> (World_Precision_Local_Ray, Reference_State, Physics_Status)
	{
		center: [3]f64 = {position.x, position.y, position.z};
		offset, residue: [3]f64;
		for axis in 0 ..< 3
		{
			offset[axis], residue[axis] = world_precision_sum_residue(prepared.segment.start[axis], -center[axis]);
		}
		sign: f64 = prepared.direction[prepared.axis];
		pivot: f64 = clamp(-offset[prepared.axis] * sign, 0, prepared.length);
		for axis in 0 ..< 3
		{
			// retain target displacement lost by subtraction from remote endpoints
			// existing two-sum also retains the cancellation residue of this shift
			shifted, shift_residue: f64;
			shifted, shift_residue = world_precision_sum_residue(offset[axis], prepared.direction[axis] * pivot);
			offset[axis] = shifted + (residue[axis] + shift_residue);
		}
		minimum: [3]f64 = {f64(bounds.min.x), f64(bounds.min.y), f64(bounds.min.z)};
		maximum: [3]f64 = {f64(bounds.max.x), f64(bounds.max.y), f64(bounds.max.z)};
		enter: f64 = -pivot;
		exit: f64 = prepared.length - pivot;
		for axis in 0 ..< 3
		{
			direction: f64 = prepared.direction[axis];
			if direction == 0
			{
				if offset[axis] < minimum[axis] || offset[axis] > maximum[axis]
				{
					return {}, .Missing, .Ok;
				}
				continue;
			}
			a: f64 = (minimum[axis] - offset[axis]) / direction;
			b: f64 = (maximum[axis] - offset[axis]) / direction;
			// outward rounding protects the closed clipping interval. the high
			// collector separately rejects hits outside the original segment
			enter = max(enter, math.nextafter_f64(min(a, b), math.inf_f64(-1)));
			exit = min(exit, math.nextafter_f64(max(a, b), math.inf_f64(1)));
			if enter > exit
			{
				return {}, .Missing, .Ok;
			}
		}
		origin: Global_Position = {
			offset[0] + prepared.direction[0] * enter,
			offset[1] + prepared.direction[1] * enter,
			offset[2] + prepared.direction[2] * enter,
		};
		span: f64 = exit - enter;
		if world_precision_position_validate(origin) != .Ok || !(span >= 0 && span <= f64(f32(math.F32_MAX)))
		{
			return {}, .Missing, .Invalid_Argument;
		}
		result: World_Precision_Local_Ray = {
			ray={
				origin=world_precision_position_cache(origin),
				direction={f32(prepared.direction[0]), f32(prepared.direction[1]), f32(prepared.direction[2])},
				maximum_t=world_precision_max_to_float(span),
			},
			position=position,
			start_axis=prepared.segment.start[prepared.axis],
			distance=prepared.length,
			axis=prepared.axis,
			sign=sign,
			origin_source=.Segment_Start if enter == -pivot else .Clipped,
		};
		return result, .Present, .Ok;
	}
}
