// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_utilities

import "base:intrinsics"
import "core:math"

CHECKMATH :: #config(CHECKMATH, 0);

math_check_f32 :: proc "contextless" (value: f32) -> Math_Check_Status
{
	if math.is_nan(value) || math.is_inf(value, 0)
	{
		return .Non_Finite;
	}
	return .Ok;
}

math_validate_f32 :: proc "contextless" (value: f32)
{
	when CHECKMATH != 0
	{
		if math_check_f32(value) != .Ok
		{
			intrinsics.debug_trap();
		}
	}
}

math_validate_vector2 :: proc "contextless" (value: Vector2)
{
	when CHECKMATH != 0
	{
		if math_check_f32(vector2_length_squared(value)) != .Ok
		{
			intrinsics.debug_trap();
		}
	}
}

math_validate_vector3 :: proc "contextless" (value: Vector3)
{
	when CHECKMATH != 0
	{
		if math_check_f32(vector3_length_squared(value)) != .Ok
		{
			intrinsics.debug_trap();
		}
	}
}

math_validate_vector4 :: proc "contextless" (value: Vector4)
{
	when CHECKMATH != 0
	{
		if math_check_f32(vector4_length_squared(value)) != .Ok
		{
			intrinsics.debug_trap();
		}
	}
}

math_validate_matrix3x3 :: proc "contextless" (value: Matrix3x3)
{
	when CHECKMATH != 0
	{
		math_validate_vector3(value.x);
		math_validate_vector3(value.y);
		math_validate_vector3(value.z);
	}
}

math_validate_matrix :: proc "contextless" (value: Matrix)
{
	when CHECKMATH != 0
	{
		math_validate_vector4(value.x);
		math_validate_vector4(value.y);
		math_validate_vector4(value.z);
		math_validate_vector4(value.w);
	}
}

math_validate_quaternion :: proc "contextless" (value: Quaternion)
{
	when CHECKMATH != 0
	{
		if math_check_f32(quaternion_length_squared(value)) != .Ok
		{
			intrinsics.debug_trap();
		}
	}
}

math_validate_orientation :: proc "contextless" (value: Quaternion)
{
	when CHECKMATH != 0
	{
		length_squared := quaternion_length_squared(value);
		if math_check_f32(length_squared) != .Ok && abs(1 - length_squared) < 1e-5
		{
			intrinsics.debug_trap();
		}
	}
}

math_validate_symmetric3x3 :: proc "contextless" (value: Symmetric3x3)
{
	when CHECKMATH != 0
	{
		math_validate_f32(value.xx);
		math_validate_f32(value.yx);
		math_validate_f32(value.yy);
		math_validate_f32(value.zx);
		math_validate_f32(value.zy);
		math_validate_f32(value.zz);
	}
}

math_validate_affine :: proc "contextless" (value: Affine_Transform)
{
	when CHECKMATH != 0
	{
		math_validate_matrix3x3(value.linear_transform);
		math_validate_vector3(value.translation);
	}
}

math_validate_bounding_box :: proc "contextless" (value: Bounding_Box)
{
	when CHECKMATH != 0
	{
		math_validate_vector3(value.min);
		math_validate_vector3(value.max);
	}
}

math_validate_bounding_sphere :: proc "contextless" (value: Bounding_Sphere)
{
	when CHECKMATH != 0
	{
		math_validate_vector3(value.center);
		math_validate_f32(value.radius);
	}
}

math_validate_f32x8 :: proc "contextless" (value: F32x8, lane_count: int = -1)
{
	when CHECKMATH != 0
	{
		count := lane_count;
		if count < -1 || count > PRODUCTION_LANE_COUNT
		{
			intrinsics.debug_trap();
			return;
		}
		if count == -1
		{
			count = PRODUCTION_LANE_COUNT;
		}
		lanes := transmute([PRODUCTION_LANE_COUNT]f32)value;
		for lane in 0 ..< count
		{
			if math_check_f32(lanes[lane]) != .Ok
			{
				intrinsics.debug_trap();
			}
		}
	}
}

math_validate_f32x8_masked :: proc "contextless" (value: F32x8, lane_mask: I32x8)
{
	when CHECKMATH != 0
	{
		lanes := transmute([PRODUCTION_LANE_COUNT]f32)value;
		masks := transmute([PRODUCTION_LANE_COUNT]i32)lane_mask;
		for lane in 0 ..< PRODUCTION_LANE_COUNT
		{
			if masks[lane] != 0 && math_check_f32(lanes[lane]) != .Ok
			{
				intrinsics.debug_trap();
			}
		}
	}
}

math_validate_vector2_wide :: proc "contextless" (value: Vector2_Wide, lane_count: int = -1)
{
	when CHECKMATH != 0
	{
		math_validate_f32x8(value.x, lane_count);
		math_validate_f32x8(value.y, lane_count);
	}
}

math_validate_vector3_wide :: proc "contextless" (value: Vector3_Wide, lane_count: int = -1)
{
	when CHECKMATH != 0
	{
		math_validate_f32x8(value.x, lane_count);
		math_validate_f32x8(value.y, lane_count);
		math_validate_f32x8(value.z, lane_count);
	}
}

math_validate_matrix2x2_wide :: proc "contextless" (value: Matrix2x2_Wide, lane_count: int = -1)
{
	when CHECKMATH != 0
	{
		math_validate_vector2_wide(value.x, lane_count);
		math_validate_vector2_wide(value.y, lane_count);
	}
}

math_validate_matrix2x3_wide :: proc "contextless" (value: Matrix2x3_Wide, lane_count: int = -1)
{
	when CHECKMATH != 0
	{
		math_validate_vector3_wide(value.x, lane_count);
		math_validate_vector3_wide(value.y, lane_count);
	}
}

math_validate_matrix3x3_wide :: proc "contextless" (value: Matrix3x3_Wide, lane_count: int = -1)
{
	when CHECKMATH != 0
	{
		math_validate_vector3_wide(value.x, lane_count);
		math_validate_vector3_wide(value.y, lane_count);
		math_validate_vector3_wide(value.z, lane_count);
	}
}

math_validate_quaternion_wide :: proc "contextless" (value: Quaternion_Wide, lane_count: int = -1)
{
	when CHECKMATH != 0
	{
		math_validate_f32x8(value.x, lane_count);
		math_validate_f32x8(value.y, lane_count);
		math_validate_f32x8(value.z, lane_count);
		math_validate_f32x8(value.w, lane_count);
	}
}

math_validate_symmetric3x3_wide :: proc "contextless" (value: Symmetric3x3_Wide, lane_count: int = -1)
{
	when CHECKMATH != 0
	{
		math_validate_f32x8(value.xx, lane_count);
		math_validate_f32x8(value.yx, lane_count);
		math_validate_f32x8(value.yy, lane_count);
		math_validate_f32x8(value.zx, lane_count);
		math_validate_f32x8(value.zy, lane_count);
		math_validate_f32x8(value.zz, lane_count);
	}
}

math_validate_symmetric6x6_wide :: proc "contextless" (value: Symmetric6x6_Wide, lane_count: int = -1)
{
	when CHECKMATH != 0
	{
		math_validate_symmetric3x3_wide(value.a, lane_count);
		math_validate_matrix3x3_wide(value.b, lane_count);
		math_validate_symmetric3x3_wide(value.d, lane_count);
	}
}

math_validate_vector2_wide_masked :: proc "contextless" (value: Vector2_Wide, lane_mask: I32x8)
{
	when CHECKMATH != 0
	{
		math_validate_f32x8_masked(value.x, lane_mask);
		math_validate_f32x8_masked(value.y, lane_mask);
	}
}

math_validate_vector3_wide_masked :: proc "contextless" (value: Vector3_Wide, lane_mask: I32x8)
{
	when CHECKMATH != 0
	{
		math_validate_f32x8_masked(value.x, lane_mask);
		math_validate_f32x8_masked(value.y, lane_mask);
		math_validate_f32x8_masked(value.z, lane_mask);
	}
}

math_validate_matrix2x2_wide_masked :: proc "contextless" (value: Matrix2x2_Wide, lane_mask: I32x8)
{
	when CHECKMATH != 0
	{
		math_validate_vector2_wide_masked(value.x, lane_mask);
		math_validate_vector2_wide_masked(value.y, lane_mask);
	}
}

math_validate_matrix2x3_wide_masked :: proc "contextless" (value: Matrix2x3_Wide, lane_mask: I32x8)
{
	when CHECKMATH != 0
	{
		math_validate_vector3_wide_masked(value.x, lane_mask);
		math_validate_vector3_wide_masked(value.y, lane_mask);
	}
}

math_validate_matrix3x3_wide_masked :: proc "contextless" (value: Matrix3x3_Wide, lane_mask: I32x8)
{
	when CHECKMATH != 0
	{
		math_validate_vector3_wide_masked(value.x, lane_mask);
		math_validate_vector3_wide_masked(value.y, lane_mask);
		math_validate_vector3_wide_masked(value.z, lane_mask);
	}
}

math_validate_quaternion_wide_masked :: proc "contextless" (value: Quaternion_Wide, lane_mask: I32x8)
{
	when CHECKMATH != 0
	{
		math_validate_f32x8_masked(value.x, lane_mask);
		math_validate_f32x8_masked(value.y, lane_mask);
		math_validate_f32x8_masked(value.z, lane_mask);
		math_validate_f32x8_masked(value.w, lane_mask);
	}
}

math_validate_symmetric3x3_wide_masked :: proc "contextless" (value: Symmetric3x3_Wide, lane_mask: I32x8)
{
	when CHECKMATH != 0
	{
		math_validate_f32x8_masked(value.xx, lane_mask);
		math_validate_f32x8_masked(value.yx, lane_mask);
		math_validate_f32x8_masked(value.yy, lane_mask);
		math_validate_f32x8_masked(value.zx, lane_mask);
		math_validate_f32x8_masked(value.zy, lane_mask);
		math_validate_f32x8_masked(value.zz, lane_mask);
	}
}

math_validate_symmetric6x6_wide_masked :: proc "contextless" (value: Symmetric6x6_Wide, lane_mask: I32x8)
{
	when CHECKMATH != 0
	{
		math_validate_symmetric3x3_wide_masked(value.a, lane_mask);
		math_validate_matrix3x3_wide_masked(value.b, lane_mask);
		math_validate_symmetric3x3_wide_masked(value.d, lane_mask);
	}
}
