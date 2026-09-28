package utilities_math_tests

import "core:math"
import "core:simd"
import "core:testing"
import entasis "entasis:entasis_utilities"

expect_f32 :: proc(t: ^testing.T, actual, expected, tolerance: f32, label: string)
{
	difference := abs(actual - expected);
	testing.expectf(
		t,
		difference <= tolerance,
		"%s: expected %.9f, got %.9f, difference %.9f",
		label,
		expected,
		actual,
		difference
	);
}

expect_vector3 :: proc(t: ^testing.T, actual, expected: entasis.Vector3, tolerance: f32, label: string)
{
	expect_f32(t, actual.x, expected.x, tolerance, label);
	expect_f32(t, actual.y, expected.y, tolerance, label);
	expect_f32(t, actual.z, expected.z, tolerance, label);
}

expect_vector2 :: proc(t: ^testing.T, actual, expected: entasis.Vector2, tolerance: f32, label: string)
{
	expect_f32(t, actual.x, expected.x, tolerance, label);
	expect_f32(t, actual.y, expected.y, tolerance, label);
}

expect_vector4 :: proc(t: ^testing.T, actual, expected: entasis.Vector4, tolerance: f32, label: string)
{
	expect_f32(t, actual.x, expected.x, tolerance, label);
	expect_f32(t, actual.y, expected.y, tolerance, label);
	expect_f32(t, actual.z, expected.z, tolerance, label);
	expect_f32(t, actual.w, expected.w, tolerance, label);
}

expect_matrix3x3 :: proc(t: ^testing.T, actual, expected: entasis.Matrix3x3, tolerance: f32, label: string)
{
	expect_vector3(t, actual.x, expected.x, tolerance, label);
	expect_vector3(t, actual.y, expected.y, tolerance, label);
	expect_vector3(t, actual.z, expected.z, tolerance, label);
}

expect_matrix4 :: proc(t: ^testing.T, actual, expected: entasis.Matrix, tolerance: f32, label: string)
{
	actual_values := [16]f32{
		actual.x.x,
		actual.x.y,
		actual.x.z,
		actual.x.w,
		actual.y.x,
		actual.y.y,
		actual.y.z,
		actual.y.w,
		actual.z.x,
		actual.z.y,
		actual.z.z,
		actual.z.w,
		actual.w.x,
		actual.w.y,
		actual.w.z,
		actual.w.w,
	};
	expected_values := [16]f32{
		expected.x.x,
		expected.x.y,
		expected.x.z,
		expected.x.w,
		expected.y.x,
		expected.y.y,
		expected.y.z,
		expected.y.w,
		expected.z.x,
		expected.z.y,
		expected.z.z,
		expected.z.w,
		expected.w.x,
		expected.w.y,
		expected.w.z,
		expected.w.w,
	};
	for value, index in actual_values
	{
		expect_f32(t, value, expected_values[index], tolerance, label);
	}
}

next_random :: proc(state: ^u32) -> f32
{
	state^ = state^ * 1664525 + 1013904223;
	return (f32(state^ >> 8) / f32(0x00ff_ffff) - 0.5) * 4;
}

wide_sequence :: proc(base, step: f32) -> entasis.F32x8
{
	values: entasis.F32x8;
	for lane in 0 ..< entasis.PRODUCTION_LANE_COUNT
	{
		values = simd.replace(values, lane, base + step * f32(lane));
	}
	return values;
}

@(test)
approximate_math_scalar_and_wide_match :: proc(t: ^testing.T)
{
	values := [8]f32{
		-2 * entasis.PI,
		-entasis.PI,
		-entasis.PI_OVER_2,
		-0.25,
		0,
		entasis.PI_OVER_4,
		entasis.PI,
		2 * entasis.PI,
	};
	wide_values := simd.from_array(values);
	wide_cos := entasis.cos_approx_wide(wide_values);
	wide_sin := entasis.sin_approx_wide(wide_values);
	for value, lane in values
	{
		expect_f32(t, entasis.cos_approx(value), f32(math.cos(f64(value))), 8e-7, "scalar cosine");
		expect_f32(t, entasis.sin_approx(value), f32(math.sin(f64(value))), 8e-7, "scalar sine");
		expect_f32(t, simd.extract(wide_cos, lane), entasis.cos_approx(value), 2e-6, "wide cosine");
		expect_f32(t, simd.extract(wide_sin, lane), entasis.sin_approx(value), 2e-6, "wide sine");
	}
	acos_inputs := entasis.F32x8{-1, -0.75, -0.25, 0, 0.25, 0.75, 1, 1.25};
	wide_acos := entasis.acos_approx_wide(acos_inputs);
	for lane in 0 ..< entasis.PRODUCTION_LANE_COUNT
	{
		value := simd.extract(acos_inputs, lane);
		expect_f32(t, simd.extract(wide_acos, lane), entasis.acos_approx(value), 2e-6, "wide acos");
	}
}

@(test)
scalar_matrix_quaternion_and_affine_round_trips :: proc(t: ^testing.T)
{
	axis := entasis.vector3_normalize(entasis.Vector3{1, -2, 3});
	rotation := entasis.quaternion_from_axis_angle(axis, 0.73);
	entasis.math_validate_quaternion(rotation);
	entasis.math_validate_orientation(rotation);
	matrix3 := entasis.matrix3x3_from_quaternion(rotation);
	expect_matrix3x3(
		t,
		entasis.matrix3x3_multiply(matrix3, entasis.matrix3x3_invert(matrix3)),
		entasis.matrix3x3_identity(),
		2e-5,
		"matrix3 inverse"
	);
	vector := entasis.Vector3{2.5, -4, 1.25};
	expect_vector3(
		t,
		entasis.matrix3x3_transform(vector, matrix3),
		entasis.quaternion_transform(vector, rotation),
		2e-5,
		"quaternion matrix agreement"
	);
	from_matrix := entasis.quaternion_from_rotation_matrix(matrix3);
	expect_f32(
		t,
		abs(rotation.x * from_matrix.x + rotation.y * from_matrix.y + rotation.z * from_matrix.z + rotation.w * from_matrix.w),
		1,
		2e-5,
		"rotation matrix quaternion"
	);
	from_matrix4 := entasis.quaternion_from_matrix(entasis.matrix_from_matrix3x3(matrix3));
	expect_f32(
		t,
		abs(rotation.x * from_matrix4.x + rotation.y * from_matrix4.y + rotation.z * from_matrix4.z + rotation.w * from_matrix4.w),
		1,
		2e-5,
		"matrix4 quaternion"
	);

	affine := entasis.affine_from_scale_orientation_translation(
		entasis.Vector3{2, 3, 4},
		rotation,
		entasis.Vector3{7, -5, 11}
	);
	entasis.math_validate_affine(affine);
	restored := entasis.affine_transform_position(
		entasis.affine_transform_position(vector, affine),
		entasis.affine_invert(affine)
	);
	expect_vector3(t, restored, vector, 3e-5, "affine inverse");

	matrix4 := entasis.matrix_create_rigid(matrix3, entasis.Vector3{4, -2, 8});
	expect_matrix4(
		t,
		entasis.matrix_multiply(matrix4, entasis.matrix_invert(matrix4)),
		entasis.matrix_identity(),
		3e-5,
		"matrix4 inverse"
	);
}

@(test)
symmetric_bounds_and_math_diagnostics :: proc(t: ^testing.T)
{
	symmetric := entasis.Symmetric3x3{4, 0.25, 5, -0.5, 0.75, 6};
	testing.expect_value(t, entasis.math_check_f32(3.5), entasis.Math_Check_Status.Ok);
	testing.expect_value(t, entasis.math_check_f32(math.inf_f32(1)), entasis.Math_Check_Status.Non_Finite);
	entasis.math_validate_symmetric3x3(symmetric);
	inverse := entasis.symmetric3x3_invert(symmetric);
	expect_matrix3x3(
		t,
		entasis.matrix3x3_multiply(entasis.symmetric3x3_as_matrix(symmetric), entasis.symmetric3x3_as_matrix(inverse)),
		entasis.matrix3x3_identity(),
		2e-5,
		"symmetric inverse"
	);

	points := [4]entasis.Vector3{{-2, 3, 1}, {4, -5, 2}, {1, 8, -7}, {0, 0, 0}};
	box, status := entasis.bounding_box_from_points(points[:]);
	testing.expect_value(t, status, entasis.Memory_Status.Ok);
	entasis.math_validate_bounding_box(box);
	entasis.math_validate_bounding_sphere(entasis.Bounding_Sphere{center={5, 0, 0}, radius=1});
	expect_vector3(t, box.min, entasis.Vector3{-2, -5, -7}, 0, "bounding minimum");
	expect_vector3(t, box.max, entasis.Vector3{4, 8, 2}, 0, "bounding maximum");
	testing.expect_value(
		t,
		entasis.bounding_box_contains(box, entasis.Bounding_Box{min={-1, -1, -1}, max={1, 1, 1}}),
		entasis.Containment_Type.Contains
	);
	testing.expect_value(
		t,
		entasis.bounding_box_intersection_state(box, entasis.Bounding_Box{min={10, 10, 10}, max={12, 12, 12}}),
		entasis.Intersection_State.Separate
	);
	testing.expect_value(
		t,
		entasis.bounding_box_intersects_sphere(box, entasis.Bounding_Sphere{center={5, 0, 0}, radius=1}),
		entasis.Intersection_State.Intersecting
	);

	wide_check := entasis.Vector3_Wide{x=entasis.F32x8(1), y=entasis.F32x8(2), z=entasis.F32x8(3)};
	entasis.math_validate_vector3_wide(wide_check);
	entasis.math_validate_vector3_wide_masked(wide_check, entasis.I32x8(-1));
}

@(test)
randomized_wide_lanes_match_scalar_operations :: proc(t: ^testing.T)
{
	state := u32(0x58a3_91c7);
	for _ in 0 ..< 64
	{
		vectors: [8]entasis.Vector3;
		quaternions: [8]entasis.Quaternion;
		matrices: [8]entasis.Matrix3x3;
		symmetric: [8]entasis.Symmetric3x3;
		wide_vectors: entasis.Vector3_Wide;
		wide_quaternions: entasis.Quaternion_Wide;
		wide_matrices: entasis.Matrix3x3_Wide;
		wide_symmetric: entasis.Symmetric3x3_Wide;
		for lane in 0 ..< entasis.PRODUCTION_LANE_COUNT
		{
			vectors[lane] = {next_random(&state), next_random(&state), next_random(&state)};
			axis := entasis.vector3_normalize(entasis.Vector3{
					next_random(&state) + 0.25,
					next_random(&state) - 0.5,
					next_random(&state) + 1
				});
			quaternions[lane] = entasis.quaternion_from_axis_angle(axis, next_random(&state));
			matrices[lane] = entasis.matrix3x3_from_quaternion(quaternions[lane]);
			diagonal := f32(lane + 2);
			symmetric[lane] = {diagonal + 1, 0.05, diagonal + 2, -0.075, 0.1, diagonal + 3};
			entasis.vector3_wide_write_slot(&wide_vectors, lane, vectors[lane]);
			entasis.quaternion_wide_write_slot(&wide_quaternions, lane, quaternions[lane]);
			entasis.vector3_wide_write_slot(&wide_matrices.x, lane, matrices[lane].x);
			entasis.vector3_wide_write_slot(&wide_matrices.y, lane, matrices[lane].y);
			entasis.vector3_wide_write_slot(&wide_matrices.z, lane, matrices[lane].z);
			entasis.symmetric3x3_wide_write_slot(&wide_symmetric, lane, symmetric[lane]);
		}
		wide_rotated := entasis.quaternion_wide_transform(wide_vectors, wide_quaternions);
		wide_matrix_rotated := entasis.matrix3x3_wide_transform(wide_vectors, wide_matrices);
		wide_inverse := entasis.matrix3x3_wide_invert(wide_matrices);
		wide_symmetric_inverse := entasis.symmetric3x3_wide_invert(wide_symmetric);
		wide_axes, wide_angles := entasis.quaternion_wide_axis_angle(wide_quaternions);
		wide_unit_x, wide_unit_y := entasis.quaternion_wide_transform_unit_xy(wide_quaternions);
		wide_unit_x_again, wide_unit_z := entasis.quaternion_wide_transform_unit_xz(wide_quaternions);
		for lane in 0 ..< entasis.PRODUCTION_LANE_COUNT
		{
			expect_vector3(
				t,
				entasis.vector3_wide_read_slot(wide_rotated, lane),
				entasis.quaternion_transform(vectors[lane], quaternions[lane]),
				4e-5,
				"wide quaternion transform"
			);
			expect_vector3(
				t,
				entasis.vector3_wide_read_slot(wide_matrix_rotated, lane),
				entasis.matrix3x3_transform(vectors[lane], matrices[lane]),
				4e-5,
				"wide matrix transform"
			);
			expect_matrix3x3(
				t,
				entasis.matrix3x3_wide_read_slot(wide_inverse, lane),
				entasis.matrix3x3_invert(matrices[lane]),
				8e-5,
				"wide matrix inverse"
			);
			actual_symmetric := entasis.Symmetric3x3{
				simd.extract(
					wide_symmetric_inverse.xx,
					lane
				), simd.extract(wide_symmetric_inverse.yx, lane), simd.extract(wide_symmetric_inverse.yy, lane),
				simd.extract(
					wide_symmetric_inverse.zx,
					lane
				), simd.extract(wide_symmetric_inverse.zy, lane), simd.extract(wide_symmetric_inverse.zz, lane),
			};
			expected_symmetric := entasis.symmetric3x3_invert(symmetric[lane]);
			expect_matrix3x3(
				t,
				entasis.symmetric3x3_as_matrix(actual_symmetric),
				entasis.symmetric3x3_as_matrix(expected_symmetric),
				3e-5,
				"wide symmetric inverse"
			);
			expected_axis, expected_angle := entasis.quaternion_axis_angle(quaternions[lane]);
			expect_vector3(
				t,
				entasis.vector3_wide_read_slot(wide_axes, lane),
				expected_axis,
				3e-5,
				"wide quaternion axis"
			);
			expect_f32(t, simd.extract(wide_angles, lane), expected_angle, 3e-5, "wide quaternion angle");
			expect_vector3(
				t,
				entasis.vector3_wide_read_slot(wide_unit_x, lane),
				entasis.quaternion_transform_unit_x(quaternions[lane]),
				3e-5,
				"wide unit x"
			);
			expect_vector3(
				t,
				entasis.vector3_wide_read_slot(wide_unit_x_again, lane),
				entasis.quaternion_transform_unit_x(quaternions[lane]),
				3e-5,
				"wide unit xz x"
			);
			expect_vector3(
				t,
				entasis.vector3_wide_read_slot(wide_unit_y, lane),
				entasis.quaternion_transform_unit_y(quaternions[lane]),
				3e-5,
				"wide unit y"
			);
			expect_vector3(
				t,
				entasis.vector3_wide_read_slot(wide_unit_z, lane),
				entasis.quaternion_transform_unit_z(quaternions[lane]),
				3e-5,
				"wide unit z"
			);
		}
	}
}

@(test)
large_symmetric_wide_coupled_inversion_and_ldlt :: proc(t: ^testing.T)
{
	matrix4 := entasis.Symmetric4x4_Wide{
		xx=wide_sequence(5, 0.15), yx=wide_sequence(0.2, 0.01), yy=wide_sequence(6, 0.12),
		zx=wide_sequence(-0.15, 0.005), zy=wide_sequence(0.1, -0.004), zz=wide_sequence(7, 0.1),
		wx=wide_sequence(0.05, 0.003), wy=wide_sequence(
			-0.08,
			0.004
		), wz=wide_sequence(0.12, -0.003), ww=wide_sequence(8, 0.08),
	};
	input4 := entasis.Vector4_Wide{
		x=wide_sequence(2, 0.25),
		y=wide_sequence(6, -0.15),
		z=wide_sequence(12, 0.2),
		w=wide_sequence(20, -0.1),
	};
	solved4 := entasis.symmetric4x4_wide_transform(input4, entasis.symmetric4x4_wide_invert(matrix4));
	result4 := entasis.symmetric4x4_wide_transform(solved4, matrix4);
	for lane in 0 ..< entasis.PRODUCTION_LANE_COUNT
	{
		expect_vector4(
			t,
			entasis.vector4_wide_read_slot(result4, lane),
			entasis.vector4_wide_read_slot(input4, lane),
			2e-4,
			"symmetric4 coupled inverse"
		);
	}

	a := entasis.Symmetric3x3_Wide{
		xx=wide_sequence(6, 0.1), yx=wide_sequence(0.3, 0.006), yy=wide_sequence(7, 0.1),
		zx=wide_sequence(-0.2, 0.004), zy=wide_sequence(0.25, -0.005), zz=wide_sequence(8, 0.1),
	};
	b2 := entasis.Matrix2x3_Wide{
		x={wide_sequence(0.2, 0.003), wide_sequence(-0.1, 0.002), wide_sequence(0.15, -0.002)},
		y={wide_sequence(-0.12, 0.002), wide_sequence(0.18, -0.003), wide_sequence(0.08, 0.002)},
	};
	d2 := entasis.Symmetric2x2_Wide{
		xx=wide_sequence(9, 0.08),
		yx=wide_sequence(0.22, -0.004),
		yy=wide_sequence(10, 0.08),
	};
	matrix5 := entasis.Symmetric5x5_Wide{a=a, b=b2, d=d2};
	input5_0 := entasis.Vector3_Wide{x=wide_sequence(2, 0.2), y=wide_sequence(6, -0.1), z=wide_sequence(12, 0.15)};
	input5_1 := entasis.Vector2_Wide{x=wide_sequence(20, 0.3), y=wide_sequence(30, -0.25)};
	solved5_0, solved5_1 := entasis.symmetric5x5_wide_transform(
		input5_0,
		input5_1,
		entasis.symmetric5x5_wide_invert(matrix5)
	);
	result5_0, result5_1 := entasis.symmetric5x5_wide_transform(solved5_0, solved5_1, matrix5);
	for lane in 0 ..< entasis.PRODUCTION_LANE_COUNT
	{
		expect_vector3(
			t,
			entasis.vector3_wide_read_slot(result5_0, lane),
			entasis.vector3_wide_read_slot(input5_0, lane),
			3e-4,
			"symmetric5 coupled inverse a"
		);
		expect_vector2(
			t,
			entasis.vector2_wide_read_slot(result5_1, lane),
			entasis.vector2_wide_read_slot(input5_1, lane),
			3e-4,
			"symmetric5 coupled inverse d"
		);
	}

	d := entasis.Symmetric3x3_Wide{
		xx=wide_sequence(11, 0.09), yx=wide_sequence(0.15, 0.003), yy=wide_sequence(12, 0.09),
		zx=wide_sequence(-0.12, 0.002), zy=wide_sequence(0.17, -0.002), zz=wide_sequence(13, 0.09),
	};
	b := entasis.Matrix3x3_Wide{
		x={wide_sequence(0.2, 0.002), wide_sequence(-0.1, 0.001), wide_sequence(0.08, -0.001)},
		y={wide_sequence(0.12, -0.001), wide_sequence(0.18, 0.002), wide_sequence(-0.07, 0.001)},
		z={wide_sequence(-0.09, 0.001), wide_sequence(0.11, -0.001), wide_sequence(0.16, 0.002)},
	};
	matrix6 := entasis.Symmetric6x6_Wide{a=a, b=b, d=d};
	v0 := entasis.Vector3_Wide{x=wide_sequence(2, 0.2), y=wide_sequence(6, -0.12), z=wide_sequence(12, 0.18)};
	v1 := entasis.Vector3_Wide{x=wide_sequence(20, 0.3), y=wide_sequence(30, -0.2), z=wide_sequence(42, 0.25)};
	inverse0, inverse1 := entasis.symmetric6x6_wide_transform(v0, v1, entasis.symmetric6x6_wide_invert(matrix6));
	solved0, solved1 := entasis.symmetric6x6_wide_ldlt_solve(v0, v1, a, b, d);
	result0, result1 := entasis.symmetric6x6_wide_transform(solved0, solved1, matrix6);
	for lane in 0 ..< entasis.PRODUCTION_LANE_COUNT
	{
		expect_vector3(
			t,
			entasis.vector3_wide_read_slot(solved0, lane),
			entasis.vector3_wide_read_slot(inverse0, lane),
			4e-4,
			"symmetric6 coupled ldlt inverse a"
		);
		expect_vector3(
			t,
			entasis.vector3_wide_read_slot(solved1, lane),
			entasis.vector3_wide_read_slot(inverse1, lane),
			4e-4,
			"symmetric6 coupled ldlt inverse d"
		);
		expect_vector3(
			t,
			entasis.vector3_wide_read_slot(result0, lane),
			entasis.vector3_wide_read_slot(v0, lane),
			5e-4,
			"symmetric6 coupled ldlt round trip a"
		);
		expect_vector3(
			t,
			entasis.vector3_wide_read_slot(result1, lane),
			entasis.vector3_wide_read_slot(v1, lane),
			5e-4,
			"symmetric6 coupled ldlt round trip d"
		);
	}
}
