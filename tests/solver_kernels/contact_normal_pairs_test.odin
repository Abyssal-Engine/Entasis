package solver_kernel_tests

import "core:math"
import "core:simd"
import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

@(test)
paired_normal_active_sets_and_scaled_admission_match_independent_oracle :: proc(t: ^testing.T)
{
	paired_normal_active_sets_and_scaled_admission_match_independent_oracle_case(t, .Serial);
	paired_normal_active_sets_and_scaled_admission_match_independent_oracle_case(t, .Parallel);
}

paired_normal_active_sets_and_scaled_admission_match_independent_oracle_case :: proc(t: ^testing.T, $pair_mode: physics.Contact_Normal_Pair_Mode)
{
	a00, a01, a11, b0, b1: util.F32x8;
	for lane: int = 0; lane < 8; lane += 1
	{
		scale: f32 = 1;
		if lane == 4
		{
			scale = 1e30;
		}
		else if lane == 5
		{
			scale = 1e-35;
		}
		left, right: f32 = -2.5, -2.5;
		if lane == 1
		{
			left, right = -2, 1;
		}
		else if lane == 2
		{
			left, right = 1, -2;
		}
		else if lane == 3
		{
			left, right = 1, 1;
		}
		a00 = simd.replace(a00, lane, 2*scale);
		a11 = simd.replace(a11, lane, 2*scale);
		a01 = simd.replace(a01, lane, 0.5*scale);
		b0 = simd.replace(b0, lane, left*scale);
		b1 = simd.replace(b1, lane, right*scale);
	}
	a00 = simd.replace(a00, 6, 1);
	a11 = simd.replace(a11, 6, 1);
	a01 = simd.replace(a01, 6, 1);
	a00 = simd.replace(a00, 7, transmute(f32)u32(0x7f800000));
	x0, x1: util.F32x8;
	accepted: util.I32x8;
	x0, x1, accepted = physics.contact_normal_lcp2_scaled(a00, a01, a11, b0, b1, util.I32x8(-1), pair_mode);
	for lane: int = 0; lane < 8; lane += 1
	{
		if lane >= 6
		{
			testing.expect_value(t, simd.extract(accepted, lane), i32(0));
			continue;
		}
		system: [4][4]f64;
		system[0][0], system[1][1] = 2, 2;
		system[0][1], system[1][0] = 0.5, 0.5;
		rhs: [4]f64 = {-2.5, -2.5, 0, 0};
		if lane == 1
		{
			rhs = {-2, 1, 0, 0};
		}
		else if lane == 2
		{
			rhs = {1, -2, 0, 0};
		}
		else if lane == 3
		{
			rhs = {1, 1, 0, 0};
		}
		expected: [4]f64;
		state: physics.Reference_State;
		expected, state = contact_oracle_lcp(system, rhs, 2);
		testing.expect_value(t, state, physics.Reference_State.Present);
		testing.expect_value(t, simd.extract(accepted, lane), i32(-1));
		testing.expect(t, math.abs(f64(simd.extract(x0, lane))-expected[0]) < 2e-5);
		testing.expect(t, math.abs(f64(simd.extract(x1, lane))-expected[1]) < 2e-5);
	}
}

@(test)
paired_normal_finite_mixed_active_sets_match_independent_oracle :: proc(t: ^testing.T)
{
	paired_normal_finite_mixed_active_sets_match_independent_oracle_case(t, .Serial);
	paired_normal_finite_mixed_active_sets_match_independent_oracle_case(t, .Parallel);
}

paired_normal_finite_mixed_active_sets_match_independent_oracle_case :: proc(t: ^testing.T, $pair_mode: physics.Contact_Normal_Pair_Mode)
{
	// every active lane is well conditioned, including mixed sets and an exact active boundary
	cases: [8][5]f32 = {
		{2, 0.5, 2, -2.5, -2.5},
		{2, 0.5, 2, -2, 1},
		{2, 0.5, 2, 1, -2},
		{2, 0.5, 2, 1, 1},
		{2, -0.5, 2, -1.5, -1.5},
		{3, 0.75, 2, -3.75, -2.75},
		{2, 1, 2, -2, -1},
		{2, -0.5, 2, 1, -2},
	};
	separated_cases: [8][5]f32 = {
		{2, 0.5, 2, -2, 1},
		{2, 0.5, 2, 1, -2},
		{2, 0.5, 2, 1, 1},
		{2, -0.5, 2, -2, 1},
		{2, -0.5, 2, 1, -2},
		{3, 0.75, 2, -3, 1},
		{2, -0.5, 2, 1, 1},
		{3, 0.75, 2, 1, -2},
	};
	// the second full/partial/empty group has no feasible both-active lane
	// the third group keeps all four active sets off the cold exact boundary
	patterns: [9]u32 = {0xff, 0x5b, 0, 0xff, 0x5b, 0, 0xff, 0x0f, 0};
	poison: f32 = transmute(f32)u32(0x7fc00001);
	for pattern: int = 0; pattern < len(patterns); pattern += 1
	{
		a00, a01, a11, b0, b1: util.F32x8;
		active: util.I32x8;
		for lane: int = 0; lane < 8; lane += 1
		{
			values: [5]f32 = {poison, poison, poison, poison, poison};
			if patterns[pattern] & (u32(1) << uint(lane)) != 0
			{
				active = simd.replace(active, lane, i32(-1));
				values = cases[lane];
				if pattern >= 3 && pattern < 6
				{
					values = separated_cases[lane];
				}
				else if pattern >= 6 && lane == 6
				{
					values = {2, 0.5, 2, -2.5, -2.5};
				}
			}
			a00 = simd.replace(a00, lane, values[0]);
			a01 = simd.replace(a01, lane, values[1]);
			a11 = simd.replace(a11, lane, values[2]);
			b0 = simd.replace(b0, lane, values[3]);
			b1 = simd.replace(b1, lane, values[4]);
		}
		x0, x1: util.F32x8;
		accepted: util.I32x8;
		x0, x1, accepted = physics.contact_normal_lcp2_scaled(a00, a01, a11, b0, b1, active, pair_mode);
		for lane: int = 0; lane < 8; lane += 1
		{
			testing.expect_value(t, simd.extract(accepted, lane), simd.extract(active, lane));
			if simd.extract(active, lane) == 0
			{
				testing.expect_value(t, transmute(u32)simd.extract(x0, lane), u32(0));
				testing.expect_value(t, transmute(u32)simd.extract(x1, lane), u32(0));
				continue;
			}
			values: [5]f32 = cases[lane];
			if pattern >= 3 && pattern < 6
			{
				values = separated_cases[lane];
			}
			else if pattern >= 6 && lane == 6
			{
				values = {2, 0.5, 2, -2.5, -2.5};
			}
			system: [4][4]f64;
			system[0][0], system[1][1] = f64(values[0]), f64(values[2]);
			system[0][1], system[1][0] = f64(values[1]), f64(values[1]);
			rhs: [4]f64 = {f64(values[3]), f64(values[4]), 0, 0};
			expected: [4]f64;
			state: physics.Reference_State;
			expected, state = contact_oracle_lcp(system, rhs, 2);
			testing.expect_value(t, state, physics.Reference_State.Present);
			actual: [2]f64 = {f64(simd.extract(x0, lane)), f64(simd.extract(x1, lane))};
			for row: int = 0; row < 2; row += 1
			{
				testing.expect(t, actual[row] >= 0 && actual[row] < 3.4e38);
				testing.expect(t, math.abs(actual[row]-expected[row]) < 2e-5);
				residual: f64 = rhs[row]+system[row][0]*actual[0]+system[row][1]*actual[1];
				testing.expect(t, residual >= -2e-5);
				testing.expect(t, math.abs(actual[row]*residual) < 2e-5);
			}
			if lane == 6 && pattern < 6
			{
				testing.expect_value(t, transmute(u32)simd.extract(x1, lane), u32(0));
			}
		}
	}
}

@(test)
paired_normal_asymmetric_cancellation_and_schur_boundary_match_f64_oracle :: proc(t: ^testing.T)
{
	paired_normal_asymmetric_cancellation_and_schur_boundary_match_f64_oracle_case(t, .Serial);
	paired_normal_asymmetric_cancellation_and_schur_boundary_match_f64_oracle_case(t, .Parallel);
}

paired_normal_asymmetric_cancellation_and_schur_boundary_match_f64_oracle_case :: proc(t: ^testing.T, $pair_mode: physics.Contact_Normal_Pair_Mode)
{
	small_diagonal: f32 = 0.00000095367431640625;
	negative_near_half: f32 = transmute(f32)u32(0xbf000001);
	cases: [4][5]f32 = {
		{small_diagonal, 0.5, 1e7, negative_near_half, -1e7},
		{1e7, 0.5, small_diagonal, -1e7, negative_near_half},
		{1, transmute(f32)u32(0x3f7fdf39), 1, -1, -1},
		{1, transmute(f32)u32(0x3f7fdf3a), 1, -1, -1},
	};
	patterns: [2]u32 = {0xff, 0x55};
	poison: f32 = transmute(f32)u32(0x7fc00001);
	for sample: int = 0; sample < len(cases); sample += 1
	{
		// separate groups prevent a cold lane from concealing cancellation in an otherwise fast solve
		values: [5]f32 = cases[sample];
		row_scale: [2]f64 = {
			max(1, max(math.abs(f64(values[0])), math.abs(f64(values[3])))),
			max(1, max(math.abs(f64(values[2])), math.abs(f64(values[4])))),
		};
		system: [4][4]f64;
		system[0][0], system[0][1] = f64(values[0])/row_scale[0], f64(values[1])/row_scale[0];
		system[1][0], system[1][1] = f64(values[1])/row_scale[1], f64(values[2])/row_scale[1];
		rhs: [4]f64 = {f64(values[3])/row_scale[0], f64(values[4])/row_scale[1], 0, 0};
		expected: [4]f64;
		state: physics.Reference_State;
		expected, state = contact_oracle_lcp(system, rhs, 2);
		testing.expect_value(t, state, physics.Reference_State.Present);
		for pattern: int = 0; pattern < len(patterns); pattern += 1
		{
			a00, a01, a11, b0, b1: util.F32x8;
			active: util.I32x8;
			for lane: int = 0; lane < 8; lane += 1
			{
				input: [5]f32 = {poison, poison, poison, poison, poison};
				if patterns[pattern] & (u32(1) << uint(lane)) != 0
				{
					active = simd.replace(active, lane, i32(-1));
					input = values;
				}
				a00 = simd.replace(a00, lane, input[0]);
				a01 = simd.replace(a01, lane, input[1]);
				a11 = simd.replace(a11, lane, input[2]);
				b0 = simd.replace(b0, lane, input[3]);
				b1 = simd.replace(b1, lane, input[4]);
			}
			x0, x1: util.F32x8;
			accepted: util.I32x8;
			x0, x1, accepted = physics.contact_normal_lcp2_scaled(a00, a01, a11, b0, b1, active, pair_mode);
			testing.expect_value(t, accepted, active);
			for lane: int = 0; lane < 8; lane += 1
			{
				if simd.extract(active, lane) == 0
				{
					testing.expect_value(t, transmute(u32)simd.extract(x0, lane), u32(0));
					testing.expect_value(t, transmute(u32)simd.extract(x1, lane), u32(0));
					continue;
				}
				actual: [2]f64 = {f64(simd.extract(x0, lane)), f64(simd.extract(x1, lane))};
				for row: int = 0; row < 2; row += 1
				{
					testing.expect(t, actual[row] >= 0 && actual[row] < 3.4e38);
					testing.expect(t, math.abs(actual[row]-expected[row]) <= 2e-5*max(1, math.abs(expected[row])));
					residual: f64 = rhs[row]+system[row][0]*actual[0]+system[row][1]*actual[1];
					testing.expect(t, math.abs(residual) <= 2e-5);
				}
			}
		}
	}
}

@(test)
paired_normal_mt_range_boundaries_preserve_coupled_solutions :: proc(t: ^testing.T)
{
	paired_normal_mt_range_boundaries_preserve_coupled_solutions_case(t, .Serial);
	paired_normal_mt_range_boundaries_preserve_coupled_solutions_case(t, .Parallel);
}

paired_normal_mt_range_boundaries_preserve_coupled_solutions_case :: proc(t: ^testing.T, $pair_mode: physics.Contact_Normal_Pair_Mode)
{
	// compare boundary and recovery inputs in separate groups so cold lanes cannot hide a fast result
	cases: [8][5]f32 = {
		{1e-7, 0, 1e-7, -1e-7, -1e-7},
		{1e7, 0, 1e7, -1e7, -1e7},
		{0.5e-7, 0, 0.5e-7, -0.5e-7, -0.5e-7},
		{2e7, 0, 2e7, -2e7, -2e7},
		{1, 0, 1, -1e-15, -1e-15},
		{1, 0, 1, -0.5e-15, -0.5e-15},
		{1, 0.25, 1, -1e7, -1e7},
		{1, 0.25, 1, -2e7, -2e7},
	};
	patterns: [3]u32 = {0xff, 0x5b, 0};
	poison: f32 = transmute(f32)u32(0x7fc00001);
	for sample: int = 0; sample < len(cases); sample += 1
	{
		values: [5]f32 = cases[sample];
		unknown_scale: f64 = max(math.abs(f64(values[3])), math.abs(f64(values[4])))/max(f64(values[0]), f64(values[2]));
		row_scale: [2]f64 = {f64(values[0])*unknown_scale, f64(values[2])*unknown_scale};
		system: [4][4]f64;
		system[0][0] = f64(values[0])*unknown_scale/row_scale[0];
		system[0][1] = f64(values[1])*unknown_scale/row_scale[0];
		system[1][0] = f64(values[1])*unknown_scale/row_scale[1];
		system[1][1] = f64(values[2])*unknown_scale/row_scale[1];
		rhs: [4]f64 = {f64(values[3])/row_scale[0], f64(values[4])/row_scale[1], 0, 0};
		expected: [4]f64;
		state: physics.Reference_State;
		expected, state = contact_oracle_lcp(system, rhs, 2);
		testing.expect_value(t, state, physics.Reference_State.Present);
		for pattern: int = 0; pattern < len(patterns); pattern += 1
		{
			a00, a01, a11, b0, b1: util.F32x8;
			active: util.I32x8;
			for lane: int = 0; lane < 8; lane += 1
			{
				input: [5]f32 = {poison, poison, poison, poison, poison};
				if patterns[pattern] & (u32(1) << uint(lane)) != 0
				{
					active = simd.replace(active, lane, i32(-1));
					input = values;
				}
				a00 = simd.replace(a00, lane, input[0]);
				a01 = simd.replace(a01, lane, input[1]);
				a11 = simd.replace(a11, lane, input[2]);
				b0 = simd.replace(b0, lane, input[3]);
				b1 = simd.replace(b1, lane, input[4]);
			}
			x0, x1: util.F32x8;
			accepted: util.I32x8;
			x0, x1, accepted = physics.contact_normal_lcp2_scaled(a00, a01, a11, b0, b1, active, pair_mode);
			testing.expect_value(t, accepted, active);
			for lane: int = 0; lane < 8; lane += 1
			{
				if simd.extract(active, lane) == 0
				{
					testing.expect_value(t, transmute(u32)simd.extract(x0, lane), u32(0));
					testing.expect_value(t, transmute(u32)simd.extract(x1, lane), u32(0));
					continue;
				}
				actual: [2]f64 = {f64(simd.extract(x0, lane))/unknown_scale, f64(simd.extract(x1, lane))/unknown_scale};
				for row: int = 0; row < 2; row += 1
				{
					testing.expect(t, actual[row] >= 0 && actual[row] < 3.4e38);
					testing.expect(t, math.abs(actual[row]-expected[row]) <= 2e-5);
					residual: f64 = rhs[row]+system[row][0]*actual[0]+system[row][1]*actual[1];
					testing.expect(t, math.abs(residual) <= 2e-5);
				}
			}
		}
	}
}

@(test)
paired_normal_nonfinite_and_limit_magnitudes_reject_active_lanes :: proc(t: ^testing.T)
{
	paired_normal_nonfinite_and_limit_magnitudes_reject_active_lanes_case(t, .Serial);
	paired_normal_nonfinite_and_limit_magnitudes_reject_active_lanes_case(t, .Parallel);
}

paired_normal_nonfinite_and_limit_magnitudes_reject_active_lanes_case :: proc(t: ^testing.T, $pair_mode: physics.Contact_Normal_Pair_Mode)
{
	limit: f32 = 3.4e38;
	values: [8]f32 = {
		transmute(f32)u32(0x7fc00001), transmute(f32)u32(0xffc00001),
		transmute(f32)u32(0x7f800001), transmute(f32)u32(0xff800001),
		transmute(f32)u32(0x7f800000), transmute(f32)u32(0xff800000), limit, -limit,
	};
	invalid: util.F32x8;
	for lane: int = 0; lane < 8; lane += 1
	{
		invalid = simd.replace(invalid, lane, values[lane]);
	}
	for component: int = 0; component < 3; component += 1
	{
		a01: util.F32x8 = util.F32x8(0.5);
		b0, b1: util.F32x8 = util.F32x8(-2.5), util.F32x8(-2.5);
		switch component
		{
		case 0:
			a01 = invalid;
		case 1:
			b0 = invalid;
		case 2:
			b1 = invalid;
		}
		x0, x1: util.F32x8;
		accepted: util.I32x8;
		x0, x1, accepted = physics.contact_normal_lcp2_scaled(
			util.F32x8(2), a01, util.F32x8(2), b0, b1, util.I32x8(-1), pair_mode,
		);
		for lane: int = 0; lane < 8; lane += 1
		{
			testing.expect_value(t, simd.extract(accepted, lane), i32(0));
			testing.expect_value(t, transmute(u32)simd.extract(x0, lane), u32(0));
			testing.expect_value(t, transmute(u32)simd.extract(x1, lane), u32(0));
		}
	}
}

@(test)
paired_normal_extreme_scales_and_threshold_preserve_coupled_admission :: proc(t: ^testing.T)
{
	paired_normal_extreme_scales_and_threshold_preserve_coupled_admission_case(t, .Serial);
	paired_normal_extreme_scales_and_threshold_preserve_coupled_admission_case(t, .Parallel);
}

paired_normal_extreme_scales_and_threshold_preserve_coupled_admission_case :: proc(t: ^testing.T, $pair_mode: physics.Contact_Normal_Pair_Mode)
{
	// asymmetric systems have finite coupled solutions despite overflowing an unscaled numerator
	cases: [9][5]f32 = {
		{1, -5e18, 1e38, -1e38, 0},
		{1e38, -5e18, 1, 0, -1e38},
		{2e30, 0.5e30, 2e30, -2.5e30, -2.5e30},
		{2e-35, 0.5e-35, 2e-35, -2.5e-35, -2.5e-35},
		{1, 0.999994, 1, -1, -1},
		{1e-35, 0.999994e-35, 1e-35, -1e-35, -1e-35},
		{1e-20, 0, 1e10, -1e-38, -1},
		{1e10, 0, 1e-20, -1, -1e-38},
		{1, 0.999996, 1, -1, -1},
	};
	for sample: int = 0; sample < len(cases); sample += 1
	{
		values: [5]f32 = cases[sample];
		x0, x1: util.F32x8;
		accepted: util.I32x8;
		x0, x1, accepted = physics.contact_normal_lcp2_scaled(
			util.F32x8(values[0]), util.F32x8(values[1]), util.F32x8(values[2]),
			util.F32x8(values[3]), util.F32x8(values[4]), util.I32x8(-1), pair_mode,
		);
		if sample == len(cases)-1
		{
			// a mathematically solvable system below the conditioning limit stays inadmissible
			determinant: f64 = 1-f64(values[1])*f64(values[1]);
			testing.expect(t, determinant > 0 && determinant < 1e-5);
			testing.expect_value(t, accepted, util.I32x8(0));
			testing.expect_value(t, x0, util.F32x8(0));
			testing.expect_value(t, x1, util.F32x8(0));
			continue;
		}

		// scale equations and unknowns independently so the f64 oracle retains its absolute tolerances
		variable_scale: [2]f64 = {1, 1};
		row_scale: [2]f64 = {f64(values[0]), f64(values[2])};
		if sample == 0
		{
			variable_scale = {1e38, 1e19};
			row_scale = {1e38, 1e57};
		}
		else if sample == 1
		{
			variable_scale = {1e19, 1e38};
			row_scale = {1e57, 1e38};
		}
		else if sample == 6 || sample == 7
		{
			// normalize by the analytic diagonal impulses so tiny nonzero answers require relative accuracy
			variable_scale = {-f64(values[3])/f64(values[0]), -f64(values[4])/f64(values[2])};
			row_scale = {-f64(values[3]), -f64(values[4])};
		}
		system: [4][4]f64;
		system[0][0] = f64(values[0])*variable_scale[0]/row_scale[0];
		system[0][1] = f64(values[1])*variable_scale[1]/row_scale[0];
		system[1][0] = f64(values[1])*variable_scale[0]/row_scale[1];
		system[1][1] = f64(values[2])*variable_scale[1]/row_scale[1];
		rhs: [4]f64 = {f64(values[3])/row_scale[0], f64(values[4])/row_scale[1], 0, 0};
		expected: [4]f64;
		state: physics.Reference_State;
		expected, state = contact_oracle_lcp(system, rhs, 2);
		testing.expect_value(t, state, physics.Reference_State.Present);
		for lane: int = 0; lane < 8; lane += 1
		{
			testing.expect_value(t, simd.extract(accepted, lane), i32(-1));
			actual: [2]f64 = {
				f64(simd.extract(x0, lane))/variable_scale[0],
				f64(simd.extract(x1, lane))/variable_scale[1],
			};
			for row: int = 0; row < 2; row += 1
			{
				testing.expect(t, actual[row] >= 0 && actual[row] < 3.4e38);
				testing.expect(t, math.abs(actual[row]-expected[row]) <= 2e-5*max(1, math.abs(expected[row])));
				residual: f64 = rhs[row]+system[row][0]*actual[0]+system[row][1]*actual[1];
				testing.expect(t, math.abs(residual) <= 2e-5);
			}
		}
	}
}

@(test)
paired_normal_uniform_support_and_zero_impulses_preserve_masked_outputs :: proc(t: ^testing.T)
{
	paired_normal_uniform_support_and_zero_impulses_preserve_masked_outputs_case(t, .Serial);
	paired_normal_uniform_support_and_zero_impulses_preserve_masked_outputs_case(t, .Parallel);
}

paired_normal_uniform_support_and_zero_impulses_preserve_masked_outputs_case :: proc(t: ^testing.T, $pair_mode: physics.Contact_Normal_Pair_Mode)
{
	patterns: [3]u32 = {0, 0x55, 0xff};
	poison: f32 = transmute(f32)u32(0x7fc00001);
	negative_zero: f32 = transmute(f32)u32(0x80000000);
	for sample: int = 0; sample < 3; sample += 1
	{
		for pattern_index: int = 0; pattern_index < len(patterns); pattern_index += 1
		{
			a00, a01, a11, b0, b1: util.F32x8;
			active: util.I32x8;
			for lane: int = 0; lane < 8; lane += 1
			{
				diagonal, coupling, left, right: f32 = poison, poison, poison, poison;
				if patterns[pattern_index] & (u32(1) << uint(lane)) != 0
				{
					active = simd.replace(active, lane, i32(-1));
					diagonal, coupling, left, right = 2, 0.5, -2.5, -2.5;
					if sample == 1
					{
						diagonal, coupling, left, right = 1, 0, 0, negative_zero;
					}
					else if sample == 2
					{
						diagonal, coupling, left, right = 2, 1, 0, 0;
					}
				}
				a00 = simd.replace(a00, lane, diagonal);
				a11 = simd.replace(a11, lane, diagonal);
				a01 = simd.replace(a01, lane, coupling);
				b0 = simd.replace(b0, lane, left);
				b1 = simd.replace(b1, lane, right);
			}

			x0, x1: util.F32x8;
			accepted: util.I32x8;
			x0, x1, accepted = physics.contact_normal_lcp2_scaled(a00, a01, a11, b0, b1, active, pair_mode);
			for lane: int = 0; lane < 8; lane += 1
			{
				expected0, expected1: f32;
				if patterns[pattern_index] & (u32(1) << uint(lane)) != 0
				{
					expected0, expected1 = 1, 1;
					if sample == 1
					{
						expected0, expected1 = negative_zero, 0;
					}
					else if sample == 2
					{
						expected0, expected1 = 0, 0;
					}
				}
				testing.expect_value(t, simd.extract(accepted, lane), simd.extract(active, lane));
				testing.expect_value(t, transmute(u32)simd.extract(x0, lane), transmute(u32)expected0);
				testing.expect_value(t, transmute(u32)simd.extract(x1, lane), transmute(u32)expected1);
			}
		}
	}
}
