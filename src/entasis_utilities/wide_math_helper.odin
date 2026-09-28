// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_utilities

import "core:simd"

cos_approx_wide :: proc "contextless" (x: F32x8) -> F32x8
{
	period_count := simd.mul(x, F32x8(f32(0.5 / 3.141592653589793239)));
	period_x := simd.mul(simd.sub(period_count, simd.floor(period_count)), F32x8(TWO_PI));
	pi_over_2 := F32x8(PI_OVER_2);
	pi := F32x8(PI);
	pi_3_over_2 := F32x8(3 * PI_OVER_2);
	y := wide_select_f32(transmute(I32x8)simd.lanes_gt(period_x, pi_over_2), simd.sub(pi, period_x), period_x);
	y = wide_select_f32(transmute(I32x8)simd.lanes_gt(period_x, pi), simd.sub(period_x, pi), y);
	y = wide_select_f32(transmute(I32x8)simd.lanes_gt(period_x, pi_3_over_2), simd.sub(F32x8(TWO_PI), period_x), y);
	numerator := simd.add(simd.mul(F32x8(-0.003436308368583229), y), F32x8(0.021317031205957775));
	numerator = simd.add(simd.mul(numerator, y), F32x8(0.06955843390178032));
	numerator = simd.sub(simd.mul(numerator, y), F32x8(0.4578088075324152));
	numerator = simd.add(simd.mul(numerator, y), F32x8(-0.15082367674208508));
	numerator = simd.add(simd.mul(numerator, y), F32x8(1));
	denominator := simd.add(simd.mul(F32x8(-0.00007650398834677185), y), F32x8(0.0007451378206294365));
	denominator = simd.add(simd.mul(denominator, y), F32x8(-0.00585321045829395));
	denominator = simd.add(simd.mul(denominator, y), F32x8(0.04219116713777847));
	denominator = simd.add(simd.mul(denominator, y), F32x8(-0.15082367538305258));
	denominator = simd.add(simd.mul(denominator, y), F32x8(1));
	result := simd.div(numerator, denominator);
	middle_mask := simd.bit_and(simd.lanes_gt(period_x, pi_over_2), simd.lanes_lt(period_x, pi_3_over_2));
	return wide_select_f32(transmute(I32x8)middle_mask, simd.neg(result), result);
}

sin_approx_wide :: proc "contextless" (x: F32x8) -> F32x8
{
	period_count := simd.mul(x, F32x8(f32(0.5 / 3.141592653589793239)));
	period_x := simd.mul(simd.sub(period_count, simd.floor(period_count)), F32x8(TWO_PI));
	pi := F32x8(PI);
	y := wide_select_f32(transmute(I32x8)simd.lanes_gt(period_x, F32x8(PI_OVER_2)), simd.sub(pi, period_x), period_x);
	second_half := transmute(I32x8)simd.lanes_gt(period_x, pi);
	y = wide_select_f32(second_half, simd.sub(period_x, pi), y);
	y = wide_select_f32(
		transmute(I32x8)simd.lanes_gt(period_x, F32x8(3 * PI_OVER_2)),
		simd.sub(F32x8(TWO_PI), period_x),
		y
	);
	numerator := simd.add(simd.mul(F32x8(0.0040507708755727605), y), F32x8(-0.006685815219853882));
	numerator = simd.add(simd.mul(numerator, y), F32x8(-0.13993701695343166));
	numerator = simd.add(simd.mul(numerator, y), F32x8(0.06174562337697123));
	numerator = simd.add(simd.mul(numerator, y), F32x8(1.00000000151466040));
	numerator = simd.mul(numerator, y);
	denominator := simd.add(simd.mul(F32x8(0.00009018370615921334), y), F32x8(0.0001700784176413186));
	denominator = simd.add(simd.mul(denominator, y), F32x8(0.003606014457152456));
	denominator = simd.add(simd.mul(denominator, y), F32x8(0.02672943625500751));
	denominator = simd.add(simd.mul(denominator, y), F32x8(0.061745651499203795));
	denominator = simd.add(simd.mul(denominator, y), F32x8(1));
	result := simd.div(numerator, denominator);
	return wide_select_f32(second_half, simd.neg(result), result);
}

acos_approx_wide :: proc "contextless" (input: F32x8) -> F32x8
{
	negative := transmute(I32x8)simd.lanes_lt(input, F32x8(0));
	x := simd.min(F32x8(1), simd.abs(input));
	numerator := simd.mul(
		simd.sqrt(simd.sub(F32x8(1), x)),
		simd.add(
		F32x8(62.95741097600742),
		simd.mul(
		x,
		simd.add(
		F32x8(69.6550664543659),
		simd.mul(x, simd.add(F32x8(17.54512349463405), simd.mul(x, F32x8(0.6022076120669532))))
	)
	)
	)
	);
	denominator := simd.add(
		F32x8(40.07993264439811),
		simd.mul(x, simd.add(F32x8(49.81949855726789), simd.mul(x, simd.add(F32x8(15.703851745284796), x))))
	);
	result := simd.div(numerator, denominator);
	return wide_select_f32(negative, simd.sub(F32x8(PI), result), result);
}

signed_angle_difference_wide :: proc "contextless" (a, b: F32x8) -> F32x8
{
	x := simd.add(simd.mul(simd.sub(b, a), F32x8(1 / TWO_PI)), F32x8(0.5));
	return simd.mul(simd.sub(simd.sub(x, simd.floor(x)), F32x8(0.5)), F32x8(TWO_PI));
}
