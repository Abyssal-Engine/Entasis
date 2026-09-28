// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_utilities

import "core:math"

PI :: f32(3.141592653589793239);
TWO_PI :: f32(6.283185307179586477);
PI_OVER_2 :: f32(1.570796326794896619);
PI_OVER_4 :: f32(0.785398163397448310);

to_radians :: proc "contextless" (degrees: f32) -> f32
{
	return degrees * (PI / 180);
}
to_degrees :: proc "contextless" (radians: f32) -> f32
{
	return radians * (180 / PI);
}
binary_sign :: proc "contextless" (value: f32) -> f32
{
	if value < 0
	{
		return -1;
	}
	return 1;
}

cos_approx :: proc "contextless" (x: f32) -> f32
{
	period_count := x * f32(0.5 / math.PI);
	period_fraction := period_count - math.floor(period_count);
	period_x := period_fraction * TWO_PI;
	y := period_x;
	if period_x > 3 * PI_OVER_2
	{
		y = TWO_PI - period_x;
	}
	else if period_x > PI
	{
		y = period_x - PI;
	}
	else if period_x > PI_OVER_2
	{
		y = PI - period_x;
	}
	numerator := ((((-0.003436308368583229 * y + 0.021317031205957775) * y + 0.06955843390178032) * y - 0.4578088075324152) * y - 0.15082367674208508) * y + 1;
	denominator := ((((-0.00007650398834677185 * y + 0.0007451378206294365) * y - 0.00585321045829395) * y + 0.04219116713777847) * y - 0.15082367538305258) * y + 1;
	result := numerator / denominator;
	if period_x > PI_OVER_2 && period_x < 3 * PI_OVER_2
	{
		return -result;
	}
	return result;
}

sin_approx :: proc "contextless" (x: f32) -> f32
{
	period_count := x * f32(0.5 / math.PI);
	period_fraction := period_count - math.floor(period_count);
	period_x := period_fraction * TWO_PI;
	y := period_x;
	if period_x > 3 * PI_OVER_2
	{
		y = TWO_PI - period_x;
	}
	else if period_x > PI
	{
		y = period_x - PI;
	}
	else if period_x > PI_OVER_2
	{
		y = PI - period_x;
	}
	numerator := ((((0.0040507708755727605 * y - 0.006685815219853882) * y - 0.13993701695343166) * y + 0.06174562337697123) * y + 1.00000000151466040) * y;
	denominator := ((((0.00009018370615921334 * y + 0.0001700784176413186) * y + 0.003606014457152456) * y + 0.02672943625500751) * y + 0.061745651499203795) * y + 1;
	result := numerator / denominator;
	if period_x > PI
	{
		return -result;
	}
	return result;
}

acos_approx :: proc "contextless" (input: f32) -> f32
{
	x := input;
	x = min(f32(1), abs(x));
	numerator := math.sqrt(1 - x) * (62.95741097600742 + x * (69.6550664543659 + x * (17.54512349463405 + x * 0.6022076120669532)));
	denominator := 40.07993264439811 + x * (49.81949855726789 + x * (15.703851745284796 + x));
	result := numerator / denominator;
	if input < 0
	{
		return PI - result;
	}
	return result;
}

signed_angle_difference :: proc "contextless" (a, b: f32) -> f32
{
	x := (b - a) * (1 / TWO_PI) + 0.5;
	return (x - math.floor(x) - 0.5) * TWO_PI;
}
