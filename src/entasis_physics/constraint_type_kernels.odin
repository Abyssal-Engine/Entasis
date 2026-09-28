// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"
import "core:simd"

Constraint_Kernel_Phase :: enum u8
{
	Prestep,
	Warmstart,
	Solve,
	Incremental_Update,
}

Constraint_Kernel_Body_Wide :: struct
{
	position:         util.Vector3_Wide,
	orientation:      util.Quaternion_Wide,
	linear_velocity:  util.Vector3_Wide,
	angular_velocity: util.Vector3_Wide,
	inverse_mass:     util.F32x8,
	inverse_inertia:  util.Symmetric3x3_Wide,
}

// contact stages never read body pose fields. keeping their hot scratch in a
// compact representation reduces stack traffic and live ranges without changing
// the fixed public constraint-kernel ABI used by registered type processors
Constraint_Contact_Body_Wide :: struct
{
	linear_velocity:  util.Vector3_Wide,
	angular_velocity: util.Vector3_Wide,
	inverse_mass:     util.F32x8,
	inverse_inertia:  util.Symmetric3x3_Wide,
}

// authored prestep prefixes retain the pinned C# AOSOA field order.
// ball socket also stores internal solve data after the authored prefix for each substep.
// the shared raw procedure signature is used only for the caller-registration ABI
Angular_Axis_Gear_Motor_Prestep :: struct
{
	local_axis_a: util.Vector3_Wide, velocity_scale: util.F32x8, settings: Motor_Settings_Wide,
}

Angular_Axis_Motor_Prestep :: struct
{
	local_axis_a: util.Vector3_Wide, target_velocity: util.F32x8, settings: Motor_Settings_Wide,
}

Angular_Hinge_Prestep :: struct
{
	local_hinge_axis_a, local_hinge_axis_b: util.Vector3_Wide, spring_settings: Spring_Settings_Wide,
}

Angular_Motor_Prestep :: struct
{
	target_velocity_local_a: util.Vector3_Wide, settings: Motor_Settings_Wide,
}

Angular_Servo_Prestep :: struct
{
	target_relative_rotation_local_a: util.Quaternion_Wide, spring_settings: Spring_Settings_Wide, servo_settings: Servo_Settings_Wide,
}

Angular_Swivel_Hinge_Prestep :: struct
{
	local_swivel_axis_a, local_hinge_axis_b: util.Vector3_Wide, spring_settings: Spring_Settings_Wide,
}

Area_Constraint_Prestep :: struct
{
	target_scaled_area: util.F32x8, spring_settings: Spring_Settings_Wide,
}

Ball_Socket_Solve_Data :: struct
{
	offset_a, offset_b, bias: util.Vector3_Wide,
	effective_mass: util.Symmetric3x3_Wide,
	softness: util.F32x8,
}

Ball_Socket_Prestep :: struct
{
	local_offset_a, local_offset_b: util.Vector3_Wide, spring_settings: Spring_Settings_Wide,
	solve_data: Ball_Socket_Solve_Data,
}

Ball_Socket_Motor_Prestep :: struct
{
	local_offset_b, target_velocity_local_a: util.Vector3_Wide, settings: Motor_Settings_Wide,
}

Ball_Socket_Servo_Prestep :: struct
{
	local_offset_a, local_offset_b: util.Vector3_Wide, spring_settings: Spring_Settings_Wide, servo_settings: Servo_Settings_Wide,
}

Center_Distance_Constraint_Prestep :: struct
{
	target_distance: util.F32x8, spring_settings: Spring_Settings_Wide,
}

Center_Distance_Limit_Prestep :: struct
{
	minimum_distance, maximum_distance: util.F32x8, spring_settings: Spring_Settings_Wide,
}

Distance_Limit_Prestep :: struct
{
	local_offset_a, local_offset_b: util.Vector3_Wide, minimum_distance, maximum_distance: util.F32x8, spring_settings: Spring_Settings_Wide,
}

Distance_Servo_Prestep :: struct
{
	local_offset_a, local_offset_b: util.Vector3_Wide, target_distance: util.F32x8, servo_settings: Servo_Settings_Wide, spring_settings: Spring_Settings_Wide,
}

Hinge_Prestep :: struct
{
	local_offset_a, local_hinge_axis_a, local_offset_b, local_hinge_axis_b: util.Vector3_Wide, spring_settings: Spring_Settings_Wide,
}

Linear_Axis_Limit_Prestep :: struct
{
	local_offset_a, local_offset_b, local_axis: util.Vector3_Wide, minimum_offset, maximum_offset: util.F32x8, spring_settings: Spring_Settings_Wide,
}

Linear_Axis_Motor_Prestep :: struct
{
	local_offset_a, local_offset_b, local_axis: util.Vector3_Wide, target_velocity: util.F32x8, settings: Motor_Settings_Wide,
}

Linear_Axis_Servo_Prestep :: struct
{
	local_offset_a, local_offset_b, local_plane_normal: util.Vector3_Wide, target_offset: util.F32x8, servo_settings: Servo_Settings_Wide, spring_settings: Spring_Settings_Wide,
}

One_Body_Angular_Motor_Prestep :: struct
{
	target_velocity: util.Vector3_Wide, settings: Motor_Settings_Wide,
}

One_Body_Angular_Servo_Prestep :: struct
{
	target_orientation: util.Quaternion_Wide, spring_settings: Spring_Settings_Wide, servo_settings: Servo_Settings_Wide,
}

One_Body_Linear_Motor_Prestep :: struct
{
	local_offset, target_velocity: util.Vector3_Wide, settings: Motor_Settings_Wide,
}

One_Body_Linear_Servo_Prestep :: struct
{
	local_offset, target: util.Vector3_Wide, spring_settings: Spring_Settings_Wide, servo_settings: Servo_Settings_Wide,
}

Point_On_Line_Servo_Prestep :: struct
{
	local_offset_a, local_offset_b, local_direction: util.Vector3_Wide, servo_settings: Servo_Settings_Wide, spring_settings: Spring_Settings_Wide,
}

Swing_Limit_Prestep :: struct
{
	axis_local_a, axis_local_b: util.Vector3_Wide, minimum_dot: util.F32x8, spring_settings: Spring_Settings_Wide,
}

Swivel_Hinge_Prestep :: struct
{
	local_offset_a, local_swivel_axis_a, local_offset_b, local_hinge_axis_b: util.Vector3_Wide, spring_settings: Spring_Settings_Wide,
}

Twist_Limit_Prestep :: struct
{
	local_basis_a, local_basis_b: util.Quaternion_Wide, minimum_angle, maximum_angle: util.F32x8, spring_settings: Spring_Settings_Wide,
}

Twist_Motor_Prestep :: struct
{
	local_axis_a, local_axis_b: util.Vector3_Wide, target_velocity: util.F32x8, settings: Motor_Settings_Wide,
}

Twist_Servo_Prestep :: struct
{
	local_basis_a, local_basis_b: util.Quaternion_Wide, target_angle: util.F32x8, spring_settings: Spring_Settings_Wide, servo_settings: Servo_Settings_Wide,
}

Volume_Constraint_Prestep :: struct
{
	target_scaled_volume: util.F32x8, spring_settings: Spring_Settings_Wide,
}

Weld_Prestep :: struct
{
	local_offset: util.Vector3_Wide, local_orientation: util.Quaternion_Wide, spring_settings: Spring_Settings_Wide,
}

Hinge_Accumulated_Impulses :: struct
{
	ball_socket: util.Vector3_Wide, hinge: util.Vector2_Wide,
}

Weld_Accumulated_Impulses :: struct
{
	orientation, offset: util.Vector3_Wide,
}

Convex_Contact_Wide :: struct
{
	offset_a: util.Vector3_Wide, depth: util.F32x8,
}

Nonconvex_Contact_Wide :: struct
{
	offset: util.Vector3_Wide, depth: util.F32x8, normal: util.Vector3_Wide,
}

Contact_Material_Wide :: struct
{
	friction_coefficient: util.F32x8, spring_settings: Spring_Settings_Wide, maximum_recovery_velocity: util.F32x8,
}

// N4 geometry and its coefficient record share the same Solver-owned substep lifetime
Contact_4_Geometry :: struct
{
	basis_sign:       util.F32x8,
	basis_scale:      util.F32x8,
	friction_center_a: util.Vector3_Wide,
	radii:            [4]util.F32x8,
}

// Solver stores derived values separately from authored prestep data
Contact_Convex_Solve_Data :: struct($contact_count: int)
{
	penetration_effective_mass: [contact_count]util.F32x8,
	penetration_bias:           [contact_count]util.F32x8,
	softness:                  util.F32x8,
	tangent_effective_mass:     util.Symmetric2x2_Wide,
	twist_effective_mass:       util.F32x8,
}

// N4 stores its compact coefficients and geometry without aliasing the old prefix
Contact_4_Solve_Data :: struct
{
	penetration_effective_mass: [4]util.F32x8,
	position_error_to_velocity: util.F32x8,
	maximum_recovery_velocity: util.F32x8,
	softness:                  util.F32x8,
	tangent_effective_mass:     util.Symmetric2x2_Wide,
	twist_effective_mass:       util.F32x8,
	geometry:                  Contact_4_Geometry,
}

Contact_1_One_Body_Prestep :: struct
{
	contacts: [1]Convex_Contact_Wide, normal: util.Vector3_Wide, material: Contact_Material_Wide,
}

Contact_2_One_Body_Prestep :: struct
{
	contacts: [2]Convex_Contact_Wide, normal: util.Vector3_Wide, material: Contact_Material_Wide,
}

Contact_3_One_Body_Prestep :: struct
{
	contacts: [3]Convex_Contact_Wide, normal: util.Vector3_Wide, material: Contact_Material_Wide,
}

Contact_4_One_Body_Prestep :: struct
{
	contacts: [4]Convex_Contact_Wide, normal: util.Vector3_Wide, material: Contact_Material_Wide,
}

Contact_1_Prestep :: struct
{
	contacts: [1]Convex_Contact_Wide, offset_b, normal: util.Vector3_Wide, material: Contact_Material_Wide,
}

Contact_2_Prestep :: struct
{
	contacts: [2]Convex_Contact_Wide, offset_b, normal: util.Vector3_Wide, material: Contact_Material_Wide,
}

Contact_3_Prestep :: struct
{
	contacts: [3]Convex_Contact_Wide, offset_b, normal: util.Vector3_Wide, material: Contact_Material_Wide,
}

Contact_4_Prestep :: struct
{
	contacts: [4]Convex_Contact_Wide, offset_b, normal: util.Vector3_Wide, material: Contact_Material_Wide,
}

Contact_2_Nonconvex_One_Body_Prestep :: struct
{
	material: Contact_Material_Wide, contacts: [2]Nonconvex_Contact_Wide,
}

Contact_3_Nonconvex_One_Body_Prestep :: struct
{
	material: Contact_Material_Wide, contacts: [3]Nonconvex_Contact_Wide,
}

Contact_4_Nonconvex_One_Body_Prestep :: struct
{
	material: Contact_Material_Wide, contacts: [4]Nonconvex_Contact_Wide,
}

Contact_2_Nonconvex_Prestep :: struct
{
	material: Contact_Material_Wide, offset_b: util.Vector3_Wide, contacts: [2]Nonconvex_Contact_Wide,
}

Contact_3_Nonconvex_Prestep :: struct
{
	material: Contact_Material_Wide, offset_b: util.Vector3_Wide, contacts: [3]Nonconvex_Contact_Wide,
}

Contact_4_Nonconvex_Prestep :: struct
{
	material: Contact_Material_Wide, offset_b: util.Vector3_Wide, contacts: [4]Nonconvex_Contact_Wide,
}

Contact_1_Accumulated_Impulses :: struct
{
	tangent: util.Vector2_Wide, penetration: [1]util.F32x8, twist: util.F32x8,
}

Contact_2_Accumulated_Impulses :: struct
{
	tangent: util.Vector2_Wide, penetration: [2]util.F32x8, twist: util.F32x8,
}

Contact_3_Accumulated_Impulses :: struct
{
	tangent: util.Vector2_Wide, penetration: [3]util.F32x8, twist: util.F32x8,
}

Contact_4_Accumulated_Impulses :: struct
{
	tangent: util.Vector2_Wide, penetration: [4]util.F32x8, twist: util.F32x8,
}

Nonconvex_Contact_Accumulated_Impulses :: struct
{
	tangent: util.Vector2_Wide, penetration: util.F32x8,
}

Contact_2_Nonconvex_Accumulated_Impulses :: struct
{
	contacts: [2]Nonconvex_Contact_Accumulated_Impulses,
}

Contact_3_Nonconvex_Accumulated_Impulses :: struct
{
	contacts: [3]Nonconvex_Contact_Accumulated_Impulses,
}

Contact_4_Nonconvex_Accumulated_Impulses :: struct
{
	contacts: [4]Nonconvex_Contact_Accumulated_Impulses,
}

Constraint_Kernel_Proc :: #type proc "contextless" (
	prestep: rawptr,
	bodies: ^[4]Constraint_Kernel_Body_Wide,
	dt, inverse_dt: f32,
	impulses: rawptr,
	active_mask: util.I32x8,
	phase: Constraint_Kernel_Phase,
);
constraint_kernel_select_vector3 :: #force_inline proc "contextless" (
	mask: util.I32x8, replacement, original: util.Vector3_Wide,
) -> util.Vector3_Wide
{
	return {
		util.wide_select_f32(mask, replacement.x, original.x),
		util.wide_select_f32(mask, replacement.y, original.y),
		util.wide_select_f32(mask, replacement.z, original.z),
	};
}

constraint_kernel_apply_linear :: #force_inline proc "contextless" (
	body: ^Constraint_Kernel_Body_Wide, impulse: util.Vector3_Wide, sign: f32, mask: util.I32x8,
)
{
	scale := simd.mul(body.inverse_mass, util.F32x8(sign));
	updated := util.Vector3_Wide{
		x=simd.add(body.linear_velocity.x, simd.mul(impulse.x, scale)),
		y=simd.add(body.linear_velocity.y, simd.mul(impulse.y, scale)),
		z=simd.add(body.linear_velocity.z, simd.mul(impulse.z, scale)),
	};
	body.linear_velocity = constraint_kernel_select_vector3(mask, updated, body.linear_velocity);
}

constraint_kernel_apply_angular :: #force_inline proc "contextless" (
	body: ^$Body, impulse: util.Vector3_Wide, sign: f32, mask: util.I32x8,
)
{
	signed := util.Vector3_Wide{
		x=simd.mul(impulse.x, util.F32x8(sign)),
		y=simd.mul(impulse.y, util.F32x8(sign)),
		z=simd.mul(impulse.z, util.F32x8(sign)),
	};
	angular_delta := util.Vector3_Wide{
		x=simd.add(
			simd.add(simd.mul(body.inverse_inertia.xx, signed.x), simd.mul(body.inverse_inertia.yx, signed.y)),
			simd.mul(body.inverse_inertia.zx, signed.z)
		),
		y=simd.add(
			simd.add(simd.mul(body.inverse_inertia.yx, signed.x), simd.mul(body.inverse_inertia.yy, signed.y)),
			simd.mul(body.inverse_inertia.zy, signed.z)
		),
		z=simd.add(
			simd.add(simd.mul(body.inverse_inertia.zx, signed.x), simd.mul(body.inverse_inertia.zy, signed.y)),
			simd.mul(body.inverse_inertia.zz, signed.z)
		),
	};
	updated := util.Vector3_Wide{
		x=simd.add(body.angular_velocity.x, angular_delta.x),
		y=simd.add(body.angular_velocity.y, angular_delta.y),
		z=simd.add(body.angular_velocity.z, angular_delta.z),
	};
	body.angular_velocity = constraint_kernel_select_vector3(mask, updated, body.angular_velocity);
}

constraint_kernel_mask_scalar :: #force_inline proc "contextless" (value: util.F32x8, mask: util.I32x8) -> util.F32x8
{
	return util.wide_select_f32(mask, value, util.F32x8(0));
}

constraint_kernel_mask_vector2 :: #force_inline proc "contextless" (
	value: util.Vector2_Wide,
	mask: util.I32x8
) -> util.Vector2_Wide
{
	return util.vector2_wide_select(mask, value, {});
}

constraint_kernel_mask_vector3 :: #force_inline proc "contextless" (
	value: util.Vector3_Wide,
	mask: util.I32x8
) -> util.Vector3_Wide
{
	return util.vector3_wide_select(mask, value, {});
}

constraint_kernel_prepare_vector3_impulses :: #force_inline proc "contextless" (
	impulses: ^util.Vector3_Wide, active_mask: util.I32x8,
)
{
	impulses^ = constraint_kernel_mask_vector3(impulses^, active_mask);
}

constraint_kernel_prepare_scalar_impulses :: #force_inline proc "contextless" (
	impulses: ^util.F32x8, active_mask: util.I32x8,
)
{
	impulses^ = constraint_kernel_mask_scalar(impulses^, active_mask);
}

constraint_kernel_clamp_accumulated_scalar :: #force_inline proc "contextless" (
	maximum_impulse: util.F32x8, accumulated, csi: ^util.F32x8, active_mask: util.I32x8,
)
{
	servo_settings_wide_clamp_impulse(maximum_impulse, accumulated, csi);
	accumulated^ = constraint_kernel_mask_scalar(accumulated^, active_mask);
	csi^ = constraint_kernel_mask_scalar(csi^, active_mask);
}

constraint_kernel_apply_world_impulses :: #force_inline proc "contextless" (
	body: ^$Body, linear_impulse, angular_impulse: util.Vector3_Wide,
	active_mask: util.I32x8,
)
{
	linear_change := util.vector3_wide_scale(linear_impulse, body.inverse_mass);
	angular_change := util.symmetric3x3_wide_transform(angular_impulse, body.inverse_inertia);
	body.linear_velocity = constraint_kernel_select_vector3(
		active_mask,
		util.vector3_wide_add(body.linear_velocity, linear_change),
		body.linear_velocity
	);
	body.angular_velocity = constraint_kernel_select_vector3(
		active_mask,
		util.vector3_wide_add(body.angular_velocity, angular_change),
		body.angular_velocity
	);
}

constraint_kernel_apply_point_one_body :: #force_inline proc "contextless" (
	body: ^Constraint_Kernel_Body_Wide, offset, impulse: util.Vector3_Wide, active_mask: util.I32x8,
)
{
	constraint_kernel_apply_world_impulses(body, impulse, util.vector3_wide_cross(offset, impulse), active_mask);
}

constraint_kernel_apply_ball_socket :: #force_inline proc "contextless" (
	body_a, body_b: ^Constraint_Kernel_Body_Wide, offset_a, offset_b, impulse: util.Vector3_Wide,
	active_mask: util.I32x8,
)
{
	constraint_kernel_apply_world_impulses(
		body_a,
		impulse,
		util.vector3_wide_cross(offset_a, impulse),
		active_mask
	);
	constraint_kernel_apply_world_impulses(
		body_b,
		util.vector3_wide_negate(impulse),
		util.vector3_wide_cross(impulse, offset_b),
		active_mask
	);
}

constraint_kernel_ball_socket_effective_mass :: #force_inline proc "contextless" (
	body_a, body_b: ^Constraint_Kernel_Body_Wide, offset_a, offset_b: util.Vector3_Wide,
	effective_mass_scale: util.F32x8,
) -> util.Symmetric3x3_Wide
{
	inverse_effective_mass := util.symmetric3x3_wide_skew_sandwich(offset_a, body_a.inverse_inertia);
	inverse_effective_mass = util.symmetric3x3_wide_add(
		inverse_effective_mass,
		util.symmetric3x3_wide_skew_sandwich(offset_b, body_b.inverse_inertia)
	);
	linear := simd.add(body_a.inverse_mass, body_b.inverse_mass);
	inverse_effective_mass.xx = simd.add(inverse_effective_mass.xx, linear);
	inverse_effective_mass.yy = simd.add(inverse_effective_mass.yy, linear);
	inverse_effective_mass.zz = simd.add(inverse_effective_mass.zz, linear);
	return util.symmetric3x3_wide_scale(util.symmetric3x3_wide_invert(inverse_effective_mass), effective_mass_scale);
}

constraint_kernel_one_body_point_effective_mass :: #force_inline proc "contextless" (
	body: ^Constraint_Kernel_Body_Wide, offset: util.Vector3_Wide, effective_mass_scale: util.F32x8,
) -> util.Symmetric3x3_Wide
{
	inverse_effective_mass := util.symmetric3x3_wide_skew_sandwich(offset, body.inverse_inertia);
	inverse_effective_mass.xx = simd.add(inverse_effective_mass.xx, body.inverse_mass);
	inverse_effective_mass.yy = simd.add(inverse_effective_mass.yy, body.inverse_mass);
	inverse_effective_mass.zz = simd.add(inverse_effective_mass.zz, body.inverse_mass);
	return util.symmetric3x3_wide_scale(util.symmetric3x3_wide_invert(inverse_effective_mass), effective_mass_scale);
}

constraint_kernel_clamp_accumulated_vector3 :: #force_inline proc "contextless" (
	maximum_impulse: util.F32x8, accumulated, csi: ^util.Vector3_Wide, active_mask: util.I32x8,
)
{
	servo_settings_wide_clamp_impulse_vector3(maximum_impulse, accumulated, csi);
	accumulated^ = constraint_kernel_mask_vector3(accumulated^, active_mask);
	csi^ = constraint_kernel_mask_vector3(csi^, active_mask);
}

constraint_kernel_build_orthonormal_basis :: #force_inline proc "contextless" (
	normal: util.Vector3_Wide,
) -> (tangent_x, tangent_y: util.Vector3_Wide)
{
	negative_z := transmute(util.I32x8)simd.lanes_lt(normal.z, util.F32x8(0));
	sign := util.wide_select_f32(negative_z, util.F32x8(-1), util.F32x8(1));
	scale := simd.div(util.F32x8(-1), simd.add(sign, normal.z));
	tangent_x = {
		simd.mul(simd.mul(normal.x, normal.y), scale),
		simd.add(sign, simd.mul(simd.mul(normal.y, normal.y), scale)),
		simd.neg(normal.y),
	};
	tangent_y = {
		simd.add(util.F32x8(1), simd.mul(simd.mul(simd.mul(sign, normal.x), normal.x), scale)),
		simd.mul(sign, tangent_x.x),
		simd.neg(simd.mul(sign, normal.x)),
	};
	return;
}

constraint_contact_4_build_basis :: #force_inline proc "contextless" (
	normal: util.Vector3_Wide, sign, scale: util.F32x8,
) -> (tangent_x, tangent_y: util.Vector3_Wide)
{
	tangent_x = {
		simd.mul(simd.mul(normal.x, normal.y), scale),
		simd.add(sign, simd.mul(simd.mul(normal.y, normal.y), scale)),
		simd.neg(normal.y),
	};
	tangent_y = {
		simd.add(util.F32x8(1), simd.mul(simd.mul(simd.mul(sign, normal.x), normal.x), scale)),
		simd.mul(sign, tangent_x.x),
		simd.neg(simd.mul(sign, normal.x)),
	};
	return;
}

constraint_contact_penetration_apply :: #force_inline proc "contextless" (
	bodies: [^]$Body, normal, offset_a, offset_b: util.Vector3_Wide,
	impulse: util.F32x8, active_mask: util.I32x8, $body_count: int,
)
{
	angular_a := util.vector3_wide_cross(offset_a, normal);
	linear_impulse := util.vector3_wide_scale(normal, impulse);
	constraint_kernel_apply_world_impulses(
		&bodies[0],
		linear_impulse,
		util.vector3_wide_scale(angular_a, impulse),
		active_mask
	);
	when body_count == 2
	{
		angular_b := util.vector3_wide_cross(normal, offset_b);
		constraint_kernel_apply_world_impulses(
			&bodies[1],
			util.vector3_wide_negate(linear_impulse),
			util.vector3_wide_scale(angular_b, impulse),
			active_mask
		);
	}
}

constraint_contact_penetration_solve :: #force_inline proc "contextless" (
	bodies: [^]$Body, normal, offset_a, offset_b: util.Vector3_Wide,
	depth, position_error_to_velocity, effective_mass_scale, maximum_recovery_velocity, softness: util.F32x8,
	dt, inverse_dt: f32, accumulated: ^util.F32x8, active_mask: util.I32x8, $body_count: int,
)
{
	_ = dt;
	angular_a := util.vector3_wide_cross(offset_a, normal);
	inverse_effective_mass := simd.add(
		bodies[0].inverse_mass,
		util.symmetric3x3_wide_vector_sandwich(angular_a, bodies[0].inverse_inertia)
	);
	csv := simd.add(
		util.vector3_wide_dot(bodies[0].linear_velocity, normal),
		util.vector3_wide_dot(bodies[0].angular_velocity, angular_a)
	);
	when body_count == 2
	{
		angular_b := util.vector3_wide_cross(normal, offset_b);
		inverse_effective_mass = simd.add(
			inverse_effective_mass,
			simd.add(
				bodies[1].inverse_mass,
				util.symmetric3x3_wide_vector_sandwich(angular_b, bodies[1].inverse_inertia)
			)
		);
		csv = simd.add(
			simd.sub(csv, util.vector3_wide_dot(bodies[1].linear_velocity, normal)),
			util.vector3_wide_dot(bodies[1].angular_velocity, angular_b)
		);
	}
	effective_mass := simd.div(effective_mass_scale, inverse_effective_mass);
	bias := simd.min(
		simd.mul(depth, util.F32x8(inverse_dt)),
		simd.min(simd.mul(depth, position_error_to_velocity), maximum_recovery_velocity)
	);
	negated_csi := simd.add(simd.mul(accumulated^, softness), simd.mul(simd.sub(csv, bias), effective_mass));
	previous := accumulated^;
	accumulated^ = simd.max(util.F32x8(0), simd.sub(accumulated^, negated_csi));
	accumulated^ = constraint_kernel_mask_scalar(accumulated^, active_mask);
	csi := constraint_kernel_mask_scalar(simd.sub(accumulated^, previous), active_mask);
	constraint_contact_penetration_apply(bodies, normal, offset_a, offset_b, csi, active_mask, body_count);
}

constraint_contact_update_depth :: #force_inline proc "contextless" (
	bodies: [^]$Body, offset_a, offset_b, normal: util.Vector3_Wide,
	dt: f32, depth: ^util.F32x8, active_mask: util.I32x8, $body_count: int,
)
{
	velocity_a := util.vector3_wide_add(
		bodies[0].linear_velocity,
		util.vector3_wide_cross(bodies[0].angular_velocity, offset_a)
	);
	velocity_difference := velocity_a;
	when body_count == 2
	{
		velocity_b := util.vector3_wide_add(
			bodies[1].linear_velocity,
			util.vector3_wide_cross(bodies[1].angular_velocity, offset_b)
		);
		velocity_difference = util.vector3_wide_subtract(velocity_a, velocity_b);
	}
	updated := simd.sub(depth^, simd.mul(util.vector3_wide_dot(normal, velocity_difference), util.F32x8(dt)));
	depth^ = util.wide_select_f32(active_mask, updated, depth^);
}

constraint_contact_tangent_apply :: #force_inline proc "contextless" (
	bodies: [^]$Body, tangent_x, tangent_y, offset_a, offset_b: util.Vector3_Wide,
	impulse: util.Vector2_Wide, active_mask: util.I32x8, $body_count: int,
)
{
	linear_jacobian := util.Matrix2x3_Wide{tangent_x, tangent_y};
	angular_a := util.Matrix2x3_Wide{
		util.vector3_wide_cross(offset_a, tangent_x),
		util.vector3_wide_cross(offset_a, tangent_y),
	};
	linear_impulse := util.matrix2x3_wide_transform(impulse, linear_jacobian);
	angular_impulse_a := util.matrix2x3_wide_transform(impulse, angular_a);
	constraint_kernel_apply_world_impulses(&bodies[0], linear_impulse, angular_impulse_a, active_mask);
	when body_count == 2
	{
		angular_b := util.Matrix2x3_Wide{
			util.vector3_wide_cross(tangent_x, offset_b),
			util.vector3_wide_cross(tangent_y, offset_b),
		};
		constraint_kernel_apply_world_impulses(
			&bodies[1],
			util.vector3_wide_negate(linear_impulse),
			util.matrix2x3_wide_transform(impulse, angular_b),
			active_mask
		);
	}
}

constraint_contact_tangent_solve :: #force_inline proc "contextless" (
	bodies: [^]$Body, tangent_x, tangent_y, offset_a, offset_b: util.Vector3_Wide,
	maximum_impulse: util.F32x8, accumulated: ^util.Vector2_Wide, active_mask: util.I32x8, $body_count: int,
)
{
	linear_jacobian := util.Matrix2x3_Wide{tangent_x, tangent_y};
	angular_a := util.Matrix2x3_Wide{
		util.vector3_wide_cross(offset_a, tangent_x),
		util.vector3_wide_cross(offset_a, tangent_y),
	};
	inverse_effective_mass := util.symmetric2x2_wide_add(
		util.symmetric2x2_wide_sandwich_scale(linear_jacobian, bodies[0].inverse_mass),
		util.symmetric3x3_wide_matrix_sandwich(angular_a, bodies[0].inverse_inertia),
	);
	csv_linear := util.vector2_wide_scale(
		util.matrix2x3_wide_transform_by_transpose(bodies[0].linear_velocity, linear_jacobian),
		-1
	);
	csv_angular := util.vector2_wide_scale(
		util.matrix2x3_wide_transform_by_transpose(bodies[0].angular_velocity, angular_a),
		-1
	);
	when body_count == 2
	{
		angular_b := util.Matrix2x3_Wide{
			util.vector3_wide_cross(tangent_x, offset_b),
			util.vector3_wide_cross(tangent_y, offset_b),
		};
		inverse_effective_mass = util.symmetric2x2_wide_add(inverse_effective_mass, util.symmetric2x2_wide_add(
				util.symmetric2x2_wide_sandwich_scale(linear_jacobian, bodies[1].inverse_mass),
				util.symmetric3x3_wide_matrix_sandwich(angular_b, bodies[1].inverse_inertia),
		));
		csv_linear = util.vector2_wide_add(
			csv_linear,
			util.matrix2x3_wide_transform_by_transpose(bodies[1].linear_velocity, linear_jacobian)
		);
		csv_angular = util.vector2_wide_subtract(
			csv_angular,
			util.matrix2x3_wide_transform_by_transpose(bodies[1].angular_velocity, angular_b)
		);
	}
	csi := util.symmetric2x2_wide_transform(
		util.vector2_wide_add(csv_linear, csv_angular),
		util.symmetric2x2_wide_invert(inverse_effective_mass)
	);
	previous := accumulated^;
	accumulated^ = util.vector2_wide_add(accumulated^, csi);
	magnitude := util.vector2_wide_length(accumulated^);
	scale := simd.min(util.F32x8(1), simd.div(maximum_impulse, simd.max(util.F32x8(1e-16), magnitude)));
	accumulated^ = constraint_kernel_mask_vector2(util.vector2_wide_scale(accumulated^, scale), active_mask);
	csi = constraint_kernel_mask_vector2(util.vector2_wide_subtract(accumulated^, previous), active_mask);
	constraint_contact_tangent_apply(bodies, tangent_x, tangent_y, offset_a, offset_b, csi, active_mask, body_count);
}

constraint_contact_twist_apply :: #force_inline proc "contextless" (
	bodies: [^]$Body, normal: util.Vector3_Wide, impulse: util.F32x8,
	active_mask: util.I32x8, $body_count: int,
)
{
	world_impulse := util.vector3_wide_scale(normal, impulse);
	constraint_kernel_apply_angular(&bodies[0], world_impulse, 1, active_mask);
	when body_count == 2
	{
		constraint_kernel_apply_angular(&bodies[1], world_impulse, -1, active_mask);
	}
}

constraint_contact_twist_solve :: #force_inline proc "contextless" (
	bodies: [^]$Body, normal: util.Vector3_Wide, maximum_impulse: util.F32x8,
	accumulated: ^util.F32x8, active_mask: util.I32x8, $body_count: int,
)
{
	inverse_effective_mass := util.symmetric3x3_wide_vector_sandwich(normal, bodies[0].inverse_inertia);
	csv := util.vector3_wide_dot(bodies[0].angular_velocity, normal);
	when body_count == 2
	{
		inverse_effective_mass = simd.add(
			inverse_effective_mass,
			util.symmetric3x3_wide_vector_sandwich(normal, bodies[1].inverse_inertia)
		);
		csv = simd.sub(csv, util.vector3_wide_dot(bodies[1].angular_velocity, normal));
	}
	effective_mass := util.wide_select_f32(
		transmute(util.I32x8)simd.lanes_eq(inverse_effective_mass, util.F32x8(0)),
		util.F32x8(0),
		simd.div(util.F32x8(1), inverse_effective_mass)
	);
	previous := accumulated^;
	accumulated^ = simd.max(
		simd.neg(maximum_impulse),
		simd.min(maximum_impulse, simd.sub(accumulated^, simd.mul(csv, effective_mass)))
	);
	accumulated^ = constraint_kernel_mask_scalar(accumulated^, active_mask);
	csi := constraint_kernel_mask_scalar(simd.sub(accumulated^, previous), active_mask);
	constraint_contact_twist_apply(bodies, normal, csi, active_mask, body_count);
}

constraint_contact_friction_center :: #force_inline proc "contextless" (
	contacts: [^]Convex_Contact_Wide, $contact_count: int,
) -> util.Vector3_Wide
{
	weights: [4]util.F32x8;
	weight_sum := util.F32x8(0);
	for contact_index in 0 ..< contact_count
	{
		weights[contact_index] = util.wide_select_f32(
			transmute(util.I32x8)simd.lanes_lt(contacts[contact_index].depth, util.F32x8(0)),
			util.F32x8(0),
			util.F32x8(1)
		);
		weight_sum = simd.add(weight_sum, weights[contact_index]);
	}
	use_uniform_weights := transmute(util.I32x8)simd.lanes_eq(weight_sum, util.F32x8(0));
	weight_sum = util.wide_select_f32(use_uniform_weights, util.F32x8(f32(contact_count)), weight_sum);
	inverse_weight_sum := simd.div(util.F32x8(1), weight_sum);
	center := util.Vector3_Wide{};
	for contact_index in 0 ..< contact_count
	{
		weight := util.wide_select_f32(
			use_uniform_weights,
			inverse_weight_sum,
			simd.mul(weights[contact_index], inverse_weight_sum)
		);
		center = util.vector3_wide_add(center, util.vector3_wide_scale(contacts[contact_index].offset_a, weight));
	}
	return center;
}

constraint_contact_convex_kernel :: #force_inline proc "contextless" (
	prestep_raw: rawptr, bodies: [^]$Body, dt, inverse_dt: f32,
	impulses_raw: rawptr, active_mask: util.I32x8, phase: Constraint_Kernel_Phase,
	$body_count, $contact_count: int,
)
{
	constraint_contact_convex_kernel_response(
		prestep_raw, bodies, dt, inverse_dt, impulses_raw, active_mask, phase,
		body_count, contact_count, nil, .Default,
	);
}

constraint_contact_convex_kernel_response :: #force_inline proc "contextless" (
	prestep_raw: rawptr, bodies: [^]$Body, dt, inverse_dt: f32,
	impulses_raw: rawptr, active_mask: util.I32x8, phase: Constraint_Kernel_Phase,
	$body_count, $contact_count: int,
	targets: [^]Restitution_Target_Wide, $response: Solver_Response_Mode,
)
{
	contacts := ([^]Convex_Contact_Wide)(prestep_raw);
	cursor := uintptr(prestep_raw) + uintptr(contact_count * size_of(Convex_Contact_Wide));
	offset_b := util.Vector3_Wide{};
	when body_count == 2
	{
		offset_b = (^util.Vector3_Wide)(rawptr(cursor))^;
		cursor += uintptr(size_of(util.Vector3_Wide));
	}
	normal := (^util.Vector3_Wide)(rawptr(cursor))^;
	cursor += uintptr(size_of(util.Vector3_Wide));
	material := (^Contact_Material_Wide)(rawptr(cursor));
	tangent := (^util.Vector2_Wide)(impulses_raw);
	penetrations := ([^]util.F32x8)(rawptr(uintptr(impulses_raw) + uintptr(2 * size_of(util.F32x8))));
	twist := (^util.F32x8)(rawptr(uintptr(impulses_raw) + uintptr((2 + contact_count) * size_of(util.F32x8))));
	if phase == .Prestep
	{
		tangent^ = constraint_kernel_mask_vector2(tangent^, active_mask);
		for contact_index in 0 ..< contact_count
		{
			penetrations[contact_index] = constraint_kernel_mask_scalar(penetrations[contact_index], active_mask);
		}
		twist^ = constraint_kernel_mask_scalar(twist^, active_mask);
		return;
	}
	if phase == .Incremental_Update
	{
		for contact_index in 0 ..< contact_count
		{
			contact_offset_b := util.vector3_wide_subtract(contacts[contact_index].offset_a, offset_b);
			constraint_contact_update_depth(
				bodies,
				contacts[contact_index].offset_a,
				contact_offset_b,
				normal,
				dt,
				&contacts[contact_index].depth,
				active_mask,
				body_count
			);
		}
		return;
	}
	tangent_x, tangent_y := constraint_kernel_build_orthonormal_basis(normal);
	friction_center_a := constraint_contact_friction_center(contacts, contact_count);
	friction_center_b := util.vector3_wide_subtract(friction_center_a, offset_b);
	if phase == .Warmstart
	{
		constraint_contact_tangent_apply(
			bodies,
			tangent_x,
			tangent_y,
			friction_center_a,
			friction_center_b,
			tangent^,
			active_mask,
			body_count
		);
		for contact_index in 0 ..< contact_count
		{
			contact_offset_b := util.vector3_wide_subtract(contacts[contact_index].offset_a, offset_b);
			constraint_contact_penetration_apply(
				bodies,
				normal,
				contacts[contact_index].offset_a,
				contact_offset_b,
				penetrations[contact_index],
				active_mask,
				body_count
			);
		}
		constraint_contact_twist_apply(bodies, normal, twist^, active_mask, body_count);
		return;
	}
	position_error_to_velocity, effective_mass_scale, softness := spring_settings_wide_compute(
		material.spring_settings,
		dt
	);
	for contact_index in 0 ..< contact_count
	{
		contact_offset_b := util.vector3_wide_subtract(contacts[contact_index].offset_a, offset_b);
		when response == .Restitution
		{
			effective_mass, bias, normal_softness: util.F32x8;
			effective_mass, bias, normal_softness = restitution_prepare_normal(
				bodies, normal, contacts[contact_index].offset_a, contact_offset_b, contacts[contact_index].depth,
				position_error_to_velocity, effective_mass_scale, material.maximum_recovery_velocity,
				softness, inverse_dt, &targets[contact_index], body_count,
			);
			constraint_contact_penetration_solve_cached(
				bodies, normal, contacts[contact_index].offset_a, contact_offset_b, normal_softness,
				&penetrations[contact_index], active_mask, body_count, &effective_mass, &bias,
			);
		}
		else
		{
			constraint_contact_penetration_solve(
				bodies, normal, contacts[contact_index].offset_a, contact_offset_b, contacts[contact_index].depth,
				position_error_to_velocity, effective_mass_scale, material.maximum_recovery_velocity, softness,
				dt, inverse_dt, &penetrations[contact_index], active_mask, body_count,
			);
		}
	}
	total_penetration := util.F32x8(0);
	maximum_twist_radius_impulse := util.F32x8(0);
	for contact_index in 0 ..< contact_count
	{
		total_penetration = simd.add(total_penetration, penetrations[contact_index]);
		when contact_count == 1
		{
			maximum_twist_radius_impulse = simd.mul(
				penetrations[contact_index],
				simd.max(util.F32x8(0), contacts[contact_index].depth)
			);
		}
		else
		{
			radius := util.vector3_wide_length(util.vector3_wide_subtract(
					friction_center_a,
					contacts[contact_index].offset_a
			));
			maximum_twist_radius_impulse = simd.add(
				maximum_twist_radius_impulse,
				simd.mul(penetrations[contact_index], radius)
			);
		}
	}
	constraint_contact_tangent_solve(
		bodies,
		tangent_x,
		tangent_y,
		friction_center_a,
		friction_center_b,
		simd.mul(material.friction_coefficient, total_penetration),
		tangent,
		active_mask,
		body_count
	);
	constraint_contact_twist_solve(
		bodies,
		normal,
		simd.mul(material.friction_coefficient, maximum_twist_radius_impulse),
		twist,
		active_mask,
		body_count
	);
}

constraint_contact_penetration_prepare_solve :: #force_inline proc "contextless" (
	bodies: [^]$Body, angular_a, angular_b: util.Vector3_Wide,
	depth, position_error_to_velocity, effective_mass_scale, maximum_recovery_velocity: util.F32x8,
	inverse_dt: f32, $body_count: int,
) -> (effective_mass, bias: util.F32x8)
{
	inverse_effective_mass: util.F32x8 = simd.add(
		bodies[0].inverse_mass,
		util.symmetric3x3_wide_vector_sandwich(angular_a, bodies[0].inverse_inertia)
	);
	when body_count == 2
	{
		inverse_effective_mass = simd.add(
			inverse_effective_mass,
			simd.add(
				bodies[1].inverse_mass,
				util.symmetric3x3_wide_vector_sandwich(angular_b, bodies[1].inverse_inertia)
			)
		);
	}
	effective_mass = simd.div(effective_mass_scale, inverse_effective_mass);
	bias = simd.min(
		simd.mul(depth, util.F32x8(inverse_dt)),
		simd.min(simd.mul(depth, position_error_to_velocity), maximum_recovery_velocity)
	);
	return;
}

constraint_contact_tangent_prepare_solve :: #force_inline proc "contextless" (
	bodies: [^]$Body, linear_jacobian, angular_a, angular_b: util.Matrix2x3_Wide,
	$body_count: int,
) -> util.Symmetric2x2_Wide
{
	inverse_effective_mass: util.Symmetric2x2_Wide = util.symmetric2x2_wide_add(
		util.symmetric2x2_wide_sandwich_scale(linear_jacobian, bodies[0].inverse_mass),
		util.symmetric3x3_wide_matrix_sandwich(angular_a, bodies[0].inverse_inertia),
	);
	when body_count == 2
	{
		inverse_effective_mass = util.symmetric2x2_wide_add(inverse_effective_mass, util.symmetric2x2_wide_add(
				util.symmetric2x2_wide_sandwich_scale(linear_jacobian, bodies[1].inverse_mass),
				util.symmetric3x3_wide_matrix_sandwich(angular_b, bodies[1].inverse_inertia),
		));
	}
	return util.symmetric2x2_wide_invert(inverse_effective_mass);
}

constraint_contact_twist_prepare_solve :: #force_inline proc "contextless" (
	bodies: [^]$Body, normal: util.Vector3_Wide, $body_count: int,
) -> util.F32x8
{
	inverse_effective_mass: util.F32x8 = util.symmetric3x3_wide_vector_sandwich(normal, bodies[0].inverse_inertia);
	when body_count == 2
	{
		inverse_effective_mass = simd.add(
			inverse_effective_mass,
			util.symmetric3x3_wide_vector_sandwich(normal, bodies[1].inverse_inertia)
		);
	}
	return util.wide_select_f32(
		transmute(util.I32x8)simd.lanes_eq(inverse_effective_mass, util.F32x8(0)),
		util.F32x8(0),
		simd.div(util.F32x8(1), inverse_effective_mass)
	);
}

constraint_contact_convex_warmstart_cached :: #force_inline proc "contextless" (
	prestep_raw: rawptr, bodies: [^]$Body,
	impulses_raw: rawptr, active_mask: util.I32x8, $body_count: int,
	data: ^Contact_4_Solve_Data,
)
{
	// prestep and depth updates are complete. geometry is shared by all later sweeps
	contacts: [^]Convex_Contact_Wide = ([^]Convex_Contact_Wide)(prestep_raw);
	cursor: uintptr = uintptr(prestep_raw) + uintptr(4 * size_of(Convex_Contact_Wide));
	offset_b: util.Vector3_Wide;
	when body_count == 2
	{
		offset_b = (^util.Vector3_Wide)(rawptr(cursor))^;
		cursor += uintptr(size_of(util.Vector3_Wide));
	}
	normal: util.Vector3_Wide = (^util.Vector3_Wide)(rawptr(cursor))^;
	geometry: ^Contact_4_Geometry = &data.geometry;
	negative_z: util.I32x8 = transmute(util.I32x8)simd.lanes_lt(normal.z, util.F32x8(0));
	sign: util.F32x8 = util.wide_select_f32(negative_z, util.F32x8(-1), util.F32x8(1));
	scale: util.F32x8 = simd.div(util.F32x8(-1), simd.add(sign, normal.z));
	tangent_x, tangent_y: util.Vector3_Wide;
	tangent_x, tangent_y = constraint_contact_4_build_basis(normal, sign, scale);
	friction_center_a: util.Vector3_Wide = constraint_contact_friction_center(contacts, 4);
	geometry.basis_sign = sign;
	geometry.basis_scale = scale;
	geometry.friction_center_a = friction_center_a;
	for contact_index: int = 0; contact_index < 4; contact_index += 1
	{
		geometry.radii[contact_index] = util.vector3_wide_length(util.vector3_wide_subtract(
				friction_center_a,
				contacts[contact_index].offset_a
		));
	}
	friction_center_b: util.Vector3_Wide = util.vector3_wide_subtract(friction_center_a, offset_b);
	tangent: ^util.Vector2_Wide = (^util.Vector2_Wide)(impulses_raw);
	penetrations: [^]util.F32x8 = ([^]util.F32x8)(rawptr(uintptr(impulses_raw) + uintptr(2 * size_of(util.F32x8))));
	twist: ^util.F32x8 = (^util.F32x8)(rawptr(uintptr(impulses_raw) + uintptr(6 * size_of(util.F32x8))));
	constraint_contact_tangent_apply(
		bodies, tangent_x, tangent_y, friction_center_a, friction_center_b,
		tangent^, active_mask, body_count,
	);
	for contact_index: int = 0; contact_index < 4; contact_index += 1
	{
		contact_offset_b: util.Vector3_Wide = util.vector3_wide_subtract(contacts[contact_index].offset_a, offset_b);
		constraint_contact_penetration_apply(
			bodies, normal, contacts[contact_index].offset_a, contact_offset_b,
			penetrations[contact_index], active_mask, body_count,
		);
	}
	constraint_contact_twist_apply(bodies, normal, twist^, active_mask, body_count);
}

constraint_contact_convex_prepare_solve :: #force_inline proc "contextless" (
	prestep_raw: rawptr, bodies: [^]$Body, dt, inverse_dt: f32,
	$body_count, $contact_count: int,
	data: ^$Solve_Data,
)
{
	constraint_contact_convex_prepare_solve_response(
		prestep_raw, bodies, dt, inverse_dt, body_count, contact_count, data, nil, .Default,
	);
}

// targets are captured before warmstart and remain immutable through all solve
// iterations. cache layouts and default callers do not acquire response fields
constraint_contact_convex_prepare_solve_response :: #force_inline proc "contextless" (
	prestep_raw: rawptr, bodies: [^]$Body, dt, inverse_dt: f32,
	$body_count, $contact_count: int,
	data: ^$Solve_Data, targets: [^]Restitution_Target_Wide,
	$response: Solver_Response_Mode,
)
{
	// the normal-batch path calls this after Warmstart with current World inertia.
	// it reads only invariant geometry, material, and inertia, not updated velocities
	contacts: [^]Convex_Contact_Wide = ([^]Convex_Contact_Wide)(prestep_raw);
	cursor: uintptr = uintptr(prestep_raw) + uintptr(contact_count * size_of(Convex_Contact_Wide));
	offset_b: util.Vector3_Wide = util.Vector3_Wide{};
	_ = offset_b;
	when body_count == 2
	{
		offset_b = (^util.Vector3_Wide)(rawptr(cursor))^;
		cursor += uintptr(size_of(util.Vector3_Wide));
	}
	normal: util.Vector3_Wide = (^util.Vector3_Wide)(rawptr(cursor))^;
	cursor += uintptr(size_of(util.Vector3_Wide));
	material: ^Contact_Material_Wide = (^Contact_Material_Wide)(rawptr(cursor));
	tangent_x, tangent_y: util.Vector3_Wide;
	friction_center_a: util.Vector3_Wide;
	when contact_count == 4
	{
		geometry: ^Contact_4_Geometry = &data.geometry;
		tangent_x, tangent_y = constraint_contact_4_build_basis(normal, geometry.basis_sign, geometry.basis_scale);
		friction_center_a = geometry.friction_center_a;
	}
	else
	{
		tangent_x, tangent_y = constraint_kernel_build_orthonormal_basis(normal);
		friction_center_a = constraint_contact_friction_center(contacts, contact_count);
	}
	position_error_to_velocity, effective_mass_scale, softness: util.F32x8;
	position_error_to_velocity, effective_mass_scale, softness = spring_settings_wide_compute(
		material.spring_settings, dt,
	);
	data.softness = softness;
	when contact_count == 4
	{
		data.position_error_to_velocity = position_error_to_velocity;
		data.maximum_recovery_velocity = material.maximum_recovery_velocity;
	}
	for contact_index in 0 ..< contact_count
	{
		effective_mass, bias: util.F32x8;
		when response == .Restitution
		{
			contact_offset_b: util.Vector3_Wide = util.vector3_wide_subtract(contacts[contact_index].offset_a, offset_b);
			effective_mass, bias, _ = restitution_prepare_normal(
				bodies, normal, contacts[contact_index].offset_a, contact_offset_b,
				contacts[contact_index].depth, position_error_to_velocity, effective_mass_scale,
				material.maximum_recovery_velocity, softness, inverse_dt, &targets[contact_index], body_count,
			);
		}
		else
		{
			angular_a: util.Vector3_Wide = util.vector3_wide_cross(contacts[contact_index].offset_a, normal);
			angular_b: util.Vector3_Wide = util.Vector3_Wide{};
			when body_count == 2
			{
				contact_offset_b: util.Vector3_Wide = util.vector3_wide_subtract(contacts[contact_index].offset_a, offset_b);
				angular_b = util.vector3_wide_cross(normal, contact_offset_b);
			}
			effective_mass, bias = constraint_contact_penetration_prepare_solve(
				bodies, angular_a, angular_b, contacts[contact_index].depth,
				position_error_to_velocity, effective_mass_scale, material.maximum_recovery_velocity,
				inverse_dt, body_count,
			);
		}
		data.penetration_effective_mass[contact_index] = effective_mass;
		when contact_count == 4
		{
			_ = bias;
		}
		else
		{
			data.penetration_bias[contact_index] = bias;
		}
	}
	linear_jacobian: util.Matrix2x3_Wide = util.Matrix2x3_Wide{tangent_x, tangent_y};
	angular_a: util.Matrix2x3_Wide = util.Matrix2x3_Wide{
		util.vector3_wide_cross(friction_center_a, tangent_x),
		util.vector3_wide_cross(friction_center_a, tangent_y),
	};
	angular_b: util.Matrix2x3_Wide = util.Matrix2x3_Wide{};
	when body_count == 2
	{
		friction_center_b: util.Vector3_Wide = util.vector3_wide_subtract(friction_center_a, offset_b);
		angular_b = {
			util.vector3_wide_cross(tangent_x, friction_center_b),
			util.vector3_wide_cross(tangent_y, friction_center_b),
		};
	}
	data.tangent_effective_mass = constraint_contact_tangent_prepare_solve(
		bodies, linear_jacobian, angular_a, angular_b, body_count,
	);
	data.twist_effective_mass = constraint_contact_twist_prepare_solve(bodies, normal, body_count);
}

// cached kernels are separate to keep the registered recomputing code and ABI unchanged.
// Solver owns aligned, disjoint coefficient storage from Warmstart through all sweeps
constraint_contact_penetration_solve_cached :: #force_inline proc "contextless" (
	bodies: [^]$Body, normal, offset_a, offset_b: util.Vector3_Wide,
	softness: util.F32x8, accumulated: ^util.F32x8, active_mask: util.I32x8,
	$body_count: int, prepared_effective_mass, prepared_bias: ^util.F32x8,
)
{
	angular_a: util.Vector3_Wide = util.vector3_wide_cross(offset_a, normal);
	angular_b: util.Vector3_Wide;
	_ = &angular_b;
	when body_count == 2
	{
		angular_b = util.vector3_wide_cross(normal, offset_b);
	}
	effective_mass: util.F32x8 = prepared_effective_mass^;
	bias: util.F32x8 = prepared_bias^;
	csv: util.F32x8 = simd.add(
		util.vector3_wide_dot(bodies[0].linear_velocity, normal),
		util.vector3_wide_dot(bodies[0].angular_velocity, angular_a)
	);
	when body_count == 2
	{
		csv = simd.add(
			simd.sub(csv, util.vector3_wide_dot(bodies[1].linear_velocity, normal)),
			util.vector3_wide_dot(bodies[1].angular_velocity, angular_b)
		);
	}
	negated_csi: util.F32x8 = simd.add(simd.mul(accumulated^, softness), simd.mul(simd.sub(csv, bias), effective_mass));
	previous: util.F32x8 = accumulated^;
	accumulated^ = simd.max(util.F32x8(0), simd.sub(accumulated^, negated_csi));
	accumulated^ = constraint_kernel_mask_scalar(accumulated^, active_mask);
	csi: util.F32x8 = constraint_kernel_mask_scalar(simd.sub(accumulated^, previous), active_mask);
	constraint_contact_penetration_apply(bodies, normal, offset_a, offset_b, csi, active_mask, body_count);
}

constraint_contact_tangent_solve_cached :: #force_inline proc "contextless" (
	bodies: [^]$Body, tangent_x, tangent_y, offset_a, offset_b: util.Vector3_Wide,
	maximum_impulse: util.F32x8, accumulated: ^util.Vector2_Wide, active_mask: util.I32x8,
	$body_count: int, prepared_effective_mass: ^util.Symmetric2x2_Wide,
)
{
	linear_jacobian: util.Matrix2x3_Wide = {tangent_x, tangent_y};
	angular_a: util.Matrix2x3_Wide = {
		util.vector3_wide_cross(offset_a, tangent_x),
		util.vector3_wide_cross(offset_a, tangent_y),
	};
	angular_b: util.Matrix2x3_Wide;
	_ = &angular_b;
	when body_count == 2
	{
		angular_b = {
			util.vector3_wide_cross(tangent_x, offset_b),
			util.vector3_wide_cross(tangent_y, offset_b),
		};
	}
	effective_mass: util.Symmetric2x2_Wide = prepared_effective_mass^;
	csv_linear: util.Vector2_Wide = util.vector2_wide_scale(
		util.matrix2x3_wide_transform_by_transpose(bodies[0].linear_velocity, linear_jacobian),
		-1
	);
	csv_angular: util.Vector2_Wide = util.vector2_wide_scale(
		util.matrix2x3_wide_transform_by_transpose(bodies[0].angular_velocity, angular_a),
		-1
	);
	when body_count == 2
	{
		csv_linear = util.vector2_wide_add(
			csv_linear,
			util.matrix2x3_wide_transform_by_transpose(bodies[1].linear_velocity, linear_jacobian)
		);
		csv_angular = util.vector2_wide_subtract(
			csv_angular,
			util.matrix2x3_wide_transform_by_transpose(bodies[1].angular_velocity, angular_b)
		);
	}
	csi: util.Vector2_Wide = util.symmetric2x2_wide_transform(
		util.vector2_wide_add(csv_linear, csv_angular),
		effective_mass
	);
	previous: util.Vector2_Wide = accumulated^;
	accumulated^ = util.vector2_wide_add(accumulated^, csi);
	magnitude: util.F32x8 = util.vector2_wide_length(accumulated^);
	scale: util.F32x8 = simd.min(util.F32x8(1), simd.div(maximum_impulse, simd.max(util.F32x8(1e-16), magnitude)));
	accumulated^ = constraint_kernel_mask_vector2(util.vector2_wide_scale(accumulated^, scale), active_mask);
	csi = constraint_kernel_mask_vector2(util.vector2_wide_subtract(accumulated^, previous), active_mask);
	constraint_contact_tangent_apply(bodies, tangent_x, tangent_y, offset_a, offset_b, csi, active_mask, body_count);
}

constraint_contact_twist_solve_cached :: #force_inline proc "contextless" (
	bodies: [^]$Body, normal: util.Vector3_Wide, maximum_impulse: util.F32x8,
	accumulated: ^util.F32x8, active_mask: util.I32x8, $body_count: int,
	prepared_effective_mass: ^util.F32x8,
)
{
	effective_mass: util.F32x8 = prepared_effective_mass^;
	csv: util.F32x8 = util.vector3_wide_dot(bodies[0].angular_velocity, normal);
	when body_count == 2
	{
		csv = simd.sub(csv, util.vector3_wide_dot(bodies[1].angular_velocity, normal));
	}
	previous: util.F32x8 = accumulated^;
	accumulated^ = simd.max(
		simd.neg(maximum_impulse),
		simd.min(maximum_impulse, simd.sub(accumulated^, simd.mul(csv, effective_mass)))
	);
	accumulated^ = constraint_kernel_mask_scalar(accumulated^, active_mask);
	csi: util.F32x8 = constraint_kernel_mask_scalar(simd.sub(accumulated^, previous), active_mask);
	constraint_contact_twist_apply(bodies, normal, csi, active_mask, body_count);
}

constraint_contact_convex_solve_cached :: #force_inline proc "contextless" (
	prestep_raw: rawptr, bodies: [^]$Body,
	impulses_raw: rawptr, active_mask: util.I32x8,
	$body_count, $contact_count: int,
	data: ^$Solve_Data,
	inverse_dt: f32 = 0,
)
{
	constraint_contact_convex_solve_cached_response(
		prestep_raw, bodies, impulses_raw, active_mask, body_count, contact_count,
		data, inverse_dt, nil, .Default,
	);
}

constraint_contact_convex_solve_cached_response :: #force_inline proc "contextless" (
	prestep_raw: rawptr, bodies: [^]$Body,
	impulses_raw: rawptr, active_mask: util.I32x8,
	$body_count, $contact_count: int,
	data: ^$Solve_Data,
	inverse_dt: f32, targets: [^]Restitution_Target_Wide,
	$response: Solver_Response_Mode,
)
{
	contacts: [^]Convex_Contact_Wide = ([^]Convex_Contact_Wide)(prestep_raw);
	cursor: uintptr = uintptr(prestep_raw) + uintptr(contact_count * size_of(Convex_Contact_Wide));
	offset_b: util.Vector3_Wide;
	when body_count == 2
	{
		offset_b = (^util.Vector3_Wide)(rawptr(cursor))^;
		cursor += uintptr(size_of(util.Vector3_Wide));
	}
	normal: util.Vector3_Wide = (^util.Vector3_Wide)(rawptr(cursor))^;
	cursor += uintptr(size_of(util.Vector3_Wide));
	material: ^Contact_Material_Wide = (^Contact_Material_Wide)(rawptr(cursor));
	tangent: ^util.Vector2_Wide = (^util.Vector2_Wide)(impulses_raw);
	penetrations: [^]util.F32x8 = ([^]util.F32x8)(rawptr(uintptr(impulses_raw) + uintptr(2 * size_of(util.F32x8))));
	twist: ^util.F32x8 = (^util.F32x8)(rawptr(uintptr(impulses_raw) + uintptr((2 + contact_count) * size_of(util.F32x8))));
	tangent_x, tangent_y: util.Vector3_Wide;
	friction_center_a: util.Vector3_Wide;
	when contact_count == 4
	{
		geometry: ^Contact_4_Geometry = &data.geometry;
		tangent_x, tangent_y = constraint_contact_4_build_basis(normal, geometry.basis_sign, geometry.basis_scale);
		friction_center_a = geometry.friction_center_a;
	}
	else
	{
		tangent_x, tangent_y = constraint_kernel_build_orthonormal_basis(normal);
		friction_center_a = constraint_contact_friction_center(contacts, contact_count);
	}
	friction_center_b: util.Vector3_Wide = util.vector3_wide_subtract(friction_center_a, offset_b);
	softness: util.F32x8 = data.softness;
	for contact_index in 0 ..< contact_count
	{
		normal_softness: util.F32x8 = softness;
		when response == .Restitution
		{
			normal_softness = util.wide_select_f32(targets[contact_index].active_mask, util.F32x8(0), softness);
		}
		contact_offset_b: util.Vector3_Wide = util.vector3_wide_subtract(contacts[contact_index].offset_a, offset_b);
		when contact_count == 4
		{
			depth: util.F32x8 = contacts[contact_index].depth;
			bias: util.F32x8 = simd.min(
				simd.mul(depth, util.F32x8(inverse_dt)),
				simd.min(simd.mul(depth, data.position_error_to_velocity), data.maximum_recovery_velocity)
			);
			when response == .Restitution
			{
				bias = util.wide_select_f32(targets[contact_index].active_mask,
					simd.max(bias, targets[contact_index].velocity), bias);
			}
			constraint_contact_penetration_solve_cached(
				bodies, normal, contacts[contact_index].offset_a, contact_offset_b, normal_softness,
				&penetrations[contact_index], active_mask, body_count,
				&data.penetration_effective_mass[contact_index], &bias,
			);
		}
		else
		{
			constraint_contact_penetration_solve_cached(
				bodies, normal, contacts[contact_index].offset_a, contact_offset_b, normal_softness,
				&penetrations[contact_index], active_mask, body_count,
				&data.penetration_effective_mass[contact_index], &data.penetration_bias[contact_index],
			);
		}
	}
	total_penetration: util.F32x8;
	maximum_twist_radius_impulse: util.F32x8;
	for contact_index in 0 ..< contact_count
	{
		total_penetration = simd.add(total_penetration, penetrations[contact_index]);
		when contact_count == 1
		{
			maximum_twist_radius_impulse = simd.mul(
				penetrations[contact_index],
				simd.max(util.F32x8(0), contacts[contact_index].depth)
			);
		}
		else
		{
			radius: util.F32x8;
			when contact_count == 4
			{
				radius = data.geometry.radii[contact_index];
			}
			else
			{
				radius = util.vector3_wide_length(util.vector3_wide_subtract(
						friction_center_a,
						contacts[contact_index].offset_a
				));
			}
			maximum_twist_radius_impulse = simd.add(
				maximum_twist_radius_impulse,
				simd.mul(penetrations[contact_index], radius)
			);
		}
	}
	constraint_contact_tangent_solve_cached(
		bodies, tangent_x, tangent_y, friction_center_a, friction_center_b,
		simd.mul(material.friction_coefficient, total_penetration), tangent, active_mask, body_count,
		&data.tangent_effective_mass,
	);
	constraint_contact_twist_solve_cached(
		bodies, normal, simd.mul(material.friction_coefficient, maximum_twist_radius_impulse),
		twist, active_mask, body_count, &data.twist_effective_mass,
	);
}

constraint_contact_nonconvex_kernel :: #force_inline proc "contextless" (
	prestep_raw: rawptr, bodies: [^]Constraint_Kernel_Body_Wide, dt, inverse_dt: f32,
	impulses_raw: rawptr, active_mask: util.I32x8, phase: Constraint_Kernel_Phase,
	$body_count, $contact_count: int,
)
{
	constraint_contact_nonconvex_kernel_response(
		prestep_raw, bodies, dt, inverse_dt, impulses_raw, active_mask, phase,
		body_count, contact_count, nil, .Default,
	);
}

constraint_contact_nonconvex_kernel_response :: #force_inline proc "contextless" (
	prestep_raw: rawptr, bodies: [^]Constraint_Kernel_Body_Wide, dt, inverse_dt: f32,
	impulses_raw: rawptr, active_mask: util.I32x8, phase: Constraint_Kernel_Phase,
	$body_count, $contact_count: int,
	targets: [^]Restitution_Target_Wide, $response: Solver_Response_Mode,
)
{
	material := (^Contact_Material_Wide)(prestep_raw);
	offset := size_of(Contact_Material_Wide);
	offset_b := util.Vector3_Wide{};
	when body_count == 2
	{
		offset_b = (^util.Vector3_Wide)(rawptr(uintptr(prestep_raw) + uintptr(offset)))^;
		offset += size_of(util.Vector3_Wide);
	}
	contacts := ([^]Nonconvex_Contact_Wide)(rawptr(uintptr(prestep_raw) + uintptr(offset)));
	contact_impulses := ([^]Nonconvex_Contact_Accumulated_Impulses)(impulses_raw);
	if phase == .Prestep
	{
		for contact_index in 0 ..< contact_count
		{
			contact_impulses[contact_index].tangent = constraint_kernel_mask_vector2(
				contact_impulses[contact_index].tangent,
				active_mask
			);
			contact_impulses[contact_index].penetration = constraint_kernel_mask_scalar(
				contact_impulses[contact_index].penetration,
				active_mask
			);
		}
		return;
	}
	if phase == .Incremental_Update
	{
		for contact_index in 0 ..< contact_count
		{
			contact_offset_b := util.vector3_wide_subtract(contacts[contact_index].offset, offset_b);
			constraint_contact_update_depth(
				bodies,
				contacts[contact_index].offset,
				contact_offset_b,
				contacts[contact_index].normal,
				dt,
				&contacts[contact_index].depth,
				active_mask,
				body_count
			);
		}
		return;
	}
	position_error_to_velocity, effective_mass_scale, softness := spring_settings_wide_compute(
		material.spring_settings,
		dt
	);
	for contact_index in 0 ..< contact_count
	{
		contact := &contacts[contact_index];
		impulse := &contact_impulses[contact_index];
		contact_offset_b := util.vector3_wide_subtract(contact.offset, offset_b);
		tangent_x, tangent_y := constraint_kernel_build_orthonormal_basis(contact.normal);
		if phase == .Warmstart
		{
			constraint_contact_tangent_apply(
				bodies,
				tangent_x,
				tangent_y,
				contact.offset,
				contact_offset_b,
				impulse.tangent,
				active_mask,
				body_count
			);
			constraint_contact_penetration_apply(
				bodies,
				contact.normal,
				contact.offset,
				contact_offset_b,
				impulse.penetration,
				active_mask,
				body_count
			);
		}
		else
		{
			when response == .Restitution
			{
				effective_mass, bias, normal_softness: util.F32x8;
				effective_mass, bias, normal_softness = restitution_prepare_normal(
					bodies, contact.normal, contact.offset, contact_offset_b, contact.depth,
					position_error_to_velocity, effective_mass_scale, material.maximum_recovery_velocity,
					softness, inverse_dt, &targets[contact_index], body_count,
				);
				constraint_contact_penetration_solve_cached(
					bodies, contact.normal, contact.offset, contact_offset_b, normal_softness,
					&impulse.penetration, active_mask, body_count, &effective_mass, &bias,
				);
			}
			else
			{
				constraint_contact_penetration_solve(
					bodies, contact.normal, contact.offset, contact_offset_b, contact.depth,
					position_error_to_velocity, effective_mass_scale, material.maximum_recovery_velocity, softness,
					dt, inverse_dt, &impulse.penetration, active_mask, body_count,
				);
			}
			constraint_contact_tangent_solve(
				bodies,
				tangent_x,
				tangent_y,
				contact.offset,
				contact_offset_b,
				simd.mul(material.friction_coefficient, impulse.penetration),
				&impulse.tangent,
				active_mask,
				body_count
			);
		}
	}
}

contact_1_one_body_kernel :: #force_inline proc "contextless" (
	p:rawptr,
	b:^[4]Constraint_Kernel_Body_Wide,
	dt,
	idt:f32,
	i:rawptr,
	m:util.I32x8,
	ph:Constraint_Kernel_Phase
)
{
	constraint_contact_convex_kernel(p, &b[0], dt, idt, i, m, ph, 1, 1);
}

contact_2_one_body_kernel :: #force_inline proc "contextless" (
	p:rawptr,
	b:^[4]Constraint_Kernel_Body_Wide,
	dt,
	idt:f32,
	i:rawptr,
	m:util.I32x8,
	ph:Constraint_Kernel_Phase
)
{
	constraint_contact_convex_kernel(p, &b[0], dt, idt, i, m, ph, 1, 2);
}

contact_3_one_body_kernel :: #force_inline proc "contextless" (
	p:rawptr,
	b:^[4]Constraint_Kernel_Body_Wide,
	dt,
	idt:f32,
	i:rawptr,
	m:util.I32x8,
	ph:Constraint_Kernel_Phase
)
{
	constraint_contact_convex_kernel(p, &b[0], dt, idt, i, m, ph, 1, 3);
}

contact_4_one_body_kernel :: #force_inline proc "contextless" (
	p:rawptr,
	b:^[4]Constraint_Kernel_Body_Wide,
	dt,
	idt:f32,
	i:rawptr,
	m:util.I32x8,
	ph:Constraint_Kernel_Phase
)
{
	constraint_contact_convex_kernel(p, &b[0], dt, idt, i, m, ph, 1, 4);
}

contact_1_kernel :: #force_inline proc "contextless" (
	p:rawptr,
	b:^[4]Constraint_Kernel_Body_Wide,
	dt,
	idt:f32,
	i:rawptr,
	m:util.I32x8,
	ph:Constraint_Kernel_Phase
)
{
	constraint_contact_convex_kernel(p, &b[0], dt, idt, i, m, ph, 2, 1);
}

contact_2_kernel :: #force_inline proc "contextless" (
	p:rawptr,
	b:^[4]Constraint_Kernel_Body_Wide,
	dt,
	idt:f32,
	i:rawptr,
	m:util.I32x8,
	ph:Constraint_Kernel_Phase
)
{
	constraint_contact_convex_kernel(p, &b[0], dt, idt, i, m, ph, 2, 2);
}

contact_3_kernel :: #force_inline proc "contextless" (
	p:rawptr,
	b:^[4]Constraint_Kernel_Body_Wide,
	dt,
	idt:f32,
	i:rawptr,
	m:util.I32x8,
	ph:Constraint_Kernel_Phase
)
{
	constraint_contact_convex_kernel(p, &b[0], dt, idt, i, m, ph, 2, 3);
}

contact_4_kernel :: #force_inline proc "contextless" (
	p:rawptr,
	b:^[4]Constraint_Kernel_Body_Wide,
	dt,
	idt:f32,
	i:rawptr,
	m:util.I32x8,
	ph:Constraint_Kernel_Phase
)
{
	constraint_contact_convex_kernel(p, &b[0], dt, idt, i, m, ph, 2, 4);
}

contact_2_nonconvex_one_body_kernel :: #force_inline proc "contextless" (
	p:rawptr,
	b:^[4]Constraint_Kernel_Body_Wide,
	dt,
	idt:f32,
	i:rawptr,
	m:util.I32x8,
	ph:Constraint_Kernel_Phase
)
{
	constraint_contact_nonconvex_kernel(p, &b[0], dt, idt, i, m, ph, 1, 2);
}

contact_3_nonconvex_one_body_kernel :: #force_inline proc "contextless" (
	p:rawptr,
	b:^[4]Constraint_Kernel_Body_Wide,
	dt,
	idt:f32,
	i:rawptr,
	m:util.I32x8,
	ph:Constraint_Kernel_Phase
)
{
	constraint_contact_nonconvex_kernel(p, &b[0], dt, idt, i, m, ph, 1, 3);
}

contact_4_nonconvex_one_body_kernel :: #force_inline proc "contextless" (
	p:rawptr,
	b:^[4]Constraint_Kernel_Body_Wide,
	dt,
	idt:f32,
	i:rawptr,
	m:util.I32x8,
	ph:Constraint_Kernel_Phase
)
{
	constraint_contact_nonconvex_kernel(p, &b[0], dt, idt, i, m, ph, 1, 4);
}

contact_2_nonconvex_kernel :: #force_inline proc "contextless" (
	p:rawptr,
	b:^[4]Constraint_Kernel_Body_Wide,
	dt,
	idt:f32,
	i:rawptr,
	m:util.I32x8,
	ph:Constraint_Kernel_Phase
)
{
	constraint_contact_nonconvex_kernel(p, &b[0], dt, idt, i, m, ph, 2, 2);
}

contact_3_nonconvex_kernel :: #force_inline proc "contextless" (
	p:rawptr,
	b:^[4]Constraint_Kernel_Body_Wide,
	dt,
	idt:f32,
	i:rawptr,
	m:util.I32x8,
	ph:Constraint_Kernel_Phase
)
{
	constraint_contact_nonconvex_kernel(p, &b[0], dt, idt, i, m, ph, 2, 3);
}

contact_4_nonconvex_kernel :: #force_inline proc "contextless" (
	p:rawptr,
	b:^[4]Constraint_Kernel_Body_Wide,
	dt,
	idt:f32,
	i:rawptr,
	m:util.I32x8,
	ph:Constraint_Kernel_Phase
)
{
	constraint_contact_nonconvex_kernel(p, &b[0], dt, idt, i, m, ph, 2, 4);
}

one_body_angular_motor_kernel :: #force_inline proc "contextless" (
	prestep_raw: rawptr, bodies: ^[4]Constraint_Kernel_Body_Wide, dt, inverse_dt: f32,
	impulses_raw: rawptr, active_mask: util.I32x8, phase: Constraint_Kernel_Phase,
)
{
	_ = inverse_dt;
	prestep := (^One_Body_Angular_Motor_Prestep)(prestep_raw);
	impulses := (^util.Vector3_Wide)(impulses_raw);
	if phase == .Prestep
	{
		constraint_kernel_prepare_vector3_impulses(impulses, active_mask);
		return;
	}
	if phase == .Incremental_Update
	{
		return;
	}
	if phase == .Warmstart
	{
		constraint_kernel_apply_angular(&bodies[0], impulses^, 1, active_mask);
		return;
	}
	effective_mass_scale, softness, maximum_impulse := motor_settings_wide_compute(prestep.settings, dt);
	effective_mass := util.symmetric3x3_wide_invert(bodies[0].inverse_inertia);
	csv := util.vector3_wide_subtract(prestep.target_velocity, bodies[0].angular_velocity);
	csi := util.symmetric3x3_wide_transform(csv, effective_mass);
	csi = util.vector3_wide_subtract(
		util.vector3_wide_scale(csi, effective_mass_scale),
		util.vector3_wide_scale(impulses^, softness)
	);
	constraint_kernel_clamp_accumulated_vector3(maximum_impulse, impulses, &csi, active_mask);
	constraint_kernel_apply_angular(&bodies[0], csi, 1, active_mask);
}

one_body_angular_servo_kernel :: #force_inline proc "contextless" (
	prestep_raw: rawptr, bodies: ^[4]Constraint_Kernel_Body_Wide, dt, inverse_dt: f32,
	impulses_raw: rawptr, active_mask: util.I32x8, phase: Constraint_Kernel_Phase,
)
{
	prestep := (^One_Body_Angular_Servo_Prestep)(prestep_raw);
	impulses := (^util.Vector3_Wide)(impulses_raw);
	if phase == .Prestep
	{
		constraint_kernel_prepare_vector3_impulses(impulses, active_mask);
		return;
	}
	if phase == .Incremental_Update
	{
		return;
	}
	if phase == .Warmstart
	{
		constraint_kernel_apply_angular(&bodies[0], impulses^, 1, active_mask);
		return;
	}
	inverse_orientation := util.quaternion_wide_conjugate(bodies[0].orientation);
	error_rotation := util.quaternion_wide_concatenate(inverse_orientation, prestep.target_orientation);
	error_axis, error_length := util.quaternion_wide_axis_angle(error_rotation);
	position_error_to_velocity, effective_mass_scale, softness := spring_settings_wide_compute(
		prestep.spring_settings,
		dt
	);
	bias, maximum_impulse := servo_settings_wide_compute_bias_vector3(
		util.vector3_wide_scale(error_axis, error_length), position_error_to_velocity,
		prestep.servo_settings, dt, inverse_dt,
	);
	csi := util.symmetric3x3_wide_transform(
		util.vector3_wide_subtract(bias, bodies[0].angular_velocity),
		util.symmetric3x3_wide_invert(bodies[0].inverse_inertia),
	);
	csi = util.vector3_wide_subtract(
		util.vector3_wide_scale(csi, effective_mass_scale),
		util.vector3_wide_scale(impulses^, softness)
	);
	constraint_kernel_clamp_accumulated_vector3(maximum_impulse, impulses, &csi, active_mask);
	constraint_kernel_apply_angular(&bodies[0], csi, 1, active_mask);
}

one_body_linear_motor_kernel :: #force_inline proc "contextless" (
	prestep_raw: rawptr, bodies: ^[4]Constraint_Kernel_Body_Wide, dt, inverse_dt: f32,
	impulses_raw: rawptr, active_mask: util.I32x8, phase: Constraint_Kernel_Phase,
)
{
	_ = inverse_dt;
	prestep := (^One_Body_Linear_Motor_Prestep)(prestep_raw);
	impulses := (^util.Vector3_Wide)(impulses_raw);
	if phase == .Prestep
	{
		constraint_kernel_prepare_vector3_impulses(impulses, active_mask);
		return;
	}
	if phase == .Incremental_Update
	{
		return;
	}
	offset := util.quaternion_wide_transform(prestep.local_offset, bodies[0].orientation);
	if phase == .Warmstart
	{
		constraint_kernel_apply_point_one_body(&bodies[0], offset, impulses^, active_mask);
		return;
	}
	effective_mass_scale, softness, maximum_impulse := motor_settings_wide_compute(prestep.settings, dt);
	point_velocity := util.vector3_wide_add(
		bodies[0].linear_velocity,
		util.vector3_wide_cross(bodies[0].angular_velocity, offset)
	);
	csv := util.vector3_wide_subtract(prestep.target_velocity, point_velocity);
	csi := util.symmetric3x3_wide_transform(
		csv,
		constraint_kernel_one_body_point_effective_mass(&bodies[0], offset, effective_mass_scale)
	);
	csi = util.vector3_wide_subtract(csi, util.vector3_wide_scale(impulses^, softness));
	constraint_kernel_clamp_accumulated_vector3(maximum_impulse, impulses, &csi, active_mask);
	constraint_kernel_apply_point_one_body(&bodies[0], offset, csi, active_mask);
}

one_body_linear_servo_kernel :: #force_inline proc "contextless" (
	prestep_raw: rawptr, bodies: ^[4]Constraint_Kernel_Body_Wide, dt, inverse_dt: f32,
	impulses_raw: rawptr, active_mask: util.I32x8, phase: Constraint_Kernel_Phase,
)
{
	prestep := (^One_Body_Linear_Servo_Prestep)(prestep_raw);
	impulses := (^util.Vector3_Wide)(impulses_raw);
	if phase == .Prestep
	{
		constraint_kernel_prepare_vector3_impulses(impulses, active_mask);
		return;
	}
	if phase == .Incremental_Update
	{
		return;
	}
	offset := util.quaternion_wide_transform(prestep.local_offset, bodies[0].orientation);
	if phase == .Warmstart
	{
		constraint_kernel_apply_point_one_body(&bodies[0], offset, impulses^, active_mask);
		return;
	}
	position_error_to_velocity, effective_mass_scale, softness := spring_settings_wide_compute(
		prestep.spring_settings,
		dt
	);
	world_point := util.vector3_wide_add(bodies[0].position, offset);
	error := util.vector3_wide_subtract(prestep.target, world_point);
	bias, maximum_impulse := servo_settings_wide_compute_bias_vector3(
		error,
		position_error_to_velocity,
		prestep.servo_settings,
		dt,
		inverse_dt
	);
	point_velocity := util.vector3_wide_add(
		bodies[0].linear_velocity,
		util.vector3_wide_cross(bodies[0].angular_velocity, offset)
	);
	csi := util.symmetric3x3_wide_transform(
		util.vector3_wide_subtract(bias, point_velocity),
		constraint_kernel_one_body_point_effective_mass(&bodies[0], offset, effective_mass_scale),
	);
	csi = util.vector3_wide_subtract(csi, util.vector3_wide_scale(impulses^, softness));
	constraint_kernel_clamp_accumulated_vector3(maximum_impulse, impulses, &csi, active_mask);
	constraint_kernel_apply_point_one_body(&bodies[0], offset, csi, active_mask);
}

constraint_kernel_ball_socket_solve :: #force_inline proc "contextless" (
	body_a, body_b: ^Constraint_Kernel_Body_Wide, offset_a, offset_b, bias: util.Vector3_Wide,
	effective_mass: util.Symmetric3x3_Wide, softness: util.F32x8, maximum_impulse: util.F32x8,
	impulses: ^util.Vector3_Wide, active_mask: util.I32x8, limit: Reference_State,
)
{
	csv: util.Vector3_Wide = util.vector3_wide_subtract(body_a.linear_velocity, body_b.linear_velocity);
	csv = util.vector3_wide_add(csv, util.vector3_wide_cross(body_a.angular_velocity, offset_a));
	csv = util.vector3_wide_add(csv, util.vector3_wide_cross(offset_b, body_b.angular_velocity));
	csi: util.Vector3_Wide = util.symmetric3x3_wide_transform(util.vector3_wide_subtract(bias, csv), effective_mass);
	csi = util.vector3_wide_subtract(csi, util.vector3_wide_scale(impulses^, softness));
	if limit == .Present
	{
		constraint_kernel_clamp_accumulated_vector3(maximum_impulse, impulses, &csi, active_mask);
	}
	else
	{
		impulses^ = constraint_kernel_mask_vector3(util.vector3_wide_add(impulses^, csi), active_mask);
		csi = constraint_kernel_mask_vector3(csi, active_mask);
	}
	constraint_kernel_apply_ball_socket(body_a, body_b, offset_a, offset_b, csi, active_mask);
}

constraint_kernel_ball_socket_prepare_solve :: #force_inline proc "contextless" (
	prestep: ^Ball_Socket_Prestep, body_a, body_b: ^Constraint_Kernel_Body_Wide,
	dt: f32, data: ^Ball_Socket_Solve_Data,
)
{
	data.offset_a = util.quaternion_wide_transform(prestep.local_offset_a, body_a.orientation);
	data.offset_b = util.quaternion_wide_transform(prestep.local_offset_b, body_b.orientation);
	position_error_to_velocity, effective_mass_scale, softness: util.F32x8 = spring_settings_wide_compute(
		prestep.spring_settings, dt,
	);
	error: util.Vector3_Wide = util.vector3_wide_subtract(
		util.vector3_wide_add(util.vector3_wide_subtract(body_b.position, body_a.position), data.offset_b),
		data.offset_a,
	);
	data.bias = util.vector3_wide_scale(error, position_error_to_velocity);
	data.effective_mass = constraint_kernel_ball_socket_effective_mass(
		body_a, body_b, data.offset_a, data.offset_b, effective_mass_scale,
	);
	data.softness = softness;
}

constraint_kernel_ball_socket :: #force_inline proc "contextless" (
	prestep: ^Ball_Socket_Prestep, body_a, body_b: ^Constraint_Kernel_Body_Wide,
	dt: f32, impulses: ^util.Vector3_Wide, active_mask: util.I32x8,
	phase: Constraint_Kernel_Phase,
)
{
	if phase == .Prestep
	{
		constraint_kernel_prepare_vector3_impulses(impulses, active_mask);
		return;
	}
	if phase == .Incremental_Update
	{
		return;
	}
	if phase == .Warmstart
	{
		offset_a: util.Vector3_Wide = util.quaternion_wide_transform(prestep.local_offset_a, body_a.orientation);
		offset_b: util.Vector3_Wide = util.quaternion_wide_transform(prestep.local_offset_b, body_b.orientation);
		constraint_kernel_apply_ball_socket(body_a, body_b, offset_a, offset_b, impulses^, active_mask);
		return;
	}
	// registered and fallback callers do not maintain a cache. recompute for each Solve
	data: Ball_Socket_Solve_Data = ---;
	constraint_kernel_ball_socket_prepare_solve(prestep, body_a, body_b, dt, &data);
	constraint_kernel_ball_socket_solve(
		body_a, body_b, data.offset_a, data.offset_b, data.bias,
		data.effective_mass, data.softness, {}, impulses, active_mask, .Missing,
	);
}

ball_socket_kernel :: #force_inline proc "contextless" (
	prestep_raw: rawptr, bodies: ^[4]Constraint_Kernel_Body_Wide, dt, inverse_dt: f32,
	impulses_raw: rawptr, active_mask: util.I32x8, phase: Constraint_Kernel_Phase,
)
{
	_ = inverse_dt;
	constraint_kernel_ball_socket(
		(^Ball_Socket_Prestep)(prestep_raw), &bodies[0], &bodies[1], dt,
		(^util.Vector3_Wide)(impulses_raw), active_mask, phase,
	);
}

ball_socket_motor_kernel :: #force_inline proc "contextless" (
	prestep_raw: rawptr, bodies: ^[4]Constraint_Kernel_Body_Wide, dt, inverse_dt: f32,
	impulses_raw: rawptr, active_mask: util.I32x8, phase: Constraint_Kernel_Phase,
)
{
	_ = inverse_dt;
	prestep := (^Ball_Socket_Motor_Prestep)(prestep_raw);
	impulses := (^util.Vector3_Wide)(impulses_raw);
	if phase == .Prestep
	{
		constraint_kernel_prepare_vector3_impulses(impulses, active_mask);
		return;
	}
	if phase == .Incremental_Update
	{
		return;
	}
	offset_b := util.quaternion_wide_transform(prestep.local_offset_b, bodies[1].orientation);
	offset_a := util.vector3_wide_add(util.vector3_wide_subtract(bodies[1].position, bodies[0].position), offset_b);
	if phase == .Warmstart
	{
		constraint_kernel_apply_ball_socket(&bodies[0], &bodies[1], offset_a, offset_b, impulses^, active_mask);
		return;
	}
	effective_mass_scale, softness, maximum_impulse := motor_settings_wide_compute(prestep.settings, dt);
	bias := util.vector3_wide_negate(util.quaternion_wide_transform(
			prestep.target_velocity_local_a,
			bodies[0].orientation
	));
	constraint_kernel_ball_socket_solve(
		&bodies[0], &bodies[1], offset_a, offset_b, bias,
		constraint_kernel_ball_socket_effective_mass(&bodies[0], &bodies[1], offset_a, offset_b, effective_mass_scale),
		softness, maximum_impulse, impulses, active_mask, .Present,
	);
}

ball_socket_servo_kernel :: #force_inline proc "contextless" (
	prestep_raw: rawptr, bodies: ^[4]Constraint_Kernel_Body_Wide, dt, inverse_dt: f32,
	impulses_raw: rawptr, active_mask: util.I32x8, phase: Constraint_Kernel_Phase,
)
{
	prestep := (^Ball_Socket_Servo_Prestep)(prestep_raw);
	impulses := (^util.Vector3_Wide)(impulses_raw);
	if phase == .Prestep
	{
		constraint_kernel_prepare_vector3_impulses(impulses, active_mask);
		return;
	}
	if phase == .Incremental_Update
	{
		return;
	}
	offset_a := util.quaternion_wide_transform(prestep.local_offset_a, bodies[0].orientation);
	offset_b := util.quaternion_wide_transform(prestep.local_offset_b, bodies[1].orientation);
	if phase == .Warmstart
	{
		constraint_kernel_apply_ball_socket(&bodies[0], &bodies[1], offset_a, offset_b, impulses^, active_mask);
		return;
	}
	position_error_to_velocity, effective_mass_scale, softness := spring_settings_wide_compute(
		prestep.spring_settings,
		dt
	);
	error := util.vector3_wide_subtract(
		util.vector3_wide_add(util.vector3_wide_subtract(bodies[1].position, bodies[0].position), offset_b),
		offset_a
	);
	bias, maximum_impulse := servo_settings_wide_compute_bias_vector3(
		error,
		position_error_to_velocity,
		prestep.servo_settings,
		dt,
		inverse_dt
	);
	constraint_kernel_ball_socket_solve(
		&bodies[0], &bodies[1], offset_a, offset_b, bias,
		constraint_kernel_ball_socket_effective_mass(&bodies[0], &bodies[1], offset_a, offset_b, effective_mass_scale),
		softness, maximum_impulse, impulses, active_mask, .Present,
	);
}

angular_axis_gear_motor_kernel :: #force_inline proc "contextless" (
	prestep_raw:rawptr, bodies:^[4]Constraint_Kernel_Body_Wide, dt, inverse_dt:f32,
	impulses_raw:rawptr, active_mask:util.I32x8, phase:Constraint_Kernel_Phase,
)
{
	_ = inverse_dt;
	prestep := (^Angular_Axis_Gear_Motor_Prestep)(prestep_raw);
	impulses := (^util.F32x8)(impulses_raw);
	if phase == .Prestep
	{
		constraint_kernel_prepare_scalar_impulses(impulses, active_mask);
		return;
	}
	if phase == .Incremental_Update
	{
		return;
	}
	axis := util.quaternion_wide_transform(prestep.local_axis_a, bodies[0].orientation);
	jacobian_a := util.vector3_wide_scale(axis, prestep.velocity_scale);
	if phase == .Warmstart
	{
		constraint_kernel_apply_angular(&bodies[0], util.vector3_wide_scale(jacobian_a, impulses^), 1, active_mask);
		constraint_kernel_apply_angular(&bodies[1], util.vector3_wide_scale(axis, impulses^), -1, active_mask);
		return;
	}
	impulse_to_velocity_a := util.symmetric3x3_wide_transform(jacobian_a, bodies[0].inverse_inertia);
	impulse_to_velocity_b := util.symmetric3x3_wide_transform(axis, bodies[1].inverse_inertia);
	inverse_effective_mass := simd.add(
		util.vector3_wide_dot(jacobian_a, impulse_to_velocity_a),
		util.vector3_wide_dot(axis, impulse_to_velocity_b)
	);
	effective_mass_scale, softness, maximum_impulse := motor_settings_wide_compute(prestep.settings, dt);
	csv_a := util.vector3_wide_dot(bodies[0].angular_velocity, jacobian_a);
	csv_b := util.vector3_wide_dot(bodies[1].angular_velocity, axis);
	csi := simd.sub(
		simd.mul(simd.sub(csv_b, csv_a), simd.div(effective_mass_scale, inverse_effective_mass)),
		simd.mul(impulses^, softness)
	);
	constraint_kernel_clamp_accumulated_scalar(maximum_impulse, impulses, &csi, active_mask);
	constraint_kernel_apply_angular(&bodies[0], util.vector3_wide_scale(jacobian_a, csi), 1, active_mask);
	constraint_kernel_apply_angular(&bodies[1], util.vector3_wide_scale(axis, csi), -1, active_mask);
}

angular_axis_motor_kernel :: #force_inline proc "contextless" (
	prestep_raw:rawptr, bodies:^[4]Constraint_Kernel_Body_Wide, dt, inverse_dt:f32,
	impulses_raw:rawptr, active_mask:util.I32x8, phase:Constraint_Kernel_Phase,
)
{
	_ = inverse_dt;
	prestep := (^Angular_Axis_Motor_Prestep)(prestep_raw);
	impulses := (^util.F32x8)(impulses_raw);
	if phase == .Prestep
	{
		constraint_kernel_prepare_scalar_impulses(impulses, active_mask);
		return;
	}
	if phase == .Incremental_Update
	{
		return;
	}
	axis := util.quaternion_wide_transform(prestep.local_axis_a, bodies[0].orientation);
	if phase == .Warmstart
	{
		world_impulse := util.vector3_wide_scale(axis, impulses^);
		constraint_kernel_apply_angular(&bodies[0], world_impulse, 1, active_mask);
		constraint_kernel_apply_angular(&bodies[1], world_impulse, -1, active_mask);
		return;
	}
	impulse_to_velocity_a := util.symmetric3x3_wide_transform(axis, bodies[0].inverse_inertia);
	impulse_to_velocity_b := util.symmetric3x3_wide_transform(axis, bodies[1].inverse_inertia);
	inverse_effective_mass := simd.add(
		util.vector3_wide_dot(axis, impulse_to_velocity_a),
		util.vector3_wide_dot(axis, impulse_to_velocity_b)
	);
	effective_mass_scale, softness, maximum_impulse := motor_settings_wide_compute(prestep.settings, dt);
	relative_velocity := util.vector3_wide_dot(
		util.vector3_wide_subtract(bodies[1].angular_velocity, bodies[0].angular_velocity),
		axis
	);
	csi := simd.sub(
		simd.mul(
			simd.add(prestep.target_velocity, relative_velocity),
			simd.div(effective_mass_scale, inverse_effective_mass)
		),
		simd.mul(impulses^, softness)
	);
	constraint_kernel_clamp_accumulated_scalar(maximum_impulse, impulses, &csi, active_mask);
	world_impulse := util.vector3_wide_scale(axis, csi);
	constraint_kernel_apply_angular(&bodies[0], world_impulse, 1, active_mask);
	constraint_kernel_apply_angular(&bodies[1], world_impulse, -1, active_mask);
}

constraint_kernel_angular_hinge_jacobian :: #force_inline proc "contextless" (
	prestep: ^Angular_Hinge_Prestep, bodies: ^[4]Constraint_Kernel_Body_Wide,
) -> (hinge_axis_a, hinge_axis_b: util.Vector3_Wide, jacobian: util.Matrix2x3_Wide)
{
	local_x, local_y := constraint_kernel_build_orthonormal_basis(prestep.local_hinge_axis_a);
	hinge_axis_a = util.quaternion_wide_transform(prestep.local_hinge_axis_a, bodies[0].orientation);
	hinge_axis_b = util.quaternion_wide_transform(prestep.local_hinge_axis_b, bodies[1].orientation);
	jacobian = {
		util.quaternion_wide_transform(local_x, bodies[0].orientation),
		util.quaternion_wide_transform(local_y, bodies[0].orientation),
	};
	return;
}

constraint_kernel_angular_hinge_error :: #force_inline proc "contextless" (
	hinge_axis_a, hinge_axis_b: util.Vector3_Wide, jacobian: util.Matrix2x3_Wide,
) -> util.Vector2_Wide
{
	dot_x := util.vector3_wide_dot(hinge_axis_b, jacobian.x);
	dot_y := util.vector3_wide_dot(hinge_axis_b, jacobian.y);
	on_plane_x := util.vector3_wide_subtract(hinge_axis_b, util.vector3_wide_scale(jacobian.x, dot_x));
	on_plane_y := util.vector3_wide_subtract(hinge_axis_b, util.vector3_wide_scale(jacobian.y, dot_y));
	length_x := util.vector3_wide_length(on_plane_x);
	length_y := util.vector3_wide_length(on_plane_y);
	on_plane_x = util.vector3_wide_scale(on_plane_x, simd.div(util.F32x8(1), length_x));
	on_plane_y = util.vector3_wide_scale(on_plane_y, simd.div(util.F32x8(1), length_y));
	on_plane_x = util.vector3_wide_select(
		transmute(util.I32x8)simd.lanes_lt(length_x, util.F32x8(1e-7)),
		hinge_axis_a,
		on_plane_x
	);
	on_plane_y = util.vector3_wide_select(
		transmute(util.I32x8)simd.lanes_lt(length_y, util.F32x8(1e-7)),
		hinge_axis_a,
		on_plane_y
	);
	error_x := util.acos_approx_wide(util.vector3_wide_dot(on_plane_x, hinge_axis_a));
	error_y := util.acos_approx_wide(util.vector3_wide_dot(on_plane_y, hinge_axis_a));
	error_x = util.wide_select_f32(
		transmute(util.I32x8)simd.lanes_lt(util.vector3_wide_dot(on_plane_x, jacobian.y), util.F32x8(0)),
		error_x,
		simd.neg(error_x)
	);
	error_y = util.wide_select_f32(
		transmute(util.I32x8)simd.lanes_lt(util.vector3_wide_dot(on_plane_y, jacobian.x), util.F32x8(0)),
		simd.neg(error_y),
		error_y
	);
	return {error_x, error_y};
}

angular_hinge_kernel :: #force_inline proc "contextless" (
	prestep_raw: rawptr, bodies: ^[4]Constraint_Kernel_Body_Wide, dt, inverse_dt: f32,
	impulses_raw: rawptr, active_mask: util.I32x8, phase: Constraint_Kernel_Phase,
)
{
	_ = inverse_dt;
	prestep := (^Angular_Hinge_Prestep)(prestep_raw);
	impulses := (^util.Vector2_Wide)(impulses_raw);
	if phase == .Prestep
	{
		impulses^ = constraint_kernel_mask_vector2(impulses^, active_mask);
		return;
	}
	if phase == .Incremental_Update
	{
		return;
	}
	hinge_axis_a, hinge_axis_b, jacobian := constraint_kernel_angular_hinge_jacobian(prestep, bodies);
	if phase == .Warmstart
	{
		world_impulse := util.matrix2x3_wide_transform(impulses^, jacobian);
		constraint_kernel_apply_angular(&bodies[0], world_impulse, 1, active_mask);
		constraint_kernel_apply_angular(&bodies[1], world_impulse, -1, active_mask);
		return;
	}
	impulse_to_velocity_a := util.matrix2x3_wide_multiply_symmetric3x3(jacobian, bodies[0].inverse_inertia);
	impulse_to_velocity_b := util.matrix2x3_wide_multiply_symmetric3x3(jacobian, bodies[1].inverse_inertia);
	inverse_effective_mass := util.symmetric2x2_wide_add(
		util.symmetric2x2_wide_complete_matrix_sandwich(impulse_to_velocity_a, jacobian),
		util.symmetric2x2_wide_complete_matrix_sandwich(impulse_to_velocity_b, jacobian),
	);
	effective_mass := util.symmetric2x2_wide_invert(inverse_effective_mass);
	position_error_to_velocity, effective_mass_scale, softness := spring_settings_wide_compute(
		prestep.spring_settings,
		dt
	);
	error := constraint_kernel_angular_hinge_error(hinge_axis_a, hinge_axis_b, jacobian);
	bias := util.vector2_wide_scale(error, simd.neg(position_error_to_velocity));
	csv := util.matrix2x3_wide_transform_by_transpose(
		util.vector3_wide_subtract(bodies[0].angular_velocity, bodies[1].angular_velocity),
		jacobian
	);
	csi := util.symmetric2x2_wide_transform(util.vector2_wide_subtract(bias, csv), effective_mass);
	csi = util.vector2_wide_subtract(
		util.vector2_wide_scale(csi, effective_mass_scale),
		util.vector2_wide_scale(impulses^, softness)
	);
	csi = constraint_kernel_mask_vector2(csi, active_mask);
	impulses^ = constraint_kernel_mask_vector2(util.vector2_wide_add(impulses^, csi), active_mask);
	world_impulse := util.matrix2x3_wide_transform(csi, jacobian);
	constraint_kernel_apply_angular(&bodies[0], world_impulse, 1, active_mask);
	constraint_kernel_apply_angular(&bodies[1], world_impulse, -1, active_mask);
}

angular_motor_kernel :: #force_inline proc "contextless" (
	prestep_raw:rawptr, bodies:^[4]Constraint_Kernel_Body_Wide, dt, inverse_dt:f32,
	impulses_raw:rawptr, active_mask:util.I32x8, phase:Constraint_Kernel_Phase,
)
{
	_ = inverse_dt;
	prestep := (^Angular_Motor_Prestep)(prestep_raw);
	impulses := (^util.Vector3_Wide)(impulses_raw);
	if phase == .Prestep
	{
		constraint_kernel_prepare_vector3_impulses(impulses, active_mask);
		return;
	}
	if phase == .Incremental_Update
	{
		return;
	}
	if phase == .Warmstart
	{
		constraint_kernel_apply_angular(&bodies[0], impulses^, 1, active_mask);
		constraint_kernel_apply_angular(&bodies[1], impulses^, -1, active_mask);
		return;
	}
	effective_mass_scale, softness, maximum_impulse := motor_settings_wide_compute(prestep.settings, dt);
	effective_mass := util.symmetric3x3_wide_invert(util.symmetric3x3_wide_add(
			bodies[0].inverse_inertia,
			bodies[1].inverse_inertia
	));
	bias := util.quaternion_wide_transform(prestep.target_velocity_local_a, bodies[0].orientation);
	csv := util.vector3_wide_subtract(bodies[0].angular_velocity, bodies[1].angular_velocity);
	csi := util.symmetric3x3_wide_transform(util.vector3_wide_subtract(bias, csv), effective_mass);
	csi = util.vector3_wide_subtract(
		util.vector3_wide_scale(csi, effective_mass_scale),
		util.vector3_wide_scale(impulses^, softness)
	);
	constraint_kernel_clamp_accumulated_vector3(maximum_impulse, impulses, &csi, active_mask);
	constraint_kernel_apply_angular(&bodies[0], csi, 1, active_mask);
	constraint_kernel_apply_angular(&bodies[1], csi, -1, active_mask);
}

angular_servo_kernel :: #force_inline proc "contextless" (
	prestep_raw:rawptr, bodies:^[4]Constraint_Kernel_Body_Wide, dt, inverse_dt:f32,
	impulses_raw:rawptr, active_mask:util.I32x8, phase:Constraint_Kernel_Phase,
)
{
	prestep := (^Angular_Servo_Prestep)(prestep_raw);
	impulses := (^util.Vector3_Wide)(impulses_raw);
	if phase == .Prestep
	{
		constraint_kernel_prepare_vector3_impulses(impulses, active_mask);
		return;
	}
	if phase == .Incremental_Update
	{
		return;
	}
	if phase == .Warmstart
	{
		constraint_kernel_apply_angular(&bodies[0], impulses^, 1, active_mask);
		constraint_kernel_apply_angular(&bodies[1], impulses^, -1, active_mask);
		return;
	}
	target_b := util.quaternion_wide_concatenate(prestep.target_relative_rotation_local_a, bodies[0].orientation);
	error_rotation := util.quaternion_wide_concatenate(util.quaternion_wide_conjugate(target_b), bodies[1].orientation);
	error_axis, error_length := util.quaternion_wide_axis_angle(error_rotation);
	position_error_to_velocity, effective_mass_scale, softness := spring_settings_wide_compute(
		prestep.spring_settings,
		dt
	);
	bias, maximum_impulse := servo_settings_wide_compute_bias_vector3(
		util.vector3_wide_scale(error_axis, error_length), position_error_to_velocity,
		prestep.servo_settings, dt, inverse_dt,
	);
	effective_mass := util.symmetric3x3_wide_invert(util.symmetric3x3_wide_add(
			bodies[0].inverse_inertia,
			bodies[1].inverse_inertia
	));
	csv := util.vector3_wide_subtract(bodies[0].angular_velocity, bodies[1].angular_velocity);
	csi := util.symmetric3x3_wide_transform(util.vector3_wide_subtract(bias, csv), effective_mass);
	csi = util.vector3_wide_subtract(
		util.vector3_wide_scale(csi, effective_mass_scale),
		util.vector3_wide_scale(impulses^, softness)
	);
	constraint_kernel_clamp_accumulated_vector3(maximum_impulse, impulses, &csi, active_mask);
	constraint_kernel_apply_angular(&bodies[0], csi, 1, active_mask);
	constraint_kernel_apply_angular(&bodies[1], csi, -1, active_mask);
}

constraint_kernel_cross_axis_jacobian :: #force_inline proc "contextless" (
	axis_local_a, axis_local_b: util.Vector3_Wide, bodies: ^[4]Constraint_Kernel_Body_Wide, perpendicular_threshold: f32,
) -> (axis_a, axis_b, jacobian: util.Vector3_Wide)
{
	axis_a = util.quaternion_wide_transform(axis_local_a, bodies[0].orientation);
	axis_b = util.quaternion_wide_transform(axis_local_b, bodies[1].orientation);
	jacobian = util.vector3_wide_cross(axis_a, axis_b);
	perpendicular_axis, _ := constraint_kernel_build_orthonormal_basis(axis_a);
	use_perpendicular_axis := transmute(util.I32x8)simd.lanes_lt(
		util.vector3_wide_length_squared(jacobian),
		util.F32x8(perpendicular_threshold)
	);
	jacobian = util.vector3_wide_select(use_perpendicular_axis, perpendicular_axis, jacobian);
	return;
}

constraint_kernel_solve_cross_axis :: #force_inline proc "contextless" (
	bodies: ^[4]Constraint_Kernel_Body_Wide, jacobian, bias: util.Vector3_Wide,
	dt: f32, spring: Spring_Settings_Wide, impulses: ^util.F32x8, active_mask: util.I32x8,
)
{
	impulse_to_velocity_a := util.symmetric3x3_wide_transform(jacobian, bodies[0].inverse_inertia);
	impulse_to_velocity_b := util.symmetric3x3_wide_transform(jacobian, bodies[1].inverse_inertia);
	inverse_effective_mass := simd.add(
		util.vector3_wide_dot(impulse_to_velocity_a, jacobian),
		util.vector3_wide_dot(impulse_to_velocity_b, jacobian)
	);
	_, effective_mass_scale, softness := spring_settings_wide_compute(spring, dt);
	effective_mass := simd.div(effective_mass_scale, inverse_effective_mass);
	csv := util.vector3_wide_dot(
		util.vector3_wide_subtract(bodies[0].angular_velocity, bodies[1].angular_velocity),
		jacobian
	);
	csi := simd.sub(simd.mul(simd.sub(bias.x, csv), effective_mass), simd.mul(impulses^, softness));
	csi = constraint_kernel_mask_scalar(csi, active_mask);
	impulses^ = constraint_kernel_mask_scalar(simd.add(impulses^, csi), active_mask);
	world_impulse := util.vector3_wide_scale(jacobian, csi);
	constraint_kernel_apply_angular(&bodies[0], world_impulse, 1, active_mask);
	constraint_kernel_apply_angular(&bodies[1], world_impulse, -1, active_mask);
}

angular_swivel_hinge_kernel :: #force_inline proc "contextless" (
	prestep_raw: rawptr, bodies: ^[4]Constraint_Kernel_Body_Wide, dt, inverse_dt: f32,
	impulses_raw: rawptr, active_mask: util.I32x8, phase: Constraint_Kernel_Phase,
)
{
	_ = inverse_dt;
	prestep := (^Angular_Swivel_Hinge_Prestep)(prestep_raw);
	impulses := (^util.F32x8)(impulses_raw);
	if phase == .Prestep
	{
		constraint_kernel_prepare_scalar_impulses(impulses, active_mask);
		return;
	}
	if phase == .Incremental_Update
	{
		return;
	}
	swivel_axis, hinge_axis, jacobian := constraint_kernel_cross_axis_jacobian(
		prestep.local_swivel_axis_a,
		prestep.local_hinge_axis_b,
		bodies,
		1e-3
	);
	if phase == .Warmstart
	{
		world_impulse := util.vector3_wide_scale(jacobian, impulses^);
		constraint_kernel_apply_angular(&bodies[0], world_impulse, 1, active_mask);
		constraint_kernel_apply_angular(&bodies[1], world_impulse, -1, active_mask);
		return;
	}
	position_error_to_velocity, _, _ := spring_settings_wide_compute(prestep.spring_settings, dt);
	bias := util.Vector3_Wide{x=simd.neg(simd.mul(
				position_error_to_velocity,
				util.vector3_wide_dot(hinge_axis, swivel_axis)
	))};
	constraint_kernel_solve_cross_axis(bodies, jacobian, bias, dt, prestep.spring_settings, impulses, active_mask);
}

swing_limit_kernel :: #force_inline proc "contextless" (
	prestep_raw: rawptr, bodies: ^[4]Constraint_Kernel_Body_Wide, dt, inverse_dt: f32,
	impulses_raw: rawptr, active_mask: util.I32x8, phase: Constraint_Kernel_Phase,
)
{
	prestep := (^Swing_Limit_Prestep)(prestep_raw);
	impulses := (^util.F32x8)(impulses_raw);
	if phase == .Prestep
	{
		constraint_kernel_prepare_scalar_impulses(impulses, active_mask);
		return;
	}
	if phase == .Incremental_Update
	{
		return;
	}
	axis_a, axis_b, jacobian := constraint_kernel_cross_axis_jacobian(
		prestep.axis_local_a,
		prestep.axis_local_b,
		bodies,
		1e-7
	);
	if phase == .Warmstart
	{
		world_impulse := util.vector3_wide_scale(jacobian, impulses^);
		constraint_kernel_apply_angular(&bodies[0], world_impulse, 1, active_mask);
		constraint_kernel_apply_angular(&bodies[1], world_impulse, -1, active_mask);
		return;
	}
	impulse_to_velocity_a := util.symmetric3x3_wide_transform(jacobian, bodies[0].inverse_inertia);
	impulse_to_velocity_b := util.symmetric3x3_wide_transform(jacobian, bodies[1].inverse_inertia);
	position_error_to_velocity, effective_mass_scale, softness := spring_settings_wide_compute(
		prestep.spring_settings,
		dt
	);
	effective_mass := simd.div(
		effective_mass_scale,
		simd.add(
			util.vector3_wide_dot(impulse_to_velocity_a, jacobian),
			util.vector3_wide_dot(impulse_to_velocity_b, jacobian)
		)
	);
	error := simd.sub(util.vector3_wide_dot(axis_a, axis_b), prestep.minimum_dot);
	bias := simd.neg(simd.min(simd.mul(error, util.F32x8(inverse_dt)), simd.mul(error, position_error_to_velocity)));
	csv := util.vector3_wide_dot(
		util.vector3_wide_subtract(bodies[0].angular_velocity, bodies[1].angular_velocity),
		jacobian
	);
	csi := simd.sub(simd.mul(simd.sub(bias, csv), effective_mass), simd.mul(impulses^, softness));
	constraint_clamp_positive_impulse(impulses, &csi);
	impulses^ = constraint_kernel_mask_scalar(impulses^, active_mask);
	csi = constraint_kernel_mask_scalar(csi, active_mask);
	world_impulse := util.vector3_wide_scale(jacobian, csi);
	constraint_kernel_apply_angular(&bodies[0], world_impulse, 1, active_mask);
	constraint_kernel_apply_angular(&bodies[1], world_impulse, -1, active_mask);
}

twist_motor_kernel :: #force_inline proc "contextless" (
	prestep_raw:rawptr, bodies:^[4]Constraint_Kernel_Body_Wide, dt, inverse_dt:f32,
	impulses_raw:rawptr, active_mask:util.I32x8, phase:Constraint_Kernel_Phase,
)
{
	_ = inverse_dt;
	prestep := (^Twist_Motor_Prestep)(prestep_raw);
	impulses:=(^util.F32x8)(impulses_raw);
	if phase==.Prestep
	{
		constraint_kernel_prepare_scalar_impulses(impulses, active_mask);
		return;
	}
	if phase==.Incremental_Update
	{
		return;
	}
	axis_a:=util.quaternion_wide_transform(prestep.local_axis_a, bodies[0].orientation);
	axis_b:=util.quaternion_wide_transform(prestep.local_axis_b, bodies[1].orientation);
	axis_sum:=util.vector3_wide_add(axis_a, axis_b);
	axis_length:=util.vector3_wide_length(axis_sum);
	jacobian:=util.vector3_wide_scale(axis_sum, simd.div(util.F32x8(1), axis_length));
	jacobian=util.vector3_wide_select(
		transmute(util.I32x8)simd.lanes_lt(axis_length, util.F32x8(1e-10)),
		axis_a,
		jacobian
	);
	if phase==.Warmstart
	{
		world:=util.vector3_wide_scale(jacobian, impulses^);
		constraint_kernel_apply_angular(&bodies[0], world, 1, active_mask);
		constraint_kernel_apply_angular(&bodies[1], world, -1, active_mask);
		return;
	}
	impulse_to_velocity_a:=util.symmetric3x3_wide_transform(jacobian, bodies[0].inverse_inertia);
	impulse_to_velocity_b:=util.symmetric3x3_wide_transform(jacobian, bodies[1].inverse_inertia);
	inverse_effective_mass:=simd.add(
		util.vector3_wide_dot(jacobian, impulse_to_velocity_a),
		util.vector3_wide_dot(jacobian, impulse_to_velocity_b)
	);
	effective_mass_scale, softness, maximum_impulse:=motor_settings_wide_compute(prestep.settings, dt);
	effective_mass:=simd.div(effective_mass_scale, inverse_effective_mass);
	velocity:=util.vector3_wide_dot(
		util.vector3_wide_subtract(bodies[0].angular_velocity, bodies[1].angular_velocity),
		jacobian
	);
	csi:=simd.sub(simd.mul(simd.sub(prestep.target_velocity, velocity), effective_mass), simd.mul(impulses^, softness));
	constraint_kernel_clamp_accumulated_scalar(maximum_impulse, impulses, &csi, active_mask);
	world:=util.vector3_wide_scale(jacobian, csi);
	constraint_kernel_apply_angular(&bodies[0], world, 1, active_mask);
	constraint_kernel_apply_angular(&bodies[1], world, -1, active_mask);
}

constraint_kernel_twist_geometry :: #force_inline proc "contextless" (
	bodies: ^[4]Constraint_Kernel_Body_Wide, local_basis_a, local_basis_b: util.Quaternion_Wide,
) -> (basis_b_x, basis_b_z: util.Vector3_Wide, basis_a: util.Matrix3x3_Wide, jacobian: util.Vector3_Wide)
{
	basis_quaternion_a := util.quaternion_wide_concatenate(local_basis_a, bodies[0].orientation);
	basis_quaternion_b := util.quaternion_wide_concatenate(local_basis_b, bodies[1].orientation);
	basis_b_x, basis_b_z = util.quaternion_wide_transform_unit_xz(basis_quaternion_b);
	basis_a = util.matrix3x3_wide_from_quaternion(basis_quaternion_a);
	unnormalized := util.vector3_wide_add(basis_a.z, basis_b_z);
	length := util.vector3_wide_length(unnormalized);
	jacobian = util.vector3_wide_scale(unnormalized, simd.div(util.F32x8(1), length));
	jacobian = util.vector3_wide_select(
		transmute(util.I32x8)simd.lanes_lt(length, util.F32x8(1e-10)),
		basis_a.z,
		jacobian
	);
	return;
}

constraint_kernel_twist_angle :: #force_inline proc "contextless" (
	basis_b_x, basis_b_z: util.Vector3_Wide, basis_a: util.Matrix3x3_Wide,
) -> util.F32x8
{
	aligning_rotation := util.quaternion_wide_between_normalized_vectors(basis_b_z, basis_a.z);
	aligned_b_x := util.quaternion_wide_transform(basis_b_x, aligning_rotation);
	x := util.vector3_wide_dot(aligned_b_x, basis_a.x);
	y := util.vector3_wide_dot(aligned_b_x, basis_a.y);
	abs_angle := util.acos_approx_wide(x);
	return util.wide_select_f32(transmute(util.I32x8)simd.lanes_lt(y, util.F32x8(0)), simd.neg(abs_angle), abs_angle);
}

constraint_kernel_twist_apply :: #force_inline proc "contextless" (
	bodies: ^[4]Constraint_Kernel_Body_Wide, jacobian: util.Vector3_Wide, impulse: util.F32x8, active_mask: util.I32x8,
)
{
	world_impulse := util.vector3_wide_scale(jacobian, impulse);
	constraint_kernel_apply_angular(&bodies[0], world_impulse, 1, active_mask);
	constraint_kernel_apply_angular(&bodies[1], world_impulse, -1, active_mask);
}

twist_limit_kernel :: #force_inline proc "contextless" (
	prestep_raw: rawptr, bodies: ^[4]Constraint_Kernel_Body_Wide, dt, inverse_dt: f32,
	impulses_raw: rawptr, active_mask: util.I32x8, phase: Constraint_Kernel_Phase,
)
{
	prestep := (^Twist_Limit_Prestep)(prestep_raw);
	impulses := (^util.F32x8)(impulses_raw);
	if phase == .Prestep
	{
		constraint_kernel_prepare_scalar_impulses(impulses, active_mask);
		return;
	}
	if phase == .Incremental_Update
	{
		return;
	}
	basis_b_x, basis_b_z, basis_a, jacobian := constraint_kernel_twist_geometry(
		bodies,
		prestep.local_basis_a,
		prestep.local_basis_b
	);
	angle := constraint_kernel_twist_angle(basis_b_x, basis_b_z, basis_a);
	minimum_error := util.signed_angle_difference_wide(prestep.minimum_angle, angle);
	maximum_error := util.signed_angle_difference_wide(prestep.maximum_angle, angle);
	use_minimum := transmute(util.I32x8)simd.lanes_lt(simd.abs(minimum_error), simd.abs(maximum_error));
	error := util.wide_select_f32(use_minimum, simd.neg(minimum_error), maximum_error);
	jacobian = util.vector3_wide_select(use_minimum, util.vector3_wide_negate(jacobian), jacobian);
	if phase == .Warmstart
	{
		constraint_kernel_twist_apply(bodies, jacobian, impulses^, active_mask);
		return;
	}
	impulse_to_velocity_a := util.symmetric3x3_wide_transform(jacobian, bodies[0].inverse_inertia);
	impulse_to_velocity_b := util.symmetric3x3_wide_transform(jacobian, bodies[1].inverse_inertia);
	position_error_to_velocity, effective_mass_scale, softness := spring_settings_wide_compute(
		prestep.spring_settings,
		dt
	);
	effective_mass := simd.div(
		effective_mass_scale,
		simd.add(
			util.vector3_wide_dot(impulse_to_velocity_a, jacobian),
			util.vector3_wide_dot(impulse_to_velocity_b, jacobian)
		)
	);
	bias := util.wide_select_f32(
		transmute(util.I32x8)simd.lanes_lt(error, util.F32x8(0)),
		simd.mul(error, util.F32x8(inverse_dt)),
		simd.mul(error, position_error_to_velocity)
	);
	csv := util.vector3_wide_dot(
		util.vector3_wide_subtract(bodies[0].angular_velocity, bodies[1].angular_velocity),
		jacobian
	);
	csi := simd.sub(simd.mul(simd.sub(bias, csv), effective_mass), simd.mul(impulses^, softness));
	constraint_clamp_positive_impulse(impulses, &csi);
	impulses^ = constraint_kernel_mask_scalar(impulses^, active_mask);
	csi = constraint_kernel_mask_scalar(csi, active_mask);
	constraint_kernel_twist_apply(bodies, jacobian, csi, active_mask);
}

twist_servo_kernel :: #force_inline proc "contextless" (
	prestep_raw: rawptr, bodies: ^[4]Constraint_Kernel_Body_Wide, dt, inverse_dt: f32,
	impulses_raw: rawptr, active_mask: util.I32x8, phase: Constraint_Kernel_Phase,
)
{
	prestep := (^Twist_Servo_Prestep)(prestep_raw);
	impulses := (^util.F32x8)(impulses_raw);
	if phase == .Prestep
	{
		constraint_kernel_prepare_scalar_impulses(impulses, active_mask);
		return;
	}
	if phase == .Incremental_Update
	{
		return;
	}
	basis_b_x, basis_b_z, basis_a, jacobian := constraint_kernel_twist_geometry(
		bodies,
		prestep.local_basis_a,
		prestep.local_basis_b
	);
	if phase == .Warmstart
	{
		constraint_kernel_twist_apply(bodies, jacobian, impulses^, active_mask);
		return;
	}
	impulse_to_velocity_a := util.symmetric3x3_wide_transform(jacobian, bodies[0].inverse_inertia);
	impulse_to_velocity_b := util.symmetric3x3_wide_transform(jacobian, bodies[1].inverse_inertia);
	position_error_to_velocity, effective_mass_scale, softness := spring_settings_wide_compute(
		prestep.spring_settings,
		dt
	);
	effective_mass := simd.div(
		effective_mass_scale,
		simd.add(
			util.vector3_wide_dot(impulse_to_velocity_a, jacobian),
			util.vector3_wide_dot(impulse_to_velocity_b, jacobian)
		)
	);
	angle := constraint_kernel_twist_angle(basis_b_x, basis_b_z, basis_a);
	error := util.signed_angle_difference_wide(prestep.target_angle, angle);
	bias, maximum_impulse := servo_settings_wide_compute_bias_scalar(
		error,
		position_error_to_velocity,
		prestep.servo_settings,
		dt,
		inverse_dt
	);
	csv := util.vector3_wide_dot(
		util.vector3_wide_subtract(bodies[0].angular_velocity, bodies[1].angular_velocity),
		jacobian
	);
	csi := simd.sub(simd.mul(simd.sub(bias, csv), effective_mass), simd.mul(impulses^, softness));
	constraint_kernel_clamp_accumulated_scalar(maximum_impulse, impulses, &csi, active_mask);
	constraint_kernel_twist_apply(bodies, jacobian, csi, active_mask);
}

constraint_kernel_direction_or_unit_x :: #force_inline proc "contextless" (
	offset:util.Vector3_Wide, epsilon:f32,
) -> (direction:util.Vector3_Wide, distance:util.F32x8)
{
	distance = util.vector3_wide_length(offset);
	direction = util.vector3_wide_scale(offset, simd.div(util.F32x8(1), distance));
	use_unit_x := transmute(util.I32x8)simd.lanes_lt(distance, util.F32x8(epsilon));
	direction = util.vector3_wide_select(use_unit_x, {x=util.F32x8(1)}, direction);
	return;
}

constraint_kernel_center_direction :: #force_inline proc "contextless" (
	offset: util.Vector3_Wide,
) -> (direction: util.Vector3_Wide, distance: util.F32x8)
{
	distance = util.vector3_wide_length(offset);
	direction = util.vector3_wide_scale(offset, util.approx_reciprocal_f32x8(distance));
	use_unit_x := transmute(util.I32x8)simd.lanes_lt(distance, util.F32x8(1e-5));
	direction = util.vector3_wide_select(use_unit_x, {x=util.F32x8(1)}, direction);
	return;
}

constraint_kernel_center_warmstart_direction :: #force_inline proc "contextless" (
	offset: util.Vector3_Wide,
) -> util.Vector3_Wide
{
	length_squared := util.vector3_wide_length_squared(offset);
	direction := util.vector3_wide_scale(offset, util.approx_reciprocal_sqrt_f32x8(length_squared));
	use_unit_x := transmute(util.I32x8)simd.lanes_lt(length_squared, util.F32x8(1e-10));
	return util.vector3_wide_select(use_unit_x, {x=util.F32x8(1)}, direction);
}

constraint_kernel_apply_distance :: #force_inline proc "contextless" (
	bodies:^[4]Constraint_Kernel_Body_Wide, direction, angular_a, angular_b:util.Vector3_Wide,
	impulse:util.F32x8, active_mask:util.I32x8,
)
{
	linear_impulse := util.vector3_wide_scale(direction, impulse);
	constraint_kernel_apply_world_impulses(
		&bodies[0],
		linear_impulse,
		util.vector3_wide_scale(angular_a, impulse),
		active_mask
	);
	constraint_kernel_apply_world_impulses(
		&bodies[1],
		util.vector3_wide_negate(linear_impulse),
		util.vector3_wide_scale(angular_b, impulse),
		active_mask
	);
}

center_distance_constraint_kernel :: #force_inline proc "contextless" (
	prestep_raw:rawptr, bodies:^[4]Constraint_Kernel_Body_Wide, dt, inverse_dt:f32,
	impulses_raw:rawptr, active_mask:util.I32x8, phase:Constraint_Kernel_Phase,
)
{
	_ = inverse_dt;
	prestep := (^Center_Distance_Constraint_Prestep)(prestep_raw);
	impulses := (^util.F32x8)(impulses_raw);
	if phase == .Prestep
	{
		constraint_kernel_prepare_scalar_impulses(impulses, active_mask);
		return;
	}
	if phase == .Incremental_Update
	{
		return;
	}
	offset := util.vector3_wide_subtract(bodies[1].position, bodies[0].position);
	if phase == .Warmstart
	{
		direction := constraint_kernel_center_warmstart_direction(offset);
		constraint_kernel_apply_linear(&bodies[0], util.vector3_wide_scale(direction, impulses^), 1, active_mask);
		constraint_kernel_apply_linear(&bodies[1], util.vector3_wide_scale(direction, impulses^), -1, active_mask);
		return;
	}
	direction, distance := constraint_kernel_center_direction(offset);
	position_error_to_velocity, effective_mass_scale, softness := spring_settings_wide_compute(
		prestep.spring_settings,
		dt
	);
	effective_mass := simd.div(effective_mass_scale, simd.add(bodies[0].inverse_mass, bodies[1].inverse_mass));
	bias := simd.mul(simd.sub(distance, prestep.target_distance), position_error_to_velocity);
	csv := util.vector3_wide_dot(
		util.vector3_wide_subtract(bodies[0].linear_velocity, bodies[1].linear_velocity),
		direction
	);
	csi := simd.sub(simd.mul(simd.sub(bias, csv), effective_mass), simd.mul(impulses^, softness));
	csi = constraint_kernel_mask_scalar(csi, active_mask);
	impulses^ = constraint_kernel_mask_scalar(simd.add(impulses^, csi), active_mask);
	constraint_kernel_apply_linear(&bodies[0], util.vector3_wide_scale(direction, csi), 1, active_mask);
	constraint_kernel_apply_linear(&bodies[1], util.vector3_wide_scale(direction, csi), -1, active_mask);
}

center_distance_limit_kernel :: #force_inline proc "contextless" (
	prestep_raw:rawptr, bodies:^[4]Constraint_Kernel_Body_Wide, dt, inverse_dt:f32,
	impulses_raw:rawptr, active_mask:util.I32x8, phase:Constraint_Kernel_Phase,
)
{
	prestep := (^Center_Distance_Limit_Prestep)(prestep_raw);
	impulses := (^util.F32x8)(impulses_raw);
	if phase == .Prestep
	{
		constraint_kernel_prepare_scalar_impulses(impulses, active_mask);
		return;
	}
	if phase == .Incremental_Update
	{
		return;
	}
	direction, distance := constraint_kernel_center_direction(util.vector3_wide_subtract(
			bodies[1].position,
			bodies[0].position
	));
	use_minimum := transmute(util.I32x8)simd.lanes_lt(
		simd.abs(simd.sub(distance, prestep.minimum_distance)),
		simd.abs(simd.sub(distance, prestep.maximum_distance))
	);
	direction = util.vector3_wide_conditional_negate(use_minimum, direction);
	if phase == .Warmstart
	{
		constraint_kernel_apply_linear(&bodies[0], util.vector3_wide_scale(direction, impulses^), 1, active_mask);
		constraint_kernel_apply_linear(&bodies[1], util.vector3_wide_scale(direction, impulses^), -1, active_mask);
		return;
	}
	position_error_to_velocity, effective_mass_scale, softness := spring_settings_wide_compute(
		prestep.spring_settings,
		dt
	);
	effective_mass := simd.div(effective_mass_scale, simd.add(bodies[0].inverse_mass, bodies[1].inverse_mass));
	error := util.wide_select_f32(
		use_minimum,
		simd.sub(prestep.minimum_distance, distance),
		simd.sub(distance, prestep.maximum_distance)
	);
	bias := constraint_inequality_bias_velocity(error, position_error_to_velocity, inverse_dt);
	csv := util.vector3_wide_dot(
		util.vector3_wide_subtract(bodies[0].linear_velocity, bodies[1].linear_velocity),
		direction
	);
	csi := simd.sub(simd.mul(simd.sub(bias, csv), effective_mass), simd.mul(impulses^, softness));
	constraint_clamp_positive_impulse(impulses, &csi);
	impulses^ = constraint_kernel_mask_scalar(impulses^, active_mask);
	csi = constraint_kernel_mask_scalar(csi, active_mask);
	constraint_kernel_apply_linear(&bodies[0], util.vector3_wide_scale(direction, csi), 1, active_mask);
	constraint_kernel_apply_linear(&bodies[1], util.vector3_wide_scale(direction, csi), -1, active_mask);
}

constraint_kernel_distance_geometry :: #force_inline proc "contextless" (
	bodies:^[4]Constraint_Kernel_Body_Wide, local_offset_a, local_offset_b:util.Vector3_Wide,
) -> (offset_a, offset_b, anchor_offset, direction:util.Vector3_Wide, distance:util.F32x8)
{
	offset_a = util.quaternion_wide_transform(local_offset_a, bodies[0].orientation);
	offset_b = util.quaternion_wide_transform(local_offset_b, bodies[1].orientation);
	anchor_offset = util.vector3_wide_add(
		util.vector3_wide_subtract(offset_b, offset_a),
		util.vector3_wide_subtract(bodies[1].position, bodies[0].position)
	);
	direction, distance = constraint_kernel_direction_or_unit_x(anchor_offset, 1e-9);
	return;
}

distance_servo_kernel :: #force_inline proc "contextless" (
	prestep_raw:rawptr, bodies:^[4]Constraint_Kernel_Body_Wide, dt, inverse_dt:f32,
	impulses_raw:rawptr, active_mask:util.I32x8, phase:Constraint_Kernel_Phase,
)
{
	prestep := (^Distance_Servo_Prestep)(prestep_raw);
	impulses := (^util.F32x8)(impulses_raw);
	if phase == .Prestep
	{
		constraint_kernel_prepare_scalar_impulses(impulses, active_mask);
		return;
	}
	if phase == .Incremental_Update
	{
		return;
	}
	offset_a, offset_b, _, direction, distance := constraint_kernel_distance_geometry(
		bodies,
		prestep.local_offset_a,
		prestep.local_offset_b
	);
	angular_a := util.vector3_wide_cross(offset_a, direction);
	angular_b := util.vector3_wide_cross(direction, offset_b);
	if phase == .Warmstart
	{
		constraint_kernel_apply_distance(bodies, direction, angular_a, angular_b, impulses^, active_mask);
		return;
	}
	angular_impulse_a := util.symmetric3x3_wide_transform(angular_a, bodies[0].inverse_inertia);
	angular_impulse_b := util.symmetric3x3_wide_transform(angular_b, bodies[1].inverse_inertia);
	inverse_effective_mass := simd.add(
		simd.add(bodies[0].inverse_mass, bodies[1].inverse_mass),
		simd.add(
			util.vector3_wide_dot(angular_a, angular_impulse_a),
			util.vector3_wide_dot(angular_b, angular_impulse_b)
		),
	);
	position_error_to_velocity, effective_mass_scale, softness := spring_settings_wide_compute(
		prestep.spring_settings,
		dt
	);
	effective_mass := simd.div(effective_mass_scale, inverse_effective_mass);
	bias, maximum_impulse := servo_settings_wide_compute_bias_scalar(
		simd.sub(distance, prestep.target_distance),
		position_error_to_velocity,
		prestep.servo_settings,
		dt,
		inverse_dt
	);
	csv := simd.add(
		simd.sub(
			util.vector3_wide_dot(bodies[0].linear_velocity, direction),
			util.vector3_wide_dot(bodies[1].linear_velocity, direction)
		),
		simd.add(
			util.vector3_wide_dot(bodies[0].angular_velocity, angular_a),
			util.vector3_wide_dot(bodies[1].angular_velocity, angular_b)
		),
	);
	csi := simd.sub(simd.mul(simd.sub(bias, csv), effective_mass), simd.mul(impulses^, softness));
	constraint_kernel_clamp_accumulated_scalar(maximum_impulse, impulses, &csi, active_mask);
	constraint_kernel_apply_distance(bodies, direction, angular_a, angular_b, csi, active_mask);
}

distance_limit_kernel :: #force_inline proc "contextless" (
	prestep_raw:rawptr, bodies:^[4]Constraint_Kernel_Body_Wide, dt, inverse_dt:f32,
	impulses_raw:rawptr, active_mask:util.I32x8, phase:Constraint_Kernel_Phase,
)
{
	prestep := (^Distance_Limit_Prestep)(prestep_raw);
	impulses := (^util.F32x8)(impulses_raw);
	if phase == .Prestep
	{
		constraint_kernel_prepare_scalar_impulses(impulses, active_mask);
		return;
	}
	if phase == .Incremental_Update
	{
		return;
	}
	offset_a, offset_b, _, direction, distance := constraint_kernel_distance_geometry(
		bodies,
		prestep.local_offset_a,
		prestep.local_offset_b
	);
	use_minimum := transmute(util.I32x8)simd.lanes_lt(
		simd.abs(simd.sub(distance, prestep.minimum_distance)),
		simd.abs(simd.sub(distance, prestep.maximum_distance))
	);
	direction = util.vector3_wide_conditional_negate(use_minimum, direction);
	angular_a := util.vector3_wide_cross(offset_a, direction);
	angular_b := util.vector3_wide_cross(direction, offset_b);
	if phase == .Warmstart
	{
		constraint_kernel_apply_distance(bodies, direction, angular_a, angular_b, impulses^, active_mask);
		return;
	}
	inverse_effective_mass := simd.add(
		simd.add(bodies[0].inverse_mass, bodies[1].inverse_mass),
		simd.add(
			util.symmetric3x3_wide_vector_sandwich(angular_a, bodies[0].inverse_inertia),
			util.symmetric3x3_wide_vector_sandwich(angular_b, bodies[1].inverse_inertia)
		),
	);
	position_error_to_velocity, effective_mass_scale, softness := spring_settings_wide_compute(
		prestep.spring_settings,
		dt
	);
	error := util.wide_select_f32(
		use_minimum,
		simd.sub(prestep.minimum_distance, distance),
		simd.sub(distance, prestep.maximum_distance)
	);
	bias := constraint_inequality_bias_velocity(error, position_error_to_velocity, inverse_dt);
	csv := simd.add(
		simd.sub(
			util.vector3_wide_dot(bodies[0].linear_velocity, direction),
			util.vector3_wide_dot(bodies[1].linear_velocity, direction)
		),
		simd.add(
			util.vector3_wide_dot(bodies[0].angular_velocity, angular_a),
			util.vector3_wide_dot(bodies[1].angular_velocity, angular_b)
		),
	);
	csi := simd.sub(
		simd.mul(simd.sub(bias, csv), simd.div(effective_mass_scale, inverse_effective_mass)),
		simd.mul(impulses^, softness)
	);
	constraint_clamp_positive_impulse(impulses, &csi);
	impulses^=constraint_kernel_mask_scalar(impulses^, active_mask);
	csi=constraint_kernel_mask_scalar(csi, active_mask);
	constraint_kernel_apply_distance(bodies, direction, angular_a, angular_b, csi, active_mask);
}

constraint_kernel_hinge_geometry :: #force_inline proc "contextless" (
	prestep: ^Hinge_Prestep, bodies: ^[4]Constraint_Kernel_Body_Wide,
) -> (offset_a, offset_b, hinge_axis_a, hinge_axis_b: util.Vector3_Wide, hinge_jacobian: util.Matrix2x3_Wide)
{
	offset_a = util.quaternion_wide_transform(prestep.local_offset_a, bodies[0].orientation);
	hinge_axis_a = util.quaternion_wide_transform(prestep.local_hinge_axis_a, bodies[0].orientation);
	offset_b = util.quaternion_wide_transform(prestep.local_offset_b, bodies[1].orientation);
	hinge_axis_b = util.quaternion_wide_transform(prestep.local_hinge_axis_b, bodies[1].orientation);
	local_x, local_y := constraint_kernel_build_orthonormal_basis(prestep.local_hinge_axis_a);
	hinge_jacobian = {
		util.quaternion_wide_transform(local_x, bodies[0].orientation),
		util.quaternion_wide_transform(local_y, bodies[0].orientation),
	};
	return;
}

constraint_kernel_hinge_apply :: #force_inline proc "contextless" (
	bodies: ^[4]Constraint_Kernel_Body_Wide, offset_a, offset_b: util.Vector3_Wide,
	hinge_jacobian: util.Matrix2x3_Wide, impulse: Hinge_Accumulated_Impulses, active_mask: util.I32x8,
)
{
	hinge_angular_impulse := util.matrix2x3_wide_transform(impulse.hinge, hinge_jacobian);
	angular_impulse_a := util.vector3_wide_add(
		util.vector3_wide_cross(offset_a, impulse.ball_socket),
		hinge_angular_impulse
	);
	angular_impulse_b := util.vector3_wide_subtract(
		util.vector3_wide_cross(impulse.ball_socket, offset_b),
		hinge_angular_impulse
	);
	constraint_kernel_apply_world_impulses(&bodies[0], impulse.ball_socket, angular_impulse_a, active_mask);
	constraint_kernel_apply_world_impulses(
		&bodies[1],
		util.vector3_wide_negate(impulse.ball_socket),
		angular_impulse_b,
		active_mask
	);
}

hinge_kernel :: #force_inline proc "contextless" (
	prestep_raw: rawptr, bodies: ^[4]Constraint_Kernel_Body_Wide, dt, inverse_dt: f32,
	impulses_raw: rawptr, active_mask: util.I32x8, phase: Constraint_Kernel_Phase,
)
{
	_ = inverse_dt;
	prestep := (^Hinge_Prestep)(prestep_raw);
	impulses := (^Hinge_Accumulated_Impulses)(impulses_raw);
	if phase == .Prestep
	{
		impulses.ball_socket = constraint_kernel_mask_vector3(impulses.ball_socket, active_mask);
		impulses.hinge = constraint_kernel_mask_vector2(impulses.hinge, active_mask);
		return;
	}
	if phase == .Incremental_Update
	{
		return;
	}
	offset_a, offset_b, hinge_axis_a, hinge_axis_b, hinge_jacobian := constraint_kernel_hinge_geometry(prestep, bodies);
	if phase == .Warmstart
	{
		constraint_kernel_hinge_apply(bodies, offset_a, offset_b, hinge_jacobian, impulses^, active_mask);
		return;
	}
	upper_left := util.symmetric3x3_wide_add(
		util.symmetric3x3_wide_skew_sandwich(offset_a, bodies[0].inverse_inertia),
		util.symmetric3x3_wide_skew_sandwich(offset_b, bodies[1].inverse_inertia),
	);
	linear_contribution := simd.add(bodies[0].inverse_mass, bodies[1].inverse_mass);
	upper_left.xx = simd.add(upper_left.xx, linear_contribution);
	upper_left.yy = simd.add(upper_left.yy, linear_contribution);
	upper_left.zz = simd.add(upper_left.zz, linear_contribution);
	hinge_inertia_a := util.matrix2x3_wide_multiply_symmetric3x3(hinge_jacobian, bodies[0].inverse_inertia);
	hinge_inertia_b := util.matrix2x3_wide_multiply_symmetric3x3(hinge_jacobian, bodies[1].inverse_inertia);
	lower_right := util.symmetric2x2_wide_add(
		util.symmetric2x2_wide_complete_matrix_sandwich(hinge_inertia_a, hinge_jacobian),
		util.symmetric2x2_wide_complete_matrix_sandwich(hinge_inertia_b, hinge_jacobian),
	);
	off_diagonal := util.Matrix2x3_Wide{
		util.vector3_wide_add(
			util.vector3_wide_cross(hinge_inertia_a.x, offset_a),
			util.vector3_wide_cross(hinge_inertia_b.x, offset_b)
		),
		util.vector3_wide_add(
			util.vector3_wide_cross(hinge_inertia_a.y, offset_a),
			util.vector3_wide_cross(hinge_inertia_b.y, offset_b)
		),
	};
	effective_mass := util.symmetric5x5_wide_invert(util.Symmetric5x5_Wide{upper_left, off_diagonal, lower_right});
	position_error_to_velocity, effective_mass_scale, softness := spring_settings_wide_compute(
		prestep.spring_settings,
		dt
	);
	ball_socket_error := util.vector3_wide_subtract(
		util.vector3_wide_add(util.vector3_wide_subtract(bodies[1].position, bodies[0].position), offset_b),
		offset_a
	);
	ball_socket_bias := util.vector3_wide_scale(ball_socket_error, position_error_to_velocity);
	hinge_error := constraint_kernel_angular_hinge_error(hinge_axis_a, hinge_axis_b, hinge_jacobian);
	hinge_bias := util.vector2_wide_scale(hinge_error, simd.neg(position_error_to_velocity));
	ball_socket_csv := util.vector3_wide_add(
		util.vector3_wide_subtract(bodies[0].linear_velocity, bodies[1].linear_velocity),
		util.vector3_wide_add(
			util.vector3_wide_cross(bodies[0].angular_velocity, offset_a),
			util.vector3_wide_cross(offset_b, bodies[1].angular_velocity)
		),
	);
	hinge_csv := util.matrix2x3_wide_transform_by_transpose(
		util.vector3_wide_subtract(bodies[0].angular_velocity, bodies[1].angular_velocity),
		hinge_jacobian
	);
	ball_socket_csi, hinge_csi := util.symmetric5x5_wide_transform(
		util.vector3_wide_subtract(
			ball_socket_bias,
			ball_socket_csv
		), util.vector2_wide_subtract(hinge_bias, hinge_csv), effective_mass,
	);
	ball_socket_csi = util.vector3_wide_subtract(
		util.vector3_wide_scale(ball_socket_csi, effective_mass_scale),
		util.vector3_wide_scale(impulses.ball_socket, softness)
	);
	hinge_csi = util.vector2_wide_subtract(
		util.vector2_wide_scale(hinge_csi, effective_mass_scale),
		util.vector2_wide_scale(impulses.hinge, softness)
	);
	ball_socket_csi = constraint_kernel_mask_vector3(ball_socket_csi, active_mask);
	hinge_csi = constraint_kernel_mask_vector2(hinge_csi, active_mask);
	impulses.ball_socket = constraint_kernel_mask_vector3(
		util.vector3_wide_add(impulses.ball_socket, ball_socket_csi),
		active_mask
	);
	impulses.hinge = constraint_kernel_mask_vector2(util.vector2_wide_add(impulses.hinge, hinge_csi), active_mask);
	constraint_kernel_hinge_apply(
		bodies,
		offset_a,
		offset_b,
		hinge_jacobian,
		{ball_socket_csi, hinge_csi},
		active_mask
	);
}

constraint_kernel_linear_axis_geometry :: #force_inline proc "contextless" (
	bodies:^[4]Constraint_Kernel_Body_Wide, local_offset_a, local_offset_b, local_axis:util.Vector3_Wide,
) -> (plane_offset:util.F32x8, normal, angular_a, angular_b:util.Vector3_Wide)
{
	normal = util.quaternion_wide_transform(local_axis, bodies[0].orientation);
	anchor_a := util.quaternion_wide_transform(local_offset_a, bodies[0].orientation);
	offset_b := util.quaternion_wide_transform(local_offset_b, bodies[1].orientation);
	anchor_b := util.vector3_wide_add(util.vector3_wide_subtract(bodies[1].position, bodies[0].position), offset_b);
	plane_offset = util.vector3_wide_dot(util.vector3_wide_subtract(anchor_b, anchor_a), normal);
	closest_offset := util.vector3_wide_subtract(anchor_b, util.vector3_wide_scale(normal, plane_offset));
	angular_a = util.vector3_wide_cross(closest_offset, normal);
	angular_b = util.vector3_wide_cross(normal, offset_b);
	return;
}

constraint_kernel_linear_axis_effective_mass :: #force_inline proc "contextless" (
	bodies:^[4]Constraint_Kernel_Body_Wide, angular_a, angular_b:util.Vector3_Wide, effective_mass_scale:util.F32x8,
) -> util.F32x8
{
	angular_contribution_a := util.symmetric3x3_wide_vector_sandwich(angular_a, bodies[0].inverse_inertia);
	angular_contribution_b := util.symmetric3x3_wide_vector_sandwich(angular_b, bodies[1].inverse_inertia);
	return simd.div(
		effective_mass_scale,
		simd.add(
			simd.add(bodies[0].inverse_mass, bodies[1].inverse_mass),
			simd.add(angular_contribution_a, angular_contribution_b)
		)
	);
}

linear_axis_servo_kernel :: #force_inline proc "contextless" (
	prestep_raw:rawptr, bodies:^[4]Constraint_Kernel_Body_Wide, dt, inverse_dt:f32,
	impulses_raw:rawptr, active_mask:util.I32x8, phase:Constraint_Kernel_Phase,
)
{
	prestep := (^Linear_Axis_Servo_Prestep)(prestep_raw);
	impulses:=(^util.F32x8)(impulses_raw);
	if phase==.Prestep
	{
		constraint_kernel_prepare_scalar_impulses(impulses, active_mask);
		return;
	}
	if phase==.Incremental_Update
	{
		return;
	}
	plane_offset, normal, angular_a, angular_b:=constraint_kernel_linear_axis_geometry(
		bodies,
		prestep.local_offset_a,
		prestep.local_offset_b,
		prestep.local_plane_normal
	);
	if phase==.Warmstart
	{
		constraint_kernel_apply_distance(bodies, normal, angular_a, angular_b, impulses^, active_mask);
		return;
	}
	position_error_to_velocity, effective_mass_scale, softness:=spring_settings_wide_compute(prestep.spring_settings, dt);
	bias, maximum_impulse:=servo_settings_wide_compute_bias_scalar(
		simd.sub(plane_offset, prestep.target_offset),
		position_error_to_velocity,
		prestep.servo_settings,
		dt,
		inverse_dt
	);
	csv:=simd.add(
		util.vector3_wide_dot(util.vector3_wide_subtract(bodies[0].linear_velocity, bodies[1].linear_velocity), normal),
		simd.add(
			util.vector3_wide_dot(bodies[0].angular_velocity, angular_a),
			util.vector3_wide_dot(bodies[1].angular_velocity, angular_b)
		)
	);
	csi:=simd.sub(
		simd.mul(
			simd.sub(bias, csv),
			constraint_kernel_linear_axis_effective_mass(bodies, angular_a, angular_b, effective_mass_scale)
		),
		simd.mul(impulses^, softness)
	);
	constraint_kernel_clamp_accumulated_scalar(maximum_impulse, impulses, &csi, active_mask);
	constraint_kernel_apply_distance(bodies, normal, angular_a, angular_b, csi, active_mask);
}

linear_axis_motor_kernel :: #force_inline proc "contextless" (
	prestep_raw:rawptr, bodies:^[4]Constraint_Kernel_Body_Wide, dt, inverse_dt:f32,
	impulses_raw:rawptr, active_mask:util.I32x8, phase:Constraint_Kernel_Phase,
)
{
	_ = inverse_dt;
	prestep := (^Linear_Axis_Motor_Prestep)(prestep_raw);
	impulses:=(^util.F32x8)(impulses_raw);
	if phase==.Prestep
	{
		constraint_kernel_prepare_scalar_impulses(impulses, active_mask);
		return;
	}
	if phase==.Incremental_Update
	{
		return;
	}
	_, normal, angular_a, angular_b:=constraint_kernel_linear_axis_geometry(
		bodies,
		prestep.local_offset_a,
		prestep.local_offset_b,
		prestep.local_axis
	);
	if phase==.Warmstart
	{
		constraint_kernel_apply_distance(bodies, normal, angular_a, angular_b, impulses^, active_mask);
		return;
	}
	effective_mass_scale, softness, maximum_impulse:=motor_settings_wide_compute(prestep.settings, dt);
	csv:=simd.add(
		util.vector3_wide_dot(util.vector3_wide_subtract(bodies[0].linear_velocity, bodies[1].linear_velocity), normal),
		simd.add(
			util.vector3_wide_dot(bodies[0].angular_velocity, angular_a),
			util.vector3_wide_dot(bodies[1].angular_velocity, angular_b)
		)
	);
	csi:=simd.sub(
		simd.mul(
			simd.sub(simd.neg(prestep.target_velocity), csv),
			constraint_kernel_linear_axis_effective_mass(bodies, angular_a, angular_b, effective_mass_scale)
		),
		simd.mul(impulses^, softness)
	);
	constraint_kernel_clamp_accumulated_scalar(maximum_impulse, impulses, &csi, active_mask);
	constraint_kernel_apply_distance(bodies, normal, angular_a, angular_b, csi, active_mask);
}

linear_axis_limit_kernel :: #force_inline proc "contextless" (
	prestep_raw:rawptr, bodies:^[4]Constraint_Kernel_Body_Wide, dt, inverse_dt:f32,
	impulses_raw:rawptr, active_mask:util.I32x8, phase:Constraint_Kernel_Phase,
)
{
	prestep := (^Linear_Axis_Limit_Prestep)(prestep_raw);
	impulses:=(^util.F32x8)(impulses_raw);
	if phase==.Prestep
	{
		constraint_kernel_prepare_scalar_impulses(impulses, active_mask);
		return;
	}
	if phase==.Incremental_Update
	{
		return;
	}
	normal:=util.quaternion_wide_transform(prestep.local_axis, bodies[0].orientation);
	anchor_a:=util.quaternion_wide_transform(prestep.local_offset_a, bodies[0].orientation);
	offset_b:=util.quaternion_wide_transform(prestep.local_offset_b, bodies[1].orientation);
	anchor_b:=util.vector3_wide_add(util.vector3_wide_subtract(bodies[1].position, bodies[0].position), offset_b);
	plane_offset:=util.vector3_wide_dot(util.vector3_wide_subtract(anchor_b, anchor_a), normal);
	minimum_error:=simd.sub(prestep.minimum_offset, plane_offset);
	maximum_error:=simd.sub(plane_offset, prestep.maximum_offset);
	use_minimum:=transmute(util.I32x8)simd.lanes_lt(simd.abs(minimum_error), simd.abs(maximum_error));
	error:=util.wide_select_f32(use_minimum, minimum_error, maximum_error);
	normal=util.vector3_wide_conditional_negate(use_minimum, normal);
	closest_offset:=util.vector3_wide_subtract(anchor_b, util.vector3_wide_scale(normal, plane_offset));
	angular_a:=util.vector3_wide_cross(closest_offset, normal);
	angular_b:=util.vector3_wide_cross(normal, offset_b);
	if phase==.Warmstart
	{
		constraint_kernel_apply_distance(bodies, normal, angular_a, angular_b, impulses^, active_mask);
		return;
	}
	position_error_to_velocity, effective_mass_scale, softness:=spring_settings_wide_compute(prestep.spring_settings, dt);
	bias:=constraint_inequality_bias_velocity(error, position_error_to_velocity, inverse_dt);
	csv:=simd.add(
		util.vector3_wide_dot(util.vector3_wide_subtract(bodies[0].linear_velocity, bodies[1].linear_velocity), normal),
		simd.add(
			util.vector3_wide_dot(bodies[0].angular_velocity, angular_a),
			util.vector3_wide_dot(bodies[1].angular_velocity, angular_b)
		)
	);
	csi:=simd.sub(
		simd.mul(
			simd.sub(bias, csv),
			constraint_kernel_linear_axis_effective_mass(bodies, angular_a, angular_b, effective_mass_scale)
		),
		simd.mul(impulses^, softness)
	);
	constraint_clamp_positive_impulse(impulses, &csi);
	impulses^=constraint_kernel_mask_scalar(impulses^, active_mask);
	csi=constraint_kernel_mask_scalar(csi, active_mask);
	constraint_kernel_apply_distance(bodies, normal, angular_a, angular_b, csi, active_mask);
}

constraint_kernel_point_on_line_jacobians :: #force_inline proc "contextless" (
	bodies: ^[4]Constraint_Kernel_Body_Wide, prestep: ^Point_On_Line_Servo_Prestep,
) -> (anchor_offset: util.Vector3_Wide, linear, angular_a, angular_b: util.Matrix2x3_Wide)
{
	local_tangent_x, local_tangent_y := constraint_kernel_build_orthonormal_basis(prestep.local_direction);
	anchor_a := util.quaternion_wide_transform(prestep.local_offset_a, bodies[0].orientation);
	offset_b := util.quaternion_wide_transform(prestep.local_offset_b, bodies[1].orientation);
	direction := util.quaternion_wide_transform(prestep.local_direction, bodies[0].orientation);
	anchor_b := util.vector3_wide_add(util.vector3_wide_subtract(bodies[1].position, bodies[0].position), offset_b);
	anchor_offset = util.vector3_wide_subtract(anchor_b, anchor_a);
	distance_along_line := util.vector3_wide_dot(anchor_offset, direction);
	offset_a := util.vector3_wide_add(util.vector3_wide_scale(direction, distance_along_line), anchor_a);
	linear = {
		util.quaternion_wide_transform(local_tangent_x, bodies[0].orientation),
		util.quaternion_wide_transform(local_tangent_y, bodies[0].orientation),
	};
	angular_a = {util.vector3_wide_cross(offset_a, linear.x), util.vector3_wide_cross(offset_a, linear.y)};
	angular_b = {util.vector3_wide_cross(linear.x, offset_b), util.vector3_wide_cross(linear.y, offset_b)};
	return;
}

constraint_kernel_point_on_line_apply :: #force_inline proc "contextless" (
	bodies: ^[4]Constraint_Kernel_Body_Wide, linear, angular_a, angular_b: util.Matrix2x3_Wide,
	impulse: util.Vector2_Wide, active_mask: util.I32x8,
)
{
	linear_impulse := util.matrix2x3_wide_transform(impulse, linear);
	angular_impulse_a := util.matrix2x3_wide_transform(impulse, angular_a);
	angular_impulse_b := util.matrix2x3_wide_transform(impulse, angular_b);
	constraint_kernel_apply_world_impulses(&bodies[0], linear_impulse, angular_impulse_a, active_mask);
	constraint_kernel_apply_world_impulses(
		&bodies[1],
		util.vector3_wide_negate(linear_impulse),
		angular_impulse_b,
		active_mask
	);
}

point_on_line_servo_kernel :: #force_inline proc "contextless" (
	prestep_raw: rawptr, bodies: ^[4]Constraint_Kernel_Body_Wide, dt, inverse_dt: f32,
	impulses_raw: rawptr, active_mask: util.I32x8, phase: Constraint_Kernel_Phase,
)
{
	prestep := (^Point_On_Line_Servo_Prestep)(prestep_raw);
	impulses := (^util.Vector2_Wide)(impulses_raw);
	if phase == .Prestep
	{
		impulses^ = constraint_kernel_mask_vector2(impulses^, active_mask);
		return;
	}
	if phase == .Incremental_Update
	{
		return;
	}
	anchor_offset, linear, angular_a, angular_b := constraint_kernel_point_on_line_jacobians(bodies, prestep);
	if phase == .Warmstart
	{
		constraint_kernel_point_on_line_apply(bodies, linear, angular_a, angular_b, impulses^, active_mask);
		return;
	}
	linear_contribution := util.symmetric2x2_wide_sandwich_scale(
		linear,
		simd.add(bodies[0].inverse_mass, bodies[1].inverse_mass)
	);
	angular_contribution_a := util.symmetric3x3_wide_matrix_sandwich(angular_a, bodies[0].inverse_inertia);
	angular_contribution_b := util.symmetric3x3_wide_matrix_sandwich(angular_b, bodies[1].inverse_inertia);
	inverse_effective_mass := util.symmetric2x2_wide_add(
		linear_contribution,
		util.symmetric2x2_wide_add(angular_contribution_a, angular_contribution_b)
	);
	position_error_to_velocity, effective_mass_scale, softness := spring_settings_wide_compute(
		prestep.spring_settings,
		dt
	);
	effective_mass := util.symmetric2x2_wide_scale(
		util.symmetric2x2_wide_invert(inverse_effective_mass),
		effective_mass_scale
	);
	linear_csv_a := util.matrix2x3_wide_transform_by_transpose(bodies[0].linear_velocity, linear);
	linear_csv_b := util.matrix2x3_wide_transform_by_transpose(bodies[1].linear_velocity, linear);
	angular_csv_a := util.matrix2x3_wide_transform_by_transpose(bodies[0].angular_velocity, angular_a);
	angular_csv_b := util.matrix2x3_wide_transform_by_transpose(bodies[1].angular_velocity, angular_b);
	csv := util.vector2_wide_add(
		util.vector2_wide_subtract(linear_csv_a, linear_csv_b),
		util.vector2_wide_add(angular_csv_a, angular_csv_b)
	);
	error := util.Vector2_Wide{
		util.vector3_wide_dot(anchor_offset, linear.x),
		util.vector3_wide_dot(anchor_offset, linear.y),
	};
	bias, maximum_impulse := servo_settings_wide_compute_bias_vector2(
		error,
		position_error_to_velocity,
		prestep.servo_settings,
		dt,
		inverse_dt
	);
	csi := util.symmetric2x2_wide_transform(util.vector2_wide_subtract(bias, csv), effective_mass);
	csi = util.vector2_wide_subtract(csi, util.vector2_wide_scale(impulses^, softness));
	servo_settings_wide_clamp_impulse_vector2(maximum_impulse, impulses, &csi);
	impulses^ = constraint_kernel_mask_vector2(impulses^, active_mask);
	csi = constraint_kernel_mask_vector2(csi, active_mask);
	constraint_kernel_point_on_line_apply(bodies, linear, angular_a, angular_b, csi, active_mask);
}

constraint_kernel_swivel_hinge_geometry :: #force_inline proc "contextless" (
	prestep: ^Swivel_Hinge_Prestep, bodies: ^[4]Constraint_Kernel_Body_Wide,
) -> (swivel_axis, hinge_axis, offset_a, offset_b, angular_jacobian: util.Vector3_Wide)
{
	offset_a = util.quaternion_wide_transform(prestep.local_offset_a, bodies[0].orientation);
	swivel_axis = util.quaternion_wide_transform(prestep.local_swivel_axis_a, bodies[0].orientation);
	offset_b = util.quaternion_wide_transform(prestep.local_offset_b, bodies[1].orientation);
	hinge_axis = util.quaternion_wide_transform(prestep.local_hinge_axis_b, bodies[1].orientation);
	angular_jacobian = util.vector3_wide_cross(swivel_axis, hinge_axis);
	angular_jacobian = util.vector3_wide_select(
		transmute(util.I32x8)simd.lanes_lt(util.vector3_wide_length_squared(angular_jacobian), util.F32x8(1e-3)),
		hinge_axis, angular_jacobian,
	);
	return;
}

constraint_kernel_swivel_hinge_apply :: #force_inline proc "contextless" (
	bodies: ^[4]Constraint_Kernel_Body_Wide, offset_a, offset_b, angular_jacobian: util.Vector3_Wide,
	impulse: util.Vector4_Wide, active_mask: util.I32x8,
)
{
	ball_socket_impulse := util.Vector3_Wide{impulse.x, impulse.y, impulse.z};
	angular_constraint_impulse := util.vector3_wide_scale(angular_jacobian, impulse.w);
	angular_impulse_a := util.vector3_wide_add(
		util.vector3_wide_cross(offset_a, ball_socket_impulse),
		angular_constraint_impulse
	);
	angular_impulse_b := util.vector3_wide_subtract(
		util.vector3_wide_cross(ball_socket_impulse, offset_b),
		angular_constraint_impulse
	);
	constraint_kernel_apply_world_impulses(&bodies[0], ball_socket_impulse, angular_impulse_a, active_mask);
	constraint_kernel_apply_world_impulses(
		&bodies[1],
		util.vector3_wide_negate(ball_socket_impulse),
		angular_impulse_b,
		active_mask
	);
}

swivel_hinge_kernel :: #force_inline proc "contextless" (
	prestep_raw: rawptr, bodies: ^[4]Constraint_Kernel_Body_Wide, dt, inverse_dt: f32,
	impulses_raw: rawptr, active_mask: util.I32x8, phase: Constraint_Kernel_Phase,
)
{
	_ = inverse_dt;
	prestep := (^Swivel_Hinge_Prestep)(prestep_raw);
	impulses := (^util.Vector4_Wide)(impulses_raw);
	if phase == .Prestep
	{
		impulses^ = util.vector4_wide_select(active_mask, impulses^, {});
		return;
	}
	if phase == .Incremental_Update
	{
		return;
	}
	swivel_axis, hinge_axis, offset_a, offset_b, angular_jacobian := constraint_kernel_swivel_hinge_geometry(
		prestep,
		bodies
	);
	if phase == .Warmstart
	{
		constraint_kernel_swivel_hinge_apply(bodies, offset_a, offset_b, angular_jacobian, impulses^, active_mask);
		return;
	}
	upper_left := util.symmetric3x3_wide_add(
		util.symmetric3x3_wide_skew_sandwich(offset_a, bodies[0].inverse_inertia),
		util.symmetric3x3_wide_skew_sandwich(offset_b, bodies[1].inverse_inertia),
	);
	linear_contribution := simd.add(bodies[0].inverse_mass, bodies[1].inverse_mass);
	upper_left.xx = simd.add(upper_left.xx, linear_contribution);
	upper_left.yy = simd.add(upper_left.yy, linear_contribution);
	upper_left.zz = simd.add(upper_left.zz, linear_contribution);
	angular_inertia_a := util.symmetric3x3_wide_transform(angular_jacobian, bodies[0].inverse_inertia);
	angular_inertia_b := util.symmetric3x3_wide_transform(angular_jacobian, bodies[1].inverse_inertia);
	off_diagonal := util.vector3_wide_add(
		util.vector3_wide_cross(angular_inertia_a, offset_a),
		util.vector3_wide_cross(angular_inertia_b, offset_b)
	);
	inverse_effective_mass := util.Symmetric4x4_Wide{
		upper_left.xx, upper_left.yx, upper_left.yy, upper_left.zx, upper_left.zy, upper_left.zz,
		off_diagonal.x, off_diagonal.y, off_diagonal.z,
		simd.add(
			util.vector3_wide_dot(angular_inertia_a, angular_jacobian),
			util.vector3_wide_dot(angular_inertia_b, angular_jacobian)
		),
	};
	effective_mass := util.symmetric4x4_wide_invert(inverse_effective_mass);
	position_error_to_velocity, effective_mass_scale, softness := spring_settings_wide_compute(
		prestep.spring_settings,
		dt
	);
	ball_socket_error := util.vector3_wide_subtract(
		util.vector3_wide_add(util.vector3_wide_subtract(bodies[1].position, bodies[0].position), offset_b),
		offset_a
	);
	bias := util.Vector4_Wide{
		simd.mul(ball_socket_error.x, position_error_to_velocity),
		simd.mul(ball_socket_error.y, position_error_to_velocity),
		simd.mul(ball_socket_error.z, position_error_to_velocity),
		simd.neg(simd.mul(util.vector3_wide_dot(hinge_axis, swivel_axis), position_error_to_velocity)),
	};
	ball_socket_angular_csv := util.vector3_wide_add(
		util.vector3_wide_cross(bodies[0].angular_velocity, offset_a),
		util.vector3_wide_cross(offset_b, bodies[1].angular_velocity)
	);
	ball_socket_csv := util.vector3_wide_add(
		ball_socket_angular_csv,
		util.vector3_wide_subtract(bodies[0].linear_velocity, bodies[1].linear_velocity)
	);
	csv := util.Vector4_Wide{
		ball_socket_csv.x, ball_socket_csv.y, ball_socket_csv.z,
		util.vector3_wide_dot(
			angular_jacobian,
			util.vector3_wide_subtract(bodies[0].angular_velocity, bodies[1].angular_velocity)
		),
	};
	csi := util.symmetric4x4_wide_transform(util.vector4_wide_subtract(bias, csv), effective_mass);
	csi = util.vector4_wide_subtract(
		util.vector4_wide_scale(csi, effective_mass_scale),
		util.vector4_wide_scale(impulses^, softness)
	);
	csi = util.vector4_wide_select(active_mask, csi, {});
	impulses^ = util.vector4_wide_select(active_mask, util.vector4_wide_add(impulses^, csi), {});
	constraint_kernel_swivel_hinge_apply(bodies, offset_a, offset_b, angular_jacobian, csi, active_mask);
}

constraint_kernel_weld_apply :: #force_inline proc "contextless" (
	bodies: ^[4]Constraint_Kernel_Body_Wide, offset, orientation_impulse, offset_impulse: util.Vector3_Wide,
	active_mask: util.I32x8,
)
{
	angular_a := util.vector3_wide_add(orientation_impulse, util.vector3_wide_cross(offset, offset_impulse));
	constraint_kernel_apply_world_impulses(&bodies[0], offset_impulse, angular_a, active_mask);
	constraint_kernel_apply_world_impulses(
		&bodies[1],
		util.vector3_wide_negate(offset_impulse),
		util.vector3_wide_negate(orientation_impulse),
		active_mask
	);
}

weld_kernel :: #force_inline proc "contextless" (
	prestep_raw: rawptr, bodies: ^[4]Constraint_Kernel_Body_Wide, dt, inverse_dt: f32,
	impulses_raw: rawptr, active_mask: util.I32x8, phase: Constraint_Kernel_Phase,
)
{
	_ = inverse_dt;
	prestep := (^Weld_Prestep)(prestep_raw);
	impulses := (^Weld_Accumulated_Impulses)(impulses_raw);
	if phase == .Prestep
	{
		impulses.orientation = constraint_kernel_mask_vector3(impulses.orientation, active_mask);
		impulses.offset = constraint_kernel_mask_vector3(impulses.offset, active_mask);
		return;
	}
	if phase == .Incremental_Update
	{
		return;
	}
	offset := util.quaternion_wide_transform(prestep.local_offset, bodies[0].orientation);
	if phase == .Warmstart
	{
		constraint_kernel_weld_apply(bodies, offset, impulses.orientation, impulses.offset, active_mask);
		return;
	}
	jmjt_a := util.symmetric3x3_wide_add(bodies[0].inverse_inertia, bodies[1].inverse_inertia);
	cross_offset := util.matrix3x3_wide_create_cross_product(offset);
	jmjt_b := util.symmetric3x3_wide_multiply_matrix3x3(bodies[0].inverse_inertia, cross_offset);
	jmjt_d := util.symmetric3x3_wide_skew_sandwich(offset, bodies[0].inverse_inertia);
	diagonal_add := simd.add(bodies[0].inverse_mass, bodies[1].inverse_mass);
	jmjt_d.xx = simd.add(jmjt_d.xx, diagonal_add);
	jmjt_d.yy = simd.add(jmjt_d.yy, diagonal_add);
	jmjt_d.zz = simd.add(jmjt_d.zz, diagonal_add);
	position_error := util.vector3_wide_subtract(
		util.vector3_wide_subtract(bodies[1].position, bodies[0].position),
		offset
	);
	target_orientation_b := util.quaternion_wide_concatenate(prestep.local_orientation, bodies[0].orientation);
	rotation_error := util.quaternion_wide_concatenate(
		util.quaternion_wide_conjugate(target_orientation_b),
		bodies[1].orientation
	);
	rotation_error_axis, rotation_error_length := util.quaternion_wide_axis_angle(rotation_error);
	position_error_to_velocity, effective_mass_scale, softness := spring_settings_wide_compute(
		prestep.spring_settings,
		dt
	);
	orientation_bias := util.vector3_wide_scale(
		rotation_error_axis,
		simd.mul(rotation_error_length, position_error_to_velocity)
	);
	offset_bias := util.vector3_wide_scale(position_error, position_error_to_velocity);
	orientation_csv := util.vector3_wide_subtract(
		orientation_bias,
		util.vector3_wide_subtract(bodies[0].angular_velocity, bodies[1].angular_velocity)
	);
	point_velocity_a := util.vector3_wide_add(
		bodies[0].linear_velocity,
		util.vector3_wide_cross(bodies[0].angular_velocity, offset)
	);
	offset_csv := util.vector3_wide_subtract(
		offset_bias,
		util.vector3_wide_subtract(point_velocity_a, bodies[1].linear_velocity)
	);
	orientation_csi, offset_csi := util.symmetric6x6_wide_ldlt_solve(
		orientation_csv,
		offset_csv,
		jmjt_a,
		jmjt_b,
		jmjt_d
	);
	orientation_csi = util.vector3_wide_subtract(
		util.vector3_wide_scale(orientation_csi, effective_mass_scale),
		util.vector3_wide_scale(impulses.orientation, softness)
	);
	offset_csi = util.vector3_wide_subtract(
		util.vector3_wide_scale(offset_csi, effective_mass_scale),
		util.vector3_wide_scale(impulses.offset, softness)
	);
	orientation_csi = constraint_kernel_mask_vector3(orientation_csi, active_mask);
	offset_csi = constraint_kernel_mask_vector3(offset_csi, active_mask);
	impulses.orientation = constraint_kernel_mask_vector3(
		util.vector3_wide_add(impulses.orientation, orientation_csi),
		active_mask
	);
	impulses.offset = constraint_kernel_mask_vector3(util.vector3_wide_add(impulses.offset, offset_csi), active_mask);
	constraint_kernel_weld_apply(bodies, offset, orientation_csi, offset_csi, active_mask);
}

constraint_kernel_area_jacobians :: #force_inline proc "contextless" (
	bodies: ^[4]Constraint_Kernel_Body_Wide,
) -> (normal_length:util.F32x8, negated_a, b, c:util.Vector3_Wide, contribution_a, contribution_b, contribution_c, inverse_length:util.F32x8)
{
	ab := util.vector3_wide_subtract(bodies[1].position, bodies[0].position);
	ac := util.vector3_wide_subtract(bodies[2].position, bodies[0].position);
	normal_raw := util.vector3_wide_cross(ab, ac);
	normal_length = util.vector3_wide_length(normal_raw);
	normal_scale := util.wide_select_f32(
		transmute(util.I32x8)simd.lanes_gt(normal_length, util.F32x8(1e-10)),
		simd.div(util.F32x8(1), normal_length), util.F32x8(0),
	);
	normal := util.vector3_wide_scale(normal_raw, normal_scale);
	b = util.vector3_wide_cross(ac, normal);
	c = util.vector3_wide_cross(normal, ab);
	negated_a = util.vector3_wide_add(b, c);
	contribution_a = util.vector3_wide_length_squared(negated_a);
	contribution_b = util.vector3_wide_length_squared(b);
	contribution_c = util.vector3_wide_length_squared(c);
	length_squared := simd.max(util.F32x8(1e-14), simd.add(simd.add(contribution_a, contribution_b), contribution_c));
	inverse_length = util.approx_reciprocal_sqrt_f32x8(length_squared);
	return;
}

area_constraint_kernel :: #force_inline proc "contextless" (
	prestep_raw:rawptr, bodies:^[4]Constraint_Kernel_Body_Wide, dt, inverse_dt:f32,
	impulses_raw:rawptr, active_mask:util.I32x8, phase:Constraint_Kernel_Phase,
)
{
	_ = inverse_dt;
	prestep := (^Area_Constraint_Prestep)(prestep_raw);
	impulses := (^util.F32x8)(impulses_raw);
	if phase == .Prestep
	{
		constraint_kernel_prepare_scalar_impulses(impulses, active_mask);
		return;
	}
	if phase == .Incremental_Update
	{
		return;
	}
	normal_length, negated_a, jacobian_b, jacobian_c, contribution_a, contribution_b, contribution_c, inverse_length := constraint_kernel_area_jacobians(bodies);
	if phase == .Warmstart
	{
		warmstart := simd.mul(inverse_length, impulses^);
		constraint_kernel_apply_linear(&bodies[0], util.vector3_wide_scale(negated_a, warmstart), -1, active_mask);
		constraint_kernel_apply_linear(&bodies[1], util.vector3_wide_scale(jacobian_b, warmstart), 1, active_mask);
		constraint_kernel_apply_linear(&bodies[2], util.vector3_wide_scale(jacobian_c, warmstart), 1, active_mask);
		return;
	}
	inverse_length_squared := simd.mul(inverse_length, inverse_length);
	inverse_effective_mass := simd.mul(inverse_length_squared, simd.add(
			simd.add(simd.mul(contribution_a, bodies[0].inverse_mass), simd.mul(contribution_b, bodies[1].inverse_mass)),
			simd.mul(contribution_c, bodies[2].inverse_mass),
	));
	inverse_effective_mass = simd.max(util.F32x8(1e-14), inverse_effective_mass);
	position_error_to_velocity, effective_mass_scale, softness := spring_settings_wide_compute(
		prestep.spring_settings,
		dt
	);
	effective_mass := simd.div(effective_mass_scale, inverse_effective_mass);
	bias := simd.mul(
		simd.mul(simd.sub(prestep.target_scaled_area, normal_length), inverse_length),
		position_error_to_velocity
	);
	velocity_a := util.vector3_wide_dot(negated_a, bodies[0].linear_velocity);
	velocity_b := util.vector3_wide_dot(jacobian_b, bodies[1].linear_velocity);
	velocity_c := util.vector3_wide_dot(jacobian_c, bodies[2].linear_velocity);
	csv := simd.mul(inverse_length, simd.sub(simd.add(velocity_b, velocity_c), velocity_a));
	csi := simd.sub(simd.mul(simd.sub(bias, csv), effective_mass), simd.mul(impulses^, softness));
	csi = constraint_kernel_mask_scalar(csi, active_mask);
	impulses^ = constraint_kernel_mask_scalar(simd.add(impulses^, csi), active_mask);
	physical_impulse := simd.mul(inverse_length, csi);
	constraint_kernel_apply_linear(&bodies[0], util.vector3_wide_scale(negated_a, physical_impulse), -1, active_mask);
	constraint_kernel_apply_linear(&bodies[1], util.vector3_wide_scale(jacobian_b, physical_impulse), 1, active_mask);
	constraint_kernel_apply_linear(&bodies[2], util.vector3_wide_scale(jacobian_c, physical_impulse), 1, active_mask);
}

constraint_kernel_volume_jacobians :: #force_inline proc "contextless" (
	bodies: ^[4]Constraint_Kernel_Body_Wide,
) -> (ad, negated_a, b, c, d:util.Vector3_Wide, contribution_a, contribution_b, contribution_c, contribution_d, inverse_length:util.F32x8)
{
	ab := util.vector3_wide_subtract(bodies[1].position, bodies[0].position);
	ac := util.vector3_wide_subtract(bodies[2].position, bodies[0].position);
	ad = util.vector3_wide_subtract(bodies[3].position, bodies[0].position);
	b = util.vector3_wide_cross(ac, ad);
	c = util.vector3_wide_cross(ad, ab);
	d = util.vector3_wide_cross(ab, ac);
	negated_a = util.vector3_wide_add(util.vector3_wide_add(b, c), d);
	contribution_a = util.vector3_wide_length_squared(negated_a);
	contribution_b = util.vector3_wide_length_squared(b);
	contribution_c = util.vector3_wide_length_squared(c);
	contribution_d = util.vector3_wide_length_squared(d);
	length_squared := simd.max(
		util.F32x8(1e-14),
		simd.add(simd.add(contribution_a, contribution_b), simd.add(contribution_c, contribution_d))
	);
	inverse_length = util.approx_reciprocal_sqrt_f32x8(length_squared);
	return;
}

volume_constraint_kernel :: #force_inline proc "contextless" (
	prestep_raw:rawptr, bodies:^[4]Constraint_Kernel_Body_Wide, dt, inverse_dt:f32,
	impulses_raw:rawptr, active_mask:util.I32x8, phase:Constraint_Kernel_Phase,
)
{
	_ = inverse_dt;
	prestep := (^Volume_Constraint_Prestep)(prestep_raw);
	impulses := (^util.F32x8)(impulses_raw);
	if phase == .Prestep
	{
		constraint_kernel_prepare_scalar_impulses(impulses, active_mask);
		return;
	}
	if phase == .Incremental_Update
	{
		return;
	}
	ad, negated_a, jacobian_b, jacobian_c, jacobian_d, contribution_a, contribution_b, contribution_c, contribution_d, inverse_length := constraint_kernel_volume_jacobians(bodies);
	if phase == .Warmstart
	{
		warmstart := simd.mul(inverse_length, impulses^);
		constraint_kernel_apply_linear(&bodies[0], util.vector3_wide_scale(negated_a, warmstart), -1, active_mask);
		constraint_kernel_apply_linear(&bodies[1], util.vector3_wide_scale(jacobian_b, warmstart), 1, active_mask);
		constraint_kernel_apply_linear(&bodies[2], util.vector3_wide_scale(jacobian_c, warmstart), 1, active_mask);
		constraint_kernel_apply_linear(&bodies[3], util.vector3_wide_scale(jacobian_d, warmstart), 1, active_mask);
		return;
	}
	inverse_length_squared := simd.mul(inverse_length, inverse_length);
	inverse_effective_mass := simd.mul(inverse_length_squared, simd.add(
			simd.add(simd.mul(contribution_a, bodies[0].inverse_mass), simd.mul(contribution_b, bodies[1].inverse_mass)),
			simd.add(simd.mul(contribution_c, bodies[2].inverse_mass), simd.mul(contribution_d, bodies[3].inverse_mass)),
	));
	inverse_effective_mass = simd.max(util.F32x8(1e-14), inverse_effective_mass);
	position_error_to_velocity, effective_mass_scale, softness := spring_settings_wide_compute(
		prestep.spring_settings,
		dt
	);
	effective_mass := simd.div(effective_mass_scale, inverse_effective_mass);
	volume := util.vector3_wide_dot(jacobian_d, ad);
	bias := simd.mul(
		simd.mul(simd.sub(prestep.target_scaled_volume, volume), inverse_length),
		position_error_to_velocity
	);
	velocity_a := util.vector3_wide_dot(negated_a, bodies[0].linear_velocity);
	velocity_b := util.vector3_wide_dot(jacobian_b, bodies[1].linear_velocity);
	velocity_c := util.vector3_wide_dot(jacobian_c, bodies[2].linear_velocity);
	velocity_d := util.vector3_wide_dot(jacobian_d, bodies[3].linear_velocity);
	csv := simd.mul(inverse_length, simd.sub(simd.add(simd.add(velocity_b, velocity_c), velocity_d), velocity_a));
	csi := simd.sub(simd.mul(simd.sub(bias, csv), effective_mass), simd.mul(impulses^, softness));
	csi = constraint_kernel_mask_scalar(csi, active_mask);
	impulses^ = constraint_kernel_mask_scalar(simd.add(impulses^, csi), active_mask);
	physical_impulse := simd.mul(inverse_length, csi);
	constraint_kernel_apply_linear(&bodies[0], util.vector3_wide_scale(negated_a, physical_impulse), -1, active_mask);
	constraint_kernel_apply_linear(&bodies[1], util.vector3_wide_scale(jacobian_b, physical_impulse), 1, active_mask);
	constraint_kernel_apply_linear(&bodies[2], util.vector3_wide_scale(jacobian_c, physical_impulse), 1, active_mask);
	constraint_kernel_apply_linear(&bodies[3], util.vector3_wide_scale(jacobian_d, physical_impulse), 1, active_mask);
}
