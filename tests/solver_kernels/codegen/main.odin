package phase5_solver_codegen

import "base:intrinsics"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"
import "core:simd"
import simd_x86 "core:simd/x86"

@(enable_target_feature="avx")
phase5_codegen_bodies :: #force_inline proc "contextless" (
	input: [^]f32, bodies: ^[4]physics.Constraint_Kernel_Body_Wide,
) -> (values0, values1: util.F32x8, mask: util.I32x8)
{
	values0 = simd_x86._mm256_loadu_ps(&input[0]);
	values1 = simd_x86._mm256_loadu_ps(&input[8]);
	zero := simd_x86._mm256_setzero_ps();
	one := simd_x86._mm256_set1_ps(1);
	mask = transmute(util.I32x8)simd.lanes_gt(values0, zero);
	bodies[0].position = {x=zero, y=zero, z=zero};
	bodies[1].position = {x=values0, y=simd_x86._mm256_mul_ps(values1, simd_x86._mm256_set1_ps(0.25)), z=zero};
	bodies[2].position = {x=zero, y=values1, z=simd_x86._mm256_set1_ps(0.5)};
	bodies[3].position = {x=simd_x86._mm256_set1_ps(0.5), y=zero, z=simd_x86._mm256_sub_ps(values0, values1)};
	for body_index in 0 ..< 4
	{
		bodies[body_index].orientation.w = one;
		bodies[body_index].linear_velocity = {
			x=simd_x86._mm256_mul_ps(values1, simd_x86._mm256_set1_ps(0.1 * f32(body_index + 1))),
			y=simd_x86._mm256_mul_ps(values0, simd_x86._mm256_set1_ps(-0.025 * f32(body_index + 1))),
			z=zero,
		};
		bodies[body_index].angular_velocity = {
			x=zero,
			y=simd_x86._mm256_mul_ps(values1, simd_x86._mm256_set1_ps(0.05 * f32(body_index + 1))),
			z=simd_x86._mm256_mul_ps(values0, simd_x86._mm256_set1_ps(0.025 * f32(body_index + 1))),
		};
		bodies[body_index].inverse_mass = one;
		bodies[body_index].inverse_inertia.xx = one;
		bodies[body_index].inverse_inertia.yy = simd_x86._mm256_set1_ps(1.25);
		bodies[body_index].inverse_inertia.zz = simd_x86._mm256_set1_ps(1.5);
	}
	return;
}

@(enable_target_feature="avx")
phase5_codegen_spring :: #force_inline proc "contextless" () -> physics.Spring_Settings_Wide
{
	return {angular_frequency=simd_x86._mm256_set1_ps(12.5), twice_damping_ratio=simd_x86._mm256_set1_ps(1.5)};
}

@(enable_target_feature="avx")
phase5_codegen_servo :: #force_inline proc "contextless" () -> physics.Servo_Settings_Wide
{
	return {
		maximum_speed=simd_x86._mm256_set1_ps(25),
		base_speed=simd_x86._mm256_set1_ps(0.5),
		maximum_force=simd_x86._mm256_set1_ps(1000),
	};
}

@(enable_target_feature="avx")
phase5_codegen_contact :: #force_inline proc "contextless" (
	values0, values1: util.F32x8,
) -> (contacts: [4]physics.Convex_Contact_Wide, material: physics.Contact_Material_Wide)
{
	zero := simd_x86._mm256_setzero_ps();
	for contact_index in 0 ..< 4
	{
		scale := simd_x86._mm256_set1_ps(0.05 * f32(contact_index + 1));
		contacts[contact_index] = {
			offset_a={
				x=simd_x86._mm256_mul_ps(values0, scale),
				y=simd_x86._mm256_mul_ps(values1, simd_x86._mm256_set1_ps(0.025 * f32(contact_index))),
				z=zero,
			},
			depth=simd_x86._mm256_mul_ps(values1, scale),
		};
	}
	material = {
		friction_coefficient=simd_x86._mm256_set1_ps(0.7),
		spring_settings=phase5_codegen_spring(),
		maximum_recovery_velocity=simd_x86._mm256_set1_ps(3),
	};
	return;
}

@(export, enable_target_feature="avx")
phase5_one_body_linear_servo_kernel :: proc "c" (input, output: [^]f32)
{
	bodies: [4]physics.Constraint_Kernel_Body_Wide;
	values0, values1, mask := phase5_codegen_bodies(input, &bodies);
	prestep := physics.One_Body_Linear_Servo_Prestep{
		local_offset={x=simd_x86._mm256_set1_ps(0.25), y=simd_x86._mm256_set1_ps(0.5), z=simd_x86._mm256_set1_ps(0.75)},
		target={x=values0, y=values1, z=simd_x86._mm256_sub_ps(values0, values1)},
		spring_settings=phase5_codegen_spring(),
		servo_settings=phase5_codegen_servo(),
	};
	impulses := util.Vector3_Wide{
		x=simd_x86._mm256_set1_ps(0.01),
		y=simd_x86._mm256_set1_ps(0.02),
		z=simd_x86._mm256_set1_ps(0.03),
	};
	physics.one_body_linear_servo_kernel(&prestep, &bodies, 1.0 / 60.0, 60, &impulses, mask, .Prestep);
	physics.one_body_linear_servo_kernel(&prestep, &bodies, 1.0 / 60.0, 60, &impulses, mask, .Warmstart);
	physics.one_body_linear_servo_kernel(&prestep, &bodies, 1.0 / 60.0, 60, &impulses, mask, .Solve);
	simd_x86._mm256_storeu_ps(&output[0], bodies[0].linear_velocity.x);
	simd_x86._mm256_storeu_ps(&output[8], bodies[0].linear_velocity.y);
	simd_x86._mm256_storeu_ps(&output[16], bodies[0].angular_velocity.z);
	simd_x86._mm256_storeu_ps(&output[24], impulses.x);
}

@(export, enable_target_feature="avx")
phase5_ball_socket_kernel :: proc "c" (input, output: [^]f32)
{
	bodies: [4]physics.Constraint_Kernel_Body_Wide;
	_, _, mask := phase5_codegen_bodies(input, &bodies);
	prestep := physics.Ball_Socket_Prestep{
		local_offset_a={x=simd_x86._mm256_set1_ps(0.1), y=simd_x86._mm256_set1_ps(0.2), z=simd_x86._mm256_set1_ps(0.3)},
		local_offset_b={x=simd_x86._mm256_set1_ps(0.4), y=simd_x86._mm256_set1_ps(0.5), z=simd_x86._mm256_set1_ps(0.6)},
		spring_settings=phase5_codegen_spring(),
	};
	impulses := util.Vector3_Wide{
		x=simd_x86._mm256_set1_ps(0.01),
		y=simd_x86._mm256_set1_ps(0.02),
		z=simd_x86._mm256_set1_ps(0.03),
	};
	physics.ball_socket_kernel(&prestep, &bodies, 1.0 / 60.0, 60, &impulses, mask, .Prestep);
	physics.ball_socket_kernel(&prestep, &bodies, 1.0 / 60.0, 60, &impulses, mask, .Warmstart);
	physics.ball_socket_kernel(&prestep, &bodies, 1.0 / 60.0, 60, &impulses, mask, .Solve);
	simd_x86._mm256_storeu_ps(&output[0], bodies[0].linear_velocity.x);
	simd_x86._mm256_storeu_ps(&output[8], bodies[1].linear_velocity.x);
	simd_x86._mm256_storeu_ps(&output[16], impulses.y);
	simd_x86._mm256_storeu_ps(&output[24], impulses.z);
}

@(export, enable_target_feature="avx")
phase5_area_constraint_kernel :: proc "c" (input, output: [^]f32)
{
	bodies: [4]physics.Constraint_Kernel_Body_Wide;
	_, _, mask := phase5_codegen_bodies(input, &bodies);
	prestep := physics.Area_Constraint_Prestep{
		target_scaled_area=simd_x86._mm256_set1_ps(3),
		spring_settings=phase5_codegen_spring(),
	};
	impulses := simd_x86._mm256_set1_ps(0.02);
	physics.area_constraint_kernel(&prestep, &bodies, 1.0 / 60.0, 60, &impulses, mask, .Prestep);
	physics.area_constraint_kernel(&prestep, &bodies, 1.0 / 60.0, 60, &impulses, mask, .Warmstart);
	physics.area_constraint_kernel(&prestep, &bodies, 1.0 / 60.0, 60, &impulses, mask, .Solve);
	simd_x86._mm256_storeu_ps(&output[0], bodies[0].linear_velocity.x);
	simd_x86._mm256_storeu_ps(&output[8], bodies[1].linear_velocity.y);
	simd_x86._mm256_storeu_ps(&output[16], bodies[2].linear_velocity.z);
	simd_x86._mm256_storeu_ps(&output[24], impulses);
}

@(export, enable_target_feature="avx")
phase5_volume_constraint_kernel :: proc "c" (input, output: [^]f32)
{
	bodies: [4]physics.Constraint_Kernel_Body_Wide;
	_, _, mask := phase5_codegen_bodies(input, &bodies);
	prestep := physics.Volume_Constraint_Prestep{
		target_scaled_volume=simd_x86._mm256_set1_ps(3),
		spring_settings=phase5_codegen_spring(),
	};
	impulses := simd_x86._mm256_set1_ps(0.02);
	physics.volume_constraint_kernel(&prestep, &bodies, 1.0 / 60.0, 60, &impulses, mask, .Prestep);
	physics.volume_constraint_kernel(&prestep, &bodies, 1.0 / 60.0, 60, &impulses, mask, .Warmstart);
	physics.volume_constraint_kernel(&prestep, &bodies, 1.0 / 60.0, 60, &impulses, mask, .Solve);
	simd_x86._mm256_storeu_ps(&output[0], bodies[0].linear_velocity.x);
	simd_x86._mm256_storeu_ps(&output[8], bodies[3].linear_velocity.z);
	simd_x86._mm256_storeu_ps(&output[16], impulses);
	simd_x86._mm256_storeu_ps(&output[24], prestep.target_scaled_volume);
}

@(export, enable_target_feature="avx")
phase5_contact4_one_body_kernel :: proc "c" (input, output: [^]f32)
{
	bodies: [4]physics.Constraint_Kernel_Body_Wide;
	values0, values1, mask := phase5_codegen_bodies(input, &bodies);
	contacts, material := phase5_codegen_contact(values0, values1);
	prestep := physics.Contact_4_One_Body_Prestep{
		contacts=contacts,
		normal={y=simd_x86._mm256_set1_ps(1)},
		material=material,
	};
	impulses := physics.Contact_4_Accumulated_Impulses{
		tangent={x=simd_x86._mm256_set1_ps(0.01), y=simd_x86._mm256_set1_ps(0.02)},
		penetration={
			simd_x86._mm256_set1_ps(0.03),
			simd_x86._mm256_set1_ps(0.04),
			simd_x86._mm256_set1_ps(0.05),
			simd_x86._mm256_set1_ps(0.06),
		},
		twist=simd_x86._mm256_set1_ps(0.01),
	};
	physics.contact_4_one_body_kernel(&prestep, &bodies, 1.0 / 60.0, 60, &impulses, mask, .Prestep);
	physics.contact_4_one_body_kernel(&prestep, &bodies, 1.0 / 60.0, 60, &impulses, mask, .Warmstart);
	physics.contact_4_one_body_kernel(&prestep, &bodies, 1.0 / 60.0, 60, &impulses, mask, .Solve);
	simd_x86._mm256_storeu_ps(&output[0], bodies[0].linear_velocity.x);
	simd_x86._mm256_storeu_ps(&output[8], impulses.penetration[0]);
	simd_x86._mm256_storeu_ps(&output[16], impulses.penetration[3]);
	simd_x86._mm256_storeu_ps(&output[24], impulses.tangent.x);
}

@(export, enable_target_feature="avx")
phase5_contact4_two_body_kernel :: proc "c" (input, output: [^]f32)
{
	bodies: [4]physics.Constraint_Kernel_Body_Wide;
	values0, values1, mask := phase5_codegen_bodies(input, &bodies);
	contacts, material := phase5_codegen_contact(values0, values1);
	prestep := physics.Contact_4_Prestep{
		contacts=contacts,
		offset_b={x=simd_x86._mm256_set1_ps(0.5), y=simd_x86._mm256_set1_ps(0.25), z=simd_x86._mm256_set1_ps(0.75)},
		normal={y=simd_x86._mm256_set1_ps(1)},
		material=material,
	};
	impulses := physics.Contact_4_Accumulated_Impulses{
		tangent={x=simd_x86._mm256_set1_ps(0.01), y=simd_x86._mm256_set1_ps(0.02)},
		penetration={
			simd_x86._mm256_set1_ps(0.03),
			simd_x86._mm256_set1_ps(0.04),
			simd_x86._mm256_set1_ps(0.05),
			simd_x86._mm256_set1_ps(0.06),
		},
		twist=simd_x86._mm256_set1_ps(0.01),
	};
	physics.contact_4_kernel(&prestep, &bodies, 1.0 / 60.0, 60, &impulses, mask, .Prestep);
	physics.contact_4_kernel(&prestep, &bodies, 1.0 / 60.0, 60, &impulses, mask, .Warmstart);
	physics.contact_4_kernel(&prestep, &bodies, 1.0 / 60.0, 60, &impulses, mask, .Solve);
	simd_x86._mm256_storeu_ps(&output[0], bodies[0].linear_velocity.x);
	simd_x86._mm256_storeu_ps(&output[8], bodies[1].linear_velocity.x);
	simd_x86._mm256_storeu_ps(&output[16], impulses.penetration[3]);
	simd_x86._mm256_storeu_ps(&output[24], impulses.tangent.x);
}

main :: proc()
{
	input: [16]f32;
	output: [32]f32;
	for lane in 0 ..< 8
	{
		input[lane] = 2 + f32(lane) * 0.125;
		input[8 + lane] = 1 + f32(lane) * 0.0625;
	}
	phase5_one_body_linear_servo_kernel(&input[0], &output[0]);
	if output[24] == 0
	{
		intrinsics.trap();
	}
	phase5_ball_socket_kernel(&input[0], &output[0]);
	if output[16] == 0
	{
		intrinsics.trap();
	}
	phase5_area_constraint_kernel(&input[0], &output[0]);
	if output[24] == 0
	{
		intrinsics.trap();
	}
	phase5_volume_constraint_kernel(&input[0], &output[0]);
	if output[16] == 0
	{
		intrinsics.trap();
	}
	phase5_contact4_one_body_kernel(&input[0], &output[0]);
	if output[8] == 0
	{
		intrinsics.trap();
	}
	phase5_contact4_two_body_kernel(&input[0], &output[0]);
	if output[16] == 0
	{
		intrinsics.trap();
	}
	volatile := output[31];
	intrinsics.volatile_store(&volatile, volatile);
}
