package phase3_body_codegen

import "base:intrinsics"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"
import simd_x86 "core:simd/x86"

Body_Dynamics_Block :: struct #align(32)
{
	values: [8]physics.Body_Dynamics,
}

@(export, enable_target_feature="avx")
phase3_body_transpose_kernel :: proc "c" (input, output: [^]f32)
{
	rows := [8]simd_x86.__m256{
		simd_x86._mm256_loadu_ps(&input[0]), simd_x86._mm256_loadu_ps(&input[8]),
		simd_x86._mm256_loadu_ps(&input[16]), simd_x86._mm256_loadu_ps(&input[24]),
		simd_x86._mm256_loadu_ps(&input[32]), simd_x86._mm256_loadu_ps(&input[40]),
		simd_x86._mm256_loadu_ps(&input[48]), simd_x86._mm256_loadu_ps(&input[56]),
	};
	columns: [8]simd_x86.__m256;
	physics.body_transpose_8x8(&rows, &columns);
	simd_x86._mm256_storeu_ps(&output[0], columns[0]);
	simd_x86._mm256_storeu_ps(&output[8], columns[1]);
	simd_x86._mm256_storeu_ps(&output[16], columns[2]);
	simd_x86._mm256_storeu_ps(&output[24], columns[3]);
	simd_x86._mm256_storeu_ps(&output[32], columns[4]);
	simd_x86._mm256_storeu_ps(&output[40], columns[5]);
	simd_x86._mm256_storeu_ps(&output[48], columns[6]);
	simd_x86._mm256_storeu_ps(&output[56], columns[7]);
}

@(export, enable_target_feature="avx")
phase3_body_motion_gather_kernel :: proc "c" (
	states: [^]physics.Body_Dynamics, encoded: [^]i32, output: [^]f32,
)
{
	indices := util.I32x8{
		encoded[0], encoded[1], encoded[2], encoded[3], encoded[4], encoded[5], encoded[6], encoded[7],
	};
	position: util.Vector3_Wide;
	orientation: util.Quaternion_Wide;
	velocity: physics.Body_Velocity_Wide;
	physics.bodies_gather_motion_kernel(states, indices, &position, &orientation, &velocity);
	simd_x86._mm256_storeu_ps(&output[0], orientation.x);
	simd_x86._mm256_storeu_ps(&output[8], orientation.w);
	simd_x86._mm256_storeu_ps(&output[16], position.x);
	simd_x86._mm256_storeu_ps(&output[24], velocity.linear.x);
	simd_x86._mm256_storeu_ps(&output[32], velocity.angular.z);
}

@(export, enable_target_feature="avx")
phase3_body_world_inertia_gather_kernel :: proc "c" (
	states: [^]physics.Body_Dynamics, encoded: [^]i32, output: [^]f32,
)
{
	indices := util.I32x8{
		encoded[0], encoded[1], encoded[2], encoded[3], encoded[4], encoded[5], encoded[6], encoded[7],
	};
	inertia: physics.Body_Inertia_Wide;
	physics.bodies_gather_inertia_kernel(states, indices, .World, &inertia);
	simd_x86._mm256_storeu_ps(&output[0], inertia.inverse_inertia_tensor.xx);
	simd_x86._mm256_storeu_ps(&output[8], inertia.inverse_inertia_tensor.zz);
	simd_x86._mm256_storeu_ps(&output[16], inertia.inverse_mass);
}

@(export, enable_target_feature="avx")
phase3_body_pose_scatter_kernel :: proc "c" (
	states: [^]physics.Body_Dynamics, encoded, masks: [^]i32, input: [^]f32,
)
{
	indices := util.I32x8{
		encoded[0], encoded[1], encoded[2], encoded[3], encoded[4], encoded[5], encoded[6], encoded[7],
	};
	lane_mask := util.I32x8{masks[0], masks[1], masks[2], masks[3], masks[4], masks[5], masks[6], masks[7]};
	position := util.Vector3_Wide{
		x=simd_x86._mm256_loadu_ps(&input[0]),
		y=simd_x86._mm256_loadu_ps(&input[8]),
		z=simd_x86._mm256_loadu_ps(&input[16]),
	};
	orientation := util.Quaternion_Wide{
		x=simd_x86._mm256_loadu_ps(&input[24]),
		y=simd_x86._mm256_loadu_ps(&input[32]),
		z=simd_x86._mm256_loadu_ps(&input[40]),
		w=simd_x86._mm256_loadu_ps(&input[48]),
	};
	physics.bodies_scatter_pose_kernel(states, indices, lane_mask, position, orientation);
}

@(export, enable_target_feature="avx")
phase3_body_inertia_scatter_kernel :: proc "c" (
	states: [^]physics.Body_Dynamics, encoded, masks: [^]i32, input: [^]f32,
)
{
	indices := util.I32x8{
		encoded[0], encoded[1], encoded[2], encoded[3], encoded[4], encoded[5], encoded[6], encoded[7],
	};
	lane_mask := util.I32x8{masks[0], masks[1], masks[2], masks[3], masks[4], masks[5], masks[6], masks[7]};
	inertia := physics.Body_Inertia_Wide{
		inverse_inertia_tensor={
			xx=simd_x86._mm256_loadu_ps(&input[0]),
			yx=simd_x86._mm256_loadu_ps(&input[8]),
			yy=simd_x86._mm256_loadu_ps(&input[16]),
			zx=simd_x86._mm256_loadu_ps(&input[24]),
			zy=simd_x86._mm256_loadu_ps(&input[32]),
			zz=simd_x86._mm256_loadu_ps(&input[40]),
		},
		inverse_mass=simd_x86._mm256_loadu_ps(&input[48]),
	};
	physics.bodies_scatter_inertia_kernel(states, indices, lane_mask, inertia);
}

@(export, enable_target_feature="sse,avx")
phase3_body_velocity_scatter_kernel :: proc "c" (
	states: [^]physics.Body_Dynamics, encoded: [^]i32, input: [^]f32,
)
{
	indices := util.I32x8{
		encoded[0], encoded[1], encoded[2], encoded[3], encoded[4], encoded[5], encoded[6], encoded[7],
	};
	velocity := physics.Body_Velocity_Wide{
		linear={
			x=simd_x86._mm256_loadu_ps(&input[0]),
			y=simd_x86._mm256_loadu_ps(&input[8]),
			z=simd_x86._mm256_loadu_ps(&input[16]),
		},
		angular={
			x=simd_x86._mm256_loadu_ps(&input[24]),
			y=simd_x86._mm256_loadu_ps(&input[32]),
			z=simd_x86._mm256_loadu_ps(&input[40]),
		},
	};
	physics.bodies_scatter_velocities_kernel(
		states, indices, velocity, {.Linear_Velocity, .Angular_Velocity},
	);
}

main :: proc()
{
	input: [64]f32;
	output: [64]f32;
	for row in 0 ..< 8
	{
		for column in 0 ..< 8
		{
			input[row * 8 + column] = f32(row * 100 + column);
		}
	}
	phase3_body_transpose_kernel(&input[0], &output[0]);
	for column in 0 ..< 8
	{
		for row in 0 ..< 8
		{
			if output[column * 8 + row] != input[row * 8 + column]
			{
				intrinsics.trap();
			}
		}
	}
	states: Body_Dynamics_Block;
	encoded := [8]i32{0, 1, 2, 3, 4, 5, 6, 7};
	masks := [8]i32{-1, -1, -1, -1, -1, -1, -1, -1};
	for lane in 0 ..< 8
	{
		states.values[lane].motion.pose.orientation.w = 1;
		states.values[lane].motion.pose.position.x = f32(lane + 1);
		states.values[lane].inertia.world.inverse_mass = f32(lane + 2);
	}
	phase3_body_motion_gather_kernel(&states.values[0], &encoded[0], &output[0]);
	if output[16] != 1
	{
		intrinsics.trap();
	}
	phase3_body_world_inertia_gather_kernel(&states.values[0], &encoded[0], &output[0]);
	if output[16] != 2
	{
		intrinsics.trap();
	}
	phase3_body_pose_scatter_kernel(&states.values[0], &encoded[0], &masks[0], &input[0]);
	phase3_body_inertia_scatter_kernel(&states.values[0], &encoded[0], &masks[0], &input[0]);
	phase3_body_velocity_scatter_kernel(&states.values[0], &encoded[0], &input[0]);
	volatile := output[63];
	intrinsics.volatile_store(&volatile, volatile);
}
