// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"
import "core:simd"
import simd_x86 "core:simd/x86"

BODY_REFERENCE_DOES_NOT_EXIST_FLAG_INDEX :: 31;
BODY_REFERENCE_KINEMATIC_FLAG_INDEX :: 30;
BODY_REFERENCE_KINEMATIC_MASK :: u32(1 << BODY_REFERENCE_KINEMATIC_FLAG_INDEX);
BODY_REFERENCE_METADATA_MASK :: u32(1 << BODY_REFERENCE_DOES_NOT_EXIST_FLAG_INDEX) | BODY_REFERENCE_KINEMATIC_MASK;
BODY_REFERENCE_INDEX_MASK :: ~BODY_REFERENCE_METADATA_MASK;
Body_Access :: enum u8
{
	Position,
	Orientation,
	Mass,
	Inertia_Tensor,
	Linear_Velocity,
	Angular_Velocity,
}

Body_Access_Mask :: distinct bit_set[Body_Access; u8];
BODY_ACCESS_ALL :: Body_Access_Mask{
	.Position, .Orientation, .Mass, .Inertia_Tensor, .Linear_Velocity, .Angular_Velocity,
};

Inertia_Source :: enum u8
{
	Local,
	World,
}

body_reference_mobility :: proc "contextless" (encoded_reference: i32) -> Body_Mobility
{
	if u32(encoded_reference) < BODY_REFERENCE_KINEMATIC_MASK
	{
		return .Dynamic;
	}
	return .Kinematic;
}

body_validate_encoded_lane :: proc "contextless" (active: ^Body_Set, encoded: i32) -> Physics_Status
{
	if encoded < 0
	{
		return .Ok;
	}
	index := int(u32(encoded) & BODY_REFERENCE_INDEX_MASK);
	if index < 0 || index >= active.count
	{
		return .Not_Found;
	}
	return .Ok;
}

body_validate_scatter_lane :: proc "contextless" (
	active: ^Body_Set, encoded, lane_mask: i32, dynamic_only: Reference_State,
) -> Physics_Status
{
	if lane_mask == 0 || encoded < 0
	{
		return .Ok;
	}
	if dynamic_only == .Present && u32(encoded) >= BODY_REFERENCE_KINEMATIC_MASK
	{
		return .Ok;
	}
	return body_validate_encoded_lane(active, encoded);
}

@(enable_target_feature="avx")
body_load_dynamics_row :: #force_inline proc "contextless" (
	states: [^]Body_Dynamics, encoded, float_offset: i32,
) -> simd_x86.__m256
{
	if encoded < 0
	{
		return simd_x86._mm256_setzero_ps();
	}
	index := int(u32(encoded) & BODY_REFERENCE_INDEX_MASK);
	values := ([^]f32)(&states[index]);
	return simd_x86._mm256_load_ps(&values[float_offset]);
}

@(require_target_feature="avx")
body_transpose_8x8 :: #force_inline proc "contextless" (
	input: ^[8]simd_x86.__m256, output: ^[8]simd_x86.__m256,
)
{
	n0 := simd.shuffle(input[0], input[1], 0, 8, 1, 9, 4, 12, 5, 13);
	n1 := simd.shuffle(input[2], input[3], 0, 8, 1, 9, 4, 12, 5, 13);
	n2 := simd.shuffle(input[4], input[5], 0, 8, 1, 9, 4, 12, 5, 13);
	n3 := simd.shuffle(input[6], input[7], 0, 8, 1, 9, 4, 12, 5, 13);
	n4 := simd.shuffle(input[0], input[1], 2, 10, 3, 11, 6, 14, 7, 15);
	n5 := simd.shuffle(input[2], input[3], 2, 10, 3, 11, 6, 14, 7, 15);
	n6 := simd.shuffle(input[4], input[5], 2, 10, 3, 11, 6, 14, 7, 15);
	n7 := simd.shuffle(input[6], input[7], 2, 10, 3, 11, 6, 14, 7, 15);
	o0 := simd.shuffle(n0, n1, 0, 1, 8, 9, 4, 5, 12, 13);
	o1 := simd.shuffle(n2, n3, 0, 1, 8, 9, 4, 5, 12, 13);
	o2 := simd.shuffle(n4, n5, 0, 1, 8, 9, 4, 5, 12, 13);
	o3 := simd.shuffle(n6, n7, 0, 1, 8, 9, 4, 5, 12, 13);
	o4 := simd.shuffle(n0, n1, 2, 3, 10, 11, 6, 7, 14, 15);
	o5 := simd.shuffle(n2, n3, 2, 3, 10, 11, 6, 7, 14, 15);
	o6 := simd.shuffle(n4, n5, 2, 3, 10, 11, 6, 7, 14, 15);
	o7 := simd.shuffle(n6, n7, 2, 3, 10, 11, 6, 7, 14, 15);
	output[0] = simd.shuffle(o0, o1, 0, 1, 2, 3, 8, 9, 10, 11);
	output[1] = simd.shuffle(o4, o5, 0, 1, 2, 3, 8, 9, 10, 11);
	output[2] = simd.shuffle(o2, o3, 0, 1, 2, 3, 8, 9, 10, 11);
	output[3] = simd.shuffle(o6, o7, 0, 1, 2, 3, 8, 9, 10, 11);
	output[4] = simd.shuffle(o0, o1, 4, 5, 6, 7, 12, 13, 14, 15);
	output[5] = simd.shuffle(o4, o5, 4, 5, 6, 7, 12, 13, 14, 15);
	output[6] = simd.shuffle(o2, o3, 4, 5, 6, 7, 12, 13, 14, 15);
	output[7] = simd.shuffle(o6, o7, 4, 5, 6, 7, 12, 13, 14, 15);
}

@(enable_target_feature="avx")
bodies_gather_motion_kernel :: proc "contextless" (
	states: [^]Body_Dynamics, encoded_body_indices: util.I32x8,
	position: ^util.Vector3_Wide, orientation: ^util.Quaternion_Wide, velocity: ^Body_Velocity_Wide,
)
{
	indices_value := encoded_body_indices;
	indices := ([^]i32)(&indices_value);
	rows := [8]simd_x86.__m256{
		body_load_dynamics_row(states, indices[0], 0), body_load_dynamics_row(states, indices[1], 0),
		body_load_dynamics_row(states, indices[2], 0), body_load_dynamics_row(states, indices[3], 0),
		body_load_dynamics_row(states, indices[4], 0), body_load_dynamics_row(states, indices[5], 0),
		body_load_dynamics_row(states, indices[6], 0), body_load_dynamics_row(states, indices[7], 0),
	};
	columns: [8]simd_x86.__m256;
	body_transpose_8x8(&rows, &columns);
	orientation.x = columns[0];
	orientation.y = columns[1];
	orientation.z = columns[2];
	orientation.w = columns[3];
	position.x = columns[4];
	position.y = columns[5];
	position.z = columns[6];
	rows = [8]simd_x86.__m256{
		body_load_dynamics_row(states, indices[0], 8), body_load_dynamics_row(states, indices[1], 8),
		body_load_dynamics_row(states, indices[2], 8), body_load_dynamics_row(states, indices[3], 8),
		body_load_dynamics_row(states, indices[4], 8), body_load_dynamics_row(states, indices[5], 8),
		body_load_dynamics_row(states, indices[6], 8), body_load_dynamics_row(states, indices[7], 8),
	};
	body_transpose_8x8(&rows, &columns);
	velocity.linear.x = columns[0];
	velocity.linear.y = columns[1];
	velocity.linear.z = columns[2];
	velocity.angular.x = columns[4];
	velocity.angular.y = columns[5];
	velocity.angular.z = columns[6];
}

@(enable_target_feature="avx")
bodies_gather_velocity_kernel :: proc "contextless" (
	states: [^]Body_Dynamics, encoded_body_indices: util.I32x8,
	velocity: ^Body_Velocity_Wide,
)
{
	indices_value := encoded_body_indices;
	indices := ([^]i32)(&indices_value);
	rows := [8]simd_x86.__m256{
		body_load_dynamics_row(states, indices[0], 8), body_load_dynamics_row(states, indices[1], 8),
		body_load_dynamics_row(states, indices[2], 8), body_load_dynamics_row(states, indices[3], 8),
		body_load_dynamics_row(states, indices[4], 8), body_load_dynamics_row(states, indices[5], 8),
		body_load_dynamics_row(states, indices[6], 8), body_load_dynamics_row(states, indices[7], 8),
	};
	columns: [8]simd_x86.__m256;
	body_transpose_8x8(&rows, &columns);
	velocity.linear.x = columns[0];
	velocity.linear.y = columns[1];
	velocity.linear.z = columns[2];
	velocity.angular.x = columns[4];
	velocity.angular.y = columns[5];
	velocity.angular.z = columns[6];
}

@(enable_target_feature="avx")
bodies_gather_inertia_kernel :: proc "contextless" (
	states: [^]Body_Dynamics, encoded_body_indices: util.I32x8, source: Inertia_Source,
	inertia: ^Body_Inertia_Wide,
)
{
	indices_value := encoded_body_indices;
	indices := ([^]i32)(&indices_value);
	offset := i32(16);
	if source == .World
	{
		offset = 24;
	}
	rows := [8]simd_x86.__m256{
		body_load_dynamics_row(states, indices[0], offset), body_load_dynamics_row(states, indices[1], offset),
		body_load_dynamics_row(states, indices[2], offset), body_load_dynamics_row(states, indices[3], offset),
		body_load_dynamics_row(states, indices[4], offset), body_load_dynamics_row(states, indices[5], offset),
		body_load_dynamics_row(states, indices[6], offset), body_load_dynamics_row(states, indices[7], offset),
	};
	columns: [8]simd_x86.__m256;
	body_transpose_8x8(&rows, &columns);
	inertia.inverse_inertia_tensor.xx = columns[0];
	inertia.inverse_inertia_tensor.yx = columns[1];
	inertia.inverse_inertia_tensor.yy = columns[2];
	inertia.inverse_inertia_tensor.zx = columns[3];
	inertia.inverse_inertia_tensor.zy = columns[4];
	inertia.inverse_inertia_tensor.zz = columns[5];
	inertia.inverse_mass = columns[6];
}

bodies_gather_active_trusted :: proc "contextless" (
	bodies: ^Bodies, encoded_body_indices: util.I32x8, inertia_source: Inertia_Source,
	access: Body_Access_Mask = BODY_ACCESS_ALL,
) -> (
	position: util.Vector3_Wide, orientation: util.Quaternion_Wide,
	velocity: Body_Velocity_Wide, inertia: Body_Inertia_Wide,
)
{
	active := &bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
	motion_position: util.Vector3_Wide;
	motion_orientation: util.Quaternion_Wide;
	motion_velocity: Body_Velocity_Wide;
	if .Position in access || .Orientation in access
	{
		bodies_gather_motion_kernel(
			active.dynamics_state.memory,
			encoded_body_indices,
			&motion_position,
			&motion_orientation,
			&motion_velocity
		);
	}
	else if .Linear_Velocity in access || .Angular_Velocity in access
	{
		// velocity-only custom kernels do not need eight pose-row loads or
		// their transpose. preserve zero outputs for all unrequested fields
		bodies_gather_velocity_kernel(
			active.dynamics_state.memory, encoded_body_indices, &motion_velocity,
		);
	}
	if .Position in access
	{
		position = motion_position;
	}
	if .Orientation in access
	{
		orientation = motion_orientation;
	}
	if .Linear_Velocity in access
	{
		velocity.linear = motion_velocity.linear;
	}
	if .Angular_Velocity in access
	{
		velocity.angular = motion_velocity.angular;
	}
	if .Mass in access || .Inertia_Tensor in access
	{
		gathered_inertia: Body_Inertia_Wide;
		bodies_gather_inertia_kernel(
			active.dynamics_state.memory,
			encoded_body_indices,
			inertia_source,
			&gathered_inertia
		);
		if .Mass in access
		{
			inertia.inverse_mass = gathered_inertia.inverse_mass;
		}
		if .Inertia_Tensor in access
		{
			inertia.inverse_inertia_tensor = gathered_inertia.inverse_inertia_tensor;
		}
	}
	return;
}

@(require_target_feature="avx")
bodies_gather_active_no_pose_trusted :: #force_inline proc "contextless" (
	bodies: ^Bodies, encoded_body_indices: util.I32x8, inertia_source: Inertia_Source,
	velocity: ^Body_Velocity_Wide, inertia: ^Body_Inertia_Wide,
)
{
	active: ^Body_Set = &bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
	states: [^]Body_Dynamics = active.dynamics_state.memory;
	indices_value: util.I32x8 = encoded_body_indices;
	indices: [^]i32 = ([^]i32)(&indices_value);
	inertia_offset: i32 = 16;
	if inertia_source == .World
	{
		inertia_offset = 24;
	}
	velocity_rows: [8]simd_x86.__m256 = ---
	inertia_rows: [8]simd_x86.__m256 = ---
	#unroll for lane in 0..<8
	{
		encoded: i32 = indices[lane];
		if encoded < 0
		{
			velocity_rows[lane] = simd_x86.__m256(0);
			inertia_rows[lane] = simd_x86.__m256(0);
		}
		else
		{
			index: int = int(u32(encoded) & BODY_REFERENCE_INDEX_MASK);
			values: [^]f32 = ([^]f32)(&states[index]);
			velocity_rows[lane] = (^simd_x86.__m256)(&values[8])^;
			inertia_rows[lane] = (^simd_x86.__m256)(&values[inertia_offset])^;
		}
	}
	columns: [8]simd_x86.__m256 = ---
	body_transpose_8x8(&velocity_rows, &columns);
	velocity.linear.x = columns[0];
	velocity.linear.y = columns[1];
	velocity.linear.z = columns[2];
	velocity.angular.x = columns[4];
	velocity.angular.y = columns[5];
	velocity.angular.z = columns[6];
	body_transpose_8x8(&inertia_rows, &columns);
	inertia.inverse_inertia_tensor.xx = columns[0];
	inertia.inverse_inertia_tensor.yx = columns[1];
	inertia.inverse_inertia_tensor.yy = columns[2];
	inertia.inverse_inertia_tensor.zx = columns[3];
	inertia.inverse_inertia_tensor.zy = columns[4];
	inertia.inverse_inertia_tensor.zz = columns[5];
	inertia.inverse_mass = columns[6];
}

bodies_gather_active :: proc "contextless" (
	bodies: ^Bodies, encoded_body_indices: util.I32x8, inertia_source: Inertia_Source,
	access: Body_Access_Mask = BODY_ACCESS_ALL,
) -> (
	position: util.Vector3_Wide, orientation: util.Quaternion_Wide,
	velocity: Body_Velocity_Wide, inertia: Body_Inertia_Wide, status: Physics_Status,
)
{
	if bodies == nil || bodies.state != .Allocated
	{
		status = .Disposed;
		return;
	}
	active := &bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
	indices_value := encoded_body_indices;
	indices := ([^]i32)(&indices_value);
	status = body_validate_encoded_lane(active, indices[0]);
	if status != .Ok
	{
		return;
	}
	status = body_validate_encoded_lane(active, indices[1]);
	if status != .Ok
	{
		return;
	}
	status = body_validate_encoded_lane(active, indices[2]);
	if status != .Ok
	{
		return;
	}
	status = body_validate_encoded_lane(active, indices[3]);
	if status != .Ok
	{
		return;
	}
	status = body_validate_encoded_lane(active, indices[4]);
	if status != .Ok
	{
		return;
	}
	status = body_validate_encoded_lane(active, indices[5]);
	if status != .Ok
	{
		return;
	}
	status = body_validate_encoded_lane(active, indices[6]);
	if status != .Ok
	{
		return;
	}
	status = body_validate_encoded_lane(active, indices[7]);
	if status != .Ok
	{
		return;
	}
	position, orientation, velocity, inertia = bodies_gather_active_trusted(
		bodies, encoded_body_indices, inertia_source, access,
	);
	status = .Ok;
	return;
}

@(enable_target_feature="avx")
body_store_pose_row :: #force_inline proc "contextless" (
	states: [^]Body_Dynamics, encoded, lane_mask: i32, row: simd_x86.__m256,
)
{
	if lane_mask == 0 || encoded < 0
	{
		return;
	}
	index := int(u32(encoded) & BODY_REFERENCE_INDEX_MASK);
	values := ([^]f32)(&states[index]);
	simd_x86._mm256_store_ps(&values[0], row);
}

@(enable_target_feature="avx")
body_store_inertia_row :: #force_inline proc "contextless" (
	states: [^]Body_Dynamics, encoded, lane_mask: i32, row: simd_x86.__m256,
)
{
	if lane_mask == 0 || encoded < 0
	{
		return;
	}
	index := int(u32(encoded) & BODY_REFERENCE_INDEX_MASK);
	values := ([^]f32)(&states[index]);
	simd_x86._mm256_store_ps(&values[24], row);
}

@(enable_target_feature="sse,avx")
body_store_velocity_row :: #force_inline proc "contextless" (
	states: [^]Body_Dynamics, encoded: i32, access: Body_Access_Mask, row: simd_x86.__m256,
)
{
	if encoded < 0 || u32(encoded) >= BODY_REFERENCE_KINEMATIC_MASK
	{
		return;
	}
	index := int(u32(encoded) & BODY_REFERENCE_INDEX_MASK);
	values := ([^]f32)(&states[index]);
	if .Linear_Velocity in access && .Angular_Velocity in access
	{
		simd_x86._mm256_store_ps(&values[8], row);
	}
	else if .Linear_Velocity in access
	{
		simd_x86._mm_store_ps(&values[8], simd_x86._mm256_castps256_ps128(row));
	}
	else if .Angular_Velocity in access
	{
		simd_x86._mm_store_ps(&values[12], simd_x86._mm256_extractf128_ps(row, 1));
	}
}

@(enable_target_feature="avx")
bodies_scatter_pose_kernel :: proc "contextless" (
	states: [^]Body_Dynamics, encoded_body_indices, lane_mask: util.I32x8,
	position: util.Vector3_Wide, orientation: util.Quaternion_Wide,
)
{
	indices_value := encoded_body_indices;
	masks_value := lane_mask;
	indices := ([^]i32)(&indices_value);
	masks := ([^]i32)(&masks_value);
	columns := [8]simd_x86.__m256{
		orientation.x, orientation.y, orientation.z, orientation.w,
		position.x, position.y, position.z, {},
	};
	rows: [8]simd_x86.__m256;
	body_transpose_8x8(&columns, &rows);
	body_store_pose_row(states, indices[0], masks[0], rows[0]);
	body_store_pose_row(states, indices[1], masks[1], rows[1]);
	body_store_pose_row(states, indices[2], masks[2], rows[2]);
	body_store_pose_row(states, indices[3], masks[3], rows[3]);
	body_store_pose_row(states, indices[4], masks[4], rows[4]);
	body_store_pose_row(states, indices[5], masks[5], rows[5]);
	body_store_pose_row(states, indices[6], masks[6], rows[6]);
	body_store_pose_row(states, indices[7], masks[7], rows[7]);
}

@(enable_target_feature="avx")
bodies_scatter_inertia_kernel :: proc "contextless" (
	states: [^]Body_Dynamics, encoded_body_indices, lane_mask: util.I32x8, inertia: Body_Inertia_Wide,
)
{
	indices_value := encoded_body_indices;
	masks_value := lane_mask;
	indices := ([^]i32)(&indices_value);
	masks := ([^]i32)(&masks_value);
	columns := [8]simd_x86.__m256{
		inertia.inverse_inertia_tensor.xx,
		inertia.inverse_inertia_tensor.yx,
		inertia.inverse_inertia_tensor.yy,
		inertia.inverse_inertia_tensor.zx,
		inertia.inverse_inertia_tensor.zy,
		inertia.inverse_inertia_tensor.zz,
		inertia.inverse_mass,
		{},
	};
	rows: [8]simd_x86.__m256;
	body_transpose_8x8(&columns, &rows);
	body_store_inertia_row(states, indices[0], masks[0], rows[0]);
	body_store_inertia_row(states, indices[1], masks[1], rows[1]);
	body_store_inertia_row(states, indices[2], masks[2], rows[2]);
	body_store_inertia_row(states, indices[3], masks[3], rows[3]);
	body_store_inertia_row(states, indices[4], masks[4], rows[4]);
	body_store_inertia_row(states, indices[5], masks[5], rows[5]);
	body_store_inertia_row(states, indices[6], masks[6], rows[6]);
	body_store_inertia_row(states, indices[7], masks[7], rows[7]);
}

@(enable_target_feature="sse,avx")
bodies_scatter_velocities_kernel :: proc "contextless" (
	states: [^]Body_Dynamics, encoded_body_indices: util.I32x8, velocity: Body_Velocity_Wide,
	access: Body_Access_Mask,
)
{
	indices_value := encoded_body_indices;
	indices := ([^]i32)(&indices_value);
	columns := [8]simd_x86.__m256{
		velocity.linear.x, velocity.linear.y, velocity.linear.z, {},
		velocity.angular.x, velocity.angular.y, velocity.angular.z, {},
	};
	rows: [8]simd_x86.__m256;
	body_transpose_8x8(&columns, &rows);
	body_store_velocity_row(states, indices[0], access, rows[0]);
	body_store_velocity_row(states, indices[1], access, rows[1]);
	body_store_velocity_row(states, indices[2], access, rows[2]);
	body_store_velocity_row(states, indices[3], access, rows[3]);
	body_store_velocity_row(states, indices[4], access, rows[4]);
	body_store_velocity_row(states, indices[5], access, rows[5]);
	body_store_velocity_row(states, indices[6], access, rows[6]);
	body_store_velocity_row(states, indices[7], access, rows[7]);
}

bodies_scatter_active_pose_trusted :: #force_inline proc "contextless" (
	bodies: ^Bodies, encoded_body_indices, lane_mask: util.I32x8,
	position: util.Vector3_Wide, orientation: util.Quaternion_Wide,
)
{
	active := &bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
	bodies_scatter_pose_kernel(
		active.dynamics_state.memory, encoded_body_indices, lane_mask, position, orientation,
	);
}

@(enable_target_feature="avx")
bodies_scatter_active_pose :: proc "contextless" (
	bodies: ^Bodies, encoded_body_indices, lane_mask: util.I32x8,
	position: util.Vector3_Wide, orientation: util.Quaternion_Wide,
) -> Physics_Status
{
	if bodies == nil || bodies.state != .Allocated
	{
		return .Disposed;
	}
	active := &bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
	indices_value := encoded_body_indices;
	masks_value := lane_mask;
	indices := ([^]i32)(&indices_value);
	masks := ([^]i32)(&masks_value);
	status := body_validate_scatter_lane(active, indices[0], masks[0], .Missing);
	if status != .Ok
	{
		return status;
	}
	status = body_validate_scatter_lane(active, indices[1], masks[1], .Missing);
	if status != .Ok
	{
		return status;
	}
	status = body_validate_scatter_lane(active, indices[2], masks[2], .Missing);
	if status != .Ok
	{
		return status;
	}
	status = body_validate_scatter_lane(active, indices[3], masks[3], .Missing);
	if status != .Ok
	{
		return status;
	}
	status = body_validate_scatter_lane(active, indices[4], masks[4], .Missing);
	if status != .Ok
	{
		return status;
	}
	status = body_validate_scatter_lane(active, indices[5], masks[5], .Missing);
	if status != .Ok
	{
		return status;
	}
	status = body_validate_scatter_lane(active, indices[6], masks[6], .Missing);
	if status != .Ok
	{
		return status;
	}
	status = body_validate_scatter_lane(active, indices[7], masks[7], .Missing);
	if status != .Ok
	{
		return status;
	}
	bodies_scatter_active_pose_trusted(
		bodies, encoded_body_indices, lane_mask, position, orientation,
	);
	return .Ok;
}

bodies_scatter_active_inertia_trusted :: #force_inline proc "contextless" (
	bodies: ^Bodies, encoded_body_indices, lane_mask: util.I32x8, inertia: Body_Inertia_Wide,
)
{
	active := &bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
	bodies_scatter_inertia_kernel(
		active.dynamics_state.memory, encoded_body_indices, lane_mask, inertia,
	);
}

@(enable_target_feature="avx")
bodies_scatter_active_inertia :: proc "contextless" (
	bodies: ^Bodies, encoded_body_indices, lane_mask: util.I32x8, inertia: Body_Inertia_Wide,
) -> Physics_Status
{
	if bodies == nil || bodies.state != .Allocated
	{
		return .Disposed;
	}
	active := &bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
	indices_value := encoded_body_indices;
	masks_value := lane_mask;
	indices := ([^]i32)(&indices_value);
	masks := ([^]i32)(&masks_value);
	status := body_validate_scatter_lane(active, indices[0], masks[0], .Missing);
	if status != .Ok
	{
		return status;
	}
	status = body_validate_scatter_lane(active, indices[1], masks[1], .Missing);
	if status != .Ok
	{
		return status;
	}
	status = body_validate_scatter_lane(active, indices[2], masks[2], .Missing);
	if status != .Ok
	{
		return status;
	}
	status = body_validate_scatter_lane(active, indices[3], masks[3], .Missing);
	if status != .Ok
	{
		return status;
	}
	status = body_validate_scatter_lane(active, indices[4], masks[4], .Missing);
	if status != .Ok
	{
		return status;
	}
	status = body_validate_scatter_lane(active, indices[5], masks[5], .Missing);
	if status != .Ok
	{
		return status;
	}
	status = body_validate_scatter_lane(active, indices[6], masks[6], .Missing);
	if status != .Ok
	{
		return status;
	}
	status = body_validate_scatter_lane(active, indices[7], masks[7], .Missing);
	if status != .Ok
	{
		return status;
	}
	bodies_scatter_active_inertia_trusted(bodies, encoded_body_indices, lane_mask, inertia);
	return .Ok;
}

bodies_scatter_active_velocities_trusted :: #force_inline proc "contextless" (
	bodies: ^Bodies, encoded_body_indices: util.I32x8, velocity: Body_Velocity_Wide,
	access: Body_Access_Mask = {.Linear_Velocity, .Angular_Velocity},
)
{
	if .Linear_Velocity not_in access && .Angular_Velocity not_in access
	{
		return;
	}
	active := &bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
	bodies_scatter_velocities_kernel(
		active.dynamics_state.memory, encoded_body_indices, velocity, access,
	);
}

@(enable_target_feature="sse,avx")
bodies_scatter_active_velocities :: proc "contextless" (
	bodies: ^Bodies, encoded_body_indices: util.I32x8, velocity: Body_Velocity_Wide,
	access: Body_Access_Mask = {.Linear_Velocity, .Angular_Velocity},
) -> Physics_Status
{
	if bodies == nil || bodies.state != .Allocated
	{
		return .Disposed;
	}
	active := &bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
	indices_value := encoded_body_indices;
	indices := ([^]i32)(&indices_value);
	status := body_validate_scatter_lane(active, indices[0], 1, .Present);
	if status != .Ok
	{
		return status;
	}
	status = body_validate_scatter_lane(active, indices[1], 1, .Present);
	if status != .Ok
	{
		return status;
	}
	status = body_validate_scatter_lane(active, indices[2], 1, .Present);
	if status != .Ok
	{
		return status;
	}
	status = body_validate_scatter_lane(active, indices[3], 1, .Present);
	if status != .Ok
	{
		return status;
	}
	status = body_validate_scatter_lane(active, indices[4], 1, .Present);
	if status != .Ok
	{
		return status;
	}
	status = body_validate_scatter_lane(active, indices[5], 1, .Present);
	if status != .Ok
	{
		return status;
	}
	status = body_validate_scatter_lane(active, indices[6], 1, .Present);
	if status != .Ok
	{
		return status;
	}
	status = body_validate_scatter_lane(active, indices[7], 1, .Present);
	if status != .Ok
	{
		return status;
	}
	bodies_scatter_active_velocities_trusted(bodies, encoded_body_indices, velocity, access);
	return .Ok;
}
