// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"
import "core:math"

Rigid_Pose :: struct
{
	orientation: util.Quaternion,
	position:    util.Vector3,
	_padding:    f32,
}

Body_Velocity :: struct
{
	linear:           util.Vector3,
	_linear_padding:  f32,
	angular:          util.Vector3,
	_angular_padding: f32,
}

Motion_State :: struct
{
	pose:     Rigid_Pose,
	velocity: Body_Velocity,
}

Body_Inertia :: struct
{
	inverse_inertia_tensor: util.Symmetric3x3,
	inverse_mass:           f32,
	_padding:               f32,
}

Body_Inertias :: struct
{
	local: Body_Inertia,
	world: Body_Inertia,
}

Body_Dynamics :: struct
{
	motion:  Motion_State,
	inertia: Body_Inertias,
}

Body_Velocity_Wide :: struct
{
	linear:  util.Vector3_Wide,
	angular: util.Vector3_Wide,
}

Body_Inertia_Wide :: struct
{
	inverse_inertia_tensor: util.Symmetric3x3_Wide,
	inverse_mass:           util.F32x8,
}

Body_Activity :: struct
{
	sleep_threshold:                  f32,
	minimum_timesteps_under_threshold: u8,
	timesteps_under_threshold_count:   u8,
	sleep_candidate:                  Sleep_Candidate_State,
	_padding:                         u8,
}

Body_Activity_Description :: struct
{
	sleep_threshold:                       f32,
	minimum_timestep_count_under_threshold: u8,
	_padding:                              [3]u8,
}

Body_Description :: struct
{
	pose:          Rigid_Pose,
	velocity:      Body_Velocity,
	local_inertia: Body_Inertia,
	collidable:    Collidable_Description,
	activity:      Body_Activity_Description,
}

Static_Description :: struct
{
	pose:       Rigid_Pose,
	shape:      Typed_Index,
	continuity: Continuous_Detection,
}

Static :: struct
{
	pose:              Rigid_Pose,
	shape:             Typed_Index,
	continuity:        Continuous_Detection,
	broad_phase_index: i32,
}

Body_Constraint_Reference :: struct
{
	connecting_constraint_handle: Constraint_Handle,
	body_index_in_constraint:     i32,
}

rigid_pose_identity :: proc "contextless" () -> Rigid_Pose
{
	return {orientation=util.quaternion_identity()};
}

rigid_pose_transform :: proc "contextless" (value: util.Vector3, pose: Rigid_Pose) -> util.Vector3
{
	return util.vector3_add(util.quaternion_transform(value, pose.orientation), pose.position);
}

rigid_pose_transform_by_inverse :: proc "contextless" (value: util.Vector3, pose: Rigid_Pose) -> util.Vector3
{
	return util.quaternion_transform(
		util.vector3_subtract(value, pose.position),
		util.quaternion_conjugate(pose.orientation)
	);
}

rigid_pose_invert :: proc "contextless" (pose: Rigid_Pose) -> Rigid_Pose
{
	orientation := util.quaternion_conjugate(pose.orientation);
	return {
		orientation=orientation,
		position=util.quaternion_transform(util.vector3_negate(pose.position), orientation),
	};
}

rigid_pose_concatenate :: proc "contextless" (a, b: Rigid_Pose) -> Rigid_Pose
{
	return {
		orientation=util.quaternion_concatenate(a.orientation, b.orientation),
		position=util.vector3_add(util.quaternion_transform(a.position, b.orientation), b.position),
	};
}

body_inertia_mobility :: proc "contextless" (inertia: Body_Inertia) -> Body_Mobility
{
	t := inertia.inverse_inertia_tensor;
	if inertia.inverse_mass == 0 && t.xx == 0 && t.yx == 0 && t.yy == 0 && t.zx == 0 && t.zy == 0 && t.zz == 0
	{
		return .Kinematic;
	}
	return .Dynamic;
}

body_description_validate :: proc "contextless" (description: ^Body_Description) -> Physics_Status
{
	if description == nil || description.local_inertia.inverse_mass < 0
	{
		return .Invalid_Description;
	}
	q := description.pose.orientation;
	length_squared := util.quaternion_length_squared(q);
	if math.abs(length_squared - 1) > 1e-3
	{
		return .Invalid_Description;
	}
	if description.collidable.minimum_speculative_margin < 0 ||
		description.collidable.maximum_speculative_margin < description.collidable.minimum_speculative_margin
	{
		return .Invalid_Description;
	}
	return .Ok;
}

static_description_validate :: proc "contextless" (description: ^Static_Description) -> Physics_Status
{
	if description == nil || typed_index_state(description.shape) != .Present
	{
		return .Invalid_Description;
	}
	if math.abs(util.quaternion_length_squared(description.pose.orientation) - 1) > 1e-3
	{
		return .Invalid_Description;
	}
	return .Ok;
}

#assert(size_of(Rigid_Pose) == 32);
#assert(size_of(Body_Velocity) == 32);
#assert(size_of(Motion_State) == 64);
#assert(size_of(Body_Inertia) == 32);
#assert(size_of(Body_Inertias) == 64);
#assert(size_of(Body_Dynamics) == 128);
#assert(size_of(Body_Activity) == 8);
