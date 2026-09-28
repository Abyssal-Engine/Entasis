package entasis

import "core:math"
import physics "entasis:entasis_physics"

// pose constructs a rigid pose. the default orientation is identity
pose :: #force_inline proc "contextless" (
	position: Vector3 = {}, orientation: Quaternion = {w=1},
) -> Rigid_Pose
{
	return {orientation=orientation, position=position};
}

// velocity constructs body linear and angular velocity
velocity :: #force_inline proc "contextless" (
	linear: Vector3 = {}, angular: Vector3 = {},
) -> Body_Velocity
{
	return {linear=linear, angular=angular};
}

// body_activity constructs sleeping thresholds. a negative sleep threshold
// disables automatic sleep for the body
body_activity :: #force_inline proc "contextless" (
	sleep_threshold: f32 = 0.01,
	minimum_timesteps_under_threshold: u8 = 32,
) -> Activity_Description
{
	return {
		sleep_threshold=sleep_threshold,
		minimum_timestep_count_under_threshold=minimum_timesteps_under_threshold,
	};
}

// body_activity_default returns the ordinary facade sleep policy
body_activity_default :: #force_inline proc "contextless" () -> Activity_Description
{
	return body_activity();
}

// collidable constructs a body collidable description. the default matches the
// low-level implicit Shape_Handle conversion: passive continuity, no minimum
// margin, and an unbounded maximum speculative margin
collidable :: #force_inline proc "contextless" (
	shape: Shape_Handle,
	continuity: Continuous_Detection = {mode=.Passive},
	minimum_speculative_margin: f32 = 0,
	maximum_speculative_margin: f32 = f32(math.F32_MAX),
) -> Collidable_Description
{
	return {
		shape=shape,
		continuity=continuity,
		minimum_speculative_margin=minimum_speculative_margin,
		maximum_speculative_margin=maximum_speculative_margin,
	};
}

// body_dynamic constructs a dynamic collidable body description
body_dynamic :: #force_inline proc "contextless" (
	shape: Shape_Handle,
	inertia: Body_Inertia,
	body_pose: Rigid_Pose,
	body_velocity: Body_Velocity = {},
	activity: Activity_Description = {
		sleep_threshold=0.01,
		minimum_timestep_count_under_threshold=32,
	},
) -> Body_Description
{
	return {
		pose=body_pose,
		velocity=body_velocity,
		local_inertia=inertia,
		collidable=collidable(shape),
		activity=activity,
	};
}

// body_kinematic constructs a kinematic collidable body description
body_kinematic :: #force_inline proc "contextless" (
	shape: Shape_Handle,
	body_pose: Rigid_Pose,
	body_velocity: Body_Velocity = {},
	activity: Activity_Description = {
		sleep_threshold=0.01,
		minimum_timestep_count_under_threshold=32,
	},
) -> Body_Description
{
	return {
		pose=body_pose,
		velocity=body_velocity,
		local_inertia={},
		collidable=collidable(shape),
		activity=activity,
	};
}

// body_shapeless constructs a dynamic body with no broad-phase collidable
body_shapeless :: #force_inline proc "contextless" (
	inertia: Body_Inertia,
	body_pose: Rigid_Pose,
	body_velocity: Body_Velocity = {},
	activity: Activity_Description = {
		sleep_threshold=0.01,
		minimum_timestep_count_under_threshold=32,
	},
) -> Body_Description
{
	return {
		pose=body_pose,
		velocity=body_velocity,
		local_inertia=inertia,
		collidable={},
		activity=activity,
	};
}

// body_add adds one body. allocation may occur when world capacity grows
// ownership: owner thread only while the world is idle
body_add :: #force_inline proc (
	world: ^World, description: Body_Description,
) -> (Body_Handle, Status)
{
	world_invalidate_views(world);
	stored_description := description;
	return physics.simulation_add_body(world_simulation(world), &stored_description);
}

// body_remove removes a body and every connected constraint
// ownership: owner thread only while the world is idle
body_remove :: #force_inline proc (
	world: ^World, handle: Body_Handle,
) -> Status
{
	world_invalidate_views(world);
	return physics.simulation_remove_body(world_simulation(world), handle);
}

// body_get returns a stable snapshot description for an active or sleeping body
// allocation: none. the returned value contains no pointers into simulation data
body_get :: #force_inline proc "contextless" (
	world: ^World, handle: Body_Handle,
) -> (Body_State, Status)
{
	simulation := world_simulation(world);
	if simulation == nil || simulation.state != .Ready
	{
		return {}, .Disposed;
	}
	return physics.bodies_get_description(&simulation.bodies, handle);
}

// body_apply replaces the complete body description. the body is awakened and
// broad-phase membership is updated when required
// ownership: owner thread only while the world is idle
body_apply :: #force_inline proc (
	world: ^World, handle: Body_Handle, description: Body_Description,
) -> Status
{
	world_invalidate_views(world);
	stored_description := description;
	return physics.simulation_apply_body_description(
		world_simulation(world), handle, &stored_description,
	);
}

// body_set_pose updates the pose through the complete body mutation path
body_set_pose :: #force_inline proc (
	world: ^World, handle: Body_Handle, body_pose: Rigid_Pose,
) -> Status
{
	state, status := body_get(world, handle);
	if status != .Ok
	{
		return status;
	}
	state.pose = body_pose;
	return body_apply(world, handle, state);
}

// body_set_velocity updates velocity and awakens a sleeping body
body_set_velocity :: #force_inline proc (
	world: ^World, handle: Body_Handle, body_velocity: Body_Velocity,
) -> Status
{
	state, status := body_get(world, handle);
	if status != .Ok
	{
		return status;
	}
	state.velocity = body_velocity;
	return body_apply(world, handle, state);
}

// body_set_inertia updates local inertia and handles dynamic/kinematic changes
body_set_inertia :: #force_inline proc (
	world: ^World, handle: Body_Handle, inertia: Body_Inertia,
) -> Status
{
	state, status := body_get(world, handle);
	if status != .Ok
	{
		return status;
	}
	state.local_inertia = inertia;
	return body_apply(world, handle, state);
}

// body_set_activity updates sleep thresholds and resets the activity counters
body_set_activity :: #force_inline proc (
	world: ^World, handle: Body_Handle, activity: Activity_Description,
) -> Status
{
	state, status := body_get(world, handle);
	if status != .Ok
	{
		return status;
	}
	state.activity = activity;
	return body_apply(world, handle, state);
}

// body_set_collidable updates shape, continuity, and speculative margins
body_set_collidable :: #force_inline proc (
	world: ^World, handle: Body_Handle, value: Collidable_Description,
) -> Status
{
	state, status := body_get(world, handle);
	if status != .Ok
	{
		return status;
	}
	state.collidable = value;
	return body_apply(world, handle, state);
}

// body_set_shape updates only the shape while retaining the other collidable
// settings. it maintains shape reference counts and broad-phase membership
body_set_shape :: #force_inline proc (
	world: ^World, handle: Body_Handle, shape: Shape_Handle,
) -> Status
{
	state, status := body_get(world, handle);
	if status != .Ok
	{
		return status;
	}
	state.collidable.shape = shape;
	return body_apply(world, handle, state);
}
