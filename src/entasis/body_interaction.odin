package entasis

import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

@(private)
body_dynamics_mut :: #force_inline proc "contextless" (
	world: ^World, handle: Body_Handle,
) -> (^world_data, ^physics.Simulation, ^physics.Body_Dynamics, physics.Body_Memory_Location, Status)
{
	data := world_data_get(world);
	if data == nil || data.simulation.state == .Disposed
	{
		return nil, nil, nil, {}, .Disposed;
	}
	if data.simulation.state != .Ready
	{
		return nil, nil, nil, {}, .Invalid_Argument;
	}
	location, status := physics.bodies_resolve(&data.simulation.bodies, handle);
	if status != .Ok
	{
		return data, &data.simulation, nil, {}, status;
	}
	set := &data.simulation.bodies.sets.memory[location.set_index];
	return data, &data.simulation, &set.dynamics_state.memory[location.index], location, .Ok;
}

// body_apply_linear_impulse applies an instantaneous linear impulse without
// awakening the body. this matches the low-level body-reference behavior
// ownership: owner thread while the world is idle. allocation: none
body_apply_linear_impulse :: proc "contextless" (
	world: ^World, handle: Body_Handle, impulse: Vector3,
) -> Status
{
	data, _, dynamics, _, status := body_dynamics_mut(world, handle);
	if status != .Ok
	{
		return status;
	}
	world_invalidate_views_data(data);
	dynamics.motion.velocity.linear = util.vector3_add(
		dynamics.motion.velocity.linear,
		util.vector3_scale(impulse, dynamics.inertia.local.inverse_mass),
	);
	return .Ok;
}

// body_apply_angular_impulse applies an instantaneous angular impulse using the
// body's current world inverse inertia tensor without awakening the body
// ownership: owner thread while the world is idle. allocation: none
body_apply_angular_impulse :: proc "contextless" (
	world: ^World, handle: Body_Handle, angular_impulse: Vector3,
) -> Status
{
	data, _, dynamics, _, status := body_dynamics_mut(world, handle);
	if status != .Ok
	{
		return status;
	}
	world_invalidate_views_data(data);
	rotation := util.matrix3x3_from_quaternion(dynamics.motion.pose.orientation);
	world_inverse_inertia := util.symmetric3x3_rotation_sandwich(
		rotation, dynamics.inertia.local.inverse_inertia_tensor,
	);
	delta := util.symmetric3x3_transform(angular_impulse, world_inverse_inertia);
	dynamics.motion.velocity.angular = util.vector3_add(
		dynamics.motion.velocity.angular, delta,
	);
	return .Ok;
}

// body_apply_impulse applies an impulse at a world-space offset from the body
// center of mass. the body is not implicitly awakened
// ownership: owner thread while the world is idle. allocation: none
body_apply_impulse :: proc "contextless" (
	world: ^World, handle: Body_Handle, impulse, offset: Vector3,
) -> Status
{
	data, _, dynamics, _, status := body_dynamics_mut(world, handle);
	if status != .Ok
	{
		return status;
	}
	world_invalidate_views_data(data);
	dynamics.motion.velocity.linear = util.vector3_add(
		dynamics.motion.velocity.linear,
		util.vector3_scale(impulse, dynamics.inertia.local.inverse_mass),
	);
	angular_impulse := util.vector3_cross(offset, impulse);
	rotation := util.matrix3x3_from_quaternion(dynamics.motion.pose.orientation);
	world_inverse_inertia := util.symmetric3x3_rotation_sandwich(
		rotation, dynamics.inertia.local.inverse_inertia_tensor,
	);
	angular_delta := util.symmetric3x3_transform(
		angular_impulse, world_inverse_inertia,
	);
	dynamics.motion.velocity.angular = util.vector3_add(
		dynamics.motion.velocity.angular, angular_delta,
	);
	return .Ok;
}

// body_velocity_at_offset returns the world-space velocity at an offset from
// the body's center of mass. allocation: none
body_velocity_at_offset :: proc "contextless" (
	world: ^World, handle: Body_Handle, offset: Vector3,
) -> (Vector3, Status)
{
	_, _, dynamics, _, status := body_dynamics_mut(world, handle);
	if status != .Ok
	{
		return {}, status;
	}
	return util.vector3_add(
		dynamics.motion.velocity.linear,
		util.vector3_cross(dynamics.motion.velocity.angular, offset),
	), .Ok;
}

// body_bounds returns the current broad-phase bounds of a collidable body.
// sleeping bodies are read from the static broad-phase tree. allocation: none
body_bounds :: proc "contextless" (
	world: ^World, handle: Body_Handle,
) -> (Bounding_Box, Status)
{
	_, simulation, _, location, status := body_dynamics_mut(world, handle);
	if status != .Ok
	{
		return {}, status;
	}
	set := &simulation.bodies.sets.memory[location.set_index];
	collidable_value := &set.collidables.memory[location.index];
	if physics.typed_index_state(collidable_value.shape) != .Present ||
		collidable_value.broad_phase_index < 0
	{
		return {}, .Not_Found;
	}
	tree_kind, find_status := physics.broad_phase_find_body_leaf(
		&simulation.broad_phase, handle, int(collidable_value.broad_phase_index),
	);
	if find_status != .Ok
	{
		return {}, find_status;
	}
	tree := &simulation.broad_phase.active_tree;
	if tree_kind == .Static
	{
		tree = &simulation.broad_phase.static_tree;
	}
	return physics.tree_get_leaf_bounds(tree, int(collidable_value.broad_phase_index));
}

// body_update_bounds recomputes one body's broad-phase bounds after an advanced
// direct pose write. it does not refit the complete tree
// ownership: owner thread while the world is idle. allocation: none
body_update_bounds :: proc "contextless" (
	world: ^World, handle: Body_Handle,
) -> Status
{
	data, simulation, dynamics, location, status := body_dynamics_mut(world, handle);
	if status != .Ok
	{
		return status;
	}
	set := &simulation.bodies.sets.memory[location.set_index];
	collidable_value := &set.collidables.memory[location.index];
	if physics.typed_index_state(collidable_value.shape) != .Present ||
		collidable_value.broad_phase_index < 0
	{
		return .Not_Found;
	}
	bounds, bounds_status := physics.shape_registry_compute_world_bounds(
		&simulation.shapes,
		collidable_value.shape,
		dynamics.motion.pose,
	);
	if bounds_status != .Ok
	{
		return bounds_status;
	}
	tree_kind, find_status := physics.broad_phase_find_body_leaf(
		&simulation.broad_phase, handle, int(collidable_value.broad_phase_index),
	);
	if find_status != .Ok
	{
		return find_status;
	}
	tree := &simulation.broad_phase.active_tree;
	if tree_kind == .Static
	{
		tree = &simulation.broad_phase.static_tree;
	}
	update_status := physics.tree_update_bounds(
		tree, int(collidable_value.broad_phase_index), bounds,
	);
	if update_status == .Ok
	{
		world_invalidate_views_data(data);
	}
	return update_status;
}

// static_bounds returns one static's current broad-phase bounds
// allocation: none
static_bounds :: proc "contextless" (
	world: ^World, handle: Static_Handle,
) -> (Bounding_Box, Status)
{
	data := world_data_get(world);
	if data == nil || data.simulation.state == .Disposed
	{
		return {}, .Disposed;
	}
	if data.simulation.state != .Ready
	{
		return {}, .Invalid_Argument;
	}
	index, status := physics.statics_resolve(&data.simulation.statics, handle);
	if status != .Ok
	{
		return {}, status;
	}
	static_value := &data.simulation.statics.statics.memory[index];
	if static_value.broad_phase_index < 0
	{
		return {}, .Not_Found;
	}
	return physics.tree_get_leaf_bounds(
		&data.simulation.broad_phase.static_tree,
		int(static_value.broad_phase_index),
	);
}

// static_update_bounds recomputes one static's broad-phase bounds after an
// advanced direct pose write. it does not awaken overlapping bodies
// ownership: owner thread while the world is idle. allocation: none
static_update_bounds :: proc "contextless" (
	world: ^World, handle: Static_Handle,
) -> Status
{
	data := world_data_get(world);
	if data == nil || data.simulation.state == .Disposed
	{
		return .Disposed;
	}
	if data.simulation.state != .Ready
	{
		return .Invalid_Argument;
	}
	index, status := physics.statics_resolve(&data.simulation.statics, handle);
	if status != .Ok
	{
		return status;
	}
	static_value := &data.simulation.statics.statics.memory[index];
	if static_value.broad_phase_index < 0
	{
		return .Not_Found;
	}
	bounds, bounds_status := physics.shape_registry_compute_world_bounds(
		&data.simulation.shapes, static_value.shape, static_value.pose,
	);
	if bounds_status != .Ok
	{
		return bounds_status;
	}
	update_status := physics.tree_update_bounds(
		&data.simulation.broad_phase.static_tree,
		int(static_value.broad_phase_index),
		bounds,
	);
	if update_status == .Ok
	{
		world_invalidate_views_data(data);
	}
	return update_status;
}
