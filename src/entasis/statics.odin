package entasis

import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

// Awakening_Policy selects whether a static mutation wakes sleeping dynamic
// bodies whose bounds overlap the old or new static bounds
Awakening_Policy :: enum u8
{
	Overlaps,
	None,
}

// static_body constructs a static description with discrete continuity
static_body :: #force_inline proc "contextless" (
	shape: Shape_Handle,
	static_pose: Rigid_Pose,
	continuity: Continuous_Detection = {},
) -> Static_Description
{
	return {pose=static_pose, shape=shape, continuity=continuity};
}

// static_add adds one static and applies the selected awakening policy.
// allocation may occur when world capacity grows
// ownership: owner thread only while the world is idle
static_add :: #force_inline proc (
	world: ^World,
	description: Static_Description,
	awakening: Awakening_Policy = .Overlaps,
) -> (Static_Handle, Status)
{
	stored_description := description;
	switch awakening
	{
		case .Overlaps:
		world_invalidate_views(world);
		return physics.simulation_add_static(
			world_simulation(world), &stored_description,
		);
		case .None:
		world_invalidate_views(world);
		return physics.simulation_add_static_without_awakening_bodies(
			world_simulation(world), &stored_description,
		);
	}
	return static_handle_invalid(), .Invalid_Argument;
}

// static_remove removes one static and applies the selected awakening policy
// ownership: owner thread only while the world is idle
static_remove :: #force_inline proc (
	world: ^World,
	handle: Static_Handle,
	awakening: Awakening_Policy = .Overlaps,
) -> Status
{
	switch awakening
	{
		case .Overlaps:
		world_invalidate_views(world);
		return physics.simulation_remove_static(world_simulation(world), handle);
		case .None:
		world_invalidate_views(world);
		return physics.simulation_remove_static_without_awakening_bodies(
			world_simulation(world), handle,
		);
	}
	return .Invalid_Argument;
}

// static_get returns a stable snapshot description
// allocation: none. the returned value contains no simulation pointers
static_get :: #force_inline proc "contextless" (
	world: ^World, handle: Static_Handle,
) -> (Static_State, Status)
{
	simulation := world_simulation(world);
	if simulation == nil || simulation.state != .Ready
	{
		return {}, .Disposed;
	}
	return physics.statics_get_description(&simulation.statics, handle);
}

// static_apply replaces the complete static description and applies the
// selected awakening policy to overlapping sleeping bodies
// ownership: owner thread only while the world is idle
static_apply :: #force_inline proc (
	world: ^World,
	handle: Static_Handle,
	description: Static_Description,
	awakening: Awakening_Policy = .Overlaps,
) -> Status
{
	stored_description := description;
	switch awakening
	{
		case .Overlaps:
		world_invalidate_views(world);
		return physics.simulation_apply_static_description(
			world_simulation(world), handle, &stored_description,
		);
		case .None:
		world_invalidate_views(world);
		return physics.simulation_apply_static_description_without_awakening_bodies(
			world_simulation(world), handle, &stored_description,
		);
	}
	return .Invalid_Argument;
}

// static_set_pose updates only the pose
static_set_pose :: #force_inline proc (
	world: ^World,
	handle: Static_Handle,
	static_pose: Rigid_Pose,
	awakening: Awakening_Policy = .Overlaps,
) -> Status
{
	state, status := static_get(world, handle);
	if status != .Ok
	{
		return status;
	}
	state.pose = static_pose;
	return static_apply(world, handle, state, awakening);
}

// static_set_shape updates only the shape
static_set_shape :: #force_inline proc (
	world: ^World,
	handle: Static_Handle,
	shape: Shape_Handle,
	awakening: Awakening_Policy = .Overlaps,
) -> Status
{
	state, status := static_get(world, handle);
	if status != .Ok
	{
		return status;
	}
	state.shape = shape;
	return static_apply(world, handle, state, awakening);
}

// static_set_continuity updates only continuous-detection settings
static_set_continuity :: #force_inline proc (
	world: ^World,
	handle: Static_Handle,
	continuity: Continuous_Detection,
	awakening: Awakening_Policy = .Overlaps,
) -> Status
{
	state, status := static_get(world, handle);
	if status != .Ok
	{
		return status;
	}
	state.continuity = continuity;
	return static_apply(world, handle, state, awakening);
}

// Static_Awakening_Filter_Proc selects sleeping dynamic bodies whose islands
// may be awakened by a static mutation. the callback may run only on the owner
// thread before the mutation is committed and must allocate no physics state
Static_Awakening_Filter_Proc :: physics.Static_Awakening_Filter_Proc;
// static_add_filtered adds a static and awakens only overlapping sleeping
// islands containing at least one body accepted by filter
static_add_filtered :: proc (
	world: ^World,
	description: Static_Description,
	filter: Static_Awakening_Filter_Proc,
	user_context: rawptr = nil,
) -> (Static_Handle, Status)
{
	if filter == nil
	{
		return static_add(world, description, .Overlaps);
	}
	simulation := world_simulation(world);
	if simulation == nil || simulation.state != .Ready
	{
		return static_handle_invalid(), .Disposed;
	}
	bounds, bounds_status := physics.shape_registry_compute_world_bounds(
		physics.simulation_shape_registry(simulation),
		description.shape, description.pose,
	);
	if bounds_status != .Ok
	{
		return static_handle_invalid(), bounds_status;
	}
	wake_status := physics.simulation_awaken_inactive_bodies_in_bounds_filtered(
		simulation, bounds, filter, user_context,
	);
	if wake_status != .Ok
	{
		return static_handle_invalid(), wake_status;
	}
	world_invalidate_views(world);
	stored := description;
	return physics.simulation_add_static_without_awakening_bodies(simulation, &stored);
}

// static_remove_filtered removes a static and applies a caller-owned awakening
// filter to sleeping bodies overlapping the removed bounds
static_remove_filtered :: proc (
	world: ^World,
	handle: Static_Handle,
	filter: Static_Awakening_Filter_Proc,
	user_context: rawptr = nil,
) -> Status
{
	if filter == nil
	{
		return static_remove(world, handle, .Overlaps);
	}
	simulation := world_simulation(world);
	if simulation == nil || simulation.state != .Ready
	{
		return .Disposed;
	}
	if simulation.triggers != nil && physics.trigger_static_member(simulation.triggers, handle) == .Present
	{
		return static_remove(world, handle, .None);
	}
	state, state_status := physics.statics_get_description(&simulation.statics, handle);
	if state_status != .Ok
	{
		return state_status;
	}
	bounds, bounds_status := physics.shape_registry_compute_world_bounds(
		physics.simulation_shape_registry(simulation), state.shape, state.pose,
	);
	if bounds_status != .Ok
	{
		return bounds_status;
	}
	wake_status := physics.mixed_colliders_awaken_static(
		simulation, handle, &state, &state, bounds, filter, user_context,
	);
	if wake_status != .Ok
	{
		return wake_status;
	}
	world_invalidate_views(world);
	return physics.simulation_remove_static_without_awakening_bodies(simulation, handle);
}

// static_apply_filtered replaces a static and filters awakening against the
// union of its old and new bounds
static_apply_filtered :: proc (
	world: ^World,
	handle: Static_Handle,
	description: Static_Description,
	filter: Static_Awakening_Filter_Proc,
	user_context: rawptr = nil,
) -> Status
{
	if filter == nil
	{
		return static_apply(world, handle, description, .Overlaps);
	}
	simulation := world_simulation(world);
	if simulation == nil || simulation.state != .Ready
	{
		return .Disposed;
	}
	if simulation.triggers != nil && physics.trigger_static_member(simulation.triggers, handle) == .Present
	{
		return static_apply(world, handle, description, .None);
	}
	old, old_status := physics.statics_get_description(&simulation.statics, handle);
	if old_status != .Ok
	{
		return old_status;
	}
	registry := physics.simulation_shape_registry(simulation);
	old_bounds, old_bounds_status := physics.shape_registry_compute_world_bounds(
		registry, old.shape, old.pose,
	);
	if old_bounds_status != .Ok
	{
		return old_bounds_status;
	}
	new_bounds, new_bounds_status := physics.shape_registry_compute_world_bounds(
		registry, description.shape, description.pose,
	);
	if new_bounds_status != .Ok
	{
		return new_bounds_status;
	}
	next_description: Static_Description = description;
	wake_status := physics.mixed_colliders_awaken_static(
		simulation, handle, &old, &next_description, util.bounding_box_merge(old_bounds, new_bounds),
		filter, user_context,
	);
	if wake_status != .Ok
	{
		return wake_status;
	}
	world_invalidate_views(world);
	stored := description;
	return physics.simulation_apply_static_description_without_awakening_bodies(
		simulation, handle, &stored,
	);
}
