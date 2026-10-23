package entasis_physics

import util "entasis:entasis_utilities"

// a world tree leaf owns one conservative envelope in its selected width
Broad_Phase_Leaf_Bounds :: Tree_Bounds;

broad_phase_body_position :: #force_inline proc "contextless" (set: ^Body_Set, index: int) -> Tree_Position
{
	when util.PRECISION_MODE == .Double_Coordinates
	{
		return set.global_positions.memory[index];
	}
	else
	{
		return set.dynamics_state.memory[index].motion.pose.position;
	}
}

broad_phase_static_position :: #force_inline proc "contextless" (statics: ^Statics, index: int) -> Tree_Position
{
	when util.PRECISION_MODE == .Double_Coordinates
	{
		return statics.global_positions.memory[index];
	}
	else
	{
		return statics.statics.memory[index].pose.position;
	}
}

broad_phase_bounds_from_local :: #force_inline proc "contextless" (
	position: Tree_Position, local: util.Bounding_Box,
) -> (Tree_Bounds, Physics_Status)
{
	when util.PRECISION_MODE == .Double_Coordinates
	{
		bounds: Global_Bounds;
		status: Physics_Status;
		bounds, status = world_precision_bounds_from_local(position, local);
		if status != .Ok
		{
			return {}, status;
		}
		limit: f64 = f64(max(f32));
		if !(bounds.min.x >= -limit && bounds.min.y >= -limit && bounds.min.z >= -limit &&
			bounds.max.x <= limit && bounds.max.y <= limit && bounds.max.z <= limit)
		{
			return {}, .Invalid_Argument;
		}
		return bounds, .Ok;
	}
	else
	{
		return {min=util.vector3_add(position, local.min), max=util.vector3_add(position, local.max)}, .Ok;
	}
}

broad_phase_shape_bounds :: proc "contextless" (
	registry: ^Shape_Registry, shape: Typed_Index,
	position: Tree_Position, orientation: util.Quaternion,
	local_bounds: ^Shape_Bounds = nil,
) -> (Tree_Bounds, Physics_Status)
{
	when util.PRECISION_MODE == .Double_Coordinates
	{
		local: Shape_Bounds;
		status: Physics_Status;
		local, status = shape_registry_compute_bounds(registry, shape, orientation);
		if status != .Ok
		{
			return {}, status;
		}
		if local_bounds != nil
		{
			local_bounds^ = local;
		}
		return broad_phase_bounds_from_local(position, {min=local.min, max=local.max});
	}
	else
	{
		return shape_registry_compute_world_bounds(registry, shape, {position=position, orientation=orientation});
	}
}

broad_phase_update_leaf_bounds :: proc "contextless" (
	broad_phase: ^Broad_Phase, tree_kind: Broad_Phase_Tree,
	leaf_index: int, bounds: Tree_Bounds,
) -> Physics_Status
{
	tree: ^Tree = &broad_phase.active_tree;
	if tree_kind == .Static
	{
		tree = &broad_phase.static_tree;
	}
	return tree_update_bounds(tree, leaf_index, bounds);
}

broad_phase_static_handle_position :: proc "contextless" (
	broad_phase: ^Broad_Phase, handle: Static_Handle,
) -> (Tree_Position, Physics_Status)
{
	index: int;
	status: Physics_Status;
	index, status = statics_resolve(broad_phase.statics, handle);
	if status != .Ok
	{
		return {}, status;
	}
	return broad_phase_static_position(broad_phase.statics, index), .Ok;
}
