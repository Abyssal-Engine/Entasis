package entasis_physics

import util "entasis:entasis_utilities"

// preflight retains each translated predicted envelope and newly represented shape
// bounds in reserved build scratch, callbacks run only before world mutation
world_precision_stage_origin_bounds :: proc "contextless" (
	broad_phase: ^Broad_Phase,
	tree: ^Tree,
	leaves: util.Buffer(Collidable_Reference),
	delta: util.Vector3,
) -> Physics_Status
{
	for leaf_index in 0 ..< tree.leaf_count
	{
		leaf: Tree_Leaf = tree.leaves.memory[leaf_index];
		child: ^Tree_Node_Child = tree_child(
			&tree.nodes.memory[tree_leaf_node_index(leaf)],
			tree_leaf_child_index(leaf),
		);
		translated: Tree_Bounds;
		status: Physics_Status;
		translated, status = tree_translate_bounds({min=child.min, max=child.max}, delta);
		if status != .Ok
		{
			return status;
		}
		geometry: Tree_Bounds;
		geometry, status = world_precision_reference_shifted_bounds(broad_phase, leaves.memory[leaf_index], delta);
		if status != .Ok
		{
			return status;
		}
		if !(abs(geometry.min.x) <= max(Tree_Scalar) && abs(geometry.min.y) <= max(Tree_Scalar) && abs(geometry.min.z) <= max(Tree_Scalar) &&
			abs(geometry.max.x) <= max(Tree_Scalar) && abs(geometry.max.y) <= max(Tree_Scalar) && abs(geometry.max.z) <= max(Tree_Scalar)) ||
			geometry.min.x > geometry.max.x || geometry.min.y > geometry.max.y || geometry.min.z > geometry.max.z
		{
			return .Invalid_Argument;
		}
		tree.build_references.memory[leaf_index] = {
			bounds={min=tree_position_min(translated.min, geometry.min), max=tree_position_max(translated.max, geometry.max)},
			leaf_index=i32(leaf_index),
		};
	}
	return .Ok;
}

world_precision_validate_broad_phase :: proc "contextless" (
	broad_phase: ^Broad_Phase,
	delta: util.Vector3,
) -> Physics_Status
{
	status: Physics_Status = world_precision_stage_origin_bounds(
		broad_phase, &broad_phase.active_tree, broad_phase.active_leaves, delta,
	);
	if status != .Ok
	{
		return status;
	}
	return world_precision_stage_origin_bounds(
		broad_phase, &broad_phase.static_tree, broad_phase.static_leaves, delta,
	);
}

// commit uses the admitted leaf mapping and performs one ancestor refit
world_precision_commit_origin_bounds :: proc "contextless" (tree: ^Tree)
{
	for leaf_index in 0 ..< tree.leaf_count
	{
		leaf: Tree_Leaf = tree.leaves.memory[leaf_index];
		child: ^Tree_Node_Child = tree_child(
			&tree.nodes.memory[tree_leaf_node_index(leaf)],
			tree_leaf_child_index(leaf),
		);
		bounds: Tree_Bounds = tree.build_references.memory[leaf_index].bounds;
		child.min = bounds.min;
		child.max = bounds.max;
	}
	if tree.leaf_count > 1
	{
		_, _ = tree_refit_subtree(tree, 0);
	}
}

world_precision_reference_shifted_bounds :: proc "contextless" (
	broad_phase: ^Broad_Phase, reference: Collidable_Reference, delta: util.Vector3,
) -> (Tree_Bounds, Physics_Status)
{
	if collidable_reference_mobility(reference) == .Static
	{
		index: int;
		status: Physics_Status;
		index, status = statics_resolve(broad_phase.statics, {collidable_reference_raw_handle(reference)});
		if status != .Ok
		{
			return {}, status;
		}
		static: ^Static = &broad_phase.statics.statics.memory[index];
		position: Tree_Position;
		when util.PRECISION_MODE == .Double_Coordinates
		{
			position = world_precision_translate_global_position(broad_phase.statics.global_positions.memory[index], delta);
		}
		else
		{
			position, _ = world_precision_translate_vector(static.pose.position, delta);
		}
		return broad_phase_shape_bounds(broad_phase.shapes, static.shape, position, static.pose.orientation);
	}
	location: Body_Memory_Location;
	status: Physics_Status;
	location, status = bodies_resolve(broad_phase.bodies, {collidable_reference_raw_handle(reference)});
	if status != .Ok
	{
		return {}, status;
	}
	set: ^Body_Set = &broad_phase.bodies.sets.memory[location.set_index];
	position: Tree_Position;
	when util.PRECISION_MODE == .Double_Coordinates
	{
		position = world_precision_translate_global_position(set.global_positions.memory[location.index], delta);
	}
	else
	{
		position, _ = world_precision_translate_vector(set.dynamics_state.memory[location.index].motion.pose.position, delta);
	}
	return broad_phase_shape_bounds(broad_phase.shapes, set.collidables.memory[location.index].shape, position, set.dynamics_state.memory[location.index].motion.pose.orientation);
}
