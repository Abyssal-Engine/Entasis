package entasis_physics

import util "entasis:entasis_utilities"
import "base:runtime"
import "core:math"
import "core:simd"
import "core:sync"

broad_phase_set_owner_index_trusted :: proc "contextless" (
	broad_phase: ^Broad_Phase, reference: Collidable_Reference, index: i32,
)
{
	mobility := collidable_reference_mobility(reference);
	if mobility == .Static
	{
		static_handle := collidable_reference_raw_handle(reference);
		static_index := broad_phase.statics.handle_to_index.memory[static_handle];
		broad_phase.statics.statics.memory[static_index].broad_phase_index = index;
		return;
	}
	location := broad_phase.bodies.handle_to_location.memory[collidable_reference_raw_handle(reference)];
	broad_phase.bodies.sets.memory[location.set_index].collidables.memory[location.index].broad_phase_index = index;
	return;
}
broad_phase_set_owner_index :: proc "contextless" (
	broad_phase: ^Broad_Phase, reference: Collidable_Reference, index: i32,
) -> Physics_Status
{
	mobility := collidable_reference_mobility(reference);
	if mobility == .Static
	{
		_, status := statics_resolve(broad_phase.statics, {collidable_reference_raw_handle(reference)});
		if status != .Ok
		{
			return status;
		}
	}
	else
	{
		_, status := bodies_resolve(broad_phase.bodies, {collidable_reference_raw_handle(reference)});
		if status != .Ok
		{
			return status;
		}
	}
	broad_phase_set_owner_index_trusted(broad_phase, reference, index);
	return .Ok;
}
broad_phase_add_leaf :: proc (
	broad_phase: ^Broad_Phase, tree_kind: Broad_Phase_Tree,
	reference: Collidable_Reference, bounds: util.Bounding_Box,
) -> (int, Physics_Status)
{
	if broad_phase == nil || broad_phase.state != .Ready
	{
		return -1, .Disposed;
	}
	tree := &broad_phase.active_tree;
	leaves := &broad_phase.active_leaves;
	if tree_kind == .Static
	{
		tree = &broad_phase.static_tree;
		leaves = &broad_phase.static_leaves;
	}
	if tree.leaf_count >= int(leaves.length)
	{
		current_capacity := int(leaves.length);
		if current_capacity > max(int) / 2
		{
			return -1, .Capacity_Missing;
		}
		target_capacity := max(current_capacity * 2, tree.leaf_count + 1);
		active_capacity := max(int(broad_phase.active_leaves.length), 1);
		static_capacity := max(int(broad_phase.static_leaves.length), 1);
		if tree_kind == .Active
		{
			active_capacity = target_capacity;
		}
		if tree_kind == .Static
		{
			static_capacity = target_capacity;
		}
		capacity_status := broad_phase_ensure_capacity(
			broad_phase, active_capacity, static_capacity, 1,
		);
		if capacity_status != .Ok
		{
			return -1, capacity_status;
		}
		tree = &broad_phase.active_tree;
		leaves = &broad_phase.active_leaves;
		if tree_kind == .Static
		{
			tree = &broad_phase.static_tree;
			leaves = &broad_phase.static_leaves;
		}
	}
	leaf_index, status := tree_add(tree, bounds);
	if status != .Ok
	{
		return -1, status;
	}
	leaves.memory[leaf_index] = reference;
	owner_status := broad_phase_set_owner_index(broad_phase, reference, i32(leaf_index));
	if owner_status != .Ok
	{
		_, _ = tree_remove_at(tree, leaf_index);
		leaves.memory[leaf_index] = {};
		return -1, owner_status;
	}
	return leaf_index, .Ok;
}
broad_phase_add_body :: proc (broad_phase: ^Broad_Phase, handle: Body_Handle) -> Physics_Status
{
	if broad_phase == nil || broad_phase.state != .Ready || broad_phase.shapes == nil
	{
		return .Disposed;
	}
	location, status := bodies_resolve(broad_phase.bodies, handle);
	if status != .Ok
	{
		return status;
	}
	set := &broad_phase.bodies.sets.memory[location.set_index];
	collidable := &set.collidables.memory[location.index];
	if typed_index_state(collidable.shape) != .Present || collidable.broad_phase_index >= 0
	{
		return .Invalid_Argument;
	}
	bounds, bounds_status := shape_registry_compute_world_bounds(
		broad_phase.shapes, collidable.shape, set.dynamics_state.memory[location.index].motion.pose,
	);
	if bounds_status != .Ok
	{
		return bounds_status;
	}
	mobility := body_inertia_mobility(set.dynamics_state.memory[location.index].inertia.local);
	reference, reference_status := collidable_reference_body(mobility, handle);
	if reference_status != .Ok
	{
		return reference_status;
	}
	tree_kind := Broad_Phase_Tree.Static;
	if location.set_index == BODIES_ACTIVE_SET_INDEX
	{
		tree_kind = .Active;
	}
	_, add_status := broad_phase_add_leaf(broad_phase, tree_kind, reference, bounds);
	return add_status;
}
broad_phase_add_static :: proc (broad_phase: ^Broad_Phase, handle: Static_Handle) -> Physics_Status
{
	if broad_phase == nil || broad_phase.state != .Ready || broad_phase.shapes == nil
	{
		return .Disposed;
	}
	index, status := statics_resolve(broad_phase.statics, handle);
	if status != .Ok
	{
		return status;
	}
	static := &broad_phase.statics.statics.memory[index];
	if static.broad_phase_index >= 0
	{
		return .Invalid_Argument;
	}
	bounds, bounds_status := shape_registry_compute_world_bounds(broad_phase.shapes, static.shape, static.pose);
	if bounds_status != .Ok
	{
		return bounds_status;
	}
	reference, reference_status := collidable_reference_static(handle);
	if reference_status != .Ok
	{
		return reference_status;
	}
	_, add_status := broad_phase_add_leaf(broad_phase, .Static, reference, bounds);
	return add_status;
}
broad_phase_remove_leaf :: proc "contextless" (
	broad_phase: ^Broad_Phase, tree_kind: Broad_Phase_Tree, leaf_index: int,
) -> Physics_Status
{
	tree := &broad_phase.active_tree;
	leaves := &broad_phase.active_leaves;
	if tree_kind == .Static
	{
		tree = &broad_phase.static_tree;
		leaves = &broad_phase.static_leaves;
	}
	if leaf_index < 0 || leaf_index >= tree.leaf_count
	{
		return .Not_Found;
	}
	removed_reference := leaves.memory[leaf_index];
	result, status := tree_remove_at(tree, leaf_index);
	if status != .Ok
	{
		return status;
	}
	if result.moved_leaf_original_index >= 0
	{
		moved_reference := leaves.memory[result.moved_leaf_original_index];
		leaves.memory[result.moved_leaf_new_index] = moved_reference;
		owner_status := broad_phase_set_owner_index(broad_phase, moved_reference, result.moved_leaf_new_index);
		if owner_status != .Ok
		{
			return owner_status;
		}
	}
	leaves.memory[tree.leaf_count] = {};
	return broad_phase_set_owner_index(broad_phase, removed_reference, -1);
}
broad_phase_remove_leaf_trusted :: proc "contextless" (
	broad_phase: ^Broad_Phase, tree_kind: Broad_Phase_Tree, leaf_index: int,
)
{
	tree := &broad_phase.active_tree;
	leaves := &broad_phase.active_leaves;
	if tree_kind == .Static
	{
		tree = &broad_phase.static_tree;
		leaves = &broad_phase.static_leaves;
	}
	removed_reference := leaves.memory[leaf_index];
	result := tree_remove_at_trusted(tree, leaf_index);
	if result.moved_leaf_original_index >= 0
	{
		moved_reference := leaves.memory[result.moved_leaf_original_index];
		leaves.memory[result.moved_leaf_new_index] = moved_reference;
		broad_phase_set_owner_index_trusted(broad_phase, moved_reference, result.moved_leaf_new_index);
	}
	leaves.memory[tree.leaf_count] = {};
	broad_phase_set_owner_index_trusted(broad_phase, removed_reference, -1);
}
broad_phase_find_body_leaf :: proc "contextless" (
	broad_phase: ^Broad_Phase, handle: Body_Handle, broad_phase_index: int,
) -> (Broad_Phase_Tree, Physics_Status)
{
	if broad_phase_index >= 0 && broad_phase_index < broad_phase.active_tree.leaf_count
	{
		reference := broad_phase.active_leaves.memory[broad_phase_index];
		if collidable_reference_mobility(reference) != .Static &&
			collidable_reference_raw_handle(reference) == handle.value
		{
			return .Active, .Ok;
		}
	}
	if broad_phase_index >= 0 && broad_phase_index < broad_phase.static_tree.leaf_count
	{
		reference := broad_phase.static_leaves.memory[broad_phase_index];
		if collidable_reference_mobility(reference) != .Static &&
			collidable_reference_raw_handle(reference) == handle.value
		{
			return .Static, .Ok;
		}
	}
	return .Active, .Not_Found;
}
broad_phase_remove_body :: proc "contextless" (broad_phase: ^Broad_Phase, handle: Body_Handle) -> Physics_Status
{
	location, status := bodies_resolve(broad_phase.bodies, handle);
	if status != .Ok
	{
		return status;
	}
	leaf_index := int(broad_phase.bodies.sets.memory[location.set_index].collidables.memory[location.index].broad_phase_index);
	tree_kind, find_status := broad_phase_find_body_leaf(broad_phase, handle, leaf_index);
	if find_status != .Ok
	{
		return find_status;
	}
	return broad_phase_remove_leaf(broad_phase, tree_kind, leaf_index);
}
broad_phase_remove_static :: proc "contextless" (broad_phase: ^Broad_Phase, handle: Static_Handle) -> Physics_Status
{
	index, status := statics_resolve(broad_phase.statics, handle);
	if status != .Ok
	{
		return status;
	}
	leaf_index := int(broad_phase.statics.statics.memory[index].broad_phase_index);
	if leaf_index < 0 || leaf_index >= broad_phase.static_tree.leaf_count
	{
		return .Not_Found;
	}
	reference := broad_phase.static_leaves.memory[leaf_index];
	if collidable_reference_mobility(reference) != .Static || collidable_reference_raw_handle(reference) != handle.value
	{
		return .Not_Found;
	}
	return broad_phase_remove_leaf(broad_phase, .Static, leaf_index);
}
broad_phase_refresh_body_membership :: proc (
	broad_phase: ^Broad_Phase, handle: Body_Handle,
) -> Physics_Status
{
	location, status := bodies_resolve(broad_phase.bodies, handle);
	if status != .Ok
	{
		return status;
	}
	leaf_index := int(broad_phase.bodies.sets.memory[location.set_index].collidables.memory[location.index].broad_phase_index);
	if leaf_index >= 0
	{
		tree_kind, find_status := broad_phase_find_body_leaf(broad_phase, handle, leaf_index);
		if find_status != .Ok
		{
			return find_status;
		}
		remove_status := broad_phase_remove_leaf(broad_phase, tree_kind, leaf_index);
		if remove_status != .Ok
		{
			return remove_status;
		}
	}
	location, status = bodies_resolve(broad_phase.bodies, handle);
	if status != .Ok
	{
		return status;
	}
	set := &broad_phase.bodies.sets.memory[location.set_index];
	if typed_index_state(set.collidables.memory[location.index].shape) != .Present
	{
		return .Ok;
	}
	return broad_phase_add_body(broad_phase, handle);
}
broad_phase_prepare_body_description_change :: proc (
	broad_phase: ^Broad_Phase, handle: Body_Handle, description: ^Body_Description,
) -> (Broad_Phase_Body_Description_Change, Physics_Status)
{
	if broad_phase == nil || broad_phase.state != .Ready ||
		broad_phase.shapes == nil || description == nil
	{
		return {}, .Disposed;
	}
	validation := body_description_validate(description);
	if validation != .Ok
	{
		return {}, validation;
	}
	location, status := bodies_resolve(broad_phase.bodies, handle);
	if status != .Ok
	{
		return {}, status;
	}
	set := &broad_phase.bodies.sets.memory[location.set_index];
	collidable := &set.collidables.memory[location.index];
	old_shape_state := typed_index_state(collidable.shape);
	new_shape_state := typed_index_state(description.collidable.shape);
	source_tree := Broad_Phase_Tree.Active;
	source_leaf_index := int(collidable.broad_phase_index);
	if old_shape_state == .Present
	{
		if source_leaf_index < 0
		{
			return {}, .Invalid_Description;
		}
		source_tree, status = broad_phase_find_body_leaf(
			broad_phase, handle, source_leaf_index,
		);
		if status != .Ok
		{
			return {}, status;
		}
	}
	else if source_leaf_index >= 0
	{
		return {}, .Invalid_Description;
	}
	target_tree := Broad_Phase_Tree.Static;
	if location.set_index == BODIES_ACTIVE_SET_INDEX
	{
		target_tree = .Active;
	}
	change := Broad_Phase_Body_Description_Change{
		handle=handle,
		source_tree=source_tree,
		target_tree=target_tree,
		source_leaf_index=i32(source_leaf_index),
		state=.Unchanged,
	};
	if new_shape_state == .Present
	{
		change.bounds, status = shape_registry_compute_world_bounds(
			broad_phase.shapes, description.collidable.shape, description.pose,
		);
		if status != .Ok
		{
			return {}, status;
		}
		if change.bounds.min.x > change.bounds.max.x ||
			change.bounds.min.y > change.bounds.max.y ||
			change.bounds.min.z > change.bounds.max.z
		{
			return {}, .Invalid_Description;
		}
		mobility := body_inertia_mobility(description.local_inertia);
		change.reference, status = collidable_reference_body(mobility, handle);
		if status != .Ok
		{
			return {}, status;
		}
	}
	if old_shape_state == .Present
	{
		if new_shape_state != .Present
		{
			change.state = .Remove;
			return change, .Ok;
		}
		if source_tree == target_tree
		{
			change.state = .Update;
			return change, .Ok;
		}
		change.state = .Move;
	}
	else if new_shape_state == .Present
	{
		change.state = .Add;
	}
	else
	{
		return change, .Ok;
	}
	target_tree_owner := &broad_phase.active_tree;
	target_leaves := &broad_phase.active_leaves;
	if target_tree == .Static
	{
		target_tree_owner = &broad_phase.static_tree;
		target_leaves = &broad_phase.static_leaves;
	}
	required_capacity := target_tree_owner.leaf_count + 1;
	target_capacity := max(int(target_leaves.length), required_capacity);
	if required_capacity > int(target_leaves.length) ||
		required_capacity > int(target_tree_owner.leaves.length)
	{
		if int(target_leaves.length) > max(int) / 2
		{
			return {}, .Capacity_Missing;
		}
		target_capacity = max(int(target_leaves.length) * 2, required_capacity);
	}
	active_capacity := max(int(broad_phase.active_leaves.length), 1);
	static_capacity := max(int(broad_phase.static_leaves.length), 1);
	if target_tree == .Active
	{
		active_capacity = target_capacity;
	}
	if target_tree == .Static
	{
		static_capacity = target_capacity;
	}
	status = broad_phase_ensure_capacity(
		broad_phase, active_capacity, static_capacity, 1,
	);
	if status != .Ok
	{
		return {}, status;
	}
	return change, .Ok;
}
broad_phase_commit_body_description_change :: proc "contextless" (
	broad_phase: ^Broad_Phase, change: ^Broad_Phase_Body_Description_Change,
)
{
	switch change.state
	{
		case .Unchanged:
			return;
		case .Remove:
			broad_phase_remove_leaf_trusted(
				broad_phase, change.source_tree, int(change.source_leaf_index),
			);
		case .Update:
			tree := &broad_phase.active_tree;
			leaves := &broad_phase.active_leaves;
			if change.target_tree == .Static
			{
				tree = &broad_phase.static_tree;
				leaves = &broad_phase.static_leaves;
			}
			_ = tree_update_bounds(tree, int(change.source_leaf_index), change.bounds);
			leaves.memory[change.source_leaf_index] = change.reference;
			broad_phase_set_owner_index_trusted(
				broad_phase, change.reference, change.source_leaf_index,
			);
		case .Add:
			tree := &broad_phase.active_tree;
			leaves := &broad_phase.active_leaves;
			if change.target_tree == .Static
			{
				tree = &broad_phase.static_tree;
				leaves = &broad_phase.static_leaves;
			}
			leaf_index := tree_add_trusted(tree, change.bounds);
			leaves.memory[leaf_index] = change.reference;
			broad_phase_set_owner_index_trusted(
				broad_phase, change.reference, i32(leaf_index),
			);
		case .Move:
			broad_phase_remove_leaf_trusted(
				broad_phase, change.source_tree, int(change.source_leaf_index),
			);
			tree := &broad_phase.active_tree;
			leaves := &broad_phase.active_leaves;
			if change.target_tree == .Static
			{
				tree = &broad_phase.static_tree;
				leaves = &broad_phase.static_leaves;
			}
			leaf_index := tree_add_trusted(tree, change.bounds);
			leaves.memory[leaf_index] = change.reference;
			broad_phase_set_owner_index_trusted(
				broad_phase, change.reference, i32(leaf_index),
			);
	}
}
broad_phase_prepare_body_migration :: proc "contextless" (
	broad_phase: ^Broad_Phase, handle: Body_Handle, target_tree: Broad_Phase_Tree,
) -> (Broad_Phase_Body_Migration, Physics_Status)
{
	location, status := bodies_resolve(broad_phase.bodies, handle);
	if status != .Ok
	{
		return {}, status;
	}
	set := &broad_phase.bodies.sets.memory[location.set_index];
	collidable := &set.collidables.memory[location.index];
	if typed_index_state(collidable.shape) != .Present
	{
		if collidable.broad_phase_index >= 0
		{
			return {}, .Invalid_Argument;
		}
		return {handle=handle, target_tree=target_tree, state=.Missing}, .Ok;
	}
	leaf_index := int(collidable.broad_phase_index);
	source_tree, find_status := broad_phase_find_body_leaf(broad_phase, handle, leaf_index);
	if find_status != .Ok || source_tree == target_tree
	{
		return {}, .Invalid_Argument;
	}
	tree := &broad_phase.active_tree;
	leaves := &broad_phase.active_leaves;
	if source_tree == .Static
	{
		tree = &broad_phase.static_tree;
		leaves = &broad_phase.static_leaves;
	}
	bounds, bounds_status := tree_get_leaf_bounds(tree, leaf_index);
	if bounds_status != .Ok
	{
		return {}, bounds_status;
	}
	return {
		bounds=bounds,
		reference=leaves.memory[leaf_index],
		handle=handle,
		source_tree=source_tree,
		target_tree=target_tree,
		state=.Present,
	}, .Ok;
}
broad_phase_validate_body_migration_capacity :: proc "contextless" (
	broad_phase: ^Broad_Phase, target_tree: Broad_Phase_Tree, migration_count: int,
) -> Physics_Status
{
	if broad_phase == nil || broad_phase.state != .Ready || migration_count < 0
	{
		return .Invalid_Argument;
	}
	tree := &broad_phase.active_tree;
	leaves := &broad_phase.active_leaves;
	if target_tree == .Static
	{
		tree = &broad_phase.static_tree;
		leaves = &broad_phase.static_leaves;
	}
	target_leaf_count := tree.leaf_count + migration_count;
	target_node_count := max(target_leaf_count - 1, 1);
	if target_leaf_count > int(leaves.length) || target_leaf_count > int(tree.leaves.length) ||
		target_node_count > int(tree.nodes.length) || target_node_count > int(tree.metanodes.length)
	{
		return .Capacity_Missing;
	}
	return .Ok;
}
broad_phase_commit_body_migration :: proc "contextless" (
	broad_phase: ^Broad_Phase, migration: ^Broad_Phase_Body_Migration,
)
{
	if migration.state == .Missing
	{
		return;
	}
	location := broad_phase.bodies.handle_to_location.memory[migration.handle.value];
	collidable := &broad_phase.bodies.sets.memory[location.set_index].collidables.memory[location.index];
	broad_phase_remove_leaf_trusted(broad_phase, migration.source_tree, int(collidable.broad_phase_index));
	tree := &broad_phase.active_tree;
	leaves := &broad_phase.active_leaves;
	if migration.target_tree == .Static
	{
		tree = &broad_phase.static_tree;
		leaves = &broad_phase.static_leaves;
	}
	leaf_index := tree_add_trusted(tree, migration.bounds);
	leaves.memory[leaf_index] = migration.reference;
	broad_phase_set_owner_index_trusted(broad_phase, migration.reference, i32(leaf_index));
}
broad_phase_reference_bounds :: proc "contextless" (
	broad_phase: ^Broad_Phase, reference: Collidable_Reference,
) -> (util.Bounding_Box, Physics_Status)
{
	if collidable_reference_mobility(reference) == .Static
	{
		index, status := statics_resolve(broad_phase.statics, {collidable_reference_raw_handle(reference)});
		if status != .Ok
		{
			return {}, status;
		}
		static := &broad_phase.statics.statics.memory[index];
		return shape_registry_compute_world_bounds(broad_phase.shapes, static.shape, static.pose);
	}
	location, status := bodies_resolve(broad_phase.bodies, {collidable_reference_raw_handle(reference)});
	if status != .Ok
	{
		return {}, status;
	}
	set := &broad_phase.bodies.sets.memory[location.set_index];
	return shape_registry_compute_world_bounds(
		broad_phase.shapes, set.collidables.memory[location.index].shape,
		set.dynamics_state.memory[location.index].motion.pose,
	);
}
broad_phase_update_bounds :: proc "contextless" (broad_phase: ^Broad_Phase) -> Physics_Status
{
	if broad_phase == nil || broad_phase.state != .Ready || broad_phase.shapes == nil
	{
		return .Disposed;
	}
	for leaf_index in 0 ..< broad_phase.active_tree.leaf_count
	{
		bounds, status := broad_phase_reference_bounds(broad_phase, broad_phase.active_leaves.memory[leaf_index]);
		if status != .Ok
		{
			return status;
		}
		status = tree_update_bounds(&broad_phase.active_tree, leaf_index, bounds);
		if status != .Ok
		{
			return status;
		}
	}
	for leaf_index in 0 ..< broad_phase.static_tree.leaf_count
	{
		bounds, status := broad_phase_reference_bounds(broad_phase, broad_phase.static_leaves.memory[leaf_index]);
		if status != .Ok
		{
			return status;
		}
		status = tree_update_bounds(&broad_phase.static_tree, leaf_index, bounds);
		if status != .Ok
		{
			return status;
		}
	}
	return .Ok;
}
broad_phase_angular_bounds_expansion :: proc "contextless" (
	angular_speed, dt, maximum_radius, maximum_angular_expansion: f32,
) -> f32
{
	angle := min(angular_speed * dt, f32(math.PI / 3));
	angle_squared := angle * angle;
	angle_fourth := angle_squared * angle_squared;
	angle_sixth := angle_fourth * angle_squared;
	cosine_minus_one :=
		angle_squared * (-1.0 / 2.0) + angle_fourth * (1.0 / 24.0) - angle_sixth * (1.0 / 720.0);
	return min(maximum_angular_expansion, math.sqrt(max(0, -2 * maximum_radius * maximum_radius * cosine_minus_one)));
}
broad_phase_motion_bounds_expansion :: proc "contextless" (
	linear_velocity, angular_velocity: util.Vector3, dt, maximum_radius,
	maximum_angular_expansion, maximum_allowed_expansion: f32,
) -> (min_expansion, max_expansion: util.Vector3)
{
	linear_displacement := util.vector3_scale(linear_velocity, dt);
	angular_expansion := broad_phase_angular_bounds_expansion(
		util.vector3_length(angular_velocity), dt, maximum_radius, maximum_angular_expansion,
	);
	angular := util.Vector3{angular_expansion, angular_expansion, angular_expansion};
	min_expansion = util.vector3_subtract(util.vector3_min({}, linear_displacement), angular);
	max_expansion = util.vector3_add(util.vector3_max({}, linear_displacement), angular);
	limit := util.Vector3{maximum_allowed_expansion, maximum_allowed_expansion, maximum_allowed_expansion};
	min_expansion = util.vector3_max(util.vector3_negate(limit), min_expansion);
	max_expansion = util.vector3_min(limit, max_expansion);
	return;
}
broad_phase_angular_bounds_expansion_wide :: #force_inline proc "contextless" (
	angular_speed: util.F32x8, dt: f32,
	maximum_radius, maximum_angular_expansion: util.F32x8,
) -> util.F32x8
{
	angle := simd.min(simd.mul(angular_speed, util.F32x8(dt)), util.F32x8(f32(math.PI / 3)));
	angle_squared := simd.mul(angle, angle);
	angle_fourth := simd.mul(angle_squared, angle_squared);
	angle_sixth := simd.mul(angle_fourth, angle_squared);
	cosine_minus_one := simd.add(
		simd.mul(angle_squared, util.F32x8(-1.0 / 2.0)),
		simd.add(
		simd.mul(angle_fourth, util.F32x8(1.0 / 24.0)),
		simd.mul(angle_sixth, util.F32x8(-1.0 / 720.0)),
	),
	);
	expansion_squared := simd.max(
		util.F32x8(0),
		simd.mul(
		util.F32x8(-2),
		simd.mul(simd.mul(maximum_radius, maximum_radius), cosine_minus_one),
	),
	);
	return simd.min(maximum_angular_expansion, simd.sqrt(expansion_squared));
}
broad_phase_motion_bounds_expansion_wide :: #force_inline proc "contextless" (
	linear_velocity: util.Vector3_Wide, dt: f32, angular_expansion,
	maximum_allowed_expansion: util.F32x8,
) -> (min_expansion, max_expansion: util.Vector3_Wide)
{
	linear_displacement := util.vector3_wide_scale(linear_velocity, util.F32x8(dt));
	angular := util.Vector3_Wide{angular_expansion, angular_expansion, angular_expansion};
	min_expansion = util.vector3_wide_subtract(
		util.vector3_wide_min({}, linear_displacement), angular,
	);
	max_expansion = util.vector3_wide_add(
		util.vector3_wide_max({}, linear_displacement), angular,
	);
	limit := util.Vector3_Wide{
		maximum_allowed_expansion, maximum_allowed_expansion, maximum_allowed_expansion,
	};
	min_expansion = util.vector3_wide_max(util.vector3_wide_negate(limit), min_expansion);
	max_expansion = util.vector3_wide_min(limit, max_expansion);
	return;
}
broad_phase_predict_bounds_body_state :: #force_inline proc "contextless" (
	broad_phase: ^Broad_Phase, body_index: int, dt: f32,
	position: util.Vector3, orientation: util.Quaternion, predicted_velocity: Body_Velocity,
) -> Physics_Status
{
	active := &broad_phase.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
	collidable := &active.collidables.memory[body_index];
	if typed_index_state(collidable.shape) != .Present
	{
		return .Ok;
	}
	bounds, bounds_status := shape_registry_compute_bounds(
		broad_phase.shapes, collidable.shape, orientation,
	);
	if bounds_status != .Ok
	{
		return bounds_status;
	}
	linear_speed := util.vector3_length(predicted_velocity.linear);
	angular_expansion := broad_phase_angular_bounds_expansion(
		util.vector3_length(predicted_velocity.angular), dt,
		bounds.maximum_radius, bounds.maximum_angular_expansion,
	);
	margin := clamp(
		linear_speed * dt + angular_expansion,
		collidable.minimum_speculative_margin, collidable.maximum_speculative_margin,
	);
	collidable.speculative_margin = margin;
	maximum_allowed_expansion := margin;
	if collidable.continuity.mode != .Discrete
	{
		maximum_allowed_expansion = math.F32_MAX;
	}
	min_expansion, max_expansion := broad_phase_motion_bounds_expansion(
		predicted_velocity.linear, predicted_velocity.angular, dt,
		bounds.maximum_radius, bounds.maximum_angular_expansion, maximum_allowed_expansion,
	);
	bounds.min = util.vector3_add(position, util.vector3_add(bounds.min, min_expansion));
	bounds.max = util.vector3_add(position, util.vector3_add(bounds.max, max_expansion));
	leaf_index := int(collidable.broad_phase_index);
	if leaf_index < 0 || leaf_index >= broad_phase.active_tree.leaf_count
	{
		return .Invalid_Argument;
	}
	leaf := broad_phase.active_tree.leaves.memory[leaf_index];
	child := tree_child(
		&broad_phase.active_tree.nodes.memory[tree_leaf_node_index(leaf)], tree_leaf_child_index(leaf),
	);
	child.min = bounds.min;
	child.max = bounds.max;
	return .Ok;
}
broad_phase_scatter_bounds_contiguous_wide :: #force_inline proc "contextless" (
	broad_phase: ^Broad_Phase, first_body_index, body_count: int,
	minimum, maximum: util.Vector3_Wide, speculative_margin: util.F32x8,
) -> Physics_Status
{
	active := &broad_phase.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
	for lane in 0 ..< body_count
	{
		body_index := first_body_index + lane;
		collidable := &active.collidables.memory[body_index];
		if typed_index_state(collidable.shape) != .Present
		{
			continue;
		}
		collidable.speculative_margin = simd.extract(speculative_margin, lane);
		leaf_index := int(collidable.broad_phase_index);
		if leaf_index < 0 || leaf_index >= broad_phase.active_tree.leaf_count
		{
			return .Invalid_Argument;
		}
		leaf := broad_phase.active_tree.leaves.memory[leaf_index];
		child := tree_child(
			&broad_phase.active_tree.nodes.memory[tree_leaf_node_index(leaf)],
			tree_leaf_child_index(leaf),
		);
		child.min = util.vector3_wide_read_slot(minimum, lane);
		child.max = util.vector3_wide_read_slot(maximum, lane);
	}
	return .Ok;
}
broad_phase_compute_contiguous_primitive_bounds_wide :: #force_inline proc "contextless" (
	broad_phase: ^Broad_Phase, first_body_index, body_count, type_id: int,
	orientation: util.Quaternion_Wide,
) -> (
	minimum, maximum: util.Vector3_Wide,
	maximum_radius, maximum_angular_expansion: util.F32x8,
	supported: Reference_State,
)
{
	active := &broad_phase.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
	batch := &broad_phase.shapes.batches[type_id];
	switch type_id
	{
		case SPHERE_TYPE_ID:
			wide: Sphere_Wide;
			for lane in 0 ..< body_count
			{
				shape_index := int(typed_index_index(active.collidables.memory[first_body_index + lane].shape));
				shape := (^Sphere)(&batch.data.memory[shape_index * batch.stride])^;
				sphere_wide_write_slot_trusted(&wide, lane, shape);
			}
			minimum, maximum = sphere_wide_bounds(wide);
			maximum_radius = wide.radius;
			maximum_angular_expansion = util.F32x8(0);
		case CAPSULE_TYPE_ID:
			wide: Capsule_Wide;
			for lane in 0 ..< body_count
			{
				shape_index := int(typed_index_index(active.collidables.memory[first_body_index + lane].shape));
				shape := (^Capsule)(&batch.data.memory[shape_index * batch.stride])^;
				capsule_wide_write_slot_trusted(&wide, lane, shape);
			}
			minimum, maximum = capsule_wide_bounds(wide, orientation);
			maximum_radius = simd.add(wide.half_length, wide.radius);
			maximum_angular_expansion = wide.half_length;
		case BOX_TYPE_ID:
			wide: Box_Wide;
			for lane in 0 ..< body_count
			{
				shape_index := int(typed_index_index(active.collidables.memory[first_body_index + lane].shape));
				shape := (^Box)(&batch.data.memory[shape_index * batch.stride])^;
				box_wide_write_slot_trusted(&wide, lane, shape);
			}
			minimum, maximum = box_wide_bounds(wide, orientation);
			maximum_radius = simd.sqrt(simd.add(
				simd.mul(wide.half_width, wide.half_width),
				simd.add(
				simd.mul(wide.half_height, wide.half_height),
				simd.mul(wide.half_length, wide.half_length),
			),
			));
			minimum_extent := simd.min(wide.half_width, simd.min(wide.half_height, wide.half_length));
			maximum_angular_expansion = simd.sub(maximum_radius, minimum_extent);
		case TRIANGLE_TYPE_ID:
			wide: Triangle_Wide;
			for lane in 0 ..< body_count
			{
				shape_index := int(typed_index_index(active.collidables.memory[first_body_index + lane].shape));
				shape := (^Triangle)(&batch.data.memory[shape_index * batch.stride])^;
				triangle_wide_write_slot_trusted(&wide, lane, shape);
			}
			minimum, maximum = triangle_wide_bounds(wide, orientation);
			maximum_radius = simd.sqrt(simd.max(
				util.vector3_wide_length_squared(wide.a),
				simd.max(
				util.vector3_wide_length_squared(wide.b),
				util.vector3_wide_length_squared(wide.c),
			),
			));
			maximum_angular_expansion = maximum_radius;
		case CYLINDER_TYPE_ID:
			wide: Cylinder_Wide;
			for lane in 0 ..< body_count
			{
				shape_index := int(typed_index_index(active.collidables.memory[first_body_index + lane].shape));
				shape := (^Cylinder)(&batch.data.memory[shape_index * batch.stride])^;
				cylinder_wide_write_slot_trusted(&wide, lane, shape);
			}
			minimum, maximum = cylinder_wide_bounds(wide, orientation);
			maximum_radius = simd.sqrt(simd.add(
				simd.mul(wide.half_length, wide.half_length),
				simd.mul(wide.radius, wide.radius),
			));
			maximum_angular_expansion = simd.sub(
				maximum_radius, simd.min(wide.half_length, wide.radius),
			);
		case:
			return {}, {}, {}, {}, .Missing;
	}
	return minimum, maximum, maximum_radius, maximum_angular_expansion, .Present;
}
broad_phase_predict_contiguous_primitive_bundle :: #force_no_inline proc "contextless" (
	broad_phase: ^Broad_Phase, first_body_index, body_count, type_id: int, dt: f32,
	position: util.Vector3_Wide, orientation: util.Quaternion_Wide,
	velocity: Body_Velocity_Wide,
) -> Physics_Status
{
	local_minimum, local_maximum, maximum_radius, maximum_angular_expansion, supported :=
		broad_phase_compute_contiguous_primitive_bounds_wide(
		broad_phase, first_body_index, body_count, type_id, orientation,
	);
	if supported != .Present
	{
		return .Invalid_Description;
	}

	active := &broad_phase.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
	minimum_margin_values: [util.PRODUCTION_LANE_COUNT]f32;
	maximum_margin_values: [util.PRODUCTION_LANE_COUNT]f32;
	allow_expansion_values: [util.PRODUCTION_LANE_COUNT]i32;
	for lane in 0 ..< body_count
	{
		collidable := &active.collidables.memory[first_body_index + lane];
		minimum_margin_values[lane] = collidable.minimum_speculative_margin;
		maximum_margin_values[lane] = collidable.maximum_speculative_margin;
		if collidable.continuity.mode != .Discrete
		{
			allow_expansion_values[lane] = -1;
		}
	}
	linear_speed := util.vector3_wide_length(velocity.linear);
	angular_expansion := broad_phase_angular_bounds_expansion_wide(
		util.vector3_wide_length(velocity.angular), dt,
		maximum_radius, maximum_angular_expansion,
	);
	margin := simd.max(
		transmute(util.F32x8)minimum_margin_values,
		simd.min(
		transmute(util.F32x8)maximum_margin_values,
		simd.add(simd.mul(linear_speed, util.F32x8(dt)), angular_expansion),
	),
	);
	maximum_allowed_expansion := util.wide_select_f32(
		transmute(util.I32x8)allow_expansion_values,
		util.F32x8(f32(math.F32_MAX)), margin,
	);
	min_expansion, max_expansion := broad_phase_motion_bounds_expansion_wide(
		velocity.linear, dt, angular_expansion, maximum_allowed_expansion,
	);
	minimum := util.vector3_wide_add(
		position, util.vector3_wide_add(local_minimum, min_expansion),
	);
	maximum := util.vector3_wide_add(
		position, util.vector3_wide_add(local_maximum, max_expansion),
	);
	return broad_phase_scatter_bounds_contiguous_wide(
		broad_phase, first_body_index, body_count, minimum, maximum, margin,
	);
}
broad_phase_scatter_bounds_indices_wide :: #force_inline proc "contextless" (
	broad_phase: ^Broad_Phase, body_indices: ^[util.PRODUCTION_LANE_COUNT]i32, body_count: int,
	minimum, maximum: util.Vector3_Wide, speculative_margin: util.F32x8,
) -> Physics_Status
{
	active := &broad_phase.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
	for lane in 0 ..< body_count
	{
		body_index := int(body_indices[lane]);
		if body_index < 0 || body_index >= active.count
		{
			return .Invalid_Argument;
		}
		collidable := &active.collidables.memory[body_index];
		if typed_index_state(collidable.shape) != .Present
		{
			continue;
		}
		collidable.speculative_margin = simd.extract(speculative_margin, lane);
		leaf_index := int(collidable.broad_phase_index);
		if leaf_index < 0 || leaf_index >= broad_phase.active_tree.leaf_count
		{
			return .Invalid_Argument;
		}
		leaf := broad_phase.active_tree.leaves.memory[leaf_index];
		child := tree_child(
			&broad_phase.active_tree.nodes.memory[tree_leaf_node_index(leaf)],
			tree_leaf_child_index(leaf),
		);
		child.min = util.vector3_wide_read_slot(minimum, lane);
		child.max = util.vector3_wide_read_slot(maximum, lane);
	}
	return .Ok;
}
broad_phase_compute_type_bounds_wide :: #force_inline proc "contextless" (
	broad_phase: ^Broad_Phase, body_indices: ^[util.PRODUCTION_LANE_COUNT]i32,
	body_count, type_id: int, orientation: util.Quaternion_Wide,
) -> (
	minimum, maximum: util.Vector3_Wide,
	maximum_radius, maximum_angular_expansion: util.F32x8,
	status: Physics_Status,
)
{
	active := &broad_phase.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
	if type_id < 0 || type_id >= broad_phase.shapes.registered_type_count
	{
		return {}, {}, {}, {}, .Invalid_Description;
	}
	batch := &broad_phase.shapes.batches[type_id];
	switch type_id
	{
		case SPHERE_TYPE_ID:
			wide: Sphere_Wide;
			for lane in 0 ..< body_count
			{
				body_index := int(body_indices[lane]);
				shape_index := int(typed_index_index(active.collidables.memory[body_index].shape));
				shape := (^Sphere)(&batch.data.memory[shape_index * batch.stride])^;
				sphere_wide_write_slot_trusted(&wide, lane, shape);
			}
			minimum, maximum = sphere_wide_bounds(wide);
			maximum_radius = wide.radius;
			maximum_angular_expansion = util.F32x8(0);
		case CAPSULE_TYPE_ID:
			wide: Capsule_Wide;
			for lane in 0 ..< body_count
			{
				body_index := int(body_indices[lane]);
				shape_index := int(typed_index_index(active.collidables.memory[body_index].shape));
				shape := (^Capsule)(&batch.data.memory[shape_index * batch.stride])^;
				capsule_wide_write_slot_trusted(&wide, lane, shape);
			}
			minimum, maximum = capsule_wide_bounds(wide, orientation);
			maximum_radius = simd.add(wide.half_length, wide.radius);
			maximum_angular_expansion = wide.half_length;
		case BOX_TYPE_ID:
			wide: Box_Wide;
			for lane in 0 ..< body_count
			{
				body_index := int(body_indices[lane]);
				shape_index := int(typed_index_index(active.collidables.memory[body_index].shape));
				shape := (^Box)(&batch.data.memory[shape_index * batch.stride])^;
				box_wide_write_slot_trusted(&wide, lane, shape);
			}
			minimum, maximum = box_wide_bounds(wide, orientation);
			maximum_radius = simd.sqrt(simd.add(
				simd.mul(wide.half_width, wide.half_width),
				simd.add(
				simd.mul(wide.half_height, wide.half_height),
				simd.mul(wide.half_length, wide.half_length),
			),
			));
			minimum_extent := simd.min(wide.half_width, simd.min(wide.half_height, wide.half_length));
			maximum_angular_expansion = simd.sub(maximum_radius, minimum_extent);
		case TRIANGLE_TYPE_ID:
			wide: Triangle_Wide;
			for lane in 0 ..< body_count
			{
				body_index := int(body_indices[lane]);
				shape_index := int(typed_index_index(active.collidables.memory[body_index].shape));
				shape := (^Triangle)(&batch.data.memory[shape_index * batch.stride])^;
				triangle_wide_write_slot_trusted(&wide, lane, shape);
			}
			minimum, maximum = triangle_wide_bounds(wide, orientation);
			maximum_radius = simd.sqrt(simd.max(
				util.vector3_wide_length_squared(wide.a),
				simd.max(
				util.vector3_wide_length_squared(wide.b),
				util.vector3_wide_length_squared(wide.c),
			),
			));
			maximum_angular_expansion = maximum_radius;
		case CYLINDER_TYPE_ID:
			wide: Cylinder_Wide;
			for lane in 0 ..< body_count
			{
				body_index := int(body_indices[lane]);
				shape_index := int(typed_index_index(active.collidables.memory[body_index].shape));
				shape := (^Cylinder)(&batch.data.memory[shape_index * batch.stride])^;
				cylinder_wide_write_slot_trusted(&wide, lane, shape);
			}
			minimum, maximum = cylinder_wide_bounds(wide, orientation);
			maximum_radius = simd.sqrt(simd.add(
				simd.mul(wide.half_length, wide.half_length),
				simd.mul(wide.radius, wide.radius),
			));
			maximum_angular_expansion = simd.sub(
				maximum_radius, simd.min(wide.half_length, wide.radius),
			);
		case CONVEX_HULL_TYPE_ID:
			for lane in 0 ..< body_count
			{
				body_index := int(body_indices[lane]);
				shape_index := int(typed_index_index(active.collidables.memory[body_index].shape));
				hull := (^Convex_Hull)(&batch.data.memory[shape_index * batch.stride]);
				if convex_hull_validate(hull) != .Ok
				{
					return {}, {}, {}, {}, .Invalid_Description;
				}
				bounds := convex_hull_bounds(hull, util.quaternion_wide_read_slot(orientation, lane));
				util.vector3_wide_write_slot(&minimum, lane, bounds.min);
				util.vector3_wide_write_slot(&maximum, lane, bounds.max);
				maximum_radius = simd.replace(maximum_radius, lane, bounds.maximum_radius);
				maximum_angular_expansion = simd.replace(
					maximum_angular_expansion, lane, bounds.maximum_angular_expansion,
				);
			}
		case COMPOUND_TYPE_ID:
			for lane in 0 ..< body_count
			{
				body_index := int(body_indices[lane]);
				shape_index := int(typed_index_index(active.collidables.memory[body_index].shape));
				compound := (^Compound)(&batch.data.memory[shape_index * batch.stride]);
				bounds, bounds_status := compound_bounds(
					compound, util.quaternion_wide_read_slot(orientation, lane), broad_phase.shapes,
				);
				if bounds_status != .Ok
				{
					return {}, {}, {}, {}, bounds_status;
				}
				util.vector3_wide_write_slot(&minimum, lane, bounds.min);
				util.vector3_wide_write_slot(&maximum, lane, bounds.max);
				maximum_radius = simd.replace(maximum_radius, lane, bounds.maximum_radius);
				maximum_angular_expansion = simd.replace(
					maximum_angular_expansion, lane, bounds.maximum_angular_expansion,
				);
			}
		case BIG_COMPOUND_TYPE_ID:
			for lane in 0 ..< body_count
			{
				body_index := int(body_indices[lane]);
				shape_index := int(typed_index_index(active.collidables.memory[body_index].shape));
				compound := (^Big_Compound)(&batch.data.memory[shape_index * batch.stride]);
				bounds := big_compound_bounds(
					compound, util.quaternion_wide_read_slot(orientation, lane),
				);
				util.vector3_wide_write_slot(&minimum, lane, bounds.min);
				util.vector3_wide_write_slot(&maximum, lane, bounds.max);
				maximum_radius = simd.replace(maximum_radius, lane, bounds.maximum_radius);
				maximum_angular_expansion = simd.replace(
					maximum_angular_expansion, lane, bounds.maximum_angular_expansion,
				);
			}
		case MESH_TYPE_ID:
			for lane in 0 ..< body_count
			{
				body_index := int(body_indices[lane]);
				shape_index := int(typed_index_index(active.collidables.memory[body_index].shape));
				mesh := (^Mesh)(&batch.data.memory[shape_index * batch.stride]);
				bounds := mesh_bounds(mesh, util.quaternion_wide_read_slot(orientation, lane));
				util.vector3_wide_write_slot(&minimum, lane, bounds.min);
				util.vector3_wide_write_slot(&maximum, lane, bounds.max);
				maximum_radius = simd.replace(maximum_radius, lane, bounds.maximum_radius);
				maximum_angular_expansion = simd.replace(
					maximum_angular_expansion, lane, bounds.maximum_angular_expansion,
				);
			}
		case:
			return {}, {}, {}, {}, .Invalid_Description;
	}
	return minimum, maximum, maximum_radius, maximum_angular_expansion, .Ok;
}
broad_phase_predict_bounds_motion_batch :: #force_inline proc "contextless" (
	broad_phase: ^Broad_Phase, body_indices: ^[util.PRODUCTION_LANE_COUNT]i32,
	body_count, type_id: int, dt: f32,
	position: util.Vector3_Wide, orientation: util.Quaternion_Wide,
	velocity: Body_Velocity_Wide,
) -> Physics_Status
{
	local_minimum, local_maximum, maximum_radius, maximum_angular_expansion, bounds_status :=
		broad_phase_compute_type_bounds_wide(
		broad_phase, body_indices, body_count, type_id, orientation,
	);
	if bounds_status != .Ok
	{
		return bounds_status;
	}

	active := &broad_phase.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
	minimum_margin_values: [util.PRODUCTION_LANE_COUNT]f32;
	maximum_margin_values: [util.PRODUCTION_LANE_COUNT]f32;
	allow_expansion_values: [util.PRODUCTION_LANE_COUNT]i32;
	for lane in 0 ..< body_count
	{
		body_index := int(body_indices[lane]);
		collidable := &active.collidables.memory[body_index];
		minimum_margin_values[lane] = collidable.minimum_speculative_margin;
		maximum_margin_values[lane] = collidable.maximum_speculative_margin;
		if collidable.continuity.mode != .Discrete
		{
			allow_expansion_values[lane] = -1;
		}
	}
	linear_speed := util.vector3_wide_length(velocity.linear);
	angular_expansion := broad_phase_angular_bounds_expansion_wide(
		util.vector3_wide_length(velocity.angular), dt,
		maximum_radius, maximum_angular_expansion,
	);
	margin := simd.max(
		transmute(util.F32x8)minimum_margin_values,
		simd.min(
		transmute(util.F32x8)maximum_margin_values,
		simd.add(simd.mul(linear_speed, util.F32x8(dt)), angular_expansion),
	),
	);
	maximum_allowed_expansion := util.wide_select_f32(
		transmute(util.I32x8)allow_expansion_values,
		util.F32x8(f32(math.F32_MAX)), margin,
	);
	min_expansion, max_expansion := broad_phase_motion_bounds_expansion_wide(
		velocity.linear, dt, angular_expansion, maximum_allowed_expansion,
	);
	minimum := util.vector3_wide_add(
		position, util.vector3_wide_add(local_minimum, min_expansion),
	);
	maximum := util.vector3_wide_add(
		position, util.vector3_wide_add(local_maximum, max_expansion),
	);
	return broad_phase_scatter_bounds_indices_wide(
		broad_phase, body_indices, body_count, minimum, maximum, margin,
	);
}
broad_phase_predict_type_batch_flush :: #force_inline proc "contextless" (
	broad_phase: ^Broad_Phase, type_batch: ^Broad_Phase_Predict_Type_Batch,
	type_id: int, dt: f32,
) -> Physics_Status
{
	if type_batch.count <= 0
	{
		return .Ok;
	}
	count := type_batch.count;
	type_batch.count = 0;
	return broad_phase_predict_bounds_motion_batch(
		broad_phase, &type_batch.body_indices, count, type_id, dt,
		type_batch.position, type_batch.orientation, type_batch.velocity,
	);
}
broad_phase_predict_type_batch_append :: #force_inline proc "contextless" (
	broad_phase: ^Broad_Phase, type_batch: ^Broad_Phase_Predict_Type_Batch,
	type_id, body_index: int, dt: f32,
	position: util.Vector3, orientation: util.Quaternion, velocity: Body_Velocity,
) -> Physics_Status
{
	lane := type_batch.count;
	type_batch.body_indices[lane] = i32(body_index);
	util.vector3_wide_write_slot(&type_batch.position, lane, position);
	util.quaternion_wide_write_slot(&type_batch.orientation, lane, orientation);
	util.vector3_wide_write_slot(&type_batch.velocity.linear, lane, velocity.linear);
	util.vector3_wide_write_slot(&type_batch.velocity.angular, lane, velocity.angular);
	type_batch.count += 1;
	if type_batch.count == util.PRODUCTION_LANE_COUNT
	{
		return broad_phase_predict_type_batch_flush(broad_phase, type_batch, type_id, dt);
	}
	return .Ok;
}
broad_phase_predict_type_batches_flush :: #force_inline proc "contextless" (
	broad_phase: ^Broad_Phase,
	type_batches: ^[BUILT_IN_SHAPE_TYPE_COUNT]Broad_Phase_Predict_Type_Batch,
	dt: f32,
) -> Physics_Status
{
	for type_id in 0 ..< BUILT_IN_SHAPE_TYPE_COUNT
	{
		status := broad_phase_predict_type_batch_flush(
			broad_phase, &type_batches[type_id], type_id, dt,
		);
		if status != .Ok
		{
			return status;
		}
	}
	return .Ok;
}
broad_phase_predict_bounds_bundle_grouped :: proc "contextless" (
	broad_phase: ^Broad_Phase, first_body_index, body_count: int, dt: f32,
	integrator: ^Pose_Integrator, worker_index: int,
	type_batches: ^[BUILT_IN_SHAPE_TYPE_COUNT]Broad_Phase_Predict_Type_Batch,
) -> Physics_Status
{
	active := &broad_phase.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
	index_values := [util.PRODUCTION_LANE_COUNT]i32{-1, -1, -1, -1, -1, -1, -1, -1};
	callback_index_values := index_values;
	type_values := index_values;
	mask_values: [util.PRODUCTION_LANE_COUNT]i32;
	first_type_id := -1;
	homogeneous := Reference_State.Present;
	for lane in 0 ..< body_count
	{
		body_index := first_body_index + lane;
		index_values[lane] = i32(body_index);
		dynamics := &active.dynamics_state.memory[body_index];
		if integrator != nil && (
			integrator.callbacks.integrate_kinematic_velocity == .Enabled ||
			body_inertia_mobility(dynamics.inertia.local) == .Dynamic
		)
		{
			callback_index_values[lane] = i32(body_index);
			mask_values[lane] = -1;
		}
		shape := active.collidables.memory[body_index].shape;
		if typed_index_state(shape) != .Present
		{
			homogeneous = .Missing;
			continue;
		}
		type_id := int(typed_index_type(shape));
		type_values[lane] = i32(type_id);
		if first_type_id < 0
		{
			first_type_id = type_id;
		}
		else if first_type_id != type_id
		{
			homogeneous = .Missing;
		}
	}
	body_indices := transmute(util.I32x8)index_values;
	position, orientation, velocity, inertia := bodies_gather_active_trusted(
		broad_phase.bodies, body_indices, .Local, BODY_ACCESS_ALL,
	);
	if integrator != nil
	{
		original_velocity := velocity;
		for lane in 0 ..< body_count
		{
			body_index := first_body_index + lane;
			lane_velocity := Body_Velocity{
				linear=util.vector3_wide_read_slot(original_velocity.linear, lane),
				angular=util.vector3_wide_read_slot(original_velocity.angular, lane),
			};
			island_sleeper_update_candidacy(
				&active.activity.memory[body_index], lane_velocity,
			);
		}
		integration_mask := transmute(util.I32x8)mask_values;
		integrator.callbacks.integrate_velocity(
			integrator.callbacks.user_context,
			transmute(util.I32x8)callback_index_values,
			position, orientation, inertia, integration_mask,
			worker_index, util.F32x8(dt), &velocity,
		);
		velocity.linear = util.vector3_wide_select(
			integration_mask, velocity.linear, original_velocity.linear,
		);
		velocity.angular = util.vector3_wide_select(
			integration_mask, velocity.angular, original_velocity.angular,
		);
	}

	if homogeneous == .Present && first_type_id >= 0 && first_type_id < BUILT_IN_SHAPE_TYPE_COUNT
	{
		if first_type_id <= CYLINDER_TYPE_ID
		{
			return broad_phase_predict_contiguous_primitive_bundle(
				broad_phase, first_body_index, body_count, first_type_id, dt,
				position, orientation, velocity,
			);
		}
		return broad_phase_predict_bounds_motion_batch(
			broad_phase, &index_values, body_count, first_type_id, dt,
			position, orientation, velocity,
		);
	}

	for lane in 0 ..< body_count
	{
		type_id := int(type_values[lane]);
		if type_id < 0
		{
			continue;
		}
		body_index := first_body_index + lane;
		lane_position := util.vector3_wide_read_slot(position, lane);
		lane_orientation := util.quaternion_wide_read_slot(orientation, lane);
		lane_velocity := Body_Velocity{
			linear=util.vector3_wide_read_slot(velocity.linear, lane),
			angular=util.vector3_wide_read_slot(velocity.angular, lane),
		};
		if type_id < BUILT_IN_SHAPE_TYPE_COUNT
		{
			status := broad_phase_predict_type_batch_append(
				broad_phase, &type_batches[type_id], type_id, body_index, dt,
				lane_position, lane_orientation, lane_velocity,
			);
			if status != .Ok
			{
				return status;
			}
		}
		else
		{
			status := broad_phase_predict_bounds_body_state(
				broad_phase, body_index, dt,
				lane_position, lane_orientation, lane_velocity,
			);
			if status != .Ok
			{
				return status;
			}
		}
	}
	return .Ok;
}
Broad_Phase_Predict_Job :: struct
{
	broad_phase: ^Broad_Phase,
	integrator:  ^Pose_Integrator,
	dt:          f32,
	body_count:  int,
	bundle_count: int,
	job_size:     int,
	job_count:    int,
	next_job:     i32,
	statuses:     ^[MAXIMUM_SOLVER_WORKER_COUNT]Physics_Status,
}
broad_phase_predict_worker :: proc "contextless" (
	worker_index: int, dispatcher: ^util.Thread_Dispatcher_Boundary,
)
{
	context = runtime.default_context();
	job := (^Broad_Phase_Predict_Job)(dispatcher.unmanaged_context);
	status := Physics_Status.Ok;
	type_batches: [BUILT_IN_SHAPE_TYPE_COUNT]Broad_Phase_Predict_Type_Batch;
	for
	{
		job_index := int(sync.atomic_add_explicit(&job.next_job, i32(1), .Relaxed));
		if job_index >= job.job_count
		{
			break;
		}
		start_bundle := job_index * job.job_size;
		end_bundle := min(job.bundle_count, start_bundle + job.job_size);
		for bundle_index in start_bundle ..< end_bundle
		{
			first_body_index := bundle_index * util.PRODUCTION_LANE_COUNT;
			count := min(util.PRODUCTION_LANE_COUNT, job.body_count - first_body_index);
			status = broad_phase_predict_bounds_bundle_grouped(
				job.broad_phase, first_body_index, count, job.dt,
				job.integrator, worker_index, &type_batches,
			);
			if status != .Ok
			{
				break;
			}
		}
		if status != .Ok
		{
			break;
		}
	}
	if status == .Ok
	{
		status = broad_phase_predict_type_batches_flush(job.broad_phase, &type_batches, job.dt);
	}
	job.statuses[worker_index] = status;
}
broad_phase_predict_bounds :: proc (
	broad_phase: ^Broad_Phase, dt: f32, integrator: ^Pose_Integrator = nil,
	dispatcher: ^util.Thread_Dispatcher_Boundary = nil,
) -> Physics_Status
{
	if broad_phase == nil || broad_phase.state != .Ready || broad_phase.bodies == nil || dt <= 0
	{
		return .Invalid_Argument;
	}
	active := &broad_phase.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
	body_count := active.count;
	bundle_count := (body_count + util.PRODUCTION_LANE_COUNT - 1) / util.PRODUCTION_LANE_COUNT;
	worker_count := 1;
	if dispatcher != nil && bundle_count > 0
	{
		worker_count = min(dispatcher.worker_count, bundle_count);
	}
	if worker_count > MAXIMUM_SOLVER_WORKER_COUNT
	{
		return .Capacity_Missing;
	}
	if dispatcher != nil && worker_count > 1
	{
		statuses: [MAXIMUM_SOLVER_WORKER_COUNT]Physics_Status;
		target_job_count := worker_count * 2;
		job_size := max(1, bundle_count / target_job_count);
		job_count := (bundle_count + job_size - 1) / job_size;
		worker_count = min(worker_count, job_count);
		job := Broad_Phase_Predict_Job{
			broad_phase=broad_phase,
			integrator=integrator,
			dt=dt,
			body_count=body_count,
			bundle_count=bundle_count,
			job_size=job_size,
			job_count=job_count,
			next_job=0,
			statuses=&statuses,
		};
		dispatch_status := dispatcher.dispatch(
			dispatcher, broad_phase_predict_worker, worker_count, &job,
		);
		if dispatch_status != .Ok
		{
			return .Invalid_Argument;
		}
		for worker_index in 0 ..< worker_count
		{
			if statuses[worker_index] != .Ok
			{
				return statuses[worker_index];
			}
		}
	}
	else
	{
		type_batches: [BUILT_IN_SHAPE_TYPE_COUNT]Broad_Phase_Predict_Type_Batch;
		for bundle_index in 0 ..< bundle_count
		{
			first_body_index := bundle_index * util.PRODUCTION_LANE_COUNT;
			count := min(util.PRODUCTION_LANE_COUNT, body_count - first_body_index);
			status := broad_phase_predict_bounds_bundle_grouped(
				broad_phase, first_body_index, count, dt, integrator, 0, &type_batches,
			);
			if status != .Ok
			{
				return status;
			}
		}
		status := broad_phase_predict_type_batches_flush(broad_phase, &type_batches, dt);
		if status != .Ok
		{
			return status;
		}
	}
	broad_phase.bounds_state = .Predicted;
	return .Ok;
}
broad_phase_default_refinement_scheduler :: proc "contextless" (
	optimization_fraction: f32, root_refinement_period: int,
	root_refinement_size_scale, subtree_refinement_size_scale: f32, nonpriority_period: int,
	frame_index: i32, leaf_count: int,
) -> Tree_Refinement_Schedule
{
	if leaf_count <= 0 || root_refinement_period <= 0 || nonpriority_period <= 0
	{
		return {};
	}
	refine_root := frame_index % i32(root_refinement_period) == 0;
	target_optimized_leaf_count := int(math.ceil(f32(leaf_count) * optimization_fraction));
	sqrt_leaf_count := math.sqrt(f32(leaf_count));
	target_root_refinement_size := int(math.ceil(sqrt_leaf_count * root_refinement_size_scale));
	subtree_refinement_size := int(math.ceil(sqrt_leaf_count * subtree_refinement_size_scale));
	subtree_refinements_per_root_cost := f32(0);
	if target_root_refinement_size > 1 && subtree_refinement_size > 1
	{
		subtree_refinements_per_root_cost =
			f32(target_root_refinement_size) * math.log2(f32(target_root_refinement_size)) /
			(f32(subtree_refinement_size) * math.log2(f32(subtree_refinement_size)));
	}
	root_cost := f32(0);
	if refine_root
	{
		root_cost = subtree_refinements_per_root_cost;
	}
	subtree_count_value := f32(target_optimized_leaf_count) / f32(subtree_refinement_size) - root_cost;
	subtree_refinement_count := 0;
	if subtree_count_value > 0
	{
		whole := int(math.floor(subtree_count_value));
		fraction := subtree_count_value - f32(whole);
		subtree_refinement_count = whole;
		if fraction > 0.5 || fraction == 0.5 && whole & 1 != 0
		{
			subtree_refinement_count += 1;
		}
	}
	if !refine_root
	{
		subtree_refinement_count = max(1, subtree_refinement_count);
	}
	priority := Reference_State.Missing;
	if (frame_index / i32(root_refinement_period)) % i32(nonpriority_period) != 0
	{
		priority = .Present;
	}
	root_refinement_size := 0;
	if refine_root
	{
		root_refinement_size = target_root_refinement_size;
	}
	return {
		root_refinement_size=root_refinement_size,
		subtree_refinement_count=subtree_refinement_count,
		subtree_refinement_size=subtree_refinement_size,
		use_priority_queue=priority,
	};
}
broad_phase_default_active_refinement_scheduler :: proc "contextless" (
	frame_index: i32, leaf_count: int,
) -> Tree_Refinement_Schedule
{
	return broad_phase_default_refinement_scheduler(1.0 / 20.0, 2, 1, 4, 16, frame_index, leaf_count);
}
broad_phase_default_static_refinement_scheduler :: proc "contextless" (
	frame_index: i32, leaf_count: int,
) -> Tree_Refinement_Schedule
{
	return broad_phase_default_refinement_scheduler(1.0 / 100.0, 2, 1, 4, 16, frame_index, leaf_count);
}
broad_phase_refinement_cost :: proc "contextless" (schedule: Tree_Refinement_Schedule) -> f32
{
	root_cost := f32(schedule.root_refinement_size) * math.log2(f32(schedule.root_refinement_size + 1));
	subtree_cost := f32(schedule.subtree_refinement_size * schedule.subtree_refinement_count) *
		math.log2(f32(schedule.subtree_refinement_size + 1));
	return root_cost + subtree_cost;
}
broad_phase_refinement_task_counts :: proc "contextless" (
	active_schedule, static_schedule: Tree_Refinement_Schedule, target_total_task_count: int,
) -> (target_active_task_count, target_static_task_count: int)
{
	if target_total_task_count <= 0
	{
		return;
	}
	active_cost := broad_phase_refinement_cost(active_schedule);
	static_cost := broad_phase_refinement_cost(static_schedule);
	total_cost := active_cost + static_cost;
	active_fraction := f32(0.5);
	if total_cost > 0
	{
		active_fraction = active_cost / total_cost;
	}
	target_active_task_count = clamp(
		int(math.ceil(active_fraction * f32(target_total_task_count))), 0, target_total_task_count,
	);
	target_static_task_count = target_total_task_count - target_active_task_count;
	return;
}
broad_phase_update :: proc "contextless" (
	broad_phase: ^Broad_Phase, dispatcher: ^util.Thread_Dispatcher_Boundary = nil,
) -> Physics_Status
{
	status := Physics_Status.Ok;
	if broad_phase.bounds_state != .Predicted
	{
		status = broad_phase_update_bounds(broad_phase);
		if status != .Ok
		{
			return status;
		}
	}
	workspace := &broad_phase.maintenance_workspace;
	if dispatcher != nil && (
		dispatcher.dispatch == nil || dispatcher.worker_count <= 0 || dispatcher.worker_count > workspace.worker_count
	)
	{
		return .Invalid_Argument;
	}
	active_schedule := broad_phase_default_active_refinement_scheduler(
		broad_phase.frame_index, broad_phase.active_tree.leaf_count,
	);
	static_schedule := broad_phase_default_static_refinement_scheduler(
		broad_phase.frame_index, broad_phase.static_tree.leaf_count,
	);
	for worker_index in 0 ..< workspace.worker_count
	{
		workspace.statuses.memory[worker_index] = .Ok;
		workspace.worker_hits.memory[worker_index] = 0;
	}
	minimum_leaf_count_for_threading :: 256;
	use_threads := dispatcher != nil && dispatcher.worker_count > 1 && (
		broad_phase.active_tree.leaf_count >= minimum_leaf_count_for_threading ||
		broad_phase.static_tree.leaf_count >= minimum_leaf_count_for_threading
	);
	if use_threads
	{
		target_total_task_count := dispatcher.worker_count;
		target_active_task_count, target_static_task_count := broad_phase_refinement_task_counts(
			active_schedule, static_schedule, target_total_task_count,
		);
		active_target_count, active_task_count, active_total_subtree_leaf_count, prepare_status := tree_prepare_refinement_tasks(
			&broad_phase.active_tree, active_schedule, &broad_phase.active_subtree_refinement_start_index,
			workspace.refinement_targets, .Active, workspace.task_records,
		);
		if prepare_status != .Ok
		{
			return prepare_status;
		}
		static_target_count, static_task_count, static_total_subtree_leaf_count, static_prepare_status := tree_prepare_refinement_tasks(
			&broad_phase.static_tree, static_schedule, &broad_phase.static_subtree_refinement_start_index,
			workspace.static_refinement_targets, .Static, workspace.static_task_records,
		);
		if static_prepare_status != .Ok
		{
			return static_prepare_status;
		}
		ctx := Tree_Update2_Context{
			active_tree=&broad_phase.active_tree,
			static_tree=&broad_phase.static_tree,
			workspace=workspace,
			active_schedule=active_schedule,
			static_schedule=static_schedule,
			active_target_count=active_target_count,
			static_target_count=static_target_count,
			active_task_count=active_task_count,
			static_task_count=static_task_count,
			active_total_subtree_leaf_count=active_total_subtree_leaf_count,
			static_total_subtree_leaf_count=static_total_subtree_leaf_count,
			target_active_task_count=target_active_task_count,
			target_static_task_count=target_static_task_count,
			target_total_task_count=target_total_task_count,
			cache_active_tree=.Present,
		};
		status = tree_dispatch_update2_entries(&ctx, dispatcher);
	}
	else
	{
		status = tree_refine2_single(
			&broad_phase.static_tree, static_schedule, &broad_phase.static_subtree_refinement_start_index,
			workspace, .Static,
		);
		if status == .Ok
		{
			status = tree_refine2_single(
				&broad_phase.active_tree, active_schedule, &broad_phase.active_subtree_refinement_start_index,
				workspace, .Active,
			);
		}
		if status == .Ok
		{
			status = tree_refit2_with_cache_optimization_single(&broad_phase.active_tree, workspace);
		}
	}
	if status == .Ok
	{
		broad_phase.bounds_state = .Current;
		next_frame := i32(0);
		if broad_phase.frame_index != max(i32)
		{
			next_frame = broad_phase.frame_index + 1;
		}
		broad_phase.active_tree.refinement_frame = u32(next_frame);
		broad_phase.static_tree.refinement_frame = u32(next_frame);
		broad_phase.frame_index = next_frame;
	}
	return status;
}
