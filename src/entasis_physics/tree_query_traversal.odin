// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"
import "base:intrinsics"
import "base:runtime"

TREE_TRAVERSAL_STACK_CAPACITY :: 256;

Tree_Ray_Leaf_Proc :: #type proc "contextless" (
	user_context: rawptr, leaf_index: int, maximum_t: ^f32,
) -> Physics_Status;

Tree_Volume_Leaf_Proc :: #type proc "contextless" (
	user_context: rawptr, leaf_index: int,
) -> Physics_Status;

tree_traversal_stack_push :: proc "contextless" (
	inline_stack: ^[TREE_TRAVERSAL_STACK_CAPACITY]i32,
	pooled_stack: ^util.Buffer(i32), stack_count: ^int, value: i32,
	pool: ^util.Buffer_Pool,
) -> Physics_Status
{
	if inline_stack == nil || pooled_stack == nil || stack_count == nil || stack_count^ < 0
	{
		return .Invalid_Argument;
	}
	context = runtime.default_context();
	if pooled_stack.memory == nil
	{
		if stack_count^ < len(inline_stack^)
		{
			return .Invalid_Argument;
		}
		if pool == nil
		{
			return .Capacity_Missing;
		}
		take_status: util.Memory_Status;
		pooled_stack^, take_status = util.buffer_pool_take_at_least(
			pool, i32, len(inline_stack^) * 2,
		);
		if take_status != .Ok
		{
			return physics_memory_status(take_status);
		}
		for index in 0 ..< stack_count^
		{
			pooled_stack.memory[index] = inline_stack[index];
		}
	}
	else if stack_count^ >= int(pooled_stack.length)
	{
		if pool == nil || int(pooled_stack.length) > max(int) / 2
		{
			return .Capacity_Missing;
		}
		resize_status := util.buffer_pool_resize_to_at_least(
			pool, pooled_stack, int(pooled_stack.length) * 2, stack_count^,
		);
		if resize_status != .Ok
		{
			return physics_memory_status(resize_status);
		}
	}
	pooled_stack.memory[stack_count^] = value;
	stack_count^ += 1;
	return .Ok;
}

tree_traversal_stack_return :: proc "contextless" (
	pool: ^util.Buffer_Pool, stack: ^util.Buffer(i32),
)
{
	context = runtime.default_context();
	physics_return_buffer(pool, stack);
}

Tree_Closest_Traversal_Entry :: struct
{
	node_index: i32,
	minimum_t: f32,
}

#assert(size_of(Tree_Closest_Traversal_Entry) == 8);

tree_closest_traversal_stack_push :: proc "contextless" (
	inline_stack: ^[TREE_TRAVERSAL_STACK_CAPACITY]Tree_Closest_Traversal_Entry,
	pooled_stack: ^util.Buffer(Tree_Closest_Traversal_Entry), stack_count: ^int,
	entry: Tree_Closest_Traversal_Entry, pool: ^util.Buffer_Pool,
) -> Physics_Status
{
	if inline_stack == nil || pooled_stack == nil || stack_count == nil || stack_count^ < 0
	{
		return .Invalid_Argument;
	}
	context = runtime.default_context();
	if pooled_stack.memory == nil
	{
		if stack_count^ < len(inline_stack^)
		{
			return .Invalid_Argument;
		}
		if pool == nil
		{
			return .Capacity_Missing;
		}
		take_status: util.Memory_Status;
		pooled_stack^, take_status = util.buffer_pool_take_at_least(
			pool, Tree_Closest_Traversal_Entry, len(inline_stack^) * 2,
		);
		if take_status != .Ok
		{
			return physics_memory_status(take_status);
		}
		for index in 0 ..< stack_count^
		{
			pooled_stack.memory[index] = inline_stack[index];
		}
	}
	else if stack_count^ >= int(pooled_stack.length)
	{
		if pool == nil || int(pooled_stack.length) > max(int) / 2
		{
			return .Capacity_Missing;
		}
		resize_status := util.buffer_pool_resize_to_at_least(
			pool, pooled_stack, int(pooled_stack.length) * 2, stack_count^,
		);
		if resize_status != .Ok
		{
			return physics_memory_status(resize_status);
		}
	}
	pooled_stack.memory[stack_count^] = entry;
	stack_count^ += 1;
	return .Ok;
}

tree_closest_traversal_stack_return :: proc "contextless" (
	pool: ^util.Buffer_Pool, stack: ^util.Buffer(Tree_Closest_Traversal_Entry),
)
{
	context = runtime.default_context();
	physics_return_buffer(pool, stack);
}

tree_closest_traversal_stack_pop :: #force_inline proc "contextless" (
	inline_stack: ^[TREE_TRAVERSAL_STACK_CAPACITY]Tree_Closest_Traversal_Entry,
	pooled_stack: ^util.Buffer(Tree_Closest_Traversal_Entry), stack_count: ^int,
	maximum_t: f32,
) -> (i32, Reference_State)
{
	for stack_count^ > 0
	{
		stack_count^ -= 1;
		entry: Tree_Closest_Traversal_Entry;
		if pooled_stack.memory != nil
		{
			entry = pooled_stack.memory[stack_count^];
		}
		else
		{
			entry = inline_stack[stack_count^];
		}
		if entry.minimum_t <= maximum_t
		{
			return entry.node_index, .Present;
		}
	}
	return 0, .Missing;
}

Tree_Ray_Traversal_Data :: struct
{
	origin:            util.Vector3,
	inverse_direction: util.Vector3,
	parallel_mask:     u8,
}

tree_ray_traversal_data :: #force_inline proc "contextless" (
	ray: Tree_Ray,
) -> Tree_Ray_Traversal_Data
{
	util.math_validate_vector3(ray.origin);
	util.math_validate_vector3(ray.direction);
	util.math_validate_f32(ray.maximum_t);
	result := Tree_Ray_Traversal_Data{origin=ray.origin};
	if abs(ray.direction.x) <= 1e-20
	{
		result.parallel_mask |= 1;
	}
	else
	{
		result.inverse_direction.x = 1 / ray.direction.x;
	}
	if abs(ray.direction.y) <= 1e-20
	{
		result.parallel_mask |= 2;
	}
	else
	{
		result.inverse_direction.y = 1 / ray.direction.y;
	}
	if abs(ray.direction.z) <= 1e-20
	{
		result.parallel_mask |= 4;
	}
	else
	{
		result.inverse_direction.z = 1 / ray.direction.z;
	}
	return result;
}

tree_ray_intersection_prepared :: #force_inline proc "contextless" (
	bounds: util.Bounding_Box,
	ray: Tree_Ray_Traversal_Data,
	maximum_t: f32,
) -> (t: f32, state: Reference_State)
{
	util.math_validate_bounding_box(bounds);
	util.math_validate_vector3(ray.origin);
	util.math_validate_vector3(ray.inverse_direction);
	util.math_validate_f32(maximum_t);
	t_min := f32(0);
	t_max := maximum_t;

	if ray.parallel_mask & 1 != 0
	{
		if ray.origin.x < bounds.min.x || ray.origin.x > bounds.max.x
		{
			return 0, .Missing;
		}
	}
	else
	{
		entry := (bounds.min.x - ray.origin.x) * ray.inverse_direction.x;
		exit := (bounds.max.x - ray.origin.x) * ray.inverse_direction.x;
		if entry > exit
		{
			entry, exit = exit, entry;
		}
		t_min = tree_ordered_max(t_min, entry);
		t_max = tree_ordered_min(t_max, exit);
		if t_min > t_max
		{
			return 0, .Missing;
		}
	}

	if ray.parallel_mask & 2 != 0
	{
		if ray.origin.y < bounds.min.y || ray.origin.y > bounds.max.y
		{
			return 0, .Missing;
		}
	}
	else
	{
		entry := (bounds.min.y - ray.origin.y) * ray.inverse_direction.y;
		exit := (bounds.max.y - ray.origin.y) * ray.inverse_direction.y;
		if entry > exit
		{
			entry, exit = exit, entry;
		}
		t_min = tree_ordered_max(t_min, entry);
		t_max = tree_ordered_min(t_max, exit);
		if t_min > t_max
		{
			return 0, .Missing;
		}
	}

	if ray.parallel_mask & 4 != 0
	{
		if ray.origin.z < bounds.min.z || ray.origin.z > bounds.max.z
		{
			return 0, .Missing;
		}
	}
	else
	{
		entry := (bounds.min.z - ray.origin.z) * ray.inverse_direction.z;
		exit := (bounds.max.z - ray.origin.z) * ray.inverse_direction.z;
		if entry > exit
		{
			entry, exit = exit, entry;
		}
		t_min = tree_ordered_max(t_min, entry);
		t_max = tree_ordered_min(t_max, exit);
		if t_min > t_max
		{
			return 0, .Missing;
		}
	}
	return t_min, .Present;
}

// query-local XYZ lanes use the existing 32-byte child layout. the fourth lane
// contains integer metadata in storage and is cleared before floating arithmetic
#assert(offset_of(Tree_Node_Child, max) == 16);
#assert(size_of(Tree_Node_Child) == 32);
#assert(offset_of(Tree_Node_Child, max) + size_of(#simd[4]f32) <= size_of(Tree_Node_Child));

@(private="file")
Tree_Query_Ray_Packed :: struct
{
	origin: #simd[4]f32,
	inverse: #simd[4]f32,
	parallel: #simd[4]u32,
	parallel_exit: #simd[4]f32,
}

@(private="file")
tree_query_ray_packed :: #force_inline proc "contextless" (
	ray: Tree_Ray,
) -> Tree_Query_Ray_Packed
{
	util.math_validate_vector3(ray.origin);
	util.math_validate_vector3(ray.direction);
	util.math_validate_f32(ray.maximum_t);
	direction := #simd[4]f32{ray.direction.x, ray.direction.y, ray.direction.z, 0};
	parallel := intrinsics.simd_lanes_le(
		intrinsics.simd_abs(direction), #simd[4]f32{1e-20, 1e-20, 1e-20, 1e-20},
	);
	ones := #simd[4]f32{1, 1, 1, 1};
	inverse := ones / intrinsics.simd_select(parallel, ones, direction);
	inverse = transmute(#simd[4]f32)((transmute(#simd[4]u32)inverse) & ~parallel);
	util.math_validate_vector3({
		intrinsics.simd_extract(inverse, 0), intrinsics.simd_extract(inverse, 1), intrinsics.simd_extract(inverse, 2),
	});
	limit := f32(3.402823466e+38);
	return {
		origin={ray.origin.x, ray.origin.y, ray.origin.z, 0},
		inverse=inverse,
		parallel=parallel,
		parallel_exit=intrinsics.simd_select(parallel,
			#simd[4]f32{limit, limit, limit, limit},
			#simd[4]f32{-limit, -limit, -limit, -limit}),
	};
}

@(private="file")
tree_ray_intersection_packed :: #force_inline proc "contextless" (
	minimum, maximum: #simd[4]f32, ray: Tree_Query_Ray_Packed, maximum_t: f32,
) -> (f32, Reference_State)
{
	util.math_validate_f32(maximum_t);
	outside := (intrinsics.simd_lanes_lt(ray.origin, minimum) |
		intrinsics.simd_lanes_gt(ray.origin, maximum)) & ray.parallel;
	if intrinsics.simd_extract_msbs(outside) != {}
	{
		return 0, .Missing;
	}
	// parallel lanes must not participate in the interval, including finite
	// extreme inputs whose subtraction overflows before multiplication by zero
	a := (minimum - ray.origin) * ray.inverse;
	b := (maximum - ray.origin) * ray.inverse;
	entry := intrinsics.simd_select(intrinsics.simd_lanes_lt(a, b), a, b);
	exit := intrinsics.simd_select(intrinsics.simd_lanes_gt(a, b), a, b);
	entry = transmute(#simd[4]f32)((transmute(#simd[4]u32)entry) & ~ray.parallel);
	exit = intrinsics.simd_select(intrinsics.simd_lanes_gt(exit, ray.parallel_exit), exit, ray.parallel_exit);
	// retain the scalar X/Y/Z fold and inclusive boundary semantics. do not
	// reassociate with a horizontal reduction or use approximate reciprocals
	t_min := tree_ordered_max(f32(0), intrinsics.simd_extract(entry, 0));
	t_min = tree_ordered_max(t_min, intrinsics.simd_extract(entry, 1));
	t_min = tree_ordered_max(t_min, intrinsics.simd_extract(entry, 2));
	t_max := tree_ordered_min(maximum_t, intrinsics.simd_extract(exit, 0));
	t_max = tree_ordered_min(t_max, intrinsics.simd_extract(exit, 1));
	t_max = tree_ordered_min(t_max, intrinsics.simd_extract(exit, 2));
	if t_min > t_max
	{
		return 0, .Missing;
	}
	return t_min, .Present;
}

@(private="file")
tree_ray_child_intersection_packed :: #force_inline proc "contextless" (
	child: ^Tree_Node_Child, ray: Tree_Query_Ray_Packed, maximum_t: f32,
	expansion: #simd[4]f32, $EXPAND: bool,
) -> (f32, Reference_State)
{
	XYZ :: #simd[4]u32{max(u32), max(u32), max(u32), 0};
	minimum := transmute(#simd[4]f32)((transmute(#simd[4]u32)
		intrinsics.unaligned_load((^#simd[4]f32)(&child.min))) & XYZ);
	maximum := transmute(#simd[4]f32)((transmute(#simd[4]u32)
		intrinsics.unaligned_load((^#simd[4]f32)(&child.max))) & XYZ);
	when EXPAND
	{
		minimum -= expansion;
		maximum += expansion;
	}
	util.math_validate_bounding_box({
		min={intrinsics.simd_extract(minimum, 0), intrinsics.simd_extract(minimum, 1), intrinsics.simd_extract(minimum, 2)},
		max={intrinsics.simd_extract(maximum, 0), intrinsics.simd_extract(maximum, 1), intrinsics.simd_extract(maximum, 2)},
	});
	return tree_ray_intersection_packed(minimum, maximum, ray, maximum_t);
}

tree_ray_traverse :: proc "contextless" (
	tree: ^Tree, ray: Tree_Ray, leaf_proc: Tree_Ray_Leaf_Proc, user_context: rawptr,
	pool: ^util.Buffer_Pool = nil,
) -> (f32, Physics_Status)
{
	if tree == nil || tree.state != .Ready || leaf_proc == nil || ray.maximum_t < 0
	{
		return ray.maximum_t, .Invalid_Argument;
	}
	maximum_t := ray.maximum_t;
	if tree.leaf_count == 0
	{
		return maximum_t, .Ok;
	}
	prepared_ray := tree_query_ray_packed(ray);
	if tree.leaf_count == 1
	{
		_, hit := tree_ray_child_intersection_packed(&tree.nodes.memory[0].a, prepared_ray, maximum_t, {}, false);
		if hit == .Present
		{
			status := leaf_proc(user_context, 0, &maximum_t);
			if status != .Ok
			{
				return maximum_t, status;
			}
		}
		return maximum_t, .Ok;
	}

	stack: [TREE_TRAVERSAL_STACK_CAPACITY]i32 = ---;
	pooled_stack: util.Buffer(i32);
	defer if pooled_stack.memory != nil
	{
		tree_traversal_stack_return(pool, &pooled_stack);
	}
	stack_count := 0;
	node_index := i32(0);
	for
	{
		if node_index < 0
		{
			status := leaf_proc(user_context, tree_decode_leaf(node_index), &maximum_t);
			if status != .Ok
			{
				return maximum_t, status;
			}
			if stack_count == 0
			{
				break;
			}
			stack_count -= 1;
			if pooled_stack.memory != nil
			{
				node_index = pooled_stack.memory[stack_count];
			}
			else
			{
				node_index = stack[stack_count];
			}
			continue;
		}
		node := &tree.nodes.memory[node_index];
		t_a, hit_a := tree_ray_child_intersection_packed(&node.a, prepared_ray, maximum_t, {}, false);
		t_b, hit_b := tree_ray_child_intersection_packed(&node.b, prepared_ray, maximum_t, {}, false);
		if hit_a == .Present && hit_b == .Present
		{
			if t_a < t_b
			{
				node_index = node.a.index;
				if pooled_stack.memory == nil && stack_count < len(stack)
				{
					stack[stack_count] = node.b.index;
					stack_count += 1;
				}
				else
				{
					status := tree_traversal_stack_push(
						&stack, &pooled_stack, &stack_count, node.b.index, pool,
					);
					if status != .Ok
					{
						return maximum_t, status;
					}
				}
			}
			else
			{
				node_index = node.b.index;
				if pooled_stack.memory == nil && stack_count < len(stack)
				{
					stack[stack_count] = node.a.index;
					stack_count += 1;
				}
				else
				{
					status := tree_traversal_stack_push(
						&stack, &pooled_stack, &stack_count, node.a.index, pool,
					);
					if status != .Ok
					{
						return maximum_t, status;
					}
				}
			}
		}
		else if hit_a == .Present
		{
			node_index = node.a.index;
		}
		else if hit_b == .Present
		{
			node_index = node.b.index;
		}
		else
		{
			if stack_count == 0
			{
				break;
			}
			stack_count -= 1;
			if pooled_stack.memory != nil
			{
				node_index = pooled_stack.memory[stack_count];
			}
			else
			{
				node_index = stack[stack_count];
			}
		}
	}
	return maximum_t, .Ok;
}

// finish an unvisited subtree with constant storage. parent links reconstruct
// the original near/far order under the immutable subtree-entry limit. deferred
// siblings are admitted against the live limit with the scalar <= comparison.
// the caller retains all work outside subtree_root in its own frontier
tree_ray_subtree_continue :: proc "contextless" (
	tree: ^Tree, subtree_root: i32, ray: Tree_Ray, maximum_t: ^f32,
	leaf_proc: Tree_Ray_Leaf_Proc, user_context: rawptr,
) -> Physics_Status
{
	if subtree_root < 0
	{
		return leaf_proc(user_context, tree_decode_leaf(subtree_root), maximum_t);
	}
	prepared := tree_ray_traversal_data(ray);
	node_index := subtree_root;
	returned_child := i32(-1);
	for
	{
		if node_index < 0 || int(node_index) >= tree.node_count
		{
			return .Invalid_Description;
		}
		node := &tree.nodes.memory[node_index];
		next_child := i32(-1);
		if returned_child < 0
		{
			t_a, hit_a := tree_ray_intersection_prepared(tree_child_bounds(node.a), prepared, maximum_t^);
			t_b, hit_b := tree_ray_intersection_prepared(tree_child_bounds(node.b), prepared, maximum_t^);
			if hit_a == .Present && (hit_b == .Missing || t_a < t_b)
			{
				next_child = 0;
			}
			else if hit_b == .Present
			{
				next_child = 1;
			}
		}
		else
		{
			t_a, hit_a := tree_ray_intersection_prepared(tree_child_bounds(node.a), prepared, ray.maximum_t);
			t_b, hit_b := tree_ray_intersection_prepared(tree_child_bounds(node.b), prepared, ray.maximum_t);
			if hit_a == .Present && hit_b == .Present
			{
				first := i32(1);
				deferred_t := t_a;
				if t_a < t_b
				{
					first = 0;
					deferred_t = t_b;
				}
				if returned_child == first && deferred_t <= maximum_t^
				{
					next_child = 1 - first;
				}
			}
		}
		if next_child >= 0
		{
			child := tree_child(node, int(next_child));
			if child.index < 0
			{
				status := leaf_proc(user_context, tree_decode_leaf(child.index), maximum_t);
				if status != .Ok
				{
					return status;
				}
				returned_child = next_child;
			}
			else
			{
				node_index = child.index;
				returned_child = -1;
			}
			continue;
		}
		if node_index == subtree_root
		{
			return .Ok;
		}
		meta := tree.metanodes.memory[node_index];
		node_index = meta.parent;
		returned_child = meta.index_in_parent;
	}
}

tree_ray_traverse_closest :: proc "contextless" (
	tree: ^Tree, ray: Tree_Ray, leaf_proc: Tree_Ray_Leaf_Proc, user_context: rawptr,
	pool: ^util.Buffer_Pool = nil,
) -> (f32, Physics_Status)
{
	if tree == nil || tree.state != .Ready || leaf_proc == nil || ray.maximum_t < 0
	{
		return ray.maximum_t, .Invalid_Argument;
	}
	maximum_t := ray.maximum_t;
	if tree.leaf_count == 0
	{
		return maximum_t, .Ok;
	}
	prepared_ray := tree_query_ray_packed(ray);
	if tree.leaf_count == 1
	{
		_, hit := tree_ray_child_intersection_packed(&tree.nodes.memory[0].a, prepared_ray, maximum_t, {}, false);
		if hit == .Present
		{
			status := leaf_proc(user_context, 0, &maximum_t);
			if status != .Ok
			{
				return maximum_t, status;
			}
		}
		return maximum_t, .Ok;
	}

	stack: [TREE_TRAVERSAL_STACK_CAPACITY]Tree_Closest_Traversal_Entry = ---;
	// closest rays never borrow mutable pool storage. the empty buffer keeps the
	// shared inline-pop operation used by sweeps without changing sweep policy
	pooled_stack: util.Buffer(Tree_Closest_Traversal_Entry);
	stack_count := 0;
	node_index := i32(0);
	for
	{
		if node_index < 0
		{
			status := leaf_proc(user_context, tree_decode_leaf(node_index), &maximum_t);
			if status != .Ok
			{
				return maximum_t, status;
			}
			next_index, next_state := tree_closest_traversal_stack_pop(
				&stack, &pooled_stack, &stack_count, maximum_t,
			);
			if next_state == .Missing
			{
				break;
			}
			node_index = next_index;
			continue;
		}
		if stack_count == len(stack)
		{
			bounded_ray := ray;
			bounded_ray.maximum_t = maximum_t;
			status := tree_ray_subtree_continue(tree, node_index, bounded_ray, &maximum_t, leaf_proc, user_context);
			if status != .Ok
			{
				return maximum_t, status;
			}
			next_index, next_state := tree_closest_traversal_stack_pop(&stack, &pooled_stack, &stack_count, maximum_t);
			if next_state == .Missing
			{
				break;
			}
			node_index = next_index;
			continue;
		}
		node := &tree.nodes.memory[node_index];
		t_a, hit_a := tree_ray_child_intersection_packed(&node.a, prepared_ray, maximum_t, {}, false);
		t_b, hit_b := tree_ray_child_intersection_packed(&node.b, prepared_ray, maximum_t, {}, false);
		if hit_a == .Present && hit_b == .Present
		{
			if t_a < t_b
			{
				node_index = node.a.index;
				stack[stack_count] = {node_index=node.b.index, minimum_t=t_b};
				stack_count += 1;
			}
			else
			{
				node_index = node.b.index;
				stack[stack_count] = {node_index=node.a.index, minimum_t=t_a};
				stack_count += 1;
			}
		}
		else if hit_a == .Present
		{
			node_index = node.a.index;
		}
		else if hit_b == .Present
		{
			node_index = node.b.index;
		}
		else
		{
			next_index, next_state := tree_closest_traversal_stack_pop(
				&stack, &pooled_stack, &stack_count, maximum_t,
			);
			if next_state == .Missing
			{
				break;
			}
			node_index = next_index;
		}
	}
	return maximum_t, .Ok;
}

tree_sweep_traverse :: proc "contextless" (
	tree: ^Tree, bounds: util.Bounding_Box, direction: util.Vector3, maximum_t: f32,
	leaf_proc: Tree_Ray_Leaf_Proc, user_context: rawptr,
	pool: ^util.Buffer_Pool = nil,
) -> (f32, Physics_Status)
{
	if tree == nil || tree.state != .Ready || leaf_proc == nil || maximum_t < 0 ||
		bounds.min.x > bounds.max.x || bounds.min.y > bounds.max.y || bounds.min.z > bounds.max.z
	{
		return maximum_t, .Invalid_Argument;
	}
	current_maximum := maximum_t;
	if tree.leaf_count == 0
	{
		return current_maximum, .Ok;
	}
	half_min := util.vector3_scale(bounds.min, 0.5);
	half_max := util.vector3_scale(bounds.max, 0.5);
	expansion := util.vector3_subtract(half_max, half_min);
	wide_expansion := #simd[4]f32{expansion.x, expansion.y, expansion.z, 0};
	origin := util.vector3_add(half_max, half_min);
	ray := Tree_Ray{origin=origin, direction=direction, maximum_t=maximum_t};
	prepared_ray := tree_query_ray_packed(ray);
	if tree.leaf_count == 1
	{
		_, hit := tree_ray_child_intersection_packed(
			&tree.nodes.memory[0].a, prepared_ray, current_maximum, wide_expansion, true,
		);
		if hit == .Present
		{
			status := leaf_proc(user_context, 0, &current_maximum);
			if status != .Ok
			{
				return current_maximum, status;
			}
		}
		return current_maximum, .Ok;
	}

	stack: [TREE_TRAVERSAL_STACK_CAPACITY]i32 = ---;
	pooled_stack: util.Buffer(i32);
	defer if pooled_stack.memory != nil
	{
		tree_traversal_stack_return(pool, &pooled_stack);
	}
	stack_count := 0;
	node_index := i32(0);
	for
	{
		if node_index < 0
		{
			status := leaf_proc(user_context, tree_decode_leaf(node_index), &current_maximum);
			if status != .Ok
			{
				return current_maximum, status;
			}
			if stack_count == 0
			{
				break;
			}
			stack_count -= 1;
			if pooled_stack.memory != nil
			{
				node_index = pooled_stack.memory[stack_count];
			}
			else
			{
				node_index = stack[stack_count];
			}
			continue;
		}
		node := &tree.nodes.memory[node_index];
		t_a, hit_a := tree_ray_child_intersection_packed(&node.a, prepared_ray, current_maximum, wide_expansion, true);
		t_b, hit_b := tree_ray_child_intersection_packed(&node.b, prepared_ray, current_maximum, wide_expansion, true);
		if hit_a == .Present && hit_b == .Present
		{
			if t_a < t_b
			{
				node_index = node.a.index;
				if pooled_stack.memory == nil && stack_count < len(stack)
				{
					stack[stack_count] = node.b.index;
					stack_count += 1;
				}
				else
				{
					status := tree_traversal_stack_push(
						&stack, &pooled_stack, &stack_count, node.b.index, pool,
					);
					if status != .Ok
					{
						return current_maximum, status;
					}
				}
			}
			else
			{
				node_index = node.b.index;
				if pooled_stack.memory == nil && stack_count < len(stack)
				{
					stack[stack_count] = node.a.index;
					stack_count += 1;
				}
				else
				{
					status := tree_traversal_stack_push(
						&stack, &pooled_stack, &stack_count, node.a.index, pool,
					);
					if status != .Ok
					{
						return current_maximum, status;
					}
				}
			}
		}
		else if hit_a == .Present
		{
			node_index = node.a.index;
		}
		else if hit_b == .Present
		{
			node_index = node.b.index;
		}
		else
		{
			if stack_count == 0
			{
				break;
			}
			stack_count -= 1;
			if pooled_stack.memory != nil
			{
				node_index = pooled_stack.memory[stack_count];
			}
			else
			{
				node_index = stack[stack_count];
			}
		}
	}
	return current_maximum, .Ok;
}

tree_sweep_traverse_closest :: proc "contextless" (
	tree: ^Tree, bounds: util.Bounding_Box, direction: util.Vector3, maximum_t: f32,
	leaf_proc: Tree_Ray_Leaf_Proc, user_context: rawptr,
	pool: ^util.Buffer_Pool = nil,
) -> (f32, Physics_Status)
{
	if tree == nil || tree.state != .Ready || leaf_proc == nil || maximum_t < 0 ||
		bounds.min.x > bounds.max.x || bounds.min.y > bounds.max.y || bounds.min.z > bounds.max.z
	{
		return maximum_t, .Invalid_Argument;
	}
	current_maximum := maximum_t;
	if tree.leaf_count == 0
	{
		return current_maximum, .Ok;
	}
	half_min := util.vector3_scale(bounds.min, 0.5);
	half_max := util.vector3_scale(bounds.max, 0.5);
	expansion := util.vector3_subtract(half_max, half_min);
	wide_expansion := #simd[4]f32{expansion.x, expansion.y, expansion.z, 0};
	origin := util.vector3_add(half_max, half_min);
	ray := Tree_Ray{origin=origin, direction=direction, maximum_t=maximum_t};
	prepared_ray := tree_query_ray_packed(ray);
	if tree.leaf_count == 1
	{
		_, hit := tree_ray_child_intersection_packed(
			&tree.nodes.memory[0].a, prepared_ray, current_maximum, wide_expansion, true,
		);
		if hit == .Present
		{
			status := leaf_proc(user_context, 0, &current_maximum);
			if status != .Ok
			{
				return current_maximum, status;
			}
		}
		return current_maximum, .Ok;
	}

	stack: [TREE_TRAVERSAL_STACK_CAPACITY]Tree_Closest_Traversal_Entry = ---;
	pooled_stack: util.Buffer(Tree_Closest_Traversal_Entry);
	defer if pooled_stack.memory != nil
	{
		tree_closest_traversal_stack_return(pool, &pooled_stack);
	}
	stack_count := 0;
	node_index := i32(0);
	for
	{
		if node_index < 0
		{
			status := leaf_proc(user_context, tree_decode_leaf(node_index), &current_maximum);
			if status != .Ok
			{
				return current_maximum, status;
			}
			next_index, next_state := tree_closest_traversal_stack_pop(
				&stack, &pooled_stack, &stack_count, current_maximum,
			);
			if next_state == .Missing
			{
				break;
			}
			node_index = next_index;
			continue;
		}
		node := &tree.nodes.memory[node_index];
		t_a, hit_a := tree_ray_child_intersection_packed(&node.a, prepared_ray, current_maximum, wide_expansion, true);
		t_b, hit_b := tree_ray_child_intersection_packed(&node.b, prepared_ray, current_maximum, wide_expansion, true);
		if hit_a == .Present && hit_b == .Present
		{
			if t_a < t_b
			{
				node_index = node.a.index;
				if pooled_stack.memory == nil && stack_count < len(stack)
				{
					stack[stack_count] = {node_index=node.b.index, minimum_t=t_b};
					stack_count += 1;
				}
				else
				{
					status := tree_closest_traversal_stack_push(
						&stack, &pooled_stack, &stack_count,
						{node_index=node.b.index, minimum_t=t_b}, pool,
					);
					if status != .Ok
					{
						return current_maximum, status;
					}
				}
			}
			else
			{
				node_index = node.b.index;
				if pooled_stack.memory == nil && stack_count < len(stack)
				{
					stack[stack_count] = {node_index=node.a.index, minimum_t=t_a};
					stack_count += 1;
				}
				else
				{
					status := tree_closest_traversal_stack_push(
						&stack, &pooled_stack, &stack_count,
						{node_index=node.a.index, minimum_t=t_a}, pool,
					);
					if status != .Ok
					{
						return current_maximum, status;
					}
				}
			}
		}
		else if hit_a == .Present
		{
			node_index = node.a.index;
		}
		else if hit_b == .Present
		{
			node_index = node.b.index;
		}
		else
		{
			next_index, next_state := tree_closest_traversal_stack_pop(
				&stack, &pooled_stack, &stack_count, current_maximum,
			);
			if next_state == .Missing
			{
				break;
			}
			node_index = next_index;
		}
	}
	return current_maximum, .Ok;
}

tree_volume_any :: proc "contextless" (
	tree: ^Tree, query: util.Bounding_Box,
) -> (int, Reference_State, Physics_Status)
{
	if tree == nil || tree.state != .Ready ||
		query.min.x > query.max.x || query.min.y > query.max.y || query.min.z > query.max.z
	{
		return -1, .Missing, .Invalid_Argument;
	}
	if tree.leaf_count == 0
	{
		return -1, .Missing, .Ok;
	}
	if tree.leaf_count == 1
	{
		if tree_bounds_intersect(tree_child_bounds(tree.nodes.memory[0].a), query) == .Present
		{
			return 0, .Present, .Ok;
		}
		return -1, .Missing, .Ok;
	}

	node_index := i32(0);
	child_index := i32(0);
	for
	{
		node := &tree.nodes.memory[node_index];
		child := &node.a;
		if child_index != 0
		{
			child = &node.b;
		}
		if child.max.x >= query.min.x && child.min.x <= query.max.x &&
			child.max.y >= query.min.y && child.min.y <= query.max.y &&
			child.max.z >= query.min.z && child.min.z <= query.max.z
		{
			if child.index < 0
			{
				return tree_decode_leaf(child.index), .Present, .Ok;
			}
			node_index = child.index;
			child_index = 0;
			continue;
		}
		if child_index == 0
		{
			child_index = 1;
			continue;
		}

		for
		{
			if node_index == 0
			{
				return -1, .Missing, .Ok;
			}
			metadata := tree.metanodes.memory[node_index];
			node_index = metadata.parent;
			if metadata.index_in_parent == 0
			{
				child_index = 1;
				break;
			}
		}
	}
}

tree_volume_traverse :: proc "contextless" (
	tree: ^Tree, query: util.Bounding_Box, leaf_proc: Tree_Volume_Leaf_Proc, user_context: rawptr,
	pool: ^util.Buffer_Pool = nil,
) -> Physics_Status
{
	if tree == nil || tree.state != .Ready || leaf_proc == nil ||
		query.min.x > query.max.x || query.min.y > query.max.y || query.min.z > query.max.z
	{
		return .Invalid_Argument;
	}
	if tree.leaf_count == 0
	{
		return .Ok;
	}
	if tree.leaf_count == 1
	{
		if tree_bounds_intersect(tree_child_bounds(tree.nodes.memory[0].a), query) == .Present
		{
			return leaf_proc(user_context, 0);
		}
		return .Ok;
	}

	stack: [TREE_TRAVERSAL_STACK_CAPACITY]i32;
	pooled_stack: util.Buffer(i32);
	defer if pooled_stack.memory != nil
	{
		tree_traversal_stack_return(pool, &pooled_stack);
	}
	stack_count := 0;
	node_index := i32(0);
	for
	{
		if node_index < 0
		{
			status := leaf_proc(user_context, tree_decode_leaf(node_index));
			if status != .Ok
			{
				return status;
			}
			if stack_count == 0
			{
				break;
			}
			stack_count -= 1;
			if pooled_stack.memory != nil
			{
				node_index = pooled_stack.memory[stack_count];
			}
			else
			{
				node_index = stack[stack_count];
			}
			continue;
		}
		node := &tree.nodes.memory[node_index];
		hit_a := tree_bounds_intersect(tree_child_bounds(node.a), query);
		hit_b := tree_bounds_intersect(tree_child_bounds(node.b), query);
		if hit_a == .Present
		{
			node_index = node.a.index;
			if hit_b == .Present
			{
				if pooled_stack.memory == nil && stack_count < len(stack)
				{
					stack[stack_count] = node.b.index;
					stack_count += 1;
				}
				else
				{
					status := tree_traversal_stack_push(
						&stack, &pooled_stack, &stack_count, node.b.index, pool,
					);
					if status != .Ok
					{
						return status;
					}
				}
			}
		}
		else if hit_b == .Present
		{
			node_index = node.b.index;
		}
		else
		{
			if stack_count == 0
			{
				break;
			}
			stack_count -= 1;
			if pooled_stack.memory != nil
			{
				node_index = pooled_stack.memory[stack_count];
			}
			else
			{
				node_index = stack[stack_count];
			}
		}
	}
	return .Ok;
}
