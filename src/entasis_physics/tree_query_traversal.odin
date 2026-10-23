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

// double-coordinate visitors may reduce the ray limit without narrowing it
// local shape visitors retain Tree_Ray_Leaf_Proc and its binary32 limit contract
when util.PRECISION_MODE == .Double_Coordinates
{
	Tree_Global_Ray_Leaf_Proc :: #type proc "contextless" (
		user_context: rawptr, leaf_index: int, maximum_t: ^f64,
	) -> Physics_Status;
}

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
	minimum_t: Tree_Scalar,
}

#assert(size_of(Tree_Closest_Traversal_Entry) == 2 * size_of(Tree_Scalar));

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
	maximum_t: Tree_Scalar,
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

// a slab parameter uses one subtraction, one rounded reciprocal and one
// multiplication. with a normal reciprocal, four representable steps enclose
// their accumulated round-to-nearest error (including underflow). apply the
// monotone enclosure after the XYZ fold, not once per axis. this is an arithmetic
// error bound, not a world-space epsilon or a change to shape-hit tolerances
// rare nonnormal reciprocals use division below. persistent tree layouts do not change
@(private="package")
tree_ray_parameter_lower :: #force_inline proc "contextless" (
	value: Tree_Scalar, $expected_sign: Tree_Bits,
) -> Tree_Scalar
{
	SIGN :: Tree_Bits(TREE_SIGN_BIT);
	NEGATIVE_INFINITY :: Tree_Bits(TREE_NEGATIVE_INFINITY);
	bits := transmute(Tree_Bits)value;
	// this expectation guides block layout while retaining every sign case
	if intrinsics.expect(bits & SIGN != 0, expected_sign != 0)
	{
		bits = min(bits, NEGATIVE_INFINITY - 4) + 4;
	}
	else if intrinsics.expect(bits >= 4, true)
	{
		bits -= 4;
	}
	else
	{
		bits = SIGN | (4 - bits);
	}
	return transmute(Tree_Scalar)bits;
}

// preserve finite parameters when subtraction overflows. power-of-two scaling
// is exact for these large opposite-sign endpoints. dividing by a tiny component
// may still overflow, but then the parameter is outside every finite query limit
@(private="package")
tree_ray_parameter_rare :: #force_inline proc "contextless" (
	boundary, origin, component: Tree_Scalar, division: u8,
) -> Tree_Scalar
{
	delta := boundary - origin;
	if abs(delta) <= max(Tree_Scalar)
	{
		if division != 0
		{
			return delta / component;
		}
		return delta * component;
	}
	half_delta := boundary * 0.5 - origin * 0.5;
	if division != 0
	{
		return half_delta / (component * 0.5);
	}
	return (half_delta * component) * 2;
}

Tree_Ray_Traversal_Data :: struct
{
	origin:            Tree_Position,
	inverse_direction: Tree_Position,
	parallel_mask:     u8,
}

@(private="file")
tree_ray_direction_axis :: #force_inline proc "contextless" (direction: Tree_Scalar) -> (Tree_Scalar, u8)
{
	if direction == 0
	{
		return 0, 1;
	}
	if abs(direction) < TREE_MINIMUM_NORMAL || abs(direction) > 1/Tree_Scalar(TREE_MINIMUM_NORMAL)
	{
		return direction, 8;
	}
	return 1/direction, 0;
}

tree_ray_traversal_data :: #force_inline proc "contextless" (
	ray: Tree_Coordinate_Ray,
) -> Tree_Ray_Traversal_Data
{
	tree_validate_position(ray.origin);
	tree_validate_position(ray.direction);
	tree_validate_scalar(ray.maximum_t);
	result: Tree_Ray_Traversal_Data = {origin=ray.origin};
	x_mask, y_mask, z_mask: u8;
	result.inverse_direction.x, x_mask = tree_ray_direction_axis(ray.direction.x);
	result.inverse_direction.y, y_mask = tree_ray_direction_axis(ray.direction.y);
	result.inverse_direction.z, z_mask = tree_ray_direction_axis(ray.direction.z);
	result.parallel_mask = x_mask | (y_mask << 1) | (z_mask << 2);
	// the range bit proves ordinary finite subtraction safe once per ray
	SAFE_ORIGIN :: max(Tree_Scalar)/TREE_ORIGIN_MARGIN;
	if abs(ray.origin.x) > SAFE_ORIGIN || abs(ray.origin.y) > SAFE_ORIGIN || abs(ray.origin.z) > SAFE_ORIGIN
	{
		result.parallel_mask |= 64;
	}
	return result;
}

tree_ray_intersection_prepared :: #force_inline proc "contextless" (
	bounds: Tree_Bounds,
	ray: Tree_Ray_Traversal_Data,
	maximum_t: Tree_Scalar,
) -> (t: Tree_Scalar, state: Reference_State)
{
	tree_validate_bounds(bounds);
	tree_validate_position(ray.origin);
	tree_validate_scalar(ray.inverse_direction.x);
	tree_validate_scalar(ray.inverse_direction.y);
	tree_validate_scalar(ray.inverse_direction.z);
	tree_validate_scalar(maximum_t);
	t_min := Tree_Scalar(0);
	t_max := max(Tree_Scalar);

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
		if ray.parallel_mask & 8 != 0 ||
			abs(bounds.min.x - ray.origin.x) > max(Tree_Scalar) ||
			abs(bounds.max.x - ray.origin.x) > max(Tree_Scalar)
		{
			entry = tree_ray_parameter_rare(bounds.min.x, ray.origin.x, ray.inverse_direction.x, ray.parallel_mask & 8);
			exit = tree_ray_parameter_rare(bounds.max.x, ray.origin.x, ray.inverse_direction.x, ray.parallel_mask & 8);
		}
		if entry > exit
		{
			entry, exit = exit, entry;
		}
		t_min = tree_ordered_max(t_min, entry);
		t_max = tree_ordered_min(t_max, exit);
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
		if ray.parallel_mask & 16 != 0 ||
			abs(bounds.min.y - ray.origin.y) > max(Tree_Scalar) ||
			abs(bounds.max.y - ray.origin.y) > max(Tree_Scalar)
		{
			entry = tree_ray_parameter_rare(bounds.min.y, ray.origin.y, ray.inverse_direction.y, ray.parallel_mask & 16);
			exit = tree_ray_parameter_rare(bounds.max.y, ray.origin.y, ray.inverse_direction.y, ray.parallel_mask & 16);
		}
		if entry > exit
		{
			entry, exit = exit, entry;
		}
		t_min = tree_ordered_max(t_min, entry);
		t_max = tree_ordered_min(t_max, exit);
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
		if ray.parallel_mask & 32 != 0 ||
			abs(bounds.min.z - ray.origin.z) > max(Tree_Scalar) ||
			abs(bounds.max.z - ray.origin.z) > max(Tree_Scalar)
		{
			entry = tree_ray_parameter_rare(bounds.min.z, ray.origin.z, ray.inverse_direction.z, ray.parallel_mask & 32);
			exit = tree_ray_parameter_rare(bounds.max.z, ray.origin.z, ray.inverse_direction.z, ray.parallel_mask & 32);
		}
		if entry > exit
		{
			entry, exit = exit, entry;
		}
		t_min = tree_ordered_max(t_min, entry);
		t_max = tree_ordered_min(t_max, exit);
	}
	t_min = tree_ordered_max(Tree_Scalar(0), tree_ray_parameter_lower(t_min, 0));
	t_max = tree_ordered_min(maximum_t, -tree_ray_parameter_lower(-t_max, Tree_Bits(TREE_SIGN_BIT)));
	if t_min > t_max
	{
		return 0, .Missing;
	}
	return t_min, .Present;
}

// query-local XYZ lanes use 32-byte float32 or 64-byte mixed64 children. the fourth lane
// contains integer metadata in storage and is cleared before floating arithmetic
#assert(offset_of(Tree_Node_Child, max) == 4 * size_of(Tree_Scalar));
#assert(size_of(Tree_Node_Child) == 8 * size_of(Tree_Scalar));
#assert(offset_of(Tree_Node_Child, max) + size_of(#simd[4]Tree_Scalar) <= size_of(Tree_Node_Child));

@(private="file")
Tree_Ray_Arithmetic :: enum u8
{
	Ordinary,
	Exceptional,
}

@(private="file")
Tree_Query_Ray_Packed :: struct
{
	origin: #simd[4]Tree_Scalar,
	exit_origin: #simd[4]Tree_Scalar,
	inverse: #simd[4]Tree_Scalar,
	parallel: #simd[4]Tree_Bits,
	parallel_exit: #simd[4]Tree_Scalar,
	division_mask: u8,
}

@(private="file")
tree_query_ray_division_mask :: #force_inline proc "contextless" (ray: Tree_Coordinate_Ray) -> u8
{
	direction := #simd[4]Tree_Scalar{ray.direction.x, ray.direction.y, ray.direction.z, 0};
	parallel := intrinsics.simd_lanes_eq(direction, #simd[4]Tree_Scalar{});
	// classify the existing scalar range rules without preparing an inverse
	ABS_MASK :: #simd[4]Tree_Bits{
		Tree_Bits(TREE_SIGN_BIT)-1, Tree_Bits(TREE_SIGN_BIT)-1,
		Tree_Bits(TREE_SIGN_BIT)-1, Tree_Bits(TREE_SIGN_BIT)-1,
	};
	abs_direction := transmute(#simd[4]Tree_Scalar)(transmute(#simd[4]Tree_Bits)direction & ABS_MASK);
	exceptional := (intrinsics.simd_lanes_lt(abs_direction,
		#simd[4]Tree_Scalar{TREE_MINIMUM_NORMAL, TREE_MINIMUM_NORMAL, TREE_MINIMUM_NORMAL, TREE_MINIMUM_NORMAL}) |
		intrinsics.simd_lanes_gt(abs_direction,
		#simd[4]Tree_Scalar{1/Tree_Scalar(TREE_MINIMUM_NORMAL), 1/Tree_Scalar(TREE_MINIMUM_NORMAL), 1/Tree_Scalar(TREE_MINIMUM_NORMAL), 1/Tree_Scalar(TREE_MINIMUM_NORMAL)})) & ~parallel;
	division_mask: u8 = transmute(u8)intrinsics.simd_extract_msbs(exceptional);
	SAFE_ORIGIN :: max(Tree_Scalar)/TREE_ORIGIN_MARGIN;
	origin := #simd[4]Tree_Scalar{ray.origin.x, ray.origin.y, ray.origin.z, 0};
	abs_origin := transmute(#simd[4]Tree_Scalar)(transmute(#simd[4]Tree_Bits)origin & ABS_MASK);
	if intrinsics.simd_extract_msbs(intrinsics.simd_lanes_gt(abs_origin,
		#simd[4]Tree_Scalar{SAFE_ORIGIN, SAFE_ORIGIN, SAFE_ORIGIN, SAFE_ORIGIN})) != {}
	{
		division_mask |= 8;
	}
	return division_mask;
}

@(private="file")
tree_query_ray_packed :: #force_inline proc "contextless" (
	ray: Tree_Coordinate_Ray, division_mask: u8,
) -> Tree_Query_Ray_Packed
{
	tree_validate_position(ray.origin);
	tree_validate_position(ray.direction);
	tree_validate_scalar(ray.maximum_t);
	direction := #simd[4]Tree_Scalar{ray.direction.x, ray.direction.y, ray.direction.z, 0};
	parallel := intrinsics.simd_lanes_eq(direction, #simd[4]Tree_Scalar{});
	x, y, z: Tree_Scalar;
	x, _ = tree_ray_direction_axis(ray.direction.x);
	y, _ = tree_ray_direction_axis(ray.direction.y);
	z, _ = tree_ray_direction_axis(ray.direction.z);
	inverse: #simd[4]Tree_Scalar = {x, y, z, 0};
	origin := #simd[4]Tree_Scalar{ray.origin.x, ray.origin.y, ray.origin.z, 0};
	tree_validate_scalar(intrinsics.simd_extract(inverse, 0));
	tree_validate_scalar(intrinsics.simd_extract(inverse, 1));
	tree_validate_scalar(intrinsics.simd_extract(inverse, 2));
	// parallel axes must not truncate a finite binary64 ray interval
	limit := max(Tree_Scalar);
	return {
		origin=origin,
		exit_origin=origin,
		inverse=inverse,
		division_mask=division_mask,
		parallel=parallel,
		parallel_exit=intrinsics.simd_select(parallel,
			#simd[4]Tree_Scalar{limit, limit, limit, limit},
			#simd[4]Tree_Scalar{-limit, -limit, -limit, -limit}),
	};
}

// sweep the authored endpoints directly: entry clips target.min - moving.max,
// exit clips target.max - moving.min. rays use the same origin for both. a
// rounded midpoint/radius can delay a hit even for normal finite coordinates
// keep both endpoints until subtraction, the existing rare parameter path then
// also retains finite results when their separation overflows the scalar type
@(private="file")
tree_query_sweep_packed :: #force_inline proc "contextless" (
	bounds: Tree_Bounds, direction: Tree_Position, maximum_t: Tree_Scalar,
) -> Tree_Query_Ray_Packed
{
	tree_validate_bounds(bounds);
	ray := Tree_Coordinate_Ray{origin=bounds.max, direction=direction, maximum_t=maximum_t};
	result := tree_query_ray_packed(ray, tree_query_ray_division_mask(ray));
	result.exit_origin = {bounds.min.x, bounds.min.y, bounds.min.z, 0};
	// the ordinary per-node path is safe only when both subtraction origins
	// meet the same once-per-query range proof as a point ray
	SAFE_ORIGIN :: max(Tree_Scalar) / TREE_ORIGIN_MARGIN;
	if abs(bounds.min.x) > SAFE_ORIGIN || abs(bounds.min.y) > SAFE_ORIGIN || abs(bounds.min.z) > SAFE_ORIGIN
	{
		result.division_mask |= 8;
	}
	return result;
}

@(private="file")
tree_ray_intersection_packed :: #force_inline proc "contextless" (
	minimum, maximum: #simd[4]Tree_Scalar, ray: Tree_Query_Ray_Packed, maximum_t: Tree_Scalar,
	$arithmetic: Tree_Ray_Arithmetic,
) -> (Tree_Scalar, Reference_State)
{
	tree_validate_scalar(maximum_t);
	outside := (intrinsics.simd_lanes_lt(ray.origin, minimum) |
		intrinsics.simd_lanes_gt(ray.exit_origin, maximum)) & ray.parallel;
	if intrinsics.simd_extract_msbs(outside) != {}
	{
		return 0, .Missing;
	}
	// parallel lanes must not participate in the interval, including finite
	// extreme inputs whose subtraction overflows before multiplication by zero
	delta_a := minimum - ray.origin;
	delta_b := maximum - ray.exit_origin;
	a := delta_a * ray.inverse;
	b := delta_b * ray.inverse;
	// low three bits request direct division, bit 3 means endpoint
	// subtraction might overflow. with neither, every finite node is safe
	when arithmetic == .Exceptional
	{
		if ray.division_mask != 0
		{
			finite_limit := #simd[4]Tree_Scalar{max(Tree_Scalar), max(Tree_Scalar), max(Tree_Scalar), max(Tree_Scalar)};
			overflow := intrinsics.simd_lanes_gt(delta_a, finite_limit) |
				intrinsics.simd_lanes_lt(delta_a, -finite_limit) |
				intrinsics.simd_lanes_gt(delta_b, finite_limit) |
				intrinsics.simd_lanes_lt(delta_b, -finite_limit);
			if ray.division_mask != 0 || intrinsics.simd_extract_msbs(overflow) != {}
			{
				for axis in 0 ..< 3
				{
					division := ray.division_mask & (u8(1) << u8(axis));
					if division != 0 || abs(intrinsics.simd_extract(delta_a, axis)) > max(Tree_Scalar) ||
						abs(intrinsics.simd_extract(delta_b, axis)) > max(Tree_Scalar)
					{
						direction: Tree_Scalar = intrinsics.simd_extract(ray.inverse, axis);
						origin: Tree_Scalar = intrinsics.simd_extract(ray.origin, axis);
						a = intrinsics.simd_replace(a, axis, tree_ray_parameter_rare(intrinsics.simd_extract(minimum, axis), origin, direction, division));
						b = intrinsics.simd_replace(b, axis, tree_ray_parameter_rare(intrinsics.simd_extract(maximum, axis), intrinsics.simd_extract(ray.exit_origin, axis), direction, division));
					}
				}
			}
		}
	}
	entry := intrinsics.simd_select(intrinsics.simd_lanes_lt(a, b), a, b);
	exit := intrinsics.simd_select(intrinsics.simd_lanes_gt(a, b), a, b);
	entry = transmute(#simd[4]Tree_Scalar)((transmute(#simd[4]Tree_Bits)entry) & ~ray.parallel);
	exit = intrinsics.simd_select(intrinsics.simd_lanes_gt(exit, ray.parallel_exit), exit, ray.parallel_exit);
	// retain the scalar X/Y/Z fold and inclusive boundary semantics. do not
	// reassociate with a horizontal reduction or use approximate reciprocals
	t_min := tree_ordered_max(Tree_Scalar(0), intrinsics.simd_extract(entry, 0));
	t_min = tree_ordered_max(t_min, intrinsics.simd_extract(entry, 1));
	t_min = tree_ordered_max(t_min, intrinsics.simd_extract(entry, 2));
	t_max := tree_ordered_min(max(Tree_Scalar), intrinsics.simd_extract(exit, 0));
	t_max = tree_ordered_min(t_max, intrinsics.simd_extract(exit, 1));
	t_max = tree_ordered_min(t_max, intrinsics.simd_extract(exit, 2));
	t_min = tree_ordered_max(Tree_Scalar(0), tree_ray_parameter_lower(t_min, 0));
	t_max = tree_ordered_min(maximum_t, -tree_ray_parameter_lower(-t_max, Tree_Bits(TREE_SIGN_BIT)));
	if t_min > t_max
	{
		return 0, .Missing;
	}
	return t_min, .Present;
}

@(private="file")
tree_ray_child_intersection_packed :: #force_inline proc "contextless" (
	child: ^Tree_Node_Child, ray: Tree_Query_Ray_Packed, maximum_t: Tree_Scalar,
	$arithmetic: Tree_Ray_Arithmetic,
) -> (Tree_Scalar, Reference_State)
{
	XYZ :: #simd[4]Tree_Bits{max(Tree_Bits), max(Tree_Bits), max(Tree_Bits), 0};
	minimum := transmute(#simd[4]Tree_Scalar)((transmute(#simd[4]Tree_Bits)
		intrinsics.unaligned_load((^#simd[4]Tree_Scalar)(&child.min))) & XYZ);
	maximum := transmute(#simd[4]Tree_Scalar)((transmute(#simd[4]Tree_Bits)
		intrinsics.unaligned_load((^#simd[4]Tree_Scalar)(&child.max))) & XYZ);
	tree_validate_bounds({
		min={intrinsics.simd_extract(minimum, 0), intrinsics.simd_extract(minimum, 1), intrinsics.simd_extract(minimum, 2)},
		max={intrinsics.simd_extract(maximum, 0), intrinsics.simd_extract(maximum, 1), intrinsics.simd_extract(maximum, 2)},
	});
	return tree_ray_intersection_packed(minimum, maximum, ray, maximum_t, arithmetic);
}

tree_ray_traverse_coordinates :: proc "contextless" (
	tree: ^Tree, ray: Tree_Coordinate_Ray, leaf_proc: $Leaf_Proc, user_context: rawptr,
	pool: ^util.Buffer_Pool = nil,
) -> (Tree_Scalar, Physics_Status)
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
	if tree_query_ray_division_mask(ray) == 0
	{
		return tree_ray_traverse_prepared(.Ordinary, tree, ray, leaf_proc, user_context, pool);
	}
	return tree_ray_traverse_prepared(.Exceptional, tree, ray, leaf_proc, user_context, pool);
}

@(private="file")
tree_ray_traverse_prepared :: #force_no_inline proc "contextless" (
	$arithmetic: Tree_Ray_Arithmetic,
	tree: ^Tree, ray: Tree_Coordinate_Ray,
	leaf_proc: $Leaf_Proc, user_context: rawptr, pool: ^util.Buffer_Pool,
) -> (Tree_Scalar, Physics_Status)
{
	// prepare inside the selected body so the inverse and masks never escape
	prepared_ray: Tree_Query_Ray_Packed;
	when arithmetic == .Ordinary
	{
		prepared_ray = tree_query_ray_packed(ray, 0);
	}
	else
	{
		prepared_ray = tree_query_ray_packed(ray, tree_query_ray_division_mask(ray));
	}
	maximum_t: Tree_Scalar = ray.maximum_t;
	if tree.leaf_count == 1
	{
		_, hit := tree_ray_child_intersection_packed(&tree.nodes.memory[0].a, prepared_ray, maximum_t, arithmetic);
		if hit == .Present
		{
			status := tree_visit_ray_leaf(leaf_proc, user_context, 0, &maximum_t);
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
			status := tree_visit_ray_leaf(leaf_proc, user_context, tree_decode_leaf(node_index), &maximum_t);
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
		t_a, hit_a := tree_ray_child_intersection_packed(&node.a, prepared_ray, maximum_t, arithmetic);
		t_b, hit_b := tree_ray_child_intersection_packed(&node.b, prepared_ray, maximum_t, arithmetic);
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
	tree: ^Tree, subtree_root: i32, ray: Tree_Coordinate_Ray, maximum_t: ^Tree_Scalar,
	leaf_proc: $Leaf_Proc, user_context: rawptr,
) -> Physics_Status
{
	if subtree_root < 0
	{
		return tree_visit_ray_leaf(leaf_proc, user_context, tree_decode_leaf(subtree_root), maximum_t);
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
				status := tree_visit_ray_leaf(leaf_proc, user_context, tree_decode_leaf(child.index), maximum_t);
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

tree_ray_traverse_closest_coordinates :: proc "contextless" (
	tree: ^Tree, ray: Tree_Coordinate_Ray, leaf_proc: $Leaf_Proc, user_context: rawptr,
	pool: ^util.Buffer_Pool = nil,
) -> (Tree_Scalar, Physics_Status)
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
	if tree_query_ray_division_mask(ray) == 0
	{
		return tree_ray_traverse_closest_prepared(.Ordinary, tree, ray, leaf_proc, user_context, pool);
	}
	return tree_ray_traverse_closest_prepared(.Exceptional, tree, ray, leaf_proc, user_context, pool);
}

@(private="file")
tree_ray_traverse_closest_prepared :: #force_no_inline proc "contextless" (
	$arithmetic: Tree_Ray_Arithmetic,
	tree: ^Tree, ray: Tree_Coordinate_Ray,
	leaf_proc: $Leaf_Proc, user_context: rawptr, pool: ^util.Buffer_Pool,
) -> (Tree_Scalar, Physics_Status)
{
	// prepare inside the selected body so the inverse and masks never escape
	prepared_ray: Tree_Query_Ray_Packed;
	when arithmetic == .Ordinary
	{
		prepared_ray = tree_query_ray_packed(ray, 0);
	}
	else
	{
		prepared_ray = tree_query_ray_packed(ray, tree_query_ray_division_mask(ray));
	}
	maximum_t: Tree_Scalar = ray.maximum_t;
	if tree.leaf_count == 1
	{
		_, hit := tree_ray_child_intersection_packed(&tree.nodes.memory[0].a, prepared_ray, maximum_t, arithmetic);
		if hit == .Present
		{
			status := tree_visit_ray_leaf(leaf_proc, user_context, 0, &maximum_t);
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
			status := tree_visit_ray_leaf(leaf_proc, user_context, tree_decode_leaf(node_index), &maximum_t);
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
		t_a, hit_a := tree_ray_child_intersection_packed(&node.a, prepared_ray, maximum_t, arithmetic);
		t_b, hit_b := tree_ray_child_intersection_packed(&node.b, prepared_ray, maximum_t, arithmetic);
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

tree_sweep_traverse_coordinates :: proc "contextless" (
	tree: ^Tree, bounds: Tree_Bounds, direction: Tree_Position, maximum_t: Tree_Scalar,
	leaf_proc: $Leaf_Proc, user_context: rawptr,
	pool: ^util.Buffer_Pool = nil,
) -> (Tree_Scalar, Physics_Status)
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
	prepared_ray := tree_query_sweep_packed(bounds, direction, maximum_t);
	if tree.leaf_count == 1
	{
		_, hit := tree_ray_child_intersection_packed(
			&tree.nodes.memory[0].a, prepared_ray, current_maximum, .Exceptional,
		);
		if hit == .Present
		{
			status := tree_visit_ray_leaf(leaf_proc, user_context, 0, &current_maximum);
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
			status := tree_visit_ray_leaf(leaf_proc, user_context, tree_decode_leaf(node_index), &current_maximum);
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
		t_a, hit_a := tree_ray_child_intersection_packed(&node.a, prepared_ray, current_maximum, .Exceptional);
		t_b, hit_b := tree_ray_child_intersection_packed(&node.b, prepared_ray, current_maximum, .Exceptional);
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

tree_sweep_traverse_closest_coordinates :: proc "contextless" (
	tree: ^Tree, bounds: Tree_Bounds, direction: Tree_Position, maximum_t: Tree_Scalar,
	leaf_proc: $Leaf_Proc, user_context: rawptr,
	pool: ^util.Buffer_Pool = nil,
) -> (Tree_Scalar, Physics_Status)
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
	prepared_ray := tree_query_sweep_packed(bounds, direction, maximum_t);
	if tree.leaf_count == 1
	{
		_, hit := tree_ray_child_intersection_packed(
			&tree.nodes.memory[0].a, prepared_ray, current_maximum, .Exceptional,
		);
		if hit == .Present
		{
			status := tree_visit_ray_leaf(leaf_proc, user_context, 0, &current_maximum);
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
			status := tree_visit_ray_leaf(leaf_proc, user_context, tree_decode_leaf(node_index), &current_maximum);
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
		t_a, hit_a := tree_ray_child_intersection_packed(&node.a, prepared_ray, current_maximum, .Exceptional);
		t_b, hit_b := tree_ray_child_intersection_packed(&node.b, prepared_ray, current_maximum, .Exceptional);
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
	tree: ^Tree, query: Tree_Bounds,
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

	query_minimum: #simd[4]Tree_Scalar = {query.min.x, query.min.y, query.min.z, 0};
	query_maximum: #simd[4]Tree_Scalar = {query.max.x, query.max.y, query.max.z, 0};
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
		minimum: #simd[4]Tree_Scalar = {child.min.x, child.min.y, child.min.z, 0};
		maximum: #simd[4]Tree_Scalar = {child.max.x, child.max.y, child.max.z, 0};
		intersection: #simd[4]Tree_Bits = intrinsics.simd_lanes_ge(maximum, query_minimum) &
			intrinsics.simd_lanes_le(minimum, query_maximum);
		if transmute(u8)intrinsics.simd_extract_msbs(intersection) == 15
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
	tree: ^Tree, query: Tree_Bounds, leaf_proc: Tree_Volume_Leaf_Proc, user_context: rawptr,
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
