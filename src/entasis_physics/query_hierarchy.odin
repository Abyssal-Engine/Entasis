package entasis_physics

import util "entasis:entasis_utilities"
import "core:math"

// an admitted immutable hierarchy has bounded depth. only selected hierarchical
// queries use these stack frames. flat queries retain their existing traversal
Query_Hierarchy_Frame :: struct
{
	root: Distance_Query_Root,
	cursor: Distance_Query_Cursor,
	part: i32,
	bounds: util.Bounding_Box,
}

Query_Hierarchy_Cursor :: struct
{
	frames: [MAXIMUM_COMPOUND_HIERARCHY_DEPTH + 2]Query_Hierarchy_Frame,
	count: int,
	// borrowed bounds remain fixed and alive until this cursor is exhausted
	world_bounds: ^util.Bounding_Box,
}

Query_Hierarchy_Leaf :: struct
{
	view: Collision_Shape_View,
	type_id: int,
	part: i32,
	mesh: Reference_State,
}

query_hierarchy_initialize :: proc "contextless" (
	cursor: ^Query_Hierarchy_Cursor, shape: rawptr, type_id: int, pose: Rigid_Pose, shapes: ^Shape_Registry,
	world_bounds: ^util.Bounding_Box = nil,
) -> Physics_Status
{
	root: Distance_Query_Root;
	status: Physics_Status;
	root, status = distance_query_root(shape, type_id, pose, shapes);
	if status != .Ok
	{
		return status;
	}
	cursor.count = 1;
	cursor.frames[0] = {root=root, part=-1};
	cursor.world_bounds = world_bounds;
	if world_bounds != nil
	{
		cursor.frames[0].bounds = query_hierarchy_local_bounds(world_bounds^, root.view.pose);
	}
	return .Ok;
}

@(private="file")
query_hierarchy_interval_add :: proc "contextless" (a, b: [2]f64) -> [2]f64
{
	lower: f64 = a[0]+b[0];
	upper: f64 = a[1]+b[1];
	if math.is_nan(lower)
	{
		lower = math.inf_f64(-1);
	}
	if math.is_nan(upper)
	{
		upper = math.inf_f64(1);
	}
	return {math.nextafter_f64(lower, math.inf_f64(-1)), math.nextafter_f64(upper, math.inf_f64(1))};
}

@(private="file")
query_hierarchy_interval_subtract :: proc "contextless" (a, b: [2]f64) -> [2]f64
{
	lower: f64 = a[0]-b[1];
	upper: f64 = a[1]-b[0];
	if math.is_nan(lower)
	{
		lower = math.inf_f64(-1);
	}
	if math.is_nan(upper)
	{
		upper = math.inf_f64(1);
	}
	return {math.nextafter_f64(lower, math.inf_f64(-1)), math.nextafter_f64(upper, math.inf_f64(1))};
}

@(private="file")
query_hierarchy_interval_multiply :: proc "contextless" (a, b: [2]f64) -> [2]f64
{
	if (a[0] == 0 && a[1] == 0) || (b[0] == 0 && b[1] == 0)
	{
		return {};
	}
	lower: f64 = math.inf_f64(1);
	upper: f64 = math.inf_f64(-1);
	for x in a
	{
		for y in b
		{
			product: f64;
			if x != 0 && y != 0
			{
				product = x*y;
			}
			lower = min(lower, product);
			upper = max(upper, product);
		}
	}
	return {math.nextafter_f64(lower, math.inf_f64(-1)), math.nextafter_f64(upper, math.inf_f64(1))};
}

@(private="file")
query_hierarchy_interval_narrow :: proc "contextless" (value: [2]f64) -> [2]f32
{
	lower: f32 = f32(value[0]);
	upper: f32 = f32(value[1]);
	if f64(lower) > value[0]
	{
		lower = math.nextafter_f32(lower, math.inf_f32(-1));
	}
	if f64(upper) < value[1]
	{
		upper = math.nextafter_f32(upper, math.inf_f32(1));
	}
	return {lower, upper};
}

query_hierarchy_local_bounds :: proc "contextless" (bounds: util.Bounding_Box, pose: Rigid_Pose) -> util.Bounding_Box
{
	minimum: [3]f32 = {bounds.min.x, bounds.min.y, bounds.min.z};
	maximum: [3]f32 = {bounds.max.x, bounds.max.y, bounds.max.z};
	position: [3]f32 = {pose.position.x, pose.position.y, pose.position.z};
	v: [3][2]f64;
	for axis in 0..<3
	{
		endpoints: [2]f64 = {f64(minimum[axis]), f64(maximum[axis])};
		translation: [2]f64 = {f64(position[axis]), f64(position[axis])};
		// earlier pruning arithmetic can already have overflowed this axis
		if !(abs(minimum[axis]) <= math.F32_MAX) || !(abs(maximum[axis]) <= math.F32_MAX)
		{
			endpoints = {math.inf_f64(-1), math.inf_f64(1)};
		}
		if !(abs(position[axis]) <= math.F32_MAX)
		{
			translation = {math.inf_f64(-1), math.inf_f64(1)};
		}
		v[axis] = query_hierarchy_interval_subtract(endpoints, translation);
	}
	q: [3][2]f64 = {
		{-f64(pose.orientation.x), -f64(pose.orientation.x)},
		{-f64(pose.orientation.y), -f64(pose.orientation.y)},
		{-f64(pose.orientation.z), -f64(pose.orientation.z)},
	};
	c, d: [3][2]f64;
	for axis in 0..<3
	{
		a: int = (axis+1)%3;
		b: int = (axis+2)%3;
		c[axis] = query_hierarchy_interval_subtract(
			query_hierarchy_interval_multiply(q[a], v[b]), query_hierarchy_interval_multiply(q[b], v[a]));
	}
	for axis in 0..<3
	{
		a: int = (axis+1)%3;
		b: int = (axis+2)%3;
		d[axis] = query_hierarchy_interval_subtract(
			query_hierarchy_interval_multiply(q[a], c[b]), query_hierarchy_interval_multiply(q[b], c[a]));
	}
	twice_w: f64 = 2*f64(pose.orientation.w);
	local: [3][2]f32;
	for axis in 0..<3
	{
		local[axis] = query_hierarchy_interval_narrow(query_hierarchy_interval_add(
			query_hierarchy_interval_add(v[axis], query_hierarchy_interval_multiply({twice_w, twice_w}, c[axis])),
			query_hierarchy_interval_multiply({2, 2}, d[axis])));
	}
	return {min={local[0][0], local[1][0], local[2][0]}, max={local[0][1], local[1][1], local[2][1]}};
}

query_hierarchy_next :: proc "contextless" (
	cursor: ^Query_Hierarchy_Cursor, triangle: ^Triangle, shapes: ^Shape_Registry,
	limit: f32 = math.F32_MAX,
) -> (Query_Hierarchy_Leaf, Reference_State, Physics_Status)
{
	for cursor.count > 0
	{
		frame: ^Query_Hierarchy_Frame = &cursor.frames[cursor.count-1];
		index: int;
		found: Reference_State;
		index, found = distance_query_next_leaf(frame.root, &frame.cursor, frame.bounds, limit);
		if found == .Missing
		{
			cursor.count -= 1;
			continue;
		}
		part: i32 = frame.part;
		if cursor.count == 1
		{
			part = distance_query_child_id(frame.root, index);
		}
		if frame.root.view.batch.metadata.batch_type == .Convex
		{
			return {view=frame.root.view, type_id=frame.root.type_id, part=part}, .Present, .Ok;
		}
		if frame.root.type_id == MESH_TYPE_ID
		{
			view: Collision_Shape_View;
			type_id: int;
			status: Physics_Status;
			view, type_id, status = distance_query_leaf(frame.root, index, triangle, shapes);
			if status != .Ok
			{
				return {}, .Missing, status;
			}
			return {view=view, type_id=type_id, part=part, mesh=.Present}, .Present, .Ok;
		}
		child: Collision_Child;
		status: Physics_Status;
		_, child, status = shape_hierarchy_child(shapes, frame.root.view.shape, frame.root.type_id, index, frame.root.view.pose);
		if status != .Ok
		{
			return {}, .Missing, status;
		}
		root: Distance_Query_Root;
		root, status = distance_query_root(child.shape, child.type_id, child.pose, shapes);
		if status != .Ok
		{
			return {}, .Missing, status;
		}
		if cursor.count >= len(cursor.frames)
		{
			return {}, .Missing, .Capacity_Missing;
		}
		cursor.frames[cursor.count] = {root=root, part=part};
		if cursor.world_bounds != nil
		{
			cursor.frames[cursor.count].bounds = query_hierarchy_local_bounds(cursor.world_bounds^, root.view.pose);
		}
		cursor.count += 1;
	}
	return {}, .Missing, .Ok;
}

query_hierarchy_distance :: #force_no_inline proc "contextless" (
	shape_a, shape_b: rawptr, type_a, type_b: int, pose_a, pose_b: Rigid_Pose,
	shapes: ^Shape_Registry, settings: Distance_Query_Settings,
) -> (Distance_Query_Pair_Result, Physics_Status)
{
	a, b: Query_Hierarchy_Cursor;
	status: Physics_Status = query_hierarchy_initialize(&a, shape_a, type_a, pose_a, shapes);
	if status != .Ok
	{
		return {}, status;
	}
	best: Distance_Query_Pair_Result;
	for
	{
		triangle_a: Triangle;
		leaf_a: Query_Hierarchy_Leaf;
		found: Reference_State;
		leaf_a, found, status = query_hierarchy_next(&a, &triangle_a, shapes);
		if status != .Ok
		{
			return {}, status;
		}
		if found == .Missing
		{
			break;
		}
		bounds: util.Bounding_Box;
		bounds, status = distance_query_relative_bounds(leaf_a.view, rigid_pose_identity(), shapes);
		if status != .Ok
		{
			return {}, status;
		}
		status = query_hierarchy_initialize(&b, shape_b, type_b, pose_b, shapes, &bounds);
		if status != .Ok
		{
			return {}, status;
		}
		for
		{
			triangle_b: Triangle;
			leaf_b: Query_Hierarchy_Leaf;
			leaf_b, found, status = query_hierarchy_next(&b, &triangle_b, shapes, distance_query_limit(best, settings));
			if status != .Ok
			{
				return {}, status;
			}
			if found == .Missing
			{
				break;
			}
			result: Distance_Query_Result;
			result, status = distance_query_convex_admitted(leaf_a.view, leaf_b.view, leaf_a.type_id, leaf_b.type_id, shapes, settings, .Distance, {});
			if status != .Ok
			{
				return {}, status;
			}
			distance_query_select(&best, result, leaf_a.part, leaf_b.part);
		}
	}
	if best.geometry.state == .Unresolved
	{
		return {}, .No_Convergence;
	}
	return best, .Ok;
}

query_hierarchy_point :: #force_no_inline proc "contextless" (
	point: util.Vector3, shape: rawptr, type_id: int, pose: Rigid_Pose, shapes: ^Shape_Registry, settings: Distance_Query_Settings,
) -> (Distance_Query_Pair_Result, Physics_Status)
{
	bounds: util.Bounding_Box = {min=point, max=point};
	cursor: Query_Hierarchy_Cursor;
	status: Physics_Status = query_hierarchy_initialize(&cursor, shape, type_id, pose, shapes, &bounds);
	if status != .Ok
	{
		return {}, status;
	}
	best: Distance_Query_Pair_Result;
	for
	{
		triangle: Triangle;
		leaf: Query_Hierarchy_Leaf;
		found: Reference_State;
		leaf, found, status = query_hierarchy_next(&cursor, &triangle, shapes, distance_query_limit(best, settings));
		if status != .Ok
		{
			return {}, status;
		}
		if found == .Missing
		{
			break;
		}
		result: Distance_Query_Result;
		result, status = distance_query_point_admitted(point, &leaf.view, leaf.type_id, shapes, settings);
		if status != .Ok
		{
			return {}, status;
		}
		distance_query_select(&best, result, -1, leaf.part);
	}
	if best.geometry.state == .Unresolved
	{
		return {}, .No_Convergence;
	}
	return best, .Ok;
}

query_hierarchy_depenetrate :: #force_no_inline proc "contextless" (
	shape_a, shape_b: rawptr, type_a, type_b: int, pose_a, pose_b: Rigid_Pose,
	shapes: ^Shape_Registry, settings: Distance_Query_Settings, scratch: Distance_Query_Scratch, tasks: ^Collision_Task_Registry,
) -> (Distance_Query_Correction, Physics_Status)
{
	a: Collision_Shape_View;
	status: Physics_Status;
	a, status = distance_query_view(shape_a, type_a, pose_a, shapes);
	if status != .Ok
	{
		return {}, status;
	}
	translation: util.Vector3;
	for iteration in 0..=settings.maximum_iterations
	{
		bounds: util.Bounding_Box;
		bounds, status = distance_query_relative_bounds(a, rigid_pose_identity(), shapes);
		if status != .Ok
		{
			return {}, status;
		}
		cursor: Query_Hierarchy_Cursor;
		status = query_hierarchy_initialize(&cursor, shape_b, type_b, pose_b, shapes, &bounds);
		if status != .Ok
		{
			return {}, status;
		}
		deepest: Distance_Query_Result;
		final_state: Distance_Query_State = .Separated;
		for
		{
			triangle: Triangle;
			leaf: Query_Hierarchy_Leaf;
			found: Reference_State;
			leaf, found, status = query_hierarchy_next(&cursor, &triangle, shapes, settings.absolute_tolerance);
			if status != .Ok
			{
				return {}, status;
			}
			if found == .Missing
			{
				break;
			}
			if leaf.mesh == .Present
			{
				if tasks == nil
				{
					return {}, .Invalid_Argument;
				}
				manifold: Convex_Contact_Manifold;
				manifold, status = collision_task_registry_test_convex(tasks, type_a, leaf.type_id, a.shape, leaf.view.shape, a.pose, leaf.view.pose, 0, shapes);
				if status != .Ok
				{
					return {}, status;
				}
				accepted: Reference_State;
				for &contact in manifold.contacts[:manifold.count]
				{
					if contact.depth >= 0
					{
						accepted = .Present;
						break;
					}
				}
				if accepted == .Missing
				{
					continue;
				}
			}
			result: Distance_Query_Result;
			result, status = distance_query_convex_admitted(a, leaf.view, type_a, leaf.type_id, shapes, settings, .Penetration, scratch);
			if status != .Ok
			{
				return {}, status;
			}
			if result.state == .Touching
			{
				final_state = .Touching;
			}
			if result.state == .Penetrating && result.depth > deepest.depth
			{
				deepest = result;
			}
		}
		if deepest.state != .Penetrating
		{
			return {state=final_state, translation=translation, iterations=iteration}, .Ok;
		}
		if iteration == settings.maximum_iterations
		{
			return {}, .No_Convergence;
		}
		translation = util.vector3_add(translation, util.vector3_scale(deepest.normal, deepest.depth));
		position: util.Vector3 = util.vector3_add(pose_a.position, translation);
		if position == a.pose.position || !(abs(position.x) <= math.F32_MAX && abs(position.y) <= math.F32_MAX && abs(position.z) <= math.F32_MAX)
		{
			return {}, .No_Convergence;
		}
		a.pose.position = position;
	}
	return {}, .No_Convergence;
}

query_hierarchy_sweep :: #force_no_inline proc "contextless" (
	shape_a, shape_b: rawptr, type_a, type_b: int, pose_a, pose_b: Rigid_Pose, velocity_a, velocity_b: Body_Velocity,
	maximum_t, minimum_progression, convergence_threshold: f32, maximum_iterations: int,
	shapes: ^Shape_Registry, tasks: ^Collision_Task_Registry, filter: Collision_Child_Filter_Proc, user: rawptr,
) -> (Sweep_Result, Physics_Status)
{
	call: ^Sweep_Task_Call_Context = (^Sweep_Task_Call_Context)(user);
	a, b: Query_Hierarchy_Cursor;
	status: Physics_Status = query_hierarchy_initialize(&a, shape_a, type_a, pose_a, shapes);
	if status != .Ok
	{
		return {}, status;
	}
	best: Sweep_Result = {state=.Miss, t0=maximum_t, t1=maximum_t, child_a=-1, child_b=-1};
	for
	{
		triangle_a: Triangle;
		leaf_a: Query_Hierarchy_Leaf;
		found: Reference_State;
		leaf_a, found, status = query_hierarchy_next(&a, &triangle_a, shapes);
		if status != .Ok
		{
			return {}, status;
		}
		if found == .Missing
		{
			break;
		}
		child_a: Collision_Child = {shape=leaf_a.view.shape, type_id=leaf_a.type_id, pose=leaf_a.view.pose, child_index=int(leaf_a.part)};
		// bounds include both roots' angular motion and relative translation
		bounds: util.Bounding_Box;
		direction: util.Vector3;
		bounds, direction, status = sweep_child_bounds_in_compound_space(child_a, pose_a, velocity_a, pose_b, velocity_b, best.t1, shapes);
		if status != .Ok
		{
			return {}, status;
		}
		displacement: util.Vector3 = util.vector3_scale(direction, best.t1);
		bounds.min = util.vector3_add(bounds.min, {min(displacement.x, 0), min(displacement.y, 0), min(displacement.z, 0)});
		bounds.max = util.vector3_add(bounds.max, {max(displacement.x, 0), max(displacement.y, 0), max(displacement.z, 0)});
		bounds = query_hierarchy_local_bounds(bounds, rigid_pose_invert(pose_b));
		status = query_hierarchy_initialize(&b, shape_b, type_b, pose_b, shapes, &bounds);
		if status != .Ok
		{
			return {}, status;
		}
		for
		{
			triangle_b: Triangle;
			leaf_b: Query_Hierarchy_Leaf;
			leaf_b, found, status = query_hierarchy_next(&b, &triangle_b, shapes, 0);
			if status != .Ok
			{
				return {}, status;
			}
			if found == .Missing
			{
				break;
			}
			if filter != nil && filter(user, 0, max(0, leaf_a.part), max(0, leaf_b.part)) != .Allow
			{
				continue;
			}
			_, _, status = sweep_task_registry_lookup(call.registry, leaf_a.type_id, leaf_b.type_id);
			if status != .Ok
			{
				return {}, status;
			}
			child_b: Collision_Child = {shape=leaf_b.view.shape, type_id=leaf_b.type_id, pose=leaf_b.view.pose, child_index=int(leaf_b.part)};
			local_a: Rigid_Pose = rigid_pose_concatenate(child_a.pose, rigid_pose_invert(pose_a));
			local_b: Rigid_Pose = rigid_pose_concatenate(child_b.pose, rigid_pose_invert(pose_b));
			result: Sweep_Result;
			result, status = sweep_task_test_registered_child(call, child_a, child_b, pose_a, pose_b, local_a, local_b,
				velocity_a, velocity_b, best.t1, minimum_progression, convergence_threshold, maximum_iterations, shapes, tasks);
			if status != .Ok
			{
				return {}, status;
			}
			if result.state == .Hit && (best.state == .Miss || result.t1 < best.t1)
			{
				best = result;
				// the established tree-backed sweep route reports zero for its
				// convex a endpoint, while linear compounds report minus one
				best.child_a = max(0, leaf_a.part) if type_b == BIG_COMPOUND_TYPE_ID || type_b == MESH_TYPE_ID else leaf_a.part;
				best.child_b = leaf_b.part;
			}
		}
	}
	return best, .Ok;
}
