#+vet !unused-imports

package entasis_physics

import util "entasis:entasis_utilities"

when util.PRECISION_MODE == .Double_Coordinates
{
	World_Precision_Ray_Mode :: enum u8
	{
		Closest,
		Any,
		All,
	}

	// t is the fraction [0, 1] of the original f64 segment. location is rebuilt
	// from the authoritative target center and the shape-local float hit, not
	// from a rounded global ray origin/direction. all-hit order is unspecified
	Global_Ray_Query_Hit :: struct
	{
		t: f64,
		location: Global_Position,
		normal: util.Vector3,
		target_id: i32,
		child_index: i32,
	}

	Global_Ray_Query_Context :: struct
	{
		hits: []Global_Ray_Query_Hit,
		count: int,
		mode: World_Precision_Ray_Mode,
		filter: Query_Filter_Procedures,
		user_context: rawptr,
		prepared: ^World_Precision_Ray,
		local: World_Precision_Local_Ray,
	}

	global_ray_filter_child :: proc "contextless" (
		user_context: rawptr, target_id, child_index: i32,
	) -> Query_Filter_Result
	{
		query: ^Global_Ray_Query_Context = (^Global_Ray_Query_Context)(user_context);
		return query_filter_child(query.filter, query.user_context, target_id, child_index);
	}

	global_ray_report :: proc "contextless" (
		user_context: rawptr, _ray: Tree_Ray, hit: ^Ray_Query_Hit, maximum_t: ^f32,
	) -> Physics_Status
	{
		query: ^Global_Ray_Query_Context = (^Global_Ray_Query_Context)(user_context);
		center: [3]f64 = {query.local.position.x, query.local.position.y, query.local.position.z};
		local: [3]f32 = {hit.location.x, hit.location.y, hit.location.z};
		// subtract the original start before adding a small local hit coordinate
		// never derive a fraction by subtracting two rounded exported locations
		distance: f64 = ((center[query.local.axis] - query.local.start_axis) + f64(local[query.local.axis])) * query.local.sign;
		location: Global_Position = {center[0] + f64(local[0]), center[1] + f64(local[1]), center[2] + f64(local[2])};
		// an inside hit at the un-clipped start is exactly the original high
		// endpoint. rounding that point to local f32 must not turn t=0 negative
		// and discard the hit, nor publish a slightly displaced initial point
		if query.local.origin_source == .Segment_Start && hit.t == 0
		{
			distance = 0;
			start: [3]f64 = query.prepared.segment.start;
			location = {start[0], start[1], start[2]};
		}
		if distance < 0 || distance > query.local.distance
		{
			return .Ok;
		}
		result: Global_Ray_Query_Hit = {
			t=distance / query.local.distance,
			location=location,
			normal=hit.normal, target_id=hit.target_id, child_index=hit.child_index,
		};
		if query.mode == .All
		{
			if query.count == len(query.hits)
			{
				return .Capacity_Missing;
			}
			query.hits[query.count] = result;
			query.count += 1;
		}
		else
		{
			// very remote starts can round distinct normalized fractions to the
			// same f64. order along the dominant hit coordinate instead, that
			// coordinate retains the target-scale distinction in the world frame
			previous: Global_Position = query.hits[0].location;
			previous_axis: [3]f64 = {previous.x, previous.y, previous.z};
			current_axis: [3]f64 = {result.location.x, result.location.y, result.location.z};
			if query.count == 0 || (current_axis[query.local.axis] - previous_axis[query.local.axis]) * query.local.sign < 0
			{
				query.hits[0] = result;
				query.count = 1;
			}
			maximum_t^ = min(maximum_t^, hit.t);
			if query.mode == .Any
			{
				maximum_t^ = 0;
			}
		}
		return .Ok;
	}

	// resolve the existing leaf identity once. the float target pose is centered
	// at zero deliberately, the authoritative center travels beside it
	simulation_query_global_target :: proc "contextless" (
		simulation: ^Simulation, reference: Collidable_Reference,
	) -> (Shape_Query_Target, Global_Position, Physics_Status)
	{
		if collidable_reference_mobility(reference) == .Static
		{
			index: int;
			status: Physics_Status;
			index, status = statics_resolve(&simulation.statics, {collidable_reference_raw_handle(reference)});
			if status != .Ok
			{
				return {}, {}, status;
			}
			value: ^Static = &simulation.statics.statics.memory[index];
			return {shape=value.shape, pose={orientation=value.pose.orientation}, target_id=i32(reference.packed)}, simulation.statics.global_positions.memory[index], .Ok;
		}
		location: Body_Memory_Location;
		status: Physics_Status;
		location, status = bodies_resolve(&simulation.bodies, {collidable_reference_raw_handle(reference)});
		if status != .Ok
		{
			return {}, {}, status;
		}
		set: ^Body_Set = &simulation.bodies.sets.memory[location.set_index];
		return {
			shape=set.collidables.memory[location.index].shape,
			pose={orientation=set.dynamics_state.memory[location.index].motion.pose.orientation},
			target_id=i32(reference.packed),
		}, set.global_positions.memory[location.index], .Ok;
	}

	simulation_global_ray_tree :: proc "contextless" (
		simulation: ^Simulation, tree: ^Tree, leaves: util.Buffer(Collidable_Reference),
		prepared: ^World_Precision_Ray, query: ^Global_Ray_Query_Context,
		stack: []i32, pool: ^util.Buffer_Pool,
	) -> Physics_Status
	{
		if tree.leaf_count == 0
		{
			return .Ok;
		}
		shapes: ^Shape_Registry = simulation_shape_registry(simulation);
		collector: Ray_Query_Collector = {
			mode=.All, delivery=.Callback,
			callbacks={filter={allow_child=global_ray_filter_child}, hit=global_ray_report, user_context=query},
		};
		stack[0] = 0;
		stack_count: int = 1;
		for stack_count > 0
		{
			stack_count -= 1;
			node: Tree_Node = tree.nodes.memory[stack[stack_count]];
			children: [2]Tree_Node_Child = {node.a, node.b};
			child_count: int = 2;
			if tree.leaf_count == 1
			{
				child_count = 1;
			}
			for child in children[:child_count]
			{
				min_t, max_t: f64;
				state: Reference_State;
				min_t, max_t, state = world_precision_segment_clip_child(&prepared.segment, child);
				_ = max_t;
				if state == .Missing || query.mode == .Closest && query.count != 0 && min_t > query.hits[0].t
				{
					continue;
				}
				if child.index >= 0
				{
					stack[stack_count] = child.index;
					stack_count += 1;
					continue;
				}
				reference: Collidable_Reference = leaves.memory[tree_decode_leaf(child.index)];
				if query_filter_target(query.filter, query.user_context, i32(reference.packed)) != .Allow
				{
					continue;
				}
				target: Shape_Query_Target;
				position: Global_Position;
				status: Physics_Status;
				target, position, status = simulation_query_global_target(simulation, reference);
				if status != .Ok
				{
					return status;
				}
				shape: rawptr;
				batch: ^Shape_Batch;
				shape, batch, status = shape_registry_resolve(shapes, target.shape);
				if status != .Ok
				{
					return status;
				}
				local_bounds: ^Static_Local_Bounds;
				if collidable_reference_mobility(reference) == .Static
				{
					// global target resolution already admitted this live handle
					index: int = int(simulation.statics.handle_to_index.memory[collidable_reference_raw_handle(reference)]);
					local_bounds = &simulation.statics.local_bounds.memory[index];
				}
				bounds: Shape_Bounds;
				if local_bounds != nil && local_bounds.state == .Present &&
					typed_index_type(target.shape) == MESH_TYPE_ID &&
					batch.dispatch == .Native && batch.metadata.bounds == mesh_bounds_proc
				{
					bounds = local_bounds.bounds;
				}
				else
				{
					bounds, status = shape_batch_compute_bounds(batch, shape, target.pose.orientation, shapes);
				}
				if status != .Ok
				{
					return status;
				}
				query.local, state, status = world_precision_ray_localize(prepared, position, bounds);
				if status != .Ok
				{
					return status;
				}
				if state == .Missing
				{
					continue;
				}
				maximum_t: f32 = query.local.ray.maximum_t;
				status = query_ray_shape_with_maximum_filtered(
					shapes, target, query.local.ray, &collector, &maximum_t, 0, .Present, pool,
				);
				if status != .Ok
				{
					return status;
				}
				if query.mode == .Any && query.count != 0
				{
					return .Ok;
				}
			}
		}
		return .Ok;
	}

	// empty trees retain an allocated root node but need no traversal storage
	world_precision_ray_stack_requirement :: proc "contextless" (broad: ^Broad_Phase) -> int
	{
		active: int = int(broad.active_tree.node_count) if broad.active_tree.leaf_count > 0 else 0;
		stationary: int = int(broad.static_tree.node_count) if broad.static_tree.leaf_count > 0 else 0;
		return max(active, stationary);
	}

	// caller owns disjoint output/stack storage, exclusive pool use and a read-only
	// simulation for this call, including callbacks. stack capacity is checked
	// for both trees before any callback or output. local child traversal reuses
	// the supplied pool, reserve it before an allocation-free recurring workload
	// all mode returns a valid written prefix on failure. Closest/Any facades
	// discard partial results. custom ray callbacks explicitly receive the local
	// frame through this new API, ordinary float query callbacks are unchanged
	simulation_global_ray_query :: proc "contextless" (
		simulation: ^Simulation, segment: World_Precision_Segment,
		mode: World_Precision_Ray_Mode, hits: []Global_Ray_Query_Hit,
		traversal_stack: []i32, pool: ^util.Buffer_Pool,
		filter: Query_Filter_Procedures = {}, user_context: rawptr = nil,
	) -> (int, Physics_Status)
	{
		if simulation == nil || simulation.state != .Ready
		{
			return 0, .Disposed;
		}
		if mode != .All && mode != .Any && mode != .Closest || mode != .All && len(hits) == 0
		{
			return 0, .Invalid_Argument;
		}
		prepared: World_Precision_Ray;
		status: Physics_Status;
		prepared, status = world_precision_ray_prepare(segment);
		if status != .Ok
		{
			return 0, status;
		}
		broad: ^Broad_Phase = &simulation.broad_phase;
		if len(traversal_stack) < world_precision_ray_stack_requirement(broad)
		{
			return 0, .Capacity_Missing;
		}
		query: Global_Ray_Query_Context = {hits=hits, mode=mode, filter=filter, user_context=user_context, prepared=&prepared};
		status = simulation_global_ray_tree(simulation, &broad.active_tree, broad.active_leaves, &prepared, &query, traversal_stack, pool);
		if status == .Ok && !(mode == .Any && query.count != 0)
		{
			status = simulation_global_ray_tree(simulation, &broad.static_tree, broad.static_leaves, &prepared, &query, traversal_stack, pool);
		}
		return query.count, status;
	}
}
