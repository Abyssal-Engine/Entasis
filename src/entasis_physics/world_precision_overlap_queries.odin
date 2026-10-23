#+vet !unused-imports

package entasis_physics

import "core:math"
import util "entasis:entasis_utilities"

when util.PRECISION_MODE == .Double_Coordinates
{
	Global_Overlap_Prepared :: struct
	{
		bounds: Global_Bounds,
		pose: Rigid_Pose,
		raw: rawptr,
		shapes: ^Shape_Registry,
	}

	Global_Overlap_Target_Context :: struct
	{
		simulation: ^Simulation,
		leaves: util.Buffer(Collidable_Reference),
		origin: Global_Position,
	}

	// the resolver consumes current authoritative positions, including sleepers
	// the query center is zero, so every existing manifold offset retains its
	// query-relative interpretation and needs no absolute-position translation
	global_overlap_target_resolve :: proc "contextless" (
		user_context: rawptr, leaf_index: int,
	) -> (Shape_Query_Target, Physics_Status)
	{
		query: ^Global_Overlap_Target_Context = (^Global_Overlap_Target_Context)(user_context);
		target: Shape_Query_Target;
		position: Global_Position;
		status: Physics_Status;
		target, position, status = simulation_query_global_target(query.simulation, query.leaves.memory[leaf_index]);
		if status != .Ok
		{
			return {}, status;
		}
		target.pose.position, status = world_precision_relative_position(query.origin, position);
		return target, status;
	}

	global_overlap_prepare :: proc "contextless" (
		simulation: ^Simulation, shape: Typed_Index, pose: Global_Pose,
		traversal_stack: []i32, batcher: ^Query_Overlap_Batcher, pool: ^util.Buffer_Pool,
	) -> (Global_Overlap_Prepared, Physics_Status)
	{
		if simulation == nil || simulation.state != .Ready
		{
			return {}, .Disposed;
		}
		if batcher == nil || batcher.state != .Ready || batcher.collision == nil ||
			batcher.collision.state != .Ready || batcher.collision.pair_count != 0 || pool == nil
		{
			return {}, .Invalid_Argument;
		}
		q: util.Quaternion = pose.orientation;
		length_squared: f32 = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
		if !(abs(pose.position.x) <= math.F64_MAX && abs(pose.position.y) <= math.F64_MAX &&
			abs(pose.position.z) <= math.F64_MAX && abs(length_squared - 1) <= 1e-4)
		{
			return {}, .Invalid_Argument;
		}
		// rays and volumes share the same worst-case live-node scratch bound
		if len(traversal_stack) < world_precision_ray_stack_requirement(&simulation.broad_phase)
		{
			return {}, .Capacity_Missing;
		}
		prepared: Global_Overlap_Prepared = {
			pose={orientation=pose.orientation}, shapes=simulation_shape_registry(simulation),
		};
		status: Physics_Status;
		prepared.raw, _, status = shape_registry_resolve(prepared.shapes, shape);
		if status != .Ok
		{
			return {}, status;
		}
		local: Shape_Bounds;
		local, status = shape_registry_compute_bounds(prepared.shapes, shape, pose.orientation);
		if status != .Ok
		{
			return {}, status;
		}
		prepared.bounds, status = world_precision_bounds_from_local(pose.position, {min=local.min, max=local.max});
		return prepared, status;
	}

	// the native Query_Context owns its exclusive batcher/pool, the caller owns
	// disjoint scratch/output. geometry callbacks receive a query-centered f32
	// frame, child callbacks keep their identities. float query inputs stay intact
	simulation_global_overlap_query :: proc "contextless" (
		simulation: ^Simulation, shape: Typed_Index, pose: Global_Pose,
		collector: ^Overlap_Query_Collector, traversal_stack: []i32,
		batcher: ^Query_Overlap_Batcher, pool: ^util.Buffer_Pool,
	) -> Physics_Status
	{
		if collector == nil || collector.count != 0 || collector.hits.length < 0 ||
			collector.hits.length > 0 && collector.hits.memory == nil
		{
			return .Invalid_Argument;
		}
		prepared: Global_Overlap_Prepared;
		status: Physics_Status;
		prepared, status = global_overlap_prepare(simulation, shape, pose, traversal_stack, batcher, pool);
		if status != .Ok
		{
			return status;
		}
		previous_pool: ^util.Buffer_Pool = batcher.collision.traversal_pool;
		batcher.collision.traversal_pool = pool;
		defer batcher.collision.traversal_pool = previous_pool;
		broad: ^Broad_Phase = &simulation.broad_phase;
		resolver_context: Global_Overlap_Target_Context = {
			simulation=simulation, leaves=broad.active_leaves, origin=pose.position,
		};
		query: Query_Overlap_Tree_Context = {
			query_shape=shape, query_raw=prepared.raw, query_pose=prepared.pose,
			resolver={user_context=&resolver_context, resolve=global_overlap_target_resolve},
			shapes=prepared.shapes, collision_tasks=&simulation.collision_tasks,
			collector=collector, pool=pool, batcher=batcher,
		};
		status = world_precision_tree_volume_traverse(
			&broad.active_tree, prepared.bounds, traversal_stack, query_overlap_tree_leaf, &query,
		);
		if status == .Ok
		{
			resolver_context.leaves = broad.static_leaves;
			status = world_precision_tree_volume_traverse(
				&broad.static_tree, prepared.bounds, traversal_stack, query_overlap_tree_leaf, &query,
			);
		}
		return status;
	}

	// preserve the any-overlap owner's nonnegative-depth and child-route error
	// semantics rather than interpreting an arbitrary callback Not_Found as a hit
	simulation_global_overlap_any_query :: proc "contextless" (
		simulation: ^Simulation, shape: Typed_Index, pose: Global_Pose,
		filter: Query_Filter_Procedures, user_context: rawptr,
		traversal_stack: []i32, batcher: ^Query_Overlap_Batcher, pool: ^util.Buffer_Pool,
	) -> (Overlap_Query_State, Physics_Status)
	{
		prepared: Global_Overlap_Prepared;
		status: Physics_Status;
		prepared, status = global_overlap_prepare(simulation, shape, pose, traversal_stack, batcher, pool);
		if status != .Ok
		{
			return .Separated, status;
		}
		previous_pool: ^util.Buffer_Pool = batcher.collision.traversal_pool;
		batcher.collision.traversal_pool = pool;
		defer batcher.collision.traversal_pool = previous_pool;
		broad: ^Broad_Phase = &simulation.broad_phase;
		resolver_context: Global_Overlap_Target_Context = {
			simulation=simulation, leaves=broad.active_leaves, origin=pose.position,
		};
		query: Query_Overlap_Any_Context = {
			query_shape=shape, query_raw=prepared.raw, query_pose=prepared.pose,
			resolver={user_context=&resolver_context, resolve=global_overlap_target_resolve},
			shapes=prepared.shapes, collision_tasks=&simulation.collision_tasks,
			filter=filter, user_context=user_context, batcher=batcher,
		};
		status = world_precision_tree_volume_traverse(
			&broad.active_tree, prepared.bounds, traversal_stack, query_overlap_any_tree_leaf, &query,
		);
		if status == .Ok
		{
			resolver_context.leaves = broad.static_leaves;
			status = world_precision_tree_volume_traverse(
				&broad.static_tree, prepared.bounds, traversal_stack, query_overlap_any_tree_leaf, &query,
			);
		}
		if status == .Not_Found && query.state == .Intersecting
		{
			return .Intersecting, .Ok;
		}
		return .Separated, status;
	}
}
