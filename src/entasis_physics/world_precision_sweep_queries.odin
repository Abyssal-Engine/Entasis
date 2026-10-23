#+vet !unused-imports

package entasis_physics

import "core:math"
import util "entasis:entasis_utilities"

when util.PRECISION_MODE == .Double_Coordinates
{
	// times and local geometry keep the existing sweep kernel's precision. only
	// the absolute contact location is widened, it is reconstructed from the high
	// query origin, never from a rounded absolute f32 pose or endpoint
	Global_Sweep_Result :: struct
	{
		state: Sweep_Hit_State,
		t0: f32,
		t1: f32,
		location: Global_Position,
		normal: util.Vector3,
		child_a: i32,
		child_b: i32,
	}

	Global_Sweep_Query_Hit :: struct
	{
		target_id: i32,
		sweep: Global_Sweep_Result,
	}

	World_Precision_Sweep_Mode :: enum u8
	{
		Any,
		Closest,
		All,
	}

	Global_Sweep_Target_Context :: struct
	{
		simulation: ^Simulation,
		leaves: util.Buffer(Collidable_Reference),
		origin: Global_Position,
	}

	global_sweep_target_resolve :: proc "contextless" (
		user_context: rawptr, leaf_index: int,
	) -> (Shape_Query_Target, Physics_Status)
	{
		query: ^Global_Sweep_Target_Context = (^Global_Sweep_Target_Context)(user_context);
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

	Global_Sweep_Query_Context :: struct
	{
		local: Query_Sweep_Tree_Context,
		origin: Global_Position,
		output: []Global_Sweep_Query_Hit,
		count: int,
		mode: World_Precision_Sweep_Mode,
	}

	// reuse the existing leaf owner, including parent/child filters, registered
	// tasks, static-target semantics, miss/error distinction and pool ownership
	// a single local hit is reused per leaf, no intermediate hit list is built
	global_sweep_tree_leaf :: proc "contextless" (
		user_context: rawptr, leaf_index: int, maximum_t: ^f32,
	) -> Physics_Status
	{
		query: ^Global_Sweep_Query_Context = (^Global_Sweep_Query_Context)(user_context);
		query.local.collector.count = 0;
		local_maximum: f32 = maximum_t^;
		status: Physics_Status = query_sweep_tree_leaf(&query.local, leaf_index, &local_maximum);
		if status != .Ok || query.local.collector.count == 0
		{
			return status;
		}
		hit: Sweep_Query_Hit = query.local.collector.hits.memory[0];
		index: int = query.count;
		if query.mode == .Closest
		{
			if query.count != 0 && hit.sweep.t1 >= query.output[0].sweep.t1
			{
				return .Ok;
			}
			index = 0;
		}
		if index == len(query.output)
		{
			return .Capacity_Missing;
		}
		location: Global_Position = {
			query.origin.x + f64(hit.sweep.location.x),
			query.origin.y + f64(hit.sweep.location.y),
			query.origin.z + f64(hit.sweep.location.z),
		};
		if !(abs(location.x) <= math.F64_MAX && abs(location.y) <= math.F64_MAX && abs(location.z) <= math.F64_MAX)
		{
			return .Invalid_Argument;
		}
		query.output[index] = {
			target_id=hit.target_id,
			sweep={state=hit.sweep.state, t0=hit.sweep.t0, t1=hit.sweep.t1, location=location,
				normal=hit.sweep.normal, child_a=hit.sweep.child_a, child_b=hit.sweep.child_b},
		};
		query.count = index + 1;
		if query.mode == .Any
		{
			// only this completed publication terminates an any-hit operation
			return .Not_Found;
		}
		if query.mode == .Closest
		{
			maximum_t^ = local_maximum;
		}
		return .Ok;
	}

	// Query_Context admission calls this before default settings invoke shape
	// callbacks. low-level callers use the same boundary before the trusted query
	// owner below. the caller owns stack/output, and the world stays immutable
	world_precision_sweep_admit :: proc "contextless" (
		simulation: ^Simulation, pose: Global_Pose, velocity: Body_Velocity,
		maximum_t: f32, traversal_stack: []i32,
	) -> Physics_Status
	{
		if simulation == nil || simulation.state != .Ready
		{
			return .Disposed;
		}
		q: util.Quaternion = pose.orientation;
		length_squared: f32 = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
		if !(abs(pose.position.x) <= math.F64_MAX && abs(pose.position.y) <= math.F64_MAX &&
			abs(pose.position.z) <= math.F64_MAX && abs(length_squared - 1) <= 1e-4 &&
			maximum_t >= 0 && maximum_t <= math.F32_MAX &&
			abs(velocity.linear.x) <= math.F32_MAX && abs(velocity.linear.y) <= math.F32_MAX &&
			abs(velocity.linear.z) <= math.F32_MAX && abs(velocity.angular.x) <= math.F32_MAX &&
			abs(velocity.angular.y) <= math.F32_MAX && abs(velocity.angular.z) <= math.F32_MAX)
		{
			return .Invalid_Argument;
		}
		if len(traversal_stack) < world_precision_ray_stack_requirement(&simulation.broad_phase)
		{
			return .Capacity_Missing;
		}
		return .Ok;
	}

	// requires world_precision_sweep_admit, valid collector mode, resolved sweep
	// settings and the context's live pool. admitted topology cannot change through
	// callbacks. local motion/shape math is f32, long rotating sweeps are not a new
	// precision guarantee. Any/closest have one output entry, all permits zero
	simulation_global_sweep_query :: proc "contextless" (
		simulation: ^Simulation, shape: Typed_Index, pose: Global_Pose, velocity: Body_Velocity,
		maximum_t, minimum_progression, convergence_threshold: f32, maximum_iteration_count: int,
		mode: World_Precision_Sweep_Mode, output: []Global_Sweep_Query_Hit,
		traversal_stack: []i32, pool: ^util.Buffer_Pool,
		filter: Query_Filter_Procedures = {}, user_context: rawptr = nil,
	) -> (int, Physics_Status)
	{
		shapes: ^Shape_Registry = simulation_shape_registry(simulation);
		shape_bounds: Shape_Bounds;
		status: Physics_Status;
		shape_bounds, status = shape_registry_compute_bounds(shapes, shape, pose.orientation);
		if status != .Ok
		{
			return 0, status;
		}
		angular_expansion: f32 = min(shape_bounds.maximum_angular_expansion,
			maximum_t * util.vector3_length(velocity.angular) * shape_bounds.maximum_radius);
		expansion: util.Vector3 = {angular_expansion, angular_expansion, angular_expansion};
		bounds: Global_Bounds;
		bounds, status = world_precision_bounds_from_local(pose.position, {
			min=util.vector3_subtract(shape_bounds.min, expansion),
			max=util.vector3_add(shape_bounds.max, expansion),
		});
		if status != .Ok
		{
			return 0, status;
		}
		broad: ^Broad_Phase = &simulation.broad_phase;
		resolver: Global_Sweep_Target_Context = {simulation=simulation, leaves=broad.active_leaves, origin=pose.position};
		local_hit: Sweep_Query_Hit;
		collector: Sweep_Query_Collector = {
			hits={memory=&local_hit, length=1, id=-1}, mode=.Earliest,
			callbacks={filter=filter, user_context=user_context},
		};
		query: Global_Sweep_Query_Context = {
			origin=pose.position, output=output, mode=mode,
			local={query_shape=shape, query_pose={orientation=pose.orientation}, query_velocity=velocity,
				resolver={user_context=&resolver, resolve=global_sweep_target_resolve},
				maximum_t=maximum_t, minimum_progression=minimum_progression, convergence_threshold=convergence_threshold,
				maximum_iteration_count=maximum_iteration_count, shapes=shapes, collision_tasks=&simulation.collision_tasks,
				sweep_tasks=&simulation.sweep_tasks, collector=&collector, pool=pool},
		};
		remaining: f32 = maximum_t;
		status = world_precision_tree_sweep_traverse(&broad.active_tree, &bounds, velocity.linear,
			&remaining, traversal_stack, global_sweep_tree_leaf, &query);
		if status == .Ok
		{
			resolver.leaves = broad.static_leaves;
			status = world_precision_tree_sweep_traverse(&broad.static_tree, &bounds, velocity.linear,
				&remaining, traversal_stack, global_sweep_tree_leaf, &query);
		}
		if mode == .Any && query.count == 1 && status == .Not_Found
		{
			status = .Ok;
		}
		return query.count, status;
	}
}
