// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"
import "core:math"

simulation_query_target :: proc "contextless" (
	simulation: ^Simulation, reference: Collidable_Reference,
) -> (Shape_Query_Target, Physics_Status)
{
	if collidable_reference_mobility(reference) == .Static
	{
		index, status := statics_resolve(&simulation.statics, {collidable_reference_raw_handle(reference)});
		if status != .Ok
		{
			return {}, status;
		}
		value := &simulation.statics.statics.memory[index];
		return {shape=value.shape, pose=value.pose, target_id=i32(reference.packed)}, .Ok;
	}
	location, status := bodies_resolve(&simulation.bodies, {collidable_reference_raw_handle(reference)});
	if status != .Ok
	{
		return {}, status;
	}
	set := &simulation.bodies.sets.memory[location.set_index];
	return {
		shape=set.collidables.memory[location.index].shape,
		pose=set.dynamics_state.memory[location.index].motion.pose,
		target_id=i32(reference.packed),
	}, .Ok;
}

Simulation_Query_Target_Context :: struct
{
	simulation: ^Simulation,
	leaves:     util.Buffer(Collidable_Reference),
}

simulation_query_target_resolve :: proc "contextless" (
	user_context: rawptr, leaf_index: int,
) -> (Shape_Query_Target, Physics_Status)
{
	resolver_context := (^Simulation_Query_Target_Context)(user_context);
	if resolver_context == nil || resolver_context.simulation == nil || leaf_index < 0 ||
	leaf_index >= int(resolver_context.leaves.length)
	{
		return {}, .Invalid_Argument;
	}
	return simulation_query_target(resolver_context.simulation, resolver_context.leaves.memory[leaf_index]);
}

simulation_query_resolver :: proc "contextless" (
	resolver_context: ^Simulation_Query_Target_Context, simulation: ^Simulation,
	leaves: util.Buffer(Collidable_Reference),
) -> Query_Target_Resolver
{
	resolver_context^ = {simulation=simulation, leaves=leaves};
	return {user_context=resolver_context, resolve=simulation_query_target_resolve};
}

@(private="file")
Simulation_Ray_Query_Context :: struct
{
	simulation: ^Simulation,
	shapes:     ^Shape_Registry,
	leaves:     util.Buffer(Collidable_Reference),
	ray:        Tree_Ray,
	collector:  ^Ray_Query_Collector,
	pool:       ^util.Buffer_Pool,
}

@(private="file")
simulation_ray_query_leaf :: proc "contextless" (
	user_context: rawptr, leaf_index: int, maximum_t: ^f32,
) -> Physics_Status
{
	query_context := (^Simulation_Ray_Query_Context)(user_context);
	if leaf_index < 0 || leaf_index >= int(query_context.leaves.length)
	{
		return .Invalid_Argument;
	}
	// this callback is only installed by simulation_ray_tree_query below.
	// resolve both active and sleeping/static leaves from their existing handles
	target, status := simulation_query_target(
		query_context.simulation, query_context.leaves.memory[leaf_index],
	);
	if status != .Ok
	{
		return status;
	}
	return query_ray_shape_with_maximum_filtered(
		query_context.shapes, target, query_context.ray, query_context.collector,
		maximum_t, 0, .Missing, query_context.pool,
	);
}

@(private="file")
simulation_ray_tree_query :: #force_inline proc "contextless" (
	simulation: ^Simulation, shapes: ^Shape_Registry, tree: ^Tree,
	leaves: util.Buffer(Collidable_Reference), ray: Tree_Ray,
	collector: ^Ray_Query_Collector, pool: ^util.Buffer_Pool,
) -> (f32, Physics_Status)
{
	query_context := Simulation_Ray_Query_Context{
		simulation=simulation, shapes=shapes, leaves=leaves, ray=ray,
		collector=collector, pool=pool,
	};
	return query_ray_tree_traverse(
		tree, ray, simulation_ray_query_leaf, &query_context, collector, pool,
	);
}

simulation_ray_query :: proc "contextless" (
	simulation: ^Simulation, ray: Tree_Ray, collector: ^Ray_Query_Collector,
	pool: ^util.Buffer_Pool,
) -> Physics_Status
{
	if simulation == nil || simulation.state != .Ready && simulation.state != .Stepping
	{
		return .Disposed;
	}
	shapes := simulation_shape_registry(simulation);
	if shapes == nil || collector == nil || shape_ray_valid(ray) != .Ok
	{
		return .Invalid_Argument;
	}
	active_maximum := ray.maximum_t;
	if simulation.broad_phase.active_tree.leaf_count > 0
	{
		status: Physics_Status;
		active_maximum, status = simulation_ray_tree_query(
			simulation, shapes, &simulation.broad_phase.active_tree,
			simulation.broad_phase.active_leaves,
			ray, collector, pool,
		);
		if status != .Ok
		{
			return status;
		}
	}
	if simulation.broad_phase.static_tree.leaf_count == 0
	{
		return .Ok;
	}
	static_ray := ray;
	static_ray.maximum_t = min(ray.maximum_t, active_maximum);
	_, status := simulation_ray_tree_query(
		simulation, shapes, &simulation.broad_phase.static_tree,
		simulation.broad_phase.static_leaves,
		static_ray, collector, pool,
	);
	return status;
}

// query_batch checks the world and every ray before its ray-run helper calls
// this function with matching completion storage and callback-free query filters
simulation_ray_query_batch :: #force_no_inline proc "contextless" (
	simulation: ^Simulation, rays: []Tree_Ray, completions: []Ray_Query_Batch_Completion,
	callbacks: Ray_Query_Callbacks, pool: ^util.Buffer_Pool,
)
{
	shapes: ^Shape_Registry = simulation_shape_registry(simulation);
	for ray, ray_index in rays
	{
		completion: ^Ray_Query_Batch_Completion = &completions[ray_index];
		completion^ = {};
		collector: Ray_Query_Collector = {
			hits={memory=&completion.hit, length=1, id=util.BUFFER_CALLER_OWNED_ID},
			mode=.Earliest,
			callbacks=callbacks,
		};
		active_maximum: f32 = ray.maximum_t;
		status: Physics_Status = .Ok;
		if simulation.broad_phase.active_tree.leaf_count > 0
		{
			active_maximum, status = simulation_ray_tree_query(
				simulation, shapes, &simulation.broad_phase.active_tree,
				simulation.broad_phase.active_leaves, ray, &collector, pool,
			);
		}
		if status == .Ok && simulation.broad_phase.static_tree.leaf_count > 0
		{
			static_ray: Tree_Ray = ray;
			static_ray.maximum_t = min(ray.maximum_t, active_maximum);
			_, status = simulation_ray_tree_query(
				simulation, shapes, &simulation.broad_phase.static_tree,
				simulation.broad_phase.static_leaves, static_ray, &collector, pool,
			);
		}
		if status != .Ok
		{
			// a failed static traversal must discard any earlier active-tree hit
			completion^ = {status=status};
		}
		else if collector.count != 0
		{
			completion.presence = .Present;
		}
	}
}

simulation_overlap_query :: proc "contextless" (
	simulation: ^Simulation, query_shape: Typed_Index, query_pose: Rigid_Pose,
	collector: ^Overlap_Query_Collector, pool: ^util.Buffer_Pool,
) -> Physics_Status
{
	if simulation == nil || simulation.state != .Ready && simulation.state != .Stepping
	{
		return .Disposed;
	}
	shapes := simulation_shape_registry(simulation);
	if simulation.query_overlap_batcher.state == .Uninitialized
	{
		bind_status := query_overlap_batcher_bind(&simulation.query_overlap_batcher, &simulation.narrow_phase);
		if bind_status != .Ok
		{
			return bind_status;
		}
	}
	active_context, static_context: Simulation_Query_Target_Context;
	status := query_overlap_resolved(
		query_shape, query_pose, &simulation.broad_phase.active_tree,
		simulation_query_resolver(&active_context, simulation, simulation.broad_phase.active_leaves),
		shapes, &simulation.collision_tasks, collector, pool,
		&simulation.query_overlap_batcher,
	);
	if status != .Ok
	{
		return status;
	}
	return query_overlap_resolved(
		query_shape, query_pose, &simulation.broad_phase.static_tree,
		simulation_query_resolver(&static_context, simulation, simulation.broad_phase.static_leaves),
		shapes, &simulation.collision_tasks, collector, pool,
		&simulation.query_overlap_batcher,
	);
}

simulation_volume_query :: proc "contextless" (
	simulation: ^Simulation, volume: util.Bounding_Box, collector: ^Volume_Query_Collector,
	pool: ^util.Buffer_Pool,
) -> Physics_Status
{
	if simulation == nil || simulation.state != .Ready && simulation.state != .Stepping
	{
		return .Disposed;
	}
	shapes := simulation_shape_registry(simulation);
	active_context, static_context: Simulation_Query_Target_Context;
	status := query_volume_resolved(
		volume, &simulation.broad_phase.active_tree,
		simulation_query_resolver(&active_context, simulation, simulation.broad_phase.active_leaves),
		shapes, collector, pool,
	);
	if status != .Ok
	{
		return status;
	}
	return query_volume_resolved(
		volume, &simulation.broad_phase.static_tree,
		simulation_query_resolver(&static_context, simulation, simulation.broad_phase.static_leaves),
		shapes, collector, pool,
	);
}

simulation_sweep_query :: proc "contextless" (
	simulation: ^Simulation, query_shape: Typed_Index, query_pose: Rigid_Pose,
	query_velocity: Body_Velocity, maximum_t, minimum_progression, convergence_threshold: f32,
	maximum_iteration_count: int, collector: ^Sweep_Query_Collector,
	pool: ^util.Buffer_Pool,
) -> Physics_Status
{
	if simulation == nil || simulation.state != .Ready && simulation.state != .Stepping
	{
		return .Disposed;
	}
	shapes := simulation_shape_registry(simulation);
	if shapes == nil || collector == nil || collector.hits.memory == nil
	{
		return .Invalid_Argument;
	}
	// registered bounds callbacks are read-only. resolve the query shape once,
	// then reuse the same geometric bounds across active and static populations
	shape_bounds, bounds_status := shape_registry_compute_bounds(shapes, query_shape, query_pose.orientation);
	if bounds_status != .Ok
	{
		return bounds_status;
	}
	query_context := Query_Sweep_Tree_Context{
		query_shape=query_shape, query_pose=query_pose, query_velocity=query_velocity,
		maximum_t=maximum_t, minimum_progression=minimum_progression,
		convergence_threshold=convergence_threshold, maximum_iteration_count=maximum_iteration_count,
		shapes=shapes, collision_tasks=&simulation.collision_tasks, sweep_tasks=&simulation.sweep_tasks,
		collector=collector, pool=pool,
	};
	resolver_context: Simulation_Query_Target_Context;
	active_maximum := maximum_t;
	if simulation.broad_phase.active_tree.leaf_count > 0
	{
		query_context.resolver = simulation_query_resolver(
			&resolver_context, simulation, simulation.broad_phase.active_leaves,
		);
		status: Physics_Status;
		active_maximum, status = query_sweep_prepared(
			&simulation.broad_phase.active_tree, &shape_bounds, &query_context, maximum_t,
		);
		if status != .Ok || simulation.broad_phase.static_tree.leaf_count == 0
		{
			return status;
		}
	}
	query_context.resolver = simulation_query_resolver(
		&resolver_context, simulation, simulation.broad_phase.static_leaves,
	);
	_, status := query_sweep_prepared(
		&simulation.broad_phase.static_tree, &shape_bounds, &query_context, active_maximum,
	);
	return status;
}

// sibling any-hit route. the shared simulation batcher is selected only by an
// idle-world caller. contexts pass their independent batcher explicitly
simulation_overlap_any_query :: proc "contextless" (
	simulation: ^Simulation, query_shape: Typed_Index, query_pose: Rigid_Pose,
	filter: Query_Filter_Procedures, user_context: rawptr,
	pool: ^util.Buffer_Pool, batcher: ^Query_Overlap_Batcher,
) -> (Overlap_Query_State, Physics_Status)
{
	q: util.Quaternion = query_pose.orientation;
	length_squared: f32 = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
	if !(abs(query_pose.position.x) <= math.F32_MAX && abs(query_pose.position.y) <= math.F32_MAX &&
		abs(query_pose.position.z) <= math.F32_MAX && abs(length_squared-1) <= 1e-4)
	{
		return .Separated, .Invalid_Argument;
	}
	shapes: ^Shape_Registry = simulation_shape_registry(simulation);
	active_context, static_context: Simulation_Query_Target_Context;
	return query_overlap_any_resolved(query_shape, query_pose, &simulation.broad_phase.active_tree,
		simulation_query_resolver(&active_context, simulation, simulation.broad_phase.active_leaves),
		&simulation.broad_phase.static_tree,
		simulation_query_resolver(&static_context, simulation, simulation.broad_phase.static_leaves),
		shapes, &simulation.collision_tasks, filter, user_context, pool, batcher);
}
