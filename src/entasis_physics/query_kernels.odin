// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"
import "core:simd"

Query_Collection_Mode :: enum u8
{
	Earliest,
	All,
}

Query_Filter_Result :: enum u8
{
	Reject,
	Allow,
}

Query_Allow_Target_Proc :: #type proc "contextless" (
	user_context: rawptr, target_id: i32,
) -> Query_Filter_Result;
Query_Allow_Child_Proc :: #type proc "contextless" (
	user_context: rawptr, target_id, child_index: i32,
) -> Query_Filter_Result;
Query_Filter_Procedures :: struct
{
	allow_target: Query_Allow_Target_Proc,
	allow_child:  Query_Allow_Child_Proc,
}

Shape_Query_Target :: struct
{
	shape:     Typed_Index,
	pose:      Rigid_Pose,
	target_id: i32,
}

Query_Target_Resolve_Proc :: #type proc "contextless" (
	user_context: rawptr, leaf_index: int,
) -> (Shape_Query_Target, Physics_Status);
Query_Target_Resolver :: struct
{
	user_context: rawptr,
	resolve:      Query_Target_Resolve_Proc,
}

Query_Buffer_Target_Context :: struct
{
	targets: util.Buffer(Shape_Query_Target),
}

query_buffer_target_resolve :: proc "contextless" (
	user_context: rawptr, leaf_index: int,
) -> (Shape_Query_Target, Physics_Status)
{
	resolver_context := (^Query_Buffer_Target_Context)(user_context);
	if resolver_context == nil || leaf_index < 0 || leaf_index >= int(resolver_context.targets.length)
	{
		return {}, .Invalid_Argument;
	}
	return resolver_context.targets.memory[leaf_index], .Ok;
}

query_target_resolver_valid :: proc "contextless" (resolver: Query_Target_Resolver) -> Reference_State
{
	if resolver.resolve == nil
	{
		return .Missing;
	}
	return .Present;
}

query_filter_target :: proc "contextless" (
	procedures: Query_Filter_Procedures, user_context: rawptr, target_id: i32,
) -> Query_Filter_Result
{
	if procedures.allow_target == nil
	{
		return .Allow;
	}
	return procedures.allow_target(user_context, target_id);
}

query_filter_child :: proc "contextless" (
	procedures: Query_Filter_Procedures, user_context: rawptr, target_id, child_index: i32,
) -> Query_Filter_Result
{
	if procedures.allow_child == nil
	{
		return .Allow;
	}
	return procedures.allow_child(user_context, target_id, child_index);
}

Ray_Query_Hit :: struct
{
	t:           f32,
	location:    util.Vector3,
	normal:      util.Vector3,
	target_id:   i32,
	child_index: i32,
	ray_id:      i32,
}

// each engine-owned ray request has its own completion. a failure ends that
// request without publishing an earlier hit or skipping later rays
Ray_Query_Batch_Completion :: struct
{
	hit: Ray_Query_Hit,
	status: Physics_Status,
	presence: Reference_State,
}

Ray_Query_Hit_Proc :: #type proc "contextless" (
	user_context: rawptr, ray: Tree_Ray, hit: ^Ray_Query_Hit, maximum_t: ^f32,
) -> Physics_Status;
Ray_Query_Callbacks :: struct
{
	filter:       Query_Filter_Procedures,
	hit:          Ray_Query_Hit_Proc,
	user_context: rawptr,
}

Ray_Query_Collector :: struct
{
	hits:      util.Buffer(Ray_Query_Hit),
	count:     int,
	mode:      Query_Collection_Mode,
	callbacks: Ray_Query_Callbacks,
}

Overlap_Query_Hit :: struct
{
	target_id: i32,
	manifold:  Manifold_Result,
}

Overlap_Query_Collector :: struct
{
	hits:         util.Buffer(Overlap_Query_Hit),
	count:        int,
	filter:       Query_Filter_Procedures,
	user_context: rawptr,
}

Query_Overlap_Batcher_State :: enum u8
{
	Uninitialized,
	Ready,
	Disposed,
}

Query_Overlap_Batcher :: struct
{
	collision:        ^Collision_Batcher,
	narrow:           ^Narrow_Phase,
	collector:        ^Overlap_Query_Collector,
	current_target_id: i32,
	state:            Query_Overlap_Batcher_State,
}

query_overlap_pair_completed :: proc "contextless" (
	user_context: rawptr, _pair_id: i32, manifold: ^Manifold_Result,
) -> Physics_Status
{
	batcher := (^Query_Overlap_Batcher)(user_context);
	if batcher == nil || batcher.state != .Ready || batcher.collector == nil || manifold == nil
	{
		return .Invalid_Argument;
	}
	// presence depends only on the active count. do not search depths or copy
	// a deepest-contact result that this collector never consumes
	contact_count: i32 = manifold.convex.count;
	if manifold.kind != .Convex
	{
		contact_count = manifold.nonconvex.count;
	}
	if contact_count <= 0
	{
		return .Ok;
	}
	if batcher.collector.count >= int(batcher.collector.hits.length)
	{
		return .Capacity_Missing;
	}
	hit: ^Overlap_Query_Hit = &batcher.collector.hits.memory[batcher.collector.count];
	hit.manifold = manifold^;
	hit.target_id = batcher.current_target_id;
	batcher.collector.count += 1;
	return .Ok;
}

query_overlap_allow_child :: proc "contextless" (
	user_context: rawptr, _pair_id, _child_a, child_b: i32,
) -> Collision_Testing_State
{
	batcher := (^Query_Overlap_Batcher)(user_context);
	if batcher == nil || batcher.collector == nil
	{
		return .Reject;
	}
	if query_filter_child(
		batcher.collector.filter, batcher.collector.user_context, batcher.current_target_id, child_b,
	) != .Allow
	{
		return .Reject;
	}
	return .Allow;
}

query_overlap_batcher_bind :: proc "contextless" (
	batcher: ^Query_Overlap_Batcher, narrow: ^Narrow_Phase,
) -> Physics_Status
{
	if batcher == nil || batcher.state != .Uninitialized || narrow == nil || narrow.state != .Ready ||
	narrow.active_worker_count <= 0
	{
		return .Invalid_Argument;
	}
	if narrow.batchers[0].state != .Ready
	{
		status := narrow_phase_prepare_batchers(narrow, narrow.active_worker_count);
		if status != .Ok
		{
			return status;
		}
	}
	batcher^ = {collision=&narrow.batchers[0], narrow=narrow, state=.Ready};
	return .Ok;
}

query_overlap_batcher_dispose :: proc (batcher: ^Query_Overlap_Batcher) -> Physics_Status
{
	if batcher == nil || batcher.state != .Ready
	{
		return .Disposed;
	}
	batcher^ = {state=.Disposed};
	return .Ok;
}

query_overlap_batcher_test :: proc "contextless" (
	batcher: ^Query_Overlap_Batcher, query_shape, target_shape: Typed_Index,
	query_pose, target_pose: Rigid_Pose, target_id: i32, collector: ^Overlap_Query_Collector,
) -> Physics_Status
{
	if batcher == nil || batcher.state != .Ready || batcher.narrow == nil ||
	batcher.narrow.state != .Ready || batcher.collision == nil || batcher.collision.state != .Ready ||
	batcher.collision.pair_count != 0 || collector == nil
	{
		return .Invalid_Argument;
	}
	procedures := batcher.collision.procedures;
	stored_pair_completed := batcher.collision.stored_pair_completed;
	user_context := batcher.collision.user_context;
	batcher.collector = collector;
	batcher.current_target_id = target_id;
	batcher.collision.procedures = {
		pair_completed=query_overlap_pair_completed,
		allow_child_pair=query_overlap_allow_child,
	};
	batcher.collision.stored_pair_completed = nil;
	batcher.collision.user_context = batcher;
	status := collision_batcher_add(
		batcher.collision, query_shape, target_shape, query_pose, target_pose, 0, 0,
	);
	if status == .Ok
	{
		status = collision_batcher_flush(batcher.collision);
	}
	if batcher.collision.state == .Faulted
	{
		_ = collision_batcher_reset_fault(batcher.collision);
	}
	batcher.collision.procedures = procedures;
	batcher.collision.stored_pair_completed = stored_pair_completed;
	batcher.collision.user_context = user_context;
	batcher.collector = nil;
	return status;
}

Volume_Query_Hit :: struct
{
	target_id: i32,
}

Volume_Query_Collector :: struct
{
	hits:         util.Buffer(Volume_Query_Hit),
	count:        int,
	filter:       Query_Filter_Procedures,
	user_context: rawptr,
}

Sweep_Query_Hit :: struct
{
	target_id: i32,
	sweep:     Sweep_Result,
}

Sweep_Query_Hit_Proc :: #type proc "contextless" (
	user_context: rawptr, hit: ^Sweep_Query_Hit, maximum_t: ^f32,
) -> Physics_Status;
Sweep_Query_Zero_Hit_Proc :: #type proc "contextless" (
	user_context: rawptr, target_id: i32, maximum_t: ^f32,
) -> Physics_Status;
Sweep_Query_Callbacks :: struct
{
	filter:       Query_Filter_Procedures,
	hit:          Sweep_Query_Hit_Proc,
	hit_at_zero:  Sweep_Query_Zero_Hit_Proc,
	user_context: rawptr,
}

Sweep_Query_Collector :: struct
{
	hits:      util.Buffer(Sweep_Query_Hit),
	count:     int,
	mode:      Query_Collection_Mode,
	callbacks: Sweep_Query_Callbacks,
}

Wide_Ray_Query :: struct
{
	rays:       [util.PRODUCTION_LANE_COUNT]Tree_Ray,
	lane_count: int,
}

Wide_Ray_Result :: struct
{
	hits:   [util.PRODUCTION_LANE_COUNT]Shape_Ray_Hit,
	status: [util.PRODUCTION_LANE_COUNT]Physics_Status,
}

Ray_Batcher_State :: enum u8
{
	Uninitialized,
	Ready,
}

Ray_Batcher_Request :: struct
{
	target: Shape_Query_Target,
	ray:    Tree_Ray,
	ray_id: i32,
}

Ray_Batcher :: struct
{
	requests: util.Buffer(Ray_Batcher_Request),
	pool:     ^util.Buffer_Pool,
	count:    int,
	state:    Ray_Batcher_State,
}

ray_query_collector_initialize :: proc "contextless" (
	collector: ^Ray_Query_Collector, storage: util.Buffer(Ray_Query_Hit), mode: Query_Collection_Mode,
	callbacks: Ray_Query_Callbacks = {},
) -> Physics_Status
{
	if collector == nil || storage.memory == nil || storage.length <= 0
	{
		return .Invalid_Argument;
	}
	collector^ = {hits=storage, mode=mode, callbacks=callbacks};
	return .Ok;
}

ray_query_collector_add :: proc "contextless" (
	collector: ^Ray_Query_Collector, hit: Ray_Query_Hit,
) -> Physics_Status
{
	if collector == nil || collector.hits.memory == nil
	{
		return .Invalid_Argument;
	}
	if collector.mode == .Earliest
	{
		if collector.count == 0
		{
			collector.hits.memory[0] = hit;
			collector.count = 1;
		}
		else if hit.t < collector.hits.memory[0].t
		{
			collector.hits.memory[0] = hit;
		}
		return .Ok;
	}
	if collector.count >= int(collector.hits.length)
	{
		return .Capacity_Missing;
	}
	collector.hits.memory[collector.count] = hit;
	collector.count += 1;
	return .Ok;
}

ray_query_collector_report :: proc "contextless" (
	collector: ^Ray_Query_Collector, ray: Tree_Ray, hit: Ray_Query_Hit, maximum_t: ^f32,
) -> Physics_Status
{
	if maximum_t == nil
	{
		return .Invalid_Argument;
	}
	status := ray_query_collector_add(collector, hit);
	if status != .Ok
	{
		return status;
	}
	if collector.mode == .Earliest
	{
		maximum_t^ = min(maximum_t^, hit.t);
	}
	if collector.callbacks.hit != nil
	{
		callback_maximum := maximum_t^;
		callback_hit := hit;
		status = collector.callbacks.hit(collector.callbacks.user_context, ray, &callback_hit, &callback_maximum);
		if status != .Ok
		{
			return status;
		}
		maximum_t^ = min(maximum_t^, callback_maximum);
	}
	return .Ok;
}

query_ray_tree_traverse :: #force_inline proc "contextless" (
	tree: ^Tree, ray: Tree_Ray, leaf: Tree_Ray_Leaf_Proc, user_context: rawptr,
	collector: ^Ray_Query_Collector, pool: ^util.Buffer_Pool,
) -> (f32, Physics_Status)
{
	if collector.mode == .Earliest || collector.callbacks.hit != nil
	{
		return tree_ray_traverse_closest(tree, ray, leaf, user_context, pool);
	}
	return tree_ray_traverse(tree, ray, leaf, user_context, pool);
}

Query_Compound_Ray_Context :: struct
{
	shapes:    ^Shape_Registry,
	children:  util.Buffer(Compound_Child),
	pose:      Rigid_Pose,
	ray:       Tree_Ray,
	target_id: i32,
	ray_id:    i32,
	collector: ^Ray_Query_Collector,
}

query_compound_ray_leaf :: proc "contextless" (
	user_context: rawptr, leaf_index: int, maximum_t: ^f32,
) -> Physics_Status
{
	query_context := (^Query_Compound_Ray_Context)(user_context);
	if query_context == nil || maximum_t == nil || leaf_index < 0 || leaf_index >= int(query_context.children.length)
	{
		return .Invalid_Argument;
	}
	if query_filter_child(
		query_context.collector.callbacks.filter, query_context.collector.callbacks.user_context,
		query_context.target_id, i32(leaf_index),
	) != .Allow
	{
		return .Ok;
	}
	child := query_context.children.memory[leaf_index];
	child_pose := rigid_pose_concatenate(
		{orientation=child.local_orientation, position=child.local_position}, query_context.pose,
	);
	ray := query_context.ray;
	ray.maximum_t = maximum_t^;
	hit, status := shape_registry_ray_test(query_context.shapes, child.shape_index, child_pose, ray);
	if status != .Ok
	{
		return status;
	}
	if hit.state == .Present
	{
		status = ray_query_collector_report(query_context.collector, ray, {
				t=hit.t,
				location=util.vector3_add(ray.origin, util.vector3_scale(ray.direction, hit.t)),
				normal=hit.normal,
				target_id=query_context.target_id,
				child_index=i32(leaf_index),
				ray_id=query_context.ray_id,
			}, maximum_t);
		if status != .Ok
		{
			return status;
		}
	}
	return .Ok;
}

Query_Mesh_Ray_Context :: struct
{
	mesh:      ^Mesh,
	pose:      Rigid_Pose,
	world_ray: Tree_Ray,
	local_ray: Tree_Ray,
	target_id: i32,
	ray_id:    i32,
	collector: ^Ray_Query_Collector,
}

query_mesh_ray_leaf :: proc "contextless" (
	user_context: rawptr, leaf_index: int, maximum_t: ^f32,
) -> Physics_Status
{
	query_context := (^Query_Mesh_Ray_Context)(user_context);
	if query_context == nil ||
	maximum_t == nil ||
	leaf_index < 0 ||
	leaf_index >= int(query_context.mesh.triangles.length)
	{
		return .Invalid_Argument;
	}
	if query_filter_child(
		query_context.collector.callbacks.filter, query_context.collector.callbacks.user_context,
		query_context.target_id, i32(leaf_index),
	) != .Allow
	{
		return .Ok;
	}
	triangle := query_context.mesh.triangles.memory[leaf_index];
	a := util.vector3_multiply(triangle.a, query_context.mesh.scale);
	b := util.vector3_multiply(triangle.b, query_context.mesh.scale);
	c := util.vector3_multiply(triangle.c, query_context.mesh.scale);
	t, local_normal, state := triangle_ray_test_local(
		a, b, c, query_context.local_ray.origin, query_context.local_ray.direction,
	);
	if state == .Present && t <= maximum_t^
	{
		status := ray_query_collector_report(query_context.collector, query_context.world_ray, {
				t=t,
				location=util.vector3_add(
					query_context.world_ray.origin, util.vector3_scale(query_context.world_ray.direction, t),
				),
				normal=util.quaternion_transform(local_normal, query_context.pose.orientation),
				target_id=query_context.target_id,
				child_index=i32(leaf_index),
				ray_id=query_context.ray_id,
			}, maximum_t);
		if status != .Ok
		{
			return status;
		}
	}
	return .Ok;
}

query_ray_shape_with_maximum_filtered :: proc "contextless" (
	shapes: ^Shape_Registry, target: Shape_Query_Target, ray: Tree_Ray,
	collector: ^Ray_Query_Collector, maximum_t: ^f32, ray_id: i32,
	target_filter_applied: Reference_State, pool: ^util.Buffer_Pool,
) -> Physics_Status
{
	if shapes == nil || collector == nil || maximum_t == nil
	{
		return .Invalid_Argument;
	}
	if target_filter_applied == .Missing && query_filter_target(
		collector.callbacks.filter, collector.callbacks.user_context, target.target_id,
	) != .Allow
	{
		return .Ok;
	}
	shape, batch, resolve_status := shape_registry_resolve(shapes, target.shape);
	if resolve_status != .Ok
	{
		return resolve_status;
	}
	type_id := int(typed_index_type(target.shape));
	if type_id == COMPOUND_TYPE_ID
	{
		children := (^Compound)(shape).children;
		for child_index in 0 ..< children.length
		{
			if query_filter_child(
				collector.callbacks.filter, collector.callbacks.user_context,
				target.target_id, i32(child_index),
			) != .Allow
			{
				continue;
			}
			child := children.memory[child_index];
			child_pose := rigid_pose_concatenate(
				{orientation=child.local_orientation, position=child.local_position}, target.pose,
			);
			bounded_ray := ray;
			bounded_ray.maximum_t = maximum_t^;
			hit, status := shape_registry_ray_test(shapes, child.shape_index, child_pose, bounded_ray);
			if status != .Ok
			{
				return status;
			}
			if hit.state == .Present
			{
				status = ray_query_collector_report(collector, bounded_ray, {
						t=hit.t,
						location=util.vector3_add(bounded_ray.origin, util.vector3_scale(bounded_ray.direction, hit.t)),
						normal=hit.normal,
						target_id=target.target_id,
						child_index=i32(child_index),
						ray_id=ray_id,
					}, maximum_t);
				if status != .Ok
				{
					return status;
				}
			}
		}
		return .Ok;
	}
	if type_id == BIG_COMPOUND_TYPE_ID
	{
		compound := (^Big_Compound)(shape);
		inverse_orientation := util.quaternion_conjugate(target.pose.orientation);
		local_ray := Tree_Ray{
			origin=util.quaternion_transform(
				util.vector3_subtract(ray.origin, target.pose.position),
				inverse_orientation
			),
			direction=util.quaternion_transform(ray.direction, inverse_orientation),
			maximum_t=ray.maximum_t,
		};
		query_context := Query_Compound_Ray_Context{
			shapes=shapes, children=compound.children, pose=target.pose, ray=ray,
			target_id=target.target_id, ray_id=ray_id, collector=collector,
		};
		local_ray.maximum_t = maximum_t^;
		final_maximum, status := query_ray_tree_traverse(
			&compound.tree, local_ray, query_compound_ray_leaf, &query_context, collector, pool,
		);
		maximum_t^ = min(maximum_t^, final_maximum);
		return status;
	}
	if type_id == MESH_TYPE_ID
	{
		mesh := (^Mesh)(shape);
		inverse_orientation := util.quaternion_conjugate(target.pose.orientation);
		local_ray := Tree_Ray{
			origin=util.quaternion_transform(
				util.vector3_subtract(ray.origin, target.pose.position),
				inverse_orientation
			),
			direction=util.quaternion_transform(ray.direction, inverse_orientation),
			maximum_t=ray.maximum_t,
		};
		query_context := Query_Mesh_Ray_Context{
			mesh=mesh, pose=target.pose, world_ray=ray, local_ray=local_ray,
			target_id=target.target_id, ray_id=ray_id, collector=collector,
		};
		local_ray.maximum_t = maximum_t^;
		final_maximum, status := query_ray_tree_traverse(
			&mesh.tree, local_ray, query_mesh_ray_leaf, &query_context, collector, pool,
		);
		maximum_t^ = min(maximum_t^, final_maximum);
		return status;
	}
	if query_filter_child(
		collector.callbacks.filter, collector.callbacks.user_context, target.target_id, 0,
	) != .Allow
	{
		return .Ok;
	}
	bounded_ray := ray;
	bounded_ray.maximum_t = maximum_t^;
	// reuse the resolved slot and its registered procedure. query callbacks are read-only
	hit: Shape_Ray_Hit;
	status: Physics_Status;
	hit, status = shape_batch_ray_test(batch, shape, target.pose, bounded_ray, shapes);
	if status != .Ok
	{
		return status;
	}
	if hit.state == .Present
	{
		return ray_query_collector_report(collector, bounded_ray, {
				t=hit.t,
				location=util.vector3_add(bounded_ray.origin, util.vector3_scale(bounded_ray.direction, hit.t)),
				normal=hit.normal,
				target_id=target.target_id,
				child_index=hit.child_index,
				ray_id=ray_id,
			}, maximum_t);
	}
	return .Ok;
}

query_ray_shape_with_maximum :: proc "contextless" (
	shapes: ^Shape_Registry, target: Shape_Query_Target, ray: Tree_Ray,
	collector: ^Ray_Query_Collector, maximum_t: ^f32, ray_id: i32,
	pool: ^util.Buffer_Pool,
) -> Physics_Status
{
	return query_ray_shape_with_maximum_filtered(
		shapes, target, ray, collector, maximum_t, ray_id, .Missing, pool,
	);
}

query_ray_shape :: proc "contextless" (
	shapes: ^Shape_Registry, target: Shape_Query_Target, ray: Tree_Ray,
	collector: ^Ray_Query_Collector, pool: ^util.Buffer_Pool,
) -> Physics_Status
{
	maximum_t := ray.maximum_t;
	return query_ray_shape_with_maximum(
		shapes, target, ray, collector, &maximum_t, 0, pool,
	);
}

query_ray_targets :: proc "contextless" (
	shapes: ^Shape_Registry, tree: ^Tree, targets: util.Buffer(Shape_Query_Target), ray: Tree_Ray,
	collector: ^Ray_Query_Collector, pool: ^util.Buffer_Pool,
) -> Physics_Status
{
	if shapes == nil || tree == nil || targets.memory == nil || int(targets.length) < tree.leaf_count ||
	collector == nil || shape_ray_valid(ray) != .Ok
	{
		return .Invalid_Argument;
	}
	resolver_context := Query_Buffer_Target_Context{targets=targets};
	_, status := query_ray_resolved(
		shapes, tree, {user_context=&resolver_context, resolve=query_buffer_target_resolve}, ray, collector,
		pool,
	);
	return status;
}

Query_Ray_Tree_Context :: struct
{
	shapes:    ^Shape_Registry,
	resolver:  Query_Target_Resolver,
	ray:       Tree_Ray,
	collector: ^Ray_Query_Collector,
	pool:      ^util.Buffer_Pool,
}

query_ray_tree_leaf :: proc "contextless" (
	user_context: rawptr, leaf_index: int, maximum_t: ^f32,
) -> Physics_Status
{
	query_context := (^Query_Ray_Tree_Context)(user_context);
	if query_context == nil || maximum_t == nil || leaf_index < 0
	{
		return .Invalid_Argument;
	}
	target, resolve_status := query_context.resolver.resolve(
		query_context.resolver.user_context, leaf_index,
	);
	if resolve_status != .Ok
	{
		return resolve_status;
	}
	return query_ray_shape_with_maximum_filtered(
		query_context.shapes, target, query_context.ray, query_context.collector,
		maximum_t, 0, .Missing, query_context.pool,
	);
}

query_ray_resolved_validated :: #force_inline proc "contextless" (
	shapes: ^Shape_Registry, tree: ^Tree, resolver: Query_Target_Resolver, ray: Tree_Ray,
	collector: ^Ray_Query_Collector, pool: ^util.Buffer_Pool,
) -> (f32, Physics_Status)
{
	query_context := Query_Ray_Tree_Context{
		shapes=shapes, resolver=resolver, ray=ray, collector=collector, pool=pool,
	};
	return query_ray_tree_traverse(
		tree, ray, query_ray_tree_leaf, &query_context, collector, pool,
	);
}

query_ray_resolved :: proc "contextless" (
	shapes: ^Shape_Registry, tree: ^Tree, resolver: Query_Target_Resolver, ray: Tree_Ray,
	collector: ^Ray_Query_Collector, pool: ^util.Buffer_Pool,
) -> (f32, Physics_Status)
{
	if shapes == nil || tree == nil || query_target_resolver_valid(resolver) != .Present ||
	collector == nil || shape_ray_valid(ray) != .Ok
	{
		return ray.maximum_t, .Invalid_Argument;
	}
	return query_ray_resolved_validated(
		shapes, tree, resolver, ray, collector, pool,
	);
}

wide_ray_test_shape :: proc "contextless" (
	shapes: ^Shape_Registry, target: Shape_Query_Target, query: Wide_Ray_Query,
) -> Wide_Ray_Result
{
	result: Wide_Ray_Result;
	if shapes == nil || query.lane_count < 0 || query.lane_count > util.PRODUCTION_LANE_COUNT
	{
		for lane in 0 ..< util.PRODUCTION_LANE_COUNT
		{
			result.status[lane] = .Invalid_Argument;
		}
		return result;
	}
	shape, _, resolve_status := shape_registry_resolve(shapes, target.shape);
	if resolve_status != .Ok
	{
		for lane in 0 ..< util.PRODUCTION_LANE_COUNT
		{
			result.status[lane] = resolve_status;
		}
		return result;
	}
	switch typed_index_type(target.shape)
	{
		case SPHERE_TYPE_ID:
		return wide_ray_test_sphere((^Sphere)(shape)^, target.pose, query);
		case CAPSULE_TYPE_ID:
		return wide_ray_test_capsule((^Capsule)(shape)^, target.pose, query);
		case BOX_TYPE_ID:
		return wide_ray_test_box((^Box)(shape)^, target.pose, query);
		case TRIANGLE_TYPE_ID:
		return wide_ray_test_triangle((^Triangle)(shape)^, target.pose, query);
		case CYLINDER_TYPE_ID:
		return wide_ray_test_cylinder((^Cylinder)(shape)^, target.pose, query);
		case CONVEX_HULL_TYPE_ID:
		return wide_ray_test_convex_hull((^Convex_Hull)(shape), target.pose, query);
	}
	for lane in 0 ..< query.lane_count
	{
		hit, status := shape_registry_ray_test(shapes, target.shape, target.pose, query.rays[lane]);
		result.hits[lane] = hit;
		result.status[lane] = status;
	}
	for lane in query.lane_count ..< util.PRODUCTION_LANE_COUNT
	{
		result.hits[lane] = shape_ray_miss();
		result.status[lane] = .Ok;
	}
	return result;
}

wide_ray_test_sphere :: proc "contextless" (
	sphere: Sphere, pose: Rigid_Pose, query: Wide_Ray_Query,
) -> Wide_Ray_Result
{
	result: Wide_Ray_Result;
	if sphere_validate(sphere) != .Ok || query.lane_count < 0 || query.lane_count > util.PRODUCTION_LANE_COUNT
	{
		for lane in 0 ..< util.PRODUCTION_LANE_COUNT
		{
			result.status[lane] = .Invalid_Argument;
		}
		return result;
	}
	origin, direction: util.Vector3_Wide;
	maximum_t: util.F32x8;
	active: util.I32x8;
	for lane in 0 ..< query.lane_count
	{
		ray := query.rays[lane];
		status := shape_ray_valid(ray);
		result.status[lane] = status;
		if status != .Ok
		{
			continue;
		}
		util.vector3_wide_write_slot(&origin, lane, util.vector3_subtract(ray.origin, pose.position));
		util.vector3_wide_write_slot(&direction, lane, ray.direction);
		maximum_t = simd.replace(maximum_t, lane, ray.maximum_t);
		active = simd.replace(active, lane, -1);
	}
	direction_length := util.vector3_wide_length(direction);
	inverse_direction_length := simd.div(util.F32x8(1), direction_length);
	normalized_direction := util.vector3_wide_scale(direction, inverse_direction_length);
	t_offset := simd.max(
		util.F32x8(0),
		simd.sub(simd.neg(util.vector3_wide_dot(origin, normalized_direction)), util.F32x8(sphere.radius)),
	);
	shifted_origin := util.vector3_wide_add(origin, util.vector3_wide_scale(normalized_direction, t_offset));
	b := util.vector3_wide_dot(shifted_origin, normalized_direction);
	c := simd.sub(util.vector3_wide_dot(shifted_origin, shifted_origin), util.F32x8(sphere.radius * sphere.radius));
	discriminant := simd.sub(simd.mul(b, b), c);
	local_t := simd.max(
		simd.sub(simd.neg(b), simd.sqrt(simd.max(util.F32x8(0), discriminant))),
		simd.neg(t_offset),
	);
	t := simd.mul(simd.add(local_t, t_offset), inverse_direction_length);
	normal := util.vector3_wide_scale(
		util.vector3_wide_add(shifted_origin, util.vector3_wide_scale(normalized_direction, local_t)),
		util.F32x8(1 / sphere.radius),
	);
	hit_mask := active &
	~(transmute(util.I32x8)simd.lanes_gt(b, util.F32x8(0)) & transmute(util.I32x8)simd.lanes_gt(c, util.F32x8(0))) &
	transmute(util.I32x8)simd.lanes_ge(discriminant, util.F32x8(0)) &
	transmute(util.I32x8)simd.lanes_ge(t, util.F32x8(0)) &
	transmute(util.I32x8)simd.lanes_le(t, maximum_t);
	for lane in 0 ..< query.lane_count
	{
		if simd.extract(hit_mask, lane) < 0
		{
			result.hits[lane] = {
				t=simd.extract(t, lane), normal=util.vector3_wide_read_slot(normal, lane),
				child_index=0, state=.Present,
			};
		}
		else
		{
			result.hits[lane] = shape_ray_miss();
		}
	}
	for lane in query.lane_count ..< util.PRODUCTION_LANE_COUNT
	{
		result.hits[lane] = shape_ray_miss();
		result.status[lane] = .Ok;
	}
	return result;
}

ray_batcher_initialize :: proc "contextless" (
	batcher: ^Ray_Batcher, storage: util.Buffer(Ray_Batcher_Request), pool: ^util.Buffer_Pool,
) -> Physics_Status
{
	if batcher == nil || storage.memory == nil || storage.length <= 0 ||
	pool == nil || pool.state != .Ready
	{
		return .Invalid_Argument;
	}
	batcher^ = {requests=storage, pool=pool, state=.Ready};
	return .Ok;
}

ray_batcher_add :: proc "contextless" (
	batcher: ^Ray_Batcher, target: Shape_Query_Target, ray: Tree_Ray, ray_id: i32 = 0,
) -> Physics_Status
{
	if batcher == nil || batcher.state != .Ready || shape_ray_valid(ray) != .Ok
	{
		return .Invalid_Argument;
	}
	if batcher.count >= int(batcher.requests.length)
	{
		return .Capacity_Missing;
	}
	batcher.requests.memory[batcher.count] = {target=target, ray=ray, ray_id=ray_id};
	batcher.count += 1;
	return .Ok;
}

ray_batcher_targets_match :: proc "contextless" (a, b: Shape_Query_Target) -> Reference_State
{
	if a.shape != b.shape
	{
		return .Missing;
	}
	if a.pose.position.x != b.pose.position.x || a.pose.position.y != b.pose.position.y ||
	a.pose.position.z != b.pose.position.z ||
	a.pose.orientation.x != b.pose.orientation.x || a.pose.orientation.y != b.pose.orientation.y ||
	a.pose.orientation.z != b.pose.orientation.z || a.pose.orientation.w != b.pose.orientation.w
	{
		return .Missing;
	}
	return .Present;
}

ray_batcher_flush :: proc "contextless" (
	batcher: ^Ray_Batcher, shapes: ^Shape_Registry, collector: ^Ray_Query_Collector,
) -> Physics_Status
{
	if batcher == nil || batcher.state != .Ready || shapes == nil || collector == nil
	{
		return .Invalid_Argument;
	}
	for start := 0; start < batcher.count;
	{
		first := batcher.requests.memory[start];
		type_id := int(typed_index_type(first.target.shape));
		if type_id >= COMPOUND_TYPE_ID
		{
			status := query_ray_shape_with_maximum(
				shapes, first.target, first.ray, collector,
				&batcher.requests.memory[start].ray.maximum_t, first.ray_id, batcher.pool,
			);
			if status != .Ok
			{
				return status;
			}
			start += 1;
			continue;
		}
		query: Wide_Ray_Query;
		request_indices: [util.PRODUCTION_LANE_COUNT]int;
		for request_index := start;
		request_index < batcher.count && query.lane_count < util.PRODUCTION_LANE_COUNT;
		request_index += 1
		{
			request := batcher.requests.memory[request_index];
			if ray_batcher_targets_match(first.target, request.target) == .Missing
			{
				break;
			}
			lane := query.lane_count;
			query.rays[lane] = request.ray;
			request_indices[lane] = request_index;
			query.lane_count += 1;
		}
		if query_filter_target(
			collector.callbacks.filter, collector.callbacks.user_context, first.target.target_id,
		) != .Allow || query_filter_child(
			collector.callbacks.filter, collector.callbacks.user_context, first.target.target_id, 0,
		) != .Allow
		{
			start += query.lane_count;
			continue;
		}
		wide := wide_ray_test_shape(shapes, first.target, query);
		for lane in 0 ..< query.lane_count
		{
			if wide.status[lane] != .Ok
			{
				return wide.status[lane];
			}
			if wide.hits[lane].state == .Missing
			{
				continue;
			}
			request := &batcher.requests.memory[request_indices[lane]];
			status := ray_query_collector_report(collector, request.ray, {
					t=wide.hits[lane].t,
					location=util.vector3_add(
						request.ray.origin, util.vector3_scale(request.ray.direction, wide.hits[lane].t),
					),
					normal=wide.hits[lane].normal,
					target_id=request.target.target_id,
					child_index=wide.hits[lane].child_index,
					ray_id=request.ray_id,
				}, &request.ray.maximum_t);
			if status != .Ok
			{
				return status;
			}
		}
		start += query.lane_count;
	}
	for index in 0 ..< batcher.count
	{
		batcher.requests.memory[index] = {};
	}
	batcher.count = 0;
	return .Ok;
}

query_overlap :: proc "contextless" (
	query_shape: Typed_Index, query_pose: Rigid_Pose,
	tree: ^Tree, targets: util.Buffer(Shape_Query_Target),
	shapes: ^Shape_Registry, collision_tasks: ^Collision_Task_Registry,
	collector: ^Overlap_Query_Collector, pool: ^util.Buffer_Pool,
	batcher: ^Query_Overlap_Batcher = nil,
) -> Physics_Status
{
	if tree == nil ||
	targets.memory == nil ||
	int(targets.length) < tree.leaf_count ||
	shapes == nil ||
	collision_tasks == nil ||
	collector == nil ||
	collector.hits.memory == nil
	{
		return .Invalid_Argument;
	}
	resolver_context := Query_Buffer_Target_Context{targets=targets};
	return query_overlap_resolved(
		query_shape, query_pose, tree,
		{user_context=&resolver_context, resolve=query_buffer_target_resolve},
		shapes, collision_tasks, collector, pool, batcher,
	);
}

query_overlap_resolved :: proc "contextless" (
	query_shape: Typed_Index, query_pose: Rigid_Pose,
	tree: ^Tree, resolver: Query_Target_Resolver,
	shapes: ^Shape_Registry, collision_tasks: ^Collision_Task_Registry,
	collector: ^Overlap_Query_Collector, pool: ^util.Buffer_Pool,
	batcher: ^Query_Overlap_Batcher = nil,
) -> Physics_Status
{
	if tree == nil || query_target_resolver_valid(resolver) != .Present || shapes == nil ||
	collision_tasks == nil || collector == nil || collector.hits.memory == nil
	{
		return .Invalid_Argument;
	}
	query_raw, _, query_status := shape_registry_resolve(shapes, query_shape);
	if query_status != .Ok
	{
		return query_status;
	}
	query_bounds, bounds_status := shape_registry_compute_world_bounds(shapes, query_shape, query_pose);
	if bounds_status != .Ok
	{
		return bounds_status;
	}
	query_context := Query_Overlap_Tree_Context{
		query_shape=query_shape, query_raw=query_raw, query_pose=query_pose,
		resolver=resolver, shapes=shapes, collision_tasks=collision_tasks,
		collector=collector, pool=pool, batcher=batcher,
	};
	if batcher != nil
	{
		if batcher.state != .Ready || batcher.collision == nil ||
		batcher.collision.state != .Ready
		{
			return .Invalid_Argument;
		}
		previous_pool := batcher.collision.traversal_pool;
		batcher.collision.traversal_pool = pool;
		defer batcher.collision.traversal_pool = previous_pool
		return tree_volume_traverse(
			tree, query_bounds, query_overlap_tree_leaf, &query_context, query_context.pool,
		);
	}
	return tree_volume_traverse(
		tree, query_bounds, query_overlap_tree_leaf, &query_context, query_context.pool,
	);
}

Query_Overlap_Tree_Context :: struct
{
	query_shape:     Typed_Index,
	query_raw:       rawptr,
	query_pose:      Rigid_Pose,
	resolver:        Query_Target_Resolver,
	shapes:          ^Shape_Registry,
	collision_tasks: ^Collision_Task_Registry,
	collector:       ^Overlap_Query_Collector,
	pool:            ^util.Buffer_Pool,
	batcher:         ^Query_Overlap_Batcher,
}

query_overlap_tree_leaf :: proc "contextless" (user_context: rawptr, leaf_index: int) -> Physics_Status
{
	query_context := (^Query_Overlap_Tree_Context)(user_context);
	if query_context == nil || leaf_index < 0
	{
		return .Invalid_Argument;
	}
	target, resolve_status := query_context.resolver.resolve(query_context.resolver.user_context, leaf_index);
	if resolve_status != .Ok
	{
		return resolve_status;
	}
	if query_filter_target(
		query_context.collector.filter, query_context.collector.user_context, target.target_id,
	) != .Allow
	{
		return .Ok;
	}
	target_raw, _, target_status := shape_registry_resolve(query_context.shapes, target.shape);
	if target_status != .Ok
	{
		return target_status;
	}
	query_type := int(typed_index_type(query_context.query_shape));
	target_type := int(typed_index_type(target.shape));
	task, _, lookup_status := collision_task_registry_lookup(query_context.collision_tasks, query_type, target_type);
	if lookup_status != .Ok
	{
		return lookup_status;
	}
	if task.kind == .Convex
	{
		manifold, status := collision_task_registry_test_convex(
			query_context.collision_tasks, query_type, target_type, query_context.query_raw, target_raw,
			query_context.query_pose, target.pose, 0, query_context.shapes,
		);
		if status != .Ok
		{
			return status;
		}
		if manifold.count > 0
		{
			if query_context.collector.count >= int(query_context.collector.hits.length)
			{
				return .Capacity_Missing;
			}
			hit: ^Overlap_Query_Hit = &query_context.collector.hits.memory[query_context.collector.count];
			hit.manifold.kind = .Convex;
			hit.manifold.convex = manifold;
			hit.manifold.nonconvex = {};
			hit.target_id = target.target_id;
			query_context.collector.count += 1;
		}
		return .Ok;
	}
	if query_context.batcher == nil || query_context.batcher.state != .Ready
	{
		return .Capacity_Missing;
	}
	return query_overlap_batcher_test(
		query_context.batcher, query_context.query_shape, target.shape,
		query_context.query_pose, target.pose, target.target_id, query_context.collector,
	);
}

query_volume :: proc "contextless" (
	volume: util.Bounding_Box, tree: ^Tree, targets: util.Buffer(Shape_Query_Target),
	shapes: ^Shape_Registry, collector: ^Volume_Query_Collector,
	pool: ^util.Buffer_Pool,
) -> Physics_Status
{
	if tree == nil ||
	targets.memory == nil ||
	int(targets.length) < tree.leaf_count ||
	shapes == nil ||
	collector == nil ||
	collector.hits.memory == nil ||
	volume.min.x > volume.max.x || volume.min.y > volume.max.y || volume.min.z > volume.max.z
	{
		return .Invalid_Argument;
	}
	resolver_context := Query_Buffer_Target_Context{targets=targets};
	return query_volume_resolved(
		volume, tree, {user_context=&resolver_context, resolve=query_buffer_target_resolve},
		shapes, collector, pool,
	);
}

query_volume_resolved :: proc "contextless" (
	volume: util.Bounding_Box, tree: ^Tree, resolver: Query_Target_Resolver,
	shapes: ^Shape_Registry, collector: ^Volume_Query_Collector,
	pool: ^util.Buffer_Pool,
) -> Physics_Status
{
	if tree == nil || query_target_resolver_valid(resolver) != .Present || shapes == nil ||
	collector == nil || collector.hits.memory == nil ||
	volume.min.x > volume.max.x || volume.min.y > volume.max.y || volume.min.z > volume.max.z
	{
		return .Invalid_Argument;
	}
	query_context := Query_Volume_Tree_Context{resolver=resolver, collector=collector};
	return tree_volume_traverse(
		tree, volume, query_volume_tree_leaf, &query_context, pool,
	);
}

Query_Volume_Tree_Context :: struct
{
	resolver:  Query_Target_Resolver,
	collector: ^Volume_Query_Collector,
}

query_volume_tree_leaf :: proc "contextless" (user_context: rawptr, leaf_index: int) -> Physics_Status
{
	query_context := (^Query_Volume_Tree_Context)(user_context);
	if query_context == nil || leaf_index < 0
	{
		return .Invalid_Argument;
	}
	target, status := query_context.resolver.resolve(query_context.resolver.user_context, leaf_index);
	if status != .Ok
	{
		return status;
	}
	if query_filter_target(
		query_context.collector.filter, query_context.collector.user_context, target.target_id,
	) != .Allow
	{
		return .Ok;
	}
	if query_context.collector.count >= int(query_context.collector.hits.length)
	{
		return .Capacity_Missing;
	}
	query_context.collector.hits.memory[query_context.collector.count] = {target.target_id};
	query_context.collector.count += 1;
	return .Ok;
}

query_sweep_targets :: proc "contextless" (
	query_shape: Typed_Index, query_pose: Rigid_Pose, query_velocity: Body_Velocity,
	tree: ^Tree, targets: util.Buffer(Shape_Query_Target),
	maximum_t, minimum_progression, convergence_threshold: f32, maximum_iteration_count: int,
	shapes: ^Shape_Registry, collision_tasks: ^Collision_Task_Registry, sweep_tasks: ^Sweep_Task_Registry,
	collector: ^Sweep_Query_Collector, pool: ^util.Buffer_Pool,
) -> Physics_Status
{
	if tree == nil ||
	targets.memory == nil ||
	int(targets.length) < tree.leaf_count ||
	shapes == nil ||
	collision_tasks == nil ||
	sweep_tasks == nil ||
	collector == nil || collector.hits.memory == nil
	{
		return .Invalid_Argument;
	}
	resolver_context := Query_Buffer_Target_Context{targets=targets};
	_, status := query_sweep_resolved(
		query_shape, query_pose, query_velocity, tree,
		{user_context=&resolver_context, resolve=query_buffer_target_resolve},
		maximum_t, minimum_progression, convergence_threshold, maximum_iteration_count,
		shapes, collision_tasks, sweep_tasks, collector, pool,
	);
	return status;
}

query_sweep_resolved :: proc "contextless" (
	query_shape: Typed_Index, query_pose: Rigid_Pose, query_velocity: Body_Velocity,
	tree: ^Tree, resolver: Query_Target_Resolver,
	maximum_t, minimum_progression, convergence_threshold: f32, maximum_iteration_count: int,
	shapes: ^Shape_Registry, collision_tasks: ^Collision_Task_Registry, sweep_tasks: ^Sweep_Task_Registry,
	collector: ^Sweep_Query_Collector, pool: ^util.Buffer_Pool,
) -> (f32, Physics_Status)
{
	if tree == nil || query_target_resolver_valid(resolver) != .Present || shapes == nil ||
	collision_tasks == nil || sweep_tasks == nil || collector == nil || collector.hits.memory == nil
	{
		return maximum_t, .Invalid_Argument;
	}
	shape_bounds, bounds_status := shape_registry_compute_bounds(shapes, query_shape, query_pose.orientation);
	if bounds_status != .Ok
	{
		return maximum_t, bounds_status;
	}
	query_context := Query_Sweep_Tree_Context{
		query_shape=query_shape, query_pose=query_pose, query_velocity=query_velocity,
		resolver=resolver, maximum_t=maximum_t, minimum_progression=minimum_progression,
		convergence_threshold=convergence_threshold, maximum_iteration_count=maximum_iteration_count,
		shapes=shapes, collision_tasks=collision_tasks, sweep_tasks=sweep_tasks,
		collector=collector, pool=pool,
	};
	return query_sweep_prepared(tree, &shape_bounds, &query_context, maximum_t);
}

// shape bounds and query state are query-local, not recreated for each tree.
// recompute angular expansion using the current closest time for every traversal
@(private="package")
query_sweep_prepared :: proc "contextless" (
	tree: ^Tree, shape_bounds: ^Shape_Bounds,
	query_context: ^Query_Sweep_Tree_Context, maximum_t: f32,
) -> (f32, Physics_Status)
{
	query_bounds := util.Bounding_Box{
		min=util.vector3_add(shape_bounds.min, query_context.query_pose.position),
		max=util.vector3_add(shape_bounds.max, query_context.query_pose.position),
	};
	angular_expansion := min(
		shape_bounds.maximum_angular_expansion,
		maximum_t * util.vector3_length(query_context.query_velocity.angular) * shape_bounds.maximum_radius,
	);
	if angular_expansion > 0
	{
		expansion := util.Vector3{angular_expansion, angular_expansion, angular_expansion};
		query_bounds.min = util.vector3_subtract(query_bounds.min, expansion);
		query_bounds.max = util.vector3_add(query_bounds.max, expansion);
	}
	collector := query_context.collector;
	if collector.mode == .Earliest || collector.callbacks.hit != nil ||
	collector.callbacks.hit_at_zero != nil
	{
		return tree_sweep_traverse_closest(
			tree, query_bounds, query_context.query_velocity.linear, maximum_t,
			query_sweep_tree_leaf, query_context, query_context.pool,
		);
	}
	// do not merge the all-hit traversal stack into the closest-query frame
	return #force_no_inline tree_sweep_traverse(
		tree, query_bounds, query_context.query_velocity.linear, maximum_t,
		query_sweep_tree_leaf, query_context, query_context.pool,
	);
}

Query_Sweep_Tree_Context :: struct
{
	query_shape:             Typed_Index,
	query_pose:              Rigid_Pose,
	query_velocity:          Body_Velocity,
	resolver:                Query_Target_Resolver,
	maximum_t:               f32,
	minimum_progression:     f32,
	convergence_threshold:   f32,
	maximum_iteration_count: int,
	shapes:                  ^Shape_Registry,
	collision_tasks:         ^Collision_Task_Registry,
	sweep_tasks:             ^Sweep_Task_Registry,
	collector:               ^Sweep_Query_Collector,
	pool:                    ^util.Buffer_Pool,
	current_target_id:       i32,
}

query_sweep_child_filter :: proc "contextless" (
	user_context: rawptr, _pair_id, _child_a, child_b: i32,
) -> Collision_Testing_State
{
	query_context := (^Query_Sweep_Tree_Context)(user_context);
	if query_context == nil
	{
		return .Reject;
	}
	if query_filter_child(
		query_context.collector.callbacks.filter, query_context.collector.callbacks.user_context,
		query_context.current_target_id, child_b,
	) != .Allow
	{
		return .Reject;
	}
	return .Allow;
}

query_sweep_tree_leaf :: proc "contextless" (
	user_context: rawptr, leaf_index: int, maximum_t: ^f32,
) -> Physics_Status
{
	query_context := (^Query_Sweep_Tree_Context)(user_context);
	if query_context == nil || maximum_t == nil || leaf_index < 0
	{
		return .Invalid_Argument;
	}
	target, resolve_status := query_context.resolver.resolve(query_context.resolver.user_context, leaf_index);
	if resolve_status != .Ok
	{
		return resolve_status;
	}
	if query_filter_target(
		query_context.collector.callbacks.filter, query_context.collector.callbacks.user_context, target.target_id,
	) != .Allow
	{
		return .Ok;
	}
	query_context.current_target_id = target.target_id;
	result, status := sweep_task_registry_test(
		query_context.sweep_tasks, query_context.query_shape, target.shape,
		query_context.query_pose, target.pose, query_context.query_velocity, {},
		maximum_t^, query_context.minimum_progression, query_context.convergence_threshold,
		query_context.maximum_iteration_count, query_context.shapes, query_context.collision_tasks,
		query_sweep_child_filter, query_context, query_context.pool,
	);
	if status == .Not_Found
	{
		return .Ok;
	}
	if status != .Ok
	{
		return status;
	}
	if result.state == .Hit
	{
		hit := Sweep_Query_Hit{target_id=target.target_id, sweep=result};
		if query_context.collector.mode == .Earliest
		{
			if query_context.collector.count == 0 || result.t1 < query_context.collector.hits.memory[0].sweep.t1
			{
				query_context.collector.hits.memory[0] = hit;
				query_context.collector.count = 1;
				maximum_t^ = result.t1;
			}
		}
		else
		{
			if query_context.collector.count >= int(query_context.collector.hits.length)
			{
				return .Capacity_Missing;
			}
			query_context.collector.hits.memory[query_context.collector.count] = hit;
			query_context.collector.count += 1;
		}
		callback_maximum := maximum_t^;
		if result.t1 > 0
		{
			if query_context.collector.callbacks.hit != nil
			{
				status = query_context.collector.callbacks.hit(
					query_context.collector.callbacks.user_context, &hit, &callback_maximum,
				);
			}
		}
		else if query_context.collector.callbacks.hit_at_zero != nil
		{
			status = query_context.collector.callbacks.hit_at_zero(
				query_context.collector.callbacks.user_context, target.target_id, &callback_maximum,
			);
		}
		if status != .Ok
		{
			return status;
		}
		maximum_t^ = min(maximum_t^, callback_maximum);
	}
	return .Ok;
}

// separate any-hit execution preserves the old all-hit collector and hot paths.
// only a completed nonnegative-depth manifold terminates scene traversal. a
// task error is never interpreted as a hit, including Not_Found from a callback
Overlap_Query_State :: enum u8
{
	Separated,
	Intersecting,
}

Query_Overlap_Any_Context :: struct
{
	query_shape: Typed_Index,
	query_raw: rawptr,
	query_pose: Rigid_Pose,
	resolver: Query_Target_Resolver,
	shapes: ^Shape_Registry,
	collision_tasks: ^Collision_Task_Registry,
	filter: Query_Filter_Procedures,
	user_context: rawptr,
	batcher: ^Query_Overlap_Batcher,
	target_id: i32,
	target_raw: rawptr,
	target_type: int,
	child_status: Physics_Status,
	pair_state: Overlap_Query_State,
	state: Overlap_Query_State,
}

query_overlap_any_completed :: proc "contextless" (
	user_context: rawptr, _pair_id: i32, manifold: ^Manifold_Result,
) -> Physics_Status
{
	query: ^Query_Overlap_Any_Context = cast(^Query_Overlap_Any_Context)user_context;
	if manifold.kind == .Convex
	{
		for i: i32 = 0; i < manifold.convex.count; i += 1
		{
			if manifold.convex.contacts[i].depth >= 0
			{
				query.pair_state = .Intersecting;
				break;
			}
		}
	}
	else
	{
		for i: i32 = 0; i < manifold.nonconvex.count; i += 1
		{
			if manifold.nonconvex.contacts[i].depth >= 0
			{
				query.pair_state = .Intersecting;
				break;
			}
		}
	}
	return .Ok;
}

// child indices are supplied by the admitted compound traversal in original
// pair order. recover only the type ID. no geometry or pose is copied here
query_overlap_any_child_type :: #force_inline proc "contextless" (
	shape: rawptr, type_id, child_index: int,
) -> int
{
	switch type_id
	{
		case COMPOUND_TYPE_ID:
		return int(typed_index_type((cast(^Compound)shape).children.memory[child_index].shape_index));
		case BIG_COMPOUND_TYPE_ID:
		return int(typed_index_type((cast(^Big_Compound)shape).children.memory[child_index].shape_index));
		case MESH_TYPE_ID:
		return TRIANGLE_TYPE_ID;
	}
	return type_id;
}

query_overlap_any_allow_child :: proc "contextless" (
	user_context: rawptr, _pair_id, child_a, child_b: i32,
) -> Collision_Testing_State
{
	query: ^Query_Overlap_Any_Context = cast(^Query_Overlap_Any_Context)user_context;
	if query.child_status != .Ok || query_filter_child(query.filter, query.user_context, query.target_id, child_b) != .Allow
	{
		return .Reject;
	}
	type_a: int = query_overlap_any_child_type(query.query_raw, int(typed_index_type(query.query_shape)), int(child_a));
	type_b: int = query_overlap_any_child_type(query.target_raw, query.target_type, int(child_b));
	// the ordinary collision batcher intentionally treats an absent child route
	// as an empty contact. the any-query contract instead reports it explicitly,
	// without adding a branch to existing collision/solver processing
	_, _, query.child_status = collision_task_registry_lookup(query.collision_tasks, type_a, type_b);
	if query.child_status != .Ok
	{
		return .Reject;
	}
	return .Allow;
}

query_overlap_any_tree_leaf :: proc "contextless" (user_context: rawptr, leaf_index: int) -> Physics_Status
{
	query: ^Query_Overlap_Any_Context = cast(^Query_Overlap_Any_Context)user_context;
	target: Shape_Query_Target;
	status: Physics_Status;
	target, status = query.resolver.resolve(query.resolver.user_context, leaf_index);
	if status != .Ok
	{
		return status;
	}
	if query_filter_target(query.filter, query.user_context, target.target_id) != .Allow
	{
		return .Ok;
	}
	target_raw: rawptr;
	target_raw, _, status = shape_registry_resolve(query.shapes, target.shape);
	if status != .Ok
	{
		return status;
	}
	query_type: int = int(typed_index_type(query.query_shape));
	target_type: int = int(typed_index_type(target.shape));
	task: ^Collision_Task;
	task, _, status = collision_task_registry_lookup(query.collision_tasks, query_type, target_type);
	if status != .Ok
	{
		return status;
	}
	if task.kind == .Convex
	{
		manifold: Convex_Contact_Manifold;
		manifold, status = collision_task_registry_test_convex(query.collision_tasks, query_type, target_type,
			query.query_raw, target_raw, query.query_pose, target.pose, 0, query.shapes);
		if status != .Ok
		{
			return status;
		}
		for i: i32 = 0; i < manifold.count; i += 1
		{
			if manifold.contacts[i].depth >= 0
			{
				query.state = .Intersecting;
				return .Not_Found;
				// private traversal termination, paired with state
			}
		}
		return .Ok;
	}
	if query.batcher == nil || query.batcher.state != .Ready
	{
		return .Capacity_Missing;
	}
	batcher: ^Collision_Batcher = query.batcher.collision;
	if batcher == nil || batcher.state != .Ready || batcher.pair_count != 0
	{
		return .Invalid_Argument;
	}
	procedures: Collision_Result_Procedures = batcher.procedures;
	stored: Collision_Stored_Pair_Result_Proc = batcher.stored_pair_completed;
	previous_context: rawptr = batcher.user_context;
	defer
	{
		if batcher.state == .Faulted
		{
			_ = collision_batcher_reset_fault(batcher);
		}
		batcher.procedures = procedures;
		batcher.stored_pair_completed = stored;
		batcher.user_context = previous_context;
	}
	query.target_id = target.target_id;
	query.target_raw = target_raw;
	query.target_type = target_type;
	query.child_status = .Ok;
	query.pair_state = .Separated;
	batcher.procedures = {pair_completed=query_overlap_any_completed, allow_child_pair=query_overlap_any_allow_child};
	batcher.stored_pair_completed = nil;
	batcher.user_context = query;
	status = collision_batcher_add(batcher, query.query_shape, target.shape, query.query_pose, target.pose, 0, 0);
	if status == .Ok
	{
		status = collision_batcher_flush(batcher);
	}
	if status != .Ok
	{
		return status;
	}
	if query.child_status != .Ok
	{
		return query.child_status;
	}
	if query.pair_state == .Intersecting
	{
		query.state = .Intersecting;
		return .Not_Found;
	}
	return .Ok;
}

// resolve query geometry once, then reuse it across active and static trees.
// only a complete parent result ends traversal. no output list is constructed
query_overlap_any_resolved :: proc "contextless" (
	query_shape: Typed_Index, query_pose: Rigid_Pose,
	tree: ^Tree, resolver: Query_Target_Resolver,
	second_tree: ^Tree, second_resolver: Query_Target_Resolver,
	shapes: ^Shape_Registry, collision_tasks: ^Collision_Task_Registry,
	filter: Query_Filter_Procedures, user_context: rawptr,
	pool: ^util.Buffer_Pool, batcher: ^Query_Overlap_Batcher,
) -> (Overlap_Query_State, Physics_Status)
{
	query_raw: rawptr;
	status: Physics_Status;
	query_raw, _, status = shape_registry_resolve(shapes, query_shape);
	if status != .Ok
	{
		return .Separated, status;
	}
	bounds: util.Bounding_Box;
	bounds, status = shape_registry_compute_world_bounds(shapes, query_shape, query_pose);
	if status != .Ok
	{
		return .Separated, status;
	}
	query: Query_Overlap_Any_Context = {query_shape=query_shape, query_raw=query_raw, query_pose=query_pose,
		resolver=resolver, shapes=shapes, collision_tasks=collision_tasks, filter=filter, user_context=user_context, batcher=batcher};
	collision: ^Collision_Batcher;
	previous_pool: ^util.Buffer_Pool;
	if batcher != nil
	{
		collision = batcher.collision;
		if collision != nil
		{
			previous_pool = collision.traversal_pool;
			collision.traversal_pool = pool;
		}
	}
	defer
	{
		if collision != nil
		{
			collision.traversal_pool = previous_pool;
		}
	}
	status = tree_volume_traverse(tree, bounds, query_overlap_any_tree_leaf, &query, pool);
	if status == .Ok
	{
		query.resolver = second_resolver;
		status = tree_volume_traverse(second_tree, bounds, query_overlap_any_tree_leaf, &query, pool);
	}
	if query.state == .Intersecting && status == .Not_Found
	{
		return .Intersecting, .Ok;
	}
	return .Separated, status;
}
