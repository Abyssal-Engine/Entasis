package entasis

import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

// Collidable_Mask selects dynamic, kinematic, and static query targets.
// an empty Query_Filter.include mask is interpreted as COLLIDABLE_ALL so the
// zero value remains the ordinary unfiltered policy
Collidable_Mask :: distinct bit_set[Body_Mobility; u8];
// COLLIDABLE_DYNAMIC dynamic-body query targets
COLLIDABLE_DYNAMIC   :: Collidable_Mask{.Dynamic};
// COLLIDABLE_KINEMATIC kinematic-body query targets
COLLIDABLE_KINEMATIC :: Collidable_Mask{.Kinematic};
// COLLIDABLE_STATIC static query targets
COLLIDABLE_STATIC    :: Collidable_Mask{.Static};
// COLLIDABLE_BODIES dynamic and kinematic body query targets
COLLIDABLE_BODIES    :: Collidable_Mask{.Dynamic, .Kinematic};
// COLLIDABLE_ALL includes all dynamic, kinematic, and static query targets
COLLIDABLE_ALL       :: Collidable_Mask{.Dynamic, .Kinematic, .Static};
// Query_Allow_Proc optionally filters complete collidables after the mobility
// mask. the callback executes synchronously on the query caller thread
Query_Allow_Proc :: #type proc "contextless" (
	user_context: rawptr,
	collidable: Collidable_Reference,
) -> bool;
// Query_Allow_Child_Proc optionally filters compound children and mesh
// triangles. convex shapes use child index zero
Query_Allow_Child_Proc :: #type proc "contextless" (
	user_context: rawptr,
	collidable: Collidable_Reference,
	child_index: i32,
) -> bool;
// Query_Filter is allocation-free and caller owned. include={} means all
// mobilities. exclude is applied after include. custom callbacks are optional
Query_Filter :: struct
{
	include:      Collidable_Mask,
	exclude:      Collidable_Mask,
	allow:        Query_Allow_Proc,
	allow_child:  Query_Allow_Child_Proc,
	user_context: rawptr,
}

// Ray_Hit is layout-compatible with the low-level collector hit. the packed
// target ID is exposed directly as a Collidable_Reference without conversion
Ray_Hit :: struct
{
	t:           f32,
	location:    Vector3,
	normal:      Vector3,
	collidable:  Collidable_Reference,
	child_index: i32,
	ray_id:      i32,
}

// Sweep_Result is an alias for the low-level sweep result
Sweep_Result    :: physics.Sweep_Result;
// Sweep_Hit_State classifies the impact state returned by the low-level sweep kernel
Sweep_Hit_State :: physics.Sweep_Hit_State;
// Sweep_Hit is layout-compatible with the low-level sweep collector hit
Sweep_Hit :: struct
{
	collidable: Collidable_Reference,
	sweep:      Sweep_Result,
}

// Overlap_Hit includes the generated contact manifold for one collidable
Overlap_Hit :: struct
{
	collidable: Collidable_Reference,
	manifold:   Manifold_Result,
}

// Volume_Hit identifies one collidable whose broad-phase bounds overlap the
// requested axis-aligned volume. this is a broad-phase query, not a narrow-phase
// shape intersection
Volume_Hit :: struct
{
	collidable: Collidable_Reference,
}

// Sweep_Settings controls iterative sweep convergence. the zero value selects
// the scale-derived defaults
Sweep_Settings :: struct
{
	minimum_progression:     f32,
	convergence_threshold:   f32,
	maximum_iteration_count: i32,
}

// Query_Kind identifies one caller-owned batch query
Query_Kind :: enum u8
{
	Ray_Any,
	Ray_Closest,
	Ray_All,
	Sweep_Closest,
	Overlap_All,
	Volume_All,
}

// Query_Output selects a caller-owned subrange in Query_Scratch
Query_Output :: struct
{
	start:    i32,
	capacity: i32,
}

@(private)
query_ray_data :: struct
{
	ray:    Ray,
	filter: Query_Filter,
	output: Query_Output,
}

@(private)
query_sweep_data :: struct
{
	shape:     Shape_Handle,
	pose:      Rigid_Pose,
	velocity:  Body_Velocity,
	maximum_t: f32,
	settings:  Sweep_Settings,
	filter:    Query_Filter,
}

@(private)
query_overlap_data :: struct
{
	shape:  Shape_Handle,
	pose:   Rigid_Pose,
	filter: Query_Filter,
	output: Query_Output,
}

@(private)
query_volume_data :: struct
{
	bounds: Bounding_Box,
	filter: Query_Filter,
	output: Query_Output,
}

// Query is a caller-owned tagged query. construct it with the query_* helpers
Query :: struct
{
	kind: Query_Kind,
	using payload: struct #raw_union
	{
		ray_data:     query_ray_data,
		sweep_data:   query_sweep_data,
		overlap_data: query_overlap_data,
		volume_data:  query_volume_data,
	},
}

// Query_Result is written by query_batch in input order. all-hit results are
// stored in the selected Query_Scratch range in low-level traversal order.
// the collection order is not canonical
Query_Result :: struct
{
	status: Status,
	count:  i32,
	hit:    bool,
	using payload: struct #raw_union
	{
		ray_hit:   Ray_Hit,
		sweep_hit: Sweep_Hit,
	},
}

// Query_Scratch binds caller-owned all-hit output arrays. results stay within
// each query's selected range. internal query scratch may grow
Query_Scratch :: struct
{
	ray_hits:     []Ray_Hit,
	overlap_hits: []Overlap_Hit,
	volume_hits:  []Volume_Hit,
}

@(private)
query_filter_context :: struct
{
	filter:        Query_Filter,
	stop_on_first: bool,
}
#assert(size_of(Ray_Hit) == size_of(physics.Ray_Query_Hit));
#assert(align_of(Ray_Hit) == align_of(physics.Ray_Query_Hit));
#assert(offset_of(Ray_Hit, collidable) == offset_of(physics.Ray_Query_Hit, target_id));
#assert(offset_of(Ray_Hit, child_index) == offset_of(physics.Ray_Query_Hit, child_index));
#assert(size_of(Sweep_Hit) == size_of(physics.Sweep_Query_Hit));
#assert(align_of(Sweep_Hit) == align_of(physics.Sweep_Query_Hit));
#assert(offset_of(Sweep_Hit, collidable) == offset_of(physics.Sweep_Query_Hit, target_id));
#assert(offset_of(Sweep_Hit, sweep) == offset_of(physics.Sweep_Query_Hit, sweep));
#assert(size_of(Overlap_Hit) == size_of(physics.Overlap_Query_Hit));
#assert(align_of(Overlap_Hit) == align_of(physics.Overlap_Query_Hit));
#assert(offset_of(Overlap_Hit, collidable) == offset_of(physics.Overlap_Query_Hit, target_id));
#assert(offset_of(Overlap_Hit, manifold) == offset_of(physics.Overlap_Query_Hit, manifold));
#assert(size_of(Volume_Hit) == size_of(physics.Volume_Query_Hit));
#assert(align_of(Volume_Hit) == align_of(physics.Volume_Query_Hit));
// ray constructs a world-space ray. direction does not need to be normalized.
// t remains measured in units of direction length
ray :: #force_inline proc "contextless" (
	origin, direction: Vector3,
	maximum_t: f32,
) -> Ray
{
	return {origin=origin, direction=direction, maximum_t=maximum_t};
}

// query_filter_all returns the explicit unfiltered policy. Query_Filter{} is
// equivalent and remains the convenient default argument
query_filter_all :: #force_inline proc "contextless" () -> Query_Filter
{
	return {include=COLLIDABLE_ALL};
}

// query_filter_mobility selects one or more mobility classes without callbacks
query_filter_mobility :: #force_inline proc "contextless" (
	include: Collidable_Mask,
) -> Query_Filter
{
	return {include=include};
}

// collidable_mobility returns the packed reference mobility class
collidable_mobility :: #force_inline proc "contextless" (
	collidable: Collidable_Reference,
) -> Body_Mobility
{
	return physics.collidable_reference_mobility(collidable);
}

// collidable_body_handle extracts a dynamic or kinematic body handle
collidable_body_handle :: #force_inline proc "contextless" (
	collidable: Collidable_Reference,
) -> (Body_Handle, Status)
{
	return physics.collidable_reference_body_handle(collidable);
}

// collidable_static_handle extracts a static handle
collidable_static_handle :: #force_inline proc "contextless" (
	collidable: Collidable_Reference,
) -> (Static_Handle, Status)
{
	return physics.collidable_reference_static_handle(collidable);
}

@(private)
query_filter_has_include :: #force_inline proc "contextless" (
	filter: Query_Filter,
) -> bool
{
	return filter.include != Collidable_Mask{};
}

@(private)
query_filter_is_empty :: #force_inline proc "contextless" (
	filter: Query_Filter,
) -> bool
{
	return filter.include == Collidable_Mask{} &&
	filter.exclude == Collidable_Mask{} &&
	filter.allow == nil && filter.allow_child == nil;
}

@(private)
query_filter_target_allowed :: #force_inline proc "contextless" (
	filter: Query_Filter,
	collidable: Collidable_Reference,
) -> bool
{
	mobility := collidable_mobility(collidable);
	if query_filter_has_include(filter) && mobility not_in filter.include
	{
		return false;
	}
	if mobility in filter.exclude
	{
		return false;
	}
	if filter.allow != nil && !filter.allow(filter.user_context, collidable)
	{
		return false;
	}
	return true;
}

@(private)
query_filter_target_bridge :: proc "contextless" (
	user_context: rawptr,
	target_id: i32,
) -> physics.Query_Filter_Result
{
	ctx := (^query_filter_context)(user_context);
	if ctx == nil
	{
		return .Reject;
	}
	collidable := Collidable_Reference{packed=u32(target_id)};
	if query_filter_target_allowed(ctx.filter, collidable)
	{
		return .Allow;
	}
	return .Reject;
}

@(private)
query_filter_child_bridge :: proc "contextless" (
	user_context: rawptr,
	target_id, child_index: i32,
) -> physics.Query_Filter_Result
{
	ctx := (^query_filter_context)(user_context);
	if ctx == nil
	{
		return .Reject;
	}
	if ctx.filter.allow_child == nil
	{
		return .Allow;
	}
	collidable := Collidable_Reference{packed=u32(target_id)};
	if ctx.filter.allow_child(ctx.filter.user_context, collidable, child_index)
	{
		return .Allow;
	}
	return .Reject;
}

@(private)
query_filter_low_level :: #force_inline proc "contextless" (
	filter: Query_Filter,
) -> physics.Query_Filter_Procedures
{
	if query_filter_is_empty(filter)
	{
		return {};
	}
	return {
		allow_target=query_filter_target_bridge,
		allow_child=query_filter_child_bridge,
	};
}

@(private)
query_ray_stop_on_first :: proc "contextless" (
	user_context: rawptr,
	_ray: physics.Tree_Ray,
	_hit: ^physics.Ray_Query_Hit,
	maximum_t: ^f32,
) -> physics.Physics_Status
{
	ctx := (^query_filter_context)(user_context);
	if ctx == nil || maximum_t == nil
	{
		return .Invalid_Argument;
	}
	if ctx.stop_on_first
	{
		maximum_t^ = 0;
	}
	return .Ok;
}

@(private)
query_world_data :: #force_inline proc "contextless" (
	world: ^World,
) -> (^world_data, Status)
{
	data := world_data_get(world);
	if data == nil
	{
		return nil, .Disposed;
	}
	if data.simulation.state != .Ready
	{
		return nil, .Invalid_Argument;
	}
	if data.pool == nil || data.pool.state != .Ready
	{
		return nil, .Disposed;
	}
	return data, .Ok;
}

@(private)
query_ray_buffer :: #force_inline proc "contextless" (
	hits: []Ray_Hit,
) -> util.Buffer(physics.Ray_Query_Hit)
{
	if len(hits) == 0
	{
		return {};
	}
	return {
		memory=([^]physics.Ray_Query_Hit)(rawptr(&hits[0])),
		length=i32(len(hits)),
		id=util.BUFFER_CALLER_OWNED_ID,
	};
}

@(private)
query_sweep_buffer :: #force_inline proc "contextless" (
	hits: []Sweep_Hit,
) -> util.Buffer(physics.Sweep_Query_Hit)
{
	if len(hits) == 0
	{
		return {};
	}
	return {
		memory=([^]physics.Sweep_Query_Hit)(rawptr(&hits[0])),
		length=i32(len(hits)),
		id=util.BUFFER_CALLER_OWNED_ID,
	};
}

@(private)
query_overlap_buffer :: #force_inline proc "contextless" (
	hits: []Overlap_Hit,
) -> util.Buffer(physics.Overlap_Query_Hit)
{
	if len(hits) == 0
	{
		return {};
	}
	return {
		memory=([^]physics.Overlap_Query_Hit)(rawptr(&hits[0])),
		length=i32(len(hits)),
		id=util.BUFFER_CALLER_OWNED_ID,
	};
}

@(private)
query_volume_buffer :: #force_inline proc "contextless" (
	hits: []Volume_Hit,
) -> util.Buffer(physics.Volume_Query_Hit)
{
	if len(hits) == 0
	{
		return {};
	}
	return {
		memory=([^]physics.Volume_Query_Hit)(rawptr(&hits[0])),
		length=i32(len(hits)),
		id=util.BUFFER_CALLER_OWNED_ID,
	};
}

// ray_cast_any reports whether the ray intersects any permitted collidable.
// it uses one stack hit and clips traversal after the first accepted impact
ray_cast_any :: proc "contextless" (
	world: ^World,
	ray_value: Ray,
	filter: Query_Filter = {},
) -> (bool, Status)
{
	data, world_status := query_world_data(world);
	if world_status != .Ok
	{
		return false, world_status;
	}
	hits: [1]Ray_Hit;
	ctx := query_filter_context{filter=filter, stop_on_first=true};
	collector: physics.Ray_Query_Collector;
	status := physics.ray_query_collector_initialize(
		&collector,
		query_ray_buffer(hits[:]),
		.Earliest,
		{
			filter=query_filter_low_level(filter),
			hit=query_ray_stop_on_first,
			user_context=&ctx,
		},
	);
	if status != .Ok
	{
		return false, status;
	}
	status = physics.simulation_ray_query(
		&data.simulation, ray_value, &collector, data.pool,
	);
	if status != .Ok
	{
		return false, status;
	}
	return collector.count > 0, .Ok;
}

// ray_cast_closest returns Not_Found when no permitted collidable is hit
ray_cast_closest :: proc "contextless" (
	world: ^World,
	ray_value: Ray,
	filter: Query_Filter = {},
) -> (Ray_Hit, Status)
{
	data, world_status := query_world_data(world);
	if world_status != .Ok
	{
		return {child_index=-1}, world_status;
	}
	hits: [1]Ray_Hit;
	ctx := query_filter_context{filter=filter};
	collector: physics.Ray_Query_Collector;
	status := physics.ray_query_collector_initialize(
		&collector,
		query_ray_buffer(hits[:]),
		.Earliest,
		{
			filter=query_filter_low_level(filter),
			user_context=&ctx,
		},
	);
	if status != .Ok
	{
		return {child_index=-1}, status;
	}
	status = physics.simulation_ray_query(
		&data.simulation, ray_value, &collector, data.pool,
	);
	if status != .Ok
	{
		return {child_index=-1}, status;
	}
	if collector.count == 0
	{
		return {child_index=-1}, .Not_Found;
	}
	return hits[0], .Ok;
}

// ray_cast_all writes a successful traversal prefix into caller-owned storage.
// Capacity_Missing reports overflow and count identifies the written prefix.
// results preserve low-level traversal order. this order is not
// canonical and may change with world topology or implementation details
ray_cast_all :: proc "contextless" (
	world: ^World,
	ray_value: Ray,
	hits: []Ray_Hit,
	filter: Query_Filter = {},
) -> (int, Status)
{
	data, world_status := query_world_data(world);
	if world_status != .Ok
	{
		return 0, world_status;
	}
	if len(hits) == 0
	{
		found, status := ray_cast_any(world, ray_value, filter);
		if status != .Ok
		{
			return 0, status;
		}
		if found
		{
			return 0, .Capacity_Missing;
		}
		return 0, .Ok;
	}
	ctx := query_filter_context{filter=filter};
	collector: physics.Ray_Query_Collector;
	status := physics.ray_query_collector_initialize(
		&collector,
		query_ray_buffer(hits),
		.All,
		{
			filter=query_filter_low_level(filter),
			user_context=&ctx,
		},
	);
	if status != .Ok
	{
		return 0, status;
	}
	status = physics.simulation_ray_query(
		&data.simulation, ray_value, &collector, data.pool,
	);
	count := collector.count;
	return count, status;
}

@(private)
sweep_settings_resolve :: proc "contextless" (
	data: ^world_data,
	shape: Shape_Handle,
	velocity_value: Body_Velocity,
	maximum_t: f32,
	settings: Sweep_Settings,
) -> (Sweep_Settings, Status)
{
	if maximum_t < 0
	{
		return {}, .Invalid_Argument;
	}
	if settings.maximum_iteration_count < 0 || settings.minimum_progression < 0 ||
	settings.convergence_threshold < 0
	{
		return {}, .Invalid_Description;
	}
	if settings.maximum_iteration_count > 0
	{
		return settings, .Ok;
	}
	registry := physics.simulation_shape_registry(&data.simulation);
	bounds, status := physics.shape_registry_compute_bounds(
		registry, shape, util.quaternion_identity(),
	);
	if status != .Ok
	{
		return {}, status;
	}
	minimum_radius := bounds.maximum_radius - bounds.maximum_angular_expansion;
	size_estimate := max(minimum_radius, bounds.maximum_radius * 0.25);
	minimum_progression_distance := 0.1 * size_estimate;
	convergence_threshold_distance := 1e-5 * size_estimate;
	tangent_velocity := f32(0);
	if maximum_t > 0
	{
		tangent_velocity = min(
			util.vector3_length(velocity_value.angular) * bounds.maximum_radius,
			bounds.maximum_angular_expansion / maximum_t,
		);
	}
	total_velocity := util.vector3_length(velocity_value.linear) + tangent_velocity;
	if total_velocity <= 1e-20
	{
		return {
			minimum_progression=0,
			convergence_threshold=0,
			maximum_iteration_count=25,
		}, .Ok;
	}
	inverse_velocity := 1 / total_velocity;
	return {
		minimum_progression=minimum_progression_distance * inverse_velocity,
		convergence_threshold=convergence_threshold_distance * inverse_velocity,
		maximum_iteration_count=25,
	}, .Ok;
}

// sweep_closest sweeps one registered shape through the world while treating
// world targets as stationary, matching Entasis sweep semantics.
// Not_Found reports no impact
sweep_closest :: proc "contextless" (
	world: ^World,
	shape: Shape_Handle,
	pose_value: Rigid_Pose,
	velocity_value: Body_Velocity,
	maximum_t: f32,
	filter: Query_Filter = {},
	settings: Sweep_Settings = {},
) -> (Sweep_Hit, Status)
{
	data, world_status := query_world_data(world);
	if world_status != .Ok
	{
		return {}, world_status;
	}
	resolved_settings, settings_status := sweep_settings_resolve(
		data, shape, velocity_value, maximum_t, settings,
	);
	if settings_status != .Ok
	{
		return {}, settings_status;
	}
	hits: [1]Sweep_Hit;
	ctx := query_filter_context{filter=filter};
	collector := physics.Sweep_Query_Collector{
		hits=query_sweep_buffer(hits[:]),
		mode=.Earliest,
		callbacks={
			filter=query_filter_low_level(filter),
			user_context=&ctx,
		},
	};
	status := physics.simulation_sweep_query(
		&data.simulation,
		shape,
		pose_value,
		velocity_value,
		maximum_t,
		resolved_settings.minimum_progression,
		resolved_settings.convergence_threshold,
		int(resolved_settings.maximum_iteration_count),
		&collector,
		data.pool,
	);
	if status != .Ok
	{
		return {}, status;
	}
	if collector.count == 0
	{
		return {}, .Not_Found;
	}
	return hits[0], .Ok;
}

// overlap_all performs exact shape overlap tests and writes contact manifolds to
// caller-owned storage in low-level traversal order. the collection order is
// not canonical and callers must not use it as a stable identity
overlap_all :: proc "contextless" (
	world: ^World,
	shape: Shape_Handle,
	pose_value: Rigid_Pose,
	hits: []Overlap_Hit,
	filter: Query_Filter = {},
) -> (int, Status)
{
	data, world_status := query_world_data(world);
	if world_status != .Ok
	{
		return 0, world_status;
	}
	if len(hits) == 0
	{
		temporary: [1]Overlap_Hit;
		count, status := overlap_all(world, shape, pose_value, temporary[:], filter);
		if count > 0
		{
			return 0, .Capacity_Missing;
		}
		return 0, status;
	}
	ctx := query_filter_context{filter=filter};
	collector := physics.Overlap_Query_Collector{
		hits=query_overlap_buffer(hits),
		filter=query_filter_low_level(filter),
		user_context=&ctx,
	};
	status := physics.simulation_overlap_query(
		&data.simulation, shape, pose_value, &collector, data.pool,
	);
	count := collector.count;
	return count, status;
}

// volume_all returns broad-phase collidables whose bounds overlap the volume in
// low-level traversal order. the collection order is not canonical
volume_all :: proc "contextless" (
	world: ^World,
	bounds: Bounding_Box,
	hits: []Volume_Hit,
	filter: Query_Filter = {},
) -> (int, Status)
{
	data, world_status := query_world_data(world);
	if world_status != .Ok
	{
		return 0, world_status;
	}
	if len(hits) == 0
	{
		temporary: [1]Volume_Hit;
		count, status := volume_all(world, bounds, temporary[:], filter);
		if count > 0
		{
			return 0, .Capacity_Missing;
		}
		return 0, status;
	}
	ctx := query_filter_context{filter=filter};
	collector := physics.Volume_Query_Collector{
		hits=query_volume_buffer(hits),
		filter=query_filter_low_level(filter),
		user_context=&ctx,
	};
	status := physics.simulation_volume_query(
		&data.simulation, bounds, &collector, data.pool,
	);
	count := collector.count;
	return count, status;
}

// query_output selects a subrange in one Query_Scratch output array
query_output :: #force_inline proc "contextless" (start, capacity: int) -> Query_Output
{
	return {start=i32(start), capacity=i32(capacity)};
}

// query_ray_any constructs one batched any-ray-hit query
query_ray_any :: #force_inline proc "contextless" (
	ray_value: Ray,
	filter: Query_Filter = {},
) -> Query
{
	return {kind=.Ray_Any, ray_data={ray=ray_value, filter=filter}};
}

// query_ray_closest constructs one batched closest-ray-hit query
query_ray_closest :: #force_inline proc "contextless" (
	ray_value: Ray,
	filter: Query_Filter = {},
) -> Query
{
	return {kind=.Ray_Closest, ray_data={ray=ray_value, filter=filter}};
}

// query_ray_all constructs one batched all-ray-hit query
query_ray_all :: #force_inline proc "contextless" (
	ray_value: Ray,
	output: Query_Output,
	filter: Query_Filter = {},
) -> Query
{
	return {kind=.Ray_All, ray_data={ray=ray_value, filter=filter, output=output}};
}

// query_sweep_closest constructs one batched closest-sweep query
query_sweep_closest :: #force_inline proc "contextless" (
	shape: Shape_Handle,
	pose_value: Rigid_Pose,
	velocity_value: Body_Velocity,
	maximum_t: f32,
	filter: Query_Filter = {},
	settings: Sweep_Settings = {},
) -> Query
{
	return {
		kind=.Sweep_Closest,
		sweep_data={
			shape=shape,
			pose=pose_value,
			velocity=velocity_value,
			maximum_t=maximum_t,
			settings=settings,
			filter=filter,
		},
	};
}

// query_overlap_all constructs one batched overlap-all query
query_overlap_all :: #force_inline proc "contextless" (
	shape: Shape_Handle,
	pose_value: Rigid_Pose,
	output: Query_Output,
	filter: Query_Filter = {},
) -> Query
{
	return {
		kind=.Overlap_All,
		overlap_data={shape=shape, pose=pose_value, filter=filter, output=output},
	};
}

// query_volume_all constructs one batched broad-phase volume query
query_volume_all :: #force_inline proc "contextless" (
	bounds: Bounding_Box,
	output: Query_Output,
	filter: Query_Filter = {},
) -> Query
{
	return {kind=.Volume_All, volume_data={bounds=bounds, filter=filter, output=output}};
}

// query_scratch binds caller-owned ray, overlap, and volume hit arrays for query_batch
query_scratch :: #force_inline proc "contextless" (
	ray_hits: []Ray_Hit = nil,
	overlap_hits: []Overlap_Hit = nil,
	volume_hits: []Volume_Hit = nil,
) -> Query_Scratch
{
	return {ray_hits=ray_hits, overlap_hits=overlap_hits, volume_hits=volume_hits};
}

@(private)
query_output_range_valid :: #force_inline proc "contextless" (
	output: Query_Output,
	capacity: int,
) -> bool
{
	if output.start < 0 || output.capacity < 0
	{
		return false;
	}
	start := int(output.start);
	count := int(output.capacity);
	return start <= capacity && count <= capacity - start;
}

@(private)
query_batch_ray_run :: proc "contextless" (
	data: ^world_data, queries: []Query, results: []Query_Result,
) -> Status
{
	QUERY_RAY_RUN_CAPACITY :: int(32);
	rays: [QUERY_RAY_RUN_CAPACITY]physics.Tree_Ray = ---;
	completions: [QUERY_RAY_RUN_CAPACITY]physics.Ray_Query_Batch_Completion = ---;
	filter := queries[0].ray_data.filter;
	ctx := query_filter_context{filter=filter};
	callbacks := physics.Ray_Query_Callbacks{filter=query_filter_low_level(filter), user_context=&ctx};
	overall := Status.Ok;
	for start := 0; start < len(queries); start += QUERY_RAY_RUN_CAPACITY
	{
		count := min(QUERY_RAY_RUN_CAPACITY, len(queries) - start);
		for lane in 0 ..< count
		{
			rays[lane] = queries[start + lane].ray_data.ray;
		}
		physics.simulation_ray_query_batch(&data.simulation, rays[:count], completions[:count], callbacks, data.pool);
		for lane in 0 ..< count
		{
			completion := completions[lane];
			result := &results[start + lane];
			result^ = {status=completion.status, ray_hit={child_index=-1}};
			if completion.status == .Ok
			{
				if completion.presence == .Present
				{
					result.hit = completion.presence == .Present;
					result.count = 1;
					result.ray_hit = transmute(Ray_Hit)completion.hit;
				}
				else
				{
					result.status = .Not_Found;
				}
			}
			if overall == .Ok && result.status != .Ok
			{
				overall = result.status;
			}
		}
	}
	return overall;
}

// query_batch combines compatible pure ray work and writes one result per input
// in input order. queries are read-only. internal query scratch may grow.
// all-hit queries write to the caller-owned Query_Scratch range selected by their
// Query_Output, preserving the low-level collector traversal order without
// canonicalization. every query is attempted. the return value is the first
// non-Ok result status
query_batch :: proc "contextless" (
	world: ^World,
	queries: []Query,
	results: []Query_Result,
	scratch: ^Query_Scratch = nil,
) -> Status
{
	if len(results) < len(queries)
	{
		return .Invalid_Argument;
	}
	data, world_status := query_world_data(world);
	ray_run_eligibility := physics.Reference_State.Present;
	if world_status != .Ok
	{
		ray_run_eligibility = .Missing;
	}
	else
	{
		shapes := physics.simulation_shape_registry(&data.simulation);
		for type_id in physics.BUILT_IN_SHAPE_TYPE_COUNT ..< shapes.registered_type_count
		{
			if shapes.batches[type_id].active_count != 0
			{
				ray_run_eligibility = .Missing;
				break;
			}
		}
	}
	overall := Status.Ok;
	for query_index := 0; query_index < len(queries);
	{
		if ray_run_eligibility == .Present && queries[query_index].kind == .Ray_Closest
		{
			first := queries[query_index].ray_data;
			if first.filter.allow == nil && first.filter.allow_child == nil && physics.shape_ray_valid(first.ray) == .Ok
			{
				include := first.filter.include;
				if include == {}
				{
					include = COLLIDABLE_ALL;
				}
				end := query_index + 1;
				for end < len(queries) && queries[end].kind == .Ray_Closest
				{
					next := queries[end].ray_data;
					next_include := next.filter.include;
					if next_include == {}
					{
						next_include = COLLIDABLE_ALL;
					}
					if next.filter.allow != nil || next.filter.allow_child != nil || next_include != include ||
					next.filter.exclude != first.filter.exclude || physics.shape_ray_valid(next.ray) != .Ok
					{
						break;
					}
					end += 1;
				}
				status := query_batch_ray_run(data, queries[query_index:end], results[query_index:end]);
				if overall == .Ok && status != .Ok
				{
					overall = status;
				}
				query_index = end;
				continue;
			}
		}
		query_value := queries[query_index];
		result := &results[query_index];
		result^ = {};
		switch query_value.kind
		{
			case .Ray_Any:
			result.hit, result.status = ray_cast_any(
				world, query_value.ray_data.ray, query_value.ray_data.filter,
			);
			if result.hit
			{
				result.count = 1;
			}
			case .Ray_Closest:
			result.ray_hit, result.status = ray_cast_closest(
				world, query_value.ray_data.ray, query_value.ray_data.filter,
			);
			if result.status == .Ok
			{
				result.hit = true;
				result.count = 1;
			}
			case .Ray_All:
			if scratch == nil || !query_output_range_valid(
				query_value.ray_data.output, len(scratch.ray_hits),
			)
			{
				result.status = .Invalid_Argument;
				break;
			}
			start := int(query_value.ray_data.output.start);
			end := start + int(query_value.ray_data.output.capacity);
			result_count, status := ray_cast_all(
				world, query_value.ray_data.ray, scratch.ray_hits[start:end],
				query_value.ray_data.filter,
			);
			result.count = i32(result_count);
			result.hit = result_count > 0;
			result.status = status;
			case .Sweep_Closest:
			result.sweep_hit, result.status = sweep_closest(
				world,
				query_value.sweep_data.shape,
				query_value.sweep_data.pose,
				query_value.sweep_data.velocity,
				query_value.sweep_data.maximum_t,
				query_value.sweep_data.filter,
				query_value.sweep_data.settings,
			);
			if result.status == .Ok
			{
				result.hit = true;
				result.count = 1;
			}
			case .Overlap_All:
			if scratch == nil || !query_output_range_valid(
				query_value.overlap_data.output, len(scratch.overlap_hits),
			)
			{
				result.status = .Invalid_Argument;
				break;
			}
			start := int(query_value.overlap_data.output.start);
			end := start + int(query_value.overlap_data.output.capacity);
			result_count, status := overlap_all(
				world, query_value.overlap_data.shape, query_value.overlap_data.pose,
				scratch.overlap_hits[start:end], query_value.overlap_data.filter,
			);
			result.count = i32(result_count);
			result.hit = result_count > 0;
			result.status = status;
			case .Volume_All:
			if scratch == nil || !query_output_range_valid(
				query_value.volume_data.output, len(scratch.volume_hits),
			)
			{
				result.status = .Invalid_Argument;
				break;
			}
			start := int(query_value.volume_data.output.start);
			end := start + int(query_value.volume_data.output.capacity);
			result_count, status := volume_all(
				world, query_value.volume_data.bounds, scratch.volume_hits[start:end],
				query_value.volume_data.filter,
			);
			result.count = i32(result_count);
			result.hit = result_count > 0;
			result.status = status;
		}
		if overall == .Ok && result.status != .Ok
		{
			overall = result.status;
		}
		query_index += 1;
	}
	return overall;
}

// Sweep_Hit_Proc is the advanced synchronous sweep-hit callback. the callback
// may reduce maximum_t to prune later traversal. it executes on the query caller
Sweep_Hit_Proc :: physics.Sweep_Query_Hit_Proc;
// Sweep_Zero_Hit_Proc handles an overlap at sweep time zero
Sweep_Zero_Hit_Proc :: physics.Sweep_Query_Zero_Hit_Proc;
// Sweep_Callbacks is the advanced low-level sweep callback table
Sweep_Callbacks :: physics.Sweep_Query_Callbacks;
// Sweep_Collector is the advanced caller-owned collector bridge
Sweep_Collector :: physics.Sweep_Query_Collector;
@(private)
query_sweep_stop_hit :: proc "contextless" (
	user_context: rawptr, _hit: ^physics.Sweep_Query_Hit, maximum_t: ^f32,
) -> Status
{
	_, _ = user_context, _hit;
	if maximum_t == nil
	{
		return .Invalid_Argument;
	}
	maximum_t^ = 0;
	return .Ok;
}

@(private)
query_sweep_stop_zero :: proc "contextless" (
	user_context: rawptr, _target_id: i32, maximum_t: ^f32,
) -> Status
{
	_, _ = user_context, _target_id;
	if maximum_t == nil
	{
		return .Invalid_Argument;
	}
	maximum_t^ = 0;
	return .Ok;
}

// sweep_query_advanced executes a sweep using a caller-owned low-level
// collector. use it for custom hit callbacks and traversal pruning while
// retaining the facade World
sweep_query_advanced :: proc "contextless" (
	world: ^World,
	shape: Shape_Handle,
	pose_value: Rigid_Pose,
	velocity_value: Body_Velocity,
	maximum_t: f32,
	collector: ^Sweep_Collector,
	settings: Sweep_Settings = {},
) -> Status
{
	data, world_status := query_world_data(world);
	if world_status != .Ok
	{
		return world_status;
	}
	if collector == nil
	{
		return .Invalid_Argument;
	}
	resolved_settings, settings_status := sweep_settings_resolve(
		data, shape, velocity_value, maximum_t, settings,
	);
	if settings_status != .Ok
	{
		return settings_status;
	}
	return physics.simulation_sweep_query(
		&data.simulation, shape, pose_value, velocity_value, maximum_t,
		resolved_settings.minimum_progression,
		resolved_settings.convergence_threshold,
		int(resolved_settings.maximum_iteration_count), collector, data.pool,
	);
}

// sweep_any returns true on the first permitted impact and stops traversal
// allocation: none
sweep_any :: proc "contextless" (
	world: ^World,
	shape: Shape_Handle,
	pose_value: Rigid_Pose,
	velocity_value: Body_Velocity,
	maximum_t: f32,
	filter: Query_Filter = {},
	settings: Sweep_Settings = {},
) -> (bool, Status)
{
	hits: [1]Sweep_Hit;
	ctx := query_filter_context{filter=filter, stop_on_first=true};
	collector := physics.Sweep_Query_Collector{
		hits=query_sweep_buffer(hits[:]),
		mode=.All,
		callbacks={
			filter=query_filter_low_level(filter),
			hit=query_sweep_stop_hit,
			hit_at_zero=query_sweep_stop_zero,
			user_context=&ctx,
		},
	};
	status := sweep_query_advanced(
		world, shape, pose_value, velocity_value, maximum_t, &collector, settings,
	);
	// only collector overflow after a hit can be collapsed into success.
	// a task or traversal failure before any hit must remain visible
	if status != .Ok && (status != .Capacity_Missing || collector.count == 0)
	{
		return false, status;
	}
	return collector.count > 0, .Ok;
}

// sweep_all writes every permitted impact into caller-owned storage in
// low-level traversal order. the collection order is not canonical
sweep_all :: proc "contextless" (
	world: ^World,
	shape: Shape_Handle,
	pose_value: Rigid_Pose,
	velocity_value: Body_Velocity,
	maximum_t: f32,
	hits: []Sweep_Hit,
	filter: Query_Filter = {},
	settings: Sweep_Settings = {},
) -> (int, Status)
{
	if len(hits) == 0
	{
		found, status := sweep_any(
			world, shape, pose_value, velocity_value, maximum_t, filter, settings,
		);
		if status != .Ok
		{
			return 0, status;
		}
		if found
		{
			return 0, .Capacity_Missing;
		}
		return 0, .Ok;
	}
	ctx := query_filter_context{filter=filter};
	collector := physics.Sweep_Query_Collector{
		hits=query_sweep_buffer(hits),
		mode=.All,
		callbacks={filter=query_filter_low_level(filter), user_context=&ctx},
	};
	status := sweep_query_advanced(
		world, shape, pose_value, velocity_value, maximum_t, &collector, settings,
	);
	count := collector.count;
	return count, status;
}

// resolves current mobility without requiring callers to pack handle bits
body_collidable_reference :: proc "contextless" (world:^World, handle:Body_Handle) -> (Collidable_Reference, Status)
{
	sim: ^physics.Simulation = world_simulation(world);
	if sim==nil
	{
		return {}, .Disposed;
	}
	loc: physics.Body_Memory_Location;
	status: Status;
	loc, status = physics.bodies_resolve(&sim.bodies, handle);
	if status!=.Ok
	{
		return {}, status;
	}
	mobility: Body_Mobility = physics.body_inertia_mobility(sim.bodies.sets.memory[loc.set_index].dynamics_state.memory[loc.index].inertia.local);
	return physics.collidable_reference_body(mobility, handle);
}

static_collidable_reference :: proc "contextless" (world:^World, handle:Static_Handle) -> (Collidable_Reference, Status)
{
	sim: ^physics.Simulation = world_simulation(world);
	if sim==nil
	{
		return {}, .Disposed;
	}
	status: Status;
	_, status = physics.statics_resolve(&sim.statics, handle);
	if status!=.Ok
	{
		return {}, status;
	}
	return physics.collidable_reference_static(handle);
}

// a successful miss is Separated/Ok, not Not_Found. only nonnegative contact
// depths count. speculative contacts are excluded. no all-hit array is built
Overlap_State :: physics.Overlap_Query_State;
overlap_any :: proc "contextless" (
	world: ^World, shape: Shape_Handle, pose_value: Rigid_Pose, filter: Query_Filter = {},
) -> (Overlap_State, Status)
{
	data: ^world_data;
	status: Status;
	data, status = query_world_data(world);
	if status != .Ok
	{
		return .Separated, status;
	}
	batcher: ^physics.Query_Overlap_Batcher = &data.simulation.query_overlap_batcher;
	if batcher.state == .Uninitialized
	{
		status = physics.query_overlap_batcher_bind(batcher, &data.simulation.narrow_phase);
		if status != .Ok
		{
			return .Separated, status;
		}
	}
	filter_context: query_filter_context = {filter=filter};
	return physics.simulation_overlap_any_query(&data.simulation, shape, pose_value,
		query_filter_low_level(filter), &filter_context, data.pool, batcher);
}
