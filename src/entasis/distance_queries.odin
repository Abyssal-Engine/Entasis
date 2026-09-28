package entasis

import "core:mem"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

// settings and pointer-free results have the same units as existing shape queries.
// geometry.normal points B-to-A. for a point query, point_a is the input point
// and point_b is the closest shape point. inside a convex solid they coincide.
// child_a/child_b are -1 for convex roots, otherwise the stored leaf index
Distance_Query_Settings :: physics.Distance_Query_Settings;
Distance_Query_State :: physics.Distance_Query_State;
Distance_Query_Geometry :: physics.Distance_Query_Result;
Shape_Distance_Result :: physics.Distance_Query_Pair_Result;
Shape_Correction_Result :: physics.Distance_Query_Correction;
// only penetration/correction needs EPA scratch. reserve explicitly while idle.
// queries never allocate. equal/smaller reservations retain existing buffers.
// capacities bound geometry storage, independently of the iteration limit
Distance_Query_Capacity :: struct
{
	vertices, faces, edges: i32,
}

distance_query_settings_default :: proc "contextless" () -> Distance_Query_Settings
{
	return physics.distance_query_settings_default();
}

distance_query_capacity_default :: proc "contextless" () -> Distance_Query_Capacity
{
	return {vertices=128, faces=256, edges=384};
}

@(private)
distance_query_storage :: struct
{
	vertices: util.Buffer(physics.Distance_Query_Vertex),
	faces: util.Buffer(physics.Distance_Query_Face),
	edges: util.Buffer(physics.Distance_Query_Edge),
	state: enum u8
	{
		Ready, Executing
	},
}

@(private)
distance_query_storage_destroy :: proc (
	storage: ^^distance_query_storage, pool: ^Buffer_Pool,
	allocator: mem.Allocator, scope: Allocation_Scope,
)
{
	if storage^ == nil
	{
		return;
	}
	data: ^distance_query_storage = storage^;
	// returns were reserved before acquiring these buffers, even for Legacy
	_ = util.buffer_pool_return(pool, &data.vertices);
	_ = util.buffer_pool_return(pool, &data.faces);
	_ = util.buffer_pool_return(pool, &data.edges);
	_ = util.allocation_free(data, size_of(distance_query_storage), align_of(distance_query_storage), allocator, scope);
	storage^ = nil;
}

@(private)
distance_query_storage_reserve :: proc (
	storage: ^^distance_query_storage, pool: ^Buffer_Pool,
	allocator: mem.Allocator, scope: Allocation_Scope, capacity: Distance_Query_Capacity,
) -> Status
{
	if capacity.vertices < 4 || capacity.faces < 4 || capacity.edges < 3
	{
		return .Invalid_Description;
	}
	data: ^distance_query_storage = storage^;
	if data != nil
	{
		if data.state != .Ready
		{
			return .Invalid_Argument;
		}
		if data.vertices.length >= capacity.vertices && data.faces.length >= capacity.faces && data.edges.length >= capacity.edges
		{
			return .Ok;
		}
	}
	else
	{
		error: mem.Allocator_Error;
		data, error = new(distance_query_storage, allocator=allocator);
		if error != nil || data == nil
		{
			return .Capacity_Missing;
		}
	}
	committed: physics.Reference_State;
	defer
	{
		if committed == .Missing && storage^ == nil
		{
			_ = util.allocation_free(data, size_of(distance_query_storage), align_of(distance_query_storage), allocator, scope);
		}
	}
	vertices: util.Buffer(physics.Distance_Query_Vertex) = data.vertices;
	faces: util.Buffer(physics.Distance_Query_Face) = data.faces;
	edges: util.Buffer(physics.Distance_Query_Edge) = data.edges;
	defer
	{
		query_context_return_replaced_buffer(pool, &vertices, data.vertices);
		query_context_return_replaced_buffer(pool, &faces, data.faces);
		query_context_return_replaced_buffer(pool, &edges, data.edges);
	}
	returns_reserved: physics.Reference_State;
	status: Status = query_context_reserve_buffer(pool, &vertices, int(capacity.vertices), &returns_reserved);
	if status != .Ok
	{
		return status;
	}
	status = query_context_reserve_buffer(pool, &faces, int(capacity.faces), &returns_reserved);
	if status != .Ok
	{
		return status;
	}
	status = query_context_reserve_buffer(pool, &edges, int(capacity.edges), &returns_reserved);
	if status != .Ok
	{
		return status;
	}
	query_context_return_replaced_buffer(pool, &data.vertices, vertices);
	query_context_return_replaced_buffer(pool, &data.faces, faces);
	query_context_return_replaced_buffer(pool, &data.edges, edges);
	data.vertices = vertices;
	data.faces = faces;
	data.edges = edges;
	storage^ = data;
	committed = .Present;
	return .Ok;
}

// cold setup only. world scratch uses the world's pool/allocator. context
// scratch uses its independent pool, including its explicitly borrowed owner.
// clear retains capacity. destroy returns all buffers before releasing the pool
distance_query_reserve :: proc (
	world: ^World, capacity: Distance_Query_Capacity = {128, 256, 384},
) -> Status
{
	data: ^world_data;
	status: Status;
	data, status = query_world_data(world);
	if status != .Ok
	{
		return status;
	}
	return distance_query_storage_reserve(&data.distance_storage, data.pool, data.allocator, data.allocation_scope, capacity);
}

distance_query_reserve_with_context :: proc (
	query_context: ^Query_Context, capacity: Distance_Query_Capacity = {128, 256, 384},
) -> Status
{
	data: ^query_context_data = query_context_get(query_context);
	if data == nil
	{
		return .Disposed;
	}
	if data.execution == .Executing || data.world.simulation.state != .Ready
	{
		return .Invalid_Argument;
	}
	return distance_query_storage_reserve(&data.distance_storage, data.pool, data.allocator, data.world.allocation_scope, capacity);
}

Distance_Query_Kind :: enum u8
{
	Closest_Point,
	Distance,
	Penetration,
	Depenetration,
}

// dedicated batch input, not a new arm in the existing heterogeneous Query.
// Closest_Point uses point and shape_b/pose_b. other kinds use both shapes
Distance_Query :: struct
{
	kind: Distance_Query_Kind,
	shape_a, shape_b: Shape_Handle,
	pose_a, pose_b: Rigid_Pose,
	point: Vector3,
	settings: Distance_Query_Settings,
}

Distance_Query_Result :: struct
{
	status: Status,
	shape: Shape_Distance_Result,
	correction: Shape_Correction_Result,
}

@(private)
distance_query_execute :: proc "contextless" (
	data: ^world_data, storage: ^distance_query_storage, query: ^Distance_Query,
) -> Distance_Query_Result
{
	if query.kind > .Depenetration
	{
		return {status=.Invalid_Argument};
	}
	shapes: ^physics.Shape_Registry = physics.simulation_shape_registry(&data.simulation);
	raw_b: rawptr;
	status: Status;
	raw_b, _, status = physics.shape_registry_resolve(shapes, query.shape_b);
	if status != .Ok
	{
		return {status=status};
	}
	type_b: int = int(physics.typed_index_type(query.shape_b));
	result: Distance_Query_Result;
	if query.kind == .Closest_Point
	{
		result.shape, result.status = physics.distance_query_shape_point(query.point, raw_b, type_b, query.pose_b, shapes, query.settings);
		return result;
	}
	raw_a: rawptr;
	raw_a, _, status = physics.shape_registry_resolve(shapes, query.shape_a);
	if status != .Ok
	{
		return {status=status};
	}
	type_a: int = int(physics.typed_index_type(query.shape_a));
	if query.kind == .Distance
	{
		result.shape, result.status = physics.distance_query_shape_pair(raw_a, raw_b, type_a, type_b, query.pose_a, query.pose_b, shapes, query.settings);
		return result;
	}
	if storage == nil
	{
		if physics.distance_query_settings_validate(query.settings) != .Ok
		{
			return {status=.Invalid_Argument};
		}
		return {status=.Capacity_Missing};
	}
	if storage.state != .Ready
	{
		return {status=.Invalid_Argument};
	}
	storage.state = .Executing;
	defer storage.state = .Ready;
	scratch: physics.Distance_Query_Scratch = {
		vertices=storage.vertices.memory[:storage.vertices.length],
		faces=storage.faces.memory[:storage.faces.length],
		edges=storage.edges.memory[:storage.edges.length],
	};
	if query.kind == .Penetration
	{
		result.shape.geometry, result.status = physics.distance_query_convex(raw_a, raw_b, type_a, type_b,
			query.pose_a, query.pose_b, shapes, query.settings, .Penetration, scratch);
		result.shape.child_a = -1;
		result.shape.child_b = -1;
	}
	else
	{
		result.correction, result.status = physics.distance_query_depenetrate(raw_a, raw_b, type_a, type_b,
			query.pose_a, query.pose_b, shapes, query.settings, scratch, &data.simulation.collision_tasks);
	}
	return result;
}

// inputs and outputs are independent caller-owned spans. capacity is preflighted
// before writes. every item executes in order. return the first failing status.
// no query allocates or changes world/trigger/restitution history
distance_query_batch :: proc "contextless" (
	world: ^World, queries: []Distance_Query, results: []Distance_Query_Result,
) -> Status
{
	if len(results) < len(queries)
	{
		return .Invalid_Argument;
	}
	data: ^world_data;
	status: Status;
	data, status = query_world_data(world);
	if status != .Ok
	{
		return status;
	}
	first: Status = .Ok;
	for &query, i in queries
	{
		results[i] = distance_query_execute(data, data.distance_storage, &query);
		if first == .Ok && results[i].status != .Ok
		{
			first = results[i].status;
		}
	}
	return first;
}

distance_query_batch_with_context :: proc "contextless" (
	query_context: ^Query_Context, queries: []Distance_Query, results: []Distance_Query_Result,
) -> Status
{
	if len(results) < len(queries)
	{
		return .Invalid_Argument;
	}
	data: ^query_context_data;
	status: Status;
	data, status = query_context_begin(query_context);
	if status != .Ok
	{
		return status;
	}
	defer data.execution = .Idle;
	first: Status = .Ok;
	for &query, i in queries
	{
		results[i] = distance_query_execute(data.world, data.distance_storage, &query);
		if first == .Ok && results[i].status != .Ok
		{
			first = results[i].status;
		}
	}
	return first;
}

shape_closest_point :: proc "contextless" (
	world: ^World, point: Vector3, shape: Shape_Handle, pose_value: Rigid_Pose,
	settings: Distance_Query_Settings = {1e-5, 1e-5, 128},
) -> (Shape_Distance_Result, Status)
{
	data: ^world_data;
	status: Status;
	data, status = query_world_data(world);
	if status != .Ok
	{
		return {}, status;
	}
	query: Distance_Query = {kind=.Closest_Point, point=point, shape_b=shape, pose_b=pose_value, settings=settings};
	result: Distance_Query_Result = distance_query_execute(data, data.distance_storage, &query);
	return result.shape, result.status;
}

shape_distance :: proc "contextless" (
	world: ^World, shape_a: Shape_Handle, pose_a: Rigid_Pose, shape_b: Shape_Handle, pose_b: Rigid_Pose,
	settings: Distance_Query_Settings = {1e-5, 1e-5, 128},
) -> (Shape_Distance_Result, Status)
{
	data: ^world_data;
	status: Status;
	data, status = query_world_data(world);
	if status != .Ok
	{
		return {}, status;
	}
	query: Distance_Query = {kind=.Distance, shape_a=shape_a, shape_b=shape_b, pose_a=pose_a, pose_b=pose_b, settings=settings};
	result: Distance_Query_Result = distance_query_execute(data, data.distance_storage, &query);
	return result.shape, result.status;
}

shape_penetration :: proc "contextless" (
	world: ^World, shape_a: Shape_Handle, pose_a: Rigid_Pose, shape_b: Shape_Handle, pose_b: Rigid_Pose,
	settings: Distance_Query_Settings = {1e-5, 1e-5, 128},
) -> (Shape_Distance_Result, Status)
{
	data: ^world_data;
	status: Status;
	data, status = query_world_data(world);
	if status != .Ok
	{
		return {}, status;
	}
	query: Distance_Query = {kind=.Penetration, shape_a=shape_a, shape_b=shape_b, pose_a=pose_a, pose_b=pose_b, settings=settings};
	result: Distance_Query_Result = distance_query_execute(data, data.distance_storage, &query);
	return result.shape, result.status;
}

shape_depenetrate :: proc "contextless" (
	world: ^World, shape_a: Shape_Handle, pose_a: Rigid_Pose, shape_b: Shape_Handle, pose_b: Rigid_Pose,
	settings: Distance_Query_Settings = {1e-5, 1e-5, 128},
) -> (Shape_Correction_Result, Status)
{
	data: ^world_data;
	status: Status;
	data, status = query_world_data(world);
	if status != .Ok
	{
		return {}, status;
	}
	query: Distance_Query = {kind=.Depenetration, shape_a=shape_a, shape_b=shape_b, pose_a=pose_a, pose_b=pose_b, settings=settings};
	result: Distance_Query_Result = distance_query_execute(data, data.distance_storage, &query);
	return result.correction, result.status;
}

shape_closest_point_with_context :: proc "contextless" (
	query_context: ^Query_Context, point: Vector3, shape: Shape_Handle, pose_value: Rigid_Pose,
	settings: Distance_Query_Settings = {1e-5, 1e-5, 128},
) -> (Shape_Distance_Result, Status)
{
	data: ^query_context_data;
	status: Status;
	data, status = query_context_begin(query_context);
	if status != .Ok
	{
		return {}, status;
	}
	defer data.execution = .Idle;
	query: Distance_Query = {kind=.Closest_Point, point=point, shape_b=shape, pose_b=pose_value, settings=settings};
	result: Distance_Query_Result = distance_query_execute(data.world, data.distance_storage, &query);
	return result.shape, result.status;
}

shape_distance_with_context :: proc "contextless" (
	query_context: ^Query_Context, shape_a: Shape_Handle, pose_a: Rigid_Pose, shape_b: Shape_Handle, pose_b: Rigid_Pose,
	settings: Distance_Query_Settings = {1e-5, 1e-5, 128},
) -> (Shape_Distance_Result, Status)
{
	data: ^query_context_data;
	status: Status;
	data, status = query_context_begin(query_context);
	if status != .Ok
	{
		return {}, status;
	}
	defer data.execution = .Idle;
	query: Distance_Query = {kind=.Distance, shape_a=shape_a, shape_b=shape_b, pose_a=pose_a, pose_b=pose_b, settings=settings};
	result: Distance_Query_Result = distance_query_execute(data.world, data.distance_storage, &query);
	return result.shape, result.status;
}

shape_penetration_with_context :: proc "contextless" (
	query_context: ^Query_Context, shape_a: Shape_Handle, pose_a: Rigid_Pose, shape_b: Shape_Handle, pose_b: Rigid_Pose,
	settings: Distance_Query_Settings = {1e-5, 1e-5, 128},
) -> (Shape_Distance_Result, Status)
{
	data: ^query_context_data;
	status: Status;
	data, status = query_context_begin(query_context);
	if status != .Ok
	{
		return {}, status;
	}
	defer data.execution = .Idle;
	query: Distance_Query = {kind=.Penetration, shape_a=shape_a, shape_b=shape_b, pose_a=pose_a, pose_b=pose_b, settings=settings};
	result: Distance_Query_Result = distance_query_execute(data.world, data.distance_storage, &query);
	return result.shape, result.status;
}

shape_depenetrate_with_context :: proc "contextless" (
	query_context: ^Query_Context, shape_a: Shape_Handle, pose_a: Rigid_Pose, shape_b: Shape_Handle, pose_b: Rigid_Pose,
	settings: Distance_Query_Settings = {1e-5, 1e-5, 128},
) -> (Shape_Correction_Result, Status)
{
	data: ^query_context_data;
	status: Status;
	data, status = query_context_begin(query_context);
	if status != .Ok
	{
		return {}, status;
	}
	defer data.execution = .Idle;
	query: Distance_Query = {kind=.Depenetration, shape_a=shape_a, shape_b=shape_b, pose_a=pose_a, pose_b=pose_b, settings=settings};
	result: Distance_Query_Result = distance_query_execute(data.world, data.distance_storage, &query);
	return result.correction, result.status;
}

// resolve a known body/static once, including sleeping bodies. returned values
// remain snapshots, not borrowed views. handle reuse follows the existing world
// lifetime contract. compose with any shape_* query. no second collision path
collidable_shape_pose :: proc "contextless" (
	world: ^World, collidable: Collidable_Reference,
) -> (Shape_Handle, Rigid_Pose, Status)
{
	data: ^world_data;
	status: Status;
	data, status = query_world_data(world);
	if status != .Ok
	{
		return {}, {}, status;
	}
	if collidable_mobility(collidable) > .Static
	{
		return {}, {}, .Invalid_Argument;
	}
	target: physics.Shape_Query_Target;
	target, status = physics.simulation_query_target(&data.simulation, collidable);
	return target.shape, target.pose, status;
}

collidable_shape_pose_with_context :: proc "contextless" (
	query_context: ^Query_Context, collidable: Collidable_Reference,
) -> (Shape_Handle, Rigid_Pose, Status)
{
	data: ^query_context_data;
	status: Status;
	data, status = query_context_begin(query_context);
	if status != .Ok
	{
		return {}, {}, status;
	}
	defer data.execution = .Idle;
	if collidable_mobility(collidable) > .Static
	{
		return {}, {}, .Invalid_Argument;
	}
	target: physics.Shape_Query_Target;
	target, status = physics.simulation_query_target(&data.world.simulation, collidable);
	return target.shape, target.pose, status;
}
