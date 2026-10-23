package large_world_precision


import "core:fmt"

import "core:math"
import "core:time"
import entasis "entasis:entasis"
import cooking "entasis:entasis_cooking"
import support "../benchmark_support"

QUERY_COUNT :: 1_024;
QUERY_WARMUP_COUNT :: 128;
QUERY_PASS_COUNT :: 8;
QUERY_TARGET_COUNT :: 3;
QUERY_OVERLAP_CAPACITY :: 4;
QUERY_POSITION_TOLERANCE :: 0.001;
QUERY_DIRECTION_TOLERANCE :: 0.0001;
QUERY_PARAMETER_TOLERANCE :: 0.0002;
QUERY_SPHERE_RADIUS :: f32(0.25);

Query_Local_State :: enum u8
{
	Empty,
	Ready,
}

Query_Expectation :: enum u8
{
	Miss,
	Hit,
}

Query_Ray_Input :: struct
{
	origin: entasis.Global_Position,
	direction: entasis.Vector3,
	maximum_t: f32,
	expectation: Query_Expectation,
	target_index: i32,
	child_index: i32,
	expected_t: f32,
	expected_location: entasis.Global_Position,
	expected_normal: entasis.Vector3,
}

Query_Ray_Output :: struct
{
	hit: Query_Candidate_Ray_Hit,
	location: entasis.Global_Position,
	status: entasis.Status,
}

Query_Sweep_Input :: struct
{
	origin: entasis.Global_Position,
	velocity: entasis.Vector3,
	maximum_t: f32,
	expectation: Query_Expectation,
	target_index: i32,
	child_index: i32,
	expected_t: f32,
	expected_location: entasis.Global_Position,
	expected_normal: entasis.Vector3,
}

Query_Sweep_Output :: struct
{
	hit: Query_Candidate_Sweep_Hit,
	location: entasis.Global_Position,
	status: entasis.Status,
}

Query_Overlap_Input :: struct
{
	origin: entasis.Global_Position,
	expectation: Query_Expectation,
	target_index: i32,
}

Query_Overlap_Output :: struct
{
	collidable: entasis.Collidable_Reference,
	count: int,
	status: entasis.Status,
}

Query_Distance_Input :: struct
{
	query_origin: entasis.Global_Position,
	target_origin: entasis.Global_Position,
	target_index: i32,
	child_index: i32,
	expected_distance: f32,
	expected_point_a: entasis.Global_Position,
	expected_point_b: entasis.Global_Position,
	expected_normal: entasis.Vector3,
}

Query_Distance_Output :: struct
{
	result: Query_Candidate_Distance_Result,
	point_a: entasis.Global_Position,
	point_b: entasis.Global_Position,
	status: entasis.Status,
}

Query_Local_Storage :: struct
{
	rays: [QUERY_COUNT]Query_Ray_Input,
	ray_outputs: [QUERY_COUNT]Query_Ray_Output,
	sweeps: [QUERY_COUNT]Query_Sweep_Input,
	sweep_outputs: [QUERY_COUNT]Query_Sweep_Output,
	overlaps: [QUERY_COUNT]Query_Overlap_Input,
	overlap_outputs: [QUERY_COUNT]Query_Overlap_Output,
	distances: [QUERY_COUNT]Query_Distance_Input,
	distance_outputs: [QUERY_COUNT]Query_Distance_Output,
}

query_local_storage: Query_Local_Storage;

Query_Local_Owner :: struct
{
	ctx: entasis.Query_Context,
	state: Query_Local_State,
	query_shape: entasis.Shape_Handle,
	target_shapes: [QUERY_TARGET_COUNT]entasis.Shape_Handle,
	target_collidables: [QUERY_TARGET_COUNT]entasis.Collidable_Reference,
	target_origins: [QUERY_TARGET_COUNT]entasis.Global_Position,
	rays: []Query_Ray_Input,
	ray_outputs: []Query_Ray_Output,
	sweeps: []Query_Sweep_Input,
	sweep_outputs: []Query_Sweep_Output,
	overlaps: []Query_Overlap_Input,
	overlap_outputs: []Query_Overlap_Output,
	distances: []Query_Distance_Input,
	distance_outputs: []Query_Distance_Output,
	overlap_hits: [QUERY_OVERLAP_CAPACITY]entasis.Overlap_Hit,
	traversal_stack: Query_Candidate_Traversal,
}

query_local_filter :: #force_inline proc "contextless" () -> entasis.Query_Filter
{
	return entasis.query_filter_mobility(entasis.COLLIDABLE_STATIC);
}

query_local_add_static :: proc(
	owner: ^Benchmark_Owner,
	shape: entasis.Shape_Handle,
	logical_position: entasis.Vector3,
	target_index: int,
) -> entasis.Status
{
	global := global_from_local_fixture(owner, logical_position);
	handle: entasis.Static_Handle;
	status: entasis.Status;
	handle, status = candidate_static_add(owner, shape, global, .None);
	if status != .Ok
	{
		return status;
	}
	owner.query.target_collidables[target_index], status = entasis.static_collidable_reference(
		&owner.world,
		handle,
	);
	if status != .Ok
	{
		return status;
	}
	owner.query.target_origins[target_index] = global;
	return .Ok;
}

query_local_import_mesh :: proc(owner: ^Benchmark_Owner) -> (entasis.Shape_Handle, entasis.Status)
{
	ctx: cooking.Cooking_Context;
	status := cooking.cooking_context_init(&ctx, 65_536);
	if status != .Ok
	{
		return {}, status;
	}
	triangles := [2]entasis.Triangle{
		entasis.triangle({-2, 0, -2}, {2, 0, -2}, {2, 0, 2}),
		entasis.triangle({-2, 0, -2}, {2, 0, 2}, {-2, 0, 2}),
	};
	cooked: cooking.Cooked_Mesh;
	cooked, status = cooking.cook_mesh(&ctx, triangles[:]);
	if status != .Ok
	{
		_ = cooking.cooking_context_destroy(&ctx);
		return {}, status;
	}
	shape: entasis.Shape_Handle;
	shape, status = cooking.cooked_mesh_import(&owner.world, &cooked);
	destroy_status := cooking.cooked_mesh_destroy(&cooked);
	context_status := cooking.cooking_context_destroy(&ctx);
	if status == .Ok
	{
		status = destroy_status;
	}
	if status == .Ok
	{
		status = context_status;
	}
	return shape, status;
}

query_local_build_fixture :: proc(owner: ^Benchmark_Owner) -> entasis.Status
{
	status: entasis.Status;
	owner.query.query_shape, status = entasis.shape_add(
		&owner.world,
		entasis.sphere(QUERY_SPHERE_RADIUS),
	);
	if status != .Ok
	{
		return status;
	}
	owner.query.target_shapes[0], status = entasis.shape_add(
		&owner.world,
		entasis.box(2, 2, 2),
	);
	if status != .Ok
	{
		return status;
	}
	compound_child_shape: entasis.Shape_Handle;
	compound_child_shape, status = entasis.shape_add(
		&owner.world,
		entasis.box(1, 1, 1),
	);
	if status != .Ok
	{
		return status;
	}
	children := [2]entasis.Compound_Child{
		entasis.compound_child(compound_child_shape, entasis.pose({0, 0, -1.5})),
		entasis.compound_child(compound_child_shape, entasis.pose({0, 0, 1.5})),
	};
	owner.query.target_shapes[1], status = entasis.shape_import_compound(
		&owner.world,
		children[:],
	);
	if status != .Ok
	{
		return status;
	}
	owner.query.target_shapes[2], status = query_local_import_mesh(owner);
	if status != .Ok
	{
		return status;
	}
	positions := [QUERY_TARGET_COUNT]entasis.Vector3{
		{-12, 0, 0},
		{0, 0, 0},
		{12, 0, 0},
	};
	for position, target_index in positions
	{
		status = query_local_add_static(
			owner,
			owner.query.target_shapes[target_index],
			position,
			target_index,
		);
		if status != .Ok
		{
			return status;
		}
	}
	status = entasis.query_context_init(
		&owner.query.ctx,
		&owner.world,
		{pair_capacity=16, child_capacity=64, traversal_maximum_power=12, traversal_slots_per_power=4},
	);
	if status != .Ok
	{
		return status;
	}
	owner.query.state = .Ready;
	when PRECISION_NATIVE_GLOBAL
	{
		required: int;
		required, status = entasis.global_bounds_stack_requirement(&owner.world);
		if status != .Ok
		{
			return status;
		}
		if required > len(owner.query.traversal_stack)
		{
			return .Capacity_Missing;
		}
	}
	return .Ok;
}

query_local_probe :: #force_inline proc "contextless" (
	target_index: int,
	jitter: f32,
) -> (entasis.Vector3, f32, i32)
{
	switch target_index
	{
		case 0: return {-12, 0, jitter}, 1, 0;
		case 1: return {0, 0, -1.5+jitter}, 0.5, 0;
		case 2: return {12.75+jitter*0.25, 0, -0.5+jitter}, 0, 0;
	}
	unreachable();
}

query_local_generate_inputs :: proc(owner: ^Benchmark_Owner)
{
	for index in 0 ..< QUERY_COUNT
	{
		target_index := index % QUERY_TARGET_COUNT;
		jitter := f32((index/6)%17-8)*0.0075;
		probe, surface_y, child_index := query_local_probe(target_index, jitter);
		expectation := Query_Expectation.Hit;
		query_x := probe.x;
		if index%2 != 0
		{
			expectation = .Miss;
			query_x += 4;
		}
		ray_origin := entasis.Vector3{query_x, 4, probe.z};
		owner.query.rays[index] = {
			origin=global_from_local_fixture(owner, ray_origin),
			direction={0, -1, 0},
			maximum_t=8,
			expectation=expectation,
			target_index=i32(target_index),
			child_index=child_index,
			expected_t=4-surface_y,
			expected_location=global_from_local_fixture(owner, {probe.x, surface_y, probe.z}),
			expected_normal={0, 1, 0},
		};
		sweep_child := child_index;
		if target_index == 0
		{
			sweep_child = -1;
		}
		owner.query.sweeps[index] = {
			origin=global_from_local_fixture(owner, ray_origin),
			velocity={0, -4, 0},
			maximum_t=2,
			expectation=expectation,
			target_index=i32(target_index),
			child_index=sweep_child,
			expected_t=(4-(surface_y+QUERY_SPHERE_RADIUS))/4,
			expected_location=global_from_local_fixture(owner, {probe.x, surface_y, probe.z}),
			expected_normal={0, 1, 0},
		};
		overlap_x := probe.x;
		if expectation == .Miss
		{
			overlap_x += 4;
		}
		owner.query.overlaps[index] = {
			origin=global_from_local_fixture(owner, {overlap_x, surface_y+0.15, probe.z}),
			expectation=expectation,
			target_index=i32(target_index),
		};
		gap := f32(0.5)+f32(index%8)*0.0625;
		query_origin := entasis.Vector3{probe.x, surface_y+QUERY_SPHERE_RADIUS+gap, probe.z};
		distance_child := i32(-1);
		if target_index != 0
		{
			distance_child = child_index;
		}
		owner.query.distances[index] = {
			query_origin=global_from_local_fixture(owner, query_origin),
			target_origin=owner.query.target_origins[target_index],
			target_index=i32(target_index),
			child_index=distance_child,
			expected_distance=gap,
			expected_point_a=global_from_local_fixture(owner, {probe.x, surface_y+gap, probe.z}),
			expected_point_b=global_from_local_fixture(owner, {probe.x, surface_y, probe.z}),
			expected_normal={0, 1, 0},
		};
	}
}

query_local_execute :: proc(
	owner: ^Benchmark_Owner,
	query_count: int,
	phase: support.Step_Phase,
) -> support.Benchmark_Result
{
	when PRECISION_NATIVE_GLOBAL
	{
		return query_local_execute_global(owner, query_count, phase);
	}
	else
	{
		filter := query_local_filter();
		checksum: f64;
		for input, index in owner.query.rays[:query_count]
		{
			local, local_status := candidate_local_position(owner, input.origin);
			if local_status != .Ok
			{
				return {status=.Validation_Failed, physics_status=local_status, phase=phase};
			}
			output := &owner.query.ray_outputs[index];
			output.hit, output.status = entasis.ray_cast_closest_with_context(
				&owner.query.ctx,
				entasis.ray(local, input.direction, input.maximum_t),
				filter,
			);
			if output.status == .Ok
			{
				output.location, local_status = candidate_global_position(owner, output.hit.location);
				if local_status != .Ok
				{
					return {status=.Validation_Failed, physics_status=local_status, phase=phase};
				}
				checksum += f64(output.hit.t) + f64(output.hit.collidable.packed)*1e-6 + output.location.x*1e-12;
			}
		}
		for input, index in owner.query.sweeps[:query_count]
		{
			local, local_status := candidate_local_position(owner, input.origin);
			if local_status != .Ok
			{
				return {status=.Validation_Failed, physics_status=local_status, phase=phase};
			}
			output := &owner.query.sweep_outputs[index];
			output.hit, output.status = entasis.sweep_closest_with_context(
				&owner.query.ctx,
				owner.query.query_shape,
				entasis.pose(local),
				entasis.velocity(input.velocity),
				input.maximum_t,
				filter,
			);
			if output.status == .Ok
			{
				output.location, local_status = candidate_global_position(owner, output.hit.sweep.location);
				if local_status != .Ok
				{
					return {status=.Validation_Failed, physics_status=local_status, phase=phase};
				}
				checksum += f64(output.hit.sweep.t1) + f64(output.hit.collidable.packed)*1e-6 + output.location.y*1e-6;
			}
		}
		for input, index in owner.query.overlaps[:query_count]
		{
			local, local_status := candidate_local_position(owner, input.origin);
			if local_status != .Ok
			{
				return {status=.Validation_Failed, physics_status=local_status, phase=phase};
			}
			output := &owner.query.overlap_outputs[index];
			output.count, output.status = entasis.overlap_all_with_context(
				&owner.query.ctx,
				owner.query.query_shape,
				entasis.pose(local),
				owner.query.overlap_hits[:],
				filter,
			);
			if output.count > 0
			{
				output.collidable = owner.query.overlap_hits[0].collidable;
			}
			checksum += f64(output.count) + f64(output.collidable.packed)*1e-6;
		}
		for input, index in owner.query.distances[:query_count]
		{
			query_local, query_status := candidate_local_position(owner, input.query_origin);
			if query_status != .Ok
			{
				return {status=.Validation_Failed, physics_status=query_status, phase=phase};
			}
			target_local, target_status := candidate_local_position(owner, input.target_origin);
			if target_status != .Ok
			{
				return {status=.Validation_Failed, physics_status=target_status, phase=phase};
			}
			output := &owner.query.distance_outputs[index];
			output.result, output.status = entasis.shape_distance_with_context(
				&owner.query.ctx,
				owner.query.query_shape,
				entasis.pose(query_local),
				owner.query.target_shapes[input.target_index],
				entasis.pose(target_local),
			);
			if output.status == .Ok
			{
				output.point_a, query_status = candidate_global_position(owner, output.result.geometry.point_a);
				if query_status != .Ok
				{
					return {status=.Validation_Failed, physics_status=query_status, phase=phase};
				}
				output.point_b, query_status = candidate_global_position(owner, output.result.geometry.point_b);
				if query_status != .Ok
				{
					return {status=.Validation_Failed, physics_status=query_status, phase=phase};
				}
				checksum += f64(output.result.geometry.distance) + output.point_a.y*1e-6 + output.point_b.y*1e-6;
			}
		}
		owner.export_checksum += checksum;
		return {status=.Ok, physics_status=.Ok, phase=phase};
	}
}

query_local_prepare :: proc(owner: ^Benchmark_Owner) -> support.Benchmark_Result
{
	query_local_storage = {};
	owner.query.rays = query_local_storage.rays[:];
	owner.query.ray_outputs = query_local_storage.ray_outputs[:];
	owner.query.sweeps = query_local_storage.sweeps[:];
	owner.query.sweep_outputs = query_local_storage.sweep_outputs[:];
	owner.query.overlaps = query_local_storage.overlaps[:];
	owner.query.overlap_outputs = query_local_storage.overlap_outputs[:];
	owner.query.distances = query_local_storage.distances[:];
	owner.query.distance_outputs = query_local_storage.distance_outputs[:];
	query_local_generate_inputs(owner);
	result := query_local_execute(owner, QUERY_WARMUP_COUNT, .Warmup);
	owner.export_checksum = 0;
	return result;
}

query_local_validate :: proc(
	owner: ^Benchmark_Owner,
	_sample: ^support.Benchmark_Sample,
) -> support.Benchmark_Result
{
	for input, index in owner.query.rays
	{
		output := owner.query.ray_outputs[index];
		if input.expectation == .Miss
		{
			if output.status != .Not_Found
			{
				return {status=.Validation_Failed, physics_status=output.status};
			}
			continue;
		}
		if output.status != .Ok ||
			output.hit.collidable != owner.query.target_collidables[input.target_index] ||
			output.hit.child_index != input.child_index ||
			precision_close(f64(output.hit.t), f64(input.expected_t), QUERY_PARAMETER_TOLERANCE) != .Accepted ||
			precision_global_close(output.location, input.expected_location, QUERY_POSITION_TOLERANCE) != .Accepted ||
			precision_vector_close(output.hit.normal, input.expected_normal, QUERY_DIRECTION_TOLERANCE) != .Accepted
		{
			return {status=.Validation_Failed, physics_status=output.status};
		}
	}
	for input, index in owner.query.sweeps
	{
		output := owner.query.sweep_outputs[index];
		if input.expectation == .Miss
		{
			if output.status != .Not_Found
			{
				return {status=.Validation_Failed, physics_status=output.status};
			}
			continue;
		}
		if output.status != .Ok || output.hit.sweep.state != .Hit ||
			output.hit.collidable != owner.query.target_collidables[input.target_index] ||
			output.hit.sweep.child_b != input.child_index ||
			precision_close(f64(output.hit.sweep.t1), f64(input.expected_t), QUERY_PARAMETER_TOLERANCE) != .Accepted ||
			precision_global_close(output.location, input.expected_location, QUERY_POSITION_TOLERANCE) != .Accepted ||
			precision_vector_close(output.hit.sweep.normal, input.expected_normal, QUERY_DIRECTION_TOLERANCE) != .Accepted
		{
			return {status=.Validation_Failed, physics_status=output.status};
		}
	}
	for input, index in owner.query.overlaps
	{
		output := owner.query.overlap_outputs[index];
		expected_count := 0;
		if input.expectation == .Hit
		{
			expected_count = 1;
		}
		if output.status != .Ok || output.count != expected_count
		{
			return {status=.Validation_Failed, physics_status=output.status};
		}
		if expected_count == 1 &&
			output.collidable != owner.query.target_collidables[input.target_index]
		{
			return {status=.Validation_Failed};
		}
	}
	for input, index in owner.query.distances
	{
		output := owner.query.distance_outputs[index];
		geometry := output.result.geometry;
		if output.status != .Ok || geometry.state != .Separated ||
			output.result.child_a != -1 || output.result.child_b != input.child_index ||
			precision_close(f64(geometry.distance), f64(input.expected_distance), QUERY_POSITION_TOLERANCE) != .Accepted ||
			precision_global_close(output.point_a, input.expected_point_a, QUERY_POSITION_TOLERANCE) != .Accepted ||
			precision_global_close(output.point_b, input.expected_point_b, QUERY_POSITION_TOLERANCE) != .Accepted ||
			precision_vector_close(geometry.normal, input.expected_normal, QUERY_DIRECTION_TOLERANCE) != .Accepted
		{
			return {status=.Validation_Failed, physics_status=output.status};
		}
	}
	return {status=.Ok};
}

query_local_measure :: proc(owner: ^Benchmark_Owner) -> (support.Benchmark_Result, support.Benchmark_Sample)
{
	owner.export_checksum = 0;
	mode := owner.sampling_mode;
	if mode == .Memory
	{
		frame: Memory_Frame;
		status := precision_memory_world(&owner.world, &frame);
		if status != .Ok
		{
			return {status=.Validation_Failed, physics_status=status}, {};
		}
		owner.memory.initial = frame;
		precision_memory_begin(owner.memory);
	}
	whole_start := time.tick_now();
	for pass_index in 0 ..< QUERY_PASS_COUNT
	{
		pass_start: time.Tick;
		if mode == .Instrumented
		{
			pass_start = time.tick_now();
		}
		result := query_local_execute(owner, QUERY_COUNT, .Measured);
		if result.status != .Ok
		{
			result.failed_step = pass_index+1;
			return result, {};
		}
		if mode == .Instrumented
		{
			owner.frame_nanoseconds[pass_index] = i64(time.tick_diff(pass_start, time.tick_now()));
		}
		if mode == .Memory
		{
			frame: Memory_Frame;
			status := precision_memory_world(&owner.world, &frame);
			if status != .Ok
			{
				return {status=.Validation_Failed, physics_status=status}, {};
			}
			precision_memory_record(owner.memory, frame);
		}
	}
	whole_end := time.tick_now();
	sample := support.Benchmark_Sample{
		elapsed_ms=f64(time.tick_diff(whole_start, whole_end))/1e6,
	};
	if sample.elapsed_ms <= 0 || math.is_nan(sample.elapsed_ms) || math.is_inf(sample.elapsed_ms, 0)
	{
		return {status=.Validation_Failed, phase=.Measured}, {};
	}
	if mode == .Instrumented
	{
		sampling_summarize(owner, QUERY_PASS_COUNT);
	}
	result := benchmark_validate(owner, &sample);
	if result.status == .Ok && mode == .Instrumented
	{
		for duration, pass_index in owner.frame_nanoseconds[:QUERY_PASS_COUNT]
		{
			fmt.printfln("precision_query_pass candidate=%s case=queries-local pass=%d elapsed_ms=%.9f queries_per_family=%d families=4",
				PRECISION_CANDIDATE, pass_index+1, f64(duration)/1e6, QUERY_COUNT);
		}
	}
	if mode == .Memory
	{
		precision_memory_emit(owner.memory, size_of(Benchmark_Storage)+size_of(Benchmark_Owner)+size_of(Query_Local_Storage));
	}
	return result, sample;
}

query_local_destroy :: proc(owner: ^Benchmark_Owner) -> entasis.Status
{
	status := entasis.query_context_destroy(&owner.query.ctx);
	if status == .Ok
	{
		owner.query.state = .Empty;
	}
	return status;
}

when PRECISION_NATIVE_GLOBAL
{
	Query_Candidate_Ray_Hit :: entasis.Global_Ray_Hit;
	Query_Candidate_Sweep_Hit :: entasis.Global_Sweep_Hit;
	Query_Candidate_Distance_Result :: entasis.Global_Shape_Distance_Result;
	Query_Candidate_Traversal :: [QUERY_STATIC_COUNT * 2 + 2]i32;

	// the same immutable inputs, filters, shape registry and output validation
	// serve origin and mixed64. native f64 witnesses never pass through a float
	// cache. conversion of segment fraction to the fixture's ray parameter is
	// charged to the measured operation, just like origin-frame reconstruction
	query_local_execute_global :: proc(
		owner: ^Benchmark_Owner, query_count: int, phase: support.Step_Phase,
	) -> support.Benchmark_Result
	{
		filter: entasis.Query_Filter = query_local_filter();
		checksum: f64;
		for input, index in owner.query.rays[:query_count]
		{
			segment: entasis.World_Precision_Segment = {
				start=input.origin,
				end={input.origin.x + f64(input.direction.x) * f64(input.maximum_t),
					input.origin.y + f64(input.direction.y) * f64(input.maximum_t),
					input.origin.z + f64(input.direction.z) * f64(input.maximum_t)},
			};
			output: ^Query_Ray_Output = &owner.query.ray_outputs[index];
			output.hit, output.status = entasis.ray_cast_global_closest_with_context(
				&owner.query.ctx, segment, owner.query.traversal_stack[:], filter);
			if output.status == .Ok
			{
				output.hit.t *= f64(input.maximum_t);
				output.location = output.hit.location;
				checksum += output.hit.t + f64(output.hit.collidable.packed) * 1e-6 + output.location.x * 1e-12;
			}
		}
		for input, index in owner.query.sweeps[:query_count]
		{
			output: ^Query_Sweep_Output = &owner.query.sweep_outputs[index];
			output.hit, output.status = entasis.sweep_global_closest_with_context(
				&owner.query.ctx, owner.query.query_shape, {position=input.origin, orientation={w=1}},
				entasis.velocity(input.velocity), input.maximum_t, owner.query.traversal_stack[:], filter);
			if output.status == .Ok
			{
				output.location = output.hit.sweep.location;
				checksum += f64(output.hit.sweep.t1) + f64(output.hit.collidable.packed) * 1e-6 + output.location.y * 1e-6;
			}
		}
		for input, index in owner.query.overlaps[:query_count]
		{
			output: ^Query_Overlap_Output = &owner.query.overlap_outputs[index];
			output.count, output.status = entasis.overlap_global_all_with_context(
				&owner.query.ctx, owner.query.query_shape, {position=input.origin, orientation={w=1}},
				owner.query.overlap_hits[:], owner.query.traversal_stack[:], filter);
			if output.count > 0
			{
				output.collidable = owner.query.overlap_hits[0].collidable;
			}
			checksum += f64(output.count) + f64(output.collidable.packed) * 1e-6;
		}
		for input, index in owner.query.distances[:query_count]
		{
			output: ^Query_Distance_Output = &owner.query.distance_outputs[index];
			output.result, output.status = entasis.shape_distance_global_with_context(
				&owner.query.ctx, owner.query.query_shape, {position=input.query_origin, orientation={w=1}},
				owner.query.target_shapes[input.target_index], {position=input.target_origin, orientation={w=1}});
			if output.status == .Ok
			{
				output.point_a = output.result.geometry.point_a;
				output.point_b = output.result.geometry.point_b;
				checksum += f64(output.result.geometry.distance) + output.point_a.y * 1e-6 + output.point_b.y * 1e-6;
			}
		}
		owner.export_checksum += checksum;
		return {status=.Ok, physics_status=.Ok, phase=phase};
	}
}
else
{
	Query_Candidate_Ray_Hit :: entasis.Ray_Hit;
	Query_Candidate_Sweep_Hit :: entasis.Sweep_Hit;
	Query_Candidate_Distance_Result :: entasis.Shape_Distance_Result;
	Query_Candidate_Traversal :: [0]i32;
}
