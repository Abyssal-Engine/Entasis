package large_world_precision


@(require) import "core:fmt"
@(require) import "core:math"
@(require) import "core:time"
import entasis "entasis:entasis"
@(require) import support "../benchmark_support"

Global_Ray_Segment :: struct
{
	start: entasis.Global_Position,
	end: entasis.Global_Position,
	id: i32,
}

SPANNING_HIT_CAPACITY :: DISTRIBUTED_WORLD_COUNT*DISTRIBUTED_GRID_X;

Spanning_Input :: struct
{
	ray: Global_Ray_Segment,
	expectation: Query_Expectation,
	y_index: int,
	z_index: int,
	direction_x: i32,
}

spanning_input :: proc "contextless" (first, last: entasis.Global_Position, index: int) -> Spanning_Input
{
	pair := index/2;
	y_index := pair % DISTRIBUTED_GRID_Y;
	z_index := (pair/DISTRIBUTED_GRID_Y) % DISTRIBUTED_GRID_Z;
	jitter := f64((pair/25)%7-3)*0.03125;
	y := 0.501+f64(y_index)*DISTRIBUTED_SPACING+jitter;
	z := f64(z_index-2)*DISTRIBUTED_SPACING;
	expectation := Query_Expectation.Hit;
	if index%2 != 0
	{
		// the 0.002 m inter-row gap is inside the conservative region bounds
		// a miss therefore still exercises every local tree and ray route
		if z_index == DISTRIBUTED_GRID_Z-1
		{
			z -= 0.501;
		}
		else
		{
			z += 0.501;
		}
		expectation = .Miss;
	}
	start := entasis.global_position(first.x-64, y, z);
	end := entasis.global_position(last.x+64, y, z);
	direction_x := i32(1);
	if pair%2 != 0
	{
		start, end = end, start;
		direction_x = -1;
	}
	return {
		ray={start=start, end=end, id=i32(index)}, expectation=expectation,
		y_index=y_index, z_index=z_index, direction_x=direction_x,
	};
}

when PRECISION_NATIVE_GLOBAL
{

	Global_Spanning_Owner :: struct
	{
		ctx: entasis.Query_Context,
		state: Owner_State,
		stack: [BODY_CAPACITY+DISTRIBUTED_WORLD_COUNT]i32,
	}

	Global_Spanning_Closest_Output :: struct
	{
		hit: entasis.Global_Ray_Hit,
		status: entasis.Status,
	}

	Global_Spanning_All_Output :: struct
	{
		hits: [SPANNING_HIT_CAPACITY]entasis.Global_Ray_Hit,
		count: int,
		status: entasis.Status,
	}

	Global_Spanning_Storage :: struct
	{
		inputs: [QUERY_COUNT]Spanning_Input,
		closest: [QUERY_COUNT]Global_Spanning_Closest_Output,
		all: [QUERY_COUNT]Global_Spanning_All_Output,
	}

	global_spanning_storage: Global_Spanning_Storage;

	global_spanning_execute :: proc(owner: ^Benchmark_Owner, query_count: int, phase: support.Step_Phase) -> support.Benchmark_Result
	{
		checksum: f64;
		filter := entasis.query_filter_mobility(entasis.COLLIDABLE_DYNAMIC);
		for input, index in global_spanning_storage.inputs[:query_count]
		{
			output := &global_spanning_storage.closest[index];
			output.hit, output.status = entasis.ray_cast_global_closest_with_context(
				&owner.spanning.ctx, {start=input.ray.start, end=input.ray.end}, owner.spanning.stack[:], filter,
			);
			if output.status != .Ok && output.status != .Not_Found
			{
				return {status=.Validation_Failed, physics_status=output.status, phase=phase};
			}
			if output.status == .Ok
			{
				// the existing spanning workload reports distance, not a segment fraction
				output.hit.t *= math.abs(input.ray.end.x-input.ray.start.x);
				checksum += output.hit.t + output.hit.location.x*1e-12;
			}
		}
		for input, index in global_spanning_storage.inputs[:query_count]
		{
			output := &global_spanning_storage.all[index];
			output.count, output.status = entasis.ray_cast_global_all_with_context(
				&owner.spanning.ctx, {start=input.ray.start, end=input.ray.end}, output.hits[:], owner.spanning.stack[:], filter,
			);
			if output.status != .Ok
			{
				return {status=.Validation_Failed, physics_status=output.status, phase=phase};
			}
			length := math.abs(input.ray.end.x-input.ray.start.x);
			for &hit in output.hits[:output.count]
			{
				hit.t *= length;
				checksum += hit.t + hit.location.x*1e-12;
			}
		}
		owner.export_checksum += checksum;
		return {status=.Ok, phase=phase};
	}

	global_spanning_prepare :: proc(owner: ^Benchmark_Owner) -> support.Benchmark_Result
	{
		status := entasis.query_context_init(&owner.spanning.ctx, &owner.world,
			{pair_capacity=16, child_capacity=64, traversal_maximum_power=12, traversal_slots_per_power=4});
		if status != .Ok
		{
			return {status=.Creation_Failed, physics_status=status};
		}
		owner.spanning.state = .Ready;
		required, requirement_status := entasis.global_ray_stack_requirement(&owner.world);
		if requirement_status != .Ok || required > len(owner.spanning.stack)
		{
			return {status=.Creation_Failed, physics_status=requirement_status};
		}
		global_spanning_storage = {};
		first := entasis.global_position(1e9, 0, 0);
		last := entasis.global_position(1e9+f64(DISTRIBUTED_WORLD_COUNT-1)*DISTRIBUTED_SEPARATION, 0, 0);
		for index in 0 ..< QUERY_COUNT
		{
			global_spanning_storage.inputs[index] = spanning_input(first, last, index);
		}
		result := global_spanning_execute(owner, QUERY_WARMUP_COUNT, .Warmup);
		owner.export_checksum = 0;
		return result;
	}

	global_spanning_hit_matches :: proc(owner: ^Benchmark_Owner, body_index: int, input: Spanning_Input, hit: entasis.Global_Ray_Hit) -> Precision_Admission
	{
		center := owner.expected_global_positions[body_index];
		x := center.x-f64(input.direction_x)*0.5;
		distance := math.abs(x-input.ray.start.x);
		location := entasis.global_position(x, input.ray.start.y, input.ray.start.z);
		normal := entasis.Vector3{-f32(input.direction_x), 0, 0};
		reference, status := entasis.body_collidable_reference(&owner.world, owner.bodies[body_index]);
		if status != .Ok || hit.collidable != reference || hit.child_index != 0 ||
			precision_close(hit.t, distance, QUERY_POSITION_TOLERANCE) != .Accepted ||
			precision_global_close(hit.location, location, QUERY_POSITION_TOLERANCE) != .Accepted ||
			precision_vector_close(hit.normal, normal, QUERY_DIRECTION_TOLERANCE) != .Accepted
		{
			return .Rejected;
		}
		return .Accepted;
	}

	global_spanning_validate :: proc(owner: ^Benchmark_Owner) -> support.Benchmark_Result
	{
		for input, index in global_spanning_storage.inputs
		{
			closest := &global_spanning_storage.closest[index];
			all := &global_spanning_storage.all[index];
			if all.status != .Ok
			{
				return {status=.Validation_Failed, physics_status=all.status};
			}
			if input.expectation == .Miss
			{
				if closest.status != .Not_Found || all.count != 0
				{
					fmt.eprintfln("global_spanning_miss_failed query=%d closest=%v count=%d", index, closest.status, all.count);
					return {status=.Validation_Failed};
				}
				continue;
			}
			if closest.status != .Ok || all.count != SPANNING_HIT_CAPACITY
			{
				fmt.eprintfln("global_spanning_hit_failed query=%d closest=%v count=%d", index, closest.status, all.count);
				return {status=.Validation_Failed};
			}
			row_start := (input.y_index*DISTRIBUTED_GRID_Z+input.z_index)*DISTRIBUTED_GRID_X;
			nearest := row_start;
			if input.direction_x < 0
			{
				nearest += (DISTRIBUTED_WORLD_COUNT-1)*DISTRIBUTED_BODY_COUNT + DISTRIBUTED_GRID_X-1;
			}
			if global_spanning_hit_matches(owner, nearest, input, closest.hit) != .Accepted
			{
				fmt.eprintfln("global_spanning_closest_geometry_failed query=%d distance=%.17g expected_body=%d", index, closest.hit.t, nearest);
				return {status=.Validation_Failed};
			}
			seen: [SPANNING_HIT_CAPACITY]Owner_State;
			for hit in all.hits[:all.count]
			{
				handle, status := entasis.collidable_body_handle(hit.collidable);
				if status != .Ok || handle.value < 0 || int(handle.value) >= len(owner.bodies)
				{
					return {status=.Validation_Failed};
				}
				body_index := int(handle.value);
				region_index := body_index/DISTRIBUTED_BODY_COUNT;
				x_index := body_index%DISTRIBUTED_BODY_COUNT-row_start;
				if x_index < 0 || x_index >= DISTRIBUTED_GRID_X
				{
					return {status=.Validation_Failed};
				}
				ordinal := region_index*DISTRIBUTED_GRID_X+x_index;
				if seen[ordinal] != .Empty || global_spanning_hit_matches(owner, body_index, input, hit) != .Accepted
				{
					fmt.eprintfln("global_spanning_all_geometry_failed query=%d body=%d distance=%.17g", index, body_index, hit.t);
					return {status=.Validation_Failed};
				}
				seen[ordinal] = .Ready;
			}
		}
		return {status=.Ok};
	}

	global_spanning_measure :: proc(owner: ^Benchmark_Owner) -> (support.Benchmark_Result, support.Benchmark_Sample)
	{
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
			result := global_spanning_execute(owner, QUERY_COUNT, .Measured);
			if result.status != .Ok
			{
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
		sample := support.Benchmark_Sample{elapsed_ms=f64(time.tick_diff(whole_start, time.tick_now()))/1e6};
		if sample.elapsed_ms <= 0 || math.is_nan(sample.elapsed_ms) || math.is_inf(sample.elapsed_ms, 0)
		{
			return {status=.Validation_Failed}, {};
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
				fmt.printfln("precision_query_pass candidate=%s case=queries-spanning pass=%d elapsed_ms=%.9f rays_per_family=%d families=2",
					PRECISION_CANDIDATE, pass_index+1, f64(duration)/1e6, QUERY_COUNT);
			}
		}
		if mode == .Memory
		{
			precision_memory_emit(owner.memory, size_of(Benchmark_Storage)+size_of(Benchmark_Owner)+size_of(Global_Spanning_Storage)+size_of(Global_Spanning_Owner));
		}
		return result, sample;
	}
}
else
{
	Global_Spanning_Owner :: struct
	{
	}
}
