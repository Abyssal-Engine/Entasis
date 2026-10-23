package spatial_query_trace

import "core:fmt"
import "core:math"
import "core:os"
import entasis "entasis:entasis"

CSV_HEADER :: "worker_count,static_body_count,shape_count,query_count,warmup_batches,measured_batches,ray_query_count,sphere_cast_query_count,overlap_query_count,ray_hit_count,sphere_cast_hit_count,overlap_hit_count,ray_checksum,sphere_cast_checksum,overlap_checksum,ray_elapsed_ms,sphere_cast_elapsed_ms,overlap_elapsed_ms,queries_per_second,ray_queries_per_second,sphere_cast_queries_per_second,overlap_queries_per_second,case_status,metric_status,physics_elapsed_ms,precision,ray_pattern";

Argument_Status :: enum u8
{
	Ok,
	Invalid,
}

Argument_Option :: enum u8
{
	Worker_Count,
	Output,
	Ray_Pattern,
	Family, Warmup, Measured,
}

Arguments :: struct
{
	worker_count: int,
	output_path:  string,
	ray_pattern:  Ray_Pattern,
	query_family: string,
	families: bit_set[Query_Family],
	warmup_batches, measured_batches: int,
}

argument_value :: proc "contextless" (argument, prefix: string) -> (string, Argument_Status)
{
	if len(argument) <= len(prefix) || argument[:len(prefix)] != prefix
	{
		return "", .Invalid;
	}
	return argument[len(prefix):], .Ok;
}

parse_positive_integer :: proc "contextless" (value: string) -> (int, Argument_Status)
{
	if len(value) == 0
	{
		return 0, .Invalid;
	}
	result := 0;
	for character in value
	{
		if character < '0' || character > '9'
		{
			return 0, .Invalid;
		}
		digit := int(character - '0');
		if result > (max(int) - digit) / 10
		{
			return 0, .Invalid;
		}
		result = result * 10 + digit;
	}
	if result <= 0
	{
		return 0, .Invalid;
	}
	return result, .Ok;
}

Query_Family_Policy :: enum
{
	Allow, Reject,
}

parse_arguments :: proc "contextless" (arguments: []string, allow_query_family: Query_Family_Policy = .Allow) -> (Arguments, Argument_Status)
{
	if len(arguments) < 3 || len(arguments) > 7
	{
		return {}, .Invalid;
	}
	result: Arguments = {query_family="all", families={.Ray, .Sphere_Cast, .Overlap}, warmup_batches=WARMUP_BATCH_COUNT, measured_batches=MEASURED_BATCH_COUNT};
	seen: bit_set[Argument_Option; u8];
	for argument in arguments[1:]
	{
		value, status := argument_value(argument, "--worker-count=");
		if status == .Ok
		{
			if .Worker_Count in seen
			{
				return {}, .Invalid;
			}
			result.worker_count, status = parse_positive_integer(value);
			if status != .Ok || result.worker_count > MAXIMUM_WORKER_COUNT
			{
				return {}, .Invalid;
			}
			seen += {.Worker_Count};
			continue;
		}
		value, status = argument_value(argument, "--output=");
		if status == .Ok
		{
			if .Output in seen || len(value) == 0
			{
				return {}, .Invalid;
			}
			result.output_path = value;
			seen += {.Output};
			continue;
		}
		value, status = argument_value(argument, "--ray-pattern=");
		if status == .Ok
		{
			if .Ray_Pattern in seen
			{
				return {}, .Invalid;
			}
			switch value
			{
			case "axial": result.ray_pattern = .Axial;
			case "oblique": result.ray_pattern = .Oblique;
			case: return {}, .Invalid;
			}
			seen += {.Ray_Pattern};
			continue;
		}
		value, status = argument_value(argument, "--query-family=");
		if status == .Ok
		{
			if .Family in seen || allow_query_family == .Reject
			{
				return {}, .Invalid;
			}
			switch value
			{
			case "all": result.families = {.Ray, .Sphere_Cast, .Overlap};
			case "ray": result.families = {.Ray};
			case "sphere-cast": result.families = {.Sphere_Cast};
			case "aabb-presence": result.families = {.Overlap};
			case: return {}, .Invalid;
			}
			result.query_family = value;
			seen += {.Family};
			continue;
		}
		for option in ([]Argument_Option{.Warmup, .Measured})
		{
			prefix: string = "--warmup-batches=" if option == .Warmup else "--measured-batches=";
			value, status = argument_value(argument, prefix);
			if status == .Ok
			{
				count: int;
				count, status = parse_positive_integer(value);
				if status != .Ok || count > 1_000_000 || option in seen
				{
					return {}, .Invalid;
				}
				if option == .Warmup
				{
					result.warmup_batches = count;
				}
				else
				{
					result.measured_batches = count;
				}
				seen += {option};
				break;
			}
		}
		if status == .Ok
		{
			continue;
		}
		return {}, .Invalid;
	}
	if .Worker_Count not_in seen || .Output not_in seen || (result.ray_pattern == .Oblique && .Ray not_in result.families)
	{
		return {}, .Invalid;
	}
	return result, .Ok;
}

finite_positive :: #force_inline proc "contextless" (value: f64) -> Argument_Status
{
	return .Ok if value > 0 && !math.is_nan(value) && !math.is_inf(value) else .Invalid;
}

write_result :: proc(path: string, worker_count: int, result: Benchmark_Result, arguments: Arguments) -> Case_Status
{
	_, stat_error := os.stat(path, context.temp_allocator);
	file, open_error := os.open(path, {.Write, .Append, .Create}, os.Permissions_Default_File);
	if open_error != nil || file == nil
	{
		return .Validation_Failed;
	}
	if stat_error != nil
	{
		if fmt.fprint(file, CSV_HEADER) <= 0 ||
			(arguments.query_family != "all" && fmt.fprint(file, ",query_family") <= 0) || fmt.fprintln(file) <= 0
		{
			_ = os.close(file);
			return .Validation_Failed;
		}
	}
	counts: [3]int = {RAY_QUERY_COUNT, SPHERE_CAST_QUERY_COUNT, OVERLAP_QUERY_COUNT};
	hits: [3]u64 = {result.ray_hits, result.sphere_cast_hits, result.overlap_hits};
	checksums: [3]u64 = {result.ray_checksum, result.sphere_cast_checksum, result.overlap_checksum};
	nanoseconds: [3]u64 = {result.ray_nanoseconds, result.sphere_nanoseconds, result.overlap_nanoseconds};
	query_count: int;
	physics_ms: f64;
	for family in Query_Family
	{
		if family in arguments.families
		{
			query_count += counts[int(family)];
			physics_ms += f64(nanoseconds[int(family)])/1e6;
		}
	}
	if finite_positive(physics_ms) != .Ok
	{
		_ = os.close(file);
		return .Validation_Failed;
	}
	written: int = fmt.fprintf(file, "%d,%d,2,%d,%d,%d", worker_count, STATIC_BODY_COUNT, query_count, arguments.warmup_batches, arguments.measured_batches);
	for values in ([3][3]u64{{u64(counts[0]), u64(counts[1]), u64(counts[2])}, hits, checksums})
	{
		for family in Query_Family
		{
			if family in arguments.families
			{
				written += fmt.fprintf(file, ",%d", values[int(family)]);
			}
			else
			{
				written += fmt.fprint(file, ",excluded");
			}
		}
	}
	for family in Query_Family
	{
		if family in arguments.families
		{
			written += fmt.fprintf(file, ",%.9f", f64(nanoseconds[int(family)])/1e6);
		}
		else
		{
			written += fmt.fprint(file, ",excluded");
		}
	}
	written += fmt.fprintf(file, ",%.9f", f64(query_count)*f64(arguments.measured_batches)*1000/physics_ms);
	for family in Query_Family
	{
		if family in arguments.families
		{
			written += fmt.fprintf(file, ",%.9f", f64(counts[int(family)])*f64(arguments.measured_batches)*1e9/f64(nanoseconds[int(family)]));
		}
		else
		{
			written += fmt.fprint(file, ",excluded");
		}
	}
	written += fmt.fprintf(file, ",ok,ok,%.9f,%s,%s", physics_ms, entasis.PRECISION_NAME, ray_pattern_name(arguments.ray_pattern));
	if arguments.query_family != "all"
	{
		written += fmt.fprintf(file, ",%s", arguments.query_family);
	}
	written += fmt.fprintln(file);
	if os.close(file) != nil || written <= 0
	{
		return .Validation_Failed;
	}
	return .Ok;
}

run_benchmark :: proc(arguments: Arguments) -> Benchmark_Result
{
	pool: entasis.Buffer_Pool;
	if entasis.buffer_pool_init(&pool, POOL_MINIMUM_BLOCK_SIZE) != .Ok
	{
		return {status=.Allocation_Failed};
	}
	defer
	{
		_ = entasis.buffer_pool_destroy(&pool);
	}
	owner: Benchmark_Owner;
	owner_status := benchmark_owner_create(&owner, &pool, arguments.worker_count, arguments.ray_pattern);
	if owner_status != .Ok
	{
		return {status=owner_status};
	}
	result := benchmark_execute(&owner, arguments.families, arguments.warmup_batches, arguments.measured_batches);
	if benchmark_owner_destroy(&owner) != .Ok
	{
		return {status=.Release_Failed};
	}
	if result.status != .Ok
	{
		return result;
	}
	result.status = write_result(arguments.output_path, arguments.worker_count, result, arguments);
	return result;
}

main :: proc()
{
	arguments, status := parse_arguments(os.args);
	if status != .Ok
	{
		fmt.eprintln("usage: entasis_spatial_query_trace --worker-count=<positive count up to engine maximum> --output=<path> [--ray-pattern=axial|oblique]");
		os.exit(2);
	}
	result := run_benchmark(arguments);
	if result.status != .Ok
	{
		fmt.eprintfln(
			"benchmark_failed status=%v physics_status=%v",
			result.status,
			result.physics_status,
		);
		os.exit(2);
	}
	os.exit(0);
}
