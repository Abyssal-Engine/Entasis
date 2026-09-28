package benchmark_support

import "core:fmt"
import "core:os"
import entasis "entasis:entasis"

Case_Status :: enum u8
{
	Ok,
	Allocation_Failed,
	Creation_Failed,
	Fixture_Failed,
	Step_Failed,
	Validation_Failed,
	Release_Failed,
	Output_Failed,
}

Step_Phase :: enum u8
{
	None,
	Warmup,
	Measured,
}

Benchmark_Result :: struct
{
	status: Case_Status,
	physics_status: entasis.Status,
	phase: Step_Phase,
	failed_step: int,
	release_status: entasis.Status,
}

Benchmark_Sample :: struct
{
	stats: entasis.World_Stats,
	invalid_transform_count: int,
	below_floor_count: int,
	out_of_bounds_count: int,
	elapsed_ms: f64,
}


RECORDING_HEADER :: ",recording_path,recording_component,recording_mode,timing_method";

CSV_HEADER :: "worker_count,step_index,active_constraints,active_pairs,body_count,shape_count,constraint_count,sleeping_body_count,invalid_transform_count,below_floor_count,out_of_bounds_count,case_status,metric_status,physics_elapsed_ms,benchmark_parameters";

// additional columns belong to an optional workload component. empty suffixes
// preserve the original shared CSV byte-for-byte for every other fixture
write_result :: proc(
	o: Options, parameters: string, sample: Benchmark_Sample,
	additional_header: string = "", additional_values: string = "",
) -> Case_Status
{
	// typed canonical text contains commas but no quotes/newlines. escape quotes
	// anyway at this CSV boundary. exact storage includes worst-case doubling
	if len(parameters) > PARAMETER_CAPACITY
	{
		return .Output_Failed;
	}
	recording_values: string = recording_columns(o.recording, "whole_loop" if o.workload == .Container else "native_step_sum");
	buffer, allocation_error := make([]byte, 2 * len(parameters) + 2048 + len(additional_values)+len(recording_values));
	if allocation_error != nil
	{
		return .Allocation_Failed;
	}
	defer delete(buffer);
	prefix := fmt.bprintf(buffer, "%d,%d,%d,%d,%d,%d,0,%d,%d,%d,%d,ok,ok,%.9f,\"",
	o.worker_count, o.steps, sample.stats.active_constraints, sample.stats.active_pairs,
	o.body_count + o.static_count, o.body_count + o.static_count,
	sample.stats.sleeping_bodies, sample.invalid_transform_count, sample.below_floor_count, sample.out_of_bounds_count, sample.elapsed_ms);
	count := len(prefix);
	for c in transmute([]byte)parameters
	{
		buffer[count] = c;
		count += 1;
		if c == '"'
		{
			buffer[count] = c;
			count += 1;
		}
	}
	buffer[count] = '"';
	count += 1;
	count += copy(buffer[count:], transmute([]byte)additional_values);
	count += copy(buffer[count:], transmute([]byte)recording_values);
	buffer[count] = '\n';
	count += 1;
	_, stat_error := os.stat(o.output, context.temp_allocator);
	if stat_error != nil && stat_error != os.Error(os.General_Error.Not_Exist)
	{
		return .Output_Failed;
	}
	file, open_error := os.open(o.output, {.Write, .Append, .Create}, os.Permissions_Default_File);
	if open_error != nil
	{
		return .Output_Failed;
	}
	if stat_error == os.Error(os.General_Error.Not_Exist)
	{
		for header in ([4]string{CSV_HEADER, additional_header, RECORDING_HEADER, "\n"})
		{
			written, error := os.write(file, transmute([]byte)header);
			if error != nil || written != len(header)
			{
				_ = os.close(file);
				return .Output_Failed;
			}
		}
	}
	written, error := os.write(file, buffer[:count]);
	close_error := os.close(file);
	if error != nil || written != count || close_error != nil
	{
		return .Output_Failed;
	}
	return .Ok;
}

recording_columns :: proc(recording: Recording_Options, off_timing: string) -> string
{
	if recording.mode == .Off
	{
		return fmt.tprintf(",,,off,%s", off_timing);
	}
	return fmt.tprintf(",%s,physics_elapsed_ms,on,native_step_sum", os.base(recording.output));
}
