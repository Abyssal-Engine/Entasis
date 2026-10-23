package benchmark_support

import "core:fmt"
import "core:os"
import "base:intrinsics"
import entasis "entasis:entasis"
import observations "../scene_observation"

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
	Physical_Failed,
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
	physical: observations.Qualification_Result,
}


RECORDING_HEADER :: ",recording_path,recording_component,recording_mode,timing_method";

CSV_HEADER :: "worker_count,step_index,active_constraints,active_pairs,body_count,shape_count,constraint_count,sleeping_body_count,invalid_transform_count,below_floor_count,out_of_bounds_count,case_status,metric_status,physics_elapsed_ms,benchmark_parameters,precision";

Physical_Output :: enum u8
{
	Present,
	Absent,
}

// additional columns belong to an optional workload component. empty suffixes
// add no component columns when both suffixes are empty
write_result :: proc(
	o: Options, parameters: string, sample: Benchmark_Sample,
	additional_header: string = "", additional_values: string = "",
	timing_method: string = "",
	physical_output: Physical_Output = .Present,
) -> Case_Status
{
	// typed canonical text contains commas but no quotes/newlines. escape quotes
	// anyway at this CSV boundary. exact storage includes worst-case doubling
	if len(parameters) > PARAMETER_CAPACITY
	{
		return .Output_Failed;
	}
	effective_timing: string = timing_method;
	if len(effective_timing) == 0
	{
		effective_timing = "whole_loop" if o.workload == .Container && o.verify == .Off else "native_step_sum";
	}
	recording_values: string = recording_columns(o.recording, effective_timing);
	physical_buffer: [4096]u8
	physical_values, physical_header: string;
	if physical_output == .Present
	{
		physical_values = observations.physical_csv(sample.physical, physical_buffer[:]);
		physical_header = observations.PHYSICAL_HEADER;
	}
	buffer, allocation_error := make([]byte, 2 * len(parameters) + 2048 + len(additional_values)+len(recording_values)+len(physical_values));
	if allocation_error != nil
	{
		return .Allocation_Failed;
	}
	defer delete(buffer);
	elapsed_text: string = "";
	if timing_method != "allocator_instrumented"
	{
		elapsed_text = fmt.tprintf("%.9f", sample.elapsed_ms);
	}
	prefix := fmt.bprintf(buffer, "%d,%d,%d,%d,%d,%d,0,%d,%d,%d,%d,ok,ok,%s,\"",
	o.worker_count, o.steps, sample.stats.active_constraints, sample.stats.active_pairs,
	o.body_count + o.static_count, o.body_count + o.static_count,
	sample.stats.sleeping_bodies, sample.invalid_transform_count, sample.below_floor_count, sample.out_of_bounds_count, elapsed_text);
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
	buffer[count] = ',';
	count += 1;
	precision_name: string = entasis.PRECISION_NAME;
	for text in ([4]string{precision_name, additional_values, physical_values, recording_values})
	{
		intrinsics.mem_copy_non_overlapping(raw_data(buffer[count:]), raw_data(text), len(text));
		count += len(text);
	}
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
		for header in ([5]string{CSV_HEADER, additional_header, physical_header, RECORDING_HEADER, "\n"})
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
