package large_world_precision

import "core:fmt"
import "base:intrinsics"

Sampling_Mode :: enum u8
{
	Instrumented,
	Batch,
	Engine_Only,
	Memory,
}

sampling_mode_parse :: proc(value: string) -> (Sampling_Mode, Precision_Admission)
{
	switch value
	{
		case "instrumented": return .Instrumented, .Accepted;
		case "batch": return .Batch, .Accepted;
		case "engine-only": return .Engine_Only, .Accepted;
		case "memory": return .Memory, .Accepted;
	}
	return {}, .Rejected;
}

sampling_mode_text :: proc(mode: Sampling_Mode) -> string
{
	switch mode
	{
		case .Instrumented: return "instrumented";
		case .Batch: return "batch";
		case .Engine_Only: return "engine-only";
		case .Memory: return "memory";
	}
	unreachable();
}

// process totals never stand in for frame latency. empty fields explicitly
// distinguish unavailable latency/maintenance duration from a measured zero
sampling_columns :: proc(
	mode: Sampling_Mode, precision_case: Precision_Case, count: int,
	median_ms, maximum_ms: f64, maintenance_count: int,
	maintenance_ms, checksum: f64,
	setup_ms, destruction_ms: f64,
) -> string
{
	if mode == .Batch || mode == .Memory
	{
		return fmt.tprintf(",%s,%s,%s,%d,,,%d,,%.17g,%.9f,%.9f",
			precision_case_text(precision_case), PRECISION_CANDIDATE,
			sampling_mode_text(mode), count, maintenance_count, checksum, setup_ms, destruction_ms);
	}
	if mode == .Engine_Only || maintenance_count == 0
	{
		return fmt.tprintf(",%s,%s,%s,%d,%.9f,%.9f,%d,,%.17g,%.9f,%.9f",
			precision_case_text(precision_case), PRECISION_CANDIDATE,
			sampling_mode_text(mode), count, median_ms, maximum_ms,
			maintenance_count, checksum, setup_ms, destruction_ms);
	}
	return fmt.tprintf(",%s,%s,%s,%d,%.9f,%.9f,%d,%.9f,%.17g,%.9f,%.9f",
		precision_case_text(precision_case), PRECISION_CANDIDATE,
		sampling_mode_text(mode), count, median_ms, maximum_ms,
		maintenance_count, maintenance_ms, checksum, setup_ms, destruction_ms);
}

sampling_timing_method :: proc(mode: Sampling_Mode) -> string
{
	switch mode
	{
		case .Instrumented, .Batch: return "whole_loop";
		case .Engine_Only: return "native_step_sum";
		case .Memory: return "allocator_instrumented";
	}
	unreachable();
}

// both arrays are reserved by benchmark_owner_create and occupy distinct storage
sampling_summarize :: proc(owner: ^Benchmark_Owner, count: int)
{
	intrinsics.mem_copy_non_overlapping(&owner.sorted_frame_nanoseconds[0], &owner.frame_nanoseconds[0], count * size_of(i64));
	sort_frame_durations(owner.sorted_frame_nanoseconds[:count]);
	middle: int = count/2;
	owner.frame_median_ms = f64(owner.sorted_frame_nanoseconds[middle-1]+owner.sorted_frame_nanoseconds[middle])/2e6;
	owner.frame_max_ms = f64(owner.sorted_frame_nanoseconds[count-1])/1e6;
}
