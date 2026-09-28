// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import "core:time"

Simulation_Stage :: enum u8
{
	Timestep,
	Sleep,
	Slept_Callback,
	Predict_Bounding_Boxes,
	Before_Collision_Callback,
	Broad_Phase,
	Collision_Detection,
	Collisions_Detected_Callback,
	Solve,
	Constraints_Solved_Callback,
	Incrementally_Optimize,
	Cleanup,
}

Simulation_Profiling_State :: enum u8
{
	Disabled,
	Enabled,
}

Simulation_Profiler :: struct
{
	trace:                        [64]Simulation_Stage,
	stage_counts:                 [len(Simulation_Stage)]u64,
	stage_durations_nanoseconds:  [len(Simulation_Stage)]i64,
	stage_start_timestamps:       [len(Simulation_Stage)]time.Tick,
	stage_activity:               [len(Simulation_Stage)]Reference_State,
	trace_count:                  int,
	state:                        Simulation_Profiling_State,
}

simulation_profiler_begin_step :: proc "contextless" (profiler: ^Simulation_Profiler)
{
	if profiler == nil
	{
		return;
	}
	profiler.trace_count = 0;
	if profiler.state != .Enabled
	{
		return;
	}
	for stage in Simulation_Stage
	{
		profiler.stage_durations_nanoseconds[stage] = 0;
		profiler.stage_activity[stage] = .Missing;
	}
}

simulation_profiler_start :: proc "contextless" (
	profiler: ^Simulation_Profiler, stage: Simulation_Stage,
)
{
	if profiler == nil || profiler.state != .Enabled
	{
		return;
	}
	profiler.stage_start_timestamps[stage] = time.tick_now();
	profiler.stage_activity[stage] = .Present;
}

simulation_profiler_end :: proc "contextless" (
	profiler: ^Simulation_Profiler, stage: Simulation_Stage,
)
{
	if profiler == nil || profiler.state != .Enabled
	{
		return;
	}
	end := time.tick_now();
	if profiler.stage_activity[stage] == .Present
	{
		duration := time.tick_diff(profiler.stage_start_timestamps[stage], end);
		profiler.stage_durations_nanoseconds[stage] += time.duration_nanoseconds(duration);
		profiler.stage_activity[stage] = .Missing;
	}
	if profiler.trace_count < len(profiler.trace)
	{
		profiler.trace[profiler.trace_count] = stage;
		profiler.trace_count += 1;
	}
	profiler.stage_counts[stage] += 1;
}
