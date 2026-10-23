package large_world_precision

import "core:fmt"
import entasis "entasis:entasis"
import support "../benchmark_support"

DISTRIBUTED_WORLD_COUNT :: 16;
DISTRIBUTED_BODY_COUNT :: 625;
DISTRIBUTED_GRID_X :: 25;
DISTRIBUTED_GRID_Y :: 5;
DISTRIBUTED_GRID_Z :: 5;
DISTRIBUTED_SEPARATION :: f64(1_000_000);
DISTRIBUTED_OFFSET :: f64(1_000_000_000);
DISTRIBUTED_SPACING :: f64(1.002);

distributed_options :: proc(
	worker_count: int,
	output: string,
	precision_case: Precision_Case,
) -> support.Options
{
	steps := MEASURED_STEPS;
	warmup := WARMUP_STEPS;
	if precision_case == .Queries_Spanning
	{
		steps = QUERY_PASS_COUNT;
		warmup = QUERY_WARMUP_COUNT;
	}
	return {
		workload=.Container,
		shape=.Box,
		static_shape=.Box,
		shape_size={1, 1, 1},
		density=1,
		layout_scale=1,
		steps=steps,
		timestep_hz=60,
		warmup_steps=warmup,
		velocity_iterations=1,
		substeps=4,
		sleep=.Disabled,
		grid={DISTRIBUTED_GRID_X, DISTRIBUTED_GRID_Y, DISTRIBUTED_GRID_Z},
		spacing={f32(DISTRIBUTED_SPACING), f32(DISTRIBUTED_SPACING), f32(DISTRIBUTED_SPACING)},
		spawn_height=0.501,
		container_size={48, 16, 24},
		worker_count=worker_count,
		output=output,
		population_count=DISTRIBUTED_WORLD_COUNT*(DISTRIBUTED_BODY_COUNT+1),
		body_count=DISTRIBUTED_WORLD_COUNT*DISTRIBUTED_BODY_COUNT,
		static_count=DISTRIBUTED_WORLD_COUNT,
	};
}

distributed_parameters :: proc(
	options: support.Options,
	precision_case: Precision_Case,
	buffer: []byte,
	mode: Sampling_Mode = .Instrumented,
) -> (string, Precision_Admission)
{
	parameters: string;
	dispatcher: string = "shared-sequential-worlds";
	when PRECISION_NATIVE_GLOBAL
	{
		dispatcher = "single-world";
	}
	if precision_case == .Queries_Spanning
	{
		parameters = fmt.bprintf(
			buffer,
			"workload=large_world_precision;fixture=%s;precision-candidate=%s;precision-case=%s;regions=%d;region-separation=%.17g;offset=%.17g;bodies-per-region=%d;bodies=%d;statics=%d;grid=25,5,5;spacing=1.002;queries-per-family=%d;families=2;passes=%d;warmup-queries-per-family=%d;expected-hits-per-hit-ray=%d;timing=serial-query-passes-with-global-clipping;sampling=%s",
			precision_fixture_text(precision_case), PRECISION_CANDIDATE, precision_case_text(precision_case),
			DISTRIBUTED_WORLD_COUNT, DISTRIBUTED_SEPARATION, DISTRIBUTED_OFFSET,
			DISTRIBUTED_BODY_COUNT, options.body_count, options.static_count,
			QUERY_COUNT, QUERY_PASS_COUNT, QUERY_WARMUP_COUNT, SPANNING_HIT_CAPACITY, sampling_mode_text(mode),
		);
	}
	else
	{
		parameters = fmt.bprintf(
			buffer,
			"workload=large_world_precision;fixture=%s;precision-candidate=%s;precision-case=%s;regions=%d;region-separation=%.17g;offset=%.17g;bodies-per-region=%d;bodies=%d;statics=%d;steps=%d;timestep-hz=60;warmup-steps=%d;velocity-iterations=%d;substeps=%d;grid=25,5,5;spacing=1.002;spawn-height=0.501;floor-size=48,1,24;dispatcher=%s;worker-budget=total;timing=%s;sampling=%s",
			precision_fixture_text(precision_case), PRECISION_CANDIDATE, precision_case_text(precision_case),
			DISTRIBUTED_WORLD_COUNT, DISTRIBUTED_SEPARATION, DISTRIBUTED_OFFSET,
			DISTRIBUTED_BODY_COUNT, options.body_count, options.static_count,
			MEASURED_STEPS, WARMUP_STEPS, options.velocity_iterations, options.substeps, dispatcher,
			"all-world-steps-sum-excluding-export" if mode == .Engine_Only else "whole-loop-with-global-export",
			sampling_mode_text(mode),
		);
	}
	if len(parameters) > 0 && len(parameters) < len(buffer)
	{
		return parameters, .Accepted;
	}
	return parameters, .Rejected;
}

distributed_fixture_position :: proc "contextless" (
	frame: entasis.Origin_Frame,
	body_index: int,
) -> entasis.Global_Position
{
	x := body_index % DISTRIBUTED_GRID_X;
	z := (body_index / DISTRIBUTED_GRID_X) % DISTRIBUTED_GRID_Z;
	y := body_index / (DISTRIBUTED_GRID_X*DISTRIBUTED_GRID_Z);
	return entasis.global_position(
		frame.origin.x + f64(x-12)*DISTRIBUTED_SPACING,
		frame.origin.y + 0.501 + f64(y)*DISTRIBUTED_SPACING,
		frame.origin.z + f64(z-2)*DISTRIBUTED_SPACING,
	);
}

