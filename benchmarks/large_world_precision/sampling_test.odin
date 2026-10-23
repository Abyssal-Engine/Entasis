package large_world_precision

import "core:fmt"
import "core:strings"
import "core:testing"
@(require) import support "../benchmark_support"
@(require) import entasis "entasis:entasis"

@(test)
sampling_arguments_require_known_unique_mode :: proc(t: ^testing.T)
{
	for name, index in ([4]string{"instrumented", "batch", "engine-only", "memory"})
	{
		mode, admission := sampling_mode_parse(name);
		testing.expect_value(t, admission, Precision_Admission.Accepted);
		testing.expect_value(t, mode, Sampling_Mode(index));
		arguments := [5]string{"benchmark", "--worker-count=1", "--output=unused.csv", "--precision-case=compact-near", fmt.tprintf("--sampling-mode=%s", name)};
		parsed, status := precision_arguments(arguments[:]);
		testing.expect_value(t, status, Precision_Admission.Accepted);
		testing.expect_value(t, parsed.sampling_mode, mode);
	}
	for name in ([4]string{"", "Batch", "unknown", "batch,memory"})
	{
		_, admission := sampling_mode_parse(name);
		testing.expect_value(t, admission, Precision_Admission.Rejected);
	}
	arguments := [6]string{"benchmark", "--worker-count=1", "--output=unused.csv", "--precision-case=compact-near", "--sampling-mode=batch", "--sampling-mode=memory"};
	_, status := precision_arguments(arguments[:]);
	testing.expect_value(t, status, Precision_Admission.Rejected);
	parsed, default_status := precision_arguments(arguments[:4]);
	testing.expect_value(t, default_status, Precision_Admission.Accepted);
	testing.expect_value(t, parsed.sampling_mode, Sampling_Mode.Instrumented);
}

@(test)
sampling_csv_does_not_invent_frame_or_maintenance_latencies :: proc(t: ^testing.T)
{
	for mode in ([4]Sampling_Mode{.Instrumented, .Batch, .Engine_Only, .Memory})
	{
		text: string = sampling_columns(mode, .Shift_Stress, 512, 123, 456, 32, 789, 10, 1.25, 2.5);
		fields: []string = strings.split(text, ",");
		defer delete(fields);
		testing.expect_value(t, len(fields), 12);
		testing.expect_value(t, fields[0], "");
		testing.expect_value(t, fields[1], precision_case_text(.Shift_Stress));
		testing.expect_value(t, fields[2], PRECISION_CANDIDATE);
		testing.expect_value(t, fields[3], sampling_mode_text(mode));
		testing.expect_value(t, fields[4], "512");
		if mode == .Batch || mode == .Memory
		{
			testing.expect_value(t, fields[5], "");
			testing.expect_value(t, fields[6], "");
		}
		else
		{
			testing.expect_value(t, fields[5], "123.000000000");
			testing.expect_value(t, fields[6], "456.000000000");
		}
		testing.expect_value(t, fields[7], "32");
		if mode == .Instrumented
		{
			testing.expect_value(t, fields[8], "789.000000000");
		}
		else
		{
			testing.expect_value(t, fields[8], "");
		}
		testing.expect_value(t, fields[9], "10");
		testing.expect_value(t, fields[10], "1.250000000");
		testing.expect_value(t, fields[11], "2.500000000");
	}
}

@(test)
precision_parameters_report_actual_distributed_solver_options :: proc(t: ^testing.T)
{
	options: support.Options = distributed_options(1, "unused.csv", .Distributed);
	buffer: [4096]byte;
	parameters: string;
	admission: Precision_Admission;
	parameters, admission = distributed_parameters(options, .Distributed, buffer[:]);
	testing.expect_value(t, admission, Precision_Admission.Accepted);
	testing.expect(t, strings.contains(parameters, "velocity-iterations=1;substeps=4"));
	options.velocity_iterations = 2;
	options.substeps = 3;
	parameters, admission = distributed_parameters(options, .Distributed, buffer[:]);
	testing.expect_value(t, admission, Precision_Admission.Accepted);
	testing.expect(t, strings.contains(parameters, "velocity-iterations=2;substeps=3"));
}

when PRECISION_CANDIDATE == "origin" || PRECISION_NATIVE_GLOBAL
{
	@(test)
	holdout_profiles_create_exact_frozen_populations :: proc(t: ^testing.T)
	{
		for precision_case in ([2]Precision_Case{.Holdout_Small, .Holdout_Mixed})
		{
			pool: entasis.Buffer_Pool;
			testing.expect_value(t, entasis.buffer_pool_init(&pool), entasis.Status.Ok);
			owner: Benchmark_Owner;
			options := precision_options(1, "unused.csv", precision_case);
			result := benchmark_owner_create(&owner, &pool, options, precision_case);
			testing.expect_value(t, result.status, support.Case_Status.Ok);
			testing.expect_value(t, len(owner.bodies), 1_000);
			testing.expect_value(t, owner.frame.origin.x, f64(1_000_000));
			testing.expect_value(t, owner.options.grid, [3]int{10, 10, 10});
			testing.expect_value(t, owner.registered_shapes, 8 if precision_case == .Holdout_Mixed else 4);
			testing.expect_value(t, benchmark_owner_destroy(&owner), entasis.Status.Ok);
			testing.expect_value(t, entasis.buffer_pool_destroy(&pool), entasis.Status.Ok);
		}
	}

}

@(test)
sampling_summary_uses_all_reserved_observations :: proc(t: ^testing.T)
{
	frames, sorted: [MEASURED_STEPS]i64;
	owner: Benchmark_Owner = {frame_nanoseconds=frames[:], sorted_frame_nanoseconds=sorted[:]};
	for count in ([3]int{2, QUERY_PASS_COUNT, MEASURED_STEPS})
	{
		for index in 0 ..< count
		{
			frames[index] = i64(count-index)*1000;
			sorted[index] = 0;
		}
		sampling_summarize(&owner, count);
		testing.expect_value(t, owner.frame_median_ms, f64(count+1)/2000);
		testing.expect_value(t, owner.frame_max_ms, f64(count)/1000);
		for duration, index in sorted[:count]
		{
			testing.expect_value(t, duration, i64(index+1)*1000);
		}
	}
}
