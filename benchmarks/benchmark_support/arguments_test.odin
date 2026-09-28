package benchmark_support

import "core:testing"
import "core:os"
import "core:fmt"
import "core:mem"
import "core:strings"

admit_test :: proc(workload: Workload, extra: []string) -> (Options, string)
{
	arguments := make([]string, len(extra)+2);
	defer delete(arguments);
	arguments[0] = "--worker-count=1";
	arguments[1] = "--output=unused.csv";
	copy(arguments[2:], extra);
	return parse_options(workload, arguments);
}

@(test)
canonical_defaults_and_counts :: proc(t: ^testing.T)
{
	for workload in Workload
	{
		o, error := admit_test(workload, {"--shape=box"});
		testing.expect_value(t, error, "");
		if workload == .Pyramid
		{
			testing.expect_value(t, o.population_count, 16206);
			testing.expect_value(t, o.body_count, 16210);
			testing.expect_value(t, o.steps, 600);
			testing.expect_value(t, o.projectile_density * shape_volume(.Sphere, {4, 4, 4}), f32(335.10321044921875));
		}
		else
		{
			testing.expect_value(t, o.body_count, 10000);
			testing.expect_value(t, o.steps, 300);
		}
	}
}

@(test)
all_shape_dimension_rules :: proc(t: ^testing.T)
{
	for shape in ([]string{"box", "sphere", "capsule", "cylinder", "hull"})
	{
		option := fmt.aprintf("--shape=%s", shape);
		defer delete(option);
		o, error := admit_test(.Container, {option});
		testing.expect_value(t, error, "");
		testing.expect(t, shape_volume(o.shape, o.shape_size)>0);
	}
	for options in ([][]string{{"--shape=sphere", "--shape-size=1,2,1"}, {"--shape=cylinder", "--shape-size=1,2,2"}, {"--shape=capsule", "--shape-size=1,1,1"}, {"--shape=hull", "--static-shape=sphere"}})
	{
		_, error := admit_test(.Container, options);
		testing.expect(t, len(error)>0);
	}
}

@(test)
explicit_overrides_replace_scaled_defaults :: proc(t: ^testing.T)
{
	o, error := admit_test(.Container, {"--shape=box", "--layout-scale=2", "--shape-size=3,4,5", "--spawn-height=8"});
	testing.expect_value(t, error, "");
	testing.expect_value(t, o.shape_size, ([3]f32{3, 4, 5}));
	testing.expect_value(t, o.spawn_height, f32(8));
	testing.expect_value(t, o.container_size, ([3]f32{58, 49, 58}));
	testing.expect_value(t, o.spacing[0], f32(2.04));
 large, large_error := admit_test(.Container, {"--shape=box", "--spawn-height=1e7"});
 testing.expect_value(t, large_error, "");
 testing.expect_value(t, large.spawn_height, f32(1e7));
}

@(test)
duplicate_unknown_and_inapplicable_options_rejected :: proc(t: ^testing.T)
{
	for options in ([][]string{{"--shape=box", "--shape=hull"}, {"--shape=box", "--rows=4"}, {"--shape=box", "--variant=x"}, {"--shape=box", "--unknown=1"}, {"--shape"}})
	{
		_, error := admit_test(.Container, options);
		testing.expect(t, len(error)>0);
	}
	_, error := admit_test(.Pyramid, {"--shape=box", "--grid=1,1,1"});
	testing.expect(t, len(error)>0);
}

@(test)
nonfinite_range_and_count_overflow_rejected :: proc(t: ^testing.T)
{
	for options in ([][]string{{"--shape=box", "--density=NaN"}, {"--shape=box", "--density=1e999"}, {"--shape=box", "--steps=0"}, {"--shape=box", "--steps=999999999999999999999"}, {"--shape=box", "--grid=2147483647,2147483647,2"}, {"--shape=box", "--spacing=0,1,1"}, {"--shape=box", "--density=0x10"}, {"--shape=box", "--spacing=3e38,1,1"}, {"--shape=box", "--shape-size=1e-30,1e-30,1e-30"}})
	{
		_, error := admit_test(.Container, options);
		testing.expect(t, len(error)>0);
	}
	_, error := admit_test(.Contact_Islands, {"--shape=box", "--island-spacing=1,1"});
	testing.expect(t, len(error)>0);

	Static_Admission_Case :: struct
	{
		workload: Workload,
		options: []string,
		expected_option: string,
	}
	static_cases := []Static_Admission_Case{
		{.Contact_Islands, {"--shape=box", "--floor-size=1e-45,1,1"}, "--floor-size"},
		{.Contact_Islands, {"--shape=box", "--static-shape=hull", "--floor-size=1e-45,1,1"}, "--floor-size"},
		{.Container, {"--shape=box", "--container-size=29,1e-45,29"}, "--container-size"},
		{.Container, {"--shape=box", "--layout-scale=1e-45", "--shape-size=1,1,1", "--spacing=1,1,1", "--spawn-height=1", "--container-size=29,24.5,29"}, "--layout-scale"},
		{.Contact_Islands, {"--shape=box", "--floor-size=0.5,1,1"}, ""},
		{.Container, {"--shape=box", "--container-size=29,0.5,29"}, ""},
		{.Container, {"--shape=box", "--layout-scale=0.5", "--shape-size=1,1,1", "--spacing=1,1,1", "--spawn-height=1", "--container-size=29,24.5,29"}, ""},
	};
	for test_case in static_cases
	{
		_, static_error := admit_test(test_case.workload, test_case.options);
		if len(test_case.expected_option) == 0
		{
			testing.expect_value(t, static_error, "");
		}
		else
		{
			testing.expect(t, strings.contains(static_error, test_case.expected_option));
		}
	}
}

@(test)
pyramid_launch_and_sleep_admission :: proc(t: ^testing.T)
{
	_, error := admit_test(.Pyramid, {"--shape=sphere", "--steps=12"});
	testing.expect(t, len(error)>0);
	o, valid := admit_test(.Pyramid, {"--shape=sphere", "--steps=12", "--projectile-count=0", "--sleep=disabled"});
	testing.expect_value(t, valid, "");
	testing.expect_value(t, o.sleep, Sleep.Disabled);
	_, warmup_error := admit_test(.Pyramid, {"--shape=box", "--warmup-steps=1"});
	testing.expect(t, len(warmup_error)>0);
}

@(test)
canonical_parameters_normalize_order_and_preserve_effective_changes :: proc(t: ^testing.T)
{
	a, _ := admit_test(.Container, {"--shape=box", "--density=2.00"});
	b, _ := admit_test(.Container, {"--density=2", "--shape=box"});
	c, _ := admit_test(.Container, {"--shape=box", "--density=3"});
	ba, bb, bc: [PARAMETER_CAPACITY]byte;
	sa, ea := canonical_parameters(a, ba[:]);
	sb, eb := canonical_parameters(b, bb[:]);
	sc, ec := canonical_parameters(c, bc[:]);
	testing.expect_value(t, ea, Admission.Ok);
	testing.expect_value(t, eb, Admission.Ok);
	testing.expect_value(t, ec, Admission.Ok);
	testing.expect_value(t, sa, sb);
	testing.expect(t, sa!=sc);
	for workload in Workload
	{
		for mode in Recording_Mode
		{
			original, diagnostic := admit_test(workload, {"--shape=box", "--layout-scale=2"});
			original.recording.mode = mode;
			testing.expect_value(t, diagnostic, "");
			buffer: [PARAMETER_CAPACITY]u8;
			parameters, admission := canonical_parameters(original, buffer[:]);
			testing.expect_value(t, admission, Admission.Ok);
			separator: int = strings.index_byte(parameters, ';');
			reordered: string = fmt.aprintf("%s;%s", parameters[separator+1:], parameters[:separator]);
			duplicate: string = fmt.aprintf("%s;shape=box", parameters);
			unknown: string = fmt.aprintf("%s;unknown=1", parameters);
			malformed: string = fmt.aprintf("%s;broken", parameters);
			defer delete(reordered);
			defer delete(duplicate);
			defer delete(unknown);
			defer delete(malformed);
			trailing: string = fmt.aprintf("%s;", parameters);
			defer delete(trailing);
			for text, index in ([8]string{parameters, reordered, duplicate, unknown, malformed, parameters[separator+1:], "workload=container;shape=box", trailing})
			{
				restored: Options;
				restored_status: Admission;
				{
					context.allocator = mem.panic_allocator();
					context.temp_allocator = mem.panic_allocator();
					restored, restored_status = options_from_saved_parameters(workload, text, 2);
				}
				testing.expect_value(t, restored_status, Admission.Ok if index < 2 else Admission.Invalid);
				if restored_status == .Ok
				{
					testing.expect_value(t, restored.shape_size, original.shape_size);
					testing.expect_value(t, restored.spacing, original.spacing);
					testing.expect_value(t, restored.worker_count, 2);
					testing.expect_value(t, restored.output, "preview.csv");
					testing.expect_value(t, restored.recording.mode, mode if workload == .Container else Recording_Mode.Off);
					canonical_buffer: [PARAMETER_CAPACITY]u8;
					canonical, canonical_status := canonical_parameters(restored, canonical_buffer[:]);
					testing.expect_value(t, canonical_status, Admission.Ok);
					testing.expect_value(t, canonical, parameters);
				}
			}
			invalid_workers: Admission;
			{
				context.allocator = mem.panic_allocator();
				context.temp_allocator = mem.panic_allocator();
				_, invalid_workers = options_from_saved_parameters(workload, parameters, 0);
			}
			testing.expect_value(t, invalid_workers, Admission.Invalid);
		}
	}

}

// optional numeric components extend the existing CSV without changing the
// old header/row prefix or rewriting the header on the next sample
@(test)
result_csv_preserves_default_and_optional_components :: proc(t: ^testing.T)
{
	directory: string;
	error: os.Error;
	directory, error = os.make_directory_temp("", "entasis-benchmark-", context.allocator);
	if !testing.expect_value(t, error, nil)
	{
		return;
	}
	defer delete(directory);
	defer os.remove_all(directory);
	for extended in 0 ..< 2
	{
		path: string = fmt.aprintf("%s/results-%d.csv", directory, extended);
		defer delete(path);
		options: Options = {worker_count=2, steps=3, body_count=4, static_count=1, output=path};
		sample: Benchmark_Sample = {elapsed_ms=1.25};
		header: string = "";
		values: string = "";
		if extended == 1
		{
			header=",component_ms";
			values=",2.500000000";
		}
		for _ in 0 ..< 2
		{
			testing.expect_value(t, write_result(options, "shape=box", sample, header, values), Case_Status.Ok);
		}
		data: []byte;
		data, error = os.read_entire_file(path, context.allocator);
		if !testing.expect_value(t, error, nil)
		{
			return;
		}
		defer delete(data);
		row: string = fmt.aprintf("2,3,0,0,5,5,0,0,0,0,0,ok,ok,1.250000000,\"shape=box\"%s,,,off,whole_loop\n", values);
		defer delete(row);
		expected: string = fmt.aprintf("%s%s%s\n%s%s", CSV_HEADER, header, RECORDING_HEADER, row, row);
		defer delete(expected);
		testing.expect_value(t, string(data), expected);
	}
}

@(test)
recording_admission_preserves_off_and_requires_supported_final_output :: proc(t: ^testing.T)
{
	_, remaining, status := recording_arguments("container", {"--shape=box"}, .Off);
	testing.expect_value(t, status, Admission.Ok);
	testing.expect_value(t, len(remaining), 1);
	for args in ([][]string{{"--record=on", "--recording-output=frame.epr"}, {"--record=off", "--compression=lz4"}, {"--record=off", "--record=off"}})
	{
		_, _, rejected := recording_arguments("container", args, .Off);
		testing.expect_value(t, rejected, Admission.Invalid);
	}
	valid: []string = {"--record=on", "--recording-output=run path/sample.epr", "--compression=lz4hc", "--memory-mib=64"};
	for name in ([]string{"container", "contact_islands", "pyramid", "ragdoll_stair_tumble", "noncontact_constraint_mix", "noncontact_fallback_smoke"})
	{
		options, _, admitted := recording_arguments(name, valid, .On);
		testing.expect_value(t, admitted, Admission.Ok);
		testing.expect_value(t, options.compression, Recording_Compression.LZ4HC);
		testing.expect_value(t, options.budget, u64(64*1024*1024));
	}
	_, _, unsupported := recording_arguments("spatial_query_batch", valid, .On);
	testing.expect_value(t, unsupported, Admission.Invalid);
	for path in ([]string{"sample.epr.partial", "..", "bad;name.epr"})
	{
		_, _, invalid := recording_arguments("container", {"--record=on", fmt.tprintf("--recording-output=%s", path)}, .On);
		testing.expect_value(t, invalid, Admission.Invalid);
	}
}
