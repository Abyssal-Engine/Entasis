package benchmark_support

import "core:testing"
import trace "../spatial_query_trace"
import entasis "entasis:entasis"
import "core:encoding/csv"
import "core:strconv"
import observations "../scene_observation"
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
	for index in 0 ..< len(extra)
	{
		arguments[index+2] = extra[index];
	}
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
					testing.expect_value(t, restored.recording.mode, mode if workload == .Container && original.verify == .Off else Recording_Mode.Off);
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
		sample: Benchmark_Sample = {elapsed_ms=1.25, physical={mode=.On, criterion=.Container_Safety, status=.Unassessed, reason=.Incomplete_Coverage, shape_limit=1e-12}};
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
		physical_buffer: [4096]u8;
		physical: string = observations.physical_csv(sample.physical, physical_buffer[:]);
		row: string = fmt.aprintf("2,3,0,0,5,5,0,0,0,0,0,ok,ok,1.250000000,\"shape=box\",%s%s%s,,,off,native_step_sum\n", entasis.PRECISION_NAME, values, physical);
		defer delete(row);
		expected: string = fmt.aprintf("%s%s%s%s\n%s%s", CSV_HEADER, header, observations.PHYSICAL_HEADER, RECORDING_HEADER, row, row);
		defer delete(expected);
		testing.expect_value(t, string(data), expected);
		arena: mem.Dynamic_Arena;
		mem.dynamic_arena_init(&arena);
		{
			context.allocator = mem.dynamic_arena_allocator(&arena);
			records, csv_error := csv.read_all_from_string(string(data), context.allocator, context.allocator);
			testing.expect_value(t, csv_error, csv.Error(nil));
			if csv_error == nil
			{
				for name, column in records[0]
				{
					if name == "physical_shape_limit_m"
					{
						value, parsed := strconv.parse_f64(records[1][column]);
						testing.expect(t, parsed);
						testing.expect_value(t, value, sample.physical.shape_limit);
					}
				}
			}
		}
		mem.dynamic_arena_destroy(&arena);
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

@(test)
verification_admission_preserves_workload_defaults :: proc(t: ^testing.T)
{
	for workload in Workload
	{
		baseline: Options
		diagnostic: string
		baseline, diagnostic = admit_test(workload, {"--shape=box"})
		testing.expect_value(t, diagnostic, "")
		testing.expect_value(t, baseline.verify, observations.Verification_Mode.On)
		for selector in ([]string{"on", "off"})
		{
			argument: string = fmt.aprintf("--verify=%s", selector)
			options: Options
			options, diagnostic = admit_test(workload, {"--shape=box", argument})
			delete(argument)
			testing.expect_value(t, diagnostic, "")
			testing.expect_value(t, options.body_count, baseline.body_count)
			testing.expect_value(t, options.steps, baseline.steps)
			testing.expect_value(t, options.warmup_steps, baseline.warmup_steps)
			testing.expect_value(t, options.spacing, baseline.spacing)
			testing.expect_value(t, options.verify, observations.Verification_Mode.On if selector == "on" else observations.Verification_Mode.Off)
		}
		for extra in ([][]string{{"--shape=box", "--verify=yes"}, {"--shape=box", "--verify=on", "--verify=off"}, {"--shape=box", "--components=common"}})
		{
			_, diagnostic = admit_test(workload, extra)
			testing.expect(t, len(diagnostic) > 0)
		}
	}
	testing.expect(t, BENCHMARK_COMPONENTS == "common" || BENCHMARK_COMPONENTS == "all")
}

@(test)
spatial_query_ray_pattern_arguments_preserve_default_and_reject_invalid_values :: proc(t: ^testing.T)
{
	baseline, baseline_status := trace.parse_arguments({"query", "--worker-count=1", "--output=unused.csv"});
	explicit, explicit_status := trace.parse_arguments({"query", "--output=unused.csv", "--ray-pattern=axial", "--worker-count=1"});
	oblique, oblique_status := trace.parse_arguments({"query", "--worker-count=64", "--output=unused.csv", "--ray-pattern=oblique"});
	testing.expect_value(t, baseline_status, trace.Argument_Status.Ok);
	testing.expect_value(t, explicit_status, trace.Argument_Status.Ok);
	testing.expect_value(t, oblique_status, trace.Argument_Status.Ok);
	testing.expect_value(t, oblique.worker_count, 64);
	testing.expect_value(t, baseline.worker_count, explicit.worker_count);
	testing.expect_value(t, baseline.output_path, explicit.output_path);
	testing.expect_value(t, baseline.ray_pattern, trace.Ray_Pattern.Axial);
	testing.expect_value(t, explicit.ray_pattern, trace.Ray_Pattern.Axial);
	testing.expect_value(t, oblique.ray_pattern, trace.Ray_Pattern.Oblique);
	for arguments in ([][]string{
		{"query", "--worker-count=1", "--output=unused.csv", "--ray-pattern="},
		{"query", "--worker-count=1", "--output=unused.csv", "--ray-pattern=unknown"},
		{"query", "--worker-count=1", "--output=unused.csv", "--ray-pattern=axial", "--ray-pattern=oblique"},
		{"query", "--worker-count=1", "--ray-pattern=oblique"},
		{"query", "--output=unused.csv", "--ray-pattern=oblique"},
		{"query", "--worker-count=0", "--output=unused.csv"},
		{"query", "--worker-count=65", "--output=unused.csv"},
		{"query", "--worker-count=1", "--output="},
		{"query", "--worker-count=1", "--worker-count=2"},
		{"query", "--output=a.csv", "--output=b.csv"},
		{"query", "--worker-count=1", "--output=unused.csv", "--unknown=axial"},
	})
	{
		_, status := trace.parse_arguments(arguments);
		testing.expect_value(t, status, trace.Argument_Status.Invalid);
	}
	testing.expect_value(t, baseline.warmup_batches, 10);
	testing.expect_value(t, baseline.measured_batches, 100);
	for family in ([]string{"all", "ray", "sphere-cast", "aabb-presence"})
	{
		family_option: string = strings.concatenate({"--query-family=", family}, context.temp_allocator);
		args, status := trace.parse_arguments({"query", "--worker-count=1", "--output=unused.csv", family_option, "--warmup-batches=2", "--measured-batches=1000000"});
		testing.expect_value(t, status, trace.Argument_Status.Ok);
		testing.expect_value(t, args.query_family, family);
		testing.expect_value(t, args.warmup_batches, 2);
		testing.expect_value(t, args.measured_batches, 1_000_000);
		_, status = trace.parse_arguments({"query", "--worker-count=1", "--output=unused.csv", family_option}, .Reject);
		testing.expect_value(t, status, trace.Argument_Status.Invalid);
	}
	for option in ([]string{"--warmup-batches=", "--measured-batches=0", "--warmup-batches=-1", "--measured-batches=+1", "--measured-batches=1000001", "--measured-batches=999999999999999999999999", "--warmup-batches=1.0", "--query-family=Ray"})
	{
		_, status := trace.parse_arguments({"query", "--worker-count=1", "--output=unused.csv", option});
		testing.expect_value(t, status, trace.Argument_Status.Invalid);
	}
	for options in ([][]string{{"--query-family=ray", "--query-family=all"}, {"--warmup-batches=1", "--warmup-batches=2"}, {"--measured-batches=1", "--measured-batches=2"}, {"--query-family=sphere-cast", "--ray-pattern=oblique"}, {"--query-family=aabb-presence", "--ray-pattern=oblique"}})
	{
		_, status := trace.parse_arguments({"query", "--worker-count=1", "--output=unused.csv", options[0], options[1]});
		testing.expect_value(t, status, trace.Argument_Status.Invalid);
	}

}

@(test)
spatial_query_oblique_generation_preserves_constructed_hits_and_misses :: proc(t: ^testing.T)
{
	arguments, argument_status := trace.parse_arguments({"query", "--worker-count=1", "--output=unused.csv", "--ray-pattern=oblique"});
	testing.expect_value(t, argument_status, trace.Argument_Status.Ok);
	faces, octants: u32;
	invalid_geometry: int;
	for index in 0 ..< trace.RAY_QUERY_COUNT
	{
		origin, direction := trace.query_generate(index, arguments.ray_pattern);
		origins: [3]f32 = {origin.x, origin.y, origin.z};
		directions: [3]f32 = {direction.x, direction.y, direction.z};
		sample: int = index / 2;
		slot: int = sample % 10_000;
		center: [3]f32 = {2*f32(slot%25)-24, 2*f32(slot/625), 2*f32((slot/25)%25)-24};
		scene_minimum: [3]f32 = {-24.5, -0.5, -24.5};
		scene_maximum: [3]f32 = {24.5, 30.5, 24.5};
		entry: f32 = 0;
		exit: f32 = 64;
		octant: u32;
		for component in 0 ..< 3
		{
			if directions[component] < 0
			{
				octant |= 1 << u32(component);
			}
			expected_magnitude: f32 = 1 if component == (sample%6)/2 else 1.0/64.0;
			if abs(directions[component]) != expected_magnitude
			{
				invalid_geometry += 1;
			}
			minimum: f32 = center[component]-0.5 if index%2 == 0 else scene_minimum[component];
			maximum: f32 = center[component]+0.5 if index%2 == 0 else scene_maximum[component];
			a: f32 = (minimum-origins[component])/directions[component];
			b: f32 = (maximum-origins[component])/directions[component];
			entry = max(entry, min(a, b));
			exit = min(exit, max(a, b));
		}
		if (index%2 == 0 && entry > exit) || (index%2 != 0 && entry <= exit)
		{
			invalid_geometry += 1;
		}
		faces |= 1 << u32(sample%6);
		octants |= 1 << octant;
	}
	testing.expect_value(t, invalid_geometry, 0);
	testing.expect_value(t, faces, u32(63));
	testing.expect_value(t, octants, u32(255));
	for index in trace.RAY_QUERY_COUNT ..< trace.QUERY_COUNT_PER_BATCH
	{
		origin, direction := trace.query_generate(index, .Axial);
		oblique_origin, oblique_direction := trace.query_generate(index, arguments.ray_pattern);
		testing.expect_value(t, origin, oblique_origin);
		testing.expect_value(t, direction, oblique_direction);
	}
}
