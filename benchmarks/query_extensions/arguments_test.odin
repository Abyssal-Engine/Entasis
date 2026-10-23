package query_extensions

import "core:testing"
import "core:fmt"
import "core:strings"
import e "entasis:entasis"
import support "../benchmark_support"

@(test)
query_case_arguments_are_bounded :: proc(t: ^testing.T)
{
	base: []string = {"query_extensions", "--worker-count=1", "--output=unused.csv"};
	control: Query_Arguments;
	status: support.Admission;
	control, status = query_arguments(base);
	testing.expect_value(t, status, support.Admission.Ok);
	testing.expect_value(t, control.query_case, "control");
	for worker in ([6]int{1, 8, 16, 24, 32, 7})
	{
		worker_buffer: [64]u8;
		selected: [6]string = {"query_extensions", fmt.bprintf(worker_buffer[:], "--worker-count=%d", worker), "--output=unused.csv", "--query-case=hierarchy", "--query-operation=ray-direct", "--query-cell=mesh-8192"};
		_, status = query_arguments(selected[:]);
		testing.expect_value(t, status, support.Admission.Ok if worker != 7 else .Invalid);
	}
	for operation in ([4]string{"ray-direct", "ray-any", "ray-closest", "ray-all"})
	{
		for cell in ([7]string{"mesh-128", "mesh-8192", "nested-mesh-128", "nested-mesh-8192", "box-1", "hull-32", "hull-256"})
		{
			testing.expect_value(t, hierarchy_cell_admit("hierarchy", operation, cell), support.Admission.Ok);
			operation_buffer, cell_buffer: [64]u8;
			selected: [6]string = {"query_extensions", "--worker-count=1", "--output=unused.csv", "--query-case=hierarchy", fmt.bprintf(operation_buffer[:], "--query-operation=%s", operation), fmt.bprintf(cell_buffer[:], "--query-cell=%s", cell)};
			_, status = query_arguments(selected[:]);
			testing.expect_value(t, status, support.Admission.Ok);
			testing.expect_value(t, hierarchy_cell_admit("global", operation, cell), support.Admission.Invalid);
		}
	}
	invalid: [11][]string = {
		{"query_extensions", "--worker-count=1", "--output=x", "--query-case=hierarchy"},
		{"query_extensions", "--worker-count=1", "--output=x", "--query-operation=ray-direct", "--query-cell=mesh-128"},
		{"query_extensions", "--worker-count=1", "--output=x", "--query-case=control", "--query-operation=ray-direct"},
		{"query_extensions", "--worker-count=1", "--output=x", "--query-case=hierarchy", "--query-operation=distance", "--query-cell=mesh-128"},
		{"query_extensions", "--worker-count=1", "--output=x", "--query-case=global", "--query-operation=ray-direct", "--query-cell=mesh-128"},
		{"query_extensions", "--worker-count=1", "--output=x", "--query-case=hierarchy", "--query-operation=ray-direct", "--query-cell=hull-unknown"},
		{"query_extensions", "--worker-count=1", "--output=x", "--query-case=hierarchy", "--query-operation=ray-direct", "--query-cell=mesh-128", "--query-case=hierarchy"},
		{"query_extensions", "--worker-count=1", "--output=x", "--query-case=hierarchy", "--query-operation=ray-direct", "--query-cell=mesh-128", "--query-group=mesh.ray_closest"},
		{"query_extensions", "--worker-count=1", "--output=x", "--query-case=hierarchy", "--query-operation=ray-direct", "--query-cell=mesh-128", "--query-repetitions=1"},
		{"query_extensions", "--worker-count=1", "--output=x", "--query-case=hierarchy", "--query-operation=ray-direct", "--query-cell="},
		{"query_extensions", "--worker-count=1", "--output=x", "--query-case=hierarchy", "--query-operation=ray-direct", "--query-cell=mesh-128", "--query-warmup-repetitions=1"},
	};
	for arguments in invalid
	{
		_, status = query_arguments(arguments);
		testing.expect_value(t, status, support.Admission.Invalid);
	}
	for cell in ([5]string{"compound-8", "big-128", "big-8192", "nested-compound-8", "nested-big-8192"})
	{
		for operation in ([4]string{"ray-direct", "ray-any", "ray-closest", "ray-all"})
		{
			expected: support.Admission = .Ok;
			if (cell == "compound-8" || cell == "big-128" || cell == "big-8192") && operation != "ray-direct"
			{
				expected = .Invalid;
			}
			testing.expect_value(t, hierarchy_cell_admit("hierarchy", operation, cell), expected);
			selected: [6]string = {"query_extensions", "--worker-count=1", "--output=unused.csv", "--query-case=hierarchy", "", ""};
			operation_buffer, cell_buffer: [64]u8;
			selected[4] = fmt.bprintf(operation_buffer[:], "--query-operation=%s", operation);
			selected[5] = fmt.bprintf(cell_buffer[:], "--query-cell=%s", cell);
			_, status = query_arguments(selected[:]);
			testing.expect_value(t, status, expected);
			testing.expect_value(t, hierarchy_cell_admit("global", operation, cell), support.Admission.Invalid);
		}
	}
	for cell in ([6]string{"mesh-8192-near", "mesh-8192-far", "mesh-8192-local", "box-near", "box-far", "box-local"})
	{
		for operation in ([4]string{"ray-any", "ray-closest", "ray-all", "ray-direct"})
		{
			expected: support.Admission = .Ok if e.PRECISION_NAME == "mixed64" && operation != "ray-direct" else .Invalid;
			operation_buffer, cell_buffer: [64]u8;
			selected: [6]string = {"query_extensions", "--worker-count=1", "--output=unused.csv", "--query-case=global", fmt.bprintf(operation_buffer[:], "--query-operation=%s", operation), fmt.bprintf(cell_buffer[:], "--query-cell=%s", cell)};
			_, status = query_arguments(selected[:]);
			testing.expect_value(t, status, expected);
			testing.expect_value(t, hierarchy_cell_admit("hierarchy", operation, cell), support.Admission.Invalid);
		}
	}

	for operation in ([6]string{"closest-point", "distance", "depenetration", "sweep-any", "sweep-closest", "sweep-all"})
	{
		for cell in ([8]string{"nested-query-64", "query-box-1", "compound4-nested64", "compound4-mesh128", "compound4-nested64-angular", "compound4-mesh128-angular", "mesh-128", "hull-32"})
		{
			for query_case in ([2]string{"hierarchy", "global"})
			{
				expected: support.Admission = .Invalid;
				if ((operation == "closest-point" || operation == "distance" || operation == "depenetration") &&
					(cell == "nested-query-64" || cell == "query-box-1") || query_case == "hierarchy" &&
					(operation == "sweep-any" || operation == "sweep-closest" || operation == "sweep-all") &&
					(cell == "query-box-1" || cell == "compound4-nested64" || cell == "compound4-mesh128" || cell == "compound4-nested64-angular" || cell == "compound4-mesh128-angular")) &&
					(query_case != "global" || e.PRECISION_NAME == "mixed64")
				{
					expected = .Ok;
				}
				case_buffer, operation_buffer, cell_buffer: [96]u8;
				selected: [6]string = {"query_extensions", "--worker-count=1", "--output=unused.csv", fmt.bprintf(case_buffer[:], "--query-case=%s", query_case), fmt.bprintf(operation_buffer[:], "--query-operation=%s", operation), fmt.bprintf(cell_buffer[:], "--query-cell=%s", cell)};
				_, status = query_arguments(selected[:]);
				testing.expect_value(t, status, expected);
			}
		}
	}

	for cell in ([16]string{"box-native-1", "box-native-8", "box-native-32", "box-native-256", "box-context-1", "box-context-8", "box-context-32", "box-context-256", "box-native-scalar", "box-context-scalar", "sphere-native-256", "sphere-context-256", "sphere-native-scalar", "sphere-context-scalar", "box-triangle-scalar", "box-hull-scalar"})
	{
		cell_buffer: [64]u8;
		selected: [6]string = {"query_extensions", "--worker-count=1", "--output=unused.csv", "--query-case=collision-batch", "--query-operation=collision", fmt.bprintf(cell_buffer[:], "--query-cell=%s", cell)};
		_, status = query_arguments(selected[:]);
		testing.expect_value(t, status, support.Admission.Ok);
		selected[4] = "--query-operation=ray-direct";
		_, status = query_arguments(selected[:]);
		testing.expect_value(t, status, support.Admission.Invalid);
		selected[4] = "--query-operation=collision";
		selected[3] = "--query-case=hierarchy";
		_, status = query_arguments(selected[:]);
		testing.expect_value(t, status, support.Admission.Invalid);
	}
	testing.expect_value(t, collision_batch_admit("collision-batch", "collision", "box-native-7"), support.Admission.Invalid);

}

@(test)
query_extension_focused_arguments_preserve_default_and_reject_invalid_values :: proc(t: ^testing.T)
{
	baseline, baseline_status := query_arguments({"query", "--worker-count=1", "--output=unused.csv"});
	testing.expect_value(t, baseline_status, support.Admission.Ok);
	testing.expect_value(t, baseline.query_group, "");
	testing.expect_value(t, baseline.repetitions, 1);
	testing.expect_value(t, baseline.warmup_repetitions, 1);
	for route in ROUTES
	{
		for operation in OPERATIONS
		{
			args, status := query_arguments({"query", "--worker-count=1", "--output=unused.csv", strings.concatenate({"--query-group=", route, ".", operation}, context.temp_allocator), "--query-repetitions=2", "--query-warmup-repetitions=1000000"});
			testing.expect_value(t, status, support.Admission.Ok);
			testing.expect_value(t, args.repetitions, 2);
			testing.expect_value(t, args.warmup_repetitions, 1_000_000);
		}
	}
	for value in ([]string{"0", "-1", "+1", "1.0", "1e2", "1000001", "184467440737095516160", "", " 2"})
	{
		_, status := query_arguments({"query", "--worker-count=1", "--output=unused.csv", "--query-group=convex.ray_all", strings.concatenate({"--query-repetitions=", value}, context.temp_allocator)});
		testing.expect_value(t, status, support.Admission.Invalid);
	}
	for options in ([][]string{
		{"--query-group=convex.ray_all", "--query-group=mesh.ray_all"},
		{"--query-repetitions=1"}, {"--query-warmup-repetitions=1"},
		{"--query-group=Convex.ray_all"}, {"--query-group=convex.*"},
		{"--query-group=mesh.shape_distance", "--query-repetitions=2", "--query-repetitions=2"},
		{"--query-group=mesh.shape_distance", "--query-warmup-repetitions=2", "--query-warmup-repetitions=2"},
		{"--unknown=1"}, {"--query-group="},
	})
	{
		args: [dynamic]string;
		append(&args, "query", "--worker-count=1", "--output=unused.csv");
		append(&args, ..options);
		_, status := query_arguments(args[:]);
		testing.expect_value(t, status, support.Admission.Invalid);
		delete(args);
	}
	for operation in ([]string{"closest_point", "shape_distance", "depenetration", "overlap_any"})
	{
		_, status := query_arguments({"query", "--worker-count=1", "--output=unused.csv", strings.concatenate({"--query-group=mesh.", operation}, context.temp_allocator)});
		testing.expect_value(t, status, support.Admission.Ok if support.BENCHMARK_COMPONENTS == "all" else support.Admission.Invalid);
	}
}
