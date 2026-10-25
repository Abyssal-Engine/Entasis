package benchmark_report

import "core:fmt"
import "core:mem"
import "core:os"
import "core:strconv"
import "core:strings"
import "core:time"

Cli_Values :: struct
{
	query: Query_Measurement,
	ray_pattern: string,
	query_options: bit_set[Query_Cli_Option],
	precision, precision_candidate, precision_case, sampling_mode: string,
	precision_counts: [4]int,
	variant: string,
	variant_count: int,
	source_directory:   string,
	package_name:       string,
	configuration:      string,
	runs:               string,
	workers:            string,
	compiler_version:   string,
	operating_system:   string,
	processor:          string,
	logical_processors: string,
	command:            string,
	counts:             [10]int,
}

Query_Cli_Option :: enum
{
	Case, Operation, Cell, Group, Repetitions, Warmup_Repetitions, Family, Warmup_Batches, Measured_Batches, Components, Pattern, Execution,
}

cli_value :: proc(args: []string, index: ^int, option: string) -> (string, Report_Status)
{
	if index^ + 1 >= len(args) || strings.has_prefix(args[index^ + 1], "--")
	{
		fmt.eprintfln("%s requires a value", option)
		return "", .Invalid_Arguments
	}
	index^ += 1
	return args[index^], .Ok
}

parse_cli :: proc(args: []string) -> (Cli_Values, Report_Status)
{
	values: Cli_Values
	for index := 1; index < len(args); index += 1
	{
		value: string
		status: Report_Status
		switch args[index]
		{
		case "--query-case", "--query-operation", "--query-cell", "--query-group", "--query-repetitions", "--query-warmup-repetitions", "--query-family", "--warmup-batches", "--measured-batches", "--components", "--ray-pattern", "--query-execution":
			option: Query_Cli_Option;
			switch args[index]
			{
			case "--query-case": option = .Case;
			case "--query-operation": option = .Operation;
			case "--query-cell": option = .Cell;
			case "--query-group": option = .Group;
			case "--query-repetitions": option = .Repetitions;
			case "--query-warmup-repetitions": option = .Warmup_Repetitions;
			case "--query-family": option = .Family;
			case "--warmup-batches": option = .Warmup_Batches;
			case "--measured-batches": option = .Measured_Batches;
			case "--components": option = .Components;
			case "--ray-pattern": option = .Pattern;
			case "--query-execution": option = .Execution;
			}
			if option in values.query_options
			{
				return {}, .Invalid_Arguments;
			}
			values.query_options += {option};
			value, status = cli_value(args, &index, args[index]);
			if status != .Ok || value == ""
			{
				return {}, .Invalid_Arguments;
			}
			count: int;
			if option in (bit_set[Query_Cli_Option]{.Repetitions, .Warmup_Repetitions, .Warmup_Batches, .Measured_Batches})
			{
				count, status = query_positive_count(value);
			}
			switch option
			{
			case .Case: values.query.case_name = value;
			case .Operation: values.query.operation = value;
			case .Cell: values.query.cell = value;
			case .Group: values.query.group = value;
			case .Repetitions: values.query.repetitions = count;
			case .Warmup_Repetitions: values.query.warmup_repetitions = count;
			case .Family: values.query.family = value;
			case .Warmup_Batches: values.query.warmup_batches = count;
			case .Measured_Batches: values.query.measured_batches = count;
			case .Components: values.query.components = value;
			case .Pattern: values.ray_pattern = value;
			case .Execution: values.query.execution = value;
			}
		case "--precision", "--precision-candidate", "--precision-case", "--sampling-mode":
			field: int;
			switch args[index]
			{
			case "--precision": field = 0;
			case "--precision-candidate": field = 1;
			case "--precision-case": field = 2;
			case "--sampling-mode": field = 3;
			}
			values.precision_counts[field] += 1;
			value, status = cli_value(args, &index, args[index]);
			switch field
			{
			case 0: values.precision = value;
			case 1: values.precision_candidate = value;
			case 2: values.precision_case = value;
			case 3: values.sampling_mode = value;
			}
		case "--variant":
			values.variant_count += 1
			value, status = cli_value(args, &index, "--variant")
			values.variant = value
		case "--source":
			values.counts[0] += 1
			value, status = cli_value(args, &index, "--source")
			values.source_directory = value
		case "--package":
			values.counts[1] += 1
			value, status = cli_value(args, &index, "--package")
			values.package_name = value
		case "--configuration":
			values.counts[2] += 1
			value, status = cli_value(args, &index, "--configuration")
			values.configuration = value
		case "--runs":
			values.counts[3] += 1
			value, status = cli_value(args, &index, "--runs")
			values.runs = value
		case "--workers":
			values.counts[4] += 1
			value, status = cli_value(args, &index, "--workers")
			values.workers = value
		case "--compiler-version":
			values.counts[5] += 1
			value, status = cli_value(args, &index, "--compiler-version")
			values.compiler_version = value
		case "--operating-system":
			values.counts[6] += 1
			value, status = cli_value(args, &index, "--operating-system")
			values.operating_system = value
		case "--processor":
			values.counts[7] += 1
			value, status = cli_value(args, &index, "--processor")
			values.processor = value
		case "--logical-processors":
			values.counts[8] += 1
			value, status = cli_value(args, &index, "--logical-processors")
			values.logical_processors = value
		case "--command":
			values.counts[9] += 1
			value, status = cli_value(args, &index, "--command")
			values.command = value
		case:
			fmt.eprintfln("unknown option: %s", args[index])
			return {}, .Invalid_Arguments
		}
		if status != .Ok
		{
			return {}, status
		}
	}
	for count in values.counts
	{
		if count != 1
		{
			fmt.eprintln("every reporter option must be supplied exactly once")
			return {}, .Invalid_Arguments
		}
	}
	required_values := [10]string{
		values.source_directory,
		values.package_name,
		values.configuration,
		values.runs,
		values.workers,
		values.compiler_version,
		values.operating_system,
		values.processor,
		values.logical_processors,
		values.command,
	}
	for value in required_values
	{
		if value == ""
		{
			fmt.eprintln("reporter option values must be non-empty")
			return {}, .Invalid_Arguments
		}
	}
	if (workload_kind(values.package_name) == .Parameterized && values.package_name != "large_world_precision" && (values.variant_count != 1 || variant_status(values.variant) != .Ok)) ||
		(workload_kind(values.package_name) == .Legacy && values.variant_count != 0)
	{
		return {}, .Invalid_Arguments;
	}
	for count in values.precision_counts
	{
		if count > 1
		{
			return {}, .Invalid_Arguments;
		}
	}
	if values.precision_counts[0] > 0 && values.precision != "float32" && values.precision != "mixed64"
	{
		return {}, .Invalid_Arguments;
	}
	if values.package_name == "large_world_precision"
	{
		for count in values.precision_counts
		{
			if count != 1
			{
				return {}, .Invalid_Arguments;
			}
		}
		if values.variant_count != 0
		{
			return {}, .Invalid_Arguments;
		}
	}
	else if values.precision_counts[1]+values.precision_counts[2]+values.precision_counts[3] != 0
	{
		return {}, .Invalid_Arguments;
	}
	for option in values.query_options
	{
		if (option in (bit_set[Query_Cli_Option]{.Case, .Operation, .Cell, .Group, .Repetitions, .Warmup_Repetitions, .Components}) && values.package_name != "query_extensions") ||
			(option in (bit_set[Query_Cli_Option]{.Warmup_Batches, .Measured_Batches, .Pattern}) && values.package_name != "spatial_query_trace" && values.package_name != "spatial_query_batch") ||
			(option == .Family && values.package_name != "spatial_query_trace") ||
			(option == .Execution && values.package_name != "spatial_query_batch")
		{
			return {}, .Invalid_Arguments;
		}
	}
	if .Execution in values.query_options && values.query.execution != "caller" && values.query.execution != "mailbox"
	{
		return {}, .Invalid_Arguments;
	}
	if values.query.case_name != "" && values.query.case_name != "control"
	{
		if query_case_admit(values.query.case_name, values.query.operation, values.query.cell) != .Ok ||
			values.query.components != "all" || (values.query.case_name == "global" && values.precision == "float32") ||
			.Group in values.query_options ||
			.Repetitions in values.query_options || .Warmup_Repetitions in values.query_options
		{
			return {}, .Invalid_Arguments;
		}
	}
	else if .Operation in values.query_options || .Cell in values.query_options
	{
		return {}, .Invalid_Arguments;
	}
	if values.query.components != "" && values.query.components != "common" && values.query.components != "all"
	{
		return {}, .Invalid_Arguments;
	}
	if values.query.group != ""
	{
		components: string = values.query.components if values.query.components != "" else "all";
		column, status := query_group_column(values.query.group, components);
		delete(column);
		if status != .Ok
		{
			return {}, .Invalid_Arguments;
		}
		if .Repetitions not_in values.query_options
		{
			values.query.repetitions = 1;
		}
		if .Warmup_Repetitions not_in values.query_options
		{
			values.query.warmup_repetitions = 1;
		}
	}
	else if .Group in values.query_options || .Repetitions in values.query_options || .Warmup_Repetitions in values.query_options
	{
		return {}, .Invalid_Arguments;
	}
	if values.query.family != "" && values.query.family != "all" && values.query.family != "ray" && values.query.family != "sphere-cast" && values.query.family != "aabb-presence"
	{
		return {}, .Invalid_Arguments;
	}
	if values.ray_pattern != "" && values.ray_pattern != "axial" && values.ray_pattern != "oblique"
	{
		return {}, .Invalid_Arguments;
	}
	if values.ray_pattern == "oblique" && values.query.family != "" && values.query.family != "all" && values.query.family != "ray"
	{
		return {}, .Invalid_Arguments;
	}
	return values, .Ok;
}

parse_workers :: proc(value: string) -> ([dynamic]int, Report_Status)
{
	parts := strings.split(value, ",")
	workers := make([dynamic]int, 0, len(parts))
	for part in parts
	{
		worker_value, parsed := strconv.parse_i64(part)
		if !parsed || worker_value < 1 || worker_value > 2147483647
		{
			return workers, .Invalid_Arguments
		}
		worker := int(worker_value)
		if worker_index(workers[:], worker) >= 0
		{
			return workers, .Invalid_Arguments
		}
		append(&workers, worker)
	}
	return workers, .Ok
}

run :: proc() -> int
{
	arena: mem.Dynamic_Arena
	mem.dynamic_arena_init(&arena)
	defer mem.dynamic_arena_destroy(&arena)
	context.allocator = mem.dynamic_arena_allocator(&arena)

	values, cli_status := parse_cli(os.args)
	if cli_status != .Ok
	{
		return 2
	}
	run_count_value, runs_parsed := strconv.parse_i64(values.runs)
	logical_value, logical_parsed := strconv.parse_i64(values.logical_processors)
	workers, worker_status := parse_workers(values.workers)
	if !runs_parsed ||
	run_count_value < 1 ||
	run_count_value > 2147483647 ||
	!logical_parsed ||
	logical_value < 1 ||
	logical_value > 2147483647 ||
	worker_status != .Ok
	{
		fmt.eprintln("invalid numeric reporter input")
		return 2
	}
	if values.configuration != "development" && values.configuration != "release"
	{
		fmt.eprintln("configuration must be development or release")
		return 2
	}
	date_buffer: [time.MIN_YYYY_DATE_LEN]u8
	run_date := time.to_string_yyyy_mm_dd(time.now(), date_buffer[:])
	request := Report_Request{
		query=values.query, ray_pattern=values.ray_pattern,
		precision=values.precision, precision_candidate=values.precision_candidate, precision_case=values.precision_case, sampling_mode=values.sampling_mode,
		source_directory = values.source_directory,
		variant = values.variant,
		package_name = values.package_name,
		configuration = values.configuration,
		run_count = int(run_count_value),
		workers = workers[:],
		compiler_version = values.compiler_version,
		operating_system = values.operating_system,
		processor = values.processor,
		logical_processors = int(logical_value),
		command = values.command,
		run_date = run_date,
	}
	result := generate_report(request)
	if result.status != .Ok
	{
		fmt.eprintln(result.diagnostic)
		return 1
	}
	fmt.printfln("BENCHMARK_REPORT_OK path=%s/README.md", request.source_directory)
	return 0
}

main :: proc()
{
	os.exit(run())
}
