package benchmark_report

import "core:fmt"
import "core:mem"
import "core:os"
import "core:strconv"
import "core:strings"
import "core:time"

Cli_Values :: struct
{
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
 if (workload_kind(values.package_name) == .Parameterized && (values.variant_count != 1 || variant_status(values.variant) != .Ok)) ||
  (workload_kind(values.package_name) == .Legacy && values.variant_count != 0)
 {
  return {}, .Invalid_Arguments
 }
 return values, .Ok
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
