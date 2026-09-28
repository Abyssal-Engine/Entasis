package physics_capture

import observation "../physics_observation"
import "core:fmt"
import "core:os"
import scenarios "../physics_scenarios"
import scene "../physics_scene"
import replay "../physics_replay"
import support "../../benchmarks/benchmark_support"

Capture_Options :: struct
{
	recipe: scenarios.Recipe,
	output: string,
	compression: replay.Compression,
	budget: u64,
}

capture_parse :: proc(arguments: []string) -> (Capture_Options, scene.Status)
{
	scenario_id, recipe_path, output: string;
	frames: int;
	compression: replay.Compression;
	budget: u64 = replay.DEFAULT_BUDGET;
	seen: bit_set[0..<6];
	for argument in arguments
	{
		key, value: string;
		found: support.Admission;
		key, value, found = support.split_once(argument, '=');
		if found != .Ok || len(value) == 0
		{
			return {}, .Invalid_Data;
		}
		index: int;
		switch key
		{
		case "--scenario": index=0;
		scenario_id=value;
		case "--recipe": index=1;
		recipe_path=value;
		case "--output": index=2;
		output=value;
		case "--frames":
			index=3;
			admission: support.Admission;
			frames, admission = support.parse_integer(value);
			if admission != .Ok || frames <= 0 || frames > 1_000_000
			{
				return {}, .Invalid_Data;
			}
		case "--compression":
			index=4;
			status: scene.Status;
			compression, status = replay.compression_parse(value);
			if status != .Ok
			{
				return {}, status;
			}
		case "--memory-mib":
			index=5;
			amount: int;
			admission: support.Admission;
			amount, admission = support.parse_integer(value);
			if admission != .Ok || amount < 64 || amount > 16384
			{
				return {}, .Invalid_Data;
			}
			budget = u64(amount)*1024*1024;
		case: return {}, .Invalid_Data;
		}
		if index in seen
		{
			return {}, .Invalid_Data;
		}
		seen += {index};
	}
	if len(output) == 0 || (len(scenario_id) == 0) == (len(recipe_path) == 0)
	{
		return {}, .Invalid_Data;
	}
	r: scenarios.Recipe;
	status: scene.Status;
	if len(recipe_path) > 0
	{
		bytes: []u8;
		error: os.Error;
		bytes, error = os.read_entire_file(recipe_path, context.allocator);
		if error != nil
		{
			return {}, .File_Error;
		}
		defer delete(bytes);
		r, status = scenarios.recipe_parse(bytes);
		if frames > 0
		{
			r.frames=frames;
		}
	}
	else
	{
		id: scenarios.Scenario;
		id, status = scenarios.resolve(scenario_id);
		if status != .Ok
		{
			return {}, status;
		}
		r, status = scenarios.recipe_admit(id, nil, frames);
	}
	if r.input == .Live
	{
		return {}, .Invalid_Data;
	}
	return {r, output, compression, budget}, status;
}

capture_budget_failure :: proc(required, available: u64) -> scene.Status
{
	fmt.eprintfln("VISUAL_BUDGET_EXCEEDED required_bytes=%d available_bytes=%d", required, available);
	return .Budget_Exceeded;
}

capture_scenario :: proc(options: Capture_Options) -> scene.Status
{
	if os.exists(options.output)
	{
		return .File_Error;
	}
	partial: string = fmt.aprintf("%s.partial", options.output);
	defer delete(partial);
	if os.exists(partial)
	{
		return .File_Error;
	}
	s: scenarios.Session;
	defer scenarios.session_destroy(&s);
	status: scene.Status = scenarios.session_create(&s, options.recipe, options.budget);
	if status != .Ok
	{
		if status == .Budget_Exceeded
		{
			return capture_budget_failure(s.required_bytes, options.budget);
		}
		return status;
	}
	packet_bytes: u64 = scenarios.session_bytes(&s);
	if packet_bytes > options.budget
	{
		return capture_budget_failure(packet_bytes, options.budget);
	}
	catalog: [scenarios.Scenario]scenarios.Descriptor = scenarios.CATALOG;
	entry: scenarios.Descriptor = catalog[options.recipe.scenario];
	settings_buffer: [support.PARAMETER_CAPACITY]u8;
	settings: string;
	settings_state: scene.Availability;
	settings, settings_state, status = scenarios.session_settings(&s, settings_buffer[:]);
	if status != .Ok
	{
		return status;
	}
	host: string = fmt.aprintf("%v", ODIN_OS);
	defer delete(host);
	w: replay.Writer;
	defer replay.writer_close(&w);
	status = replay.writer_open(&w, options.output, {scenario=entry.id, source=SOURCE, revision="Unknown", compiler=ODIN_VERSION,
		host=host, configuration=CONFIGURATION, components=#config(ENTASIS_BENCHMARK_COMPONENTS, "all"), settings=settings,
		axis=s.axis, timestep=s.timestep, settings_state=settings_state,
		events_state=.Available if .Events in entry.capabilities else .Unavailable}, options.compression, options.budget-packet_bytes);
	if status != .Ok
	{
		if status == .Budget_Exceeded
		{
			return capture_budget_failure(w.required_bytes, options.budget-packet_bytes);
		}
		return status;
	}
	count: int = s.extent+1;
	if options.recipe.frames > 0
	{
		count = min(count, options.recipe.frames);
	}
	observations: int;
	for index in 0 ..< count
	{
		if index > 0
		{
			occupied: u64 = replay.writer_bytes(&w)+scenarios.session_bytes(&s)-observation.observation_bytes(&s.observation);
			if occupied > options.budget
			{
				return capture_budget_failure(occupied, options.budget);
			}
			s.observation.budget = options.budget-occupied;
			status = scenarios.session_step(&s);
			if status != .Ok
			{
				if status == .Budget_Exceeded
				{
					return capture_budget_failure(s.observation.required_bytes, s.observation.budget);
				}
				return status;
			}
		}
		packet_bytes = scenarios.session_bytes(&s);
		if packet_bytes > options.budget
		{
			return capture_budget_failure(packet_bytes, options.budget);
		}
		w.budget = options.budget-packet_bytes;
		status = replay.writer_frame(&w, &s.observation.packet);
		if status != .Ok
		{
			if status == .Budget_Exceeded
			{
				return capture_budget_failure(w.required_bytes, w.budget);
			}
			return status;
		}
		observations += 1;
		if s.state == .Completed
		{
			break;
		}
	}
	completion: scene.Completion = .Completed if s.state == .Completed else .Prefix;
	status = replay.writer_finish(&w, completion);
	if status == .Ok
	{
		fmt.printfln("PHYSICS_CAPTURE_OK scenario=%s observations=%d tick=%d extent=%v compression=%v raw_payload_bytes=%d compressed_payload_bytes=%d file_bytes=%d peak_recording_bytes=%d encode_ns=%d output=%s",
			entry.id, observations, s.tick, completion, options.compression, w.raw_bytes, w.compressed_bytes, w.offset, w.peak_bytes, i64(w.encode_time), options.output);
	}
	return status;
}

main :: proc()
{
	options: Capture_Options;
	status: scene.Status;
	options, status = capture_parse(os.args[1:]);
	if status != .Ok
	{
		fmt.eprintfln("capture arguments: %v", status);
		os.exit(2);
	}
	status = capture_scenario(options);
	if status != .Ok
	{
		fmt.eprintfln("capture failed: %v", status);
		os.exit(1);
	}
}
