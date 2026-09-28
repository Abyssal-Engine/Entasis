package physics_scenarios

import "core:encoding/json"
import "core:fmt"
import "core:strings"
import support "../../benchmarks/benchmark_support"
import scene "../physics_scene"

Recipe :: struct
{
	scenario: Scenario,
	options: support.Options,
	input: scene.Input_Mode,
	frames: int,
}

recipe_admit :: proc(scenario: Scenario, settings: []string, frames: int = 0, input: scene.Input_Mode = .Scripted) -> (Recipe, scene.Status)
{
	catalog: [Scenario]Descriptor = CATALOG;
	if frames < 0 || frames > 1_000_000 || (input == .Live && .Input not_in catalog[scenario].capabilities)
	{
		return {}, .Invalid_Data;
	}
	r: Recipe = {scenario=scenario, frames=frames, input=input};
	if len(settings) != 0
	{
		return {}, .Invalid_Data;
	}
	return r, .Ok;
}

recipe_parse :: proc(data: []u8) -> (Recipe, scene.Status)
{
	value: json.Value;
	error: json.Error;
	value, error = json.parse(data, spec=.JSON, parse_integers=true);
	if error != .None
	{
		return {}, .Invalid_Data;
	}
	defer json.destroy_value(value);
	object, is_object := value.(json.Object);
	if !is_object
	{
		return {}, .Invalid_Data;
	}
	for key in object
	{
		if key != "scenario" && key != "settings" && key != "input" && key != "frames"
		{
			return {}, .Invalid_Data;
		}
	}
	id, is_string := object["scenario"].(json.String);
	if !is_string
	{
		return {}, .Invalid_Data;
	}
	scenario: Scenario;
	status: scene.Status;
	scenario, status = resolve(id);
	if status != .Ok
	{
		return {}, status;
	}
	frames: int;
	if f, present := object["frames"]; present
	{
		count, is_integer := f.(json.Integer);
		if !is_integer || count <= 0 || count > 1_000_000
		{
			return {}, .Invalid_Data;
		}
		frames = int(count);
	}
	input: scene.Input_Mode;
	if i, present := object["input"]; present
	{
		mode, is_mode := i.(json.String);
		if !is_mode || (mode != "scripted" && mode != "live")
		{
			return {}, .Invalid_Data;
		}
		if mode == "live"
		{
			input = .Live;
		}
	}
	settings: [dynamic]string;
	defer
	{
		for argument in settings
		{
			delete(argument);
		}
		delete(settings);
	}
	if s, present := object["settings"]; present
	{
		entries, is_settings := s.(json.Object);
		if !is_settings
		{
			return {}, .Invalid_Data;
		}
		for key, entry in entries
		{
			builder: strings.Builder;
			defer strings.builder_destroy(&builder);
			fmt.sbprintf(&builder, "--%s=", key);
			#partial switch v in entry
			{
			case json.String: strings.write_string(&builder, v);
			case json.Integer: fmt.sbprintf(&builder, "%d", v);
			case json.Float: fmt.sbprintf(&builder, "%.9g", v);
			case json.Array:
				for element, index in v
				{
					if index > 0
					{
						strings.write_byte(&builder, ',');
					}
					#partial switch n in element
					{
					case json.Integer: fmt.sbprintf(&builder, "%d", n);
					case json.Float: fmt.sbprintf(&builder, "%.9g", n);
					case: return {}, .Invalid_Data;
					}
				}
			case: return {}, .Invalid_Data;
			}
			append(&settings, strings.clone(strings.to_string(builder)));
		}
	}
	return recipe_admit(scenario, settings[:], frames, input);
}
