package physics_scenarios

import "core:encoding/json"
import support "../../benchmarks/benchmark_support"
import scene "../physics_scene"

Setting_Value :: union
{
	int, f32, [2]int, [3]int, [2]f32, [3]f32, support.Shape, support.Sleep,
}
Setting :: struct
{
	name: string,
	value: Setting_Value,
}
SETTING_CAPACITY :: len(support.OPTION_NAMES)-1;

recipe_settings :: proc(recipe: Recipe, buffer: ^[SETTING_CAPACITY]Setting) -> []Setting
{
	return buffer[:0];
}

recipe_encode :: proc(recipe: Recipe) -> ([]u8, scene.Status)
{
	catalog: [Scenario]Descriptor = CATALOG;
	settings: json.Object = make(json.Object);
	defer
	{
		for _, value in settings
		{
			if array, found := value.(json.Array); found
			{
				delete(array);
			}
		}
		delete(settings);
	}
	buffer: [SETTING_CAPACITY]Setting;
	for field in recipe_settings(recipe, &buffer)
	{
		switch value in field.value
		{
		case int: settings[field.name] = json.Integer(value);
		case f32: settings[field.name] = json.Float(value);
		case support.Shape:
			names: [support.Shape]string = {.Box="box", .Sphere="sphere", .Capsule="capsule", .Cylinder="cylinder", .Hull="hull"};
			settings[field.name] = json.String(names[value]);
		case support.Sleep: settings[field.name] = json.String("enabled" if value == .Enabled else "disabled");
		case [2]int:
			array: json.Array;
			for component in value
			{
				append(&array, json.Value(json.Integer(component)));
			}
			settings[field.name] = array;
		case [3]int:
			array: json.Array;
			for component in value
			{
				append(&array, json.Value(json.Integer(component)));
			}
			settings[field.name] = array;
		case [2]f32:
			array: json.Array;
			for component in value
			{
				append(&array, json.Value(json.Float(component)));
			}
			settings[field.name] = array;
		case [3]f32:
			array: json.Array;
			for component in value
			{
				append(&array, json.Value(json.Float(component)));
			}
			settings[field.name] = array;
		}
	}
	object: json.Object = make(json.Object);
	defer delete(object);
	object["scenario"] = json.String(catalog[recipe.scenario].id);
	object["settings"] = settings;
	object["input"] = json.String("live" if recipe.input == .Live else "scripted");
	if recipe.frames > 0
	{
		object["frames"] = json.Integer(recipe.frames);
	}
	bytes: []u8;
	error: json.Marshal_Error;
	bytes, error = json.marshal(object, {pretty=true, sort_maps_by_key=true});
	if error != nil
	{
		delete(bytes);
		return nil, .Invalid_Data;
	}
	return bytes, .Ok;
}
