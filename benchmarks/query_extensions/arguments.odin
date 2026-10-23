package query_extensions

import "core:strings"
import e "entasis:entasis"
import support "../benchmark_support"

Query_Arguments :: struct
{
	workers: int,
	output, query_group: string,
	query_case, query_operation, query_cell: string,
	route, operation: int,
	repetitions, warmup_repetitions: int,
}

Query_Argument :: enum
{
	Workers, Output, Group, Repetitions, Warmup, Case, Operation, Cell,
}

query_count_parse :: proc "contextless" (value: string, limit: int) -> (int, support.Admission)
{
	if len(value) == 0
	{
		return 0, .Invalid;
	}
	result: int;
	for character in value
	{
		if character < '0' || character > '9' || result > (limit-int(character-'0'))/10
		{
			return 0, .Invalid;
		}
		result = result*10+int(character-'0');
	}
	return result, .Ok if result > 0 && result <= limit else .Invalid;
}

query_arguments :: proc(arguments: []string) -> (Query_Arguments, support.Admission)
{
	if len(arguments) < 3
	{
		return {}, .Invalid;
	}
	result: Query_Arguments = {route=-1, operation=-1, repetitions=1, warmup_repetitions=1, query_case="control"};
	seen: bit_set[Query_Argument];
	for argument in arguments[1:]
	{
		separator: int = strings.index_byte(argument, '=');
		if separator < 0 || separator == len(argument)-1
		{
			return {}, .Invalid;
		}
		name: string = argument[:separator];
		value: string = argument[separator+1:];
		option: Query_Argument;
		status: support.Admission = .Ok;
		switch name
		{
		case "--worker-count":
			option = .Workers;
			result.workers, status = query_count_parse(value, e.MAXIMUM_WORKER_COUNT);
		case "--output":
			option = .Output;
			result.output = value;
		case "--query-case":
			option = .Case;
			result.query_case = value;
		case "--query-operation":
			option = .Operation;
			result.query_operation = value;
		case "--query-cell":
			option = .Cell;
			result.query_cell = value;
		case "--query-group":
			option = .Group;
			result.query_group = value;
			for route, route_index in ROUTES
			{
				if !strings.has_prefix(value, route) || len(value) <= len(route) || value[len(route)] != '.'
				{
					continue;
				}
				operation: string = value[len(route)+1:];
				for operation_name, index in OPERATIONS
				{
					if operation_name == operation
					{
						result.route, result.operation = route_index, index;
					}
				}
				when support.BENCHMARK_COMPONENTS == "all"
				{
					for operation_name, index in DISTANCE_OPERATIONS
					{
						if operation_name == operation
						{
							result.route, result.operation = route_index, len(OPERATIONS)+index;
						}
					}
				}
			}
			if result.route < 0
			{
				return {}, .Invalid;
			}
		case "--query-repetitions":
			option = .Repetitions;
			result.repetitions, status = query_count_parse(value, 1_000_000);
		case "--query-warmup-repetitions":
			option = .Warmup;
			result.warmup_repetitions, status = query_count_parse(value, 1_000_000);
		case:
			return {}, .Invalid;
		}
		if status != .Ok || option in seen
		{
			return {}, .Invalid;
		}
		seen += {option};
	}
	if .Workers not_in seen || .Output not_in seen ||
		(.Group not_in seen && (.Repetitions in seen || .Warmup in seen))
	{
		return {}, .Invalid;
	}
	if result.query_case == "control"
	{
		if .Operation in seen || .Cell in seen
		{
			return {}, .Invalid;
		}
	}
	else
	{
		if .Group in seen || .Repetitions in seen || .Warmup in seen ||
			.Operation not_in seen || .Cell not_in seen || support.BENCHMARK_COMPONENTS != "all" ||
			(hierarchy_cell_admit(result.query_case, result.query_operation, result.query_cell) != .Ok &&
				global_cell_admit(result.query_case, result.query_operation, result.query_cell) != .Ok &&
				collision_batch_admit(result.query_case, result.query_operation, result.query_cell) != .Ok) ||
			(result.workers != 1 && result.workers != 8 && result.workers != 16 && result.workers != 24 && result.workers != 32)
		{
			return {}, .Invalid;
		}
	}
	return result, .Ok;
}

// nonray fixtures have bounded geometry and private query contexts
hierarchy_geometry_admit :: proc "contextless" (operation, cell: string) -> support.Admission
{
	switch operation
	{
	case "closest-point", "distance", "depenetration":
		return .Ok if cell == "nested-query-64" || cell == "query-box-1" else .Invalid;
	case "sweep-any", "sweep-closest", "sweep-all":
		return .Ok if cell == "compound4-nested64" || cell == "compound4-mesh128" ||
			cell == "compound4-nested64-angular" || cell == "compound4-mesh128-angular" || cell == "query-box-1" else .Invalid;
	}
	return .Invalid;
}

// only implemented cells enter this table
hierarchy_cell_admit :: proc "contextless" (query_case, operation, cell: string) -> support.Admission
{
	if query_case == "hierarchy" && hierarchy_geometry_admit(operation, cell) == .Ok
	{
		return .Ok;
	}
	if query_case != "hierarchy" ||
		(operation != "ray-direct" && operation != "ray-any" && operation != "ray-closest" && operation != "ray-all")
	{
		return .Invalid;
	}
	switch cell
	{
	case "mesh-128", "mesh-8192", "nested-mesh-128", "nested-mesh-8192", "box-1", "hull-32", "hull-256":
		return .Ok;
	case "compound-8", "big-128", "big-8192":
		return .Ok if operation == "ray-direct" else .Invalid;
	case "nested-compound-8", "nested-big-8192":
		return .Ok;
	}
	return .Invalid;
}

// global cells use the mixed64 native context routes or their local controls
global_cell_admit :: proc "contextless" (query_case, operation, cell: string) -> support.Admission
{
	if query_case == "global" && e.PRECISION_NAME == "mixed64" &&
		(operation == "closest-point" || operation == "distance" || operation == "depenetration") &&
		hierarchy_geometry_admit(operation, cell) == .Ok
	{
		return .Ok;
	}
	if query_case != "global" || e.PRECISION_NAME != "mixed64" ||
		(operation != "ray-any" && operation != "ray-closest" && operation != "ray-all")
	{
		return .Invalid;
	}
	switch cell
	{
	case "mesh-8192-near", "mesh-8192-far", "mesh-8192-local", "box-near", "box-far", "box-local":
		return .Ok;
	}
	return .Invalid;
}

collision_batch_admit :: proc "contextless" (case_name, operation, cell: string) -> support.Admission
{
	if case_name != "collision-batch" || operation != "collision"
	{
		return .Invalid;
	}
	admission: support.Admission;
	_, admission = collision_batch_cell(cell);
	return admission;
}
