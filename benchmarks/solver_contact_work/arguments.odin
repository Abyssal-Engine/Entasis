package solver_contact_work

import "core:strconv"
import "core:strings"
import support "../benchmark_support"

Options :: struct
{
	input, output: string,
	workers: int,
	affinity: u64,
}

parse_options :: proc(arguments: []string) -> (Options, support.Admission)
{
	o: Options;
	seen: u32;
	for argument in arguments
	{
		if !strings.has_prefix(argument, "--")
		{
			return {}, .Invalid;
		}
		key, value: string;
		status: support.Admission;
		key, value, status = support.split_once(argument[2:], '=');
		if status != .Ok || value == "" || strings.contains_any(value, "\"\r\n")
		{
			return {}, .Invalid;
		}
		bit: u32;
		switch key
		{
		case "input":
			bit = 1;
			o.input = value;
		case "output":
			bit = 2;
			o.output = value;
		case "worker-count":
			bit = 4;
			o.workers, status = support.parse_integer(value);
			if status != .Ok || o.workers < 1 || o.workers > 64
			{
				return {}, .Invalid;
			}
		case "processor-affinity":
			bit = 8;
			parsed: u64;
			valid: bool;
			parsed, valid = strconv.parse_u64(value, 10);
			if !valid || parsed == 0
			{
				return {}, .Invalid;
			}
			o.affinity = parsed;
		case:
			return {}, .Invalid;
		}
		if seen & bit != 0
		{
			return {}, .Invalid;
		}
		seen |= bit;
	}
	if seen != 15 || o.input == o.output
	{
		return {}, .Invalid;
	}
	return o, .Ok;
}
