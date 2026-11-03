package solver_contact_capture

import "core:strings"
import support "../benchmark_support"

Options :: struct
{
	origin, output_root: string,
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
		if status != .Ok || value == ""
		{
			return {}, .Invalid;
		}
		bit: u32;
		switch key
		{
		case "origin":
			bit = 1;
			if value != "accepted" && value != "r02-pairs"
			{
				return {}, .Invalid;
			}
			o.origin = value;
		case "output-root":
			bit = 2;
			o.output_root = value;
		case:
			return {}, .Invalid;
		}
		if seen & bit != 0
		{
			return {}, .Invalid;
		}
		seen |= bit;
	}
	if seen != 3
	{
		return {}, .Invalid;
	}
	return o, .Ok;
}
