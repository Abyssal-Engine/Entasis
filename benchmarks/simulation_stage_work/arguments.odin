package simulation_stage_work

import "core:strings"
import support "../benchmark_support"

Stage :: enum u8
{
	Tree_Update,
	Pair_Traversal,
	Narrow_Phase,
	Collision,
}
Fixture :: enum u8
{
	Container,
	Islands,
}
Motion :: enum u8
{
	Steady,
	Churn,
}
Options :: struct
{
	stage: Stage,
	fixture: Fixture,
	motion: Motion,
	workers: int,
	output: string,
}

parse_options :: proc(arguments: []string) -> (Options, support.Admission)
{
	o: Options;
	seen: u8;
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
		bit: u8;
		switch key
		{
		case "stage":
			bit = 1;
			switch value
			{
			case "tree-update": o.stage = .Tree_Update;
			case "pair-traversal": o.stage = .Pair_Traversal;
			case "narrow-phase": o.stage = .Narrow_Phase;
			case "collision": o.stage = .Collision;
			case: return {}, .Invalid;
			}
		case "fixture":
			bit = 2;
			switch value
			{
			case "container": o.fixture = .Container;
			case "islands": o.fixture = .Islands;
			case: return {}, .Invalid;
			}
		case "motion":
			bit = 4;
			switch value
			{
			case "steady": o.motion = .Steady;
			case "churn": o.motion = .Churn;
			case: return {}, .Invalid;
			}
		case "worker-count":
			bit = 8;
			o.workers, status = support.parse_integer(value);
			if status != .Ok || o.workers < 1 || o.workers > 64
			{
				return {}, .Invalid;
			}
		case "output":
			bit = 16;
			o.output = value;
		case:
			return {}, .Invalid;
		}
		if seen & bit != 0
		{
			return {}, .Invalid;
		}
		seen |= bit;
	}
	if seen != 31
	{
		return {}, .Invalid;
	}
	return o, .Ok;
}

stage_text :: proc "contextless" (stage: Stage) -> string
{
	switch stage
	{
	case .Tree_Update: return "tree-update";
	case .Pair_Traversal: return "pair-traversal";
	case .Narrow_Phase: return "narrow-phase";
	case .Collision: return "collision";
	}
	unreachable();
}
fixture_text :: proc "contextless" (fixture: Fixture) -> string
{
	return "container" if fixture == .Container else "islands";
}
motion_text :: proc "contextless" (motion: Motion) -> string
{
	return "steady" if motion == .Steady else "churn";
}
