package benchmark_support

import "core:strings"
import entasis "entasis:entasis"

Solver_Joint_Scenario :: enum u8
{
	Hinge, Weld, Mixed,
}
Solver_Observation_Mode :: enum u8
{
	Timing, Quality,
}
Solver_Angular_Options :: struct
{
	scenario: Solver_Joint_Scenario,
	angular_mode: entasis.Angular_Integration_Mode,
	workers, steps, chains, links, mass_ratio: int,
	output, snapshot: string,
}

parse_solver_angular_options :: proc(arguments: []string, mode: Solver_Observation_Mode) -> (Solver_Angular_Options, Admission)
{
	o: Solver_Angular_Options = {workers=1, steps=1200, chains=128, links=8, mass_ratio=1};
	seen: u32;
	for argument in arguments
	{
		if !strings.has_prefix(argument, "--")
		{
			return {}, .Invalid;
		}
		key, value: string;
		status: Admission;
		key, value, status = split_once(argument[2:], '=');
		if status != .Ok || value == ""
		{
			return {}, .Invalid;
		}
		bit: u32;
		switch key
		{
		case "scenario":
			bit = 1;
			switch value
			{
			case "hinge": o.scenario = .Hinge;
			case "weld": o.scenario = .Weld;
			case "mixed": o.scenario = .Mixed;
			case: return {}, .Invalid;
			}
		case "angular-mode":
			bit = 2;
			switch value
			{
			case "momentum": o.angular_mode = .Conserve_Momentum;
			case "gyro": o.angular_mode = .Conserve_Momentum_With_Gyroscopic_Torque;
			case: return {}, .Invalid;
			}
		case "output":
			bit = 4;
			o.output = value;
		case "snapshot":
			bit = 8;
			o.snapshot = value;
		case "worker-count", "steps", "chains", "links", "mass-ratio":
			count: int;
			count, status = parse_integer(value);
			if status != .Ok || count < 1
			{
				return {}, .Invalid;
			}
			switch key
			{
			case "worker-count":
				bit = 16;
				o.workers = count;
			case "steps":
				bit = 32;
				o.steps = count;
			case "chains":
				bit = 64;
				o.chains = count;
			case "links":
				bit = 128;
				o.links = count;
			case "mass-ratio":
				bit = 256;
				o.mass_ratio = count;
			}
		case "warmup-steps", "velocity-iterations", "substeps", "timestep-hz", "sleep", "record", "sample-every", "package":
			names: [8]string = {"warmup-steps", "velocity-iterations", "substeps", "timestep-hz", "sleep", "record", "sample-every", "package"};
			values: [8]string = {"0", "1", "4", "60", "disabled", "off", "30", "solver_joint_angular"};
			for name, index in names
			{
				if key == name
				{
					bit = u32(512) << uint(index);
					if value != values[index] || (mode == .Timing && index >= 6)
					{
						return {}, .Invalid;
					}
				}
			}
		case: return {}, .Invalid;
		}
		if seen & bit != 0
		{
			return {}, .Invalid;
		}
		seen |= bit;
	}
	if seen & 7 != 7 || o.workers > 64 || o.steps > 100000 || o.chains > 16384 || o.links > 128 ||
		o.mass_ratio > 10000 || o.chains*(o.links+1) > 1000000 ||
		(mode == .Timing && seen & 8 != 0) || (mode == .Quality && (o.snapshot == "" || o.snapshot == o.output))
	{
		return {}, .Invalid;
	}
	return o, .Ok;
}

Solver_Fixed_Case :: enum
{
	Conserving,
	Convex,
	Cached_Convex,
	Nonconvex,
	Joints,
	Nonconserving,
	Pose,
	Constrained_Final,
	Free_Final,
	Kinematic,
}

Fixed_Substeps :: enum u8
{
	Disabled,
	Enabled,
}

solver_fixed_case_name :: proc(value: Solver_Fixed_Case) -> string
{
	switch value
	{
	case .Conserving: return "conserving";
	case .Convex: return "convex";
	case .Cached_Convex: return "cached-convex";
	case .Nonconvex: return "nonconvex";
	case .Joints: return "joints";
	case .Nonconserving: return "nonconserving";
	case .Pose: return "pose";
	case .Constrained_Final: return "constrained-final";
	case .Free_Final: return "free-final";
	case .Kinematic: return "kinematic";
	}
	unreachable();
}

Solver_Fixed_Options :: struct
{
	case_name: Solver_Fixed_Case,
	workers, passes, bundles, warmup: int,
	layout, angular, output: string,
	scenario: Solver_Joint_Scenario,
	chains, links, mass_ratio: int,
	velocity_iterations, substeps: int,
	unconstrained_substeps: Fixed_Substeps,
}

parse_solver_fixed_options :: proc(arguments: []string) -> (Solver_Fixed_Options, Admission)
{
	o: Solver_Fixed_Options = {workers=1, passes=100, bundles=1250, warmup=10,
		chains=1000, links=9, mass_ratio=1, velocity_iterations=1, substeps=4};
	seen: u32;
	for argument in arguments
	{
		if !strings.has_prefix(argument, "--")
		{
			return {}, .Invalid;
		}
		key, value: string;
		status: Admission;
		key, value, status = split_once(argument[2:], '=');
		if status != .Ok || value == ""
		{
			return {}, .Invalid;
		}
		bit: u32;
		switch key
		{
		case "case":
			bit = 1;
			matched: Admission = .Invalid;
			for candidate in Solver_Fixed_Case
			{
				if value == solver_fixed_case_name(candidate)
				{
					o.case_name = candidate;
					matched = .Ok;
				}
			}
			if matched != .Ok
			{
				return {}, .Invalid;
			}
		case "layout":
			bit = 2;
			if value != "full" && value != "partial" && value != "sparse" && value != "isotropic"
			{
				return {}, .Invalid;
			}
			o.layout = value;
		case "angular-mode":
			bit = 4;
			if value != "momentum" && value != "gyro" && value != "nonconserving"
			{
				return {}, .Invalid;
			}
			o.angular = value;
		case "output":
			bit = 8;
			o.output = value;
		case "scenario":
			bit = 256;
			switch value
			{
			case "hinge": o.scenario = .Hinge;
			case "weld": o.scenario = .Weld;
			case "mixed": o.scenario = .Mixed;
			case: return {}, .Invalid;
			}
		case "unconstrained-substeps":
			bit = 16384;
			switch value
			{
			case "enabled": o.unconstrained_substeps = .Enabled;
			case "disabled": o.unconstrained_substeps = .Disabled;
			case: return {}, .Invalid;
			}
		case "worker-count", "passes", "bundles", "warmup-passes", "chains", "links", "mass-ratio", "velocity-iterations", "substeps":
			count: int;
			count, status = parse_integer(value);
			if status != .Ok
			{
				return {}, .Invalid;
			}
			switch key
			{
			case "worker-count":
				bit = 16;
				o.workers = count;
			case "passes":
				bit = 32;
				o.passes = count;
			case "bundles":
				bit = 64;
				o.bundles = count;
			case "warmup-passes":
				bit = 128;
				o.warmup = count;
			case "chains":
				bit = 512;
				o.chains = count;
			case "links":
				bit = 1024;
				o.links = count;
			case "mass-ratio":
				bit = 2048;
				o.mass_ratio = count;
			case "velocity-iterations":
				bit = 4096;
				o.velocity_iterations = count;
			case "substeps":
				bit = 8192;
				o.substeps = count;
			}
		case: return {}, .Invalid;
		}
		if seen & bit != 0
		{
			return {}, .Invalid;
		}
		seen |= bit;
	}
	if seen & 9 != 9 || o.workers < 1 || o.workers > 64 || o.passes < 1 || o.passes > 100000 ||
		o.bundles < 8 || o.bundles > 65536 || o.warmup < 0 || o.warmup > 1000 ||
		o.chains < 1 || o.chains > 16384 || o.links < 1 || o.links > 128 ||
		o.mass_ratio < 1 || o.mass_ratio > 10000 || o.chains*(o.links+1) > 1000000
	{
		return {}, .Invalid;
	}
	allowed: u32 = 1|8|16|32|128;
	switch o.case_name
	{
	case .Conserving:
		allowed |= 2|4|64;
		if seen & 6 != 6 || o.angular == "nonconserving"
		{
			return {}, .Invalid;
		}
		if seen & 32 == 0
		{
			o.passes = 512;
		}
		if seen & 64 == 0
		{
			o.bundles = 4096;
		}
		if seen & 128 == 0
		{
			o.warmup = 4;
		}
	case .Convex, .Nonconvex:
		allowed |= 64;
	case .Cached_Convex:
		allowed |= 64|4096|8192;
		if seen & (4096|8192) == 0
		{
			o.velocity_iterations, o.substeps = 2, 2;
		}
		if !((o.velocity_iterations == 2 && o.substeps == 2) || (o.velocity_iterations == 4 && o.substeps == 1))
		{
			return {}, .Invalid;
		}
	case .Joints:
		allowed |= 4|256|512|1024|2048;
		if seen & (4|256) != (4|256) || o.angular == "nonconserving"
		{
			return {}, .Invalid;
		}
	case .Nonconserving, .Pose, .Constrained_Final, .Free_Final:
		allowed |= 2|4|64;
		if seen & 6 != 6 || (o.case_name == .Nonconserving && o.angular != "nonconserving")
		{
			return {}, .Invalid;
		}
		if (o.case_name == .Pose || o.case_name == .Constrained_Final || o.case_name == .Free_Final) &&
			o.layout != "full" && o.layout != "isotropic"
		{
			return {}, .Invalid;
		}
		if o.case_name == .Free_Final
		{
			allowed |= 16384;
			if seen & 16384 == 0
			{
				return {}, .Invalid;
			}
		}
	case .Kinematic:
		allowed |= 2|64;
		if seen & 2 == 0
		{
			return {}, .Invalid;
		}
	}
	if seen & ~allowed != 0
	{
		return {}, .Invalid;
	}
	return o, .Ok;
}
