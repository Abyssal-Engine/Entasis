package pyramid_wall

import "core:fmt"
import "core:os"
import "core:time"
import entasis "entasis:entasis"
import support "../benchmark_support"

parse_options :: proc(arguments: []string) -> (Options, support.Admission)
{
	options: Options = {
		steps=18000, worker_count=1, velocity_iterations=1, substeps=4,
		timestep_hz=60, sample_every=60, sleep_threshold=-1,
		friction=0.5, hertz=30, damping_ratio=1, recovery=2,
	}
	names: [15]string = {"rows", "steps", "worker-count", "velocity-iterations", "substeps", "timestep-hz", "sample-every", "sleep", "friction", "hertz", "damping-ratio", "recovery", "recycle-distance", "output", "snapshot"}
	seen: u32
	for argument in arguments
	{
		if len(argument) < 3 || argument[:2] != "--"
		{
			return {}, .Invalid
		}
		key, value: string
		status: support.Admission
		key, value, status = support.split_once(argument[2:], '=')
		if status != .Ok || len(value) == 0
		{
			return {}, .Invalid
		}
		index: int = -1
		for name, candidate in names
		{
			if name == key
			{
				index = candidate
				break
			}
		}
		if index < 0 || seen & (u32(1) << u32(index)) != 0
		{
			return {}, .Invalid
		}
		seen |= u32(1) << u32(index)
		if index <= 6
		{
			number: int
			number, status = support.parse_integer(value)
			if status != .Ok || number < 1 || number > 1_000_000
			{
				return {}, .Invalid
			}
			switch index
			{
				case 0:
				if number != ROWS
				{
					return {}, .Invalid
				}
				case 1:
				options.steps = number
				case 2:
				if number > 64
				{
					return {}, .Invalid
				}
				options.worker_count = number
				case 3:
				options.velocity_iterations = number
				case 4:
				options.substeps = number
				case 5:
				options.timestep_hz = number
				case 6:
				options.sample_every = number
			}
		}
		else if index == 7
		{
			switch value
			{
				case "disabled":
				options.sleep_threshold = -1
				case "enabled":
				options.sleep_threshold = 0.01
				case:
				return {}, .Invalid
			}
		}
		else if index <= 12
		{
			number: f32
			number, status = support.parse_number(value)
			if status != .Ok || number < 0 || (index == 12 && number != 0.05)
			{
				return {}, .Invalid
			}
			switch index
			{
				case 8:
				options.friction = number
				case 9:
				options.hertz = number
				case 10:
				options.damping_ratio = number
				case 11:
				options.recovery = number
			}
		}
		else if index == 13
		{
			options.output = value
		}
		else
		{
			options.snapshot = value
		}
	}
	if len(options.output) == 0 || len(options.snapshot) == 0 || options.output == options.snapshot ||
		os.exists(options.output) || os.exists(options.snapshot) || options.hertz <= 0 || options.damping_ratio <= 0
	{
		return {}, .Invalid
	}
	return options, .Ok
}

main :: proc()
{
	options: Options
	admission: support.Admission
	options, admission = parse_options(os.args[1:])
	if admission != .Ok
	{
		fmt.eprintln("invalid wall options: rows must be 180, outputs must be fresh")
		os.exit(2)
	}
	pool: entasis.Buffer_Pool
	if entasis.buffer_pool_init(&pool, 65536, 16) != .Ok
	{
		os.exit(1)
	}
	defer entasis.buffer_pool_destroy(&pool)
	owner: Owner
	defer owner_destroy(&owner)
	if owner_create(&owner, &pool, options) != .Ok
	{
		os.exit(1)
	}
	file, snapshot: ^os.File
	file_error: os.Error
	file, file_error = os.open(options.output, {.Write, .Create, .Excl})
	if file_error != nil
	{
		os.exit(1)
	}
	snapshot, file_error = os.open(options.snapshot, {.Write, .Create, .Excl})
	if file_error != nil
	{
		os.exit(1)
	}
	if write_header(file) != .Ok
	{
		fmt.eprintln("WALL_OUTPUT_FAILED stage=header")
		os.exit(1)
	}
	fmt.printfln("WALL_CONFIG rows=%d bodies=%d workers=%d steps=%d velocity_iterations=%d substeps=%d timestep_hz=%d sleep_threshold=%g friction=%g hertz=%g damping_ratio=%g recovery=%g continuity=discrete", ROWS, BODY_COUNT, options.worker_count, options.steps, options.velocity_iterations, options.substeps, options.timestep_hz, options.sleep_threshold, options.friction, options.hertz, options.damping_ratio, options.recovery)
	elapsed_ms: f64
	for step: int = 0; step <= options.steps; step += 1
	{
		if step > 0
		{
			start: time.Tick = time.tick_now()
			status: entasis.Status = entasis.world_step(&owner.world, 1 / f32(options.timestep_hz))
			end: time.Tick = time.tick_now()
			elapsed_ms += f64(time.tick_diff(start, end)) / 1_000_000
			if status != .Ok
			{
				fmt.eprintfln("WALL_STEP_FAILED step=%d status=%v", step, status)
				os.exit(1)
			}
		}
		if step == 0 || step == 1 || step % options.sample_every == 0 || step == options.steps
		{
			if write_observation(file, &owner, options, step, elapsed_ms) != .Ok
			{
				os.exit(1)
			}
		}
	}
	if write_snapshot(snapshot, &owner) != .Ok
	{
		os.exit(1)
	}
	file_error = os.close(file)
	snapshot_error: os.Error = os.close(snapshot)
	if file_error != nil || snapshot_error != nil
	{
		fmt.eprintln("OBSERVATION_OUTPUT_FAILED stage=close")
		os.exit(1)
	}
	fmt.println("PYRAMID_WALL_COMPLETED")
}
