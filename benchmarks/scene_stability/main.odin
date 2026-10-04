package scene_stability

import "base:runtime"
import "core:fmt"
import "core:os"
import entasis "entasis:entasis"
import islands "../contact_islands"
import pyramid "../pyramid"
import support "../benchmark_support"
import observations "../scene_observation"

main :: proc()
{
	arguments: [64]string
	argument_count: int
	workload: support.Workload = .Pyramid
	snapshot_path: string
	sample_every: int = 60
	seen: u8
	for argument in os.args[1:]
	{
		if len(argument) < 3 || argument[:2] != "--"
		{
			os.exit(2)
		}
		key, value: string
		admission: support.Admission
		key, value, admission = support.split_once(argument[2:], '=')
		if admission != .Ok || len(value) == 0
		{
			os.exit(2)
		}
		bit: u8
		switch key
		{
			case "package":
			bit = 1
			switch value
			{
				case "contact_islands":
				workload = .Contact_Islands
				case "pyramid":
				workload = .Pyramid
				case:
				os.exit(2)
			}
			case "snapshot":
			bit = 2
			snapshot_path = value
			case "sample-every":
			bit = 4
			sample_every, admission = support.parse_integer(value)
			if admission != .Ok || sample_every < 1
			{
				os.exit(2)
			}
			case:
			if argument_count == len(arguments)
			{
				os.exit(2)
			}
			arguments[argument_count] = argument
			argument_count += 1
		}
		if bit != 0 && seen & bit != 0
		{
			os.exit(2)
		}
		seen |= bit
	}
	if seen & 3 != 3
	{
		os.exit(2)
	}
	options: support.Options
	diagnostic: string
	options, diagnostic = support.parse_options(workload, arguments[:argument_count])
	if len(diagnostic) != 0 || options.shape != .Box || options.warmup_steps != 0 ||
		(workload == .Pyramid && options.projectile_count != 0) || os.exists(options.output) ||
		os.exists(snapshot_path) || options.output == snapshot_path
	{
		fmt.eprintfln("invalid stability options: %s", diagnostic)
		os.exit(2)
	}
	pool: entasis.Buffer_Pool
	if entasis.buffer_pool_init(&pool, 65536, 16) != .Ok
	{
		os.exit(1)
	}
	defer entasis.buffer_pool_destroy(&pool)
	island_owner: islands.Benchmark_Owner
	pyramid_owner: pyramid.Benchmark_Owner
	defer islands.benchmark_owner_destroy(&island_owner)
	defer pyramid.benchmark_owner_destroy(&pyramid_owner)
	world: ^entasis.World
	bodies: []entasis.Body_Handle
	if workload == .Contact_Islands
	{
		result: islands.Benchmark_Result = islands.benchmark_owner_create(&island_owner, &pool, options)
		if result.status != .Ok
		{
			os.exit(1)
		}
		world = &island_owner.world
		bodies = island_owner.bodies
	}
	else
	{
		result: pyramid.Benchmark_Result = pyramid.benchmark_owner_create(&pyramid_owner, &pool, options)
		if result.status != .Ok
		{
			os.exit(1)
		}
		world = &pyramid_owner.world
		bodies = pyramid_owner.bodies
	}
	initial: []entasis.Vector3
	allocation_error: runtime.Allocator_Error
	initial, allocation_error = make([]entasis.Vector3, len(bodies))
	if allocation_error != nil
	{
		os.exit(1)
	}
	defer delete(initial)
	for body, index in bodies
	{
		state: entasis.Body_State
		status: entasis.Status
		state, status = entasis.body_get(world, body)
		if status != .Ok
		{
			os.exit(1)
		}
		initial[index] = state.pose.position
	}
	file, snapshot: ^os.File
	file_error: os.Error
	file, file_error = os.open(options.output, {.Write, .Create, .Excl})
	if file_error != nil
	{
		os.exit(1)
	}
	snapshot, file_error = os.open(snapshot_path, {.Write, .Create, .Excl})
	if file_error != nil
	{
		os.exit(1)
	}
	if write_header(file) != .Ok
	{
		fmt.eprintln("STABILITY_OUTPUT_FAILED stage=header")
		os.exit(1)
	}
	for step: int = 0; step <= options.steps; step += 1
	{
		if step > 0
		{
			status: entasis.Status = entasis.world_step(world, 1 / f32(options.timestep_hz))
			if status != .Ok
			{
				fmt.eprintfln("STABILITY_STEP_FAILED step=%d status=%v", step, status)
				os.exit(1)
			}
		}
		if step == 0 || step == 1 || step % sample_every == 0 || step == options.steps
		{
			if observe_scene(file, world, bodies, initial, options, step) != .Ok
			{
				fmt.eprintfln("STABILITY_OBSERVATION_FAILED step=%d", step)
				os.exit(1)
			}
		}
	}
	if write_snapshot(snapshot, world, bodies) != .Ok
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
	fmt.println("SCENE_STABILITY_COMPLETED")
}

observe_scene :: proc(file: ^os.File, world: ^entasis.World, bodies: []entasis.Body_Handle, initial: []entasis.Vector3, options: support.Options, step: int) -> support.Case_Status
{
	group_count: int = 1
	group_size: int = len(bodies)
	if options.workload == .Contact_Islands
	{
		group_count = options.island_grid[0] * options.island_grid[1]
		group_size = options.grid[0] * options.grid[1] * options.grid[2]
	}
	stats: entasis.World_Stats
	stats_status: entasis.Status
	stats, stats_status = entasis.world_stats(world)
	if stats_status != .Ok
	{
		os.exit(1)
	}
	for group: int = 0; group < group_count; group += 1
	{
		floor_center: entasis.Vector3
		if options.workload == .Contact_Islands
		{
			floor_center.x = islands.island_origin(group % options.island_grid[0], options.island_grid[0], options.island_spacing[0])
			floor_center.z = islands.island_origin(group / options.island_grid[0], options.island_grid[1], options.island_spacing[1])
		}
		observation: Observation
		for index: int = group * group_size; index < (group + 1) * group_size; index += 1
		{
			state: entasis.Body_State
			status: entasis.Status
			state, status = entasis.body_get(world, bodies[index])
			if status != .Ok
			{
				observation.count += 1
				observation.invalid += 1
				observation.initial_sum_y += f64(initial[index].y)
				continue
			}
			observations.observe_body(&observation, state, initial[index], options.shape_size, floor_center, {options.floor_size[0]/2, options.floor_size[2]/2})
		}
		status: support.Case_Status = write_observation(file, observation, step, options.timestep_hz, group, group_size, stats.active_bodies + stats.sleeping_bodies)
		if status != .Ok
		{
			return status
		}
	}
	return .Ok
}
