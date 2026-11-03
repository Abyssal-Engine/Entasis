package solver_joint_angular

import "core:fmt"
import "core:os"
import "core:time"
import entasis "entasis:entasis"
import support "../benchmark_support"

execute :: proc(o: Options, mode: Observation_Mode) -> int
{
	pool: entasis.Buffer_Pool;
	if entasis.buffer_pool_init(&pool, 65536, 16) != .Ok
	{
		return 1;
	}
	defer entasis.buffer_pool_destroy(&pool);
	owner: Benchmark_Owner;
	defer benchmark_owner_destroy(&owner);
	result: support.Benchmark_Result = benchmark_owner_create(&owner, &pool, o);
	if result.status != .Ok
	{
		fmt.eprintfln("ANGULAR_FIXTURE_FAILED status=%v physics=%v", result.status, result.physics_status);
		return 1;
	}
	observation: Observation = {hash=14695981039346656037};
	file: ^os.File;
	if mode == .Quality
	{
		error: os.Error;
		file, error = os.open(o.output, {.Write, .Create, .Excl});
		if error != nil
		{
			return 1;
		}
		fmt.fprintfln(file, "step_index,%s", OBSERVATION_HEADER);
	}
	defer if file != nil
	{
		os.close(file);
	}
	elapsed: f64;
	for step in 1 ..= o.steps
	{
		start: time.Tick = time.tick_now();
		status: entasis.Status = entasis.world_step(&owner.world, 1.0 / 60.0);
		elapsed += f64(time.tick_diff(start, time.tick_now()))/1e6;
		if status != .Ok
		{
			fmt.eprintfln("ANGULAR_STEP_FAILED step=%d status=%v", step, status);
			return 1;
		}
		if mode == .Quality || step == o.steps
		{
			if observe(&owner, step, &observation) != .Ok
			{
				return 1;
			}
			if mode == .Quality && (step%30 == 0 || step == o.steps)
			{
				stats: entasis.World_Stats;
				stats, status = entasis.world_stats(&owner.world);
				if status != .Ok
				{
					return 1;
				}
				buffer: [1024]u8;
				fmt.fprintfln(file, "%d,%s", step, observation_csv(observation, stats, buffer[:]));
			}
		}
	}
	stats: entasis.World_Stats;
	status: entasis.Status;
	stats, status = entasis.world_stats(&owner.world);
	if status != .Ok
	{
		return 1;
	}
	if mode == .Timing
	{
		parameters: [support.PARAMETER_CAPACITY]u8;
		observation_buffer: [1024]u8;
		csv_options: support.Options = {worker_count=o.workers, steps=o.steps, body_count=len(owner.bodies), output=o.output};
		sample: support.Benchmark_Sample = {stats=stats, elapsed_ms=elapsed, invalid_transform_count=observation.invalid};
		if support.write_result(csv_options, canonical_parameters(o, parameters[:]), sample,
			","+OBSERVATION_HEADER, fmt.tprintf(",%s", observation_csv(observation, stats, observation_buffer[:])),
			"native_step_sum", .Absent, shape_count=0, constraint_count=len(owner.edges)) != .Ok
		{
			return 1;
		}
	}
	else if write_snapshot(&owner, o.snapshot) != .Ok
	{
		return 1;
	}
	if observation.invalid > 0 || stats.sleeping_bodies > 0
	{
		return 3;
	}
	return 0;
}

run_quality :: proc(arguments: []string) -> int
{
	o: Options;
	admission: support.Admission;
	o, admission = support.parse_solver_angular_options(arguments, .Quality);
	if admission != .Ok
	{
		return 2;
	}
	return execute(o, .Quality);
}

main :: proc()
{
	o: Options;
	admission: support.Admission;
	o, admission = support.parse_solver_angular_options(os.args[1:], .Timing);
	if admission != .Ok
	{
		os.exit(2);
	}
	os.exit(execute(o, .Timing));
}
