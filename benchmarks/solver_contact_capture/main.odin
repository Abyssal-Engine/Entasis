package solver_contact_capture

import "core:fmt"
import "core:os"
import "base:runtime"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"
import islands "../contact_islands"
import support "../benchmark_support"
import observations "../scene_observation"
import contacts "../solver_contacts"

Capture_Context :: struct
{
	simulation: ^physics.Simulation,
	origin: contacts.Origin,
	paths: [2]string,
	captured: u8,
}

capture_stage :: proc "contextless" (
	user_context: rawptr, stage: physics.Timestep_Completion_Stage, dt: f32,
	dispatcher: ^util.Thread_Dispatcher_Boundary,
) -> physics.Physics_Status
{
	context = runtime.default_context();
	if stage != .Collisions_Detected
	{
		return .Ok;
	}
	c: ^Capture_Context = (^Capture_Context)(user_context);
	step: int = int(c.simulation.step_index) + 1;
	index: int;
	if step == 180
	{
		index = 0;
	}
	else if step == 330
	{
		index = 1;
	}
	else
	{
		return .Ok;
	}
	input: contacts.Input;
	defer contacts.input_destroy(&input);
	status: contacts.Status = contacts.input_capture(c.simulation, &input, c.origin, i32(step-30));
	if status == .Ok
	{
		status = contacts.input_write(c.paths[index], &input);
	}
	if status != .Ok
	{
		fmt.eprintfln("CONTACT_CAPTURE_FAILED step=%d status=%v path=%s", step, status, c.paths[index]);
		return .Invalid_Argument;
	}
	counts: contacts.Counts = contacts.input_counts(&input);
	c.captured |= u8(1) << uint(index);
	fmt.printfln("CONTACT_CAPTURE step=%d measured_step=%d bodies=%d manifolds=%d points=%d batches=%d path=%s",
		step, step-30, counts.bodies, counts.manifolds, counts.points, counts.batches, c.paths[index]);
	return .Ok;
}

capture_timestep :: proc "contextless" (
	user_context: rawptr, simulation: ^physics.Simulation, dt: f32,
	dispatcher: ^util.Thread_Dispatcher_Boundary,
) -> physics.Physics_Status
{
	c: ^Capture_Context = (^Capture_Context)(user_context);
	c.simulation = simulation;
	return physics.default_timestepper_step(nil, simulation, dt, dispatcher);
}

capture_run :: proc(o: Options) -> (exit_code: int)
{
	c: Capture_Context;
	c.origin = .Accepted if o.origin == "accepted" else .R02_Pairs;
	c.paths[0] = fmt.aprintf("%s/%s.step150.bin", o.output_root, o.origin);
	c.paths[1] = fmt.aprintf("%s/%s.step300.bin", o.output_root, o.origin);
	defer delete(c.paths[0]);
	defer delete(c.paths[1]);
	for path in c.paths
	{
		if os.exists(path)
		{
			fmt.eprintfln("CONTACT_CAPTURE_REJECT existing_path=%s", path);
			return 2;
		}
	}
	output: string = fmt.aprintf("%s/%s.capture.csv", o.output_root, o.origin);
	defer delete(output);
	output_argument: string = fmt.aprintf("--output=%s", output);
	defer delete(output_argument);
	options: support.Options;
	diagnostic: string;
	options, diagnostic = support.parse_options(.Contact_Islands, {
		"--worker-count=1", "--shape=box", "--steps=300", "--warmup-steps=30",
		"--velocity-iterations=1", "--substeps=4", "--timestep-hz=60",
		"--sleep=disabled", "--record=off", "--verify=on", output_argument,
	});
	if diagnostic != ""
	{
		fmt.eprintfln("CONTACT_CAPTURE_REJECT %s", diagnostic);
		return 2;
	}
	q: observations.Qualification;
	if observations.qualification_create(&q, islands.benchmark_qualification_configuration(options), .On) != nil
	{
		return 3;
	}
	defer observations.qualification_destroy(&q);
	pool: entasis.Buffer_Pool;
	if entasis.buffer_pool_init(&pool, islands.POOL_MINIMUM_BLOCK_SIZE) != .Ok
	{
		return 3;
	}
	owner: islands.Benchmark_Owner;
	defer
	{
		release_status: entasis.Status = islands.benchmark_owner_destroy(&owner);
		pool_status: entasis.Status = entasis.buffer_pool_destroy(&pool);
		if release_status != .Ok || pool_status != .Ok
		{
			fmt.eprintfln("CONTACT_CAPTURE_RELEASE_FAILED world=%v pool=%v", release_status, pool_status);
			if exit_code == 0
			{
				exit_code = 3;
			}
		}
	}
	result: support.Benchmark_Result = islands.benchmark_owner_create(&owner, &pool, options,
		{step=capture_timestep, user_context=&c}, {stage_completed=capture_stage, user_context=&c});
	if result.status != .Ok
	{
		fmt.eprintfln("CONTACT_CAPTURE_CREATE_FAILED status=%v physics=%v", result.status, result.physics_status);
		return 3;
	}
	if islands.benchmark_assess(&owner, &q, .Construction, 0) != .Ok
	{
		return 3;
	}
	for total_step in 1 ..= 330
	{
		status: entasis.Status = entasis.world_step(&owner.world, 1.0/60.0);
		if status != .Ok
		{
			fmt.eprintfln("CONTACT_CAPTURE_STEP_FAILED step=%d physics=%v", total_step, status);
			return 3;
		}
		phase: observations.Assessment_Phase = .Warmup if total_step <= 30 else .Measured;
		phase_step: int = total_step if total_step <= 30 else total_step-30;
		if islands.benchmark_assess(&owner, &q, phase, phase_step) != .Ok
		{
			return 3;
		}
	}
	sample: support.Benchmark_Sample;
	validation: support.Benchmark_Result = islands.benchmark_validate(&owner, &sample);
	if validation.status != .Ok
	{
		fmt.eprintfln("CONTACT_CAPTURE_VALIDATE_FAILED status=%v physics=%v", validation.status, validation.physics_status);
		return 3;
	}
	observations.qualification_complete(&q);
	physical_buffer: [4096]u8;
	fmt.println("CONTACT_CAPTURE_VERIFICATION"+observations.PHYSICAL_HEADER);
	fmt.printfln("CONTACT_CAPTURE_VERIFICATION%s", observations.physical_csv(q.result, physical_buffer[:]));
	fmt.printfln("CONTACT_CAPTURE_STATE step=%d invalid_transforms=%d below_floor=%d out_of_bounds=%d",
		330, sample.invalid_transform_count, sample.below_floor_count, sample.out_of_bounds_count);
	if c.captured != 3 || q.result.status != .Pass || q.result.coverage != .Complete
	{
		return 2;
	}
	return 0;
}

main :: proc()
{
	o: Options;
	status: support.Admission;
	o, status = parse_options(os.args[1:]);
	if status != .Ok
	{
		fmt.eprintln("CONTACT_CAPTURE_REJECT requires --origin=accepted|r02-pairs --output-root=PATH");
		os.exit(2);
	}
	os.exit(capture_run(o));
}
