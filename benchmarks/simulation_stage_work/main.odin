package simulation_stage_work

import "core:fmt"
import "core:os"
import entasis "entasis:entasis"
import util "entasis:entasis_utilities"
import support "../benchmark_support"

write_result :: proc(run: ^Run) -> int
{
	file: ^os.File;
	error: os.Error;
	file, error = os.open(run.options.output, {.Write, .Append, .Create});
	if error != nil
	{
		return 1;
	}
	defer os.close(file);
	size: i64;
	size, error = os.file_size(file);
	if error != nil
	{
		return 1;
	}
	if size == 0
	{
		fmt.fprintln(file, "case_status,metric_status,worker_count,stage,fixture,motion,bodies,statics,measured_passes,warmup_passes,fixed_elapsed_ms,pass_ms,physics_status,quality_coverage,quality_failures,pair_count,manifold_count,contact_points,pool_capacity_before,pool_capacity_after,worker_pool_capacity_before,worker_pool_capacity_after,precision,benchmark_parameters");
	}
	case_status, metric_status, physics_status: string = "ok", "ok", "pass";
	exit_code: int;
	if run.completed != MEASURED_FRAMES || run.elapsed_ms <= 0 || run.status != .Ok
	{
		case_status = "failed";
		metric_status = "failed";
		exit_code = 3;
	}
	if run.failures != 0 || run.completed != MEASURED_FRAMES
	{
		physics_status = "fail";
		if exit_code == 0
		{
			exit_code = 2;
		}
	}
	mean: f64;
	if run.completed > 0
	{
		mean = run.elapsed_ms/f64(run.completed);
	}
	fmt.fprintfln(file, "%s,%s,%d,%s,%s,%s,%d,%d,%d,%d,%.9f,%.9f,%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%s,\"workload=simulation_stage_work;stage=%s;fixture=%s;motion=%s;bodies=%d;statics=%d;region=selected-native-stage;warmup-passes=%d;measured-passes=%d;timestep-hz=60;history=persistent-prescribed;capacity-window=measured-frame;geometry=unit-axis-boxes\"",
		case_status, metric_status, run.options.workers, stage_text(run.options.stage), fixture_text(run.options.fixture), motion_text(run.options.motion),
		len(run.initial), len(run.statics), MEASURED_FRAMES, WARMUP_FRAMES, run.elapsed_ms, mean,
		physics_status, run.completed, run.failures, run.pair_count, run.manifold_count, run.contact_points,
		run.pool_before, run.pool_after, run.worker_before, run.worker_after, util.PRECISION_NAME,
		stage_text(run.options.stage), fixture_text(run.options.fixture), motion_text(run.options.motion), len(run.initial), len(run.statics), WARMUP_FRAMES, MEASURED_FRAMES);
	fmt.printfln("STAGE_WORK_COMPLETE stage=%s fixture=%s motion=%s frames=%d/%d quality_failures=%d first_failure=%d status=%v exit_code=%d",
		stage_text(run.options.stage), fixture_text(run.options.fixture), motion_text(run.options.motion), run.completed, MEASURED_FRAMES,
		run.failures, run.first_failure, run.status, exit_code);
	return exit_code;
}

work_run :: proc(o: Options) -> int
{
	run: Run;
	defer run_destroy(&run);
	grid: [3]int = {25, 16, 25};
	groups: [2]int = {1, 1};
	if o.fixture == .Islands
	{
		grid = {5, 8, 5};
		groups = {10, 5};
	}
	run.status = run_create(&run, o, grid, groups);
	if run.status != .Ok
	{
		fmt.eprintfln("STAGE_WORK_CREATE_FAILED status=%v", run.status);
		return 3;
	}
	for _ in 0 ..< WARMUP_FRAMES+MEASURED_FRAMES
	{
		status: entasis.Status = entasis.world_step(run_world(&run), TIMESTEP);
		if status != .Ok
		{
			run.status = .Invalid_Argument;
			run.first_failure = run.frame_index+1;
			break;
		}
	}
	return write_result(&run);
}

main :: proc()
{
	o: Options;
	status: support.Admission;
	o, status = parse_options(os.args[1:]);
	if status != .Ok
	{
		fmt.eprintln("STAGE_WORK_REJECT requires --stage=tree-update|pair-traversal|narrow-phase|collision --fixture=container|islands --motion=steady|churn --worker-count=N --output=CSV");
		os.exit(2);
	}
	os.exit(work_run(o));
}
