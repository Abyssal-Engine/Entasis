package simulation_stage_work

import "base:runtime"
import "core:time"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

stage_timestep :: proc "contextless" (
	user_context: rawptr, simulation: ^physics.Simulation, dt: f32,
	dispatcher: ^util.Thread_Dispatcher_Boundary,
) -> physics.Physics_Status
{
	context = runtime.default_context();
	run: ^Run = (^Run)(user_context);
	run.simulation = simulation;
	if run.frame_index == WARMUP_FRAMES
	{
		run.pool_before = util.buffer_pool_total_allocated_byte_count(simulation.pool);
		run.worker_before = worker_capacity(dispatcher);
	}
	prescribe(run, simulation, run.frame_index);
	status: physics.Physics_Status = physics.simulation_predict_bounding_boxes(simulation, dt, dispatcher);
	if status != .Ok
	{
		return status;
	}
	if run.options.stage == .Pair_Traversal || run.options.stage == .Narrow_Phase
	{
		status = physics.broad_phase_update(&simulation.broad_phase, dispatcher);
		if status != .Ok
		{
			return status;
		}
	}
	pairs_reset(&run.pairs);
	start: time.Tick = time.tick_now();
	switch run.options.stage
	{
	case .Tree_Update:
		status = physics.broad_phase_update(&simulation.broad_phase, dispatcher);
	case .Pair_Traversal:
		status = physics.broad_phase_visit_pairs(&simulation.broad_phase, pair_collect, &run.pairs, dispatcher);
	case .Narrow_Phase:
		status = physics.narrow_phase_execute(&simulation.narrow_phase, dispatcher, dt);
	case .Collision:
		status = physics.simulation_collision_detection(simulation, dt, dispatcher);
	}
	elapsed: time.Duration = time.tick_diff(start, time.tick_now());
	if status != .Ok
	{
		return status;
	}
	if run.options.stage != .Pair_Traversal
	{
		status = physics.broad_phase_visit_pairs(&simulation.broad_phase, pair_collect, &run.pairs, dispatcher);
		if status != .Ok
		{
			return status;
		}
	}
	status = observe(run, simulation, run.frame_index);
	if run.frame_index >= WARMUP_FRAMES
	{
		run.elapsed_ms += f64(elapsed)/1e6;
		run.completed += 1;
		run.pair_count += i64(run.last_pairs);
		run.manifold_count += i64(run.last_manifolds);
		if status != .Ok
		{
			run.failures += 1;
			if run.first_failure == 0
			{
				run.first_failure = run.frame_index+1;
			}
		}
	}
	else if status != .Ok
	{
		return status;
	}
	run.pool_after = util.buffer_pool_total_allocated_byte_count(simulation.pool);
	run.worker_after = worker_capacity(dispatcher);
	run.frame_index += 1;
	simulation.step_index += 1;
	return .Ok;
}
