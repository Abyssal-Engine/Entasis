package solver_fixed_work

import "base:runtime"
import "core:simd"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

fixed_integration_worker :: proc "contextless" (worker_index: int, dispatcher: ^util.Thread_Dispatcher_Boundary)
{
	context = runtime.default_context();
	s: ^Fixed_State = (^Fixed_State)(dispatcher.unmanaged_context);
	count: int = len(s.initial);
	bundle_count: int = (count+7)/8;
	first: int = bundle_count*worker_index/dispatcher.worker_count;
	end: int = bundle_count*(worker_index+1)/dispatcher.worker_count;
	status: physics.Physics_Status = .Ok;
	for bundle in first ..< end
	{
		if s.options.case_name == .Constrained_Final
		{
			job: physics.Solver_Substep_Job = {solver=&s.solver, dt=1.0/240.0, inverse_dt=240,
				post_solve_callback_dt=1.0/60.0, post_solve_callback_substep_count=1};
			status = physics.solver_execute_post_solve_bundle(&job, bundle, .Integrate_Constrained_Poses, worker_index);
			if status != .Ok
			{
				break;
			}
			continue;
		}
		references: util.I32x8 = util.I32x8(-1);
		mask: util.I32x8 = fixed_lane_mask(s.options.layout, bundle, count);
		for lane in 0 ..< 8
		{
			index: int = bundle*8+lane;
			if index >= count
			{
				break;
			}
			encoded: i32 = i32(index);
			if s.options.case_name == .Kinematic
			{
				encoded = i32(u32(index)|physics.BODY_REFERENCE_KINEMATIC_MASK);
			}
			references = simd.replace(references, lane, encoded);
			if s.options.case_name == .Nonconserving
			{
				s.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX].dynamics_state.memory[index] = s.initial[index];
			}
		}
		for substep in 0 ..< 4
		{
			phase: physics.Solver_Substep_Phase = .First if substep == 0 else .Continuation;
			if s.options.case_name == .Kinematic
			{
				physics.solver_integrate_kinematic_lanes(&s.solver, references, mask, 1.0/240.0, phase, worker_index);
			}
			else
			{
				position: util.Vector3_Wide;
				orientation: util.Quaternion_Wide;
				velocity: physics.Body_Velocity_Wide;
				inertia: physics.Body_Inertia_Wide;
				position, orientation, velocity, inertia = physics.bodies_gather_active_trusted(
					&s.bodies, references, .Local, physics.BODY_ACCESS_ALL);
				_ = physics.solver_integrate_constraint_body(&s.solver, references, mask,
					1.0/240.0, phase, worker_index, &position, &orientation, &velocity, inertia);
			}
		}
	}
	s.worker_statuses[worker_index] = status;
}

fixed_execute :: proc(s: ^Fixed_State, dispatcher: ^util.Thread_Dispatcher_Boundary = nil) -> physics.Physics_Status
{
	switch s.options.case_name
	{
	case .Convex, .Cached_Convex, .Nonconvex, .Joints:
		status: physics.Physics_Status = physics.pose_integrator_prepare(&s.integrator, 1.0/(60*f32(s.options.substeps)));
		if status != .Ok
		{
			return status;
		}
		solve: physics.Solve_Description = {velocity_iteration_count=i32(s.options.velocity_iterations), substep_count=i32(s.options.substeps), fallback_batch_threshold=64};
		return physics.solver_solve(&s.solver, 1.0/60.0, &solve, dispatcher);
	case .Pose:
		status: physics.Physics_Status = physics.pose_integrator_prepare(&s.integrator, 1.0/60.0);
		if status != .Ok
		{
			return status;
		}
		return physics.pose_integrator_integrate_poses(&s.integrator, 1.0/60.0, .Missing, dispatcher);
	case .Free_Final:
		return physics.pose_integrator_integrate_after_substepping(&s.integrator, 1.0/60.0, 4, dispatcher);
	case .Nonconserving, .Constrained_Final, .Kinematic:
		status: physics.Physics_Status = physics.pose_integrator_prepare(&s.integrator, 1.0/240.0);
		if status != .Ok
		{
			return status;
		}
		if dispatcher == nil
		{
			serial: util.Thread_Dispatcher_Boundary = {worker_count=1, unmanaged_context=s};
			fixed_integration_worker(0, &serial);
			return s.worker_statuses[0];
		}
		if dispatcher.dispatch(dispatcher, fixed_integration_worker, dispatcher.worker_count, s) != .Ok
		{
			return .Capacity_Missing;
		}
		for worker in 0 ..< dispatcher.worker_count
		{
			if s.worker_statuses[worker] != .Ok
			{
				return s.worker_statuses[worker];
			}
		}
		return .Ok;
	case .Conserving:
		return .Invalid_Argument;
	}
	unreachable();
}
