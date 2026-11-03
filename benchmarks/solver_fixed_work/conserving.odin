package solver_fixed_work

import "core:math"
import "core:simd"
import "core:time"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"
import support "../benchmark_support"

PRECISION_NAME :: util.PRECISION_NAME;

when util.PRECISION_MODE == .Double_Coordinates
{
	Conserving_Block :: struct #align(32)
	{
		states: [8]physics.Body_Dynamics,
		global_positions: [8]physics.Global_Position,
	}
}
else
{
	Conserving_Block :: struct #align(32)
	{
		states: [8]physics.Body_Dynamics,
	}
}
Conserving_Input :: struct
{
	initial: Conserving_Block,
	references, mask: util.I32x8,
}
Conserving_Run :: struct
{
	inputs: []Conserving_Input,
	outputs: []Conserving_Block,
	callbacks: physics.Pose_Integrator_Callbacks,
	passes, workers: int,
}

conserving_worker :: proc "contextless" (index: int, dispatcher: ^util.Thread_Dispatcher_Boundary)
{
	run: ^Conserving_Run = (^Conserving_Run)(dispatcher.unmanaged_context);
	first: int = len(run.inputs)*index/run.workers;
	end: int = len(run.inputs)*(index+1)/run.workers;
	set: physics.Body_Set = {count=8};
	bodies: physics.Bodies = {sets={memory=&set, length=1}};
	integrator: physics.Pose_Integrator = {callbacks=run.callbacks, bodies=&bodies};
	solver: physics.Solver = {bodies=&bodies, integrator=&integrator};
	for pass in 0 ..< run.passes
	{
		for i in first ..< end
		{
			input: ^Conserving_Input = &run.inputs[i];
			output: ^Conserving_Block = &run.outputs[i];
			output^ = input.initial;
			output.states[0].motion.velocity.linear.x += f32(pass&7)*0.00001;
			set.dynamics_state = {memory=&output.states[0], length=8};
			when util.PRECISION_MODE == .Double_Coordinates
			{
				set.global_positions = {memory=&output.global_positions[0], length=8};
			}
			for substep in 0 ..< 4
			{
				position: util.Vector3_Wide;
				orientation: util.Quaternion_Wide;
				velocity: physics.Body_Velocity_Wide;
				local: physics.Body_Inertia_Wide;
				position, orientation, velocity, local = physics.bodies_gather_active_trusted(
					&bodies, input.references, .Local, physics.BODY_ACCESS_ALL,
				);
				phase: physics.Solver_Substep_Phase = .First if substep == 0 else .Continuation;
				_ = physics.solver_integrate_constraint_body(&solver, input.references, input.mask,
					1.0 / 240.0, phase, index, &position, &orientation, &velocity, local);
			}
		}
	}
}

conserving_measure :: proc(o: Options) -> (f64, u64, int, support.Admission)
{
	inputs: []Conserving_Input = make([]Conserving_Input, o.bundles);
	defer delete(inputs);
	outputs: []Conserving_Block = make([]Conserving_Block, o.bundles);
	defer delete(outputs);
	for &input, i in inputs
	{
		pattern: u8 = 255;
		if o.layout == "partial"
		{
			pattern = u8(i%254+1);
		}
		else if o.layout == "sparse"
		{
			pattern = u8(1)<<uint(i%8);
		}
		for lane in 0 ..< 8
		{
			input.references = simd.replace(input.references, lane, i32(lane));
			if pattern & (u8(1)<<uint(lane)) != 0
			{
				input.mask = simd.replace(input.mask, lane, i32(-1));
			}
			body: ^physics.Body_Dynamics = &input.initial.states[lane];
			body.motion.pose.position = {f32(lane)*0.17, 2+f32(i%3)*0.21, -0.13};
			body.motion.pose.orientation = util.quaternion_normalize(util.Quaternion{f32(lane-3)*0.11, 0.21, -0.41, 1});
			body.motion.velocity = {linear={0.13, -0.17, f32(lane)*0.01}, angular={0.21+f32(i%7)*0.31, 0.47, -0.15}};
			body.inertia.local = {inverse_mass=0.5, inverse_inertia_tensor={xx=0.7, yx=0.09, yy=1.1, zx=-0.13, zy=0.21, zz=1.9}};
			if o.layout == "isotropic"
			{
				body.inertia.local.inverse_inertia_tensor = {xx=1, yy=1, zz=1};
			}
			physics.pose_integrator_update_world_inertia(body);
			when util.PRECISION_MODE == .Double_Coordinates
			{
				input.initial.global_positions[lane] = {f64(body.motion.pose.position.x), f64(body.motion.pose.position.y), f64(body.motion.pose.position.z)};
			}
		}
	}
	admitted_units: int = conserving_admitted_units(inputs);
	gravity: physics.Default_Pose_Integrator_Context = {gravity={0, -9.81, 0}, linear_damping=0.02, angular_damping=0.03};
	if physics.pose_integrator_default_prepare(&gravity, 1.0 / 240.0) != .Ok
	{
		return 0, 0, 0, .Invalid;
	}
	callbacks: physics.Pose_Integrator_Callbacks = physics.pose_integrator_default_callbacks(&gravity);
	callbacks.angular_mode = .Conserve_Momentum if o.angular == "momentum" else .Conserve_Momentum_With_Gyroscopic_Torque;
	run: Conserving_Run = {inputs=inputs, outputs=outputs, callbacks=callbacks, passes=o.warmup, workers=o.workers};
	dispatcher: util.Thread_Dispatcher;
	if util.thread_dispatcher_initialize(&dispatcher, o.workers) != .Ok
	{
		return 0, 0, 0, .Invalid;
	}
	defer util.thread_dispatcher_shutdown(&dispatcher);
	boundary: ^util.Thread_Dispatcher_Boundary = util.thread_dispatcher_boundary(&dispatcher);
	if boundary.dispatch(boundary, conserving_worker, o.workers, &run) != .Ok
	{
		return 0, 0, 0, .Invalid;
	}
	run.passes = o.passes;
	start: time.Tick = time.tick_now();
	status: util.Threading_Status = boundary.dispatch(boundary, conserving_worker, o.workers, &run);
	elapsed: f64 = f64(time.tick_diff(start, time.tick_now()))/1e6;
	if status != .Ok
	{
		return 0, 0, 0, .Invalid;
	}
	hash: u64 = 14695981039346656037;
	for block in outputs
	{
		for body in block.states
		{
			values: [20]f32 = {body.motion.pose.position.x, body.motion.pose.position.y, body.motion.pose.position.z,
				body.motion.pose.orientation.x, body.motion.pose.orientation.y, body.motion.pose.orientation.z, body.motion.pose.orientation.w,
				body.motion.velocity.linear.x, body.motion.velocity.linear.y, body.motion.velocity.linear.z,
				body.motion.velocity.angular.x, body.motion.velocity.angular.y, body.motion.velocity.angular.z,
				body.inertia.world.inverse_mass, body.inertia.world.inverse_inertia_tensor.xx, body.inertia.world.inverse_inertia_tensor.yx,
				body.inertia.world.inverse_inertia_tensor.yy, body.inertia.world.inverse_inertia_tensor.zx,
				body.inertia.world.inverse_inertia_tensor.zy, body.inertia.world.inverse_inertia_tensor.zz};
			for value in values
			{
				if math.is_nan(value) || math.is_inf(value)
				{
					return 0, 0, 0, .Invalid;
				}
				hash = (hash ~ u64(transmute(u32)value))*1099511628211;
			}
		}
	}
	return elapsed, hash, admitted_units, .Ok;
}

conserving_admitted_units :: proc(inputs: []Conserving_Input) -> int
{
	count: int;
	for input in inputs
	{
		for lane in 0 ..< 8
		{
			count += int(simd.extract(input.mask, lane) != 0);
		}
	}
	return count;
}
