package custom_extensions

@(require)
import "core:math"
@(require)
import "core:time"
@(require)
import e "entasis:entasis"
@(require)
import util "entasis:entasis_utilities"
import support "../benchmark_support"

when support.BENCHMARK_COMPONENTS == "all"
{
	JOINT_BREAK_PAIR_COUNT :: 1024;
	Joint_Break_Case :: enum u8
	{
		Disabled,
		Enabled_Empty,
		Sparse,
		Dense,
		Crossing,
	}
	JOINT_BREAK_NAMES: [5]string = {"disabled", "enabled_empty", "sparse", "dense", "crossing"};
	Joint_Break_Benchmark_Owner :: struct
	{
		world: e.World,
		description: e.World_Description,
		bodies: [JOINT_BREAK_PAIR_COUNT*2]e.Body_Handle,
		joints: [JOINT_BREAK_PAIR_COUNT]e.Constraint_Handle,
	}
	Joint_Break_Component :: struct
	{
		step_ms, drain_ms, rebuild_ms: f64,
		watches, events, remaining_constraints: int,
		maximum_force: f32,
		world_pool_bytes, worker_pool_bytes: u64,
	}
	joint_break_body_description :: proc "contextless" (index: int) -> e.Body_Description
	{
		position: e.Vector3 = {f32(index/2)*4+f32(index%2), 0, 0};
		linear: e.Vector3 = {1, 0, 0};
		if index%2 != 0
		{
			linear.x = -1;
		}
		return e.body_shapeless({inverse_mass=1, inverse_inertia_tensor={xx=1, yy=1, zz=1}},
			e.pose(position), e.velocity(linear), e.body_activity(-1, 255));
	}
	joint_break_create_constraints :: proc(owner: ^Joint_Break_Benchmark_Owner, mode: Joint_Break_Case) -> e.Status
	{
		joint: e.Ball_Socket = {local_offset_a={0.5, 0, 0}, local_offset_b={-0.5, 0, 0},
			spring_settings=e.spring_settings(30, 1)};
		for &handle, index in owner.joints
		{
			status: e.Status;
			handle, status = e.constraint_add_2(&owner.world, owner.bodies[index*2], owner.bodies[index*2+1], joint);
			if status != .Ok
			{
				return status;
			}
			if mode == .Dense || mode == .Crossing || (mode == .Sparse && index%16 == 0)
			{
				limits: e.Joint_Break_Limits = {metrics={.Force, .Torque}, force=1e20, torque=1e20, user_id=u64(index+1)};
				if mode == .Crossing
				{
					limits.force = 0;
				}
				if failure: e.Status = e.constraint_set_break_limits(&owner.world, handle, limits); failure != .Ok
				{
					return failure;
				}
			}
		}
		return .Ok;
	}
	joint_break_benchmark_init :: proc(owner: ^Joint_Break_Benchmark_Owner, workers: int, mode: Joint_Break_Case) -> e.Status
	{
		description: e.World_Description = e.world_description_default();
		description.gravity = {};
		description.damping = {};
		description.threading.worker_count = i32(workers);
		description.capacity.bodies = JOINT_BREAK_PAIR_COUNT*2;
		description.capacity.constraints = JOINT_BREAK_PAIR_COUNT;
		description.capacity.initial_constraints_per_type_batch = JOINT_BREAK_PAIR_COUNT;
		description.solve = {velocity_iterations=4, substeps=1, fallback_batch_threshold=64};
		owner.description = description;
		if failure: e.Status = e.world_init(&owner.world, description); failure != .Ok
		{
			return failure;
		}
		if mode != .Disabled
		{
			if failure: e.Status = e.world_enable_joint_breaks(&owner.world, JOINT_BREAK_PAIR_COUNT); failure != .Ok
			{
				return failure;
			}
			if failure: e.Status = e.joint_break_reserve(&owner.world, JOINT_BREAK_PAIR_COUNT); failure != .Ok
			{
				return failure;
			}
		}
		for &body, index in owner.bodies
		{
			status: e.Status;
			body, status = e.body_add(&owner.world, joint_break_body_description(index));
			if status != .Ok
			{
				return status;
			}
		}
		return joint_break_create_constraints(owner, mode);
	}
	joint_break_validate_reaction :: proc(owner: ^Joint_Break_Benchmark_Owner, index: int, reaction: e.Joint_Reaction) -> e.Status
	{
		if reaction.state != .Solved || reaction.sample.body_count != 2 || reaction.sample.substep_duration != DT ||
		!(reaction.maximum_force > 0) || math.is_nan(reaction.maximum_force) || math.is_inf(reaction.maximum_force, 0)
		{
			return .Invalid_Argument;
		}
		for body_index in 0..<2
		{
			body: e.Body_Handle = owner.bodies[index*2+body_index];
			state: e.Body_State;
			status: e.Status;
			state, status = e.body_get(&owner.world, body);
			if status != .Ok
			{
				return status;
			}
			initial: e.Body_Description = joint_break_body_description(index*2+body_index);
			force_x: f32 = (state.velocity.linear.x-initial.velocity.linear.x)/DT;
			if reaction.sample.bodies[body_index] != body ||
			abs(reaction.sample.forces[body_index].linear.x-force_x) > 0.002 ||
			abs(reaction.sample.forces[body_index].linear.y)+abs(reaction.sample.forces[body_index].linear.z) > 0.002
			{
				return .Invalid_Argument;
			}
		}
		return .Ok;
	}
	joint_break_validate_step :: proc(owner: ^Joint_Break_Benchmark_Owner, mode: Joint_Break_Case, events: []e.Joint_Break_Event,
		written, required: int, previous_lifetime: ^u64, result: ^Joint_Break_Component) -> e.Status
	{
		status: e.Status;
		expected_events: int;
		if mode == .Crossing
		{
			expected_events = JOINT_BREAK_PAIR_COUNT;
		}
		if written != expected_events || required != expected_events
		{
			return .Invalid_Argument;
		}
		for event in events[:written]
		{
			if event.user_id == 0 || event.user_id > JOINT_BREAK_PAIR_COUNT || event.lifetime <= previous_lifetime^ ||
			.Force not_in event.exceeded || event.constraint != owner.joints[event.user_id-1]
			{
				return .Invalid_Argument;
			}
			previous_lifetime^ = event.lifetime;
			status = joint_break_validate_reaction(owner, int(event.user_id-1), event.reaction);
			if status != .Ok
			{
				return status;
			}
			result.maximum_force = max(result.maximum_force, event.reaction.maximum_force);
		}
		if mode == .Sparse || mode == .Dense
		{
			stride: int = 1;
			if mode == .Sparse
			{
				stride = 16;
			}
			for index: int = 0; index < JOINT_BREAK_PAIR_COUNT; index += stride
			{
				reaction: e.Joint_Reaction;
				reaction, status = e.constraint_reaction(&owner.world, owner.joints[index]);
				if status != .Ok
				{
					return status;
				}
				status = joint_break_validate_reaction(owner, index, reaction);
				if status != .Ok
				{
					return status;
				}
				result.maximum_force = max(result.maximum_force, reaction.maximum_force);
			}
		}
		stats: e.World_Stats;
		stats, status = e.world_stats(&owner.world);
		if status != .Ok
		{
			return status;
		}
		result.remaining_constraints = JOINT_BREAK_PAIR_COUNT-expected_events;
		if stats.active_bodies != JOINT_BREAK_PAIR_COUNT*2 || stats.sleeping_bodies != 0 ||
		stats.active_constraints != result.remaining_constraints || stats.active_pairs != 0
		{
			return .Invalid_Argument;
		}
		result.events += written;
		return .Ok;
	}

	joint_break_measure_scene :: proc(workers: int, mode: Joint_Break_Case, steps: int) -> Joint_Break_Component
	{
		owner: Joint_Break_Benchmark_Owner;
		support.extension_require(joint_break_benchmark_init(&owner, workers, mode), "joint break create");
		result: Joint_Break_Component;
		if mode == .Sparse
		{
			result.watches = JOINT_BREAK_PAIR_COUNT/16;
		}
		else if mode == .Dense || mode == .Crossing
		{
			result.watches = JOINT_BREAK_PAIR_COUNT;
		}
		events: [JOINT_BREAK_PAIR_COUNT]e.Joint_Break_Event;
		previous_lifetime: u64;
		for step in 0..<steps
		{
			// restore loading outside timing, retaining the ordinary solver's warmstart history
			for body, index in owner.bodies
			{
				support.extension_require(e.body_apply(&owner.world, body, joint_break_body_description(index)), "joint break reset");
			}
			if mode == .Crossing && step != 0
			{
				start: time.Tick = time.tick_now();
				rebuild_status: e.Status = joint_break_create_constraints(&owner, mode);
				result.rebuild_ms += f64(time.tick_diff(start, time.tick_now()))/1e6;
				support.extension_require(rebuild_status, "joint break recreate");
			}
			start: time.Tick = time.tick_now();
			status: e.Status = e.world_step(&owner.world, DT);
			result.step_ms += f64(time.tick_diff(start, time.tick_now()))/1e6;
			support.extension_require(status, "joint break measured step");
			written, required: int;
			if mode != .Disabled
			{
				start = time.tick_now();
				written, required, status = e.constraint_break_events_drain(&owner.world, events[:]);
				result.drain_ms += f64(time.tick_diff(start, time.tick_now()))/1e6;
				support.extension_require(status, "joint break event drain");
			}
			support.extension_require(joint_break_validate_step(&owner, mode, events[:], written, required, &previous_lifetime, &result), "joint break validation");
		}
		pool: ^e.Buffer_Pool;
		status: e.Status;
		pool, status = e.world_borrow_pool(&owner.world);
		support.extension_require(status, "joint break pool");
		result.world_pool_bytes = util.buffer_pool_total_allocated_byte_count(pool);
		dispatcher: ^e.Dispatcher;
		dispatcher, status = e.world_borrow_dispatcher(&owner.world);
		support.extension_require(status, "joint break dispatcher");
		if dispatcher != nil
		{
			for index in 0..<dispatcher.worker_count
			{
				worker: ^util.Buffer_Pool;
				pool_status: util.Threading_Status;
				worker, pool_status = dispatcher.worker_pool(dispatcher, index);
				if pool_status != .Ok
				{
					panic("joint break worker pool");
				}
				result.worker_pool_bytes += util.buffer_pool_total_allocated_byte_count(worker);
			}
		}
		support.extension_require(e.world_destroy(&owner.world), "joint break destroy");
		if !(result.step_ms > 0) || math.is_inf(result.step_ms, 0) || math.is_nan(result.step_ms)
		{
			panic("joint break timing");
		}
		return result;
	}
	measure_joint_break_components :: proc(workers: int) -> [5]Joint_Break_Component
	{
		results: [5]Joint_Break_Component;
		for &result, index in results
		{
			mode: Joint_Break_Case = Joint_Break_Case(index);
			_ = joint_break_measure_scene(workers, mode, WARMUP_STEPS);
			result = joint_break_measure_scene(workers, mode, MEASURED_STEPS);
		}
		return results;
	}
}
