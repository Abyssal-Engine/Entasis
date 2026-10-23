package custom_extensions

@(require)
import "core:fmt"
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
	GRAVITY_BODY_COUNT :: 2051;
	// reserve the rounded handle span consumed by sleeper scratch preparation
	GRAVITY_BODY_CAPACITY :: 4096;
	GRAVITY_MEASURED_STEPS :: 256;
	GRAVITY_SUBSTEPS :: 4;
	Gravity_Case :: enum u8
	{
		Planetary,
		Planetary_Control,
		Per_Body,
		Per_Body_Control,
		Uniform_Control,
		Counted_Uniform,
		Global_Near,
		Global_Far,
	}
	when e.PRECISION_MODE == .Double_Coordinates
	{
		Gravity_Global_Policy :: e.Global_Planetary_Gravity_Policy;
		GRAVITY_NATIVE_COUNT :: 8;
		GRAVITY_NAMES: [8]string = {"planetary", "planetary_control", "per_body", "per_body_control", "uniform_control", "counted_uniform", "global_near", "global_far"};
	}
	else
	{
		Gravity_Global_Policy :: struct
		{
		}
		GRAVITY_NATIVE_COUNT :: 6;
		GRAVITY_NAMES: [6]string = {"planetary", "planetary_control", "per_body", "per_body_control", "uniform_control", "counted_uniform"};
	}
	Gravity_Result :: struct
	{
		step_ms, position_checksum, velocity_checksum: f64,
		active_bodies, steps: int,
		world_pool_bytes, worker_pool_bytes: u64,
		visits, tails: u64,
	}
	Gravity_Owner :: struct
	{
		world: e.World,
		bodies: [GRAVITY_BODY_COUNT]e.Body_Handle,
		gravities: e.Body_Property_Table(e.Vector3),
		planetary: e.Planetary_Gravity_Policy,
		per_body: e.Per_Body_Gravity_Policy,
		uniform: e.Uniform_Gravity_Policy,
		counted: support.Counted_Gravity,
		global_planetary: Gravity_Global_Policy,
	}

	gravity_acceleration :: proc "contextless" (index: int) -> f32
	{
		return -f32(index%7+1)/16;
	}

	gravity_control_damping :: proc "contextless" () -> e.Body_Damping
	{
		return {mode=.Override,
			linear=f32(-math.ln(f64(f32(1)-f32(0.02)))),
			angular=f32(-math.ln(f64(f32(1)-f32(0.01))))};
	}

	gravity_description :: proc (workers: int) -> e.World_Description
	{
		description: e.World_Description = e.world_description_default();
		description.gravity = {};
		description.damping = {};
		description.capacity.bodies = GRAVITY_BODY_CAPACITY;
		// runtime contact preparation admits a handle slot for every reserved pair
		description.capacity.constraints = max(description.capacity.constraints, description.capacity.pairs);
		description.threading.worker_count = i32(workers);
		description.solve = {velocity_iterations=1, substeps=GRAVITY_SUBSTEPS, fallback_batch_threshold=64};
		return description;
	}

	gravity_pool_bytes :: proc(world: ^e.World) -> (u64, u64)
	{
		pool: ^e.Buffer_Pool;
		status: e.Status;
		pool, status = e.world_borrow_pool(world);
		support.extension_require(status, "gravity pool");
		world_bytes: u64 = util.buffer_pool_total_allocated_byte_count(pool);
		worker_bytes: u64;
		dispatcher: ^e.Dispatcher;
		dispatcher, status = e.world_borrow_dispatcher(world);
		support.extension_require(status, "gravity dispatcher");
		if dispatcher != nil
		{
			for index in 0..<dispatcher.worker_count
			{
				worker: ^util.Buffer_Pool;
				worker_status: util.Threading_Status;
				worker, worker_status = dispatcher.worker_pool(dispatcher, index);
				if worker_status != .Ok
				{
					panic("gravity worker pool");
				}
				worker_bytes += util.buffer_pool_total_allocated_byte_count(worker);
			}
		}
		return world_bytes, worker_bytes;
	}

	gravity_owner_init :: proc(owner: ^Gravity_Owner, workers: int, mode: Gravity_Case) -> e.Status
	{
		description: e.World_Description = gravity_description(workers);
		if mode == .Per_Body || mode == .Per_Body_Control
		{
			if status: e.Status = e.body_property_init(&owner.gravities, GRAVITY_BODY_COUNT); status != .Ok
			{
				return status;
			}
			owner.per_body = e.per_body_gravity_policy(&owner.gravities, 0.02, 0.01);
			description.pose_callbacks = e.pose_policy_per_body(&owner.per_body);
		}
		else if mode == .Counted_Uniform
		{
			owner.counted.gravity = gravity_acceleration(0);
			owner.counted.retention = .Half;
			description.pose_callbacks = {
				initialize=support.counted_gravity_initialize,
				prepare_for_integration=support.counted_gravity_prepare,
				integrate_velocity=support.counted_gravity_velocity,
				dispose=support.counted_gravity_dispose, user_context=&owner.counted,
			};
		}
		else if mode == .Uniform_Control
		{
			owner.uniform = e.uniform_gravity_policy({gravity_acceleration(0), 0, 0}, 0.02, 0.01);
			description.pose_callbacks = e.pose_policy_uniform(&owner.uniform);
		}
		else
		{
			owner.planetary = e.planetary_gravity_policy({0.125, 0.25, -0.125}, 1, 0.02, 0.01);
			description.pose_callbacks = e.pose_policy_planetary(&owner.planetary);
		}
		when e.PRECISION_MODE == .Double_Coordinates
		{
			if mode == .Global_Near || mode == .Global_Far
			{
				offset: f64 = 1e9 if mode == .Global_Far else 0;
				owner.global_planetary = e.planetary_gravity_policy_global({offset+0.125, -offset+0.25, offset-0.125}, 1, 0.02, 0.01);
				description.pose_callbacks = e.pose_policy_planetary_global(&owner.global_planetary);
			}
		}
		description.pose_callbacks.allow_substeps_for_unconstrained = .Enabled;
		if status: e.Status = e.world_init(&owner.world, description); status != .Ok
		{
			return status;
		}
		if mode == .Planetary_Control || mode == .Per_Body_Control || mode == .Uniform_Control
		{
			if status: e.Status = e.world_enable_body_control(&owner.world, {1, 1}); status != .Ok
			{
				return status;
			}
		}
		for &body, index in owner.bodies
		{
			position: e.Vector3 = {f32(2.126953125+f64(index)/8), 0.25, -0.125};
			body_description: e.Body_Description = e.body_shapeless(
				{inverse_mass=1, inverse_inertia_tensor={xx=1, yy=1, zz=1}},
				e.pose(position), e.velocity({}, {0.25, 0, 0}), e.body_activity(-1, 255));
			status: e.Status;
			when e.PRECISION_MODE == .Double_Coordinates
			{
				offset: f64 = 1e9 if mode == .Global_Far else 0;
				body, status = e.body_add_global(&owner.world, body_description, {offset+f64(position.x), -offset+f64(position.y), offset+f64(position.z)});
			}
			else
			{
				body, status = e.body_add(&owner.world, body_description);
			}
			if status != .Ok
			{
				return status;
			}
			if mode == .Per_Body || mode == .Per_Body_Control
			{
				if status = e.body_property_set(&owner.gravities, body, e.Vector3{gravity_acceleration(index), 0, 0}); status != .Ok
				{
					return status;
				}
			}
			if index == 0 && (mode == .Planetary_Control || mode == .Per_Body_Control || mode == .Uniform_Control)
			{
				if status = e.body_set_damping(&owner.world, body, gravity_control_damping()); status != .Ok
				{
					return status;
				}
			}
		}
		return .Ok;
	}

	// scalar radial acceleration and four updates are independent of the wide policies
	gravity_oracle :: proc "contextless" (mode: Gravity_Case, index, steps: int, offset: f64 = 0) -> (f64, f32, f32)
	{
		position: f64 = offset+2.126953125+f64(index)/8;
		velocity: f32;
		angular: f32 = 0.25;
		h: f32 = DT/GRAVITY_SUBSTEPS;
		linear_factor: f32 = f32(math.pow(f64(f32(1)-f32(0.02)), f64(h)));
		angular_factor: f32 = f32(math.pow(f64(f32(1)-f32(0.01)), f64(h)));
		if index == 0 && (mode == .Planetary_Control || mode == .Per_Body_Control || mode == .Uniform_Control)
		{
			damping: e.Body_Damping = gravity_control_damping();
			linear_factor = f32(math.exp(-f64(damping.linear)*f64(h)));
			angular_factor = f32(math.exp(-f64(damping.angular)*f64(h)));
		}
		for _ in 0..<steps*GRAVITY_SUBSTEPS
		{
			delta: f32;
			if mode == .Uniform_Control
			{
				delta = gravity_acceleration(0)*h;
			}
			else if mode == .Per_Body || mode == .Per_Body_Control
			{
				delta = gravity_acceleration(index)*h;
			}
			else if mode == .Global_Near || mode == .Global_Far
			{
				radius: f64 = position-(offset+0.125);
				delta = f32(-f64(h)*radius/max(f64(1), abs(radius)*radius*radius));
			}
			else
			{
				radius: f32 = f32(position)-0.125;
				squared: f32 = radius*radius;
				scale: f32 = -h/max(f32(1), squared*f32(math.sqrt(f64(squared))));
				delta = radius*scale;
			}
			velocity = (velocity+delta)*linear_factor;
			angular *= angular_factor;
			displacement: f32 = velocity*h;
			when e.PRECISION_MODE == .Double_Coordinates
			{
				position += f64(displacement);
			}
			else
			{
				position = f64(f32(position)+displacement);
			}
		}
		return position-offset, velocity, angular;
	}

	gravity_validate :: proc(owner: ^Gravity_Owner, mode: Gravity_Case, steps: int) -> Gravity_Result
	{
		stats: e.World_Stats;
		status: e.Status;
		stats, status = e.world_stats(&owner.world);
		support.extension_require(status, "gravity stats");
		if stats.active_bodies != GRAVITY_BODY_COUNT || stats.sleeping_bodies != 0 || stats.active_pairs != 0 || stats.active_constraints != 0 || stats.step_index != u64(steps)
		{
			panic("gravity population");
		}
		result: Gravity_Result = {active_bodies=stats.active_bodies, steps=steps};
		for body, index in owner.bodies
		{
			state: e.Body_State;
			state, status = e.body_get(&owner.world, body);
			support.extension_require(status, "gravity body");
			offset: f64 = 1e9 if mode == .Global_Far else 0;
			position: f64 = f64(state.pose.position.x);
			position_y: f64 = f64(state.pose.position.y);
			position_z: f64 = f64(state.pose.position.z);
			when e.PRECISION_MODE == .Double_Coordinates
			{
				pose: e.Global_Pose;
				pose, status = e.body_get_global_pose(&owner.world, body);
				support.extension_require(status, "gravity global body");
				position = pose.position.x-offset;
				position_y = pose.position.y;
				position_z = pose.position.z;
			}
			if mode == .Counted_Uniform
			{
				// prediction invokes the callback on scratch once; four solve updates commit
				expected_y: f64 = 0.25;
				expected_v: f32;
				h: f32 = DT/GRAVITY_SUBSTEPS;
				for _ in 0..<steps*GRAVITY_SUBSTEPS
				{
					expected_v = (expected_v+gravity_acceleration(0)*h)*0.5;
					displacement: f32 = expected_v*h;
					when e.PRECISION_MODE == .Double_Coordinates
					{
						expected_y += f64(displacement);
					}
					else
					{
						expected_y = f64(f32(expected_y)+displacement);
					}
				}
				if !(position == 2.126953125+f64(index)/8 && abs(position_y-expected_y) <= 0.001 &&
					position_z == -0.125 && state.velocity.linear.x == 0 && abs(state.velocity.linear.y-expected_v) <= 0.0001 &&
					state.velocity.linear.z == 0 && state.velocity.angular.x == 0.25 && state.velocity.angular.y == 0 && state.velocity.angular.z == 0 &&
					abs(state.pose.orientation.x) <= math.F32_MAX && abs(state.pose.orientation.y) <= math.F32_MAX &&
					abs(state.pose.orientation.z) <= math.F32_MAX && abs(state.pose.orientation.w) <= math.F32_MAX &&
					owner.counted.counts[index] == i32(steps*(GRAVITY_SUBSTEPS+1)))
				{
					panic("counted gravity scalar oracle");
				}
				result.position_checksum += position_y;
				result.velocity_checksum += f64(state.velocity.linear.y);
				continue;
			}
			expected_position: f64;
			expected_velocity, expected_angular: f32;
			expected_position, expected_velocity, expected_angular = gravity_oracle(mode, index, steps, 1e9 if mode == .Global_Far else 0);
			if !(abs(position-expected_position) <= 0.001 && abs(state.velocity.linear.x-expected_velocity) <= 0.0001 &&
				abs(state.velocity.angular.x-expected_angular) <= 0.0001 &&
				position_y == -offset+0.25 && position_z == offset-0.125 &&
				state.velocity.linear.y == 0 && state.velocity.linear.z == 0 && state.velocity.angular.y == 0 && state.velocity.angular.z == 0 &&
				abs(state.pose.orientation.x) <= math.F32_MAX && abs(state.pose.orientation.y) <= math.F32_MAX &&
				abs(state.pose.orientation.z) <= math.F32_MAX && abs(state.pose.orientation.w) <= math.F32_MAX)
			{
				fmt.eprintfln("gravity_oracle_failed mode=%v body=%d position=%v expected_position=%v velocity=%v expected_velocity=%v angular=%v expected_angular=%v", mode, index, position, expected_position, state.velocity.linear.x, expected_velocity, state.velocity.angular.x, expected_angular);
				panic("gravity scalar oracle");
			}
			result.position_checksum += position;
			result.velocity_checksum += f64(state.velocity.linear.x);
		}
		result.world_pool_bytes, result.worker_pool_bytes = gravity_pool_bytes(&owner.world);
		return result;
	}

	gravity_measure_scene :: proc(workers: int, mode: Gravity_Case, steps: int) -> Gravity_Result
	{
		owner: Gravity_Owner;
		support.extension_require(gravity_owner_init(&owner, workers, mode), "gravity create");
		world_before, workers_before: u64;
		world_before, workers_before = gravity_pool_bytes(&owner.world);
		statuses: [GRAVITY_MEASURED_STEPS]e.Status;
		start: time.Tick = time.tick_now();
		for &status in statuses[:steps]
		{
			status = e.world_step(&owner.world, DT);
		}
		elapsed: f64 = f64(time.tick_diff(start, time.tick_now()))/1e6;
		for status in statuses[:steps]
		{
			support.extension_require(status, "gravity step");
		}
		result: Gravity_Result = gravity_validate(&owner, mode, steps);
		result.step_ms = elapsed;
		if result.world_pool_bytes != world_before || result.worker_pool_bytes != workers_before || !(elapsed > 0 && elapsed < math.F64_MAX)
		{
			fmt.eprintfln("gravity_capacity_failed mode=%v workers=%d steps=%d world_before=%d world_after=%d workers_before=%d workers_after=%d elapsed=%v", mode, workers, steps, world_before, result.world_pool_bytes, workers_before, result.worker_pool_bytes, elapsed);
			panic("gravity steady capacity or timing");
		}
		support.extension_require(e.world_destroy(&owner.world), "gravity destroy");
		if mode == .Per_Body || mode == .Per_Body_Control
		{
			support.extension_require(e.body_property_destroy(&owner.gravities), "gravity properties destroy");
		}
		return result;
	}

	measure_gravity_components :: proc(workers: int) -> [GRAVITY_NATIVE_COUNT]Gravity_Result
	{
		results: [GRAVITY_NATIVE_COUNT]Gravity_Result;
		for &result, index in results
		{
			mode: Gravity_Case = Gravity_Case(index);
			_ = gravity_measure_scene(workers, mode, WARMUP_STEPS);
			result = gravity_measure_scene(workers, mode, GRAVITY_MEASURED_STEPS);
		}
		when e.PRECISION_MODE == .Double_Coordinates
		{
			near: Gravity_Result = results[int(Gravity_Case.Global_Near)];
			far: Gravity_Result = results[int(Gravity_Case.Global_Far)];
			if abs(near.position_checksum-far.position_checksum) > GRAVITY_BODY_COUNT*0.0002 ||
				abs(near.velocity_checksum-far.velocity_checksum) > GRAVITY_BODY_COUNT*0.0001
			{
				panic("gravity near far oracle");
			}
		}
		return results;
	}
}
