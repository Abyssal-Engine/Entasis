package custom_extensions

@(require)
import "base:intrinsics"
@(require)
import "base:runtime"
@(require)
import "core:fmt"
@(require)
import "core:math"
@(require)
import "core:mem"
@(require)
import "core:sync"
@(require)
import "core:time"
@(require)
import c "entasis:entasis_c"
@(require)
import e "entasis:entasis"
import support "../benchmark_support"

when support.BENCHMARK_COMPONENTS == "all"
{
	when e.PRECISION_MODE == .Double_Coordinates
	{
		C_GRAVITY_COUNT :: 2;
		C_GRAVITY_NAMES: [2]string = {"per_body", "global_callback"};
	}
	else
	{
		C_GRAVITY_COUNT :: 1;
		C_GRAVITY_NAMES: [1]string = {"per_body"};
	}
	C_Gravity_Case :: enum u8
	{
		Per_Body,
		Global_Callback,
	}

	C_Gravity_Allocation :: struct
	{
		lock: sync.Mutex,
		requests, live_bytes: u64,
	}
	// callback writers keep their observation counters on separate cache lines
	C_Gravity_Worker :: struct #align(64)
	{
		visits, tails, precise_visits: u64,
		status: e.Status,
	}
	C_Gravity_Owner :: struct
	{
		world: c.Entasis_World,
		pool: c.Entasis_Buffer_Pool,
		thread_pool: c.Entasis_Thread_Pool,
		dispatcher: c.Entasis_Dispatcher_Interface,
		world_allocation, worker_allocation: C_Gravity_Allocation,
		bodies: [GRAVITY_BODY_COUNT]c.Entasis_Body_Handle,
		gravities: [GRAVITY_BODY_COUNT]c.Entasis_Vector3,
		callback_workers: []C_Gravity_Worker,
		linear_factor, angular_factor: f32,
		initialized, disposed: int,
	}

	c_gravity_allocate :: proc "c" (user: rawptr, size, alignment: u64) -> rawptr
	{
		context = runtime.default_context();
		counter: ^C_Gravity_Allocation = (^C_Gravity_Allocation)(user);
		sync.mutex_lock(&counter.lock);
		defer sync.mutex_unlock(&counter.lock);
		counter.requests += 1;
		memory: rawptr;
		error: mem.Allocator_Error;
		memory, error = mem.alloc(int(size), int(alignment), runtime.heap_allocator());
		if error != nil
		{
			return nil;
		}
		counter.live_bytes += size;
		return memory;
	}

	c_gravity_deallocate :: proc "c" (user: rawptr, memory: rawptr, size, alignment: u64)
	{
		context = runtime.default_context();
		counter: ^C_Gravity_Allocation = (^C_Gravity_Allocation)(user);
		sync.mutex_lock(&counter.lock);
		defer sync.mutex_unlock(&counter.lock);
		allocator: mem.Allocator = runtime.heap_allocator();
		_, error := allocator.procedure(allocator.data, .Free, 0, int(alignment), memory, int(size));
		if error != nil
		{
			panic("C gravity deallocation");
		}
		counter.live_bytes -= size;
	}

	c_gravity_reallocate :: proc "c" (user: rawptr, memory: rawptr, old_size, size, alignment: u64) -> rawptr
	{
		context = runtime.default_context();
		next: rawptr = c_gravity_allocate(user, size, alignment);
		if next == nil
		{
			return nil;
		}
		if memory != nil
		{
			intrinsics.mem_copy_non_overlapping(next, memory, int(min(old_size, size)));
			c_gravity_deallocate(user, memory, old_size, alignment);
		}
		return next;
	}

	c_gravity_allocator :: proc "contextless" (counter: ^C_Gravity_Allocation) -> c.Entasis_Allocator
	{
		return {struct_size=u32(size_of(c.Entasis_Allocator)), struct_version=c.ENTASIS_STRUCT_VERSION,
			user_context=counter, allocate=c_gravity_allocate, reallocate=c_gravity_reallocate, deallocate=c_gravity_deallocate};
	}

	when e.PRECISION_MODE == .Double_Coordinates
	{
		c_gravity_initialize :: proc "c" (user: rawptr, world: c.Entasis_World) -> c.Entasis_Status
		{
			owner: ^C_Gravity_Owner = (^C_Gravity_Owner)(user);
			owner.initialized += 1;
			return c.Entasis_Status(e.Status.Ok);
		}
		c_gravity_prepare :: proc "c" (user: rawptr, dt: f32) -> c.Entasis_Status
		{
			context = runtime.default_context();
			owner: ^C_Gravity_Owner = (^C_Gravity_Owner)(user);
			owner.linear_factor = f32(math.pow(f64(f32(1)-f32(0.02)), f64(dt)));
			owner.angular_factor = f32(math.pow(f64(f32(1)-f32(0.01)), f64(dt)));
			return c.Entasis_Status(e.Status.Ok);
		}
		c_gravity_dispose :: proc "c" (user: rawptr)
		{
			owner: ^C_Gravity_Owner = (^C_Gravity_Owner)(user);
			owner.disposed += 1;
		}
		c_gravity_integrate :: proc "c" (user: rawptr, view: ^c.Entasis_Global_Pose_Integration_View)
		{
			context = runtime.default_context();
			owner: ^C_Gravity_Owner = (^C_Gravity_Owner)(user);
			worker: ^C_Gravity_Worker = &owner.callback_workers[view.worker_index];
			live: int;
			for lane in 0..<8
			{
				if view.active_mask.lanes[lane] == 0
				{
					continue;
				}
				live += 1;
				worker.visits += 1;
				position: c.Entasis_Global_Position = view.position[lane];
				if !(abs(position.x-1e9) < 300 && position.y == -1e9+0.25 && position.z == 1e9-0.125)
				{
					worker.status = .Invalid_Argument;
				}
				if position.x != f64(f32(position.x))
				{
					worker.precise_visits += 1;
				}
				h: f32 = view.dt.lanes[lane];
				index: int = int(view.body_handles[lane].value);
				view.velocity.linear.x.lanes[lane] = (view.velocity.linear.x.lanes[lane]+gravity_acceleration(index)*h)*owner.linear_factor;
				view.velocity.angular.x.lanes[lane] *= owner.angular_factor;
			}
			if live > 0 && live < 8
			{
				worker.tails += 1;
			}
		}
	}

	c_gravity_owner_init :: proc(owner: ^C_Gravity_Owner, workers: int, mode: C_Gravity_Case) -> c.Entasis_Status
	{
		owner.callback_workers = make([]C_Gravity_Worker, workers);
		world_allocator: c.Entasis_Allocator = c_gravity_allocator(&owner.world_allocation);
		worker_allocator: c.Entasis_Allocator = c_gravity_allocator(&owner.worker_allocation);
		status: c.Entasis_Status = c.export_entasis_buffer_pool_init_extended(&owner.pool, 131072, 16, &world_allocator, 1, nil);
		if status != c.Entasis_Status(e.Status.Ok)
		{
			return status;
		}
		if c.export_entasis_thread_pool_init_extended(&owner.thread_pool, u32(workers), 65536, &worker_allocator, 1, nil) != 0 ||
			c.export_entasis_dispatcher_from_thread_pool(&owner.thread_pool, &owner.dispatcher) != 0
		{
			return c.Entasis_Status(e.Status.Invalid_Argument);
		}
		description: c.Entasis_World_Description = c.export_entasis_world_description_default();
		description.gravity = {};
		description.damping = {};
		description.capacity.bodies = GRAVITY_BODY_CAPACITY;
		description.capacity.constraints = max(description.capacity.constraints, description.capacity.pairs);
		description.solve.velocity_iterations = 1;
		description.solve.substeps = GRAVITY_SUBSTEPS;
		description.solve.fallback_batch_threshold = 64;
		description.threading.worker_count = u32(workers);
		description.threading.external_dispatcher = &owner.dispatcher;
		description.allocator = world_allocator;
		extensions: c.Entasis_World_Extensions = {struct_size=u32(size_of(c.Entasis_World_Extensions)), struct_version=c.ENTASIS_STRUCT_VERSION, allocation_scope=1};
		if mode == .Per_Body
		{
			policy: c.Entasis_Per_Body_Gravity_Policy = c.export_entasis_per_body_gravity_policy(raw_data(owner.gravities[:]), GRAVITY_BODY_COUNT, 0.02, 0.01);
			description.pose_policy = c.export_entasis_pose_policy_per_body(&policy);
			description.pose_policy.allow_substeps_for_unconstrained = 1;
			description.use_pose_policy = 1;
			status = c.export_entasis_world_init_with_pool_extended(&owner.world, &description, &owner.pool, &extensions, nil);
		}
		else
		{
			when e.PRECISION_MODE == .Double_Coordinates
			{
				description.use_pose_policy = 0;
				callbacks: c.Entasis_Global_Velocity_Callbacks = {
					struct_size=u32(size_of(c.Entasis_Global_Velocity_Callbacks)), struct_version=c.ENTASIS_STRUCT_VERSION,
					user_context=owner, allow_substeps_for_unconstrained=1,
					initialize=c_gravity_initialize, prepare=c_gravity_prepare, integrate=c_gravity_integrate, dispose=c_gravity_dispose,
				};
				status = c.export_entasis_world_init_global(&owner.world, &description, &extensions, &owner.pool, &callbacks, nil);
			}
		}
		if status != c.Entasis_Status(e.Status.Ok)
		{
			return status;
		}
		for &body, index in owner.bodies
		{
			pose: c.Entasis_Rigid_Pose = {orientation={w=1}, position={f32(2.126953125+f64(index)/8), 0.25, -0.125}};
			description: c.Entasis_Body_Description = c.export_entasis_body_shapeless(
				{inverse_mass=1, inverse_inertia_tensor={xx=1, yy=1, zz=1}}, pose,
				{angular={0.25, 0, 0}}, c.export_entasis_body_activity(-1, 255));
			when e.PRECISION_MODE == .Double_Coordinates
			{
				offset: f64 = 1e9 if mode == .Global_Callback else 0;
				position: c.Entasis_Global_Position = {offset+f64(pose.position.x), -offset+f64(pose.position.y), offset+f64(pose.position.z)};
				status = c.export_entasis_body_add_global(&owner.world, &description, &position, &body, nil);
			}
			else
			{
				status = c.export_entasis_body_add(&owner.world, &description, &body, nil);
			}
			if status != c.Entasis_Status(e.Status.Ok)
			{
				return status;
			}
			owner.gravities[body.value] = {gravity_acceleration(index), 0, 0};
		}
		return c.Entasis_Status(e.Status.Ok);
	}

	c_gravity_validate :: proc(owner: ^C_Gravity_Owner, mode: C_Gravity_Case, steps: int) -> Gravity_Result
	{
		stats: c.Entasis_World_Stats;
		support.extension_require(e.Status(c.export_entasis_world_stats(&owner.world, &stats, nil)), "C gravity stats");
		if stats.active_bodies != GRAVITY_BODY_COUNT || stats.sleeping_bodies != 0 || stats.active_pairs != 0 || stats.active_constraints != 0 || stats.step_index != u64(steps)
		{
			panic("C gravity population");
		}
		result: Gravity_Result = {active_bodies=int(stats.active_bodies), steps=steps};
		for body, index in owner.bodies
		{
			state: c.Entasis_Body_State;
			support.extension_require(e.Status(c.export_entasis_body_get(&owner.world, body, &state, nil)), "C gravity body");
			position: f64 = f64(state.pose.position.x);
			position_y: f64 = f64(state.pose.position.y);
			position_z: f64 = f64(state.pose.position.z);
			offset: f64 = 1e9 if mode == .Global_Callback else 0;
			when e.PRECISION_MODE == .Double_Coordinates
			{
				pose: c.Entasis_Global_Pose;
				support.extension_require(e.Status(c.export_entasis_body_get_global_pose(&owner.world, body, &pose, nil)), "C gravity high body");
				position = pose.position.x-offset;
				position_y = pose.position.y;
				position_z = pose.position.z;
			}
			expected_position: f64;
			expected_velocity, expected_angular: f32;
			expected_position, expected_velocity, expected_angular = gravity_oracle(.Per_Body, index, steps, offset);
			if !(abs(position-expected_position) <= 0.001 && abs(state.velocity.linear.x-expected_velocity) <= 0.0001 &&
				abs(state.velocity.angular.x-expected_angular) <= 0.0001 &&
				position_y == -offset+0.25 && position_z == offset-0.125 &&
				state.velocity.linear.y == 0 && state.velocity.linear.z == 0 && state.velocity.angular.y == 0 && state.velocity.angular.z == 0 &&
				abs(state.pose.orientation.x) <= math.F32_MAX && abs(state.pose.orientation.y) <= math.F32_MAX &&
				abs(state.pose.orientation.z) <= math.F32_MAX && abs(state.pose.orientation.w) <= math.F32_MAX)
			{
				fmt.eprintfln("c_gravity_oracle_failed mode=%v body=%d position=%v expected_position=%v velocity=%v expected_velocity=%v angular=%v expected_angular=%v", mode, index, position, expected_position, state.velocity.linear.x, expected_velocity, state.velocity.angular.x, expected_angular);
				panic("C gravity scalar oracle");
			}
			result.position_checksum += position;
			result.velocity_checksum += f64(state.velocity.linear.x);
		}
		if mode == .Global_Callback
		{
			precise: u64;
			for worker in owner.callback_workers
			{
				support.extension_require(worker.status, "C global callback delivery");
				result.visits += worker.visits;
				result.tails += worker.tails;
				precise += worker.precise_visits;
			}
			if owner.initialized != 1 || result.visits < u64(steps*GRAVITY_SUBSTEPS*GRAVITY_BODY_COUNT) || result.tails == 0 || precise == 0
			{
				panic("C global callback masks or lifecycle");
			}
		}
		result.world_pool_bytes = owner.world_allocation.live_bytes;
		result.worker_pool_bytes = owner.worker_allocation.live_bytes;
		return result;
	}

	c_gravity_measure_scene :: proc(workers: int, mode: C_Gravity_Case, steps: int) -> Gravity_Result
	{
		owner: C_Gravity_Owner;
		support.extension_require(e.Status(c_gravity_owner_init(&owner, workers, mode)), "C gravity create");
		world_requests: u64 = owner.world_allocation.requests;
		worker_requests: u64 = owner.worker_allocation.requests;
		world_before: u64 = owner.world_allocation.live_bytes;
		workers_before: u64 = owner.worker_allocation.live_bytes;
		statuses: [GRAVITY_MEASURED_STEPS]e.Status;
		start: time.Tick = time.tick_now();
		for &status in statuses[:steps]
		{
			status = e.Status(c.export_entasis_world_step(&owner.world, DT, nil));
		}
		elapsed: f64 = f64(time.tick_diff(start, time.tick_now()))/1e6;
		for status in statuses[:steps]
		{
			support.extension_require(status, "C gravity step");
		}
		result: Gravity_Result = c_gravity_validate(&owner, mode, steps);
		result.step_ms = elapsed;
		if owner.world_allocation.requests != world_requests || owner.worker_allocation.requests != worker_requests ||
			result.world_pool_bytes != world_before || result.worker_pool_bytes != workers_before || !(elapsed > 0 && elapsed < math.F64_MAX)
		{
			panic("C gravity steady allocation or timing");
		}
		support.extension_require(e.Status(c.export_entasis_world_destroy(&owner.world, nil)), "C gravity world destroy");
		support.extension_require(e.Status(c.export_entasis_buffer_pool_destroy(&owner.pool, nil)), "C gravity pool destroy");
		if c.export_entasis_thread_pool_destroy(&owner.thread_pool, nil) != 0
		{
			panic("C gravity workers destroy");
		}
		if owner.world_allocation.live_bytes != 0 || owner.worker_allocation.live_bytes != 0 || (mode == .Global_Callback && owner.disposed != 1)
		{
			panic("C gravity disposal");
		}
		delete(owner.callback_workers);
		return result;
	}

	measure_c_gravity_components :: proc(workers: int) -> [C_GRAVITY_COUNT]Gravity_Result
	{
		results: [C_GRAVITY_COUNT]Gravity_Result;
		for &result, index in results
		{
			mode: C_Gravity_Case = C_Gravity_Case(index);
			_ = c_gravity_measure_scene(workers, mode, WARMUP_STEPS);
			result = c_gravity_measure_scene(workers, mode, GRAVITY_MEASURED_STEPS);
		}
		return results;
	}
}
