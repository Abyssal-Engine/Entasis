package contact_islands

@(require)
import "core:fmt"
@(require)
import "core:time"
@(require)
import entasis "entasis:entasis"
@(require)
import util "entasis:entasis_utilities"
import support "../benchmark_support"

when support.BENCHMARK_COMPONENTS == "all"
{
	TRIGGER_COLLIDABLE_COUNT :: 640;
	TRIGGER_PAIR_COUNT :: TRIGGER_COLLIDABLE_COUNT/2;
	TRIGGER_WARMUP_TRANSITIONS :: 4;
	TRIGGER_MEASURED_TRANSITIONS :: 24;
	TRIGGER_DT :: f32(1.0/60.0);
	Trigger_Component :: struct
	{
		step_ms, consume_ms: f64,
		sensors, enters, exits, final_constraints: int,
		world_pool_bytes, worker_pool_bytes: u64,
	}
	Trigger_Owner :: struct
	{
		world: entasis.World,
		description: entasis.World_Description,
		shape: entasis.Shape_Handle,
		inertia: entasis.Body_Inertia,
		bodies: [TRIGGER_PAIR_COUNT]entasis.Body_Handle,
		statics: [TRIGGER_PAIR_COUNT]entasis.Static_Handle,
		events: [TRIGGER_PAIR_COUNT]entasis.Trigger_Event,
		selected_pairs, written, required: int,
		kind: entasis.Trigger_Event_Kind,
		y: f32,
		velocity: entasis.Body_Velocity,
	}
	trigger_create :: proc(owner: ^Trigger_Owner, worker_count, selected_percent: int) -> entasis.Status
	{
		description: entasis.World_Description = entasis.world_description_default();
		description.gravity = {};
		description.damping = {};
		description.threading.worker_count = i32(worker_count);
		description.capacity.bodies = TRIGGER_PAIR_COUNT;
		description.capacity.statics = TRIGGER_PAIR_COUNT;
		description.capacity.constraints = TRIGGER_PAIR_COUNT;
		description.capacity.pairs = TRIGGER_PAIR_COUNT;
		description.capacity.broad_phase_candidates = TRIGGER_COLLIDABLE_COUNT;
		description.solve = {velocity_iterations=4, substeps=1, fallback_batch_threshold=64};
		owner.description = description;
		if failure: entasis.Status = entasis.world_init(&owner.world, description); failure != .Ok
		{
			return failure;
		}
		status: entasis.Status;
		owner.shape, status = entasis.shape_add(&owner.world, entasis.sphere(0.5));
		if status != .Ok
		{
			return status;
		}
		owner.inertia, status = entasis.shape_inertia(entasis.sphere(0.5), 1);
		if status != .Ok
		{
			return status;
		}
		for &body, index in owner.bodies
		{
			owner.statics[index], status = entasis.static_add(&owner.world, entasis.static_body(owner.shape,
				entasis.pose({f32(index)*4, 0, 0})), .None);
			if status != .Ok
			{
				return status;
			}
			body, status = entasis.body_add(&owner.world, entasis.body_dynamic(owner.shape, owner.inertia,
				entasis.pose({f32(index)*4, 3, 0}), {}, entasis.body_activity(-1, 255)));
			if status != .Ok
			{
				return status;
			}
		}
		if failure: entasis.Status = entasis.world_enable_triggers(&owner.world,
			{pair_capacity=TRIGGER_PAIR_COUNT, candidates_per_worker=TRIGGER_COLLIDABLE_COUNT, child_capacity=TRIGGER_COLLIDABLE_COUNT}); failure != .Ok
		{
			return failure;
		}
		owner.selected_pairs = TRIGGER_PAIR_COUNT*selected_percent/100;
		for index in 0 ..< owner.selected_pairs
		{
			reference: entasis.Collidable_Reference;
			reference, status = entasis.body_collidable_reference(&owner.world, owner.bodies[index]);
			if status != .Ok
			{
				return status;
			}
			if failure: entasis.Status = entasis.trigger_set(&owner.world, reference, {user_id=u64(index+1)}); failure != .Ok
			{
				return failure;
			}
			reference, status = entasis.static_collidable_reference(&owner.world, owner.statics[index]);
			if status != .Ok
			{
				return status;
			}
			if failure: entasis.Status = entasis.trigger_set(&owner.world, reference, {user_id=u64(index+1)}); failure != .Ok
			{
				return failure;
			}
		}
	return .Ok;
	}
	trigger_prepare :: proc(owner: ^Trigger_Owner, transition: int) -> entasis.Status
	{
		owner.kind, owner.y, owner.velocity = .Enter, 0.75, entasis.velocity({0, -1, 0});
		if transition%2 == 1
		{
			owner.kind, owner.y, owner.velocity = .Exit, 3, {};
		}
		for body, index in owner.bodies
		{
			if failure: entasis.Status = entasis.body_apply(&owner.world, body, entasis.body_dynamic(owner.shape, owner.inertia,
				entasis.pose({f32(index)*4, owner.y, 0}), owner.velocity, entasis.body_activity(-1, 255))); failure != .Ok
			{
				return failure;
			}
		}
	return .Ok;
	}
	trigger_validate :: proc(owner: ^Trigger_Owner) -> (i32, entasis.Status)
	{
		if owner.written != owner.selected_pairs || owner.required != owner.selected_pairs
		{
			return 0, .Invalid_Argument;
		}
		for event in owner.events[:owner.written]
		{
			if event.kind != owner.kind || event.pair.flags&3 != 3 || event.pair.user_a != event.pair.user_b ||
				event.pair.user_a == 0 || event.pair.user_a > u64(owner.selected_pairs)
			{
				return 0, .Invalid_Argument;
			}
		}
		for body in owner.bodies[:owner.selected_pairs]
		{
			state, status := entasis.body_get(&owner.world, body);
			if status != .Ok
			{
				return 0, status;
			}
			if abs(state.velocity.linear.y-owner.velocity.linear.y) > 0.0001 ||
				abs(state.pose.position.y-(owner.y+owner.velocity.linear.y*TRIGGER_DT)) > 0.0001
			{
				return 0, .Invalid_Argument;
			}
		}
		stats, status := entasis.world_stats(&owner.world);
		if status != .Ok
		{
			return 0, status;
		}
		expected_constraints: int = 0;
		if owner.kind == .Enter
		{
			expected_constraints = TRIGGER_PAIR_COUNT-owner.selected_pairs;
		}
		if stats.active_bodies != TRIGGER_PAIR_COUNT || stats.statics != TRIGGER_PAIR_COUNT ||
			stats.active_constraints != expected_constraints || stats.sleeping_bodies != 0
		{
			return 0, .Invalid_Argument;
		}
		return i32(stats.active_constraints), .Ok;
	}
	measure_trigger_components :: proc(worker_count: int) -> [3]Trigger_Component
	{
		components: [3]Trigger_Component;
		for selected_percent, component in ([3]int{0, 10, 100})
		{
			owner: Trigger_Owner;
			support.extension_require(trigger_create(&owner, worker_count, selected_percent), "trigger component create");
			components[component].sensors = owner.selected_pairs*2;
			for transition in 0 ..< TRIGGER_WARMUP_TRANSITIONS+TRIGGER_MEASURED_TRANSITIONS
			{
				support.extension_require(trigger_prepare(&owner, transition), "trigger component reset");
				start: time.Tick = time.tick_now();
				status: entasis.Status = entasis.world_step(&owner.world, TRIGGER_DT);
				step_ms: f64 = f64(time.tick_diff(start, time.tick_now()))/1e6;
				support.extension_require(status, "trigger component step");
				start = time.tick_now();
				owner.written, owner.required, status = entasis.trigger_events_drain(&owner.world, owner.events[:]);
				consume_ms: f64 = f64(time.tick_diff(start, time.tick_now()))/1e6;
				support.extension_require(status, "trigger component event drain");
				if owner.written != owner.selected_pairs || owner.required != owner.selected_pairs || step_ms <= 0 || consume_ms < 0
				{
					fmt.eprintfln("trigger_component_failed percent=%d transition=%d written=%d required=%d expected=%d",
						selected_percent, transition, owner.written, owner.required, owner.selected_pairs);
					panic("trigger component event coverage");
				}
				constraints: i32;
				constraints, status = trigger_validate(&owner);
				support.extension_require(status, "trigger component validation");
				if transition >= TRIGGER_WARMUP_TRANSITIONS
				{
					components[component].step_ms += step_ms;
					components[component].consume_ms += consume_ms;
					components[component].final_constraints = int(constraints);
					if owner.kind == .Enter
					{
						components[component].enters += owner.written;
					}
					else
					{
						components[component].exits += owner.written;
					}
				}
			}
			if components[component].consume_ms <= 0
			{
				panic("trigger event-drain timing below clock resolution");
			}
			pool, status := entasis.world_borrow_pool(&owner.world);
			support.extension_require(status, "trigger component pool");
			components[component].world_pool_bytes = util.buffer_pool_total_allocated_byte_count(pool);
			dispatcher: ^entasis.Dispatcher;
			dispatcher, status = entasis.world_borrow_dispatcher(&owner.world);
			support.extension_require(status, "trigger component dispatcher");
			if dispatcher != nil
			{
				for index in 0 ..< dispatcher.worker_count
				{
					worker, pool_status := dispatcher.worker_pool(dispatcher, index);
					if pool_status != .Ok
					{
						panic("trigger component worker pool");
					}
					components[component].worker_pool_bytes += util.buffer_pool_total_allocated_byte_count(worker);
				}
			}
			support.extension_require(entasis.world_destroy(&owner.world), "trigger component destroy");
		}
		return components;
	}
}
