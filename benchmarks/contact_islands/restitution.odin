package contact_islands

@(require)
import "core:time"
@(require)
import util "entasis:entasis_utilities"
@(require)
import entasis "entasis:entasis"
import support "../benchmark_support"

when support.BENCHMARK_COMPONENTS == "all"
{
	Restitution_Component_State :: enum u8
	{
		Unavailable, Available,
	}
	Restitution_Components :: struct
	{
		state: Restitution_Component_State,
		elapsed_ms: [3]f64,
		bounces: [3]int,
		pairs: [3]i32,
		pool_bytes: [3]u64,
	}
	RESTITUTION_BODY_COUNT :: 640;
	RESTITUTION_WARMUP_COUNT :: 4;
	RESTITUTION_MEASURED_COUNT :: 24;
	RESTITUTION_DT :: f32(1.0/60.0);
	Restitution_Owner :: struct
	{
		world: entasis.World,
		description: entasis.World_Description,
		bodies: [RESTITUTION_BODY_COUNT]entasis.Body_Handle,
		shape: entasis.Shape_Handle,
		inertia: entasis.Body_Inertia,
		selected_count: int,
		pairs: i32,
	}

	restitution_create :: proc(owner: ^Restitution_Owner, worker_count, selected_percent: int) -> Benchmark_Result
	{
		description: entasis.World_Description = entasis.world_description_default();
		description.gravity = {};
		description.damping = {};
		description.threading.worker_count = i32(worker_count);
		description.solve = {velocity_iterations=4, substeps=1, fallback_batch_threshold=64};
		description.capacity.bodies = RESTITUTION_BODY_COUNT;
		description.capacity.statics = RESTITUTION_BODY_COUNT;
		description.capacity.constraints = RESTITUTION_BODY_COUNT;
		description.capacity.pairs = RESTITUTION_BODY_COUNT;
		description.capacity.broad_phase_candidates = RESTITUTION_BODY_COUNT*2;
		owner.description = description;
		status: entasis.Status = entasis.world_init(&owner.world, description);
		if status != .Ok
		{
			return {status=.Creation_Failed, physics_status=status};
		}
		owner.shape, status = entasis.shape_add(&owner.world, entasis.sphere(1));
		if status != .Ok
		{
			return {status=.Fixture_Failed, physics_status=status};
		}
		owner.inertia, status = entasis.shape_inertia(entasis.sphere(1), 1);
		if status != .Ok
		{
			return {status=.Fixture_Failed, physics_status=status};
		}
		for &body, index in owner.bodies
		{
			_, status = entasis.static_add(&owner.world, entasis.static_body(owner.shape, entasis.pose({f32(index)*5, 0, 0})), .None);
			if status != .Ok
			{
				return {status=.Fixture_Failed, physics_status=status};
			}
			body, status = entasis.body_add(&owner.world, entasis.body_dynamic(owner.shape, owner.inertia,
				entasis.pose({f32(index)*5, 4, 0}), {}, entasis.body_activity(-1, 255)));
			if status != .Ok
			{
				return {status=.Fixture_Failed, physics_status=status};
			}
		}
		configuration: entasis.Restitution_Configuration = entasis.restitution_configuration_default();
		configuration.collidable_capacity = RESTITUTION_BODY_COUNT;
		configuration.pair_capacity = RESTITUTION_BODY_COUNT;
		status = entasis.world_enable_restitution(&owner.world, configuration);
		if status != .Ok
		{
			return {status=.Fixture_Failed, physics_status=status};
		}
		owner.selected_count = RESTITUTION_BODY_COUNT*selected_percent/100;
		for body in owner.bodies[:owner.selected_count]
		{
			reference: entasis.Collidable_Reference;
			reference, status = entasis.body_collidable_reference(&owner.world, body);
			if status == .Ok
			{
				status = entasis.restitution_set(&owner.world, reference, {0.5, 1});
			}
			if status != .Ok
			{
				return {status=.Fixture_Failed, physics_status=status};
			}
		}
		return {status=.Ok};
	}

	restitution_position :: proc(owner: ^Restitution_Owner, y: f32, velocity: entasis.Body_Velocity) -> entasis.Status
	{
		for body, index in owner.bodies
		{
			status: entasis.Status = entasis.body_apply(&owner.world, body, entasis.body_dynamic(owner.shape, owner.inertia,
				entasis.pose({f32(index)*5, y, 0}), velocity, entasis.body_activity(-1, 255)));
			if status != .Ok
			{
				return status;
			}
		}
		return .Ok;
	}

	restitution_validate :: proc(owner: ^Restitution_Owner, iteration: int) -> (int, Benchmark_Result)
	{
		observed_bounces: int;
		for body, index in owner.bodies
		{
			state, status := entasis.body_get(&owner.world, body);
			if status != .Ok
			{
				return 0, {status=.Validation_Failed, physics_status=status};
			}
			if index < owner.selected_count
			{
				if !(abs(state.velocity.linear.y-3) < 0.0003)
				{
					return 0, {status=.Validation_Failed, phase=.Measured, failed_step=iteration+1};
				}
				observed_bounces += 1;
			}
			else
			{
				// compliance may retain inward speed; require braking, not a rigid limit
				if !(state.velocity.linear.y > -6 && state.velocity.linear.y <= 0)
				{
					return 0, {status=.Validation_Failed, phase=.Measured, failed_step=iteration+1};
				}
			}
		}
		stats, status := entasis.world_stats(&owner.world);
		if status != .Ok || stats.active_bodies != RESTITUTION_BODY_COUNT || stats.active_constraints != RESTITUTION_BODY_COUNT ||
			stats.active_pairs != RESTITUTION_BODY_COUNT || stats.sleeping_bodies != 0
		{
			return 0, {status=.Validation_Failed, physics_status=status};
		}
		owner.pairs = i32(stats.active_pairs);
		return observed_bounces, {status=.Ok};
	}

	measure_restitution_components :: proc(worker_count: int) -> (Benchmark_Result, Restitution_Components)
	{
		components: Restitution_Components;
		for selected_percent, component in ([3]int{0, 10, 100})
		{
			owner: Restitution_Owner;
			result: Benchmark_Result = restitution_create(&owner, worker_count, selected_percent);
			defer entasis.world_destroy(&owner.world);
			if result.status != .Ok
			{
				return result, components;
			}
			for iteration in 0 ..< RESTITUTION_WARMUP_COUNT+RESTITUTION_MEASURED_COUNT
			{
				// separated stepping rearms a new impact outside its measured region
				status: entasis.Status = restitution_position(&owner, 4, {});
				if status != .Ok
				{
					return {status=.Fixture_Failed, physics_status=status}, components;
				}
				status = entasis.world_step(&owner.world, RESTITUTION_DT);
				if status != .Ok
				{
					return {status=.Step_Failed, physics_status=status}, components;
				}
				status = restitution_position(&owner, 2, entasis.velocity({0, -6, 0}));
				if status != .Ok
				{
					return {status=.Fixture_Failed, physics_status=status}, components;
				}
				start: time.Tick = time.tick_now();
				status = entasis.world_step(&owner.world, RESTITUTION_DT);
				elapsed: i64 = i64(time.tick_diff(start, time.tick_now()));
				if status != .Ok
				{
					return {status=.Step_Failed, physics_status=status}, components;
				}
				bounces: int;
				bounces, result = restitution_validate(&owner, iteration);
				if result.status != .Ok
				{
					return result, components;
				}
				if elapsed <= 0
				{
					return {status=.Validation_Failed}, components;
				}
				if iteration >= RESTITUTION_WARMUP_COUNT
				{
					components.elapsed_ms[component] += f64(elapsed)/1e6;
					components.bounces[component] += bounces;
					components.pairs[component] = owner.pairs;
				}
			}
			pool, status := entasis.world_borrow_pool(&owner.world);
			if status != .Ok
			{
				return {status=.Validation_Failed, physics_status=status}, components;
			}
			components.pool_bytes[component] = util.buffer_pool_total_allocated_byte_count(pool);
			status = entasis.world_destroy(&owner.world);
			if status != .Ok
			{
				return {status=.Release_Failed, release_status=status}, components;
			}
		}
		components.state = .Available;
		return {status=.Ok}, components;
	}
}
