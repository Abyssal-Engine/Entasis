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
	Mixed_Collider_Case :: enum u8
	{
		All_Solid,
		Whole_Trigger,
		Mixed_Flat,
		Mixed_Nested,
		Mixed_Sleeping,
	}
	MIXED_COLLIDER_NAMES: [5]string = {"all_solid", "whole_trigger", "mixed_flat", "mixed_nested", "mixed_sleeping"};
	MIXED_COLLIDER_HEADERS :: ",mc01_state,mc01_collidables,mc01_measured_transitions,mc01_all_solid_step_ms,mc01_all_solid_parent_drain_ms,mc01_all_solid_part_drain_ms,mc01_all_solid_dormant_ms,mc01_all_solid_parent_enters,mc01_all_solid_parent_exits,mc01_all_solid_part_enters,mc01_all_solid_part_exits,mc01_all_solid_physical_responses,mc01_all_solid_world_pool_bytes,mc01_whole_trigger_step_ms,mc01_whole_trigger_parent_drain_ms,mc01_whole_trigger_part_drain_ms,mc01_whole_trigger_dormant_ms,mc01_whole_trigger_parent_enters,mc01_whole_trigger_parent_exits,mc01_whole_trigger_part_enters,mc01_whole_trigger_part_exits,mc01_whole_trigger_physical_responses,mc01_whole_trigger_world_pool_bytes,mc01_mixed_flat_step_ms,mc01_mixed_flat_parent_drain_ms,mc01_mixed_flat_part_drain_ms,mc01_mixed_flat_dormant_ms,mc01_mixed_flat_parent_enters,mc01_mixed_flat_parent_exits,mc01_mixed_flat_part_enters,mc01_mixed_flat_part_exits,mc01_mixed_flat_physical_responses,mc01_mixed_flat_world_pool_bytes,mc01_mixed_nested_step_ms,mc01_mixed_nested_parent_drain_ms,mc01_mixed_nested_part_drain_ms,mc01_mixed_nested_dormant_ms,mc01_mixed_nested_parent_enters,mc01_mixed_nested_parent_exits,mc01_mixed_nested_part_enters,mc01_mixed_nested_part_exits,mc01_mixed_nested_physical_responses,mc01_mixed_nested_world_pool_bytes,mc01_mixed_sleeping_step_ms,mc01_mixed_sleeping_parent_drain_ms,mc01_mixed_sleeping_part_drain_ms,mc01_mixed_sleeping_dormant_ms,mc01_mixed_sleeping_parent_enters,mc01_mixed_sleeping_parent_exits,mc01_mixed_sleeping_part_enters,mc01_mixed_sleeping_part_exits,mc01_mixed_sleeping_physical_responses,mc01_mixed_sleeping_world_pool_bytes";
	Mixed_Collider_Component :: struct
	{
		step_ms, parent_drain_ms, part_drain_ms, dormant_ms: f64,
		parent_enters, parent_exits, part_enters, part_exits, physical_responses: int,
		world_pool_bytes: u64,
	}
	mixed_benchmark_shape :: proc(world: ^entasis.World, mode: Mixed_Collider_Case, solid_child: int = -1) -> (entasis.Shape_Handle, entasis.Status)
	{
		leaf: entasis.Shape_Handle;
		status: entasis.Status;
		leaf, status = entasis.shape_add(world, entasis.sphere(0.5));
		if status != .Ok
		{
			return {}, status;
		}
		if mode == .Mixed_Nested
		{
			descendants: [2]entasis.Compound_Child = {
				entasis.compound_child(leaf, entasis.pose({0, 0, -0.2})),
				entasis.compound_child(leaf, entasis.pose({0, 0, 0.2})),
			};
			leaf, status = entasis.shape_import_compound(world, descendants[:]);
			if status != .Ok
			{
				return {}, status;
			}
		}
		children: [2]entasis.Compound_Child = {
			entasis.compound_child(leaf, entasis.pose({-0.75, 0, 0})),
			entasis.compound_child(leaf, entasis.pose({0.75, 0, 0})),
		};
		selected: []entasis.Compound_Child = children[:];
		if solid_child >= 0
		{
			selected = children[solid_child:solid_child+1];
		}
		shape: entasis.Shape_Handle;
		shape, status = entasis.shape_import_compound(world, selected);
		if status != .Ok
		{
			return {}, status;
		}
		return shape, .Ok;
	}
	mixed_benchmark_description :: proc(workers: int) -> entasis.World_Description
	{
		description: entasis.World_Description = entasis.world_description_default();
		description.gravity = {};
		description.damping = {};
		description.threading.worker_count = i32(workers);
		description.capacity.bodies = TRIGGER_PAIR_COUNT;
		description.capacity.statics = TRIGGER_PAIR_COUNT;
		description.capacity.constraints = TRIGGER_COLLIDABLE_COUNT;
		description.capacity.pairs = TRIGGER_PAIR_COUNT;
		description.capacity.broad_phase_candidates = TRIGGER_COLLIDABLE_COUNT;
		description.solve = {velocity_iterations=4, substeps=1, fallback_batch_threshold=64};
		return description;
	}
	mixed_benchmark_solid_response :: proc(mode: Mixed_Collider_Case) -> ([2]entasis.Body_Velocity, entasis.Status)
	{
		world: entasis.World;
		defer entasis.world_destroy(&world);
		if failure: entasis.Status = entasis.world_init(&world, mixed_benchmark_description(1)); failure != .Ok
		{
			return {}, failure;
		}
		box: entasis.Shape_Handle;
		status: entasis.Status;
		box, status = entasis.shape_add(&world, entasis.box(3, 1, 2));
		if status != .Ok
		{
			return {}, status;
		}
		bodies: [2]entasis.Body_Handle;
		for &body, index in bodies
		{
			child: int = 1-index;
			if mode == .All_Solid
			{
				child = -1;
			}
			shape: entasis.Shape_Handle;
			shape, status = mixed_benchmark_shape(&world, mode, child);
			if status != .Ok
			{
				return {}, status;
			}
			ground: entasis.Static_Handle;
			ground, status = entasis.static_add(&world, entasis.static_body(box, entasis.pose({f32(index)*8, 0, 0})), .None);
			if status != .Ok
			{
				return {}, status;
			}
			body, status = entasis.body_add(&world, entasis.body_dynamic(shape,
					{inverse_mass=1, inverse_inertia_tensor={xx=1, yy=1, zz=1}}, entasis.pose({f32(index)*8, 0.75, 0}),
					entasis.velocity({0, -1, 0}), entasis.body_activity(-1, 255)));
			if status != .Ok
			{
				return {}, status;
			}
		}
		if failure: entasis.Status = entasis.world_step(&world, TRIGGER_DT); failure != .Ok
		{
			return {}, failure;
		}
		result: [2]entasis.Body_Velocity;
		for &velocity, index in result
		{
			state: entasis.Body_State;
			state, status = entasis.body_get(&world, bodies[index]);
			if status != .Ok
			{
				return {}, status;
			}
			velocity = state.velocity;
		}
		if failure: entasis.Status = entasis.world_destroy(&world); failure != .Ok
		{
			return {}, failure;
		}
		return result, .Ok;
	}
	Mixed_Owner :: struct
	{
		world: entasis.World,
		description: entasis.World_Description,
		mode: Mixed_Collider_Case,
		shape, box: entasis.Shape_Handle,
		bodies: [TRIGGER_PAIR_COUNT]entasis.Body_Handle,
		statics: [TRIGGER_PAIR_COUNT]entasis.Static_Handle,
		parents: [TRIGGER_PAIR_COUNT]entasis.Trigger_Event,
		parts: [TRIGGER_PAIR_COUNT]entasis.Collider_Part_Event,
		parent_written, part_written: int,
		reference: [2]entasis.Body_Velocity,
	}

	mixed_create :: proc(owner: ^Mixed_Owner, workers: int, mode: Mixed_Collider_Case) -> entasis.Status
	{
		status: entasis.Status;
		owner.mode, owner.description = mode, mixed_benchmark_description(workers);
		if failure: entasis.Status = entasis.world_init(&owner.world, owner.description); failure != .Ok
		{
			return failure;
		}
		owner.shape, status = mixed_benchmark_shape(&owner.world, mode);
		if status != .Ok
		{
			return status;
		}
		box_description: entasis.Box = entasis.box(3, 1, 2);
		if mode == .Mixed_Sleeping
		{
			box_description = entasis.box(1, 1, 1);
		}
		owner.box, status = entasis.shape_add(&owner.world, box_description);
		if status != .Ok
		{
			return status;
		}
		if failure: entasis.Status = entasis.world_enable_triggers(&owner.world,
				{pair_capacity=TRIGGER_PAIR_COUNT, candidates_per_worker=TRIGGER_COLLIDABLE_COUNT, child_capacity=TRIGGER_COLLIDABLE_COUNT*4}); failure != .Ok
		{
			return failure;
		}
		if mode == .Mixed_Flat || mode == .Mixed_Nested || mode == .Mixed_Sleeping
		{
			if failure: entasis.Status = entasis.collider_parts_reserve(&owner.world,
					{instance_capacity=TRIGGER_COLLIDABLE_COUNT, part_capacity=TRIGGER_COLLIDABLE_COUNT, pair_capacity=TRIGGER_PAIR_COUNT,
						observations_per_worker=TRIGGER_COLLIDABLE_COUNT*4, event_subscription=.Enabled}); failure != .Ok
			{
				return failure;
			}
		}
		for &body, index in owner.bodies
		{
			static_x: f32 = f32(index)*8;
			activity: entasis.Activity_Description = entasis.body_activity(-1, 255);
			if mode == .Mixed_Sleeping
			{
				static_x += -0.75+f32(index%2)*1.5;
				activity = entasis.body_activity(0.01, 1);
			}
			owner.statics[index], status = entasis.static_add(&owner.world, entasis.static_body(owner.box, entasis.pose({static_x, 0, 0})), .None);
			if status != .Ok
			{
				return status;
			}
			body, status = entasis.body_add(&owner.world, entasis.body_dynamic(owner.shape,
					{inverse_mass=1, inverse_inertia_tensor={xx=1, yy=1, zz=1}}, entasis.pose({f32(index)*8, 0.75, 0}), {}, activity));
			if status != .Ok
			{
				return status;
			}
			collidable: entasis.Collidable_Reference;
			collidable, status = entasis.body_collidable_reference(&owner.world, body);
			if status != .Ok
			{
				return status;
			}
			if mode == .Whole_Trigger
			{
				if failure: entasis.Status = entasis.trigger_set(&owner.world, collidable, {user_id=u64(index+1)}); failure != .Ok
				{
					return failure;
				}
			}
			else if mode != .All_Solid
			{
				part: entasis.Collider_Part_Info;
				part, status = entasis.collider_part_set(&owner.world, collidable, i32(index%2), {role=.Trigger, user_id=u64(index+1)});
				if status != .Ok
				{
					return status;
				}
			}
		}
		return .Ok;
	}
	mixed_prepare :: proc(owner: ^Mixed_Owner, transition: int) -> (entasis.Trigger_Event_Kind, entasis.Status)
	{
		mode: Mixed_Collider_Case = owner.mode;
		kind: entasis.Trigger_Event_Kind = .Enter;
		y: f32 = 0.75;
		if transition%2 != 0
		{
			kind = .Exit;
			y = 3;
		}
		for body, index in owner.bodies
		{
			if mode == .Mixed_Sleeping
			{
				x: f32 = f32(index)*8-0.75+f32(index%2)*1.5;
				static_y: f32;
				if kind == .Exit
				{
					static_y = -3;
				}
				if failure: entasis.Status = entasis.static_apply(&owner.world, owner.statics[index], entasis.static_body(owner.box, entasis.pose({x, static_y, 0}))); failure != .Ok
				{
					return {}, failure;
				}
			}
			else
			{
				if failure: entasis.Status = entasis.body_apply(&owner.world, body, entasis.body_dynamic(owner.shape,
							{inverse_mass=1, inverse_inertia_tensor={xx=1, yy=1, zz=1}}, entasis.pose({f32(index)*8, y, 0}),
							entasis.velocity({0, -1, 0}), entasis.body_activity(-1, 255))); failure != .Ok
				{
					return {}, failure;
				}
			}
		}
		return kind, .Ok;
	}

	mixed_validate_motion :: proc(owner: ^Mixed_Owner, kind: entasis.Trigger_Event_Kind, transition: int) -> (int, entasis.Status)
	{
		mode: Mixed_Collider_Case = owner.mode;
		status: entasis.Status;
		physical_responses: int;
		for body, index in owner.bodies
		{
			state: entasis.Body_State;
			state, status = entasis.body_get(&owner.world, body);
			if status != .Ok
			{
				return 0, status;
			}
			expected_velocity: entasis.Body_Velocity = entasis.velocity({0, -1, 0});
			if mode == .Mixed_Sleeping
			{
				expected_velocity = {};
			}
			else if kind == .Enter && mode != .Whole_Trigger
			{
				expected_velocity = owner.reference[index%2];
				physical_responses += 1;
			}
			delta_linear: entasis.Vector3 = util.vector3_subtract(state.velocity.linear, expected_velocity.linear);
			delta_angular: entasis.Vector3 = util.vector3_subtract(state.velocity.angular, expected_velocity.angular);
			if !(abs(delta_linear.x)+abs(delta_linear.y)+abs(delta_linear.z)+abs(delta_angular.x)+abs(delta_angular.y)+abs(delta_angular.z) < 0.005)
			{
				return 0, .Invalid_Argument;
			}
		}
		stats: entasis.World_Stats;
		stats, status = entasis.world_stats(&owner.world);
		if status != .Ok
		{
			return 0, status;
		}
		if mode == .Mixed_Sleeping && (stats.sleeping_bodies != TRIGGER_PAIR_COUNT || stats.active_constraints != 0)
		{
			fmt.eprintf("mixed_sleep_transition_failed transition=%d sleeping=%d active_constraints=%d\n", transition, stats.sleeping_bodies, stats.active_constraints);
			return 0, .Invalid_Argument;
		}
		return physical_responses, .Ok;
	}

	mixed_benchmark_drain :: proc(owner: ^Mixed_Owner, expected: int,
		kind: entasis.Trigger_Event_Kind, component: ^Mixed_Collider_Component) -> entasis.Status
	{
		owner.parent_written, owner.part_written = 0, 0;
		written, required: int;
		status: entasis.Status;
		start: time.Tick = time.tick_now();
		written, required, status = entasis.trigger_events_drain(&owner.world, owner.parents[:]);
		component.parent_drain_ms += f64(time.tick_diff(start, time.tick_now()))/1e6;
		if status != .Ok
		{
			return status;
		}
		owner.parent_written = written;
		if written != expected || required != expected
		{
			return .Invalid_Argument;
		}
		for event in owner.parents[:written]
		{
			if event.kind != kind
			{
				return .Invalid_Argument;
			}
		}
		if kind == .Enter
		{
			component.parent_enters += written;
		}
		else
		{
			component.parent_exits += written;
		}
		if owner.mode == .Mixed_Flat || owner.mode == .Mixed_Nested || owner.mode == .Mixed_Sleeping
		{
			start = time.tick_now();
			written, required, status = entasis.trigger_part_events_drain(&owner.world, owner.parts[:]);
			component.part_drain_ms += f64(time.tick_diff(start, time.tick_now()))/1e6;
			if status != .Ok
			{
				return status;
			}
			owner.part_written = written;
			if written != expected || required != expected
			{
				return .Invalid_Argument;
			}
			for event in owner.parts[:written]
			{
				endpoint: entasis.Collider_Part_Endpoint = event.pair.a;
				if endpoint.user_id == 0
				{
					endpoint = event.pair.b;
				}
				if event.kind != kind || endpoint.user_id == 0 || endpoint.user_id > TRIGGER_PAIR_COUNT ||
				endpoint.child_index != i32((endpoint.user_id-1)%2) || endpoint.serial == 0
				{
					return .Invalid_Argument;
				}
			}
			if kind == .Enter
			{
				component.part_enters += written;
			}
			else
			{
				component.part_exits += written;
			}
		}
		return .Ok;
	}
	measure_mixed_collider_components :: proc(workers: int) -> [5]Mixed_Collider_Component
	{
		results: [5]Mixed_Collider_Component;
		for &result, case_index in results
		{
			mode: Mixed_Collider_Case = Mixed_Collider_Case(case_index);
			owner: Mixed_Owner;
			support.extension_require(mixed_create(&owner, workers, mode), "mixed benchmark create");
			status: entasis.Status;
			if mode == .Mixed_Sleeping
			{
				for setup in 0..<TRIGGER_PAIR_COUNT+1
				{
					support.extension_require(entasis.world_step(&owner.world, TRIGGER_DT), "mixed sleep setup");
					expected: int;
					if setup == 0
					{
						expected = TRIGGER_PAIR_COUNT;
					}
					discarded: Mixed_Collider_Component;
					support.extension_require(mixed_benchmark_drain(&owner, expected, .Enter, &discarded), "mixed benchmark drain");
					stats: entasis.World_Stats;
					stats, status = entasis.world_stats(&owner.world);
					support.extension_require(status, "mixed sleep progress");
					if stats.sleeping_bodies == TRIGGER_PAIR_COUNT
					{
						break;
					}
				}
				stats: entasis.World_Stats;
				stats, status = entasis.world_stats(&owner.world);
				support.extension_require(status, "mixed sleep setup stats");
				if stats.sleeping_bodies != TRIGGER_PAIR_COUNT
				{
					panic("mixed sleep setup did not settle");
				}
				for _ in 0..<300
				{
					start: time.Tick = time.tick_now();
					status = entasis.world_step(&owner.world, TRIGGER_DT);
					result.dormant_ms += f64(time.tick_diff(start, time.tick_now()))/1e6;
					support.extension_require(status, "mixed dormant step");
					discarded: Mixed_Collider_Component;
					support.extension_require(mixed_benchmark_drain(&owner, 0, .Enter, &discarded), "mixed benchmark drain");
				}
				// clear the initial overlap before the alternating Enter/Exit series
				for handle, index in owner.statics
				{
					x: f32 = f32(index)*8-0.75+f32(index%2)*1.5;
					support.extension_require(entasis.static_apply(&owner.world, handle, entasis.static_body(owner.box, entasis.pose({x, -3, 0}))), "mixed sleep initial separation");
				}
				support.extension_require(entasis.world_step(&owner.world, TRIGGER_DT), "mixed sleep separated step");
				discarded: Mixed_Collider_Component;
				support.extension_require(mixed_benchmark_drain(&owner, TRIGGER_PAIR_COUNT, .Exit, &discarded), "mixed benchmark drain");
			}
			if mode == .All_Solid || mode == .Mixed_Flat || mode == .Mixed_Nested
			{
				owner.reference, status = mixed_benchmark_solid_response(mode);
				support.extension_require(status, "mixed solid oracle");
			}
			for transition in 0..<TRIGGER_WARMUP_TRANSITIONS+TRIGGER_MEASURED_TRANSITIONS
			{
				kind: entasis.Trigger_Event_Kind;
				kind, status = mixed_prepare(&owner, transition);
				support.extension_require(status, "mixed benchmark prepare");
				iteration: Mixed_Collider_Component;
				start: time.Tick = time.tick_now();
				status = entasis.world_step(&owner.world, TRIGGER_DT);
				iteration.step_ms = f64(time.tick_diff(start, time.tick_now()))/1e6;
				if status != .Ok
				{
					fmt.eprintf("mixed_benchmark_failed mode=%s transition=%d status=%v\n", MIXED_COLLIDER_NAMES[case_index], transition, status);
				}
				support.extension_require(status, "mixed benchmark step");
				expected: int = TRIGGER_PAIR_COUNT;
				if mode == .All_Solid
				{
					expected = 0;
				}
				support.extension_require(mixed_benchmark_drain(&owner, expected, kind, &iteration), "mixed benchmark drain");
				iteration.physical_responses, status = mixed_validate_motion(&owner, kind, transition);
				support.extension_require(status, "mixed benchmark motion");
				if transition >= TRIGGER_WARMUP_TRANSITIONS
				{
					result.step_ms += iteration.step_ms;
					result.parent_drain_ms += iteration.parent_drain_ms;
					result.part_drain_ms += iteration.part_drain_ms;
					result.parent_enters += iteration.parent_enters;
					result.parent_exits += iteration.parent_exits;
					result.part_enters += iteration.part_enters;
					result.part_exits += iteration.part_exits;
					result.physical_responses += iteration.physical_responses;
				}
			}
			pool: ^entasis.Buffer_Pool;
			pool, status = entasis.world_borrow_pool(&owner.world);
			support.extension_require(status, "mixed benchmark pool");
			result.world_pool_bytes = util.buffer_pool_total_allocated_byte_count(pool);
			support.extension_require(entasis.world_destroy(&owner.world), "mixed benchmark destroy");
		}
		return results;
	}
}
