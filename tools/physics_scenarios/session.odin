package physics_scenarios

import observation "../physics_observation"
import entasis "entasis:entasis"
import falling_box "../../examples/headless/falling_box"
import minimal_world "../../examples/headless/minimal_world"
import fixed_step_loop "../../examples/headless/fixed_step_loop"
import sleep_and_awaken "../../examples/headless/sleep_and_awaken"
import box_pile_batch "../../examples/headless/box_pile_batch"
import bulk_body_update "../../examples/headless/bulk_body_update"
import planetary_gravity "../../examples/headless/planetary_gravity"
import compounds "../../examples/headless/compounds"
import constraints "../../examples/headless/constraints"
import kinematic_platform "../../examples/headless/kinematic_platform"
import per_body_gravity "../../examples/headless/per_body_gravity"
import continuous_collision "../../examples/headless/continuous_collision"
import substepping "../../examples/headless/substepping"
import gyroscope "../../examples/headless/gyroscope"
import convex_hulls "../../examples/headless/convex_hulls"
import ray_queries "../../examples/headless/ray_queries"
import sweep_queries "../../examples/headless/sweep_queries"
import batched_queries "../../examples/headless/batched_queries"
import meshes "../../examples/headless/meshes"
import background_cooking "../../examples/headless/background_cooking"
import cloth "../../examples/headless/cloth"
import rope "../../examples/headless/rope"
import external_dispatcher "../../examples/headless/external_dispatcher"
import profiling_and_state_export "../../examples/headless/profiling_and_state_export"
import materials_and_filtering "../../examples/headless/materials_and_filtering"
import ragdoll "../../examples/headless/ragdoll"
import direct_collision_queries "../../examples/headless/direct_collision_queries"
import impulses_solver_contacts "../../examples/headless/impulses_solver_contacts"
import overlap_and_events "../../examples/headless/overlap_and_events"
import custom_shape "../../examples/headless/custom_shape"
import custom_constraint "../../examples/headless/custom_constraint"
import character_controller "../../examples/headless/character_controller"
import vehicle "../../examples/headless/vehicle"
import tank_controller "../../examples/headless/tank_controller"
import triggers "../../examples/headless/triggers"
import scene "../physics_scene"

Session_State :: enum
{
	Loading, Paused, Running, Completed, Failed,
}
Owner :: union
{
	^falling_box.Case,
	^minimal_world.Case,
	^fixed_step_loop.Case,
	^sleep_and_awaken.Case,
	^box_pile_batch.Case,
	^bulk_body_update.Case,
	^planetary_gravity.Case,
	^compounds.Case,
	^constraints.Case,
	^kinematic_platform.Case,
	^per_body_gravity.Case,
	^continuous_collision.Case,
	^substepping.Case,
	^gyroscope.Case,
	^convex_hulls.Case,
	^ray_queries.Case,
	^sweep_queries.Case,
	^batched_queries.Case,
	^meshes.Case,
	^background_cooking.Case,
	^cloth.Case,
	^rope.Case,
	^external_dispatcher.Case,
	^profiling_and_state_export.Case,
	^materials_and_filtering.Case,
	^ragdoll.Case,
	^direct_collision_queries.Case,
	^impulses_solver_contacts.Case,
	^overlap_and_events.Case,
	^custom_shape.Case,
	^custom_constraint.Case,
	^character_controller.Case,
	^vehicle.Case,
	^tank_controller.Case,
	^triggers.Case,
}
Session :: struct
{
	recipe: Recipe,
	owner: Owner,
	pool: entasis.Buffer_Pool,
	observation: observation.Observation,
	state: Session_State,
	tick, extent, cycle: int,
	phase: scene.Phase,
	timestep: f32,
	axis: scene.Time_Axis,
	required_bytes: u64,
}

@(private)
session_world :: proc(s: ^Session) -> ^entasis.World
{
	switch owner in s.owner
	{
	case ^falling_box.Case: return &owner.world;
	case ^minimal_world.Case: return &owner.world;
	case ^fixed_step_loop.Case: return &owner.world;
	case ^sleep_and_awaken.Case: return &owner.world;
	case ^box_pile_batch.Case: return &owner.world;
	case ^bulk_body_update.Case: return &owner.world;
	case ^planetary_gravity.Case: return &owner.world;
	case ^compounds.Case: return &owner.world;
	case ^constraints.Case: return &owner.world;
	case ^kinematic_platform.Case: return &owner.world;
	case ^per_body_gravity.Case: return &owner.world;
	case ^continuous_collision.Case: return &owner.world;
	case ^substepping.Case: return &owner.world;
	case ^gyroscope.Case: return &owner.world;
	case ^convex_hulls.Case: return &owner.world;
	case ^ray_queries.Case: return &owner.world;
	case ^sweep_queries.Case: return &owner.world;
	case ^batched_queries.Case: return &owner.world;
	case ^meshes.Case: return &owner.world;
	case ^background_cooking.Case: return &owner.world;
	case ^cloth.Case: return &owner.world;
	case ^rope.Case: return &owner.world;
	case ^external_dispatcher.Case: return &owner.world;
	case ^profiling_and_state_export.Case: return &owner.world;
	case ^materials_and_filtering.Case: return &owner.world;
	case ^ragdoll.Case: return &owner.world;
	case ^direct_collision_queries.Case: return &owner.world;
	case ^impulses_solver_contacts.Case: return &owner.world;
	case ^overlap_and_events.Case: return &owner.world;
	case ^custom_shape.Case: return &owner.world;
	case ^custom_constraint.Case: return &owner.world;
	case ^character_controller.Case: return &owner.world;
	case ^vehicle.Case: return &owner.world;
	case ^tank_controller.Case: return &owner.world;
	case ^triggers.Case: return &owner.world;
	}
	unreachable();
}

session_destroy :: proc(s: ^Session)
{
	switch owner in s.owner
	{
	case ^falling_box.Case: falling_box.case_destroy(owner);
	free(owner);
	case ^minimal_world.Case: minimal_world.case_destroy(owner);
	free(owner);
	case ^fixed_step_loop.Case: fixed_step_loop.case_destroy(owner);
	free(owner);
	case ^sleep_and_awaken.Case: sleep_and_awaken.case_destroy(owner);
		free(owner);
	case ^box_pile_batch.Case: box_pile_batch.case_destroy(owner);
		free(owner);
	case ^bulk_body_update.Case: bulk_body_update.case_destroy(owner);
		free(owner);
	case ^planetary_gravity.Case: planetary_gravity.case_destroy(owner);
		free(owner);
	case ^compounds.Case: compounds.case_destroy(owner);
		free(owner);
	case ^constraints.Case: constraints.case_destroy(owner);
		free(owner);
	case ^kinematic_platform.Case: kinematic_platform.case_destroy(owner);
		free(owner);
	case ^per_body_gravity.Case: per_body_gravity.case_destroy(owner);
		free(owner);
	case ^continuous_collision.Case: continuous_collision.case_destroy(owner);
		free(owner);
	case ^substepping.Case: substepping.case_destroy(owner);
		free(owner);
	case ^gyroscope.Case: gyroscope.case_destroy(owner);
		free(owner);
	case ^convex_hulls.Case: convex_hulls.case_destroy(owner);
		free(owner);
	case ^ray_queries.Case: ray_queries.case_destroy(owner);
		free(owner);
	case ^sweep_queries.Case: sweep_queries.case_destroy(owner);
		free(owner);
	case ^batched_queries.Case: batched_queries.case_destroy(owner);
		free(owner);
	case ^meshes.Case: meshes.case_destroy(owner);
		free(owner);
	case ^background_cooking.Case: background_cooking.case_destroy(owner);
		free(owner);
	case ^cloth.Case: cloth.case_destroy(owner);
		free(owner);
	case ^rope.Case: rope.case_destroy(owner);
		free(owner);
	case ^external_dispatcher.Case: external_dispatcher.case_destroy(owner);
		free(owner);
	case ^profiling_and_state_export.Case: profiling_and_state_export.case_destroy(owner);
		free(owner);
	case ^materials_and_filtering.Case: materials_and_filtering.case_destroy(owner);
		free(owner);
	case ^ragdoll.Case: ragdoll.case_destroy(owner);
		free(owner);
	case ^direct_collision_queries.Case: direct_collision_queries.case_destroy(owner);
		free(owner);
	case ^impulses_solver_contacts.Case: impulses_solver_contacts.case_destroy(owner);
		free(owner);
	case ^overlap_and_events.Case: overlap_and_events.case_destroy(owner);
		free(owner);
	case ^custom_shape.Case: custom_shape.case_destroy(owner);
		free(owner);
	case ^custom_constraint.Case: custom_constraint.case_destroy(owner);
		free(owner);
	case ^character_controller.Case: character_controller.case_destroy(owner);
		free(owner);
	case ^vehicle.Case: vehicle.case_destroy(owner);
		free(owner);
	case ^tank_controller.Case: tank_controller.case_destroy(owner);
		free(owner);
	case ^triggers.Case: triggers.case_destroy(owner);
		free(owner);
	}
	entasis.buffer_pool_destroy(&s.pool);
	observation.observation_destroy(&s.observation);
	s^ = {};
}

session_observe :: proc(s: ^Session) -> scene.Status
{
	status: scene.Status = observation.observation_read(&s.observation, session_world(s));
	if status != .Ok
	{
		return status;
	}
	s.observation.packet.frame.tick = u64(s.tick);
	s.observation.packet.frame.phase = s.phase;
	s.observation.packet.frame.cycle = u32(s.cycle);
	#partial switch owner in s.owner
	{
	case ^fixed_step_loop.Case:
		append(&s.observation.packet.frame.metrics,
			scene.Metric{.Step_Alpha, f64(owner.alpha)},
			scene.Metric{.Step_Accumulator_Seconds, f64(owner.stepper.accumulator)},
			scene.Metric{.Steps_This_Operation, f64(owner.last_steps)},
			scene.Metric{.Total_Physics_Steps, f64(owner.total_steps)});
	case ^planetary_gravity.Case:
		frame: ^scene.Frame = &s.observation.packet.frame;
		center: scene.Vec3 = observation.copy_vector(owner.gravity_policy.center);
		position: scene.Vec3 = frame.poses[0].position;
		append(&frame.overlays,
			scene.Overlay{kind=.Point, entity_a=scene.NO_ENTITY, entity_b=scene.NO_ENTITY, a=center},
			scene.Overlay{kind=.Line, entity_a=0, entity_b=scene.NO_ENTITY, a=position, b=center});
		if owner.tick > 0
		{
			acceleration: scene.Vec3 = (observation.copy_vector(owner.final_velocity)-observation.copy_vector(owner.initial_velocity))/planetary_gravity.TIMESTEP;
			append(&frame.overlays, scene.Overlay{kind=.Gravity, entity_a=0, entity_b=scene.NO_ENTITY,
				a=position, b=position+acceleration});
		}
	case ^constraints.Case:
		frame: ^scene.Frame = &s.observation.packet.frame;
		append(&frame.overlays, scene.Overlay{kind=.Constraint, entity_a=0, entity_b=1,
			a=frame.poses[0].position, b=frame.poses[1].position, value=2});
	case ^cloth.Case:
		frame: ^scene.Frame = &s.observation.packet.frame;
		for link in owner.links
		{
			append(&frame.overlays, scene.Overlay{kind=.Constraint, entity_a=u32(link[0]), entity_b=u32(link[1]),
				a=frame.poses[link[0]].position, b=frame.poses[link[1]].position, value=cloth.SPACING});
		}
	case ^rope.Case:
		frame: ^scene.Frame = &s.observation.packet.frame;
		for index in 1 ..< len(owner.bodies)
		{
			append(&frame.overlays, scene.Overlay{kind=.Constraint, entity_a=u32(index-1), entity_b=u32(index),
				a=frame.poses[index-1].position, b=frame.poses[index].position, value=rope.SEGMENT_LENGTH});
		}
	case ^per_body_gravity.Case:
		frame: ^scene.Frame = &s.observation.packet.frame;
		for gravity, index in per_body_gravity.GRAVITIES
		{
			position: scene.Vec3 = frame.poses[index].position;
			append(&frame.overlays, scene.Overlay{kind=.Gravity, entity_a=u32(index),
				a=position, b=position+observation.copy_vector(gravity)});
		}
	case ^continuous_collision.Case:
		s.observation.packet.frame.tick = u64(owner.tick);
	case ^materials_and_filtering.Case:
		s.observation.packet.frame.tick = u64(owner.tick);
	case ^triggers.Case:
		s.observation.packet.frame.tick = u64(owner.tick);
		for event in owner.events[:owner.written]
		{
			observation.observation_trigger(&s.observation, event.pair.a, event.pair.b, event.kind, -1, -1);
		}
		for event in owner.parts[:owner.result.part_events_written]
		{
			observation.observation_trigger(&s.observation, event.pair.a.reference, event.pair.b.reference, event.kind,
				event.pair.a.child_index, event.pair.b.child_index);
		}
		status = observation.observation_contacts(&s.observation, &owner.world, owner.contacts[:owner.result.contact_events_written]);
	case ^ragdoll.Case:
		frame: ^scene.Frame = &s.observation.packet.frame;
		for link, index in ragdoll.LINKS
		{
			a, b: entasis.Body_State;
			a, _ = entasis.body_get(&owner.world, owner.bodies[link[0]]);
			b, _ = entasis.body_get(&owner.world, owner.bodies[link[1]]);
			append(&frame.overlays, scene.Overlay{kind=.Constraint, entity_a=frame.ids[link[0]], entity_b=frame.ids[link[1]],
				a=observation.copy_vector(a.pose.position)+observation.copy_vector(ragdoll.rotate_vector(a.pose.orientation, owner.offsets_a[index])),
				b=observation.copy_vector(b.pose.position)+observation.copy_vector(ragdoll.rotate_vector(b.pose.orientation, owner.offsets_b[index]))},
				scene.Overlay{kind=.Line, entity_a=frame.ids[link[0]], entity_b=frame.ids[link[1]],
					a=observation.copy_vector(a.pose.position), b=observation.copy_vector(b.pose.position)});
		}
	case ^external_dispatcher.Case:
		s.observation.packet.frame.tick = u64(owner.tick);
		append(&s.observation.packet.frame.metrics, scene.Metric{.Workers, 1 if owner.cycle == 0 else f64(owner.interface.worker_count)});
	case ^profiling_and_state_export.Case:
		if owner.tick == profiling_and_state_export.STEPS
		{
			append(&s.observation.packet.frame.metrics,
				scene.Metric{.Timestep_Nanoseconds, f64(owner.profile.stage_durations_nanoseconds[entasis.Profile_Stage.Timestep])});
		}
	case ^ray_queries.Case:
		observation_ray(&s.observation, owner.ray, owner.hits[:owner.count], .Query_Path);
	case ^custom_shape.Case:
		observation_ray(&s.observation, owner.ray, ([^]entasis.Ray_Hit)(&owner.hit)[:owner.tick], .Query_Path);
	case ^custom_constraint.Case:
		point: scene.Vec3 = s.observation.packet.frame.poses[0].position;
		append(&s.observation.packet.frame.overlays, scene.Overlay{kind=.Constraint, entity_a=0, entity_b=scene.NO_ENTITY,
			part_a=-1, part_b=-1, a=point, b=point+scene.Vec3{owner.target.target_speed, 0, 0}, value=owner.target.target_speed});
	case ^vehicle.Case:
		frame: ^scene.Frame = &s.observation.packet.frame;
		for mount, index in vehicle.MOUNTS
		{
			append(&frame.overlays, scene.Overlay{kind=.Constraint, entity_a=frame.ids[0], entity_b=frame.ids[index+1],
				part_a=-1, part_b=-1, a=scene.pose_transform(frame.poses[0], observation.copy_vector(mount)), b=frame.poses[index+1].position, value=0.5});
		}
	case ^tank_controller.Case:
		frame: ^scene.Frame = &s.observation.packet.frame;
		for mount, index in owner.mounts
		{
			append(&frame.overlays, scene.Overlay{kind=.Constraint, entity_a=frame.ids[0], entity_b=frame.ids[index+1],
				part_a=-1, part_b=-1, a=scene.pose_transform(frame.poses[0], observation.copy_vector(mount)), b=frame.poses[index+1].position, value=0.5});
		}
	case ^character_controller.Case:
		if owner.tick > 0
		{
			origin: scene.Vec3 = observation.copy_vector(owner.query_pose.position);
			append(&s.observation.packet.frame.overlays,
				scene.Overlay{kind=.Query_Path, entity_a=0, entity_b=scene.NO_ENTITY, a=origin, b=origin+scene.Vec3{0, -0.08, 0}, value=0.08},
				scene.Overlay{kind=.Query_Path, entity_a=0, entity_b=scene.NO_ENTITY,
					a=origin, b=origin+observation.copy_vector(owner.query_direction)*owner.query_distance, value=owner.query_distance});
			hits: [2]entasis.Sweep_Hit = {owner.ground_hit, owner.horizontal_hit};
			statuses: [2]entasis.Status = {owner.ground_status, owner.horizontal_status};
			for hit, index in hits
			{
				if statuses[index] == .Ok
				{
					point: scene.Vec3 = observation.copy_vector(hit.sweep.location);
					append(&s.observation.packet.frame.overlays, scene.Overlay{kind=.Ray_Hit, entity_a=observation.observation_collidable(&s.observation, hit.collidable),
						entity_b=0, part_a=hit.sweep.child_b, part_b=hit.sweep.child_a,
						a=point, b=point+observation.copy_vector(hit.sweep.normal), value=hit.sweep.t0});
				}
			}
		}
	case ^direct_collision_queries.Case:
		frame: ^scene.Frame = &s.observation.packet.frame;
		switch owner.tick
		{
		case 1:
			for contact in owner.manifold.convex.contacts[:owner.manifold.convex.count]
			{
				point: scene.Vec3 = observation.copy_vector(owner.pose.position)+observation.copy_vector(contact.offset);
				append(&frame.overlays, scene.Overlay{kind=.Contact, entity_a=0, entity_b=1, part_a=-1, part_b=-1,
					a=point, b=point+observation.copy_vector(owner.manifold.convex.normal), value=contact.depth});
			}
		case 2:
			append(&frame.overlays, scene.Overlay{kind=.Ray_Miss, entity_a=0, entity_b=1, part_a=-1, part_b=-1,
				a=observation.copy_vector(owner.pose.position), b={}});
		case 3:
			append(&frame.overlays, scene.Overlay{kind=.Query_Path, entity_a=0, entity_b=scene.NO_ENTITY,
				a={0, 3, 0}, b={0, -2, 0}, value=1});
			for hit in owner.hits[:owner.count]
			{
				point: scene.Vec3 = observation.copy_vector(hit.sweep.location);
				append(&frame.overlays, scene.Overlay{kind=.Ray_Hit, entity_a=observation.observation_collidable(&s.observation, hit.collidable),
					entity_b=0, part_a=hit.sweep.child_b, part_b=hit.sweep.child_a,
					a=point, b=point+observation.copy_vector(hit.sweep.normal), value=hit.sweep.t0});
			}
		}
	case ^impulses_solver_contacts.Case:
		if owner.tick == 1
		{
			status = observation.observation_contacts(&s.observation, &owner.world, owner.events[:owner.written]);
		}
		else if owner.tick == 2
		{
			point: scene.Vec3 = s.observation.packet.frame.poses[0].position+scene.Vec3{1, 0, 0};
			append(&s.observation.packet.frame.overlays, scene.Overlay{kind=.Line, entity_a=0, entity_b=scene.NO_ENTITY,
				part_a=-1, part_b=-1, a=point, b=point+scene.Vec3{0, 2, 0}, value=2});
		}
	case ^overlap_and_events.Case:
		status = observation.observation_contacts(&s.observation, &owner.world, owner.events[:owner.written]);
	case ^meshes.Case:
		observation_ray(&s.observation, owner.ray, ([^]entasis.Ray_Hit)(&owner.hit)[:1], .Query_Path);
	case ^background_cooking.Case:
		if owner.tick > 0
		{
			observation_ray(&s.observation, owner.ray, ([^]entasis.Ray_Hit)(&owner.hit)[:1], .Query_Path);
		}
	case ^batched_queries.Case:
		if owner.tick < 2
		{
			observation_ray(&s.observation, owner.ray, owner.ray_hits[:owner.results[2].count], .Query_Path);
			append(&s.observation.packet.frame.overlays, scene.Overlay{kind=.Bounds,
				entity_a=scene.NO_ENTITY, entity_b=scene.NO_ENTITY, a=observation.copy_vector(owner.bounds.min), b=observation.copy_vector(owner.bounds.max)});
		}
		else
		{
			for query, index in owner.rays
			{
				observation_ray(&s.observation, query.ray_data.ray,
					([^]entasis.Ray_Hit)(&owner.ray_results[index].ray_hit)[:1], .Query_Path);
			}
		}
	case ^sweep_queries.Case:
		frame: ^scene.Frame = &s.observation.packet.frame;
		append(&frame.overlays, scene.Overlay{kind=.Query_Path, entity_a=scene.NO_ENTITY, entity_b=scene.NO_ENTITY, a={}, b={10, 0, 0}, value=1});
		if owner.tick > 0
		{
			point: scene.Vec3 = observation.copy_vector(owner.hit.sweep.location);
			append(&frame.overlays, scene.Overlay{kind=.Ray_Hit, entity_a=observation.observation_collidable(&s.observation, owner.hit.collidable),
				entity_b=scene.NO_ENTITY, part_a=owner.hit.sweep.child_b,
				a=point, b=point+observation.copy_vector(owner.hit.sweep.normal), value=owner.hit.sweep.t0});
		}
	case ^substepping.Case:
		frame: ^scene.Frame = &s.observation.packet.frame;
		frame.tick = u64(owner.tick);
		append(&frame.overlays, scene.Overlay{kind=.Constraint, entity_a=frame.ids[0], entity_b=frame.ids[1],
			a=frame.poses[0].position, b=frame.poses[1].position, value=1});
		append(&frame.metrics, scene.Metric{.Substeps, 1 if owner.cycle == 0 else 4});
	case ^gyroscope.Case:
		frame: ^scene.Frame = &s.observation.packet.frame;
		frame.tick = u64(owner.tick);
		state: entasis.Body_State;
		physics_status: entasis.Status;
		state, physics_status = entasis.body_get(&owner.world, owner.body);
		if physics_status != .Ok
		{
			return .Invalid_Data;
		}
		append(&frame.overlays, scene.Overlay{kind=.Angular_Velocity, entity_a=frame.ids[0],
			a=frame.poses[0].position, b=frame.poses[0].position+observation.copy_vector(state.velocity.angular)});
	}
	return status;
}

session_create :: proc(s: ^Session, recipe: Recipe, budget: u64 = scene.DEFAULT_BUDGET) -> scene.Status
{
	s.recipe = recipe;
	s.extent = recipe.options.steps;
	handles: []entasis.Body_Handle;
	status: scene.Status = .Invalid_Data;
	reserved: u64 = size_of(Session)-size_of(observation.Observation)-size_of(s.pool)+example_fixture_size(recipe.scenario);
	defer if status != .Ok
	{
		required: u64 = max(s.required_bytes, reserved+s.observation.required_bytes);
		session_destroy(s);
		s.required_bytes = required;
	}
	s.required_bytes = reserved+size_of(observation.Observation);
	if s.required_bytes > budget
	{
		status = .Budget_Exceeded;
		return status;
	}
	s.observation.budget = budget-reserved;
	switch recipe.scenario
	{
	case .Falling_Box:
		owner: ^falling_box.Case = new(falling_box.Case);
		if owner == nil
		{
			return .Out_Of_Memory;
		}
		s.owner = owner;
		if falling_box.case_create(owner) != .Ok
		{
			return status;
		}
		handles = ([^]entasis.Body_Handle)(&owner.body)[:1];
		s.extent, s.timestep = falling_box.STEPS, falling_box.TIMESTEP;
	case .Minimal_World:
		owner: ^minimal_world.Case = new(minimal_world.Case);
		if owner == nil
		{
			return .Out_Of_Memory;
		}
		s.owner = owner;
		if minimal_world.case_create(owner)!=.Ok
		{
			return status;
		}
		s.extent, s.timestep = minimal_world.STEPS, minimal_world.TIMESTEP;
	case .Fixed_Step_Loop:
		owner: ^fixed_step_loop.Case = new(fixed_step_loop.Case);
		if owner == nil
		{
			return .Out_Of_Memory;
		}
		s.owner = owner;
		if fixed_step_loop.case_create(owner)!=.Ok
		{
			return status;
		}
		handles = ([^]entasis.Body_Handle)(&owner.body)[:1];
		s.extent, s.timestep, s.axis = fixed_step_loop.STEPS, fixed_step_loop.TIMESTEP, .Operation;
	case .Sleep_And_Awaken:
		owner: ^sleep_and_awaken.Case = new(sleep_and_awaken.Case);
		if owner == nil
		{
			return .Out_Of_Memory;
		}
		s.owner = owner;
		if sleep_and_awaken.case_create(owner)!=.Ok
		{
			return status;
		}
		handles = ([^]entasis.Body_Handle)(&owner.body)[:1];
		s.extent, s.timestep, s.axis = sleep_and_awaken.STEPS, sleep_and_awaken.TIMESTEP, .Operation;
	case .Box_Pile_Batch:
		owner: ^box_pile_batch.Case = new(box_pile_batch.Case);
		if owner == nil
		{
			return .Out_Of_Memory;
		}
		s.owner = owner;
		if box_pile_batch.case_create(owner) != .Ok
		{
			return status;
		}
		handles = owner.bodies[:];
		s.extent, s.timestep = box_pile_batch.STEPS, box_pile_batch.TIMESTEP;
	case .Bulk_Body_Update:
		owner: ^bulk_body_update.Case = new(bulk_body_update.Case);
		if owner == nil
		{
			return .Out_Of_Memory;
		}
		s.owner = owner;
		if bulk_body_update.case_create(owner) != .Ok
		{
			return status;
		}
		handles = owner.bodies[:];
		s.extent, s.timestep, s.axis = bulk_body_update.STEPS, bulk_body_update.TIMESTEP, .Operation;
	case .Planetary_Gravity:
		owner: ^planetary_gravity.Case = new(planetary_gravity.Case);
		if owner == nil
		{
			return .Out_Of_Memory;
		}
		s.owner = owner;
		if planetary_gravity.case_create(owner) != .Ok
		{
			return status;
		}
		handles = ([^]entasis.Body_Handle)(&owner.body)[:1];
		s.extent, s.timestep = planetary_gravity.STEPS, planetary_gravity.TIMESTEP;
	case .Compounds:
		owner: ^compounds.Case = new(compounds.Case);
		if owner == nil
		{
			return .Out_Of_Memory;
		}
		s.owner = owner;
		if compounds.case_create(owner) != .Ok
		{
			return status;
		}
		handles = owner.bodies[:];
		s.extent, s.timestep = compounds.STEPS, compounds.TIMESTEP;
	case .Constraints:
		owner: ^constraints.Case = new(constraints.Case);
		if owner == nil
		{
			return .Out_Of_Memory;
		}
		s.owner = owner;
		if constraints.case_create(owner) != .Ok
		{
			return status;
		}
		handles = owner.bodies[:];
		s.extent, s.timestep = constraints.STEPS, constraints.TIMESTEP;
	case .Kinematic_Platform:
		owner: ^kinematic_platform.Case = new(kinematic_platform.Case);
		if owner == nil
		{
			return .Out_Of_Memory;
		}
		s.owner = owner;
		if kinematic_platform.case_create(owner) != .Ok
		{
			return status;
		}
		handles = owner.bodies[:];
		s.extent, s.timestep = kinematic_platform.STEPS, kinematic_platform.TIMESTEP;
	case .Per_Body_Gravity:
		owner: ^per_body_gravity.Case = new(per_body_gravity.Case);
		if owner == nil
		{
			return .Out_Of_Memory;
		}
		s.owner = owner;
		if per_body_gravity.case_create(owner) != .Ok
		{
			return status;
		}
		handles = owner.bodies[:];
		s.extent, s.timestep = per_body_gravity.STEPS, per_body_gravity.TIMESTEP;
	case .Continuous_Collision:
		owner: ^continuous_collision.Case = new(continuous_collision.Case);
		if owner == nil
		{
			return .Out_Of_Memory;
		}
		s.owner = owner;
		if continuous_collision.case_create(owner) != .Ok
		{
			return status;
		}
		handles = ([^]entasis.Body_Handle)(&owner.body)[:1];
		s.extent, s.timestep = continuous_collision.STEPS, continuous_collision.TIMESTEP;
	case .Substepping:
		owner: ^substepping.Case = new(substepping.Case);
		if owner == nil
		{
			return .Out_Of_Memory;
		}
		s.owner = owner;
		if substepping.case_create(owner) != .Ok
		{
			return status;
		}
		handles = owner.bodies[:];
		s.extent, s.timestep = substepping.STEPS, substepping.TIMESTEP;
	case .Gyroscope:
		owner: ^gyroscope.Case = new(gyroscope.Case);
		if owner == nil
		{
			return .Out_Of_Memory;
		}
		s.owner = owner;
		if gyroscope.case_create(owner) != .Ok
		{
			return status;
		}
		handles = ([^]entasis.Body_Handle)(&owner.body)[:1];
		s.extent, s.timestep = gyroscope.STEPS, gyroscope.TIMESTEP;
	case .Convex_Hulls:
		owner: ^convex_hulls.Case = new(convex_hulls.Case);
		if owner == nil
		{
			return .Out_Of_Memory;
		}
		s.owner = owner;
		if convex_hulls.case_create(owner) != .Ok
		{
			return status;
		}
		s.extent, s.axis = convex_hulls.STEPS, .Operation;
	case .Ray_Queries:
		owner: ^ray_queries.Case = new(ray_queries.Case);
		if owner == nil
		{
			return .Out_Of_Memory;
		}
		s.owner = owner;
		if ray_queries.case_create(owner) != .Ok
		{
			return status;
		}
		s.extent, s.axis = ray_queries.STEPS, .Operation;
	case .Sweep_Queries:
		owner: ^sweep_queries.Case = new(sweep_queries.Case);
		if owner == nil
		{
			return .Out_Of_Memory;
		}
		s.owner = owner;
		if sweep_queries.case_create(owner) != .Ok
		{
			return status;
		}
		s.extent, s.axis = sweep_queries.STEPS, .Operation;
	case .Batched_Queries:
		owner: ^batched_queries.Case = new(batched_queries.Case);
		if owner == nil
		{
			return .Out_Of_Memory;
		}
		s.owner = owner;
		if batched_queries.case_create(owner) != .Ok
		{
			return status;
		}
		s.extent, s.axis = batched_queries.STEPS, .Operation;
	case .Meshes:
		owner: ^meshes.Case = new(meshes.Case);
		if owner == nil
		{
			return .Out_Of_Memory;
		}
		s.owner = owner;
		if meshes.case_create(owner) != .Ok
		{
			return status;
		}
		handles = ([^]entasis.Body_Handle)(&owner.body)[:1];
		s.extent, s.timestep = meshes.STEPS, meshes.TIMESTEP;
	case .Background_Cooking:
		owner: ^background_cooking.Case = new(background_cooking.Case);
		if owner == nil
		{
			return .Out_Of_Memory;
		}
		s.owner = owner;
		if background_cooking.case_create(owner) != .Ok
		{
			return status;
		}
		s.extent, s.axis = background_cooking.STEPS, .Operation;
	case .Cloth:
		owner: ^cloth.Case = new(cloth.Case);
		if owner == nil
		{
			return .Out_Of_Memory;
		}
		s.owner = owner;
		if cloth.case_create(owner) != .Ok
		{
			return status;
		}
		handles = owner.bodies[:];
		s.extent, s.timestep = cloth.STEPS, cloth.TIMESTEP;
	case .Rope:
		owner: ^rope.Case = new(rope.Case);
		if owner == nil
		{
			return .Out_Of_Memory;
		}
		s.owner = owner;
		if rope.case_create(owner) != .Ok
		{
			return status;
		}
		handles = owner.bodies[:];
		s.extent, s.timestep = rope.STEPS, rope.TIMESTEP;
	case .External_Dispatcher:
		owner: ^external_dispatcher.Case = new(external_dispatcher.Case);
		if owner == nil
		{
			return .Out_Of_Memory;
		}
		s.owner = owner;
		if external_dispatcher.case_create(owner) != .Ok
		{
			return status;
		}
		handles = owner.bodies[:];
		s.extent, s.timestep = external_dispatcher.STEPS, external_dispatcher.TIMESTEP;
	case .Profiling_And_State_Export:
		owner: ^profiling_and_state_export.Case = new(profiling_and_state_export.Case);
		if owner == nil
		{
			return .Out_Of_Memory;
		}
		s.owner = owner;
		if profiling_and_state_export.case_create(owner) != .Ok
		{
			return status;
		}
		handles = owner.bodies[:];
		s.extent, s.timestep = profiling_and_state_export.STEPS, profiling_and_state_export.TIMESTEP;
	case .Triggers:
		owner: ^triggers.Case = new(triggers.Case);
		if owner == nil
		{
			return .Out_Of_Memory;
		}
		s.owner = owner;
		if triggers.case_create(owner) != .Ok
		{
			return status;
		}
		handles = ([^]entasis.Body_Handle)(&owner.body)[:1];
		s.extent, s.timestep = triggers.STEPS, triggers.TIMESTEP;
	case .Vehicle:
		owner: ^vehicle.Case = new(vehicle.Case);
		if owner == nil
		{
			return .Out_Of_Memory;
		}
		s.owner = owner;
		if vehicle.case_create(owner) != .Ok
		{
			return status;
		}
		handles = owner.bodies[:];
		s.extent, s.timestep = vehicle.STEPS, vehicle.TIMESTEP;
	case .Tank_Controller:
		owner: ^tank_controller.Case = new(tank_controller.Case);
		if owner == nil
		{
			return .Out_Of_Memory;
		}
		s.owner = owner;
		if tank_controller.case_create(owner) != .Ok
		{
			return status;
		}
		handles = owner.bodies[:];
		s.extent, s.timestep = tank_controller.STEPS, tank_controller.TIMESTEP;
	case .Character_Controller:
		owner: ^character_controller.Case = new(character_controller.Case);
		if owner == nil
		{
			return .Out_Of_Memory;
		}
		s.owner = owner;
		if character_controller.case_create(owner) != .Ok
		{
			return status;
		}
		handles = ([^]entasis.Body_Handle)(&owner.body)[:1];
		s.extent, s.timestep = character_controller.STEPS, character_controller.TIMESTEP;
	case .Custom_Shape:
		owner: ^custom_shape.Case = new(custom_shape.Case);
		if owner == nil
		{
			return .Out_Of_Memory;
		}
		s.owner = owner;
		if custom_shape.case_create(owner) != .Ok
		{
			return status;
		}
		bounds: entasis.Shape_Bounds;
		physics_status: entasis.Status;
		bounds, physics_status = entasis.shape_bounds(&owner.world, owner.shape);
		if physics_status != .Ok
		{
			return status;
		}
		status = observation.observation_reserve(&s.observation, geometries=1);
		if status != .Ok
		{
			return status;
		}
		append(&s.observation.packet.geometries, scene.Geometry{kind=.Sphere, size={owner.shape_value.radius, 0, 0},
			minimum=observation.copy_vector(bounds.min), maximum=observation.copy_vector(bounds.max)});
		append(&s.observation.shapes, observation.Shape_Binding{owner.shape, 0});
		s.extent, s.axis = custom_shape.STEPS, .Operation;
	case .Custom_Constraint:
		owner: ^custom_constraint.Case = new(custom_constraint.Case);
		if owner == nil
		{
			return .Out_Of_Memory;
		}
		s.owner = owner;
		if custom_constraint.case_create(owner) != .Ok
		{
			return status;
		}
		handles = ([^]entasis.Body_Handle)(&owner.body)[:1];
		s.extent, s.timestep = custom_constraint.STEPS, custom_constraint.TIMESTEP;
	case .Direct_Collision_Queries:
		owner: ^direct_collision_queries.Case = new(direct_collision_queries.Case);
		if owner == nil
		{
			return .Out_Of_Memory;
		}
		s.owner = owner;
		if direct_collision_queries.case_create(owner) != .Ok
		{
			return status;
		}
		s.extent, s.axis = direct_collision_queries.STEPS, .Operation;
	case .Impulses_Solver_Contacts:
		owner: ^impulses_solver_contacts.Case = new(impulses_solver_contacts.Case);
		if owner == nil
		{
			return .Out_Of_Memory;
		}
		s.owner = owner;
		if impulses_solver_contacts.case_create(owner) != .Ok
		{
			return status;
		}
		handles = ([^]entasis.Body_Handle)(&owner.body)[:1];
		s.extent, s.timestep, s.axis = impulses_solver_contacts.STEPS, impulses_solver_contacts.TIMESTEP, .Operation;
	case .Overlap_And_Events:
		owner: ^overlap_and_events.Case = new(overlap_and_events.Case);
		if owner == nil
		{
			return .Out_Of_Memory;
		}
		s.owner = owner;
		if overlap_and_events.case_create(owner) != .Ok
		{
			return status;
		}
		handles = ([^]entasis.Body_Handle)(&owner.body)[:1];
		s.extent, s.timestep = overlap_and_events.STEPS, overlap_and_events.TIMESTEP;
	case .Materials_And_Filtering:
		owner: ^materials_and_filtering.Case = new(materials_and_filtering.Case);
		if owner == nil
		{
			return .Out_Of_Memory;
		}
		s.owner = owner;
		if materials_and_filtering.case_create(owner) != .Ok
		{
			return status;
		}
		handles = ([^]entasis.Body_Handle)(&owner.body)[:1];
		s.extent, s.timestep = materials_and_filtering.STEPS, materials_and_filtering.TIMESTEP;
	case .Ragdoll:
		owner: ^ragdoll.Case = new(ragdoll.Case);
		if owner == nil
		{
			return .Out_Of_Memory;
		}
		s.owner = owner;
		if ragdoll.case_create(owner) != .Ok
		{
			return status;
		}
		handles = owner.bodies[:];
		s.extent, s.timestep = ragdoll.STEPS, ragdoll.TIMESTEP;
	}
	status = observation.observation_create(&s.observation, session_world(s), handles);
	if status != .Ok
	{
		return status;
	}
	#partial switch owner in s.owner
	{
	case ^sweep_queries.Case:
		status = observation.observation_add_shape(&s.observation, &owner.world, owner.moving_shape, scene.IDENTITY, {.Query});
		if status != .Ok
		{
			return status;
		}
	case ^overlap_and_events.Case:
		status = observation.observation_add_shape(&s.observation, &owner.world, owner.query_shape, scene.IDENTITY, {.Query});
		if status != .Ok
		{
			return status;
		}
	case ^direct_collision_queries.Case:
		status = observation.observation_add_shape(&s.observation, &owner.world, owner.sphere, observation.copy_pose(owner.pose), {.Query});
		if status != .Ok
		{
			return status;
		}
		status = observation.observation_add_shape(&s.observation, &owner.world, owner.box);
		if status != .Ok
		{
			return status;
		}
	}
	overlays: int = 4 if recipe.scenario == .Ray_Queries else 2;
	if recipe.scenario == .Batched_Queries
	{
		overlays = 38;
	}
	#partial switch owner in s.owner
	{
	case ^cloth.Case: overlays = len(owner.links);
	case ^planetary_gravity.Case: overlays = 3;
	case ^rope.Case: overlays = len(owner.bodies)-1;
	case ^ragdoll.Case: overlays = 2*len(ragdoll.LINKS);
	case ^direct_collision_queries.Case: overlays = 9;
	case ^character_controller.Case: overlays = 4;
	case ^vehicle.Case: overlays = 4;
	case ^tank_controller.Case: overlays = 6;
	case ^triggers.Case: overlays = 56;
	case ^impulses_solver_contacts.Case, ^overlap_and_events.Case: overlays = 36;
	}
	s.observation.overlay_capacity = overlays;
	metrics: int = 8 if recipe.scenario == .Fixed_Step_Loop else 5;
	status = observation.observation_reserve(&s.observation, instances=len(s.observation.packet.entities), overlays=overlays+s.observation.part_capacity, metrics=metrics);
	if status != .Ok
	{
		return status;
	}
	status = session_observe(s);
	if status == .Ok
	{
		s.state = .Paused;
	}
	return status;
}

session_step :: proc(s: ^Session, input: scene.Input = {}) -> scene.Status
{
	catalog: [Scenario]Descriptor = CATALOG;
	if u32(input.mode) > u32(scene.Input_Mode.Live) || scene.finite(input.x) != .Ok || scene.finite(input.y) != .Ok ||
		abs(input.x) > 1 || abs(input.y) > 1 || (input.mode == .Live && .Input not_in catalog[s.recipe.scenario].capabilities)
	{
		return .Invalid_Data;
	}
	if s.state == .Completed
	{
		return .End;
	}
	if s.state == .Failed || s.state == .Loading
	{
		return .Invalid_Data;
	}
	status: entasis.Status;
	replacement: []entasis.Body_Handle;
	effective_input: scene.Input = input;
	switch owner in s.owner
	{
	case ^falling_box.Case:
		status = falling_box.case_step(owner);
	case ^minimal_world.Case:
		status = minimal_world.case_step(owner);
	case ^fixed_step_loop.Case:
		status = fixed_step_loop.case_step(owner);
	case ^sleep_and_awaken.Case:
		status = sleep_and_awaken.case_step(owner);
	case ^box_pile_batch.Case:
		status = box_pile_batch.case_step(owner);
	case ^bulk_body_update.Case:
		status = bulk_body_update.case_step(owner);
	case ^planetary_gravity.Case:
		status = planetary_gravity.case_step(owner);
	case ^compounds.Case:
		status = compounds.case_step(owner);
	case ^constraints.Case:
		status = constraints.case_step(owner);
	case ^kinematic_platform.Case:
		status = kinematic_platform.case_step(owner);
	case ^per_body_gravity.Case:
		status = per_body_gravity.case_step(owner);
	case ^continuous_collision.Case:
		status = continuous_collision.case_step(owner);
		if owner.cycle != s.cycle
		{
			s.cycle = owner.cycle;
			replacement = ([^]entasis.Body_Handle)(&owner.body)[:1];
		}
	case ^substepping.Case:
		status = substepping.case_step(owner);
		if owner.cycle != s.cycle
		{
			s.cycle = owner.cycle;
			replacement = owner.bodies[:];
		}
	case ^gyroscope.Case:
		status = gyroscope.case_step(owner);
		if owner.cycle != s.cycle
		{
			s.cycle = owner.cycle;
			replacement = ([^]entasis.Body_Handle)(&owner.body)[:1];
		}
	case ^convex_hulls.Case:
		status = convex_hulls.case_step(owner);
		if status == .Ok
		{
			visual_status: scene.Status = observation.observation_add_shape(&s.observation, &owner.world, owner.shape);
			if visual_status != .Ok
			{
				s.state = .Failed;
				return visual_status;
			}
		}
	case ^ray_queries.Case:
		status = ray_queries.case_step(owner);
	case ^batched_queries.Case:
		status = batched_queries.case_step(owner);
	case ^meshes.Case:
		status = meshes.case_step(owner);
	case ^cloth.Case:
		status = cloth.case_step(owner);
	case ^rope.Case:
		status = rope.case_step(owner);
	case ^external_dispatcher.Case:
		status = external_dispatcher.case_step(owner);
		if owner.cycle != s.cycle
		{
			s.cycle = owner.cycle;
			replacement = owner.bodies[:];
		}
	case ^profiling_and_state_export.Case:
		status = profiling_and_state_export.case_step(owner);
	case ^materials_and_filtering.Case:
		status = materials_and_filtering.case_step(owner);
		if owner.cycle != s.cycle
		{
			s.cycle = owner.cycle;
			replacement = ([^]entasis.Body_Handle)(&owner.body)[:1];
		}
	case ^triggers.Case:
		status = triggers.case_step(owner);
		if owner.cycle != s.cycle
		{
			s.cycle = owner.cycle;
			replacement = ([^]entasis.Body_Handle)(&owner.body)[:1];
		}
	case ^ragdoll.Case:
		status = ragdoll.case_step(owner);
	case ^impulses_solver_contacts.Case:
		status = impulses_solver_contacts.case_step(owner);
	case ^custom_shape.Case:
		status = custom_shape.case_step(owner);
	case ^custom_constraint.Case:
		status = custom_constraint.case_step(owner);
	case ^character_controller.Case:
		if input.mode == .Scripted
		{
			effective_input.x, effective_input.y = 1, 0;
		}
		status = character_controller.case_step(owner, {effective_input.x, effective_input.y});
	case ^vehicle.Case:
		steering: f32 = input.y*0.6;
		if input.mode == .Scripted
		{
			effective_input.x, effective_input.y, steering = 1, 0.3, 0.18;
		}
		status = vehicle.case_control(owner, effective_input.x, steering);
		if status == .Ok
		{
			status = vehicle.case_step(owner);
		}
	case ^tank_controller.Case:
		if input.mode == .Scripted
		{
			effective_input.x, effective_input.y = 1, 0.4;
		}
		status = tank_controller.case_control(owner, {effective_input.x, effective_input.y});
		if status == .Ok
		{
			status = tank_controller.case_step(owner);
		}
	case ^overlap_and_events.Case:
		status = overlap_and_events.case_step(owner);
	case ^direct_collision_queries.Case:
		status = direct_collision_queries.case_step(owner);
		if status == .Ok
		{
			s.observation.previews[0].pose = observation.copy_pose(owner.pose);
			if owner.tick == 3
			{
				visual_status: scene.Status = observation.observation_remove(&s.observation, .Shape, i32(owner.box.packed));
				if visual_status == .Ok
				{
					_, visual_status = observation.observation_add(&s.observation, &owner.world, .Static, owner.static.value);
				}
				if visual_status != .Ok
				{
					s.state = .Failed;
					return visual_status;
				}
			}
		}
	case ^background_cooking.Case:
		status = background_cooking.case_step(owner);
		if status == .Ok
		{
			visual_status: scene.Status;
			_, visual_status = observation.observation_add(&s.observation, &owner.world, .Static, owner.static.value);
			if visual_status != .Ok
			{
				s.state = .Failed;
				return visual_status;
			}
		}
	case ^sweep_queries.Case:
		status = sweep_queries.case_step(owner);
		if status == .Ok
		{
			s.observation.previews[0].pose.position.x = 10*owner.hit.sweep.t0;
		}
	}
	if status != .Ok
	{
		s.state = .Failed;
		return .Invalid_Data;
	}
	s.tick += 1;
	s.phase = .Active;
	if len(replacement) > 0
	{
		observation.observation_clear(&s.observation);
		visual_status: scene.Status = observation.observation_create(&s.observation, session_world(s), replacement);
		if visual_status != .Ok
		{
			s.state = .Failed;
			return visual_status;
		}
		s.phase = .Reset;
	}
	if s.tick == s.extent
	{
		s.state = .Completed;
		s.phase = .Completed;
		#partial switch owner in s.owner
		{
		case ^falling_box.Case:
			if falling_box.case_check(owner) != .Ok
			{
				s.state = .Failed;
				return .Invalid_Data;
			}
		case ^fixed_step_loop.Case:
			if fixed_step_loop.case_check(owner) != .Ok
			{
				s.state = .Failed;
				return .Invalid_Data;
			}
		case ^box_pile_batch.Case:
			if box_pile_batch.case_check(owner) != .Ok
			{
				s.state = .Failed;
				return .Invalid_Data;
			}
		case ^planetary_gravity.Case:
			if planetary_gravity.case_check(owner) != .Ok
			{
				s.state = .Failed;
				return .Invalid_Data;
			}
		case ^compounds.Case:
			if compounds.case_check(owner) != .Ok
			{
				s.state = .Failed;
				return .Invalid_Data;
			}
		case ^meshes.Case:
			if meshes.case_check(owner) != .Ok
			{
				s.state = .Failed;
				return .Invalid_Data;
			}
		case ^cloth.Case:
			if cloth.case_check(owner) != .Ok
			{
				s.state = .Failed;
				return .Invalid_Data;
			}
		case ^rope.Case:
			if rope.case_check(owner) != .Ok
			{
				s.state = .Failed;
				return .Invalid_Data;
			}
		case ^profiling_and_state_export.Case:
			if profiling_and_state_export.case_check(owner) != .Ok
			{
				s.state = .Failed;
				return .Invalid_Data;
			}
		case ^materials_and_filtering.Case:
			if materials_and_filtering.case_check(owner) != .Ok
			{
				s.state = .Failed;
				return .Invalid_Data;
			}
		case ^ragdoll.Case:
			if ragdoll.case_check(owner) != .Ok
			{
				s.state = .Failed;
				return .Invalid_Data;
			}
		case ^character_controller.Case:
			if character_controller.case_check(owner) != .Ok
			{
				s.state = .Failed;
				return .Invalid_Data;
			}
		case ^vehicle.Case:
			if vehicle.case_check(owner) != .Ok
			{
				s.state = .Failed;
				return .Invalid_Data;
			}
		case ^tank_controller.Case:
			if tank_controller.case_check(owner) != .Ok
			{
				s.state = .Failed;
				return .Invalid_Data;
			}
		case ^constraints.Case:
			if constraints.case_check(owner) != .Ok
			{
				s.state = .Failed;
				return .Invalid_Data;
			}
		case ^kinematic_platform.Case:
			if kinematic_platform.case_check(owner) != .Ok
			{
				s.state = .Failed;
				return .Invalid_Data;
			}
		case ^per_body_gravity.Case:
			if per_body_gravity.case_check(owner) != .Ok
			{
				s.state = .Failed;
				return .Invalid_Data;
			}
		case ^continuous_collision.Case:
			if continuous_collision.case_check(owner) != .Ok
			{
				s.state = .Failed;
				return .Invalid_Data;
			}
		case ^substepping.Case:
			if substepping.case_check(owner) != .Ok
			{
				s.state = .Failed;
				return .Invalid_Data;
			}
		case ^gyroscope.Case:
			if gyroscope.case_check(owner) != .Ok
			{
				s.state = .Failed;
				return .Invalid_Data;
			}
		}
	}
	s.observation.packet.frame.input = effective_input;
	return session_observe(s);
}
