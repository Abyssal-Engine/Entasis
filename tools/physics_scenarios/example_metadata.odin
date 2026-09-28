package physics_scenarios

import "core:fmt"
import entasis "entasis:entasis"
import scene "../physics_scene"
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

example_fixture_size :: proc(scenario: Scenario) -> u64
{
	#partial switch scenario
	{
	case .Falling_Box: return size_of(falling_box.Case);
	case .Minimal_World: return size_of(minimal_world.Case);
	case .Fixed_Step_Loop: return size_of(fixed_step_loop.Case);
	case .Sleep_And_Awaken: return size_of(sleep_and_awaken.Case);
	case .Box_Pile_Batch: return size_of(box_pile_batch.Case);
	case .Bulk_Body_Update: return size_of(bulk_body_update.Case);
	case .Planetary_Gravity: return size_of(planetary_gravity.Case);
	case .Compounds: return size_of(compounds.Case);
	case .Constraints: return size_of(constraints.Case);
	case .Kinematic_Platform: return size_of(kinematic_platform.Case);
	case .Per_Body_Gravity: return size_of(per_body_gravity.Case);
	case .Continuous_Collision: return size_of(continuous_collision.Case);
	case .Substepping: return size_of(substepping.Case);
	case .Gyroscope: return size_of(gyroscope.Case);
	case .Convex_Hulls: return size_of(convex_hulls.Case);
	case .Ray_Queries: return size_of(ray_queries.Case);
	case .Sweep_Queries: return size_of(sweep_queries.Case);
	case .Batched_Queries: return size_of(batched_queries.Case);
	case .Meshes: return size_of(meshes.Case);
	case .Background_Cooking: return size_of(background_cooking.Case);
	case .Cloth: return size_of(cloth.Case);
	case .Rope: return size_of(rope.Case);
	case .External_Dispatcher: return size_of(external_dispatcher.Case);
	case .Profiling_And_State_Export: return size_of(profiling_and_state_export.Case);
	case .Materials_And_Filtering: return size_of(materials_and_filtering.Case);
	case .Ragdoll: return size_of(ragdoll.Case);
	case .Direct_Collision_Queries: return size_of(direct_collision_queries.Case);
	case .Impulses_Solver_Contacts: return size_of(impulses_solver_contacts.Case);
	case .Overlap_And_Events: return size_of(overlap_and_events.Case);
	case .Custom_Shape: return size_of(custom_shape.Case);
	case .Custom_Constraint: return size_of(custom_constraint.Case);
	case .Character_Controller: return size_of(character_controller.Case);
	case .Vehicle: return size_of(vehicle.Case);
	case .Tank_Controller: return size_of(tank_controller.Case);
	case .Triggers: return size_of(triggers.Case);
	}
	return 0;
}

example_settings :: proc(s: ^Session, buffer: []u8) -> (string, scene.Availability, scene.Status)
{
	description: entasis.World_Description;
	#partial switch owner in s.owner
	{
	case ^falling_box.Case: description = owner.description;
	case ^minimal_world.Case: description = owner.description;
	case ^fixed_step_loop.Case: description = owner.description;
	case ^sleep_and_awaken.Case: description = owner.description;
	case ^box_pile_batch.Case: description = owner.description;
	case ^bulk_body_update.Case: description = owner.description;
	case ^planetary_gravity.Case: description = owner.description;
	case ^compounds.Case: description = owner.description;
	case ^constraints.Case: description = owner.description;
	case ^kinematic_platform.Case: description = owner.description;
	case ^per_body_gravity.Case: description = owner.description;
	case ^continuous_collision.Case: description = owner.description;
	case ^substepping.Case: description = owner.description;
	case ^gyroscope.Case: description = owner.description;
	case ^convex_hulls.Case: description = owner.description;
	case ^ray_queries.Case: description = owner.description;
	case ^sweep_queries.Case: description = owner.description;
	case ^batched_queries.Case: description = owner.description;
	case ^meshes.Case: description = owner.description;
	case ^background_cooking.Case: description = owner.description;
	case ^cloth.Case: description = owner.description;
	case ^rope.Case: description = owner.description;
	case ^external_dispatcher.Case: description = owner.description;
	case ^profiling_and_state_export.Case: description = owner.description;
	case ^materials_and_filtering.Case: description = owner.description;
	case ^ragdoll.Case: description = owner.description;
	case ^direct_collision_queries.Case: description = owner.description;
	case ^impulses_solver_contacts.Case: description = owner.description;
	case ^overlap_and_events.Case: description = owner.description;
	case ^custom_shape.Case: description = owner.description;
	case ^custom_constraint.Case: description = owner.description;
	case ^character_controller.Case: description = owner.description;
	case ^vehicle.Case: description = owner.description;
	case ^tank_controller.Case: description = owner.description;
	case ^triggers.Case: description = owner.description;
	}
	policy: string = "built-in";
	detail_buffer: [1024]u8;
	#partial switch owner in s.owner
	{
	case ^planetary_gravity.Case:
		description.damping = {owner.gravity_policy.linear_damping, owner.gravity_policy.angular_damping};
		policy = fmt.bprintf(detail_buffer[:], "inverse-square;center=%.9g,%.9g,%.9g;strength=%.9g",
			owner.gravity_policy.center.x, owner.gravity_policy.center.y, owner.gravity_policy.center.z, owner.gravity_policy.gravity);
	case ^per_body_gravity.Case:
		description.damping = {owner.gravity_policy.linear_damping, owner.gravity_policy.angular_damping};
		values: [2]entasis.Vector3 = per_body_gravity.GRAVITIES;
		policy = fmt.bprintf(detail_buffer[:], "per-body-gravity=%.9g,%.9g,%.9g|%.9g,%.9g,%.9g",
			values[0].x, values[0].y, values[0].z, values[1].x, values[1].y, values[1].z);
	case ^gyroscope.Case:
		description.gravity = owner.pose_policy.gravity;
		description.damping = {owner.pose_policy.linear_damping, owner.pose_policy.angular_damping};
		policy = "cycles=nonconserving,gyroscopic-torque;steps-per-cycle=120;reset-between-cycles=1;initial-angular-velocity=0.7,1.1,2.3";
	case ^materials_and_filtering.Case:
		description.gravity = owner.pose_policy.gravity;
		description.damping = {owner.pose_policy.linear_damping, owner.pose_policy.angular_damping};
		policy = "cycles=friction-0.05,friction-1,recovery-0.1,recovery-4;steps-per-cycle=180,180,1,1;reset-between-cycles=1;friction-gravity=0,-9.81,0;recovery-gravity=0,0,0;layers=all";
	case ^ragdoll.Case:
		description.gravity = owner.pose_policy.gravity;
		description.damping = {owner.pose_policy.linear_damping, owner.pose_policy.angular_damping};
		policy = "layer-material;allow=ragdoll-ground;deny=ragdoll-ragdoll;links=ball-socket,swing-limit,twist-limit";
	case ^vehicle.Case:
		description.gravity = owner.pose_policy.gravity;
		description.damping = {owner.pose_policy.linear_damping, owner.pose_policy.angular_damping};
		policy = "layer-material;deny=vehicle-vehicle;material=1.4,2,30,1;wheels=4;input=throttle,steering";
	case ^tank_controller.Case:
		description.gravity = owner.pose_policy.gravity;
		description.damping = {owner.pose_policy.linear_damping, owner.pose_policy.angular_damping};
		policy = fmt.bprintf(detail_buffer[:], "layer-material;deny=tank-tank;wheels=%d;input=throttle,steering", len(owner.bodies)-1);
	case ^substepping.Case:
		policy = "cycles=substeps-1,substeps-4;steps-per-cycle=30;reset-between-cycles=1";
	case ^continuous_collision.Case:
		policy = "cycles=discrete,continuous;steps-per-cycle=1;reset-between-cycles=1";
	case ^external_dispatcher.Case:
		policy = fmt.bprintf(detail_buffer[:], "cycles=included,external;external-workers=%d;steps-per-cycle=30;reset-between-cycles=1", owner.interface.worker_count);
	case ^triggers.Case:
		policy = "cycles=whole-sensors,nested-mixed-parts;steps-per-cycle=180,1;reset-between-cycles=1;stepper-max-steps=4;events=parent,parts,contacts;single-drain=1";
	case ^fixed_step_loop.Case:
		values: [4]f32 = fixed_step_loop.FRAME_TIMES;
		policy = fmt.bprintf(detail_buffer[:], "fixed-stepper;max-steps=4;elapsed=%.9g,%.9g,%.9g,%.9g", values[0], values[1], values[2], values[3]);
	case ^sleep_and_awaken.Case:
		policy = "operations=8-steps,awaken;activity=1,1";
	case ^custom_shape.Case:
		policy = fmt.bprintf(detail_buffer[:], "registered-native-sphere;radius=%.9g;query=ray", owner.shape_value.radius);
	case ^custom_constraint.Case:
		policy = fmt.bprintf(detail_buffer[:], "custom-constraint;target-x-speed=%.9g", owner.target.target_speed);
	case ^character_controller.Case:
		policy = "sweep-controller;input=horizontal-movement;authored-ground-sweep";
	}
	if len(policy) == len(detail_buffer)
	{
		return "", .Unavailable, .Invalid_Data;
	}
	catalog: [Scenario]Descriptor = CATALOG;
	settings: string = fmt.bprintf(buffer,
		"fixture=%s;policy=%s;input=%v;axis=%v;operations=%d;timestep=%.9g;current-cycle=%d;workers=%d;gravity=%.9g,%.9g,%.9g;damping=%.9g,%.9g;velocity-iterations=%d;substeps=%d;fallback-threshold=%d;profiling=%d",
		catalog[s.recipe.scenario].id, policy, s.recipe.input, s.axis, s.extent, s.timestep, s.cycle,
		description.threading.worker_count, description.gravity.x, description.gravity.y, description.gravity.z,
		description.damping.linear, description.damping.angular, description.solve.velocity_iterations, description.solve.substeps,
		description.solve.fallback_batch_threshold, 1 if description.profiling else 0);
	if len(settings) == len(buffer)
	{
		return "", .Unavailable, .Invalid_Data;
	}
	return settings, .Available, .Ok;
}
