package simulation_tests

import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

resolution_reference_ray :: proc "contextless" (
	simulation: ^physics.Simulation, ray: physics.Tree_Ray,
	collector: ^physics.Ray_Query_Collector, pool: ^util.Buffer_Pool,
) -> physics.Physics_Status
{
	shapes := physics.simulation_shape_registry(simulation);
	active_context, static_context: physics.Simulation_Query_Target_Context;
	maximum_t, status := physics.query_ray_resolved(
		shapes, &simulation.broad_phase.active_tree,
		physics.simulation_query_resolver(&active_context, simulation, simulation.broad_phase.active_leaves),
		ray, collector, pool,
	);
	if status != .Ok
	{
		return status;
	}
	bounded := ray;
	bounded.maximum_t = maximum_t;
	_, status = physics.query_ray_resolved(
		shapes, &simulation.broad_phase.static_tree,
		physics.simulation_query_resolver(&static_context, simulation, simulation.broad_phase.static_leaves),
		bounded, collector, pool,
	);
	return status;
}

resolution_filter_even :: proc "contextless" (
	user_context: rawptr, target_id: i32,
) -> physics.Query_Filter_Result
{
	_ = user_context;
	if target_id & 1 != 0
	{
		return .Reject;
	}
	return .Allow;
}

resolution_clip_ray :: proc "contextless" (
	user_context: rawptr, ray: physics.Tree_Ray,
	hit: ^physics.Ray_Query_Hit, maximum_t: ^f32,
) -> physics.Physics_Status
{
	_, _ = user_context, ray;
	maximum_t^ = min(maximum_t^, hit.t);
	return .Ok;
}

@(test)
simulation_direct_ray_resolution_matches_generic_after_sleep_and_handle_reuse :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	simulation := allocate_simulation(t);
	if simulation == nil
	{
		return;
	}
	defer free_simulation(t, simulation);
	description := small_description(&pool);
	description.default_pose_context.gravity = {};
	testing.expect_value(t, physics.simulation_create(simulation, &description).status, physics.Physics_Status.Ok);
	defer physics.simulation_destroy(simulation);
	simulation.sleeper.tested_fraction_per_frame = 1;
	simulation.sleeper.target_slept_fraction = 1;
	simulation.sleeper.target_traversed_fraction = 1;
	sphere := physics.Sphere{0.5};
	box := physics.Box{0.7, 0.5, 0.6};
	capsule := physics.Capsule{0.4, 0.3};
	cylinder := physics.Cylinder{0.5, 0.6};
	indices: [4]physics.Typed_Index;
	statuses: [4]physics.Physics_Status;
	indices[0], statuses[0] = physics.shape_registry_add(&simulation.shapes, physics.SPHERE_TYPE_ID, &sphere);
	indices[1], statuses[1] = physics.shape_registry_add(&simulation.shapes, physics.BOX_TYPE_ID, &box);
	indices[2], statuses[2] = physics.shape_registry_add(&simulation.shapes, physics.CAPSULE_TYPE_ID, &capsule);
	indices[3], statuses[3] = physics.shape_registry_add(&simulation.shapes, physics.CYLINDER_TYPE_ID, &cylinder);
	for status in statuses
	{
		testing.expect_value(t, status, physics.Physics_Status.Ok);
	}
	bodies: [12]physics.Body_Handle;
	statics: [12]physics.Static_Handle;
	poses: [12]physics.Rigid_Pose;
	for index in 0 ..< len(poses)
	{
		poses[index] = {
			position={f32(index % 4) * 3, f32(index / 4) * 3, 0},
			orientation=util.quaternion_from_axis_angle({0.36, 0.48, 0.8}, 0.17 * f32(index)),
		};
		if index % 3 == 0
		{
			static_description := physics.Static_Description{pose=poses[index], shape=indices[index % 4]};
			status: physics.Physics_Status;
			statics[index], status = physics.simulation_add_static(simulation, &static_description);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
		}
		else
		{
			body := dynamic_body(indices[index % 4]);
			body.pose = poses[index];
			body.velocity = {};
			body.activity.sleep_threshold = -1;
			if index % 3 == 2
			{
				body.local_inertia = {};
			}
			if index == 1
			{
				body.activity = {sleep_threshold=1, minimum_timestep_count_under_threshold=1};
			}
			status: physics.Physics_Status;
			bodies[index], status = physics.simulation_add_body(simulation, &body);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
		}
	}
	directions := [6]util.Vector3{
		{1, 0.37, 0.21}, {-0.81, 0.46, -0.35}, {0.42, -0.73, 0.54},
		{-0.58, -0.31, 0.75}, {1, 0, 0}, {0, 0, 1},
	};
	for phase in 0 ..< 3
	{
		for pose in poses
		{
			for direction in directions
			{
				ray := physics.Tree_Ray{
					origin=util.vector3_subtract(pose.position, util.vector3_scale(direction, 4)),
					direction=direction, maximum_t=30,
				};
				for mode in 0 ..< 4
				{
					actual_hits, reference_hits: [16]physics.Ray_Query_Hit;
					actual := physics.Ray_Query_Collector{hits={memory=&actual_hits[0], length=16, id=-1}, mode=.All};
					if mode == 0
					{
						actual.mode = .Earliest;
					}
					else if mode == 2
					{
						actual.callbacks.hit = resolution_clip_ray;
					}
					else if mode == 3
					{
						actual.callbacks.filter.allow_target = resolution_filter_even;
					}
					reference := actual;
					reference.hits.memory = &reference_hits[0];
					actual_status := physics.simulation_ray_query(simulation, ray, &actual, &pool);
					reference_status := resolution_reference_ray(simulation, ray, &reference, &pool);
					testing.expect_value(t, actual_status, reference_status);
					testing.expect_value(t, actual_status, physics.Physics_Status.Ok);
					testing.expect_value(t, actual.count, reference.count);
					for index in 0 ..< min(actual.count, reference.count)
					{
						testing.expect_value(t, actual_hits[index], reference_hits[index]);
					}
				}
			}
		}
		if phase == 0
		{
			for _ in 0 ..< 3
			{
				testing.expect_value(t, physics.simulation_timestep(simulation, 1.0 / 60.0), physics.Physics_Status.Ok);
			}
			location, status := physics.bodies_resolve(&simulation.bodies, bodies[1]);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
			testing.expect(t, location.set_index > 0);
		}
		else if phase == 1
		{
			testing.expect_value(t, physics.simulation_remove_body(simulation, bodies[5]), physics.Physics_Status.Ok);
			body := dynamic_body(indices[2]);
			body.pose = poses[5];
			body.velocity = {};
			_, status := physics.simulation_add_body(simulation, &body);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
			testing.expect_value(t, physics.simulation_remove_static_without_awakening_bodies(simulation, statics[0]), physics.Physics_Status.Ok);
			static_description := physics.Static_Description{shape=indices[3], pose=poses[0]};
			_, status = physics.simulation_add_static(simulation, &static_description);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
		}
	}
}
