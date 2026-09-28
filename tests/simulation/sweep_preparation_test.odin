package simulation_tests

import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

// only the registry owned by this test installs this callback. no runtime instrumentation
sweep_preparation_bounds_calls: int;
sweep_preparation_bounds_error: physics.Physics_Status;
SWEEP_PREPARATION_EXPECTED_CALLS :: #config(SWEEP_PREPARATION_EXPECTED_CALLS, 1);

sweep_preparation_count_bounds :: proc "contextless" (
	shape: rawptr, orientation: util.Quaternion, registry: ^physics.Shape_Registry,
) -> (physics.Shape_Bounds, physics.Physics_Status)
{
	sweep_preparation_bounds_calls += 1;
	if sweep_preparation_bounds_error != .Ok
	{
		return {}, sweep_preparation_bounds_error;
	}
	return physics.sphere_bounds_proc(shape, orientation, registry);
}

// independent population dispatch retained as a test oracle. each public query
// prepares its own shape bounds and context, as before shared simulation setup
sweep_preparation_reference :: proc(
	simulation: ^physics.Simulation, shape: physics.Typed_Index,
	pose: physics.Rigid_Pose, velocity: physics.Body_Velocity,
	maximum_t: f32, collector: ^physics.Sweep_Query_Collector, pool: ^util.Buffer_Pool,
) -> physics.Physics_Status
{
	active, statics: physics.Simulation_Query_Target_Context;
	shapes := physics.simulation_shape_registry(simulation);
	active_maximum, status := physics.query_sweep_resolved(
		shape, pose, velocity, &simulation.broad_phase.active_tree,
		physics.simulation_query_resolver(&active, simulation, simulation.broad_phase.active_leaves),
		maximum_t, 1e-4, 1e-4, 32, shapes, &simulation.collision_tasks,
		&simulation.sweep_tasks, collector, pool,
	);
	if status != .Ok
	{
		return status;
	}
	_, status = physics.query_sweep_resolved(
		shape, pose, velocity, &simulation.broad_phase.static_tree,
		physics.simulation_query_resolver(&statics, simulation, simulation.broad_phase.static_leaves),
		active_maximum, 1e-4, 1e-4, 32, shapes, &simulation.collision_tasks,
		&simulation.sweep_tasks, collector, pool,
	);
	return status;
}

@(test)
sweep_preparation_is_shared_and_preserves_population_results_and_errors :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	for population in 0 ..< 4
	{
		simulation := allocate_simulation(t);
		if simulation == nil
		{
			return;
		}
		description := small_description(&pool);
		testing.expect_value(t, physics.simulation_create(simulation, &description).status, physics.Physics_Status.Ok);
		sphere := physics.Sphere{radius=0.375};
		box := physics.Box{half_width=0.5, half_height=0.75, half_length=0.625};
		sphere_shape, sphere_status := physics.shape_registry_add(&simulation.shapes, physics.SPHERE_TYPE_ID, &sphere);
		box_shape, box_status := physics.shape_registry_add(&simulation.shapes, physics.BOX_TYPE_ID, &box);
		testing.expect_value(t, sphere_status, physics.Physics_Status.Ok);
		testing.expect_value(t, box_status, physics.Physics_Status.Ok);
		if population & 1 != 0
		{
			body := dynamic_body(box_shape);
			body.pose.position = {2.5, 0, 0};
			body.velocity = {};
			_, status := physics.simulation_add_body(simulation, &body);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
		}
		if population & 2 != 0
		{
			body := physics.Static_Description{
				pose={orientation=util.quaternion_identity(), position={0, 0, 0}}, shape=box_shape,
			};
			_, status := physics.simulation_add_static(simulation, &body);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
		}
		simulation.shapes.batches[physics.SPHERE_TYPE_ID].metadata.bounds = sweep_preparation_count_bounds;
		query_shapes := [2]physics.Typed_Index{sphere_shape, box_shape};
		modes := [2]physics.Query_Collection_Mode{.Earliest, .All};
		for query_shape, shape_index in query_shapes
		{
			for mode in modes
			{
				for sample in 0 ..< 8
				{
					pose := physics.rigid_pose_identity();
					pose.position = {-4, 0, 0};
					velocity := physics.Body_Velocity{linear={1, 0, 0}};
					maximum_t := f32(12);
					if sample == 1
					{
						pose.position = {6, 0, 0};
						velocity.linear = {-1, 0, 0};
					}
					if sample == 2
					{
						pose.position.y = 10;
					}
					if sample == 3
					{
						pose.orientation = util.quaternion_from_axis_angle({0.36, 0.48, 0.8}, 0.37);
						velocity.linear = {1, 0.05, -0.03};
						velocity.angular = {0.3, -0.2, 0.1};
					}
					if sample == 4
					{
						maximum_t = 0;
					}
					if sample == 5
					{
						maximum_t = -1;
					}
					input_shape := query_shape;
					if sample == 6
					{
						input_shape = {};
					}
					sweep_preparation_bounds_error = .Ok;
					if sample == 7
					{
						sweep_preparation_bounds_error = .Invalid_Description;
					}
					reference_hits, actual_hits: [8]physics.Sweep_Query_Hit;
					reference := physics.Sweep_Query_Collector{hits={memory=&reference_hits[0], length=8, id=-1}, mode=mode};
					actual := physics.Sweep_Query_Collector{hits={memory=&actual_hits[0], length=8, id=-1}, mode=mode};
					reference_status := sweep_preparation_reference(simulation, input_shape, pose, velocity, maximum_t, &reference, &pool);
					sweep_preparation_bounds_calls = 0;
					actual_status := physics.simulation_sweep_query(
						simulation, input_shape, pose, velocity, maximum_t, 1e-4, 1e-4, 32, &actual, &pool,
					);
					if shape_index == 0 && sample == 0
					{
						testing.expect_value(t, sweep_preparation_bounds_calls, SWEEP_PREPARATION_EXPECTED_CALLS);
					}
					testing.expect_value(t, actual_status, reference_status);
					testing.expect_value(t, actual.count, reference.count);
					for hit_index in 0 ..< min(actual.count, reference.count)
					{
						testing.expect_value(t, actual_hits[hit_index], reference_hits[hit_index]);
					}
				}
			}
		}
		sweep_preparation_bounds_error = .Ok;
		testing.expect_value(t, physics.simulation_destroy(simulation), physics.Physics_Status.Ok);
		free_simulation(t, simulation);
	}
}
