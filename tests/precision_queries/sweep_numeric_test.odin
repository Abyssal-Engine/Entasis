package precision_queries_tests

import "core:testing"
import "core:math"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

@(test)
global_sweep_two_millimeter_sphere_oracle :: proc(t: ^testing.T)
{
	cases: int;
	for origin in ([4]f64{0, 1e4, 1e6, 1e9})
	{
		for population in Overlap_Population
		{
			scene: Scene;
			scene_initialize(t, &scene);
			shape: physics.Typed_Index = add_shape(t, &scene, .Sphere);
			center: physics.Global_Position = {origin + 0.125, -origin + 0.25, origin - 0.375};
			reference: physics.Collidable_Reference;
			if population == .Static
			{
				handle: physics.Static_Handle = add_static(t, &scene, shape, center);
				reference, _ = physics.collidable_reference_create(.Static, int(handle.value));
			}
			else
			{
				mobility: physics.Body_Mobility = .Kinematic if population == .Kinematic else .Dynamic;
				handle: physics.Body_Handle = add_body(t, &scene, shape, center, mobility);
				reference, _ = physics.collidable_reference_create(mobility, int(handle.value));
				if population == .Sleeping
				{
					sleep_body(t, &scene, handle);
				}
			}
			stack: [128]i32;
			hits: [2]entasis.Global_Sweep_Hit;
			for axis in 0 ..< 3
			{
				for direction in ([2]f64{-1, 1})
				{
					for gap in ([2]f64{-0.002, 0.002})
					{
						start: [3]f64 = {center.x, center.y, center.z};
						start[axis] -= direction * (3 + gap);
						motion: [3]f32;
						motion[axis] = f32(direction);
						pose: entasis.Global_Pose = {position={start[0], start[1], start[2]}, orientation=util.quaternion_identity()};
						velocity: entasis.Body_Velocity = {linear={motion[0], motion[1], motion[2]}};
						count: int;
						status: entasis.Status;
						count, status = entasis.sweep_global_all_with_context(&scene.query, shape, pose, velocity, 2, hits[:], stack[:]);
						testing.expect_value(t, status, entasis.Status.Ok);
						state: entasis.Query_Hit_State;
						state, status = entasis.sweep_global_any_with_context(&scene.query, shape, pose, velocity, 2, stack[:]);
						testing.expect_value(t, status, entasis.Status.Ok);
						closest: entasis.Global_Sweep_Hit;
						closest, status = entasis.sweep_global_closest_with_context(&scene.query, shape, pose, velocity, 2, stack[:]);
						if gap > 0
						{
							testing.expectf(t, count == 0 && state == .Miss && status == .Not_Found,
								"origin %v population %v axis %v direction %v count %v state %v status %v", origin, population, axis, direction, count, state, status);
						}
						else
						{
							testing.expect_value(t, status, entasis.Status.Ok);
							testing.expect_value(t, count, 1);
							testing.expect_value(t, state, entasis.Query_Hit_State.Hit);
							testing.expect_value(t, closest, hits[0]);
							testing.expect_value(t, closest.collidable, reference);
							centers: [3]f64 = {center.x, center.y, center.z};
							expected_time: f64 = abs(centers[axis] - start[axis]) - 1;
							testing.expectf(t, abs(f64(closest.sweep.t1) - expected_time) <= 0.00002,
								"time %v expected %v", closest.sweep.t1, expected_time);
							centers[axis] -= direction * 0.5;
							expect_position(t, closest.sweep.location, {centers[0], centers[1], centers[2]}, 0.00002);
							expected_normal: [3]f32;
							expected_normal[axis] = -f32(direction);
							testing.expect_value(t, closest.sweep.normal, util.Vector3{expected_normal[0], expected_normal[1], expected_normal[2]});
						}
						cases += 1;
					}
				}
			}
			scene_dispose(t, &scene);
		}
	}
	testing.expect_value(t, cases, 192);
}

expect_sweep_local_result :: proc(t: ^testing.T, actual: entasis.Global_Sweep_Hit, expected: entasis.Sweep_Hit, origin: physics.Global_Position)
{
	testing.expect_value(t, actual.collidable, expected.collidable);
	testing.expect_value(t, actual.sweep.state, expected.sweep.state);
	testing.expect_value(t, actual.sweep.t0, expected.sweep.t0);
	testing.expect_value(t, actual.sweep.t1, expected.sweep.t1);
	testing.expect_value(t, actual.sweep.normal, expected.sweep.normal);
	testing.expect_value(t, actual.sweep.child_a, expected.sweep.child_a);
	testing.expect_value(t, actual.sweep.child_b, expected.sweep.child_b);
	expect_position(t, actual.sweep.location, {
		origin.x + f64(expected.sweep.location.x), origin.y + f64(expected.sweep.location.y), origin.z + f64(expected.sweep.location.z),
	}, 0);
}

@(test)
global_sweep_builtin_routes_and_rotations_match_local_kernels :: proc(t: ^testing.T)
{
	cases, contacts: int;
	for target_kind in Shape_Kind
	{
		near, far: Scene;
		scene_initialize(t, &near);
		scene_initialize(t, &far);
		origin: physics.Global_Position = {1e9 + 0.125, -1e9 + 0.25, 1e9 - 0.375};
		shape_a: physics.Typed_Index = add_shape(t, &near, target_kind);
		shape_b: physics.Typed_Index = add_shape(t, &far, target_kind);
		a: physics.Static_Handle = add_static(t, &near, shape_a, {0, -3, 0});
		b: physics.Static_Handle = add_static(t, &far, shape_b, {origin.x, origin.y - 3, origin.z});
		index_a, index_b: int;
		index_a, _ = physics.statics_resolve(&near.simulation.statics, a);
		index_b, _ = physics.statics_resolve(&far.simulation.statics, b);
		orientation: util.Quaternion = util.quaternion_from_yaw_pitch_roll(0.43, -0.72, 0.19);
		near.simulation.statics.statics.memory[index_a].pose.orientation = orientation;
		far.simulation.statics.statics.memory[index_b].pose.orientation = orientation;
		testing.expect_value(t, physics.broad_phase_update_bounds(&near.simulation.broad_phase), physics.Physics_Status.Ok);
		testing.expect_value(t, physics.broad_phase_update_bounds(&far.simulation.broad_phase), physics.Physics_Status.Ok);
		stack: [128]i32;
		for query_kind in ([4]Shape_Kind{.Sphere, .Box, .Hull, .Compound})
		{
			query_a: physics.Typed_Index = add_shape(t, &near, query_kind);
			query_b: physics.Typed_Index = add_shape(t, &far, query_kind);
			for angular in ([3]util.Vector3{{}, {0.125, -0.25, 0.5}, {1.25, 0.5, -0.75}})
			{
				local: entasis.Rigid_Pose = {orientation=util.quaternion_from_yaw_pitch_roll(-0.31, 0.29, -0.17)};
				global: entasis.Global_Pose = {position=origin, orientation=local.orientation};
				velocity: entasis.Body_Velocity = {linear={0, -3, 0}, angular=angular};
				expected: entasis.Sweep_Hit;
				actual: entasis.Global_Sweep_Hit;
				near_status, far_status: entasis.Status;
				expected, near_status = entasis.sweep_closest_with_context(&near.query, query_a, local, velocity, 2);
				actual, far_status = entasis.sweep_global_closest_with_context(&far.query, query_b, global, velocity, 2, stack[:]);
				testing.expectf(t, near_status == .Ok && far_status == .Ok,
					"query %v target %v angular %v near %v far %v", query_kind, target_kind, angular, near_status, far_status);
				if near_status == .Ok && far_status == .Ok
				{
					expect_sweep_local_result(t, actual, expected, origin);
					contacts += 1;
				}
				cases += 1;
			}
		}
		scene_dispose(t, &far);
		scene_dispose(t, &near);
	}
	testing.expect_value(t, cases, 108);
	testing.expect_value(t, contacts, cases);
}

@(test)
global_sweep_pure_rotation_expands_tree_bounds_without_translation :: proc(t: ^testing.T)
{
	cases: int;
	for origin in ([4]f64{0, 1e4, 1e6, 1e9})
	{
		for spin in ([2]f32{-2, 2})
		{
			scene: Scene;
			scene_initialize(t, &scene);
			capsule: physics.Capsule = {radius=0.1, half_length=2};
			sphere: physics.Sphere = {radius=0.25};
			query_shape, status := physics.shape_registry_add(&scene.simulation.shapes, physics.CAPSULE_TYPE_ID, &capsule);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
			target_shape: physics.Typed_Index;
			target_shape, status = physics.shape_registry_add(&scene.simulation.shapes, physics.SPHERE_TYPE_ID, &sphere);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
			_ = add_static(t, &scene, target_shape, {origin + 1.5, -origin, origin});
			pose: entasis.Global_Pose = {position={origin, -origin, origin}, orientation=util.quaternion_identity()};
			velocity: entasis.Body_Velocity = {angular={0, 0, spin}};
			stack: [128]i32;
			hit: entasis.Global_Sweep_Hit;
			hit, status = entasis.sweep_global_closest_with_context(&scene.query, query_shape, pose, velocity, 1, stack[:]);
			testing.expect_value(t, status, entasis.Status.Ok);
			testing.expect_value(t, hit.sweep.state, physics.Sweep_Hit_State.Hit);
			// the sphere center projects inside the capsule's long segment.
			// its separation is 1.5*cos(angle), reaching the radius sum first
			expected_time: f64 = math.acos((f64(capsule.radius) + f64(sphere.radius))/1.5)/2;
			testing.expectf(t, abs(f64(hit.sweep.t1) - expected_time) < 0.001, "rotation hit %v expected %v", hit.sweep.t1, expected_time);
			result: entasis.Query_Hit_State;
			result, status = entasis.sweep_global_any_with_context(&scene.query, query_shape, pose, {}, 1, stack[:], {},
				{minimum_progression=1e-5, convergence_threshold=1e-5, maximum_iteration_count=25});
			testing.expect_value(t, status, entasis.Status.Ok);
			testing.expect_value(t, result, entasis.Query_Hit_State.Miss);
			cases += 1;
			scene_dispose(t, &scene);
		}
	}
	testing.expect_value(t, cases, 8);
}
