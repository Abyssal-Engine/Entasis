package precision_queries_tests

import "core:math"
import "core:testing"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Overlap_Population :: enum u8
{
	Dynamic,
	Kinematic,
	Sleeping,
	Static,
}

@(test)
global_overlap_two_millimeter_sphere_oracle :: proc(t: ^testing.T)
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
			hits: [2]entasis.Overlap_Hit;
			for axis in 0 ..< 3
			{
				for direction in ([2]f64{-1, 1})
				{
					for separation in ([2]f64{-0.002, 0.002})
					{
						components: [3]f64 = {center.x, center.y, center.z};
						components[axis] += direction * (1 + separation);
						pose: entasis.Global_Pose = {
							position={components[0], components[1], components[2]}, orientation=util.quaternion_identity(),
						};
						count: int;
						status: entasis.Status;
						count, status = entasis.overlap_global_all_with_context(&scene.query, shape, pose, hits[:], stack[:]);
						testing.expect_value(t, status, entasis.Status.Ok);
						state: entasis.Overlap_State;
						state, status = entasis.overlap_global_any_with_context(&scene.query, shape, pose, stack[:]);
						testing.expect_value(t, status, entasis.Status.Ok);
						if separation > 0
						{
							testing.expectf(t, count == 0 && state == .Separated, "origin %v population %v axis %v direction %v count %v state %v", origin, population, axis, direction, count, state);
						}
						else
						{
							testing.expect_value(t, count, 1);
							testing.expect_value(t, state, entasis.Overlap_State.Intersecting);
							testing.expect_value(t, hits[0].collidable, reference);
							testing.expect_value(t, hits[0].manifold.kind, physics.Manifold_Kind.Convex);
							testing.expect_value(t, hits[0].manifold.convex.count, i32(1));
							dx: f64 = pose.position.x - center.x;
							dy: f64 = pose.position.y - center.y;
							dz: f64 = pose.position.z - center.z;
							depth: f64 = 1 - math.sqrt(dx*dx + dy*dy + dz*dz);
							testing.expectf(t, abs(f64(hits[0].manifold.convex.contacts[0].depth) - depth) <= 0.000002,
								"origin %v population %v depth %v expected %v", origin, population, hits[0].manifold.convex.contacts[0].depth, depth);
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

expect_overlap_manifold :: proc(t: ^testing.T, actual, expected: physics.Manifold_Result)
{
	testing.expect_value(t, actual.kind, expected.kind);
	if actual.kind == .Convex
	{
		testing.expect_value(t, actual.convex.count, expected.convex.count);
		testing.expect_value(t, actual.convex.offset_b, expected.convex.offset_b);
		testing.expect_value(t, actual.convex.normal, expected.convex.normal);
		for index in 0 ..< int(expected.convex.count)
		{
			testing.expect_value(t, actual.convex.contacts[index], expected.convex.contacts[index]);
		}
	}
	else
	{
		testing.expect_value(t, actual.nonconvex.count, expected.nonconvex.count);
		testing.expect_value(t, actual.nonconvex.offset_b, expected.nonconvex.offset_b);
		for index in 0 ..< int(expected.nonconvex.count)
		{
			testing.expect_value(t, actual.nonconvex.contacts[index], expected.nonconvex.contacts[index]);
		}
	}
}

@(test)
global_overlap_builtin_routes_match_existing_local_queries :: proc(t: ^testing.T)
{
	for target_kind in Shape_Kind
	{
		near: Scene;
		far: Scene;
		scene_initialize(t, &near);
		scene_initialize(t, &far);
		offset: physics.Global_Position = {1e9 + 0.125, -1e9 + 0.25, 1e9 - 0.375};
		shape_a: physics.Typed_Index = add_shape(t, &near, target_kind);
		shape_b: physics.Typed_Index = add_shape(t, &far, target_kind);
		a: physics.Static_Handle = add_static(t, &near, shape_a, {});
		b: physics.Static_Handle = add_static(t, &far, shape_b, offset);
		orientation: util.Quaternion = util.quaternion_from_yaw_pitch_roll(0.43, -0.72, 0.19);
		index_a, index_b: int;
		index_a, _ = physics.statics_resolve(&near.simulation.statics, a);
		index_b, _ = physics.statics_resolve(&far.simulation.statics, b);
		near.simulation.statics.statics.memory[index_a].pose.orientation = orientation;
		far.simulation.statics.statics.memory[index_b].pose.orientation = orientation;
		testing.expect_value(t, physics.broad_phase_update_bounds(&far.simulation.broad_phase), physics.Physics_Status.Ok);
		stack: [128]i32;
		near_hits, far_hits: [16]entasis.Overlap_Hit;
		for query_kind in ([4]Shape_Kind{.Sphere, .Box, .Hull, .Compound})
		{
			query_a: physics.Typed_Index = add_shape(t, &near, query_kind);
			query_b: physics.Typed_Index = add_shape(t, &far, query_kind);
			for displacement in ([3]f32{0.125, 0.375, 2})
			{
				// compare the original float API in the same query-centered frame
				// otherwise rotated child placement at a nonzero f32 origin adds
				// unrelated rounding before the original kernel's subtraction
				testing.expect_value(t, physics.statics_set_global_position(&near.simulation.statics, a, {0, -f64(displacement), 0.125}), physics.Physics_Status.Ok);
				testing.expect_value(t, physics.broad_phase_update_bounds(&near.simulation.broad_phase), physics.Physics_Status.Ok);
				local: physics.Rigid_Pose = {orientation=util.quaternion_from_yaw_pitch_roll(-0.31, 0.29, -0.17)};
				global: entasis.Global_Pose = {
					position={offset.x, offset.y + f64(displacement), offset.z - 0.125}, orientation=local.orientation,
				};
				near_count, far_count: int;
				near_status, far_status: entasis.Status;
				near_count, near_status = entasis.overlap_all_with_context(&near.query, query_a, local, near_hits[:]);
				far_count, far_status = entasis.overlap_global_all_with_context(&far.query, query_b, global, far_hits[:], stack[:]);
				testing.expectf(t, near_status == .Ok && far_status == .Ok, "query %v target %v near %v far %v", query_kind, target_kind, near_status, far_status);
				testing.expectf(t, far_count == near_count, "query %v target %v count %v expected %v", query_kind, target_kind, far_count, near_count);
				if displacement == 0.125
				{
					testing.expectf(t, near_count > 0, "query %v target %v must exercise real contacts", query_kind, target_kind);
				}
				for index in 0 ..< min(near_count, far_count)
				{
					testing.expect_value(t, far_hits[index].collidable, near_hits[index].collidable);
					expect_overlap_manifold(t, far_hits[index].manifold, near_hits[index].manifold);
				}
				near_state, far_state: entasis.Overlap_State;
				near_state, near_status = entasis.overlap_any_with_context(&near.query, query_a, local);
				far_state, far_status = entasis.overlap_global_any_with_context(&far.query, query_b, global, stack[:]);
				testing.expect_value(t, near_status, entasis.Status.Ok);
				testing.expect_value(t, far_status, near_status);
				testing.expect_value(t, far_state, near_state);
			}
		}
		scene_dispose(t, &far);
		scene_dispose(t, &near);
	}
}
