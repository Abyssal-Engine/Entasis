package precision_queries_tests

import "core:math"
import "core:testing"
import e "entasis:entasis"
import util "entasis:entasis_utilities"

@(test)
global_distance_sphere_millimeter_witnesses :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	shape: e.Shape_Handle = add_shape(t, &scene, .Sphere);
	cases: int;
	for magnitude in ([4]f64{0, 1e4, 1e6, 1e9})
	{
		origin: [3]f64 = {magnitude + 0.125, -magnitude - 0.25, magnitude + 0.375};
		for axis in 0 ..< 3
		{
			for sign in ([2]f64{-1, 1})
			{
				for gap in ([4]f64{0.002, 0.125, 7.5, 127.25})
				{
					b: [3]f64 = origin;
					b[axis] += sign * (1 + gap);
					a_pose: e.Global_Pose = {position={origin[0], origin[1], origin[2]}, orientation=util.quaternion_identity()};
					b_pose: e.Global_Pose = {position={b[0], b[1], b[2]}, orientation=util.quaternion_identity()};
					result: e.Global_Shape_Distance_Result;
					status: e.Status;
					result, status = e.shape_distance_global_with_context(&scene.query, shape, a_pose, shape, b_pose);
					testing.expect_value(t, status, e.Status.Ok);
					testing.expect_value(t, result.geometry.state, e.Distance_Query_State.Separated);
					expected: f64 = abs(b[axis] - origin[axis]) - 1;
					testing.expectf(t, abs(f64(result.geometry.distance) - expected) <= 0.000002,
						"distance %v expected %v at %v", result.geometry.distance, expected, magnitude);
					wa, wb: [3]f64 = origin, b;
					wa[axis] += sign * 0.5;
					wb[axis] -= sign * 0.5;
					expect_position(t, result.geometry.point_a, {wa[0], wa[1], wa[2]});
					expect_position(t, result.geometry.point_b, {wb[0], wb[1], wb[2]});
					normal: [3]f32;
					normal[axis] = f32(-sign);
					testing.expect_value(t, result.geometry.normal, util.Vector3{normal[0], normal[1], normal[2]});
					testing.expect_value(t, result.child_a, i32(-1));
					testing.expect_value(t, result.child_b, i32(-1));
					cases += 1;
				}
			}
		}
	}
	testing.expect_value(t, cases, 96);
}

@(test)
global_distance_point_exact_anchor_and_solid_inside :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	shapes: [2]e.Shape_Handle = {add_shape(t, &scene, .Sphere), add_shape(t, &scene, .Box)};
	cases: int;
	for magnitude in ([4]f64{0, 1e4, 1e6, 1e9})
	{
		center: [3]f64 = {magnitude + 0.125, -magnitude - 0.25, magnitude + 0.375};
		pose: e.Global_Pose = {position={center[0], center[1], center[2]}, orientation=util.quaternion_identity()};
		for shape in shapes
		{
			for axis in 0 ..< 3
			{
				for sign in ([2]f64{-1, 1})
				{
					for radius in ([3]f64{0.125, 0.498, 0.502})
					{
						point: [3]f64 = center;
						point[axis] += sign * radius;
						input: e.Global_Position = {point[0], point[1], point[2]};
						result: e.Global_Shape_Distance_Result;
						status: e.Status;
						result, status = e.shape_closest_point_global_with_context(&scene.query, input, shape, pose);
						testing.expect_value(t, status, e.Status.Ok);
						expect_position(t, result.geometry.point_a, input, 0);
						if radius < 0.5
						{
							testing.expect_value(t, result.geometry.state, e.Distance_Query_State.Intersecting);
							expect_position(t, result.geometry.point_b, input, 0);
							testing.expect_value(t, result.geometry.distance, f32(0));
						}
						else
						{
							testing.expect_value(t, result.geometry.state, e.Distance_Query_State.Separated);
							testing.expect(t, abs(f64(result.geometry.distance) - (abs(point[axis] - center[axis]) - 0.5)) < 0.000002);
							point[axis] = center[axis] + sign * 0.5;
							expect_position(t, result.geometry.point_b, {point[0], point[1], point[2]});
						}
						cases += 1;
					}
				}
			}
		}
	}
	testing.expect_value(t, cases, 144);
}

@(test)
global_distance_penetration_sphere_depth_and_translation :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	shape: e.Shape_Handle = add_shape(t, &scene, .Sphere);
	testing.expect_value(t, e.distance_query_reserve_with_context(&scene.query), e.Status.Ok);
	cases: int;
	for magnitude in ([4]f64{0, 1e4, 1e6, 1e9})
	{
		origin: [3]f64 = {magnitude + 0.125, -magnitude - 0.25, magnitude + 0.375};
		for axis in 0 ..< 3
		{
			for sign in ([2]f64{-1, 1})
			{
				for gap in ([3]f64{-0.002, 0, 0.002})
				{
					b: [3]f64 = origin;
					b[axis] += sign * (1 + gap);
					a_pose: e.Global_Pose = {position={origin[0], origin[1], origin[2]}, orientation=util.quaternion_identity()};
					b_pose: e.Global_Pose = {position={b[0], b[1], b[2]}, orientation=util.quaternion_identity()};
					result: e.Global_Shape_Distance_Result;
					status: e.Status;
					result, status = e.shape_penetration_global_with_context(&scene.query, shape, a_pose, shape, b_pose);
					testing.expect_value(t, status, e.Status.Ok);
					expected_gap: f64 = abs(b[axis] - origin[axis]) - 1;
					testing.expect(t, abs(f64(result.geometry.depth) - max(f64(0), -expected_gap)) < 0.000002);
					testing.expect(t, abs(f64(result.geometry.distance) - max(f64(0), expected_gap)) < 0.000002);
					expected_state: e.Distance_Query_State = .Touching;
					if gap < 0
					{
						expected_state = .Penetrating;
					}
					else if gap > 0
					{
						expected_state = .Separated;
					}
					testing.expect_value(t, result.geometry.state, expected_state);
					if gap != 0
					{
						wa, wb: [3]f64 = origin, b;
						wa[axis] += sign * 0.5;
						wb[axis] -= sign * 0.5;
						expect_position(t, result.geometry.point_a, {wa[0], wa[1], wa[2]});
						expect_position(t, result.geometry.point_b, {wb[0], wb[1], wb[2]});
					}
					if gap < 0
					{
						correction: e.Shape_Correction_Result;
						correction, status = e.shape_depenetrate_global_with_context(&scene.query, shape, a_pose, shape, b_pose);
						testing.expect_value(t, status, e.Status.Ok);
						a_pose.position.x += f64(correction.translation.x);
						a_pose.position.y += f64(correction.translation.y);
						a_pose.position.z += f64(correction.translation.z);
						result, status = e.shape_penetration_global_with_context(&scene.query, shape, a_pose, shape, b_pose);
						testing.expect_value(t, status, e.Status.Ok);
						testing.expect(t, result.geometry.state != .Penetrating);
					}
					cases += 1;
				}
			}
		}
	}
	testing.expect_value(t, cases, 72);
}

@(test)
global_distance_box_grid_independent_witness_distance :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	box: e.Shape_Handle = add_shape(t, &scene, .Box);
	cases: int;
	for magnitude in ([4]f64{0, 1e4, 1e6, 1e9})
	{
		a: e.Global_Pose = {position={magnitude + 0.125, -magnitude - 0.25, magnitude + 0.375}, orientation=util.quaternion_identity()};
		for x in -2 ..= 2
		{
			for y in -2 ..= 2
			{
				for z in -2 ..= 2
				{
					dx, dy, dz: f64 = f64(x)*1.125, f64(y)*1.25, f64(z)*1.375;
					b: e.Global_Pose = a;
					b.position = {a.position.x + dx, a.position.y + dy, a.position.z + dz};
					result: e.Global_Shape_Distance_Result;
					status: e.Status;
					result, status = e.shape_distance_global_with_context(&scene.query, box, a, box, b);
					testing.expect_value(t, status, e.Status.Ok);
					gx, gy, gz: f64 = max(abs(dx) - 1, 0), max(abs(dy) - 1, 0), max(abs(dz) - 1, 0);
					expected: f64 = math.sqrt(gx*gx + gy*gy + gz*gz);
					testing.expect(t, abs(f64(result.geometry.distance) - expected) < 0.000002);
					wa: e.Global_Position = result.geometry.point_a;
					wb: e.Global_Position = result.geometry.point_b;
					testing.expect(t, abs(wa.x - a.position.x) <= 0.500002 && abs(wa.y - a.position.y) <= 0.500002 && abs(wa.z - a.position.z) <= 0.500002);
					testing.expect(t, abs(wb.x - b.position.x) <= 0.500002 && abs(wb.y - b.position.y) <= 0.500002 && abs(wb.z - b.position.z) <= 0.500002);
					vx, vy, vz: f64 = wa.x - wb.x, wa.y - wb.y, wa.z - wb.z;
					testing.expect(t, abs(math.sqrt(vx*vx + vy*vy + vz*vz) - expected) < 0.000002);
					cases += 1;
				}
			}
		}
	}
	testing.expect_value(t, cases, 500);
}
