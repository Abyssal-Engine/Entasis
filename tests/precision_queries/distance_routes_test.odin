package precision_queries_tests

import "core:testing"
import e "entasis:entasis"
import util "entasis:entasis_utilities"

// compare the existing kernel result in the same translation-only frame, not
// a far absolute-float call that has already lost the displacement
distance_expect_local :: proc(t: ^testing.T, high: e.Global_Shape_Distance_Result, local: e.Shape_Distance_Result, origin: e.Global_Position)
{
	testing.expect_value(t, high.geometry.state, local.geometry.state);
	testing.expect_value(t, high.geometry.distance, local.geometry.distance);
	testing.expect_value(t, high.geometry.depth, local.geometry.depth);
	testing.expect_value(t, high.geometry.normal, local.geometry.normal);
	testing.expect_value(t, high.geometry.iterations, local.geometry.iterations);
	testing.expect_value(t, high.child_a, local.child_a);
	testing.expect_value(t, high.child_b, local.child_b);
	expect_position(t, high.geometry.point_a, {
		origin.x + f64(local.geometry.point_a.x), origin.y + f64(local.geometry.point_a.y), origin.z + f64(local.geometry.point_a.z),
	}, 0);
	expect_position(t, high.geometry.point_b, {
		origin.x + f64(local.geometry.point_b.x), origin.y + f64(local.geometry.point_b.y), origin.z + f64(local.geometry.point_b.z),
	}, 0);
}

@(test)
global_distance_all_shape_routes_match_local_kernel :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	queries: [3]e.Shape_Handle = {add_shape(t, &scene, .Sphere), add_shape(t, &scene, .Box), add_shape(t, &scene, .Hull)};
	cases: int;
	for kind in Shape_Kind
	{
		target: e.Shape_Handle = add_shape(t, &scene, kind);
		rotation: util.Quaternion = util.quaternion_from_axis_angle({0, 0, 1}, 0.25);
		local_b: e.Rigid_Pose = {position={3.25, 4.5, 0.125}, orientation=rotation};
		local_a: e.Rigid_Pose = {orientation=util.quaternion_identity()};
		point_reference: e.Shape_Distance_Result;
		status: e.Status;
		point_reference, status = e.shape_closest_point_with_context(&scene.query, {}, target, local_b);
		testing.expect_value(t, status, e.Status.Ok);
		for shape in queries
		{
			local: e.Shape_Distance_Result;
			local, status = e.shape_distance_with_context(&scene.query, shape, local_a, target, local_b);
			testing.expect_value(t, status, e.Status.Ok);
			for magnitude in ([4]f64{0, 1e4, 1e6, 1e9})
			{
				origin: e.Global_Position = {magnitude + 0.125, -magnitude - 0.25, magnitude + 0.375};
				a: e.Global_Pose = {position=origin, orientation=local_a.orientation};
				b: e.Global_Pose = {position={origin.x + 3.25, origin.y + 4.5, origin.z + 0.125}, orientation=rotation};
				result: e.Global_Shape_Distance_Result;
				result, status = e.shape_distance_global_with_context(&scene.query, shape, a, target, b);
				testing.expect_value(t, status, e.Status.Ok);
				distance_expect_local(t, result, local, origin);
				result, status = e.shape_closest_point_global_with_context(&scene.query, origin, target, b);
				testing.expect_value(t, status, e.Status.Ok);
				distance_expect_local(t, result, point_reference, origin);
				cases += 1;
			}
		}
	}
	testing.expect_value(t, cases, 108);
}

@(test)
global_distance_nested_union_leaf_identity_and_symmetry :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	leaf: e.Shape_Handle = add_shape(t, &scene, .Big_Compound);
	children: [2]e.Compound_Child = {
		e.compound_child(leaf, e.pose({-4, 0, 0})), e.compound_child(leaf, e.pose({4, 0, 0})),
	};
	nested: e.Shape_Handle;
	status: e.Status;
	nested, status = e.shape_import_compound(&scene.world, children[:]);
	testing.expect_value(t, status, e.Status.Ok);
	sphere: e.Shape_Handle = add_shape(t, &scene, .Sphere);
	for target in ([2]e.Shape_Handle{leaf, nested})
	{
		for magnitude in ([4]f64{0, 1e4, 1e6, 1e9})
		{
			a: e.Global_Pose = {position={magnitude + 8.125, -magnitude - 0.25, magnitude + 0.375}, orientation=util.quaternion_identity()};
			b: e.Global_Pose = a;
			b.position.x -= 8;
			local, reverse_local: e.Shape_Distance_Result;
			local, status = e.shape_distance_with_context(&scene.query, sphere, e.pose(), target, e.pose({-8, 0, 0}));
			testing.expect_value(t, status, e.Status.Ok);
			reverse_local, status = e.shape_distance_with_context(&scene.query, target, e.pose(), sphere, e.pose({8, 0, 0}));
			testing.expect_value(t, status, e.Status.Ok);
			result, reverse: e.Global_Shape_Distance_Result;
			result, status = e.shape_distance_global_with_context(&scene.query, sphere, a, target, b);
			testing.expect_value(t, status, e.Status.Ok);
			reverse, status = e.shape_distance_global_with_context(&scene.query, target, b, sphere, a);
			testing.expect_value(t, status, e.Status.Ok);
			distance_expect_local(t, result, local, a.position);
			distance_expect_local(t, reverse, reverse_local, b.position);
			testing.expect_value(t, result.geometry.distance, reverse.geometry.distance);
			expect_position(t, result.geometry.point_a, reverse.geometry.point_b, 0);
			expect_position(t, result.geometry.point_b, reverse.geometry.point_a, 0);
			testing.expect_value(t, result.child_b, reverse.child_a);
			testing.expect(t, result.child_b >= 0);
		}
	}
}

@(test)
global_distance_compound_correction_requeries_and_mesh_sidedness :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	testing.expect_value(t, e.distance_query_reserve_with_context(&scene.query), e.Status.Ok);
	x_wall, y_wall: e.Shape_Handle;
	status: e.Status;
	x_wall, status = e.shape_add(&scene.world, e.box(2, 8, 8));
	testing.expect_value(t, status, e.Status.Ok);
	y_wall, status = e.shape_add(&scene.world, e.box(8, 2, 8));
	testing.expect_value(t, status, e.Status.Ok);
	children: [2]e.Compound_Child = {e.compound_child(x_wall, e.pose({-1, 0, 0})), e.compound_child(y_wall, e.pose({0, -1, 0}))};
	targets: [2]e.Shape_Handle;
	targets[0], status = e.shape_import_compound(&scene.world, children[:]);
	testing.expect_value(t, status, e.Status.Ok);
	targets[1], status = e.shape_import_big_compound(&scene.world, children[:]);
	testing.expect_value(t, status, e.Status.Ok);
	box: e.Shape_Handle = add_shape(t, &scene, .Box);
	for target in targets
	{
		for magnitude in ([4]f64{0, 1e4, 1e6, 1e9})
		{
			pose: e.Global_Pose = {position={magnitude + 0.125, -magnitude - 0.25, magnitude + 0.375}, orientation=util.quaternion_identity()};
			correction: e.Shape_Correction_Result;
			correction, status = e.shape_depenetrate_global_with_context(&scene.query, box, pose, target, pose);
			testing.expect_value(t, status, e.Status.Ok);
			testing.expect(t, correction.iterations >= 2);
			testing.expect(t, abs(correction.translation.x - 0.5) < 1e-5 && abs(correction.translation.y - 0.5) < 1e-5);
			moved: e.Global_Pose = pose;
			moved.position.x += f64(correction.translation.x);
			moved.position.y += f64(correction.translation.y);
			moved.position.z += f64(correction.translation.z);
			for child in children
			{
				obstacle: e.Global_Pose = pose;
				obstacle.position.x += f64(child.local_position.x);
				obstacle.position.y += f64(child.local_position.y);
				result: e.Global_Shape_Distance_Result;
				result, status = e.shape_penetration_global_with_context(&scene.query, box, moved, child.shape_index, obstacle);
				testing.expect_value(t, status, e.Status.Ok);
				testing.expect(t, result.geometry.state != .Penetrating);
			}
			correction, status = e.shape_depenetrate_global_with_context(&scene.query, box, pose, target, pose, {1e-5, 1e-5, 1});
			testing.expect_value(t, status, e.Status.No_Convergence);
			testing.expect_value(t, correction, e.Shape_Correction_Result{});
		}
	}
	mesh: e.Shape_Handle = add_shape(t, &scene, .Mesh);
	sphere: e.Shape_Handle = add_shape(t, &scene, .Sphere);
	for sign in ([2]f32{-1, 1})
	{
		near_a: e.Rigid_Pose = e.pose({-0.25, sign*0.25, -0.25});
		local: e.Shape_Correction_Result;
		local, status = e.shape_depenetrate_with_context(&scene.query, sphere, near_a, mesh, e.pose());
		testing.expect_value(t, status, e.Status.Ok);
		for magnitude in ([4]f64{0, 1e4, 1e6, 1e9})
		{
			b: e.Global_Pose = {position={magnitude, -magnitude, magnitude}, orientation=util.quaternion_identity()};
			a: e.Global_Pose = b;
			a.position = {b.position.x - 0.25, b.position.y + f64(sign)*0.25, b.position.z - 0.25};
			result: e.Shape_Correction_Result;
			result, status = e.shape_depenetrate_global_with_context(&scene.query, sphere, a, mesh, b);
			testing.expect_value(t, status, e.Status.Ok);
			testing.expect_value(t, result, local);
		}
	}
}
