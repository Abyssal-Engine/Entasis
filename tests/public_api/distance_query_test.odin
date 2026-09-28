package public_api_tests

import "core:testing"
import "core:math"
import "core:thread"
import "core:simd"
import "base:runtime"
import e "entasis:entasis"
import p "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Distance_Test_Scratch :: struct
{
	vertices: [512]p.Distance_Query_Vertex,
	faces: [1024]p.Distance_Query_Face,
	edges: [512]p.Distance_Query_Edge,
}

distance_test_scratch :: proc "contextless" (storage: ^Distance_Test_Scratch) -> p.Distance_Query_Scratch
{
	return {vertices=storage.vertices[:], faces=storage.faces[:], edges=storage.edges[:]};
}

@(test)
distance_geometry_sphere_witnesses_and_small_gaps :: proc (t: ^testing.T)
{
	world: e.World;
	if !testing.expect_value(t, e.world_init(&world, small_world_description()), e.Status.Ok)
	{
		return;
	}
	defer e.world_destroy(&world);
	simulation: ^p.Simulation;
	status: e.Status;
	simulation, status = e.world_borrow_simulation(&world);
	a: p.Sphere = {radius=1};
	b: p.Sphere = {radius=2};
	settings: p.Distance_Query_Settings = p.distance_query_settings_default();
	for gap in ([4]f32{0.0001, 0.125, 3, 100})
	{
		result: p.Distance_Query_Result;
		result, status = p.distance_query_convex(&a, &b, p.SPHERE_TYPE_ID, p.SPHERE_TYPE_ID,
			e.pose(), e.pose({3+gap, 0, 0}), &simulation.shapes, settings);
		if !testing.expect_value(t, status, e.Status.Ok)
		{
			continue;
		}
		testing.expect_value(t, result.state, p.Distance_Query_State.Separated);
		testing.expect(t, abs(result.distance-gap) < 2e-5);
		testing.expect(t, util.vector3_distance(result.point_a, {1, 0, 0}) < 2e-5);
		testing.expect(t, abs(util.vector3_distance(result.point_a, result.point_b)-result.distance) < 2e-5);
		testing.expect(t, result.normal.x < -0.999);
	}
}

@(test)
distance_geometry_point_convex_inside_rotation_and_translation :: proc (t: ^testing.T)
{
	world: e.World;
	if !testing.expect_value(t, e.world_init(&world, small_world_description()), e.Status.Ok)
	{
		return;
	}
	defer e.world_destroy(&world);
	simulation: ^p.Simulation;
	status: e.Status;
	simulation, status = e.world_borrow_simulation(&world);
	box: p.Box = {half_width=1, half_height=2, half_length=3};
	settings: p.Distance_Query_Settings = p.distance_query_settings_default();
	for origin in ([2]util.Vector3{{}, {100000, -50000, 100000}})
	{
		pose: p.Rigid_Pose = e.pose(origin);
		pose.orientation = {0, 0, f32(math.sqrt(0.5)), f32(math.sqrt(0.5))};
		for offset in ([4]util.Vector3{{}, {0.25, 0.25, 0.25}, {4, 0, 0}, {0, 4, 0}})
		{
			point: util.Vector3 = util.vector3_add(origin, offset);
			result: p.Distance_Query_Result;
			result, status = p.distance_query_point(point, &box, p.BOX_TYPE_ID, pose, &simulation.shapes, settings);
			if !testing.expect_value(t, status, e.Status.Ok)
			{
				continue;
			}
			if offset.x < 1 && offset.y < 1
			{
				testing.expect_value(t, result.state, p.Distance_Query_State.Intersecting);
				testing.expect_value(t, result.point_b, point);
				testing.expect_value(t, result.normal, util.Vector3{});
			}
			else
			{
				expected: f32 = 2;
				if offset.y == 4
				{
					expected = 3;
				}
				testing.expect(t, abs(result.distance-expected) < 2e-4);
			}
		}
	}
}

@(test)
distance_geometry_penetration_translation_and_budget :: proc (t: ^testing.T)
{
	world: e.World;
	if !testing.expect_value(t, e.world_init(&world, small_world_description()), e.Status.Ok)
	{
		return;
	}
	defer e.world_destroy(&world);
	simulation: ^p.Simulation;
	status: e.Status;
	simulation, status = e.world_borrow_simulation(&world);
	box: p.Box = {half_width=1, half_height=1, half_length=1};
	settings: p.Distance_Query_Settings = p.distance_query_settings_default();
	storage: Distance_Test_Scratch;
	scratch: p.Distance_Query_Scratch = distance_test_scratch(&storage);
	for offset in ([4]util.Vector3{{1.5, 0.1, 0.1}, {0, 0, 0}, {2, 0, 0}, {3, 0, 0}})
	{
		result: p.Distance_Query_Result;
		result, status = p.distance_query_convex(&box, &box, p.BOX_TYPE_ID, p.BOX_TYPE_ID,
			e.pose(), e.pose(offset), &simulation.shapes, settings, .Penetration, scratch);
		if !testing.expect_value(t, status, e.Status.Ok)
		{
			continue;
		}
		if offset.x < 2
		{
			testing.expect_value(t, result.state, p.Distance_Query_State.Penetrating);
			testing.expect(t, abs(result.depth-(2-offset.x)) < 5e-5);
			moved: p.Distance_Query_Result;
			moved, status = p.distance_query_convex(&box, &box, p.BOX_TYPE_ID, p.BOX_TYPE_ID,
				e.pose(util.vector3_scale(result.normal, result.depth+settings.absolute_tolerance*2)),
				e.pose(offset), &simulation.shapes, settings, .Penetration, scratch);
			testing.expect_value(t, status, e.Status.Ok);
			testing.expect(t, moved.state != .Penetrating);
		}
		else
		{
			testing.expect_value(t, result.depth, f32(0));
		}
	}
	result: p.Distance_Query_Result;
	result, status = p.distance_query_convex(&box, &box, p.BOX_TYPE_ID, p.BOX_TYPE_ID, e.pose(), e.pose(), &simulation.shapes, settings, .Penetration);
	testing.expect_value(t, status, e.Status.Capacity_Missing);
	testing.expect_value(t, result.state, p.Distance_Query_State.Unresolved);
	settings.maximum_iterations=1;
	result, status=p.distance_query_convex(&box, &box, p.BOX_TYPE_ID, p.BOX_TYPE_ID, e.pose(), e.pose({0.8, 0.5, 0.3}), &simulation.shapes, settings, .Penetration, scratch);
	testing.expect_value(t, status, e.Status.No_Convergence);
	testing.expect_value(t, result.state, p.Distance_Query_State.Unresolved);
}

@(test)
distance_geometry_boundaries_and_triangle :: proc (t: ^testing.T)
{
	world: e.World;
	if !testing.expect_value(t, e.world_init(&world, small_world_description()), e.Status.Ok)
	{
		return;
	}
	defer e.world_destroy(&world);
	simulation: ^p.Simulation;
	status: e.Status;
	simulation, status=e.world_borrow_simulation(&world);
	triangle: p.Triangle={a={-1, 0, 0}, b={1, 0, 0}, c={0, 1, 0}};
	settings: p.Distance_Query_Settings=p.distance_query_settings_default();
	for point in ([3]util.Vector3{{0, 0.25, 2}, {0, 0.25, 0}, {3, 0, 0}})
	{
		result: p.Distance_Query_Result;
		result, status=p.distance_query_point(point, &triangle, p.TRIANGLE_TYPE_ID, e.pose(), &simulation.shapes, settings);
		testing.expect_value(t, status, e.Status.Ok);
		if point.z==2 || point.x==3
		{
			testing.expect(t, abs(result.distance-2)<1e-5);
		}
		if point.z==0 && point.x==0
		{
			testing.expect_value(t, result.point_b, point);
		}
	}
	for value in ([4]f32{0, -1, math.nan_f32(), math.inf_f32(1)})
	{
		settings.absolute_tolerance=value;
		testing.expect_value(t, p.distance_query_settings_validate(settings), e.Status.Invalid_Argument);
	}
	settings=p.distance_query_settings_default();
	settings.relative_tolerance=math.nan_f32();
	testing.expect_value(t, p.distance_query_settings_validate(settings), e.Status.Invalid_Argument);
}

@(test)
distance_geometry_box_grid_independent_distance_and_depth :: proc (t: ^testing.T)
{
	world: e.World;
	if !testing.expect_value(t, e.world_init(&world, small_world_description()), e.Status.Ok)
	{
		return;
	}
	defer e.world_destroy(&world);
	simulation: ^p.Simulation;
	status: e.Status;
	simulation, status=e.world_borrow_simulation(&world);
	a: p.Box={half_width=1, half_height=0.7, half_length=1.3};
	b: p.Box={half_width=0.4, half_height=0.9, half_length=0.6};
	storage: Distance_Test_Scratch;
	scratch: p.Distance_Query_Scratch=distance_test_scratch(&storage);
	settings: p.Distance_Query_Settings=p.distance_query_settings_default();
	for i in 0..<500
	{
		offset: util.Vector3={f32(i%11)*0.37-1.85, f32((i/11)%11)*0.31-1.55, f32(i/121)*0.63-1.26};
		gx: f64=abs(f64(offset.x))-f64(a.half_width)-f64(b.half_width);
		gy: f64=abs(f64(offset.y))-f64(a.half_height)-f64(b.half_height);
		gz: f64=abs(f64(offset.z))-f64(a.half_length)-f64(b.half_length);
		expected: f64=math.sqrt(max(f64(0), gx)*max(f64(0), gx)+max(f64(0), gy)*max(f64(0), gy)+max(f64(0), gz)*max(f64(0), gz));
		result: p.Distance_Query_Result;
		result, status=p.distance_query_convex(&a, &b, p.BOX_TYPE_ID, p.BOX_TYPE_ID, e.pose(), e.pose(offset), &simulation.shapes, settings, .Penetration, scratch);
		if !testing.expect_value(t, status, e.Status.Ok)
		{
			continue;
		}
		if expected>f64(settings.absolute_tolerance)
		{
			testing.expect_value(t, result.state, p.Distance_Query_State.Separated);
			testing.expect(t, abs(f64(result.distance)-expected)<3e-5);
		}
		else
		{
			depth: f64=min(-gx, min(-gy, -gz));
			testing.expect(t, abs(f64(result.depth)-depth)<4e-5);
		}
	}
}

@(test)
distance_geometry_contextual_native_support_and_callback_failure :: proc (t: ^testing.T)
{
	world: e.World;
	if !testing.expect_value(t, e.world_init(&world, small_world_description()), e.Status.Ok)
	{
		return;
	}
	defer e.world_destroy(&world);
	state: Contextual_Shape_State={scale=1.5};
	contextual_type, native_type: e.Shape_Type_ID;
	status: e.Status;
	contextual_type, status=e.custom_shape_register_contextual(&world, contextual_sphere_registration(&state));
	if !testing.expect_value(t, status, e.Status.Ok)
	{
		return;
	}
	native_type, status=e.custom_shape_register(&world, e.custom_shape_registration(Custom_Sphere, .Convex,
			custom_sphere_bounds, custom_sphere_inertia, custom_sphere_ray, custom_sphere_support));
	if !testing.expect_value(t, status, e.Status.Ok)
	{
		return;
	}
	simulation: ^p.Simulation;
	simulation, status=e.world_borrow_simulation(&world);
	payload: Custom_Sphere={radius=1};
	settings: p.Distance_Query_Settings=p.distance_query_settings_default();
	storage: Distance_Test_Scratch;
	scratch: p.Distance_Query_Scratch=distance_test_scratch(&storage);
	for types in ([4][2]int{{int(native_type), int(native_type)}, {int(native_type), int(contextual_type)},
			{int(contextual_type), int(native_type)}, {int(contextual_type), int(contextual_type)}})
	{
		for offset in ([3]util.Vector3{{4, 3, 2}, {0.8, 0.5, 0.3}, {2, 1, 0}})
		{
			result: p.Distance_Query_Result;
			result, status=p.distance_query_convex(&payload, &payload, types[0], types[1], e.pose(), e.pose(offset), &simulation.shapes, settings, .Penetration, scratch);
			if !testing.expect_value(t, status, e.Status.Ok)
			{
				continue;
			}
			radius: f64=2;
			if types[0]==int(contextual_type)
			{
				radius+=0.5;
			}
			if types[1]==int(contextual_type)
			{
				radius+=0.5;
			}
			gap: f64=math.sqrt(f64(offset.x)*f64(offset.x)+f64(offset.y)*f64(offset.y)+f64(offset.z)*f64(offset.z))-radius;
			testing.expect(t, abs(f64(result.distance)-max(f64(0), gap))<1e-4);
			testing.expect(t, abs(f64(result.depth)-max(f64(0), -gap))<1e-4);
		}
	}
	state.failure=.Capacity_Missing;
	result: p.Distance_Query_Result;
	result, status=p.distance_query_convex(&payload, &payload, int(contextual_type), int(native_type), e.pose(), e.pose({5, 0, 0}), &simulation.shapes, settings);
	testing.expect_value(t, status, e.Status.Capacity_Missing);
	testing.expect_value(t, result.state, p.Distance_Query_State.Unresolved);
	state.failure=.Ok;
}

Distance_Late_Failure_State :: struct
{
	base: Contextual_Shape_State,
	calls: int,
	fail_after: int,
	injected_status: e.Status,
}

distance_late_failure_support :: proc "contextless" (
	user_context, shape: rawptr, direction: ^util.Vector3,
	access: e.Shape_Access, result: ^util.Vector3,
) -> e.Status
{
	state: ^Distance_Late_Failure_State = cast(^Distance_Late_Failure_State)user_context;
	state.calls += 1;
	if state.calls == state.fail_after
	{
		return state.injected_status;
	}
	return contextual_shape_support_test(&state.base, shape, direction, access, result);
}

@(test)
distance_geometry_epa_preserves_late_callback_errors :: proc (t: ^testing.T)
{
	world: e.World;
	if !testing.expect_value(t, e.world_init(&world, small_world_description()), e.Status.Ok)
	{
		return;
	}
	defer e.world_destroy(&world);
	state: Distance_Late_Failure_State = {base={scale=1}, fail_after=3};
	registration: e.Contextual_Custom_Shape_Registration = contextual_sphere_registration(&state.base);
	registration.user_context = &state;
	registration.support = distance_late_failure_support;
	type_id: e.Shape_Type_ID;
	status: e.Status;
	type_id, status = e.custom_shape_register_contextual(&world, registration);
	if !testing.expect_value(t, status, e.Status.Ok)
	{
		return;
	}
	simulation: ^p.Simulation;
	simulation, status = e.world_borrow_simulation(&world);
	payload: Custom_Sphere = {radius=1};
	sphere: p.Sphere = {radius=1};
	storage: Distance_Test_Scratch;
	for failure in ([3]e.Status{.Not_Found, .Capacity_Missing, .No_Convergence})
	{
		state.calls = 0;
		state.injected_status = failure;
		result: p.Distance_Query_Result;
		result, status = p.distance_query_convex(&payload, &sphere, int(type_id), p.SPHERE_TYPE_ID,
			e.pose(), e.pose(), &simulation.shapes, p.distance_query_settings_default(), .Penetration, distance_test_scratch(&storage));
		testing.expect_value(t, status, failure);
		testing.expect_value(t, state.calls, 3);
		testing.expect_value(t, result.state, p.Distance_Query_State.Unresolved);
	}
}

@(test)
distance_geometry_rotated_box_depth_matches_independent_sat :: proc (t: ^testing.T)
{
	world: e.World;
	if !testing.expect_value(t, e.world_init(&world, small_world_description()), e.Status.Ok)
	{
		return;
	}
	defer e.world_destroy(&world);
	simulation: ^p.Simulation;
	status: e.Status;
	simulation, status = e.world_borrow_simulation(&world);
	box: p.Box = {half_width=1, half_height=0.7, half_length=1.3};
	half: [3]f64 = {1, 0.7, 1.3};
	settings: p.Distance_Query_Settings = p.distance_query_settings_default();
	storage: Distance_Test_Scratch;
	for i in 0..<64
	{
		angle: f32 = f32(i+1)*0.043;
		a: p.Rigid_Pose = e.pose();
		b: p.Rigid_Pose = e.pose({f32(i%7)*0.1-0.3, 0.2, -0.3});
		a.orientation = {0, 0, math.sin(angle), math.cos(angle)};
		b.orientation = {0, math.sin(angle*0.7), 0, math.cos(angle*0.7)};
		axes_a: [3]util.Vector3 = p.collision_pose_axes(a);
		axes_b: [3]util.Vector3 = p.collision_pose_axes(b);
		axes: [15]util.Vector3;
		for axis in 0..<3
		{
			axes[axis] = axes_a[axis];
			axes[axis+3] = axes_b[axis];
			for other in 0..<3
			{
				axes[6+axis*3+other] = util.vector3_cross(axes_a[axis], axes_b[other]);
			}
		}
		expected: f64 = math.F64_MAX;
		for axis in axes
		{
			length: f64 = math.sqrt(f64(axis.x)*f64(axis.x)+f64(axis.y)*f64(axis.y)+f64(axis.z)*f64(axis.z));
			if length < 1e-8
			{
				continue;
			}
			n: util.Vector3 = util.vector3_scale(axis, f32(1/length));
			radius: f64;
			for j in 0..<3
			{
				radius += half[j]*(abs(f64(util.vector3_dot(n, axes_a[j])))+abs(f64(util.vector3_dot(n, axes_b[j]))));
			}
			expected = min(expected, radius-abs(f64(util.vector3_dot(n, b.position))));
		}
		result: p.Distance_Query_Result;
		result, status = p.distance_query_convex(&box, &box, p.BOX_TYPE_ID, p.BOX_TYPE_ID, a, b, &simulation.shapes, settings, .Penetration, distance_test_scratch(&storage));
		if !testing.expect_value(t, status, e.Status.Ok)
		{
			continue;
		}
		testing.expect_value(t, result.state, p.Distance_Query_State.Penetrating);
		testing.expect(t, abs(f64(result.depth)-expected)<2e-4);
	}
}

@(test)
distance_geometry_analytic_primitives_and_spheres :: proc (t: ^testing.T)
{
	world: e.World;
	if !testing.expect_value(t, e.world_init(&world, small_world_description()), e.Status.Ok)
	{
		return;
	}
	defer e.world_destroy(&world);
	simulation: ^p.Simulation;
	status: e.Status;
	simulation, status = e.world_borrow_simulation(&world);
	settings: p.Distance_Query_Settings = p.distance_query_settings_default();
	capsule: p.Capsule = {radius=0.7, half_length=1.2};
	cylinder: p.Cylinder = {radius=0.7, half_length=1.2};
	box: p.Box = {half_width=0.7, half_height=1.2, half_length=0.9};
	for i in 0..<320
	{
		point: util.Vector3 = {f32(i%13)*0.31-1.55, f32((i/13)%11)*0.37-1.85, f32(i/143)*0.71-0.71};
		for kind in 0..<3
		{
			shape: rawptr = &capsule;
			type_id: int = p.CAPSULE_TYPE_ID;
			x: f64 = f64(point.x);
			y: f64 = max(abs(f64(point.y))-f64(capsule.half_length), f64(0));
			z: f64 = f64(point.z);
			expected: f64 = max(math.sqrt(x*x+y*y+z*z)-f64(capsule.radius), f64(0));
			if kind == 1
			{
				shape = &cylinder;
				type_id = p.CYLINDER_TYPE_ID;
				radial: f64 = max(math.sqrt(x*x+z*z)-f64(cylinder.radius), f64(0));
				expected = math.sqrt(radial*radial+y*y);
			}
			else if kind == 2
			{
				shape = &box;
				type_id = p.BOX_TYPE_ID;
				x = max(abs(x)-f64(box.half_width), f64(0));
				z = max(abs(z)-f64(box.half_length), f64(0));
				expected = math.sqrt(x*x+y*y+z*z);
			}
			result: p.Distance_Query_Result;
			result, status = p.distance_query_point(point, shape, type_id, e.pose(), &simulation.shapes, settings);
			testing.expect_value(t, status, e.Status.Ok);
			testing.expect(t, abs(f64(result.distance)-expected)<3e-6);
			testing.expect_value(t, result.iterations, i32(0));
		}
	}
	a: p.Sphere = {radius=0.8};
	b: p.Sphere = {radius=1.4};
	for offset in ([5]util.Vector3{{}, {0.5, 0, 0}, {2.2, 0, 0}, {4, 1, -2}, {0, 0, -1}})
	{
		result: p.Distance_Query_Result;
		result, status = p.distance_query_convex(&a, &b, p.SPHERE_TYPE_ID, p.SPHERE_TYPE_ID, e.pose(offset), e.pose(), &simulation.shapes, settings, .Penetration);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, result.iterations, i32(0));
		length: f64 = math.sqrt(f64(offset.x)*f64(offset.x)+f64(offset.y)*f64(offset.y)+f64(offset.z)*f64(offset.z));
		testing.expect(t, abs(f64(result.depth)-max(f64(0), f64(a.radius)+f64(b.radius)-length))<2e-6);
		if result.state == .Penetrating
		{
			moved: p.Distance_Query_Result;
			moved, status = p.distance_query_convex(&a, &b, p.SPHERE_TYPE_ID, p.SPHERE_TYPE_ID,
				e.pose(util.vector3_add(offset, util.vector3_scale(result.normal, result.depth))), e.pose(), &simulation.shapes, settings, .Penetration);
			testing.expect_value(t, status, e.Status.Ok);
			testing.expect(t, moved.state!=.Penetrating);
		}
	}
}

@(test)
distance_geometry_compound_nearest_matches_independent_sphere_union :: proc (t: ^testing.T)
{
	world: e.World;
	if !testing.expect_value(t, e.world_init(&world, small_world_description()), e.Status.Ok)
	{
		return;
	}
	defer e.world_destroy(&world);
	simulation: ^p.Simulation;
	status: e.Status;
	simulation, status = e.world_borrow_simulation(&world);
	shape: e.Shape_Handle;
	shape, status = e.shape_add(&world, e.sphere(0.5));
	testing.expect_value(t, status, e.Status.Ok);
	children: [37]e.Compound_Child;
	for &child, i in children
	{
		child = e.compound_child(shape, e.pose({f32(i%7)*3-9, f32(i/7)*3-6, 0}));
	}
	handles: [2]e.Shape_Handle;
	handles[0], status = e.shape_import_compound(&world, children[:]);
	testing.expect_value(t, status, e.Status.Ok);
	handles[1], status = e.shape_import_big_compound(&world, children[:]);
	testing.expect_value(t, status, e.Status.Ok);
	settings: p.Distance_Query_Settings = p.distance_query_settings_default();
	pose: p.Rigid_Pose = e.pose({10000, -20000, 10000});
	pose.orientation = {0, 0, f32(math.sin(0.37)), f32(math.cos(0.37))};
	for handle in handles
	{
		payload: rawptr;
		batch: ^p.Shape_Batch;
		payload, batch, status = p.shape_registry_resolve(&simulation.shapes, handle);
		for i in 0..<100
		{
			local: util.Vector3 = {f32(i%10)*2.7-12.1, f32(i/10)*2.1-8.7, 0.5+f32(i%3)};
			point: util.Vector3 = p.rigid_pose_transform(local, pose);
			expected: f64 = math.F64_MAX;
			nearest: int;
			for child, j in children
			{
				center: util.Vector3 = p.rigid_pose_transform(child.local_position, pose);
				x: f64 = f64(point.x)-f64(center.x);
				y: f64 = f64(point.y)-f64(center.y);
				z: f64 = f64(point.z)-f64(center.z);
				distance: f64 = max(f64(0), math.sqrt(x*x+y*y+z*z)-0.5);
				if distance < expected
				{
					expected = distance;
					nearest = j;
				}
			}
			result: p.Distance_Query_Pair_Result;
			result, status = p.distance_query_shape_point(point, payload, int(p.typed_index_type(handle)), pose, &simulation.shapes, settings);
			if !testing.expect_value(t, status, e.Status.Ok)
			{
				continue;
			}
			testing.expect(t, abs(f64(result.geometry.distance)-expected)<0.003);
			testing.expect_value(t, result.child_b, i32(nearest));
		}
	}
	// stored order, not tree traversal order, determines equal-distance ties
	tie_children: [2]e.Compound_Child = {e.compound_child(shape, e.pose({3, 0, 0})), e.compound_child(shape, e.pose({-3, 0, 0}))};
	tied: e.Shape_Handle;
	tied, status = e.shape_import_big_compound(&world, tie_children[:]);
	payload: rawptr;
	batch: ^p.Shape_Batch;
	payload, batch, status = p.shape_registry_resolve(&simulation.shapes, tied);
	result: p.Distance_Query_Pair_Result;
	result, status = p.distance_query_shape_point({}, payload, p.BIG_COMPOUND_TYPE_ID, e.pose(), &simulation.shapes, settings);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, result.child_b, i32(0));
	testing.expect_value(t, result.geometry.distance, f32(2.5));
}

@(test)
distance_geometry_composite_pair_symmetry_and_leaf_witnesses :: proc (t: ^testing.T)
{
	world: e.World;
	if !testing.expect_value(t, e.world_init(&world, small_world_description()), e.Status.Ok)
	{
		return;
	}
	defer e.world_destroy(&world);
	simulation: ^p.Simulation;
	status: e.Status;
	simulation, status = e.world_borrow_simulation(&world);
	shape: e.Shape_Handle;
	shape, status = e.shape_add(&world, e.sphere(0.5));
	children: [3]e.Compound_Child = {
		e.compound_child(shape, e.pose({-4, 0, 0})), e.compound_child(shape, e.pose()), e.compound_child(shape, e.pose({4, 0, 0})),
	};
	handle: e.Shape_Handle;
	handle, status = e.shape_import_big_compound(&world, children[:]);
	payload: rawptr;
	batch: ^p.Shape_Batch;
	payload, batch, status = p.shape_registry_resolve(&simulation.shapes, handle);
	settings: p.Distance_Query_Settings = p.distance_query_settings_default();
	for i in 0..<50
	{
		pose_b: p.Rigid_Pose = e.pose({f32(i%10)*0.53, f32(i/10)*0.61+1.2, 0.2});
		expected: f32 = math.F32_MAX;
		for child_a in children
		{
			for child_b in children
			{
				delta: util.Vector3 = util.vector3_subtract(child_a.local_position, util.vector3_add(child_b.local_position, pose_b.position));
				expected = min(expected, max(f32(0), util.vector3_length(delta)-1));
			}
		}
		forward, reverse: p.Distance_Query_Pair_Result;
		forward, status = p.distance_query_shape_pair(payload, payload, p.BIG_COMPOUND_TYPE_ID, p.BIG_COMPOUND_TYPE_ID,
			e.pose(), pose_b, &simulation.shapes, settings);
		testing.expect_value(t, status, e.Status.Ok);
		reverse, status = p.distance_query_shape_pair(payload, payload, p.BIG_COMPOUND_TYPE_ID, p.BIG_COMPOUND_TYPE_ID,
			pose_b, e.pose(), &simulation.shapes, settings);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect(t, abs(forward.geometry.distance-expected)<4e-6);
		testing.expect(t, abs(forward.geometry.distance-reverse.geometry.distance)<4e-6);
		testing.expect(t, abs(util.vector3_distance(forward.geometry.point_a, forward.geometry.point_b)-expected)<4e-6);
	}
}

@(test)
distance_geometry_mesh_two_sided_scaled_and_tree_pruning :: proc (t: ^testing.T)
{
	world: e.World;
	if !testing.expect_value(t, e.world_init(&world, small_world_description()), e.Status.Ok)
	{
		return;
	}
	defer e.world_destroy(&world);
	simulation: ^p.Simulation;
	status: e.Status;
	simulation, status = e.world_borrow_simulation(&world);
	pool: util.Buffer_Pool;
	testing.expect_value(t, util.buffer_pool_initialize(&pool), util.Memory_Status.Ok);
	defer util.buffer_pool_dispose(&pool);
	triangles: [65]p.Triangle;
	for &triangle, i in triangles
	{
		x: f32 = f32(i)*4;
		triangle = {a={x-1, -1, 0}, b={x+1, -1, 0}, c={x, 1, 0}};
	}
	mesh: p.Mesh;
	testing.expect_value(t, p.mesh_create(&mesh, &triangles[0], len(triangles), {-2, 3, 1}, &pool), p.Physics_Status.Ok);
	defer p.mesh_dispose(&mesh, &pool);
	settings: p.Distance_Query_Settings = p.distance_query_settings_default();
	pose: p.Rigid_Pose = e.pose({32, -16, 8});
	pose.orientation = {0, f32(math.sin(0.31)), 0, f32(math.cos(0.31))};
	for i in 0..<len(triangles)
	{
		for side in ([2]f32{-1, 1})
		{
			point: util.Vector3 = p.rigid_pose_transform({-f32(i)*8, -1, 2*side}, pose);
			result: p.Distance_Query_Pair_Result;
			result, status = p.distance_query_shape_point(point, &mesh, p.MESH_TYPE_ID, pose, &simulation.shapes, settings);
			if !testing.expect_value(t, status, e.Status.Ok)
			{
				continue;
			}
			testing.expect_value(t, result.child_b, i32(i));
			testing.expect(t, abs(result.geometry.distance-2)<1e-4);
			testing.expect(t, abs(util.vector3_distance(result.geometry.point_a, result.geometry.point_b)-2)<1e-4);
		}
	}
}

@(test)
distance_geometry_composite_correction_requeries_all_obstacles :: proc (t: ^testing.T)
{
	world: e.World;
	if !testing.expect_value(t, e.world_init(&world, small_world_description()), e.Status.Ok)
	{
		return;
	}
	defer e.world_destroy(&world);
	simulation: ^p.Simulation;
	status: e.Status;
	simulation, status = e.world_borrow_simulation(&world);
	x_wall, y_wall: e.Shape_Handle;
	x_wall, status = e.shape_add(&world, e.box(2, 8, 8));
	y_wall, status = e.shape_add(&world, e.box(8, 2, 8));
	children: [2]e.Compound_Child = {e.compound_child(x_wall, e.pose({-1, 0, 0})), e.compound_child(y_wall, e.pose({0, -1, 0}))};
	handles: [2]e.Shape_Handle;
	handles[0], status = e.shape_import_compound(&world, children[:]);
	handles[1], status = e.shape_import_big_compound(&world, children[:]);
	settings: p.Distance_Query_Settings = p.distance_query_settings_default();
	storage: Distance_Test_Scratch;
	box: p.Box = {half_width=0.5, half_height=0.5, half_length=0.5};
	for handle in handles
	{
		payload: rawptr;
		batch: ^p.Shape_Batch;
		payload, batch, status = p.shape_registry_resolve(&simulation.shapes, handle);
		correction: p.Distance_Query_Correction;
		correction, status = p.distance_query_depenetrate(&box, payload, p.BOX_TYPE_ID, int(p.typed_index_type(handle)),
			e.pose(), e.pose(), &simulation.shapes, settings, distance_test_scratch(&storage));
		if !testing.expect_value(t, status, e.Status.Ok)
		{
			continue;
		}
		testing.expect(t, correction.iterations>=2);
		testing.expect(t, correction.translation.x>=0.4999 && correction.translation.y>=0.4999);
		for child in children
		{
			obstacle: rawptr;
			obstacle, batch, status = p.shape_registry_resolve(&simulation.shapes, child.shape_index);
			result: p.Distance_Query_Result;
			result, status = p.distance_query_convex(&box, obstacle, p.BOX_TYPE_ID, p.BOX_TYPE_ID,
				e.pose(correction.translation), e.pose(child.local_position), &simulation.shapes, settings, .Penetration, distance_test_scratch(&storage));
			testing.expect_value(t, status, e.Status.Ok);
			testing.expect(t, result.state!=.Penetrating);
		}
		correction, status = p.distance_query_depenetrate(&box, payload, p.BOX_TYPE_ID, int(p.typed_index_type(handle)),
			e.pose(), e.pose(), &simulation.shapes, settings, {});
		testing.expect_value(t, status, e.Status.Capacity_Missing);
		testing.expect_value(t, correction.state, p.Distance_Query_State.Unresolved);
		limited: p.Distance_Query_Settings = settings;
		limited.maximum_iterations=1;
		correction, status = p.distance_query_depenetrate(&box, payload, p.BOX_TYPE_ID, int(p.typed_index_type(handle)),
			e.pose(), e.pose(), &simulation.shapes, limited, distance_test_scratch(&storage));
		testing.expect_value(t, status, e.Status.No_Convergence);
		testing.expect_value(t, correction.translation, util.Vector3{});
	}
}

@(test)
distance_geometry_mesh_correction_respects_registered_sidedness :: proc (t: ^testing.T)
{
	world: e.World;
	if !testing.expect_value(t, e.world_init(&world, small_world_description()), e.Status.Ok)
	{
		return;
	}
	defer e.world_destroy(&world);
	simulation: ^p.Simulation;
	status: e.Status;
	simulation, status = e.world_borrow_simulation(&world);
	pool: util.Buffer_Pool;
	testing.expect_value(t, util.buffer_pool_initialize(&pool), util.Memory_Status.Ok);
	defer util.buffer_pool_dispose(&pool);
	triangle: p.Triangle = {a={-8, -8, 0}, b={8, -8, 0}, c={0, 8, 0}};
	mesh: p.Mesh;
	testing.expect_value(t, p.mesh_create(&mesh, &triangle, 1, {1, 1, 1}, &pool), e.Status.Ok);
	defer p.mesh_dispose(&mesh, &pool);
	settings: p.Distance_Query_Settings = p.distance_query_settings_default();
	storage: Distance_Test_Scratch;
	sphere: p.Sphere = {radius=1};
	for z in ([2]f32{-0.5, 0.5})
	{
		correction: p.Distance_Query_Correction;
		correction, status = p.distance_query_depenetrate(&sphere, &mesh, p.SPHERE_TYPE_ID, p.MESH_TYPE_ID,
			e.pose({0, 0, z}), e.pose(), &simulation.shapes, settings, distance_test_scratch(&storage), &simulation.collision_tasks);
		if !testing.expect_value(t, status, e.Status.Ok)
		{
			continue;
		}
		if z < 0
		{
			testing.expect(t, correction.translation.z < -0.4999);
			result: p.Convex_Contact_Manifold;
			result, status = p.collision_task_registry_test_convex(&simulation.collision_tasks, p.SPHERE_TYPE_ID, p.TRIANGLE_TYPE_ID,
				&sphere, &triangle, e.pose(util.vector3_add({0, 0, z}, correction.translation)), e.pose(), 0, &simulation.shapes);
			testing.expect_value(t, status, e.Status.Ok);
			for contact in result.contacts[:result.count]
			{
				testing.expect(t, contact.depth <= settings.absolute_tolerance);
			}
		}
		else
		{
			testing.expect_value(t, correction.translation, util.Vector3{});
		}
	}
	correction: p.Distance_Query_Correction;
	correction, status = p.distance_query_depenetrate(&sphere, &mesh, p.SPHERE_TYPE_ID, p.MESH_TYPE_ID,
		e.pose({0, 0, -0.5}), e.pose(), &simulation.shapes, settings, distance_test_scratch(&storage));
	testing.expect_value(t, status, e.Status.Invalid_Argument);
	testing.expect_value(t, correction.state, p.Distance_Query_State.Unresolved);
}

@(test)
distance_geometry_custom_leaf_errors_and_pruned_support :: proc (t: ^testing.T)
{
	world: e.World;
	if !testing.expect_value(t, e.world_init(&world, small_world_description()), e.Status.Ok)
	{
		return;
	}
	defer e.world_destroy(&world);
	state: Distance_Late_Failure_State = {base={scale=1}};
	registration: e.Contextual_Custom_Shape_Registration = contextual_sphere_registration(&state.base);
	registration.user_context = &state;
	registration.support = distance_late_failure_support;
	type_id: e.Shape_Type_ID;
	status: e.Status;
	type_id, status = e.custom_shape_register_contextual(&world, registration);
	testing.expect_value(t, status, e.Status.Ok);
	custom, ordinary: e.Shape_Handle;
	custom, status = e.custom_shape_add(&world, type_id, &Custom_Sphere{radius=1});
	testing.expect_value(t, status, e.Status.Ok);
	ordinary, status = e.shape_add(&world, e.sphere(1));
	children: [2]e.Compound_Child = {e.compound_child(ordinary, e.pose()), e.compound_child(custom, e.pose({100, 0, 0}))};
	handles: [2]e.Shape_Handle;
	handles[0], status = e.shape_import_compound(&world, children[:]);
	handles[1], status = e.shape_import_big_compound(&world, children[:]);
	simulation: ^p.Simulation;
	simulation, status = e.world_borrow_simulation(&world);
	for handle in handles
	{
		payload: rawptr;
		batch: ^p.Shape_Batch;
		payload, batch, status = p.shape_registry_resolve(&simulation.shapes, handle);
		state.calls=0;
		state.fail_after=1;
		state.injected_status=.Not_Found;
		result: p.Distance_Query_Pair_Result;
		result, status = p.distance_query_shape_point({2, 0, 0}, payload, int(p.typed_index_type(handle)), e.pose(), &simulation.shapes, p.distance_query_settings_default());
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, state.calls, 0);
		testing.expect_value(t, result.child_b, i32(0));
		probe: p.Sphere = {radius=0.5};
		result, status = p.distance_query_shape_pair(payload, &probe, int(p.typed_index_type(handle)), p.SPHERE_TYPE_ID,
			e.pose(), e.pose({4, 0, 0}), &simulation.shapes, p.distance_query_settings_default());
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, state.calls, 0);
		testing.expect_value(t, result.child_a, i32(0));
		for failure in ([3]e.Status{.Not_Found, .Capacity_Missing, .No_Convergence})
		{
			state.calls=0;
			state.injected_status=failure;
			result, status = p.distance_query_shape_point({102, 0, 0}, payload, int(p.typed_index_type(handle)), e.pose(), &simulation.shapes, p.distance_query_settings_default());
			testing.expect_value(t, status, failure);
			testing.expect_value(t, result.geometry.state, p.Distance_Query_State.Unresolved);
		}
	}
}

@(test)
distance_geometry_hull_and_thin_triangle_scales :: proc (t: ^testing.T)
{
	world: e.World;
	if !testing.expect_value(t, e.world_init(&world, small_world_description()), e.Status.Ok)
	{
		return;
	}
	defer e.world_destroy(&world);
	simulation: ^p.Simulation;
	status: e.Status;
	simulation, status = e.world_borrow_simulation(&world);
	pool: util.Buffer_Pool;
	testing.expect_value(t, util.buffer_pool_initialize(&pool), util.Memory_Status.Ok);
	defer util.buffer_pool_dispose(&pool);
	points: [8]util.Vector3 = {{-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1}, {-1, -1, 1}, {1, -1, 1}, {1, 1, 1}, {-1, 1, 1}};
	starts: [6]i32 = {0, 4, 8, 12, 16, 20};
	indices: [24]i32 = {0, 3, 2, 1, 4, 5, 6, 7, 0, 4, 7, 3, 1, 2, 6, 5, 0, 1, 5, 4, 3, 7, 6, 2};
	hull: p.Convex_Hull;
	testing.expect_value(t, p.convex_hull_create(&hull, &points[0], 8, &starts[0], 6, &indices[0], 24, &pool), e.Status.Ok);
	defer p.convex_hull_dispose(&hull, &pool);
	settings: p.Distance_Query_Settings = p.distance_query_settings_default();
	for point in ([4]util.Vector3{{}, {3, 2, -4}, {1, 0, 0}, {-2, -3, 4}})
	{
		result: p.Distance_Query_Result;
		result, status = p.distance_query_point(point, &hull, p.CONVEX_HULL_TYPE_ID, e.pose(), &simulation.shapes, settings);
		testing.expect_value(t, status, e.Status.Ok);
		x: f64 = max(abs(f64(point.x))-1, f64(0));
		y: f64 = max(abs(f64(point.y))-1, f64(0));
		z: f64 = max(abs(f64(point.z))-1, f64(0));
		testing.expect(t, abs(f64(result.distance)-math.sqrt(x*x+y*y+z*z))<3e-5);
	}
	for scale in ([3]f32{0.001, 1, 10000})
	{
		triangle: p.Triangle = {a={0, 0, 0}, b={scale, 0, 0}, c={scale, scale*0.001, 0}};
		point: util.Vector3 = {scale*0.75, scale*0.00025, scale*0.2};
		settings.absolute_tolerance = scale*1e-6;
		result: p.Distance_Query_Result;
		result, status = p.distance_query_point(point, &triangle, p.TRIANGLE_TYPE_ID, e.pose(), &simulation.shapes, settings);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect(t, abs(result.distance-scale*0.2) < scale*2e-6);
		testing.expect(t, abs(result.point_b.z) < scale*1e-6);
	}
}

@(test)
distance_facade_scalar_context_and_batch_contracts :: proc (t: ^testing.T)
{
	world: e.World;
	if !testing.expect_value(t, e.world_init(&world, small_world_description()), e.Status.Ok)
	{
		return;
	}
	defer e.world_destroy(&world);
	box, sphere, compound: e.Shape_Handle;
	box, _ = e.shape_add(&world, e.box(2, 2, 2));
	sphere, _ = e.shape_add(&world, e.sphere(1));
	children: [2]e.Compound_Child = {e.compound_child(box, e.pose()), e.compound_child(box, e.pose({4, 0, 0}))};
	compound, _ = e.shape_import_big_compound(&world, children[:]);
	point: e.Shape_Distance_Result;
	status: e.Status;
	point, status = e.shape_closest_point(&world, {4, 3, 0}, compound, e.pose());
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, point.child_b, i32(1));
	testing.expect(t, abs(point.geometry.distance-2)<1e-5);
	pair: e.Shape_Distance_Result;
	pair, status = e.shape_distance(&world, sphere, e.pose({8, 0, 0}), compound, e.pose());
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect(t, abs(pair.geometry.distance-2)<1e-5);
	reverse: e.Shape_Distance_Result;
	reverse, status = e.shape_distance(&world, compound, e.pose(), sphere, e.pose({8, 0, 0}));
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, reverse.child_a, pair.child_b);
	testing.expect(t, util.vector3_distance(reverse.geometry.point_a, pair.geometry.point_b)<1e-5);
	testing.expect_value(t, reverse.geometry.normal, util.vector3_negate(pair.geometry.normal));
	pair, status = e.shape_penetration(&world, box, e.pose(), box, e.pose({1.5, 0, 0}));
	testing.expect_value(t, status, e.Status.Capacity_Missing);
	testing.expect_value(t, pair.geometry.state, e.Distance_Query_State.Unresolved);
	testing.expect_value(t, e.distance_query_reserve(&world), e.Status.Ok);
	pair, status = e.shape_penetration(&world, box, e.pose(), box, e.pose({1.5, 0, 0}));
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, pair.geometry.state, e.Distance_Query_State.Penetrating);
	testing.expect(t, abs(pair.geometry.depth-0.5)<1e-4);
	correction: e.Shape_Correction_Result;
	correction, status = e.shape_depenetrate(&world, box, e.pose({1.5, 0, 0}), compound, e.pose());
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect(t, correction.iterations>0);
	query_context: e.Query_Context;
	testing.expect_value(t, e.query_context_init(&query_context, &world), e.Status.Ok);
	defer e.query_context_destroy(&query_context);
	pair, status = e.shape_penetration_with_context(&query_context, box, e.pose(), box, e.pose({1.5, 0, 0}));
	testing.expect_value(t, status, e.Status.Capacity_Missing);
	testing.expect_value(t, e.distance_query_reserve_with_context(&query_context), e.Status.Ok);
	settings: e.Distance_Query_Settings = e.distance_query_settings_default();
	queries: [6]e.Distance_Query = {
		{kind=.Closest_Point, point={4, 3, 0}, shape_b=compound, pose_b=e.pose(), settings=settings},
		{kind=.Distance, shape_a=sphere, shape_b=compound, pose_a=e.pose({8, 0, 0}), pose_b=e.pose(), settings=settings},
		{kind=.Penetration, shape_a=box, shape_b=box, pose_a=e.pose(), pose_b=e.pose({1.5, 0, 0}), settings=settings},
		{kind=.Depenetration, shape_a=box, shape_b=compound, pose_a=e.pose({1.5, 0, 0}), pose_b=e.pose(), settings=settings},
		{kind=.Closest_Point, shape_b=box, pose_b=e.pose()}, // invalid zero tolerances
		{kind=.Closest_Point, point={}, shape_b=box, pose_b=e.pose(), settings=settings},
	};
	results, context_results: [6]e.Distance_Query_Result;
	results[0].status = .Shape_In_Use;
	testing.expect_value(t, e.distance_query_batch(&world, queries[:], results[:2]), e.Status.Invalid_Argument);
	testing.expect_value(t, results[0].status, e.Status.Shape_In_Use);
	testing.expect_value(t, e.distance_query_batch(&world, queries[:], results[:]), e.Status.Invalid_Argument);
	testing.expect_value(t, e.distance_query_batch_with_context(&query_context, queries[:], context_results[:]), e.Status.Invalid_Argument);
	for result, i in results
	{
		testing.expect_value(t, result, context_results[i]);
		testing.expect_value(t, result.status, e.Status.Invalid_Argument if i==4 else e.Status.Ok);
	}
	testing.expect_value(t, results[5].shape.geometry.state, e.Distance_Query_State.Intersecting);
	queries[0].kind = e.Distance_Query_Kind(255);
	testing.expect_value(t, e.distance_query_batch(&world, queries[:1], results[:1]), e.Status.Invalid_Argument);
	testing.expect_value(t, e.world_destroy(&world), e.Status.Invalid_Argument);
}

@(test)
distance_facade_scratch_ownership_reuse_and_failure :: proc (t: ^testing.T)
{
	tracker: allocation_test_tracker;
	description: e.World_Description = small_world_description();
	description.allocator = allocation_test_allocator(&tracker);
	world: e.World;
	if !testing.expect_value(t, e.world_init_with_allocation_scope(&world, description, .All_Owned), e.Status.Ok)
	{
		return;
	}
	box: e.Shape_Handle;
	box, _ = e.shape_add(&world, e.box(2, 2, 2));
	testing.expect_value(t, e.distance_query_reserve(&world), e.Status.Ok);
	query_context: e.Query_Context;
	testing.expect_value(t, e.query_context_init(&query_context, &world), e.Status.Ok);
	testing.expect_value(t, e.distance_query_reserve_with_context(&query_context), e.Status.Ok);
	overlap_add: e.Status;
	_, overlap_add = e.static_add(&world, e.static_body(box, e.pose()), .None);
	testing.expect_value(t, overlap_add, e.Status.Ok);
	overlap_warm: e.Status;
	_, overlap_warm = e.overlap_any(&world, box, e.pose());
	testing.expect_value(t, overlap_warm, e.Status.Ok);
	requests: int = tracker.requests;
	tracker.fail_from = requests+1;
	testing.expect_value(t, e.distance_query_reserve(&world), e.Status.Ok);
	testing.expect_value(t, e.distance_query_reserve(&world, {4, 4, 3}), e.Status.Ok);
	testing.expect_value(t, e.distance_query_reserve_with_context(&query_context), e.Status.Ok);
	for _ in 0..<100
	{
		result: e.Shape_Distance_Result;
		status: e.Status;
		result, status = e.shape_penetration(&world, box, e.pose(), box, e.pose({1.5, 0, 0}));
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect(t, abs(result.geometry.depth-0.5)<1e-4);
		result, status = e.shape_penetration_with_context(&query_context, box, e.pose(), box, e.pose({1.5, 0, 0}));
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect(t, abs(result.geometry.depth-0.5)<1e-4);
		presence: e.Overlap_State;
		overlap_status: e.Status;
		presence, overlap_status = e.overlap_any(&world, box, e.pose());
		testing.expect_value(t, overlap_status, e.Status.Ok);
		testing.expect_value(t, presence, e.Overlap_State.Intersecting);
		presence, overlap_status = e.overlap_any_with_context(&query_context, box, e.pose());
		testing.expect_value(t, overlap_status, e.Status.Ok);
		testing.expect_value(t, presence, e.Overlap_State.Intersecting);
	}
	testing.expect_value(t, tracker.requests, requests);
	testing.expect_value(t, e.distance_query_reserve(&world, {32768, 65536, 32768}), e.Status.Capacity_Missing);
	result: e.Shape_Distance_Result;
	status: e.Status;
	result, status = e.shape_penetration(&world, box, e.pose(), box, e.pose({1.5, 0, 0}));
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect(t, abs(result.geometry.depth-0.5)<1e-4);
	testing.expect_value(t, e.query_context_destroy(&query_context), e.Status.Ok);
	testing.expect_value(t, e.world_destroy(&world), e.Status.Ok);
	allocation_test_empty(t, &tracker);
	// fail each allocation in a new reservation without losing the empty owner
	for failure in 1..=18
	{
		tracker = {};
		world = nil;
		testing.expect_value(t, e.world_init_with_allocation_scope(&world, description, .All_Owned), e.Status.Ok);
		tracker.fail_from = tracker.requests+failure;
		status = e.distance_query_reserve(&world);
		testing.expect(t, status==.Ok || status==.Capacity_Missing);
		testing.expect_value(t, e.world_destroy(&world), e.Status.Ok);
		allocation_test_empty(t, &tracker);
	}
}

Distance_Context_Run :: struct
{
	query_context: e.Query_Context,
	box, compound: e.Shape_Handle,
	failures: int,
}

distance_context_thread :: proc (host: ^thread.Thread)
{
	context = runtime.default_context();
	call: ^Distance_Context_Run = (^Distance_Context_Run)(host.data);
	for _ in 0..<100
	{
		result: e.Shape_Distance_Result;
		status: e.Status;
		result, status = e.shape_penetration_with_context(&call.query_context, call.box, e.pose(), call.box, e.pose({1.5, 0, 0}));
		if status != .Ok || abs(result.geometry.depth-0.5)>1e-4
		{
			call.failures += 1;
		}
		result, status = e.shape_distance_with_context(&call.query_context, call.compound, e.pose(), call.box, e.pose({8, 0, 0}));
		if status != .Ok || abs(result.geometry.distance-2)>1e-4 || result.child_a != 1
		{
			call.failures += 1;
		}
		result, status = e.shape_closest_point_with_context(&call.query_context, {4, 3, 0}, call.compound, e.pose());
		if status != .Ok || abs(result.geometry.distance-2)>1e-4 || result.child_b != 1
		{
			call.failures += 1;
		}
		correction: e.Shape_Correction_Result;
		correction, status = e.shape_depenetrate_with_context(&call.query_context, call.box, e.pose({-1.5, 0, 0}), call.compound, e.pose());
		if status != .Ok || abs(correction.translation.x+0.5)>1e-4
		{
			call.failures += 1;
		}
	}
}

@(test)
distance_facade_parallel_private_scratch_and_borrowed_owner :: proc (t: ^testing.T)
{
	world: e.World;
	if !testing.expect_value(t, e.world_init(&world, small_world_description()), e.Status.Ok)
	{
		return;
	}
	defer e.world_destroy(&world);
	box, compound: e.Shape_Handle;
	box, _ = e.shape_add(&world, e.box(2, 2, 2));
	children: [2]e.Compound_Child = {e.compound_child(box, e.pose()), e.compound_child(box, e.pose({4, 0, 0}))};
	compound, _ = e.shape_import_big_compound(&world, children[:]);
	pool: e.Buffer_Pool;
	testing.expect_value(t, e.buffer_pool_init(&pool, 16384), e.Status.Ok);
	defer e.buffer_pool_destroy(&pool);
	calls: [4]Distance_Context_Run;
	hosts: [4]^thread.Thread;
	for &call, i in calls
	{
		call.box = box;
		call.compound = compound;
		testing.expect_value(t, e.query_context_init(&call.query_context, &world, borrowed_pool=&pool if i==0 else nil), e.Status.Ok);
		testing.expect_value(t, e.distance_query_reserve_with_context(&call.query_context), e.Status.Ok);
		hosts[i] = thread.create(distance_context_thread);
		hosts[i].data = &call;
	}
	for host in hosts
	{
		thread.start(host);
	}
	for host in hosts
	{
		thread.join(host);
		thread.destroy(host);
	}
	for &call in calls
	{
		testing.expect_value(t, call.failures, 0);
		testing.expect_value(t, e.query_context_destroy(&call.query_context), e.Status.Ok);
	}
	testing.expect_value(t, pool.state, util.Pool_State.Ready);
	buffer: util.Buffer(u8);
	memory_status: util.Memory_Status;
	buffer, memory_status = util.buffer_pool_take(&pool, u8, 128);
	testing.expect_value(t, memory_status, util.Memory_Status.Ok);
	testing.expect_value(t, util.buffer_pool_return(&pool, &buffer), util.Memory_Status.Ok);
}

Distance_Reentry_State :: struct
{
	base: Contextual_Shape_State,
	query_context: ^e.Query_Context,
	shape: e.Shape_Handle,
	calls: int,
	query_status, reserve_status, destroy_status: e.Status,
}

distance_context_reentrant_support :: proc "contextless" (
	raw, shape: rawptr, direction: ^util.Vector3,
	access: e.Shape_Access, result: ^util.Vector3,
) -> e.Status
{
	context = runtime.default_context();
	state: ^Distance_Reentry_State = (^Distance_Reentry_State)(raw);
	state.calls += 1;
	_, state.query_status = e.shape_distance_with_context(state.query_context, state.shape, e.pose(), state.shape, e.pose({8, 0, 0}));
	state.reserve_status = e.distance_query_reserve_with_context(state.query_context);
	state.destroy_status = e.query_context_destroy(state.query_context);
	return contextual_shape_support_test(&state.base, shape, direction, access, result);
}

@(test)
distance_facade_context_callback_reentry_is_rejected :: proc (t: ^testing.T)
{
	world: e.World;
	if !testing.expect_value(t, e.world_init(&world, small_world_description()), e.Status.Ok)
	{
		return;
	}
	defer e.world_destroy(&world);
	state: Distance_Reentry_State;
	state.base.scale = 1;
	description: e.Contextual_Custom_Shape_Registration = contextual_sphere_registration(&state.base);
	description.user_context = &state;
	description.support = distance_context_reentrant_support;
	type_id: e.Shape_Type_ID;
	status: e.Status;
	type_id, status = e.custom_shape_register_contextual(&world, description);
	testing.expect_value(t, status, e.Status.Ok);
	shape_data: Custom_Sphere = {radius=1};
	state.shape, status = e.custom_shape_add(&world, type_id, &shape_data);
	testing.expect_value(t, status, e.Status.Ok);
	query_context: e.Query_Context;
	testing.expect_value(t, e.query_context_init(&query_context, &world), e.Status.Ok);
	state.query_context = &query_context;
	result: e.Shape_Distance_Result;
	result, status = e.shape_distance_with_context(&query_context, state.shape, e.pose(), state.shape, e.pose({4, 0, 0}));
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect(t, abs(result.geometry.distance-2)<1e-4);
	testing.expect(t, state.calls>0);
	testing.expect_value(t, state.query_status, e.Status.Invalid_Argument);
	testing.expect_value(t, state.reserve_status, e.Status.Invalid_Argument);
	testing.expect_value(t, state.destroy_status, e.Status.Invalid_Argument);
	testing.expect_value(t, e.query_context_destroy(&query_context), e.Status.Ok);
}

@(test)
distance_geometry_sphere_triangle_analytic_tangency_and_translation :: proc (t: ^testing.T)
{
	world: e.World;
	if !testing.expect_value(t, e.world_init(&world, small_world_description()), e.Status.Ok)
	{
		return;
	}
	defer e.world_destroy(&world);
	simulation: ^p.Simulation;
	status: e.Status;
	simulation, status = e.world_borrow_simulation(&world);
	sphere: e.Sphere = e.sphere(1);
	triangle: e.Triangle = e.triangle({-2, -2, 0}, {2, -2, 0}, {0, 2, 0});
	settings: p.Distance_Query_Settings = p.distance_query_settings_default();
	for angle in ([3]f32{0, 0.3, 0.7})
	{
		triangle_pose: e.Rigid_Pose = e.pose({100, -200, 10});
		triangle_pose.orientation = {0, f32(math.sin(angle)), 0, f32(math.cos(angle))};
		for z in ([7]f32{-3, -1, -0.5, 0, 0.5, 1, 3})
		{
			position: e.Vector3 = p.rigid_pose_transform({0, 0, z}, triangle_pose);
			result, reverse: p.Distance_Query_Result;
			result, status = p.distance_query_convex(&sphere, &triangle, p.SPHERE_TYPE_ID, p.TRIANGLE_TYPE_ID,
				e.pose(position), triangle_pose, &simulation.shapes, settings, .Penetration);
			testing.expect_value(t, status, e.Status.Ok);
			testing.expect_value(t, result.iterations, i32(0));
			if abs(z)<1
			{
				testing.expect_value(t, result.state, p.Distance_Query_State.Penetrating);
				testing.expect(t, abs(result.depth-(1-abs(z)))<2e-5);
				moved: p.Distance_Query_Result;
				moved, status = p.distance_query_convex(&sphere, &triangle, p.SPHERE_TYPE_ID, p.TRIANGLE_TYPE_ID,
					e.pose(util.vector3_add(position, util.vector3_scale(result.normal, result.depth))), triangle_pose, &simulation.shapes, settings, .Penetration);
				testing.expect_value(t, status, e.Status.Ok);
				testing.expect(t, moved.state!=.Penetrating);
			}
			reverse, status = p.distance_query_convex(&triangle, &sphere, p.TRIANGLE_TYPE_ID, p.SPHERE_TYPE_ID,
				triangle_pose, e.pose(position), &simulation.shapes, settings, .Penetration);
			testing.expect_value(t, status, e.Status.Ok);
			testing.expect_value(t, reverse.state, result.state);
			testing.expect_value(t, reverse.point_a, result.point_b);
			testing.expect_value(t, reverse.normal, util.vector3_negate(result.normal));
		}
	}
	// an interior projection must not be reconstructed by cancellation of
	// huge vertices. the unrepaired path returned a false 16,383-unit gap
	for scale in ([4]f32{1e-3, 1, 1e6, 1e20})
	{
		triangle = e.triangle({-scale, -scale, 0}, {scale, -scale, 0}, {0, scale, 0});
		for z in ([3]f32{0, 1, 2})
		{
			result: p.Distance_Query_Result;
			result, status = p.distance_query_convex(&sphere, &triangle, p.SPHERE_TYPE_ID, p.TRIANGLE_TYPE_ID,
				e.pose({0, 0, z}), e.pose(), &simulation.shapes, settings, .Penetration);
			testing.expect_value(t, status, e.Status.Ok);
			testing.expect(t, util.vector3_length(result.point_b)<settings.absolute_tolerance);
			testing.expect_value(t, result.state, p.Distance_Query_State.Penetrating if z==0 else (p.Distance_Query_State.Touching if z==1 else p.Distance_Query_State.Separated));
			testing.expect_value(t, result.depth, max(f32(0), 1-z));
			testing.expect_value(t, result.distance, max(f32(0), z-1));
			if z == 0
			{
				testing.expect_value(t, result.normal, util.Vector3{0, 0, 1});
			}
		}
	}
}

@(test)
distance_facade_failed_growth_preserves_each_owner_and_clear_reuse :: proc (t: ^testing.T)
{
	for failure in 1..=18
	{
		tracker: allocation_test_tracker;
		description: e.World_Description = small_world_description();
		description.allocator = allocation_test_allocator(&tracker);
		world: e.World;
		if !testing.expect_value(t, e.world_init_with_allocation_scope(&world, description, .All_Owned), e.Status.Ok)
		{
			return;
		}
		box: e.Shape_Handle;
		box, _ = e.shape_add(&world, e.box(2, 2, 2));
		testing.expect_value(t, e.distance_query_reserve(&world), e.Status.Ok);
		tracker.fail_from = tracker.requests+failure;
		status: e.Status = e.distance_query_reserve(&world, {32768, 65536, 32768});
		testing.expect(t, status == .Ok || status == .Capacity_Missing);
		requests: int = tracker.requests;
		result: e.Shape_Distance_Result;
		result, status = e.shape_penetration(&world, box, e.pose(), box, e.pose({1.5, 0, 0}));
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect(t, abs(result.geometry.depth-0.5)<1e-4);
		testing.expect_value(t, tracker.requests, requests);
		testing.expect_value(t, e.world_destroy(&world), e.Status.Ok);
		allocation_test_empty(t, &tracker);
	}
	world: e.World;
	if !testing.expect_value(t, e.world_init(&world, small_world_description()), e.Status.Ok)
	{
		return;
	}
	defer e.world_destroy(&world);
	query_context: e.Query_Context;
	testing.expect_value(t, e.query_context_init(&query_context, &world), e.Status.Ok);
	defer e.query_context_destroy(&query_context);
	testing.expect_value(t, e.distance_query_reserve(&world), e.Status.Ok);
	testing.expect_value(t, e.distance_query_reserve_with_context(&query_context), e.Status.Ok);
	for _ in 0..<2
	{
		shape: e.Shape_Handle;
		shape, _ = e.shape_add(&world, e.box(2, 2, 2));
		handle: e.Static_Handle;
		handle, _ = e.static_add(&world, e.static_body(shape, e.pose({4, 0, 0})), .None);
		reference: e.Collidable_Reference;
		reference, _ = p.collidable_reference_create(.Static, int(handle.value));
		resolved: e.Shape_Handle;
		pose_value: e.Rigid_Pose;
		status: e.Status;
		resolved, pose_value, status = e.collidable_shape_pose_with_context(&query_context, reference);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, resolved, shape);
		testing.expect_value(t, pose_value.position, e.Vector3{4, 0, 0});
		result: e.Shape_Distance_Result;
		result, status = e.shape_penetration_with_context(&query_context, shape, e.pose({3, 0, 0}), resolved, pose_value);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect(t, abs(result.geometry.depth-1)<1e-4);
		resolved, pose_value, status = e.collidable_shape_pose(&world, reference);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, resolved, shape);
		testing.expect_value(t, e.world_clear(&world), e.Status.Ok);
		_, _, status = e.collidable_shape_pose(&world, reference);
		testing.expect_value(t, status, e.Status.Not_Found);
	}
}

Distance_Any_Trace :: struct
{
	calls: int,
	ctx: ^e.Query_Context,
	shape: e.Shape_Handle,
	reentry: e.Status,
}

distance_any_trace :: proc "contextless" (raw: rawptr, _collidable: e.Collidable_Reference) -> bool
{
	trace: ^Distance_Any_Trace = cast(^Distance_Any_Trace)raw;
	trace.calls += 1;
	if trace.ctx != nil
	{
		_, trace.reentry = e.overlap_any_with_context(trace.ctx, trace.shape, e.pose());
	}
	return true;
	// existing query_filter callback ABI is boolean
}

@(test)
distance_overlap_any_early_out_geometry_and_context_reentry :: proc (t: ^testing.T)
{
	world: e.World;
	if !testing.expect_value(t, e.world_init(&world, small_world_description()), e.Status.Ok)
	{
		return;
	}
	defer e.world_destroy(&world);
	sphere: e.Shape_Handle;
	sphere, _ = e.shape_add(&world, e.sphere(1));
	for _ in 0..<4
	{
		status: e.Status;
		_, status = e.static_add(&world, e.static_body(sphere, e.pose()), .None);
		testing.expect_value(t, status, e.Status.Ok);
	}
	query_context: e.Query_Context;
	testing.expect_value(t, e.query_context_init(&query_context, &world), e.Status.Ok);
	defer e.query_context_destroy(&query_context);
	trace: Distance_Any_Trace;
	state: e.Overlap_State;
	status: e.Status;
	state, status = e.overlap_any(&world, sphere, e.pose(), {allow=distance_any_trace, user_context=&trace});
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, state, e.Overlap_State.Intersecting);
	testing.expect_value(t, trace.calls, 1);
	// not an all-hit query into a one-element buffer
	trace = {ctx=&query_context, shape=sphere};
	state, status = e.overlap_any_with_context(&query_context, sphere, e.pose(), {allow=distance_any_trace, user_context=&trace});
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, state, e.Overlap_State.Intersecting);
	testing.expect_value(t, trace.calls, 1);
	testing.expect_value(t, trace.reentry, e.Status.Invalid_Argument);
	for offset in ([3]util.Vector3{{1.9, 1.9, 0}, {3, 0, 0}, {2, 0, 0}})
	{
		state, status = e.overlap_any(&world, sphere, e.pose(offset));
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, state, e.Overlap_State.Intersecting if offset.x==2 else e.Overlap_State.Separated);
	}
	state, status = e.overlap_any(&world, sphere, e.pose({math.inf_f32(1), 0, 0}));
	testing.expect_value(t, status, e.Status.Invalid_Argument);
	testing.expect_value(t, state, e.Overlap_State.Separated);
}

Distance_Any_Callback :: struct
{
	depth: f32,
	status: e.Status,
}

distance_any_scalar :: proc "contextless" (
	user_context: rawptr, _a, _b: rawptr, _pa, _pb: p.Rigid_Pose,
	_margin: f32, _shapes: ^p.Shape_Registry,
) -> (p.Convex_Contact_Manifold, e.Status)
{
	state: ^Distance_Any_Callback = cast(^Distance_Any_Callback)user_context;
	manifold: p.Convex_Contact_Manifold = {count=1, normal={0, 1, 0}};
	manifold.contacts[0].depth = state.depth;
	return manifold, state.status;
}

distance_any_wide :: proc "contextless" (
	user_context: rawptr, bundle: ^p.Collision_Convex_Wide_Bundle,
	_shapes: ^p.Shape_Registry, result: ^p.Collision_Wide_Manifold_Result,
) -> e.Status
{
	state: ^Distance_Any_Callback = cast(^Distance_Any_Callback)user_context;
	result^ = {kind=.One_Contact};
	for lane in 0..<bundle.count
	{
		result.one.normal.y = simd.replace(result.one.normal.y, lane, 1);
		result.one.depth = simd.replace(result.one.depth, lane, state.depth);
		result.one.contact_exists = simd.replace(result.one.contact_exists, lane, -1);
	}
	return state.status;
}

@(test)
distance_overlap_any_negative_contacts_and_task_errors_are_not_hits :: proc (t: ^testing.T)
{
	f: Contextual_Task_Fixture;
	if contextual_shape_only(t, &f, small_world_description()) != .Ok
	{
		return;
	}
	defer e.world_destroy(&f.world);
	add_status: e.Status;
	_, add_status = e.static_add(&f.world, e.static_body(f.sphere, e.pose()), .None);
	testing.expect_value(t, add_status, e.Status.Ok);
	state: e.Overlap_State;
	status: e.Status;
	state, status = e.overlap_any(&f.world, f.custom, e.pose());
	testing.expect_value(t, status, e.Status.Not_Found);
	testing.expect_value(t, state, e.Overlap_State.Separated);
	callback: Distance_Any_Callback;
	_, status = e.collision_task_register_contextual(&f.world, {shape_type_a=f.type_id, shape_type_b=e.SHAPE_TYPE_SPHERE,
			batch_size=16, user_context=&callback, test=distance_any_scalar, wide_test=distance_any_wide});
	testing.expect_value(t, status, e.Status.Ok);
	for depth in ([3]f32{-0.01, 0, 0.1})
	{
		callback.depth = depth;
		state, status = e.overlap_any(&f.world, f.custom, e.pose());
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, state, e.Overlap_State.Separated if depth<0 else e.Overlap_State.Intersecting);
	}
	for failure in ([3]e.Status{.Not_Found, .Capacity_Missing, .No_Convergence})
	{
		callback.status = failure;
		state, status = e.overlap_any(&f.world, f.custom, e.pose());
		testing.expect_value(t, status, failure);
		testing.expect_value(t, state, e.Overlap_State.Separated);
	}
}

@(test)
distance_overlap_any_composite_filters_and_failure_recovery :: proc (t: ^testing.T)
{
	f: Contextual_Task_Fixture;
	if contextual_shape_only(t, &f, small_world_description()) != .Ok || contextual_bind_fixture(t, &f) != .Ok
	{
		return;
	}
	defer e.world_destroy(&f.world);
	children: [2]e.Compound_Child = {e.compound_child(f.custom, e.pose({-0.4, 0, 0})), e.compound_child(f.custom, e.pose({0.4, 0, 0}))};
	compound: e.Shape_Handle;
	status: e.Status;
	compound, status = e.shape_import_compound(&f.world, children[:]);
	testing.expect_value(t, status, e.Status.Ok);
	_, status = e.static_add(&f.world, e.static_body(compound, e.pose()), .None);
	testing.expect_value(t, status, e.Status.Ok);
	query_context: e.Query_Context;
	testing.expect_value(t, e.query_context_init(&query_context, &f.world), e.Status.Ok);
	defer e.query_context_destroy(&query_context);
	state: e.Overlap_State;
	state, status = e.overlap_any(&f.world, f.sphere, e.pose(), {allow_child=query_reject_every_child});
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, state, e.Overlap_State.Separated);
	for failure in ([3]e.Status{.Not_Found, .Capacity_Missing, .Invalid_Description})
	{
		f.state.wide_status = failure;
		state, status = e.overlap_any(&f.world, f.sphere, e.pose());
		testing.expect_value(t, status, failure);
		testing.expect_value(t, state, e.Overlap_State.Separated);
		state, status = e.overlap_any_with_context(&query_context, f.sphere, e.pose());
		testing.expect_value(t, status, failure);
		testing.expect_value(t, state, e.Overlap_State.Separated);
		f.state.wide_status = .Ok;
		state, status = e.overlap_any_with_context(&query_context, f.sphere, e.pose());
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, state, e.Overlap_State.Intersecting);
		state, status = e.overlap_any(&f.world, f.sphere, e.pose());
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, state, e.Overlap_State.Intersecting);
	}
}

@(test)
distance_overlap_any_does_not_publish_or_consume_trigger_history :: proc (t: ^testing.T)
{
	scene: Trigger_Test_Scene;
	if trigger_test_scene(t, &scene) != .Ok
	{
		return;
	}
	defer e.world_destroy(&scene.world);
	testing.expect_value(t, e.world_step(&scene.world, 0.01), e.Status.Ok);
	query_context: e.Query_Context;
	testing.expect_value(t, e.query_context_init(&query_context, &scene.world), e.Status.Ok);
	defer e.query_context_destroy(&query_context);
	for _ in 0..<8
	{
		state: e.Overlap_State;
		status: e.Status;
		state, status = e.overlap_any_with_context(&query_context, scene.shape, e.pose());
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, state, e.Overlap_State.Intersecting);
	}
	events: [2]e.Trigger_Event;
	written, required: int;
	status: e.Status;
	written, required, status = e.trigger_events_drain(&scene.world, events[:]);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, written, 1);
	testing.expect_value(t, required, 1);
	testing.expect_value(t, events[0].kind, e.Trigger_Event_Kind.Enter);
}

@(test)
distance_custom_smooth_penetration_explicit_budget_and_recheck :: proc (t: ^testing.T)
{
	f: Contextual_Task_Fixture;
	if contextual_shape_only(t, &f, small_world_description()) != .Ok
	{
		return;
	}
	defer e.world_destroy(&f.world);
	testing.expect_value(t, e.distance_query_reserve(&f.world, {512, 1024, 1536}), e.Status.Ok);
	settings: e.Distance_Query_Settings = e.distance_query_settings_default();
	settings.maximum_iterations = 1;
	result: e.Shape_Distance_Result;
	status: e.Status;
	result, status = e.shape_penetration(&f.world, f.sphere, e.pose({0, 0.25, 0}), f.custom, e.pose(), settings);
	testing.expect_value(t, status, e.Status.No_Convergence);
	testing.expect_value(t, result.geometry.state, e.Distance_Query_State.Unresolved);
	settings.maximum_iterations = 512;
	for x in ([5]f32{0, 0.005, 0.01, 0.025, 0.04})
	{
		result, status = e.shape_penetration(&f.world, f.sphere, e.pose({x, 0.25, 0}), f.custom, e.pose(), settings);
		if !testing.expect_value(t, status, e.Status.Ok)
		{
			continue;
		}
		expected: f64 = 1.5-math.sqrt(f64(x)*f64(x)+0.25*0.25);
		testing.expect_value(t, result.geometry.state, e.Distance_Query_State.Penetrating);
		testing.expect(t, abs(f64(result.geometry.depth)-expected) < 3e-5);
		correction: e.Shape_Correction_Result;
		correction, status = e.shape_depenetrate(&f.world, f.sphere, e.pose({x, 0.25, 0}), f.custom, e.pose(), settings);
		if !testing.expect_value(t, status, e.Status.Ok)
		{
			continue;
		}
		position: util.Vector3 = util.vector3_add({x, 0.25, 0}, correction.translation);
		separation: f64 = math.sqrt(f64(position.x)*f64(position.x)+f64(position.y)*f64(position.y)+f64(position.z)*f64(position.z));
		testing.expect(t, separation >= 1.5-1e-5);
	}
}

@(test)
distance_overlap_any_reports_missing_compound_child_route :: proc (t: ^testing.T)
{
	f: Contextual_Task_Fixture;
	if contextual_shape_only(t, &f, small_world_description()) != .Ok
	{
		return;
	}
	defer e.world_destroy(&f.world);
	children: [1]e.Compound_Child = {e.compound_child(f.custom, e.pose())};
	compound: e.Shape_Handle;
	status: e.Status;
	compound, status = e.shape_import_compound(&f.world, children[:]);
	testing.expect_value(t, status, e.Status.Ok);
	target: e.Static_Handle;
	target, status = e.static_add(&f.world, e.static_body(compound, e.pose()), .None);
	testing.expect_value(t, status, e.Status.Ok);
	query_context: e.Query_Context;
	testing.expect_value(t, e.query_context_init(&query_context, &f.world), e.Status.Ok);
	defer e.query_context_destroy(&query_context);
	state: e.Overlap_State;
	state, status = e.overlap_any(&f.world, f.sphere, e.pose());
	testing.expect_value(t, status, e.Status.Not_Found);
	testing.expect_value(t, state, e.Overlap_State.Separated);
	state, status = e.overlap_any_with_context(&query_context, f.sphere, e.pose());
	testing.expect_value(t, status, e.Status.Not_Found);
	testing.expect_value(t, state, e.Overlap_State.Separated);
	state, status = e.overlap_any_with_context(&query_context, f.sphere, e.pose(), {allow_child=query_reject_every_child});
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, state, e.Overlap_State.Separated);
	// the original query/target child ordering must survive the flipped parent route
	testing.expect_value(t, e.static_remove(&f.world, target, .None), e.Status.Ok);
	_, status = e.static_add(&f.world, e.static_body(f.sphere, e.pose()), .None);
	testing.expect_value(t, status, e.Status.Ok);
	state, status = e.overlap_any_with_context(&query_context, compound, e.pose());
	testing.expect_value(t, status, e.Status.Not_Found);
	testing.expect_value(t, state, e.Overlap_State.Separated);
	if contextual_bind_fixture(t, &f) != .Ok
	{
		return;
	}
	state, status = e.overlap_any_with_context(&query_context, compound, e.pose());
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, state, e.Overlap_State.Intersecting);
}

@(test)
distance_hierarchy_parallel_private_scratch_and_borrowed_owner :: proc (t: ^testing.T)
{
	world: e.World;
	if !testing.expect_value(t, e.world_init(&world, small_world_description()), e.Status.Ok)
	{
		return;
	}
	defer e.world_destroy(&world);
	box, compound: e.Shape_Handle;
	box, _ = e.shape_add(&world, e.box(2, 2, 2));
	inner: e.Shape_Handle;
	status: e.Status;
	inner, status = e.shape_import_compound(&world, []e.Compound_Child{e.compound_child(box, e.pose())});
	testing.expect_value(t, status, e.Status.Ok);
	children: [2]e.Compound_Child = {e.compound_child(inner, e.pose()), e.compound_child(inner, e.pose({4, 0, 0}))};
	compound, _ = e.shape_import_big_compound(&world, children[:]);
	pool: e.Buffer_Pool;
	testing.expect_value(t, e.buffer_pool_init(&pool, 16384), e.Status.Ok);
	defer e.buffer_pool_destroy(&pool);
	calls: [4]Distance_Context_Run;
	hosts: [4]^thread.Thread;
	for &call, i in calls
	{
		call.box = box;
		call.compound = compound;
		testing.expect_value(t, e.query_context_init(&call.query_context, &world, borrowed_pool=&pool if i==0 else nil), e.Status.Ok);
		testing.expect_value(t, e.distance_query_reserve_with_context(&call.query_context), e.Status.Ok);
		hosts[i] = thread.create(distance_context_thread);
		hosts[i].data = &call;
	}
	for host in hosts
	{
		thread.start(host);
	}
	for host in hosts
	{
		thread.join(host);
		thread.destroy(host);
	}
	for &call in calls
	{
		testing.expect_value(t, call.failures, 0);
		testing.expect_value(t, e.query_context_destroy(&call.query_context), e.Status.Ok);
	}
	testing.expect_value(t, pool.state, util.Pool_State.Ready);
	buffer: util.Buffer(u8);
	memory_status: util.Memory_Status;
	buffer, memory_status = util.buffer_pool_take(&pool, u8, 128);
	testing.expect_value(t, memory_status, util.Memory_Status.Ok);
	testing.expect_value(t, util.buffer_pool_return(&pool, &buffer), util.Memory_Status.Ok);
}
