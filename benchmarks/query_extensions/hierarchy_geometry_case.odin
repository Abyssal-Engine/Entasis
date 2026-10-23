package query_extensions

import "base:runtime"
import "core:fmt"
import "core:math"
import "core:os"
import "core:strings"
import "core:time"
import e "entasis:entasis"
import cooking "entasis:entasis_cooking"
import util "entasis:entasis_utilities"
import support "../benchmark_support"

Hierarchy_Geometry_Observation :: struct
{
	status: e.Status,
	count: int,
	shape: e.Shape_Distance_Result,
	correction: e.Shape_Correction_Result,
	sweep: e.Sweep_Hit,
	global_points: [2][3]f64,
}

Hierarchy_Geometry_Case :: struct
{
	world: e.World,
	target, query: e.Shape_Handle,
	target_pose: e.Rigid_Pose,
	contexts: []e.Query_Context,
	poses: []e.Rigid_Pose,
	observations: []Hierarchy_Geometry_Observation,
	args: Query_Arguments,
	count, passes: int,
}

hierarchy_geometry_prepare :: proc(fixture: ^Hierarchy_Geometry_Case) -> e.Status
{
	status: e.Status;
	fixture.target_pose = e.pose({16, -8, 32}, {0, 1, 0, 0});
	support.extension_require(e.world_init(&fixture.world, world_description(1)), "hierarchy geometry world");
	box, sphere: e.Shape_Handle;
	box, status = e.shape_add(&fixture.world, e.box(1, 1, 1));
	support.extension_require(status, "hierarchy geometry box");
	sphere, status = e.shape_add(&fixture.world, e.sphere(0.25));
	support.extension_require(status, "hierarchy geometry sphere");
	fixture.target, fixture.query = box, sphere;
	cell: string = fixture.args.query_cell;
	if strings.contains(cell, "mesh128")
	{
		triangles: [128]e.Triangle;
		for z in 0..<8
		{
			for x in 0..<8
			{
				a: e.Vector3 = {f32(x), 0, f32(z)};
				b: e.Vector3 = {f32(x+1), 0, f32(z)};
				c: e.Vector3 = {f32(x+1), 0, f32(z+1)};
				d: e.Vector3 = {f32(x), 0, f32(z+1)};
				triangles[2*(8*z+x)] = e.triangle(a, b, c);
				triangles[2*(8*z+x)+1] = e.triangle(a, c, d);
			}
		}
		cooker: cooking.Cooking_Context;
		support.extension_require(cooking.cooking_context_init(&cooker), "hierarchy geometry cooker");
		defer cooking.cooking_context_destroy(&cooker);
		mesh: cooking.Cooked_Mesh;
		mesh, status = cooking.cook_mesh(&cooker, triangles[:]);
		support.extension_require(status, "hierarchy geometry cook");
		defer cooking.cooked_mesh_destroy(&mesh);
		fixture.target, status = cooking.cooked_mesh_import(&fixture.world, &mesh);
		support.extension_require(status, "hierarchy geometry mesh");
		wrapper: [1]e.Compound_Child = {e.compound_child(fixture.target, e.pose())};
		fixture.target, status = e.shape_import_compound(&fixture.world, wrapper[:]);
		support.extension_require(status, "hierarchy geometry mesh wrapper");
	}
	else if cell != "query-box-1"
	{
		columns, rows: [8]e.Compound_Child;
		for &child, column in columns
		{
			child = e.compound_child(box, e.pose({4*f32(column), 0, 0}));
		}
		row: e.Shape_Handle;
		row, status = e.shape_import_big_compound(&fixture.world, columns[:]);
		support.extension_require(status, "hierarchy geometry row");
		for &child, index in rows
		{
			child = e.compound_child(row, e.pose({0, 0, 4*f32(index)}));
		}
		fixture.target, status = e.shape_import_compound(&fixture.world, rows[:]);
		support.extension_require(status, "hierarchy geometry grid");
	}
	if cell != "query-box-1" && fixture.args.query_operation == "distance" || strings.has_prefix(cell, "compound4-")
	{
		children: [8]e.Compound_Child;
		count: int = 4 if strings.has_prefix(cell, "compound4-") else 8;
		spacing: f32 = 1.5 if strings.contains(cell, "mesh128") else 4;
		for &child, index in children[:count]
		{
			child = e.compound_child(sphere, e.pose({spacing*f32(index), 0, 0}));
		}
		fixture.query, status = e.shape_import_compound(&fixture.world, children[:count]);
		support.extension_require(status, "hierarchy geometry query union");
	}
	_, status = e.static_add(&fixture.world, e.static_body(fixture.target, fixture.target_pose), .None);
	support.extension_require(status, "hierarchy geometry static");
	for &pose, index in fixture.poses
	{
		local: e.Vector3 = {0, 2+2*f32(index%2), 4*f32((index/2)%8)};
		if fixture.args.query_operation == "closest-point"
		{
			local.x = 4*f32((index/16)%8);
		}
		if fixture.args.query_operation == "depenetration"
		{
			local.y = 0.625 if index%2 == 0 else 2;
		}
		if strings.has_prefix(fixture.args.query_operation, "sweep-")
		{
			local.y = 4;
			if index%2 != 0
			{
				local.z = 40;
			}
		}
		if cell == "query-box-1"
		{
			local.x, local.z = 0, 0 if index%2 == 0 || !strings.has_prefix(fixture.args.query_operation, "sweep-") else 40;
		}
		if strings.contains(cell, "mesh128")
		{
			local.x = 0.75;
			if local.z != 40
			{
				local.z = f32((index/2)%8)+0.25;
			}
		}
		pose = e.pose(util.vector3_add(fixture.target_pose.position, util.quaternion_transform(local, fixture.target_pose.orientation)), fixture.target_pose.orientation);
	}
	for &query_context in fixture.contexts
	{
		support.extension_require(e.query_context_init(&query_context, &fixture.world), "hierarchy geometry context");
		support.extension_require(e.distance_query_reserve_with_context(&query_context, {128, 256, 384}), "hierarchy geometry reserve");
	}
	return .Ok;
}

hierarchy_geometry_query :: proc "contextless" (fixture: ^Hierarchy_Geometry_Case, worker, index: int) -> Hierarchy_Geometry_Observation
{
	observation: Hierarchy_Geometry_Observation;
	query_context: ^e.Query_Context = &fixture.contexts[worker];
	pose: e.Rigid_Pose = fixture.poses[index];
	if fixture.args.query_case == "global"
	{
		when e.PRECISION_NAME == "mixed64"
		{
			pose_a: e.Global_Pose = {position={1e9+f64(pose.position.x), f64(pose.position.y), f64(pose.position.z)}, orientation=pose.orientation};
			pose_b: e.Global_Pose = {position={1e9+f64(fixture.target_pose.position.x), f64(fixture.target_pose.position.y), f64(fixture.target_pose.position.z)}, orientation=fixture.target_pose.orientation};
			result: e.Global_Shape_Distance_Result;
			switch fixture.args.query_operation
			{
			case "closest-point":
				result, observation.status = e.shape_closest_point_global_with_context(query_context, pose_a.position, fixture.target, pose_b);
			case "distance":
				result, observation.status = e.shape_distance_global_with_context(query_context, fixture.query, pose_a, fixture.target, pose_b);
			case "depenetration":
				observation.correction, observation.status = e.shape_depenetrate_global_with_context(query_context, fixture.query, pose_a, fixture.target, pose_b);
			}
			observation.shape = {geometry={state=result.geometry.state, normal=result.geometry.normal, distance=result.geometry.distance, depth=result.geometry.depth, iterations=result.geometry.iterations}, child_a=result.child_a, child_b=result.child_b};
			observation.global_points = {{result.geometry.point_a.x, result.geometry.point_a.y, result.geometry.point_a.z}, {result.geometry.point_b.x, result.geometry.point_b.y, result.geometry.point_b.z}};
		}
	}
	else
	{
		switch fixture.args.query_operation
		{
		case "closest-point":
			observation.shape, observation.status = e.shape_closest_point_with_context(query_context, pose.position, fixture.target, fixture.target_pose);
		case "distance":
			observation.shape, observation.status = e.shape_distance_with_context(query_context, fixture.query, pose, fixture.target, fixture.target_pose);
		case "depenetration":
			observation.correction, observation.status = e.shape_depenetrate_with_context(query_context, fixture.query, pose, fixture.target, fixture.target_pose);
		case:
			motion: e.Body_Velocity = e.velocity(util.quaternion_transform({0.25, -4, 0.125}, fixture.target_pose.orientation));
			if (fixture.args.query_cell == "compound4-nested64-angular" || fixture.args.query_cell == "compound4-mesh128-angular")
			{
				motion.angular = util.quaternion_transform({1, 0, 0}, fixture.target_pose.orientation);
			}
			switch fixture.args.query_operation
			{
			case "sweep-any":
				state: e.Query_Hit_State;
				state, observation.status = e.sweep_any_with_context(query_context, fixture.query, pose, motion, 1);
				observation.count = 1 if state == .Hit else 0;
			case "sweep-closest":
				observation.sweep, observation.status = e.sweep_closest_with_context(query_context, fixture.query, pose, motion, 1);
				observation.count = 1 if observation.status == .Ok else 0;
			case "sweep-all":
				hits: [1]e.Sweep_Hit;
				observation.count, observation.status = e.sweep_all_with_context(query_context, fixture.query, pose, motion, 1, hits[:]);
				observation.sweep = hits[0];
			}
			return observation;
		}
	}
	observation.count = 1 if observation.status == .Ok else 0;
	return observation;
}

hierarchy_geometry_worker :: proc "contextless" (worker: int, dispatcher: ^util.Thread_Dispatcher_Boundary)
{
	fixture: ^Hierarchy_Geometry_Case = (^Hierarchy_Geometry_Case)(dispatcher.unmanaged_context);
	width: int = HIERARCHY_INPUTS/fixture.args.workers;
	for _ in 0..<fixture.passes
	{
		for offset in 0..<fixture.count
		{
			index: int = worker*width+offset%width;
			fixture.observations[index] = hierarchy_geometry_query(fixture, worker, index);
		}
	}
}

hierarchy_geometry_validate :: proc "contextless" (fixture: ^Hierarchy_Geometry_Case, index: int) -> e.Status
{
	observation: Hierarchy_Geometry_Observation = fixture.observations[index];
	operation, cell: string = fixture.args.query_operation, fixture.args.query_cell;
	if (operation == "sweep-any" || operation == "sweep-closest" || operation == "sweep-all")
	{
		count: int = 1 if index%2 == 0 else 0;
		status: e.Status = .Not_Found if count == 0 && operation == "sweep-closest" else .Ok;
		if observation.status != status || observation.count != count
		{
			return .Invalid_Description;
		}
		if count > 0 && operation != "sweep-any"
		{
			expected_t: f32 = 0.9375 if cell == "compound4-mesh128" || cell == "compound4-mesh128-angular" else 0.8125;
			if observation.sweep.sweep.state != .Hit || !(abs(observation.sweep.sweep.t1-expected_t) <= 5e-4) ||
				!(observation.sweep.sweep.t0 <= observation.sweep.sweep.t1 && observation.sweep.sweep.t0 >= 0) ||
				!(abs(observation.sweep.sweep.normal.x) <= 1e-4 && observation.sweep.sweep.normal.y >= 0.999 && observation.sweep.sweep.normal.y <= 1.001 && abs(observation.sweep.sweep.normal.z) <= 1e-4) ||
				!(abs(observation.sweep.sweep.location.x) <= 256 && abs(observation.sweep.sweep.location.y) <= 256 && abs(observation.sweep.sweep.location.z) <= 256)
			{
				return .Invalid_Description;
			}
		}
		return .Ok;
	}
	if observation.status != .Ok || observation.count != 1
	{
		return .Invalid_Description;
	}
	if operation == "depenetration"
	{
		expected: f32 = 0.125 if index%2 == 0 else 0;
		if !(abs(observation.correction.translation.y-expected) <= 1e-4) ||
			!(abs(observation.correction.translation.x) <= 1e-4 && abs(observation.correction.translation.z) <= 1e-4) ||
			(observation.correction.state != .Separated && observation.correction.state != .Touching) ||
			(observation.correction.iterations != 1 && index%2 == 0) || (observation.correction.iterations != 0 && index%2 != 0)
		{
			return .Invalid_Description;
		}
		return .Ok;
	}
	expected_distance: f32 = 1.5+2*f32(index%2);
	if operation == "distance"
	{
		expected_distance -= 0.25;
	}
	geometry: e.Distance_Query_Geometry = observation.shape.geometry;
	expected_a: i32 = 0 if operation == "distance" && cell != "query-box-1" else -1;
	expected_b: i32 = i32((index/2)%8) if cell != "query-box-1" else -1;
	if geometry.state != .Separated || !(abs(geometry.distance-expected_distance) <= 1e-4) ||
		geometry.depth != 0 || observation.shape.child_a != expected_a || observation.shape.child_b != expected_b ||
		!(abs(geometry.normal.x) <= 1e-4 && abs(geometry.normal.y-1) <= 1e-4 && abs(geometry.normal.z) <= 1e-4)
	{
		return .Invalid_Description;
	}
	point_a: e.Vector3 = fixture.poses[index].position;
	if operation == "distance"
	{
		point_a.y -= 0.25;
	}
	point_b: e.Vector3 = point_a;
	point_b.y = fixture.target_pose.position.y+0.5;
	points: [2][3]f64 = {
		{f64(geometry.point_a.x), f64(geometry.point_a.y), f64(geometry.point_a.z)},
		{f64(geometry.point_b.x), f64(geometry.point_b.y), f64(geometry.point_b.z)},
	};
	if fixture.args.query_case == "global"
	{
		points = observation.global_points;
		points[0][0] -= 1e9;
		points[1][0] -= 1e9;
	}
	if operation == "closest-point"
	{
		for expected, side in ([2]e.Vector3{point_a, point_b})
		{
			for value, axis in ([3]f64{f64(expected.x), f64(expected.y), f64(expected.z)})
			{
				if !(abs(points[side][axis]-value) <= 1e-4)
				{
					return .Invalid_Description;
				}
			}
		}
	}
	else
	{
		// GJK witnesses may move tangentially while satisfying the same surfaces
		center: e.Vector3 = fixture.poses[index].position;
		dx, dy, dz: f64 = points[0][0]-f64(center.x), points[0][1]-f64(center.y), points[0][2]-f64(center.z);
		sx, sy, sz: f64 = points[0][0]-points[1][0], points[0][1]-points[1][1], points[0][2]-points[1][2];
		if !(abs(math.sqrt(dx*dx+dy*dy+dz*dz)-0.25) <= 1e-4) ||
			!(abs(points[1][0]-f64(center.x)) <= 0.5 && abs(points[1][2]-f64(center.z)) <= 0.5) ||
			!(abs(points[1][1]-f64(point_b.y)) <= 1e-4) ||
			!(abs(math.sqrt(sx*sx+sy*sy+sz*sz)-f64(expected_distance)) <= 1e-4)
		{
			return .Invalid_Description;
		}
	}
	return .Ok;
}

run_hierarchy_geometry_case :: proc(args: Query_Arguments) -> e.Status
{
	fixture: Hierarchy_Geometry_Case = {args=args};
	context_error, pose_error, observation_error: runtime.Allocator_Error;
	fixture.contexts, context_error = make([]e.Query_Context, args.workers);
	defer delete(fixture.contexts);
	fixture.poses, pose_error = make([]e.Rigid_Pose, HIERARCHY_INPUTS);
	defer delete(fixture.poses);
	fixture.observations, observation_error = make([]Hierarchy_Geometry_Observation, HIERARCHY_INPUTS);
	defer delete(fixture.observations);
	if context_error != nil || pose_error != nil || observation_error != nil
	{
		panic("hierarchy geometry storage");
	}
	defer e.world_destroy(&fixture.world);
	defer
	{
		for &query_context in fixture.contexts
		{
			e.query_context_destroy(&query_context);
		}
	}
	support.extension_require(hierarchy_geometry_prepare(&fixture), "hierarchy geometry prepare");
	dispatcher: util.Thread_Dispatcher;
	if util.thread_dispatcher_initialize(&dispatcher, args.workers) != .Ok
	{
		return .Capacity_Missing;
	}
	defer util.thread_dispatcher_shutdown(&dispatcher);
	boundary: ^util.Thread_Dispatcher_Boundary = util.thread_dispatcher_boundary(&dispatcher);
	fixture.count, fixture.passes = 128, 1;
	if boundary.dispatch(boundary, hierarchy_geometry_worker, args.workers, &fixture) != .Ok
	{
		return .Invalid_Description;
	}
	fixture.count = HIERARCHY_INPUTS/args.workers;
	if boundary.dispatch(boundary, hierarchy_geometry_worker, args.workers, &fixture) != .Ok
	{
		return .Invalid_Description;
	}
	for index in 0..<HIERARCHY_INPUTS
	{
		support.extension_require(hierarchy_geometry_validate(&fixture, index), "hierarchy geometry independent oracle");
	}
	fixture.passes = HIERARCHY_PASSES;
	start: time.Tick = time.tick_now();
	dispatch_status: util.Threading_Status = boundary.dispatch(boundary, hierarchy_geometry_worker, args.workers, &fixture);
	elapsed: f64 = f64(time.tick_diff(start, time.tick_now()))/1e6;
	if dispatch_status != .Ok || !(elapsed > 0 && elapsed < math.F64_MAX)
	{
		return .Invalid_Description;
	}
	results: u64;
	for index in 0..<HIERARCHY_INPUTS
	{
		support.extension_require(hierarchy_geometry_validate(&fixture, index), "hierarchy geometry measured oracle");
		results += u64(fixture.observations[index].count)*HIERARCHY_PASSES;
	}
	file: ^os.File;
	state: support.Extension_CSV_State;
	file, state = support.extension_csv_open(args.output);
	if state == .Empty
	{
		if fmt.fprintln(file, "worker_count,case_status,metric_status,physics_elapsed_ms,benchmark_components,precision,query_case,query_operation,query_cell,input_count,passes,query_count,warmup_queries,hit_count,checksum,oracle_queries") <= 0
		{
			panic("hierarchy geometry CSV header");
		}
	}
	if fmt.fprintf(file, "%d,ok,ok,%.9f,all,%s,%s,%s,%s,1536,8,12288,%d,%d,%d,1536\n", args.workers, elapsed, e.PRECISION_NAME, args.query_case, args.query_operation, args.query_cell, 128*args.workers, results, results) <= 0
	{
		panic("hierarchy geometry CSV row");
	}
	support.extension_csv_close(file);
	return .Ok;
}
