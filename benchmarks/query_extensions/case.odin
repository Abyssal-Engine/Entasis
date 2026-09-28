package query_extensions

import "core:fmt"
@(require)
import "core:math"
import e "entasis:entasis"
import cooking "entasis:entasis_cooking"
import support "../benchmark_support"

// fixed sample counts keep route and operation workloads comparable
QUERY_COUNT :: 8192;
WARMUP_COUNT :: 128;
ROUTES: [4]string = {"convex", "compound", "mesh", "native_custom"};
OPERATIONS: [8]string = {"ray_closest", "ray_filtered", "ray_all", "sweep_closest", "sweep_all", "geometric_overlap", "direct_collision", "collision_batch"};
Retention :: enum
{
	Discard, Keep,
}

Ray_Results :: struct
{
	hits: [8]e.Ray_Hit,
}
Sweep_Results :: struct
{
	hits: [8]e.Sweep_Hit,
}
Overlap_Results :: struct
{
	hits: [8]e.Overlap_Hit,
}
Collision_Results :: struct
{
	manifolds: [2]e.Manifold_Result,
}
Observation :: struct
{
	origin: e.Vector3,
	result: union
	{
		Ray_Results, Sweep_Results, Overlap_Results, Collision_Results, e.Shape_Distance_Result, e.Shape_Correction_Result, e.Overlap_State,
	},
}

Scene :: struct
{
	world: e.World,
	query_shape,
	target: e.Shape_Handle,
	custom_sphere: support.Custom_Sphere,
}

Sample :: struct
{
	status: e.Status,
	count: int,
	distance: f32,
}

allow_static :: proc "contextless" (_: rawptr, collidable: e.Collidable_Reference) -> bool
{
	return e.collidable_mobility(collidable) == .Static;
}

world_description :: proc(workers: int) -> e.World_Description
{
	desc: e.World_Description = e.world_description_default();
	desc.gravity = {};
	desc.threading.worker_count = i32(workers);
	return desc;
}

scene_init :: proc(scene: ^Scene, workers, route: int) -> e.Status
{
	desc: e.World_Description = world_description(workers);
	if failure: e.Status = e.world_init(&scene.world, desc); failure != .Ok
	{
		return failure;
	}
	status: e.Status;
	scene.query_shape, status = e.shape_add(&scene.world, e.sphere(0.5));
	if status != .Ok
	{
		return status;
	}
	switch route
	{
		case 0:
		scene.target, status = e.shape_add(&scene.world, e.sphere(1));
		case 1:
		child: e.Shape_Handle;
		child_status: e.Status;
		child, child_status = e.shape_add(&scene.world, e.sphere(0.7));
		if child_status != .Ok
		{
			return child_status;
		}
		children: [2]e.Compound_Child = [2]e.Compound_Child{
			e.compound_child(child, e.pose({-0.4, 0, 0})),
			e.compound_child(child, e.pose({0.4, 0, 0})),
		};
		scene.target, status = e.shape_import_compound(&scene.world, children[:]);
		case 2:
		ctx: cooking.Cooking_Context;
		defer cooking.cooking_context_destroy(&ctx);
		if failure: e.Status = cooking.cooking_context_init(&ctx); failure != .Ok
		{
			return failure;
		}
		triangles: [2]e.Triangle = [2]e.Triangle{
			e.triangle({-2, 0, -2}, {2, 0, -2}, {2, 0, 2}),
			e.triangle({-2, 0, -2}, {2, 0, 2}, {-2, 0, 2}),
		};
		mesh: cooking.Cooked_Mesh;
		defer cooking.cooked_mesh_destroy(&mesh);
		cook_status: e.Status;
		mesh, cook_status = cooking.cook_mesh(&ctx, triangles[:]);
		if cook_status != .Ok
		{
			return cook_status;
		}
		scene.target, status = cooking.cooked_mesh_import(&scene.world, &mesh);
		if failure: e.Status = cooking.cooked_mesh_destroy(&mesh); failure != .Ok
		{
			return failure;
		}
		if failure: e.Status = cooking.cooking_context_destroy(&ctx); failure != .Ok
		{
			return failure;
		}
		case 3:
		id: e.Shape_Type_ID;
		id, status = support.extension_register_sphere(&scene.world);
		if status != .Ok
		{
			return status;
		}
		scene.custom_sphere = {radius=1};
		scene.target, status = e.custom_shape_add(&scene.world, id, &scene.custom_sphere);
	}
	if status != .Ok
	{
		return status;
	}
	_, status = e.static_add(&scene.world, e.static_body(scene.target, e.pose()), .None);
	if status != .Ok
	{
		return status;
	}
	return .Ok;
}

// store every status/result for untimed validation. do not print or inspect world
// statistics in the measured loops. public entry points own all query execution
run_operation :: proc(scene: ^Scene, operation: int, samples: []Sample, $retention: Retention, observations: []Observation = nil)
{
	filter: e.Query_Filter = e.Query_Filter{allow=allow_static};
	for &sample, index in samples
	{
		x: f32 = f32(index % 17 - 8) * 0.005;
		ray: e.Ray = e.ray({x, 4, 0}, {0, -1, 0}, 8);
		when retention == .Keep
		{
			observations[index].origin = {x, 4 if operation <= 4 else 0.25, 0};
		}
		switch operation
		{
			case 0, 1:
			selected: e.Query_Filter = e.Query_Filter{};
			if operation == 1
			{
				selected = filter;
			}
			hit: e.Ray_Hit;
			status: e.Status;
			hit, status = e.ray_cast_closest(&scene.world, ray, selected);
			sample = {status=status, count=int(status == .Ok), distance=hit.t};
			when retention == .Keep
			{
				result: Ray_Results;
				result.hits[0] = hit;
				observations[index].result = result;
			}
			case 2:
			hits: [8]e.Ray_Hit;
			count: int;
			status: e.Status;
			count, status = e.ray_cast_all(&scene.world, ray, hits[:], filter);
			sample = {status=status, count=count, distance=hits[0].t};
			when retention == .Keep
			{
				observations[index].result = Ray_Results{hits};
			}
			case 3:
			hit: e.Sweep_Hit;
			status: e.Status;
			hit, status = e.sweep_closest(&scene.world, scene.query_shape, e.pose({x, 4, 0}), e.velocity({0, -5, 0}), 1);
			sample = {status=status, count=int(status == .Ok), distance=hit.sweep.t1};
			when retention == .Keep
			{
				result: Sweep_Results;
				result.hits[0] = hit;
				observations[index].result = result;
			}
			case 4:
			hits: [8]e.Sweep_Hit;
			count: int;
			status: e.Status;
			count, status = e.sweep_all(&scene.world, scene.query_shape, e.pose({x, 4, 0}), e.velocity({0, -5, 0}), 1, hits[:], filter);
			sample = {status=status, count=count, distance=hits[0].sweep.t1};
			when retention == .Keep
			{
				observations[index].result = Sweep_Results{hits};
			}
			case 5:
			hits: [8]e.Overlap_Hit;
			count: int;
			status: e.Status;
			count, status = e.overlap_all(&scene.world, scene.query_shape, e.pose({x, 0.25, 0}), hits[:], filter);
			sample = {status=status, count=int(count)};
			when retention == .Keep
			{
				observations[index].result = Overlap_Results{hits};
			}
			case 6:
			manifold: e.Manifold_Result;
			status: e.Status;
			manifold, status = e.collision_query(&scene.world, scene.query_shape, e.pose({x, 0.25, 0}), scene.target, e.pose());
			count: i32 = manifold.convex.count;
			if manifold.kind == .Nonconvex
			{
				count = manifold.nonconvex.count;
			}
			sample = {status=status, count=int(count)};
			when retention == .Keep
			{
				result: Collision_Results;
				result.manifolds[0] = manifold;
				observations[index].result = result;
			}
			case 7:
			queries: [2]e.Collision_Query = [2]e.Collision_Query{
				{shape_a=scene.query_shape, shape_b=scene.target, pose_a=e.pose({x, 0.25, 0}), pose_b=e.pose()},
				{shape_a=scene.query_shape, shape_b=scene.target, pose_a=e.pose({-x, 0.25, 0}), pose_b=e.pose()},
			};
			results: [2]e.Collision_Query_Result;
			status: e.Status = e.collision_query_batch(&scene.world, queries[:], results[:]);
			sample = {status=status, count=int(results[0].hit) + int(results[1].hit)};
			when retention == .Keep
			{
				observations[index].result = Collision_Results{{results[0].manifold, results[1].manifold}};
			}
		}
	}
}

validate :: proc(samples: []Sample, operation, route: int) -> (u64, f64, e.Status)
{
	hits: u64;
	checksum: f64;
	for sample in samples
	{
		if sample.status != .Ok
		{
			fmt.eprintfln("query_failure route=%s operation=%s status=%v", ROUTES[route], OPERATIONS[operation], sample.status);
			return 0, 0, .Invalid_Argument;
		}
		if sample.count <= 0 || (operation == 7 && sample.count != 2) ||
		sample.distance != sample.distance || sample.distance < 0 || sample.distance > 8
		{
			return 0, 0, .Invalid_Argument;
		}
		hits += u64(sample.count);
		checksum += f64(sample.distance);
	}
	return hits, checksum, .Ok;
}

// distance-query groups are measured separately, after the common queries. their
// total never changes physics_elapsed_ms or any pre-existing timed region
when support.BENCHMARK_COMPONENTS == "all"
{
	DISTANCE_OPERATIONS: [4]string = {"closest_point", "shape_distance", "depenetration", "overlap_any"};
	run_distance_operation :: proc(scene: ^Scene, operation: int, samples: []Sample, $retention: Retention, observations: []Observation = nil)
	{
		// deep smooth custom support needs a larger explicit EPA budget than the
		// API default. no retry or default-policy change occurs inside timing
		settings: e.Distance_Query_Settings = e.distance_query_settings_default();
		settings.maximum_iterations = 512;
		for &sample, index in samples
		{
			x: f32 = f32(index % 17 - 8) * 0.005;
			when retention == .Keep
			{
				observations[index].origin = {x, 4 if operation < 2 else 0.25, 0};
			}
			switch operation
			{
				case 0:
				result: e.Shape_Distance_Result;
				status: e.Status;
				result, status = e.shape_closest_point(&scene.world, {x, 4, 0}, scene.target, e.pose(), settings);
				sample = {status=status, count=int(result.geometry.state != .Unresolved), distance=result.geometry.distance};
				when retention == .Keep
				{
					observations[index].result = result;
				}
				case 1:
				result: e.Shape_Distance_Result;
				status: e.Status;
				result, status = e.shape_distance(&scene.world, scene.query_shape, e.pose({x, 4, 0}), scene.target, e.pose(), settings);
				sample = {status=status, count=int(result.geometry.state != .Unresolved), distance=result.geometry.distance};
				when retention == .Keep
				{
					observations[index].result = result;
				}
				case 2:
				result: e.Shape_Correction_Result;
				status: e.Status;
				result, status = e.shape_depenetrate(&scene.world, scene.query_shape, e.pose({x, 0.25, 0}), scene.target, e.pose(), settings);
				shift: e.Vector3 = result.translation;
				sample = {status=status, count=int(result.state != .Unresolved), distance=math.sqrt(shift.x*shift.x+shift.y*shift.y+shift.z*shift.z)};
				when retention == .Keep
				{
					observations[index].result = result;
				}
				case 3:
				state: e.Overlap_State;
				status: e.Status;
				state, status = e.overlap_any(&scene.world, scene.query_shape, e.pose({x, 0.25, 0}));
				sample = {status=status, count=int(state == .Intersecting)};
				when retention == .Keep
				{
					observations[index].result = state;
				}
			}
		}
	}
	validate_distance :: proc(samples: []Sample, operation, route: int) -> (u64, f64, e.Status)
	{
		count: u64;
		checksum: f64;
		for &sample in samples
		{
			if sample.status != .Ok || sample.count != 1 || !(sample.distance >= 0 && sample.distance <= 8)
			{
				fmt.eprintfln("distance_failure route=%s operation=%s status=%v count=%d distance=%v", ROUTES[route], DISTANCE_OPERATIONS[operation], sample.status, sample.count, sample.distance);
				return 0, 0, .Invalid_Argument;
			}
			count += u64(sample.count);
			checksum += f64(sample.distance);
		}
		return count, checksum, .Ok;
	}
}
