#+vet !unused-imports

package query_extensions

import "base:runtime"
import "core:fmt"
import "core:os"
import "core:strings"
import "core:time"
import e "entasis:entasis"
import cooking "entasis:entasis_cooking"
import util "entasis:entasis_utilities"
import support "../benchmark_support"

when util.PRECISION_MODE == .Double_Coordinates
{
	GLOBAL_RAY_INPUTS :: 1536;
	GLOBAL_RAY_PASSES :: 8;

	Global_Ray_Route :: enum u8
	{
		Global, Local,
	}

	Global_Ray_Geometry :: enum u8
	{
		Mesh, Box,
	}

	Global_Ray_Observation :: struct
	{
		status: e.Status,
		count: int,
		fraction: f64,
		location: e.Global_Position,
		normal: e.Vector3,
		child: i32,
		target: e.Collidable_Reference,
	}

	Global_Ray_Worker :: struct
	{
		query: e.Query_Context,
		stack: [1024]i32,
	}

	Global_Ray_Case :: struct
	{
		world: e.World,
		workers: []Global_Ray_Worker,
		segments: []e.World_Precision_Segment,
		observations: []Global_Ray_Observation,
		targets: [1024]e.Collidable_Reference,
		route: Global_Ray_Route,
		geometry: Global_Ray_Geometry,
		operation: string,
		count, passes: int,
	}

	global_ray_worker :: proc "contextless" (worker: int, dispatcher: ^util.Thread_Dispatcher_Boundary)
	{
		fixture: ^Global_Ray_Case = (^Global_Ray_Case)(dispatcher.unmanaged_context);
		owner: ^Global_Ray_Worker = &fixture.workers[worker];
		partition: int = GLOBAL_RAY_INPUTS/len(fixture.workers);
		begin: int = worker*partition;
		for _ in 0 ..< fixture.passes
		{
			for offset in 0 ..< fixture.count
			{
				index: int = begin+offset%partition;
				segment: e.World_Precision_Segment = fixture.segments[index];
				observation: Global_Ray_Observation;
				if fixture.route == .Global
				{
					hit: e.Global_Ray_Hit;
					switch fixture.operation
					{
					case "ray-any":
						state: e.Query_Hit_State;
						state, observation.status = e.ray_cast_global_any_with_context(&owner.query, segment, owner.stack[:]);
						observation.count = 1 if state == .Hit else 0;
					case "ray-closest":
						hit, observation.status = e.ray_cast_global_closest_with_context(&owner.query, segment, owner.stack[:]);
						observation.count = 1 if observation.status == .Ok else 0;
					case "ray-all":
						hits: [2]e.Global_Ray_Hit;
						observation.count, observation.status = e.ray_cast_global_all_with_context(&owner.query, segment, hits[:], owner.stack[:]);
						hit = hits[0];
					}
					observation.fraction, observation.location, observation.normal = hit.t, hit.location, hit.normal;
					observation.child, observation.target = hit.child_index, hit.collidable;
				}
				else
				{
					ray: e.Ray = e.ray({f32(segment.start.x), f32(segment.start.y), f32(segment.start.z)}, {0, -1, 0}, 8);
					hit: e.Ray_Hit;
					switch fixture.operation
					{
					case "ray-any":
						state: e.Query_Hit_State;
						state, observation.status = e.ray_cast_any_with_context(&owner.query, ray);
						observation.count = 1 if state == .Hit else 0;
					case "ray-closest":
						hit, observation.status = e.ray_cast_closest_with_context(&owner.query, ray);
						observation.count = 1 if observation.status == .Ok else 0;
					case "ray-all":
						hits: [2]e.Ray_Hit;
						observation.count, observation.status = e.ray_cast_all_with_context(&owner.query, ray, hits[:]);
						hit = hits[0];
					}
					observation.fraction = f64(hit.t)/8;
					observation.location = {f64(hit.location.x), f64(hit.location.y), f64(hit.location.z)};
					observation.normal, observation.child, observation.target = hit.normal, hit.child_index, hit.collidable;
				}
				fixture.observations[index] = observation;
			}
		}
	}

	global_ray_validate :: proc(fixture: ^Global_Ray_Case) -> e.Status
	{
		for observation, index in fixture.observations
		{
			expected_count: int = 1 if index%2 == 0 else 0;
			expected_status: e.Status = .Not_Found if expected_count == 0 && fixture.operation == "ray-closest" else .Ok;
			if observation.status != expected_status || observation.count != expected_count
			{
				fmt.eprintln("global ray independent hit/miss/cardinality oracle", index, observation);
				return .Invalid_Description;
			}
			if expected_count == 0 || fixture.operation == "ray-any"
			{
				continue;
			}
			segment: e.World_Precision_Segment = fixture.segments[index];
			expected_location: e.Global_Position = {segment.start.x, segment.start.y-4, segment.start.z};
			expected_child: i32 = 0 if fixture.geometry == .Box else i32(2*((index/2)%4096));
			if observation.fraction != 0.5 || observation.location != expected_location || observation.child != expected_child ||
				observation.normal != (e.Vector3{0, 1, 0}) || observation.target != fixture.targets[((index/2)*37)%1024]
			{
				fmt.eprintln("global ray independent plane/box/location/identity oracle", index, observation, expected_location, expected_child);
				return .Invalid_Description;
			}
		}
		return .Ok;
	}

	run_global_case :: proc(args: Query_Arguments) -> e.Status
	{
		fixture: Global_Ray_Case = {operation=args.query_operation};
		if strings.has_suffix(args.query_cell, "-local")
		{
			fixture.route = .Local;
		}
		if strings.has_prefix(args.query_cell, "box-")
		{
			fixture.geometry = .Box;
		}
		worker_error, segment_error, observation_error: runtime.Allocator_Error;
		fixture.workers, worker_error = make([]Global_Ray_Worker, args.workers);
		defer delete(fixture.workers);
		fixture.segments, segment_error = make([]e.World_Precision_Segment, GLOBAL_RAY_INPUTS);
		defer delete(fixture.segments);
		fixture.observations, observation_error = make([]Global_Ray_Observation, GLOBAL_RAY_INPUTS);
		defer delete(fixture.observations);
		if worker_error != nil || segment_error != nil || observation_error != nil
		{
			panic("global ray fixture storage");
		}
		description: e.World_Description = world_description(1);
		description.capacity.statics = 1024;
		support.extension_require(e.world_init(&fixture.world, description), "global ray world");
		defer e.world_destroy(&fixture.world);
		for &worker in fixture.workers
		{
			support.extension_require(e.query_context_init(&worker.query, &fixture.world), "global ray context");
		}
		defer
		{
			for &worker in fixture.workers
			{
				e.query_context_destroy(&worker.query);
			}
		}
		shape: e.Shape_Handle;
		status: e.Status;
		if fixture.geometry == .Box
		{
			shape, status = e.shape_add(&fixture.world, e.box(1, 1, 1));
			support.extension_require(status, "global ray box");
		}
		else
		{
			triangles: []e.Triangle;
			allocation_error: runtime.Allocator_Error;
			triangles, allocation_error = make([]e.Triangle, 8192);
			if allocation_error != nil
			{
				panic("global ray triangle storage");
			}
			defer delete(triangles);
			for z in 0 ..< 64
			{
				for x in 0 ..< 64
				{
					a: e.Vector3 = {f32(x), 0, f32(z)};
					b: e.Vector3 = {f32(x+1), 0, f32(z)};
					c: e.Vector3 = {f32(x+1), 0, f32(z+1)};
					d: e.Vector3 = {f32(x), 0, f32(z+1)};
					triangles[2*(z*64+x)] = e.triangle(a, b, c);
					triangles[2*(z*64+x)+1] = e.triangle(a, c, d);
				}
			}
			cooker: cooking.Cooking_Context;
			support.extension_require(cooking.cooking_context_init(&cooker), "global ray cooker");
			defer cooking.cooking_context_destroy(&cooker);
			mesh: cooking.Cooked_Mesh;
			mesh, status = cooking.cook_mesh(&cooker, triangles);
			support.extension_require(status, "global ray cook");
			defer cooking.cooked_mesh_destroy(&mesh);
			shape, status = cooking.cooked_mesh_import(&fixture.world, &mesh);
			support.extension_require(status, "global ray import");
		}
		offset: f64 = 0;
		if fixture.route == .Global
		{
			offset = 1e9 if strings.has_suffix(args.query_cell, "-far") else 1e3;
		}
		positions: [1024]e.Global_Position;
		descriptions: [1024]e.Static_Description;
		handles: [1024]e.Static_Handle;
		for &position, index in positions
		{
			position = {offset+f64(index%32)*128+0.125, offset+0.25+f64(index%5)/1024, -offset+f64(index/32)*128-0.375};
			descriptions[index] = e.static_body(shape, e.pose({}, {0, 1, 0, 0}));
		}
		written: int;
		written, status = e.static_add_batch_global(&fixture.world, descriptions[:], handles[:], positions[:], .None);
		support.extension_require(status, "global ray insertion");
		if written != len(handles)
		{
			panic("global ray insertion prefix");
		}
		for handle, index in handles
		{
			fixture.targets[index], status = e.static_collidable_reference(&fixture.world, handle);
			support.extension_require(status, "global ray target identity");
		}
		for &segment, index in fixture.segments
		{
			position: e.Global_Position = positions[((index/2)*37)%1024];
			x: f64 = -f64((index/2)%64)-0.75;
			z: f64 = -f64((index/2/64)%64)-0.25;
			y: f64 = 4;
			if fixture.geometry == .Box
			{
				x, z, y = 0, 0, 4.5;
			}
			if index%2 != 0
			{
				x = 4 if fixture.geometry == .Box else 32;
			}
			segment = {start={position.x+x, position.y+y, position.z+z}, end={position.x+x, position.y+y-8, position.z+z}};
		}
		dispatcher: util.Thread_Dispatcher;
		if util.thread_dispatcher_initialize(&dispatcher, args.workers) != .Ok
		{
			panic("global ray dispatcher");
		}
		defer util.thread_dispatcher_shutdown(&dispatcher);
		boundary: ^util.Thread_Dispatcher_Boundary = util.thread_dispatcher_boundary(&dispatcher);
		fixture.count, fixture.passes = 128, 1;
		if boundary.dispatch(boundary, global_ray_worker, args.workers, &fixture) != .Ok
		{
			return .Invalid_Description;
		}
		fixture.count = GLOBAL_RAY_INPUTS/args.workers;
		if boundary.dispatch(boundary, global_ray_worker, args.workers, &fixture) != .Ok
		{
			return .Invalid_Description;
		}
		support.extension_require(global_ray_validate(&fixture), "global ray untimed oracle");
		fixture.passes = GLOBAL_RAY_PASSES;
		start: time.Tick = time.tick_now();
		dispatch_status: util.Threading_Status = boundary.dispatch(boundary, global_ray_worker, args.workers, &fixture);
		elapsed: f64 = f64(time.tick_diff(start, time.tick_now()))/1e6;
		if dispatch_status != .Ok || elapsed <= 0
		{
			return .Invalid_Description;
		}
		support.extension_require(global_ray_validate(&fixture), "global ray final oracle");
		file: ^os.File;
		state: support.Extension_CSV_State;
		file, state = support.extension_csv_open(args.output);
		if state == .Empty
		{
			if fmt.fprintln(file, "worker_count,case_status,metric_status,physics_elapsed_ms,benchmark_components,precision,query_case,query_operation,query_cell,input_count,passes,query_count,warmup_queries,hit_count,checksum,oracle_queries") <= 0
			{
				panic("global ray CSV header");
			}
		}
		if fmt.fprintf(file, "%d,ok,ok,%.9f,all,mixed64,global,%s,%s,1536,8,12288,%d,6144,6144,1536\n", args.workers, elapsed, args.query_operation, args.query_cell, 128*args.workers) <= 0
		{
			panic("global ray CSV row");
		}
		support.extension_csv_close(file);
		return .Ok;
	}
}
