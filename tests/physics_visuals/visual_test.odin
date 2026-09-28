package physics_visuals

import observation "../../tools/physics_observation"
import "core:testing"
import "core:strings"
import "core:math"
import "core:mem"
import entasis "entasis:entasis"
import cooking "entasis:entasis_cooking"
import scene "../../tools/physics_scene"
import scenarios "../../tools/physics_scenarios"
import replay "../../tools/physics_replay"
import trace "../../benchmarks/spatial_query_trace"
import batch "../../benchmarks/spatial_query_batch"
import queries "../../benchmarks/query_extensions"
import support "../../benchmarks/benchmark_support"
import native "../../benchmarks/c_abi_overhead"
import lifecycle "../../benchmarks/world_lifecycle"

@(test)
visual_lifecycle_reinitializes_disposed_fixture :: proc(t: ^testing.T)
{
	owner: lifecycle.Owner;
	defer entasis.world_destroy(&owner.world);
	defer cooking.cooking_context_destroy(&owner.cooking);
	statuses: [2]entasis.Status;
	lifecycle.initialize(&owner, lifecycle.world_description(1), &statuses);
	for status in statuses
	{
		testing.expect_value(t, status, entasis.Status.Ok);
	}
	testing.expect_value(t, entasis.world_destroy(&owner.world), entasis.Status.Ok);
	testing.expect_value(t, cooking.cooking_context_destroy(&owner.cooking), entasis.Status.Ok);
	testing.expect_value(t, owner.cooking.state, cooking.Cooking_Context_State.Disposed);
	owner.cooking = {};
	lifecycle.initialize(&owner, lifecycle.world_description(1), &statuses);
	for status in statuses
	{
		testing.expect_value(t, status, entasis.Status.Ok);
	}
	testing.expect(t, owner.world != nil);
	testing.expect_value(t, owner.cooking.state, cooking.Cooking_Context_State.Ready);
}

@(test)
visual_recipe_admits_only_case_settings :: proc(t: ^testing.T)
{
	r: scenarios.Recipe;
	status: scene.Status;
	valid: string = `{"scenario":"example/falling_box","settings":{},"input":"scripted","frames":61}`;
	r, status = scenarios.recipe_parse(transmute([]u8)valid);
	testing.expect_value(t, status, scene.Status.Ok);
	testing.expect_value(t, r.frames, 61);
	invalid: [6]string = {
		`{"scenario":"example/falling_box","settings":{"worker-count":2}}`,
		`{"scenario":"example/falling_box","scenario":"example/falling_box"}`,
		`{"scenario":"example/falling_box","unknown":1}`,
		`{"scenario":"example/absent"}`,
		`{"scenario":"example/falling_box","input":"live"}`,
		`{"scenario":"benchmark/container","settings":{"rows":3}}`,
	};
	for data in invalid
	{
		_, status = scenarios.recipe_parse(transmute([]u8)data);
		testing.expect_value(t, status, scene.Status.Invalid_Data);
	}
	for _, index in scenarios.CATALOG
	{
		r, status = scenarios.recipe_admit(scenarios.Scenario(index), nil, frames=3);
		testing.expect_value(t, status, scene.Status.Ok);
		bytes: []u8;
		bytes, status = scenarios.recipe_encode(r);
		defer delete(bytes);
		testing.expect_value(t, status, scene.Status.Ok);
		decoded: scenarios.Recipe;
		decoded, status = scenarios.recipe_parse(bytes);
		testing.expect_value(t, status, scene.Status.Ok);
		testing.expect_value(t, decoded, r);
	}

}

@(test)
visual_examples_complete_reset_and_release :: proc(t: ^testing.T)
{
	for entry, index in scenarios.CATALOG
	{
		for _ in 0 ..< 2
		{
			r: scenarios.Recipe;
			status: scene.Status;
			r, status = scenarios.recipe_admit(scenarios.Scenario(index), nil);
			testing.expect_value(t, status, scene.Status.Ok);
			s: scenarios.Session;
			defer scenarios.session_destroy(&s);
			status = scenarios.session_create(&s, r);
			testing.expect_value(t, status, scene.Status.Ok);
			if status != .Ok
			{
				return;
			}
			testing.expect_value(t, s.state, scenarios.Session_State.Paused);
			settings_buffer: [4096]u8;
			settings: string;
			availability: scene.Availability;
			settings, availability, status = scenarios.session_settings(&s, settings_buffer[:]);
			testing.expect_value(t, status, scene.Status.Ok);
			testing.expect_value(t, availability, scene.Availability.Available);
			testing.expect(t, strings.contains(settings, entry.id));
			_, _, status = scenarios.session_settings(&s, settings_buffer[:8]);
			testing.expect_value(t, status, scene.Status.Invalid_Data);
			previous_cycle: u32;
			previous_tick: u64;
			for _ in 0 ..< s.extent
			{
				status = scenarios.session_step(&s);
				testing.expect_value(t, status, scene.Status.Ok);
				if status != .Ok
				{
					return;
				}
				frame: ^scene.Frame = &s.observation.packet.frame;
				testing.expect(t, frame.cycle > previous_cycle || (frame.cycle == previous_cycle && frame.tick > previous_tick), entry.id);
				testing.expect_value(t, scene.frame_admit(frame, s.observation.packet.entities[:], s.observation.packet.geometries[:]), scene.Status.Ok);
				previous_cycle, previous_tick = frame.cycle, frame.tick;
			}
			testing.expect_value(t, s.state, scenarios.Session_State.Completed);
			testing.expect_value(t, s.tick, s.extent);
			testing.expect_value(t, scene.frame_admit(&s.observation.packet.frame, s.observation.packet.entities[:], s.observation.packet.geometries[:]), scene.Status.Ok);
		}
	}
}

@(test)
visual_input_applies_once_at_tick_boundary :: proc(t: ^testing.T)
{
	r: scenarios.Recipe;
	status: scene.Status;
	r, status = scenarios.recipe_admit(.Character_Controller, nil, input=.Live);
	testing.expect_value(t, status, scene.Status.Ok);
	s: scenarios.Session;
	defer scenarios.session_destroy(&s);
	testing.expect_value(t, scenarios.session_create(&s, r), scene.Status.Ok);
	initial: scene.Pose = s.observation.packet.frame.poses[0];
	testing.expect_value(t, s.tick, 0);
	testing.expect_value(t, scenarios.session_step(&s, {mode=.Live, x=2}), scene.Status.Invalid_Data);
	testing.expect_value(t, s.tick, 0);
	testing.expect_value(t, s.observation.packet.frame.poses[0], initial);
	testing.expect_value(t, scenarios.session_step(&s, {mode=.Live, y=1}), scene.Status.Ok);
	testing.expect_value(t, s.tick, 1);
	testing.expect_value(t, s.observation.packet.frame.input, scene.Input{mode=.Live, y=1});
	testing.expect(t, abs(s.observation.packet.frame.poses[0].position.z-initial.position.z-0.1) < 0.00001);
	paused: scene.Pose = s.observation.packet.frame.poses[0];
	testing.expect_value(t, scenarios.session_observe(&s), scene.Status.Ok);
	testing.expect_value(t, scenarios.session_observe(&s), scene.Status.Ok);
	testing.expect_value(t, s.tick, 1);
	testing.expect_value(t, s.observation.packet.frame.poses[0], paused);
	testing.expect_value(t, scenarios.session_step(&s, {mode=.Live}), scene.Status.Ok);
	testing.expect_value(t, s.tick, 2);
	testing.expect_value(t, s.observation.packet.frame.poses[0].position, paused.position);
	testing.expect_value(t, scenarios.session_step(&s), scene.Status.Ok);
	testing.expect_value(t, s.observation.packet.frame.input, scene.Input{x=1});
	testing.expect(t, abs(s.observation.packet.frame.poses[0].position.x-paused.position.x-0.1) < 0.00001);
	before_diagonal: scene.Pose = s.observation.packet.frame.poses[0];
	testing.expect_value(t, scenarios.session_step(&s, {mode=.Live, x=1, y=1}), scene.Status.Ok);
	horizontal_paths: int;
	for overlay in s.observation.packet.frame.overlays
	{
		if overlay.kind == .Query_Path && overlay.b.y == overlay.a.y
		{
			horizontal_paths += 1;
			testing.expect_value(t, overlay.value, f32(0.1));
			testing.expect(t, abs(overlay.b.x-overlay.a.x-0.07071068) < 0.00001);
			testing.expect(t, abs(overlay.b.z-overlay.a.z-0.07071068) < 0.00001);
			testing.expect_value(t, overlay.a, before_diagonal.position);
		}
	}
	testing.expect_value(t, horizontal_paths, 1);
	after_diagonal: scene.Pose = s.observation.packet.frame.poses[0];
	testing.expect(t, abs(after_diagonal.position.x-before_diagonal.position.x-0.07071068) < 0.00001);
	testing.expect(t, abs(after_diagonal.position.z-before_diagonal.position.z-0.07071068) < 0.00001);
}

@(test)
visual_native_mutation_observations :: proc(t: ^testing.T)
{
	owner: native.Owner;
	testing.expect_value(t, entasis.world_init(&owner.world, native.world_description(2, 1, 0)), entasis.Status.Ok);
	defer native.case_destroy(&owner);
	owner.body_descriptions = make([]entasis.Body_Description, 2);
	owner.bodies = make([]entasis.Body_Handle, 2);
	owner.static_descriptions = make([]entasis.Static_Description, 1);
	owner.statics = make([]entasis.Static_Handle, 1);
	owner.a = make([]entasis.Body_Description, 2);
	owner.b = make([]entasis.Body_Description, 2);
	shape, status := entasis.shape_add(&owner.world, entasis.sphere(0.25));
	testing.expect_value(t, status, entasis.Status.Ok);
	owner.static_descriptions[0] = entasis.static_body(shape, entasis.pose({0, -10, 0}));
	for &description, index in owner.body_descriptions
	{
		description = entasis.body_shapeless(native.shapeless_inertia(), entasis.pose({f32(index), 0, 0}), {}, entasis.body_activity(-1, 255));
		owner.a[index], owner.b[index] = description, description;
		owner.a[index].pose.position.y, owner.b[index].pose.position.y = 1, 2;
	}
	bodies, statics, body_status, static_status := native.create_batches(&owner);
	testing.expect_value(t, bodies, 2);
	testing.expect_value(t, statics, 1);
	testing.expect_value(t, body_status, entasis.Status.Ok);
	testing.expect_value(t, static_status, entasis.Status.Ok);
	for iteration in 0 ..< 2
	{
		applied, mutation_status := native.mutate(&owner, iteration);
		testing.expect_value(t, applied, 2);
		testing.expect_value(t, mutation_status, entasis.Status.Ok);
		checksum, checksum_status := native.case_checksum(&owner, .Mutate);
		testing.expect_value(t, checksum_status, entasis.Status.Ok);
		testing.expect_value(t, checksum, f64(iteration+1));
	}
}

@(test)
visual_query_extension_observations :: proc(t: ^testing.T)
{
	for route in 0 ..< len(queries.ROUTES)
	{
		fixture: queries.Scene;
		testing.expect_value(t, queries.scene_init(&fixture, 1, route), entasis.Status.Ok);
		defer entasis.world_destroy(&fixture.world);
		samples: [1]queries.Sample;
		observations: [1]queries.Observation;
		for operation in 0 ..< len(queries.OPERATIONS)
		{
			queries.run_operation(&fixture, operation, samples[:], .Keep, observations[:]);
			_, _, status := queries.validate(samples[:], operation, route);
			testing.expect_value(t, status, entasis.Status.Ok);
			retained: queries.Sample = samples[0];
			origin: entasis.Vector3 = observations[0].origin;
			queries.run_operation(&fixture, operation, samples[:], .Discard);
			testing.expect_value(t, samples[0], retained);
			testing.expect_value(t, observations[0].origin, origin);

		}
		when support.BENCHMARK_COMPONENTS == "all"
		{
			testing.expect_value(t, entasis.distance_query_reserve(&fixture.world, {512, 1024, 1536}), entasis.Status.Ok);
			for operation in 0 ..< len(queries.DISTANCE_OPERATIONS)
			{
				queries.run_distance_operation(&fixture, operation, samples[:], .Keep, observations[:]);
				_, _, status := queries.validate_distance(samples[:], operation, route);
				testing.expect_value(t, status, entasis.Status.Ok);
				retained: queries.Sample = samples[0];
				queries.run_distance_operation(&fixture, operation, samples[:], .Discard);
				testing.expect_value(t, samples[0], retained);

			}
		}
	}
}

@(test)
visual_query_validation_observations :: proc(t: ^testing.T)
{
	owner: trace.Benchmark_Owner;
	testing.expect_value(t, entasis.world_init(&owner.world, entasis.world_description_default()), entasis.Status.Ok);
	owner.state, owner.worker_count = .Ready, 1;
	defer trace.benchmark_owner_destroy(&owner);
	shape, status := entasis.shape_add(&owner.world, entasis.box(1, 1, 1));
	testing.expect_value(t, status, entasis.Status.Ok);
	_, add_status := entasis.static_add(&owner.world, entasis.static_body(shape, entasis.pose({})), .None);
	testing.expect_value(t, add_status, entasis.Status.Ok);
	owner.sphere_shape, status = entasis.shape_add(&owner.world, entasis.sphere(trace.SPHERE_CAST_RADIUS));
	testing.expect_value(t, status, entasis.Status.Ok);
	owner.simulation, status = entasis.world_borrow_simulation(&owner.world);
	testing.expect_value(t, status, entasis.Status.Ok);
	owner.owner_pool, status = entasis.world_borrow_pool(&owner.world);
	testing.expect_value(t, status, entasis.Status.Ok);
	owner.ray_inputs = make([]trace.Ray_Input, 2);
	owner.sphere_cast_inputs = make([]trace.Sphere_Cast_Input, 2);
	owner.overlap_inputs = make([]trace.Overlap_Input, 2);
	for index in 0 ..< 2
	{
		x: f32 = f32(index)*4;
		owner.ray_inputs[index] = {ray={origin={x, 3, 0}, direction={0, -1, 0}, maximum_t=6}};
		owner.sphere_cast_inputs[index] = {pose=entasis.pose({x, 3, 0}), velocity=entasis.velocity({0, -1, 0})};
		owner.overlap_inputs[index] = {bounds={min={x-0.25, -0.25, -0.25}, max={x+0.25, 0.25, 0.25}}};
	}
	testing.expect_value(t, trace.observations_create(&owner, {.Ray, .Sphere_Cast, .Overlap}), trace.Case_Status.Ok);
	for family in trace.Query_Family
	{
		result: trace.Family_Result = trace.family_execute(&owner, family, .Validate);
		testing.expect_value(t, result.status, entasis.Status.Ok);
		testing.expect_value(t, result.hit_count, u64(1));
	}
	testing.expect_value(t, owner.observations.rays[0].state, trace.Observation_State.Hit);
	testing.expect_value(t, owner.observations.rays[1].state, trace.Observation_State.Miss);
	testing.expect_value(t, owner.observations.rays[0].hit.t, f32(2.5));
	testing.expect_value(t, owner.observations.sweeps[0].state, trace.Observation_State.Hit);
	testing.expect_value(t, owner.observations.overlaps[0].state, trace.Observation_State.Hit);
	retained: trace.Ray_Observation = owner.observations.rays[0];
	worker: batch.Worker;
	job: batch.Job = {owner=&owner, workers=([^]batch.Worker)(&worker)[:1], mode=.Validate};
	batch.execute_lane(&job, 0, owner.owner_pool);
	testing.expect_value(t, worker.status, entasis.Status.Ok);
	testing.expect_value(t, worker.hits, u64(1));
	testing.expect_value(t, owner.observations.rays[0], retained);
	owner.ray_inputs[0].ray.origin.x = 10;
	timed: trace.Family_Result = trace.family_execute(&owner, .Ray, .Timed);
	testing.expect_value(t, timed.hit_count, u64(0));
	testing.expect_value(t, owner.observations.rays[0], retained);
	owner.ray_inputs[0].ray.origin.x = 0;

}

@(test)
visual_operations_preserve_queries_and_events :: proc(t: ^testing.T)
{
	ids: [4]scenarios.Scenario = {.Direct_Collision_Queries, .Overlap_And_Events, .Impulses_Solver_Contacts, .Triggers};
	for id in ids
	{
		r: scenarios.Recipe;
		status: scene.Status;
		r, status = scenarios.recipe_admit(id, nil);
		testing.expect_value(t, status, scene.Status.Ok);
		s: scenarios.Session;
		defer scenarios.session_destroy(&s);
		testing.expect_value(t, scenarios.session_create(&s, r), scene.Status.Ok);
		if id == .Triggers
		{
			testing.expect(t, .Sensor in s.observation.packet.frame.states[1]);
		}
		enters, exits, parts, impulses, misses, hits: int;
		for step in 0 ..< s.extent
		{
			testing.expect_value(t, scenarios.session_step(&s), scene.Status.Ok);
			frame: ^scene.Frame = &s.observation.packet.frame;
			observations: [64]scene.Overlay;
			testing.expect(t, len(frame.overlays) <= len(observations));
			count: int = copy(observations[:], frame.overlays[:]);
			for overlay in frame.overlays
			{
				#partial switch overlay.kind
				{
				case .Enter: enters += 1;
				case .Exit: exits += 1;
				case .Contact_Impulse:
					impulses += 1;
					testing.expect(t, overlay.value > 0);
				case .Ray_Miss: misses += 1;
				case .Ray_Hit: hits += 1;
				case .Part_State:
					parts += 1;
					testing.expect_value(t, overlay.part_a, i32(1));
					testing.expect_value(t, overlay.value, f32(1));
				}
			}
			testing.expect_value(t, scenarios.session_observe(&s), scene.Status.Ok);
			testing.expect_value(t, scenarios.session_observe(&s), scene.Status.Ok);
			testing.expect_value(t, s.tick, step+1);
			testing.expect_value(t, len(frame.overlays), count);
			for overlay, index in frame.overlays
			{
				testing.expect_value(t, overlay, observations[index]);
			}
		}
		#partial switch id
		{
		case .Direct_Collision_Queries:
			testing.expect_value(t, misses, 1);
			testing.expect_value(t, hits, 1);
			testing.expect_value(t, s.axis, scene.Time_Axis.Operation);
		case .Overlap_And_Events:
			testing.expect_value(t, enters, 1);
			testing.expect_value(t, exits, 1);
		case .Impulses_Solver_Contacts:
			testing.expect_value(t, enters, 1);
			testing.expect_value(t, impulses, 1);
		case .Triggers:
			testing.expect_value(t, enters, 4);
			testing.expect_value(t, exits, 1);
			testing.expect_value(t, parts, 2);
			testing.expect_value(t, s.observation.packet.frame.cycle, u32(1));
		case: unreachable();
		}
	}
}

@(test)
visual_geometry_matches_registered_shapes :: proc(t: ^testing.T)
{
	world: entasis.World;
	testing.expect_value(t, entasis.world_init(&world, entasis.world_description_default()), entasis.Status.Ok);
	defer entasis.world_destroy(&world);
	shapes: [5]entasis.Shape_Handle;
	status: entasis.Status;
	shapes[0], status = entasis.shape_add(&world, entasis.box(2, 4, 6));
	testing.expect_value(t, status, entasis.Status.Ok);
	shapes[1], status = entasis.shape_add(&world, entasis.sphere(0.75));
	testing.expect_value(t, status, entasis.Status.Ok);
	shapes[2], status = entasis.shape_add(&world, entasis.capsule(0.5, 3));
	testing.expect_value(t, status, entasis.Status.Ok);
	shapes[3], status = entasis.shape_add(&world, entasis.cylinder(0.6, 2));
	testing.expect_value(t, status, entasis.Status.Ok);
	shapes[4], status = entasis.shape_add(&world, entasis.triangle({0, 0, 0}, {2, 0, 0}, {0, 3, 0}));
	testing.expect_value(t, status, entasis.Status.Ok);
	o: observation.Observation = {budget=scene.DEFAULT_BUDGET};
	defer observation.observation_destroy(&o);
	for shape in shapes
	{
		testing.expect_value(t, observation.observation_add_shape(&o, &world, shape), scene.Status.Ok);
	}
	testing.expect_value(t, observation.observation_read(&o, &world), scene.Status.Ok);
	testing.expect_value(t, len(o.packet.geometries), 5);
	testing.expect_value(t, o.packet.geometries[0].size, scene.Vec3{2, 4, 6});
	testing.expect_value(t, o.packet.geometries[1].size.x, f32(0.75));
	testing.expect_value(t, o.packet.geometries[2].size, scene.Vec3{0.5, 3, 0});
	testing.expect_value(t, o.packet.geometries[3].size, scene.Vec3{0.6, 2, 0});
	testing.expect_value(t, o.packet.geometries[4].vertices[2], scene.Vec3{0, 3, 0});
	for &geometry, index in o.packet.geometries
	{
		testing.expect_value(t, scene.geometry_admit(&geometry, index), scene.Status.Ok);
	}
	testing.expect_value(t, entasis.world_clear(&world), entasis.Status.Ok);
	testing.expect_value(t, o.packet.geometries[4].vertices[1], scene.Vec3{2, 0, 0});
	cases: [2]scenarios.Scenario = {.Convex_Hulls, .Compounds};
	for id in cases
	{
		r: scenarios.Recipe;
		visual_status: scene.Status;
		r, visual_status = scenarios.recipe_admit(id, nil);
		testing.expect_value(t, visual_status, scene.Status.Ok);
		s: scenarios.Session;
		defer scenarios.session_destroy(&s);
		visual_status = scenarios.session_create(&s, r);
		testing.expect_value(t, visual_status, scene.Status.Ok);
		if visual_status != .Ok
		{
			return;
		}
		testing.expect_value(t, scenarios.session_step(&s), scene.Status.Ok);
		p: ^scene.Scene_Packet = &s.observation.packet;
		if id == .Convex_Hulls
		{
			testing.expect_value(t, len(p.entities), 1);
			testing.expect_value(t, p.entities[0].kind, scene.Object_Kind.Shape);
			testing.expect_value(t, len(p.geometries[0].vertices), 8);
			testing.expect_value(t, len(p.geometries[0].indices), 36);
			testing.expect_value(t, p.frame.metrics[0].value, f64(0));
		}
		else
		{
			for entity in p.entities
			{
				g: ^scene.Geometry = &p.geometries[entity.geometry];
				testing.expect_value(t, g.kind, scene.Geometry_Kind.Compound);
				testing.expect_value(t, len(g.children), 3);
				testing.expect(t, math.abs(g.children[0].pose.position.x-(-1-5.0/6.0)) < 1e-5);
				testing.expect_value(t, g.children[0].geometry, g.children[2].geometry);
			}
		}
	}
}

@(test)
visual_transport_comparison_and_budget :: proc(t: ^testing.T)
{
	metadata: scene.Metadata = {settings="workers=1", settings_state=.Available, configuration="release", components="all"};
	other: scene.Metadata = metadata;
	testing.expect_value(t, scene.comparison_admit(metadata, other), scene.Status.Unsupported);
	metadata.scenario, other.scenario = "example/a", "example/b";
	testing.expect_value(t, scene.comparison_admit(metadata, other), scene.Status.Unsupported);
	other.scenario = metadata.scenario;
	testing.expect_value(t, scene.comparison_admit(metadata, other), scene.Status.Ok);

	testing.expect_value(t, scene.comparison_configuration(metadata, other), scene.Configuration_Comparison.Matching);
	other.settings = "workers=2";
	testing.expect_value(t, scene.comparison_configuration(metadata, other), scene.Configuration_Comparison.Different);
	other = metadata;
	other.configuration = "development";
	testing.expect_value(t, scene.comparison_configuration(metadata, other), scene.Configuration_Comparison.Different);
	other = metadata;
	other.components = "common";
	testing.expect_value(t, scene.comparison_configuration(metadata, other), scene.Configuration_Comparison.Different);
	other.settings_state = .Unavailable;
	testing.expect_value(t, scene.comparison_configuration(metadata, other), scene.Configuration_Comparison.Unavailable);
	testing.expect_value(t, scene.comparison_configuration(other, metadata), scene.Configuration_Comparison.Unavailable);
	r: scenarios.Recipe;
	admission: scene.Status;
	r, admission = scenarios.recipe_admit(.Convex_Hulls, nil);
	testing.expect_value(t, admission, scene.Status.Ok);
	session: scenarios.Session;
	defer scenarios.session_destroy(&session);
	failed_status: scene.Status;
	{
		context.allocator = mem.nil_allocator();
		failed_status = scenarios.session_create(&session, r);
	}
	testing.expect_value(t, failed_status, scene.Status.Out_Of_Memory);
	testing.expect(t, session.owner == nil);
	testing.expect_value(t, scenarios.session_create(&session, r, 1), scene.Status.Budget_Exceeded);
	testing.expect(t, session.required_bytes > 1);
	testing.expect_value(t, scenarios.session_create(&session, r, scenarios.session_bytes(&session)+1), scene.Status.Budget_Exceeded);
	testing.expect_value(t, scenarios.session_create(&session, r, scene.DEFAULT_BUDGET), scene.Status.Ok);
	testing.expect_value(t, scenarios.session_step(&session), scene.Status.Ok);
	testing.expect_value(t, scene.frame_admit(&session.observation.packet.frame, session.observation.packet.entities[:], session.observation.packet.geometries[:]), scene.Status.Ok);
	testing.expect_value(t, session.observation.pending_bytes, u64(0));
	testing.expect(t, scenarios.session_bytes(&session) <= scene.DEFAULT_BUDGET);
	a, b: replay.Reader;
	a.metadata = {scenario=strings.clone("example/test"), axis=.Physics, timestep=1.0/60};
	b.metadata = a.metadata;
	b.metadata.scenario = strings.clone(a.metadata.scenario);
	a.index = make([]replay.Index_Entry, 5);
	b.index = make([]replay.Index_Entry, 4);
	defer replay.reader_close(&a);
	defer replay.reader_close(&b);
	for &entry, index in a.index
	{
		entry.tick = u64(index);
		entry.phase = .Active;
	}
	for &entry, index in b.index
	{
		entry.tick = u64(index+2);
		entry.phase = .Active;
	}
	b.index[1].phase = .Warmup;
	pairs: [][2]int;
	status: scene.Status;
	pairs, status = replay.comparison_index(&a, &b, 1024);
	defer delete(pairs);
	testing.expect_value(t, status, scene.Status.Ok);
	testing.expect_value(t, len(pairs), 2);
	testing.expect_value(t, pairs[0], [2]int{2, 0});
	testing.expect_value(t, pairs[1], [2]int{4, 2});
	_, status = replay.comparison_index(&a, &b, 1);
	testing.expect_value(t, status, scene.Status.Budget_Exceeded);
	b.metadata.timestep = 1.0/30;
	_, status = replay.comparison_index(&a, &b, 1024);
	testing.expect_value(t, status, scene.Status.Unsupported);
	cursor: scene.Cursor = {count=5, speed=3, playback=.Running};
	testing.expect_value(t, scene.cursor_advance(&cursor, 1, 1.0/60, 2), 2);
	testing.expect_value(t, cursor.index, 2);
	testing.expect(t, cursor.accumulator>0.9);
	testing.expect_value(t, scene.cursor_seek(&cursor, 4), scene.Status.Ok);
	testing.expect_value(t, cursor.accumulator, f64(0));
}

@(test)
visual_identity_survives_topology_changes :: proc(t: ^testing.T)
{
	world: entasis.World;
	testing.expect_value(t, entasis.world_init(&world, entasis.world_description_default()), entasis.Status.Ok);
	defer entasis.world_destroy(&world);
	body: entasis.Body_Handle;
	status: entasis.Status;
	description: entasis.Body_Description = entasis.body_shapeless({inverse_mass=1}, entasis.pose(), {}, entasis.body_activity(-1, 255));
	body, status = entasis.body_add(&world, description);
	testing.expect_value(t, status, entasis.Status.Ok);
	o: observation.Observation = {budget=scene.DEFAULT_BUDGET};
	defer observation.observation_destroy(&o);
	id: u32;
	visual_status: scene.Status;
	id, visual_status = observation.observation_add(&o, &world, .Body, body.value);
	testing.expect_value(t, visual_status, scene.Status.Ok);
	testing.expect_value(t, id, u32(0));
	testing.expect_value(t, observation.observation_read(&o, &world), scene.Status.Ok);
	testing.expect_value(t, observation.observation_remove(&o, .Body, body.value), scene.Status.Ok);
	testing.expect_value(t, entasis.body_remove(&world, body), entasis.Status.Ok);
	replacement: entasis.Body_Handle;
	replacement, status = entasis.body_add(&world, description);
	testing.expect_value(t, status, entasis.Status.Ok);
	id, visual_status = observation.observation_add(&o, &world, .Body, replacement.value);
	testing.expect_value(t, visual_status, scene.Status.Ok);
	testing.expect_value(t, id, u32(1));
	testing.expect_value(t, observation.observation_read(&o, &world), scene.Status.Ok);
	testing.expect_value(t, len(o.packet.frame.ids), 1);
	testing.expect_value(t, o.packet.frame.ids[0], u32(1));
	testing.expect_value(t, scene.frame_admit(&o.packet.frame, o.packet.entities[:], o.packet.geometries[:]), scene.Status.Ok);
	observation.observation_clear(&o);
	testing.expect_value(t, entasis.world_clear(&world), entasis.Status.Ok);
	body, status = entasis.body_add(&world, description);
	testing.expect_value(t, status, entasis.Status.Ok);
	id, visual_status = observation.observation_add(&o, &world, .Body, body.value);
	testing.expect_value(t, visual_status, scene.Status.Ok);
	testing.expect_value(t, id, u32(2));
	testing.expect_value(t, observation.observation_read(&o, &world), scene.Status.Ok);
	testing.expect_value(t, o.packet.frame.ids[0], u32(2));
}
