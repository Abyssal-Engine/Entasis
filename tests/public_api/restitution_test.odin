package public_api_tests

import "base:runtime"
import "core:math"
import "core:mem"
import "core:simd"
import "core:sync"
import "core:testing"
import e "entasis:entasis"
import p "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Restitution_Test_Scene :: struct
{
	world: e.World,
	simulation: ^p.Simulation,
	shape: e.Shape_Handle,
	body: e.Body_Handle,
	ground: e.Static_Handle,
	body_reference, static_reference: e.Collidable_Reference,
}

restitution_test_scene :: proc (scene: ^Restitution_Test_Scene) -> e.Status
{
	description: e.World_Description = small_world_description();
	description.gravity = {};
	description.damping = {};
	status: e.Status = e.world_init(&scene.world, description);
	if status != .Ok
	{
		return status;
	}
	scene.simulation, status = e.world_borrow_simulation(&scene.world);
	scene.shape, status = e.shape_add(&scene.world, e.box(2, 2, 2));
	if status != .Ok
	{
		return status;
	}
	scene.ground, status = e.static_add(&scene.world, e.static_body(scene.shape, e.pose()), .None);
	if status != .Ok
	{
		return status;
	}
	inertia: e.Body_Inertia;
	inertia_status: e.Status;
	inertia, inertia_status = e.shape_inertia(e.box(2, 2, 2), 1);
	if inertia_status != .Ok
	{
		return inertia_status;
	}
	scene.body, status = e.body_add(&scene.world, e.body_dynamic(
			scene.shape, inertia, e.pose({0, 1.9, 0}), {}, e.body_activity(-1, 255),
	));
	if status != .Ok
	{
		return status;
	}
	scene.body_reference, status = p.collidable_reference_body(.Dynamic, scene.body);
	scene.static_reference, status = p.collidable_reference_static(scene.ground);
	return status;
}

@(test)
restitution_settings_boundary_and_default_combination :: proc (t: ^testing.T)
{
	configuration: p.Restitution_Configuration = p.restitution_configuration_default();
	testing.expect_value(t, configuration.fallback, p.Restitution_Settings{threshold=1});
	for value in ([5]p.Restitution_Settings{{-0.1, 1}, {1.1, 1}, {0, -1}, {math.nan_f32(), 0}, {0, math.inf_f32(1)}})
	{
		testing.expect_value(t, p.restitution_settings_validate(value), p.Physics_Status.Invalid_Argument);
	}
	for value in ([4]p.Restitution_Settings{{0, 0}, {0.5, 1}, {1, 0}, {1, 100}})
	{
		testing.expect_value(t, p.restitution_settings_validate(value), p.Physics_Status.Ok);
	}
	a: p.Restitution_Settings = {0.25, 2};
	b: p.Restitution_Settings = {0.75, 1};
	testing.expect_value(t, p.restitution_combine_default(a, b), p.Restitution_Settings{0.75, 2});
	testing.expect_value(t, p.restitution_combine_default(a, b), p.restitution_combine_default(b, a));
}

@(test)
restitution_storage_sparse_settings_reuse_and_lifetimes :: proc (t: ^testing.T)
{
	scene: Restitution_Test_Scene;
	if !testing.expect_value(t, restitution_test_scene(&scene), e.Status.Ok)
	{
		return;
	}
	defer e.world_destroy(&scene.world);
	testing.expect_value(t, scene.simulation.restitution, (^p.Restitution_Storage)(nil));
	configuration: p.Restitution_Configuration = p.restitution_configuration_default();
	testing.expect_value(t, p.restitution_storage_initialize(scene.simulation, configuration, {}, .Legacy), e.Status.Ok);
	storage: ^p.Restitution_Storage = scene.simulation.restitution;
	configuration.fallback = {1, 10};
	testing.expect_value(t, storage.configuration.fallback, p.Restitution_Settings{threshold=1});
	testing.expect_value(t, p.restitution_storage_set(storage, scene.body_reference, {0.5, 2}), e.Status.Ok);
	testing.expect_value(t, p.restitution_storage_set(storage, scene.static_reference, {0.75, 3}), e.Status.Ok);
	testing.expect_value(t, storage.settings.count, 2);
	testing.expect_value(t, p.restitution_storage_lookup(storage, scene.body_reference), p.Restitution_Settings{0.5, 2});
	testing.expect_value(t, p.restitution_storage_lookup(storage, scene.static_reference), p.Restitution_Settings{0.75, 3});
	kinematic_reference: p.Collidable_Reference;
	kinematic_reference, _ = p.collidable_reference_body(.Kinematic, scene.body);
	testing.expect_value(t, p.restitution_key(kinematic_reference), p.restitution_key(scene.body_reference));
	keys: [^]u64 = storage.settings.keys.memory;
	pairs: [^]p.Restitution_Pair_State = storage.pairs[0].values.memory;
	testing.expect_value(t, p.restitution_storage_reserve(storage, 1, 1), e.Status.Ok);
	testing.expect_value(t, storage.settings.keys.memory, keys);
	testing.expect_value(t, storage.pairs[0].values.memory, pairs);
	testing.expect_value(t, p.restitution_storage_set(storage, scene.body_reference, {1.5, 0}), e.Status.Invalid_Argument);
	testing.expect_value(t, p.restitution_storage_lookup(storage, scene.body_reference), p.Restitution_Settings{0.5, 2});
	testing.expect_value(t, e.body_remove(&scene.world, scene.body), e.Status.Ok);
	testing.expect_value(t, storage.settings.count, 1);
	inertia: e.Body_Inertia;
	inertia, _ = e.shape_inertia(e.box(2, 2, 2), 1);
	replacement: e.Body_Handle;
	status: e.Status;
	replacement, status = e.body_add(&scene.world, e.body_dynamic(scene.shape, inertia, e.pose({0, 1.9, 0})));
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, replacement, scene.body);
	testing.expect_value(t, p.restitution_storage_lookup(storage, scene.body_reference), storage.configuration.fallback);
	testing.expect_value(t, e.static_remove(&scene.world, scene.ground, .None), e.Status.Ok);
	testing.expect_value(t, storage.settings.count, 0);
	testing.expect_value(t, p.restitution_storage_destroy(scene.simulation), e.Status.Ok);
	testing.expect_value(t, scene.simulation.restitution, (^p.Restitution_Storage)(nil));
	testing.expect_value(t, scene.simulation.bodies.restitution, (^p.Restitution_Storage)(nil));
	testing.expect_value(t, scene.simulation.statics.restitution, (^p.Restitution_Storage)(nil));
}

Restitution_Test_Combine_Mode :: enum u8
{
	Valid,
	Invalid,
}

Restitution_Test_Combine_Context :: struct
{
	storage: ^p.Restitution_Storage,
	mode: Restitution_Test_Combine_Mode,
	calls: int,
	reentry: e.Status,
}

restitution_test_combine :: proc "contextless" (
	user_context: rawptr, pair: p.Collidable_Pair, a, b: p.Restitution_Settings,
) -> p.Restitution_Settings
{
	context = runtime.default_context();
	state: ^Restitution_Test_Combine_Context = (^Restitution_Test_Combine_Context)(user_context);
	state.calls += 1;
	state.reentry = p.restitution_storage_set(state.storage, pair.a, {1, 0});
	if state.mode == .Invalid
	{
		return {coefficient=-1};
	}
	return p.restitution_combine_default(a, b);
}

@(test)
restitution_pair_preparation_commit_failure_and_feature_identity :: proc (t: ^testing.T)
{
	scene: Restitution_Test_Scene;
	if !testing.expect_value(t, restitution_test_scene(&scene), e.Status.Ok)
	{
		return;
	}
	defer e.world_destroy(&scene.world);
	// obtain actual accepted solid contacts. storage does not yet activate solve response
	if !testing.expect_value(t, e.world_step(&scene.world, 1.0/60), e.Status.Ok)
	{
		return;
	}
	state: Restitution_Test_Combine_Context;
	configuration: p.Restitution_Configuration = p.restitution_configuration_default();
	configuration.fallback = {0.5, 1};
	configuration.combine = restitution_test_combine;
	configuration.user_context = &state;
	if !testing.expect_value(t, p.restitution_storage_initialize(scene.simulation, configuration, {}, .Legacy), e.Status.Ok)
	{
		return;
	}
	storage: ^p.Restitution_Storage = scene.simulation.restitution;
	state.storage = storage;
	testing.expect_value(t, p.restitution_storage_prepare(storage), e.Status.Ok);
	testing.expect_value(t, state.reentry, e.Status.Invalid_Argument);
	testing.expect_value(t, scene.simulation.state, p.Simulation_State.Ready);
	pending: ^util.Quick_Dictionary(p.Collidable_Pair, p.Restitution_Pair_State) = &storage.pairs[1-storage.committed];
	if !testing.expect_value(t, pending.count, 1)
	{
		return;
	}
	testing.expect_value(t, pending.values.memory[0].contact_count, u8(4));
	pending.values.memory[0].phases[0] = .Spent;
	testing.expect_value(t, p.restitution_storage_complete(storage, .Ok), e.Status.Ok);
	committed: u8 = storage.committed;
	value: p.Restitution_Pair_State = storage.pairs[committed].values.memory[0];
	state.mode = .Invalid;
	testing.expect_value(t, p.restitution_storage_prepare(storage), e.Status.Invalid_Argument);
	testing.expect_value(t, storage.committed, committed);
	testing.expect_value(t, storage.pairs[committed].values.memory[0], value);
	state.mode = .Valid;
	testing.expect_value(t, p.restitution_storage_prepare(storage), e.Status.Ok);
	testing.expect_value(t, storage.pairs[1-committed].values.memory[0].phases[0], p.Restitution_Impact_Phase.Spent);
	storage.pairs[1-committed].values.memory[0].phases[0] = .Armed;
	testing.expect_value(t, p.restitution_storage_complete(storage, .Capacity_Missing), e.Status.Ok);
	testing.expect_value(t, storage.committed, committed);
	testing.expect_value(t, storage.pairs[committed].values.memory[0], value);
	// feature ordering is not a contact identity. rebuild from a reordered cache
	cache: ^p.Constraint_Cache = &scene.simulation.narrow_phase.pair_cache.mapping.values.memory[0];
	cache.feature_ids[0], cache.feature_ids[3] = cache.feature_ids[3], cache.feature_ids[0];
	testing.expect_value(t, p.restitution_storage_prepare(storage), e.Status.Ok);
	testing.expect_value(t, storage.pairs[1-committed].values.memory[0].phases[3], p.Restitution_Impact_Phase.Spent);
	testing.expect_value(t, storage.pairs[1-committed].values.memory[0].phases[0], p.Restitution_Impact_Phase.Armed);
	testing.expect_value(t, p.restitution_storage_complete(storage, .Ok), e.Status.Ok);
	committed = storage.committed;
	// a settings edit invalidates incident impact state, not unrelated world data
	testing.expect_value(t, p.restitution_storage_set(storage, scene.body_reference, {0.8, 1}), e.Status.Ok);
	testing.expect_value(t, storage.pairs[committed].count, 0);
}

@(test)
restitution_storage_all_owned_failure_and_allocation_free_prepare :: proc (t: ^testing.T)
{
	// refuse each construction allocation persistently, including cleanup
	failures: int = 0;
	completed: p.Reference_State = .Missing;
	for offset in 1 ..= 48
	{
		tracker: allocation_test_tracker;
		world: e.World;
		description: e.World_Description = small_world_description();
		description.allocator = allocation_test_allocator(&tracker);
		if !testing.expect_value(t, e.world_init_with_allocation_scope(&world, description, .All_Owned), e.Status.Ok)
		{
			return;
		}
		simulation: ^p.Simulation;
		simulation, _ = e.world_borrow_simulation(&world);
		tracker.fail_from = tracker.requests + offset;
		status: e.Status = p.restitution_storage_initialize(simulation, p.restitution_configuration_default(), description.allocator, .All_Owned);
		if status == .Ok
		{
			storage: ^p.Restitution_Storage = simulation.restitution;
			tracker.fail_from = tracker.requests + 1;
			requests: int = tracker.requests;
			for _ in 0 ..< 8
			{
				testing.expect_value(t, p.restitution_storage_reserve(storage, 1, 1), e.Status.Ok);
				testing.expect_value(t, p.restitution_storage_prepare(storage), e.Status.Ok);
				testing.expect_value(t, p.restitution_storage_complete(storage, .Ok), e.Status.Ok);
			}
			testing.expect_value(t, tracker.requests, requests);
			testing.expect_value(t, e.world_destroy(&world), e.Status.Ok);
			allocation_test_empty(t, &tracker);
			completed = .Present;
			break;
		}
		failures += 1;
		testing.expect_value(t, status, e.Status.Capacity_Missing);
		testing.expect_value(t, simulation.restitution, (^p.Restitution_Storage)(nil));
		testing.expect_value(t, e.world_destroy(&world), e.Status.Ok);
		allocation_test_empty(t, &tracker);
	}
	testing.expect(t, failures > 0);
	testing.expect_value(t, completed, p.Reference_State.Present);
}

@(test)
restitution_history_survives_sleep_and_resets_on_clear :: proc (t: ^testing.T)
{
	scene: Restitution_Test_Scene;
	if !testing.expect_value(t, restitution_test_scene(&scene), e.Status.Ok)
	{
		return;
	}
	defer e.world_destroy(&scene.world);
	testing.expect_value(t, e.world_step(&scene.world, 1.0/60), e.Status.Ok);
	state: Restitution_Test_Combine_Context;
	configuration: p.Restitution_Configuration = p.restitution_configuration_default();
	configuration.fallback = {0.5, 1};
	configuration.combine = restitution_test_combine;
	configuration.user_context = &state;
	if !testing.expect_value(t, p.restitution_storage_initialize(scene.simulation, configuration, {}, .Legacy), e.Status.Ok)
	{
		return;
	}
	storage: ^p.Restitution_Storage = scene.simulation.restitution;
	state.storage = storage;
	testing.expect_value(t, p.restitution_storage_prepare(storage), e.Status.Ok);
	storage.pairs[1-storage.committed].values.memory[0].phases[0] = .Spent;
	testing.expect_value(t, p.restitution_storage_complete(storage, .Ok), e.Status.Ok);
	expected: p.Restitution_Pair_State = storage.pairs[storage.committed].values.memory[0];
	// the explicit sleep operation preserves a connected contact island
	sleeper: ^p.Island_Sleeper = &scene.simulation.sleeper;
	location: p.Body_Memory_Location;
	location, _ = p.bodies_resolve(&scene.simulation.bodies, scene.body);
	scene.simulation.bodies.sets.memory[0].activity.memory[location.index].sleep_candidate = .Candidate;
	testing.expect_value(t, p.island_scaffold_clear(&sleeper.scaffold), e.Status.Ok);
	body_count, constraint_count: int;
	can_sleep: p.Reference_State;
	build_status: p.Physics_Status;
	body_count, constraint_count, can_sleep, build_status = p.island_sleeper_build_island(sleeper, &sleeper.scaffold, scene.body);
	testing.expect_value(t, can_sleep, p.Reference_State.Present);
	testing.expect_value(t, constraint_count, 1);
	testing.expect_value(t, build_status, e.Status.Ok);
	testing.expect_value(t, p.island_sleeper_sleep_island(sleeper, &sleeper.scaffold, body_count, constraint_count), e.Status.Ok);
	calls: int = state.calls;
	testing.expect_value(t, p.restitution_storage_prepare(storage), e.Status.Ok);
	testing.expect_value(t, state.calls, calls);
	testing.expect_value(t, storage.pairs[1-storage.committed].count, 1);
	testing.expect_value(t, storage.pairs[1-storage.committed].values.memory[0], expected);
	testing.expect_value(t, p.restitution_storage_complete(storage, .Ok), e.Status.Ok);
	testing.expect_value(t, e.body_awaken(&scene.world, scene.body), e.Status.Ok);
	testing.expect_value(t, p.restitution_storage_prepare(storage), e.Status.Ok);
	testing.expect_value(t, storage.pairs[1-storage.committed].values.memory[0].phases[0], p.Restitution_Impact_Phase.Spent);
	testing.expect_value(t, p.restitution_storage_complete(storage, .Ok), e.Status.Ok);
	testing.expect_value(t, p.restitution_storage_set(storage, scene.static_reference, {0.8, 1}), e.Status.Ok);
	testing.expect_value(t, e.world_clear(&scene.world), e.Status.Ok);
	testing.expect_value(t, storage.settings.count, 0);
	testing.expect_value(t, storage.pairs[0].count, 0);
	testing.expect_value(t, storage.pairs[1].count, 0);
	testing.expect_value(t, storage.state, p.Restitution_Preparation_State.Idle);
	testing.expect_value(t, scene.simulation.restitution, storage);
}

@(test)
restitution_capacity_failure_keeps_committed_history :: proc (t: ^testing.T)
{
	scene: Restitution_Test_Scene;
	if !testing.expect_value(t, restitution_test_scene(&scene), e.Status.Ok)
	{
		return;
	}
	defer e.world_destroy(&scene.world);
	testing.expect_value(t, e.world_step(&scene.world, 1.0/60), e.Status.Ok);
	configuration: p.Restitution_Configuration = p.restitution_configuration_default();
	configuration.pair_capacity = 1;
	configuration.fallback = {0.5, 1};
	if !testing.expect_value(t, p.restitution_storage_initialize(scene.simulation, configuration, {}, .Legacy), e.Status.Ok)
	{
		return;
	}
	storage: ^p.Restitution_Storage = scene.simulation.restitution;
	testing.expect_value(t, p.restitution_storage_prepare(storage), e.Status.Ok);
	testing.expect_value(t, p.restitution_storage_complete(storage, .Ok), e.Status.Ok);
	committed: u8 = storage.committed;
	expected: p.Restitution_Pair_State = storage.pairs[committed].values.memory[0];
	// add one more independent accepted pair than the actual rounded table holds
	inertia: e.Body_Inertia;
	inertia, _ = e.shape_inertia(e.box(2, 2, 2), 1);
	count: int = int(storage.pairs[1-committed].keys.length);
	for index in 1 ..= count
	{
		status: e.Status;
		_, status = e.static_add(&scene.world, e.static_body(scene.shape, e.pose({f32(index)*6, 0, 0})), .None);
		testing.expect_value(t, status, e.Status.Ok);
		_, status = e.body_add(&scene.world, e.body_dynamic(scene.shape, inertia,
				e.pose({f32(index)*6, 1.9, 0}), {}, e.body_activity(-1, 255)));
		testing.expect_value(t, status, e.Status.Ok);
	}
	// preparation now belongs to world solving. failure publishes no new history
	testing.expect_value(t, e.world_step(&scene.world, 1.0/60), e.Status.Capacity_Missing);
	testing.expect_value(t, p.restitution_storage_prepare(storage), e.Status.Capacity_Missing);
	testing.expect_value(t, storage.committed, committed);
	testing.expect_value(t, storage.pairs[committed].values.memory[0], expected);
	testing.expect_value(t, storage.pairs[1-committed].count, 0);
	testing.expect_value(t, storage.state, p.Restitution_Preparation_State.Idle);
	testing.expect_value(t, p.restitution_storage_reserve(storage, 16, i32(count+1)), e.Status.Ok);
	testing.expect_value(t, p.restitution_storage_prepare(storage), e.Status.Ok);
	testing.expect_value(t, storage.pairs[1-committed].count, count+1);
	testing.expect_value(t, p.restitution_storage_complete(storage, .Ok), e.Status.Ok);
}

@(test)
restitution_failed_reserve_preserves_owned_settings :: proc (t: ^testing.T)
{
	tracker: allocation_test_tracker;
	world: e.World;
	description: e.World_Description = small_world_description();
	description.allocator = allocation_test_allocator(&tracker);
	if !testing.expect_value(t, e.world_init_with_allocation_scope(&world, description, .All_Owned), e.Status.Ok)
	{
		return;
	}
	simulation: ^p.Simulation;
	simulation, _ = e.world_borrow_simulation(&world);
	inertia: e.Body_Inertia = {inverse_mass=1, inverse_inertia_tensor={xx=1, yy=1, zz=1}};
	body: e.Body_Handle;
	status: e.Status;
	body, status = e.body_add(&world, e.body_shapeless(inertia, e.pose()));
	testing.expect_value(t, status, e.Status.Ok);
	reference: p.Collidable_Reference;
	reference, _ = p.collidable_reference_body(.Dynamic, body);
	testing.expect_value(t, p.restitution_storage_initialize(simulation, p.restitution_configuration_default(), description.allocator, .All_Owned), e.Status.Ok);
	storage: ^p.Restitution_Storage = simulation.restitution;
	testing.expect_value(t, p.restitution_storage_set(storage, reference, {0.6, 2}), e.Status.Ok);
	tracker.fail_from = tracker.requests + 1;
	testing.expect_value(t, p.restitution_storage_reserve(storage, 32768, 32768), e.Status.Capacity_Missing);
	testing.expect_value(t, storage.settings.count, 1);
	testing.expect_value(t, p.restitution_storage_lookup(storage, reference), p.Restitution_Settings{0.6, 2});
	testing.expect_value(t, p.restitution_storage_set(storage, reference, {0.7, 2}), e.Status.Ok);
	testing.expect_value(t, p.restitution_storage_prepare(storage), e.Status.Ok);
	testing.expect_value(t, p.restitution_storage_complete(storage, .Ok), e.Status.Ok);
	testing.expect_value(t, e.world_destroy(&world), e.Status.Ok);
	allocation_test_empty(t, &tracker);
}

@(test)
restitution_world_impacts_use_integrated_velocity_before_warmstart :: proc (t: ^testing.T)
{
	for workers in ([3]i32{1, 2, 4})
	{
		for iterations in ([2]i32{1, 4})
		{
			for coefficient in ([3]f32{0.25, 0.5, 1})
			{
				world: e.World;
				description: e.World_Description = small_world_description();
				description.gravity = {0, -6, 0};
				description.damping = {};
				description.solve.velocity_iterations = iterations;
				description.threading.worker_count = workers;
				if !testing.expect_value(t, e.world_init(&world, description), e.Status.Ok)
				{
					return;
				}
				simulation, _ := e.world_borrow_simulation(&world);
				shape, _ := e.shape_add(&world, e.sphere(1));
				inertia, _ := e.shape_inertia(e.sphere(1), 1);
				bodies: [17]e.Body_Handle;
				for &body, index in bodies
				{
					_, status := e.static_add(&world, e.static_body(shape, e.pose({f32(index)*5, 0, 0})), .None);
					testing.expect_value(t, status, e.Status.Ok);
					body, status = e.body_add(&world, e.body_dynamic(shape, inertia, e.pose({f32(index)*5, 2, 0}),
							e.velocity({0, -10, 0}), e.body_activity(-1, 255)));
					testing.expect_value(t, status, e.Status.Ok);
				}
				configuration: p.Restitution_Configuration = p.restitution_configuration_default();
				configuration.fallback = {coefficient, 1};
				if !testing.expect_value(t, p.restitution_storage_initialize(simulation, configuration, {}, .Legacy), e.Status.Ok)
				{
					_ = e.world_destroy(&world);
					return;
				}
				if testing.expect_value(t, e.world_step(&world, 1.0/60), e.Status.Ok)
				{
					for body in bodies
					{
						state, status := e.body_get(&world, body);
						testing.expect_value(t, status, e.Status.Ok);
						testing.expect(t, abs(state.velocity.linear.y-coefficient*10.1) < 0.0002);
					}
					history := &simulation.restitution.pairs[simulation.restitution.committed];
					testing.expect_value(t, history.count, len(bodies));
					for value in history.values.memory[:history.count]
					{
						testing.expect_value(t, value.phases[0], p.Restitution_Impact_Phase.Spent);
					}
				}
				testing.expect_value(t, e.world_destroy(&world), e.Status.Ok);
			}
		}
	}
}

Restitution_Test_Integration :: struct
{
	counts: [64]i32,
	gravity: f32,
}

restitution_test_integrator_initialize :: proc "contextless" (user_context: rawptr, simulation: ^p.Simulation) -> p.Physics_Status
{
	return .Ok;
}

restitution_test_integrator_prepare :: proc "contextless" (user_context: rawptr, dt: f32) -> p.Physics_Status
{
	return .Ok;
}

restitution_test_integrator_dispose :: proc "contextless" (user_context: rawptr)
{
}

restitution_test_integrator_velocity :: proc "contextless" (
	user_context: rawptr, indices: util.I32x8, position: util.Vector3_Wide,
	orientation: util.Quaternion_Wide, inertia: p.Body_Inertia_Wide,
	mask: util.I32x8, worker_index: int, dt: util.F32x8, velocity: ^p.Body_Velocity_Wide,
)
{
	state: ^Restitution_Test_Integration = (^Restitution_Test_Integration)(user_context);
	for lane in 0 ..< util.PRODUCTION_LANE_COUNT
	{
		if simd.extract(mask, lane) == 0
		{
			continue;
		}
		index: i32 = simd.extract(indices, lane);
		if index >= 0
		{
			_ = sync.atomic_add_explicit(&state.counts[index], 1, .Relaxed);
		}
	}
	velocity.linear.y = simd.add(velocity.linear.y, simd.mul(util.F32x8(state.gravity), dt));
}

@(test)
restitution_fallback_integrates_joint_owned_bodies_once_per_substep :: proc (t: ^testing.T)
{
	for workers in ([3]i32{1, 2, 4})
	{
		for iterations in ([2]i32{1, 4})
		{
			for substeps in ([2]i32{1, 3})
			{
				state: Restitution_Test_Integration = {gravity=-6};
				description := small_world_description();
				description.threading.worker_count = workers;
				description.solve = {velocity_iterations=iterations, substeps=substeps, fallback_batch_threshold=1};
				description.pose_callbacks = {
					initialize=restitution_test_integrator_initialize,
					prepare_for_integration=restitution_test_integrator_prepare,
					integrate_velocity=restitution_test_integrator_velocity,
					dispose=restitution_test_integrator_dispose, user_context=&state,
				};
				world: e.World;
				if !testing.expect_value(t, e.world_init(&world, description), e.Status.Ok)
				{
					return;
				}
				simulation, _ := e.world_borrow_simulation(&world);
				shape, _ := e.shape_add(&world, e.sphere(1));
				inertia, _ := e.shape_inertia(e.sphere(1), 1);
				type_id, valid := register_velocity_target_constraint(t, &world);
				if !valid
				{
					_ = e.world_destroy(&world);
					return;
				}
				bodies: [17]e.Body_Handle;
				for &body, index in bodies
				{
					_, status := e.static_add(&world, e.static_body(shape, e.pose({f32(index)*5, 0, 0})), .None);
					testing.expect_value(t, status, e.Status.Ok);
					body, status = e.body_add(&world, e.body_dynamic(shape, inertia, e.pose({f32(index)*5, 2, 0}),
							e.velocity({0, -10, 0}), e.body_activity(-1, 255)));
					testing.expect_value(t, status, e.Status.Ok);
					for _ in 0 ..< 3
					{
						target: Velocity_Target_Constraint;
						_, status = e.custom_constraint_add_typed(&world, bodies[index:index+1], type_id, &target);
						testing.expect_value(t, status, e.Status.Ok);
					}
				}
				configuration := p.restitution_configuration_default();
				configuration.fallback = {0.5, 1};
				testing.expect_value(t, p.restitution_storage_initialize(simulation, configuration, {}, .Legacy), e.Status.Ok);
				testing.expect_value(t, e.world_stage_predict_bounds(&world, 1.0/60), e.Status.Ok);
				testing.expect_value(t, e.world_stage_collision_detection(&world, 1.0/60), e.Status.Ok);
				state.counts = {};
				if testing.expect_value(t, e.world_stage_solve(&world, 1.0/60), e.Status.Ok)
				{
					for body, index in bodies
					{
						actual, status := e.body_get(&world, body);
						testing.expect_value(t, status, e.Status.Ok);
						expected: f32 = 0.5*(10+0.1/f32(substeps))-0.1*f32(substeps-1)/f32(substeps);
						testing.expect(t, abs(actual.velocity.linear.y-expected) < 0.0003);
						testing.expect_value(t, state.counts[index], substeps);
					}
					testing.expect_value(t, simulation.restitution.state, p.Restitution_Preparation_State.Idle);
					testing.expect(t, simulation.solver.active_set.batch_count > simulation.solver.fallback_batch_index);
				}
				testing.expect_value(t, e.world_destroy(&world), e.Status.Ok);
			}
		}
	}
}

Restitution_Test_Manifold :: struct
{
	policy: e.Default_Narrow_Policy,
	count: i32,
	kind: p.Manifold_Kind,
}

restitution_test_manifold :: proc "contextless" (
	user_context: rawptr, worker_index: int, a, b: p.Collidable_Reference,
	manifold: ^p.Manifold_Result, material: ^p.Contact_Material_Properties,
) -> p.Collision_Testing_State
{
	settings: ^Restitution_Test_Manifold = (^Restitution_Test_Manifold)(user_context);
	contact: p.Convex_Contact = manifold.convex.contacts[0];
	normal: util.Vector3 = manifold.convex.normal;
	offset_b: util.Vector3 = manifold.convex.offset_b;
	material^ = settings.policy.material;
	manifold.kind = settings.kind;
	if settings.kind == .Convex
	{
		manifold.convex.count = settings.count;
		for i in 0 ..< int(settings.count)
		{
			manifold.convex.contacts[i] = contact;
			manifold.convex.contacts[i].feature_id = i32(10+i);
		}
	}
	else
	{
		manifold.nonconvex = {offset_b=offset_b, count=settings.count};
		for i in 0 ..< int(settings.count)
		{
			manifold.nonconvex.contacts[i] = {offset=contact.offset, normal=normal, depth=contact.depth, feature_id=i32(10+i)};
		}
	}
	return .Allow;
}

@(test)
restitution_world_covers_all_contact_families_and_moving_kinematics :: proc (t: ^testing.T)
{
	for kind in ([2]p.Manifold_Kind{.Convex, .Nonconvex})
	{
		for count in 1 ..= 4
		{
			if kind == .Nonconvex && count == 1
			{
				continue;
			}
			for body_count in 1 ..= 2
			{
				for iterations in ([2]i32{1, 4})
				{
					policy: Restitution_Test_Manifold = {policy=e.default_narrow_policy(), count=i32(count), kind=kind};
					policy.policy.material.friction_coefficient = 0;
					description := small_world_description();
					description.gravity = {};
					description.damping = {};
					description.solve.velocity_iterations = iterations;
					description.narrow_callbacks = e.narrow_policy_default(&policy.policy);
					description.narrow_callbacks.configure = restitution_test_manifold;
					world: e.World;
					if !testing.expect_value(t, e.world_init(&world, description), e.Status.Ok)
					{
						return;
					}
					simulation, _ := e.world_borrow_simulation(&world);
					shape, _ := e.shape_add(&world, e.sphere(1));
					inertia, _ := e.shape_inertia(e.sphere(1), 1);
					if body_count == 1
					{
						_, status := e.static_add(&world, e.static_body(shape, e.pose()), .None);
						testing.expect_value(t, status, e.Status.Ok);
					}
					else
					{
						_, status := e.body_add(&world, e.body_kinematic(shape, e.pose(), e.velocity({0, 2, 0}), e.body_activity(-1, 255)));
						testing.expect_value(t, status, e.Status.Ok);
					}
					body, status := e.body_add(&world, e.body_dynamic(shape, inertia, e.pose({0, 2, 0}), e.velocity({0, -10, 0}), e.body_activity(-1, 255)));
					testing.expect_value(t, status, e.Status.Ok);
					configuration := p.restitution_configuration_default();
					configuration.fallback = {0.5, 0};
					testing.expect_value(t, p.restitution_storage_initialize(simulation, configuration, {}, .Legacy), e.Status.Ok);
					if testing.expect_value(t, e.world_step(&world, 1.0/60), e.Status.Ok)
					{
						actual, result := e.body_get(&world, body);
						testing.expect_value(t, result, e.Status.Ok);
						expected: f32 = 5;
						if body_count == 2
						{
							expected = 8;
						}
						testing.expect(t, abs(actual.velocity.linear.y-expected) < 0.0003);
						history := &simulation.restitution.pairs[simulation.restitution.committed];
						if testing.expect_value(t, history.count, 1)
						{
							location := simulation.solver.handle_to_constraint.memory[history.values.memory[0].constraint.value];
							expected_type := p.contact_constraint_type_id(body_count, count, p.Contact_Constraint_Kind(kind));
							testing.expect_value(t, location.type_id, expected_type);
						}
					}
					testing.expect_value(t, e.world_destroy(&world), e.Status.Ok);
				}
			}
		}
	}
}

Restitution_Test_Step_Mode :: enum u8
{
	Success, Fail_After_Solve, Twice
}

Restitution_Test_Step :: struct
{
	mode: Restitution_Test_Step_Mode,
	committed_before: u8,
	premature_commits: int,
}

restitution_test_custom_step :: proc "contextless" (
	raw: rawptr, simulation: ^p.Simulation, dt: f32, dispatcher: ^util.Thread_Dispatcher_Boundary,
) -> p.Physics_Status
{
	context = runtime.default_context();
	state: ^Restitution_Test_Step = (^Restitution_Test_Step)(raw);
	status := p.simulation_predict_bounding_boxes(simulation, dt, dispatcher);
	if status == .Ok
	{
		status = p.simulation_collision_detection(simulation, dt, dispatcher);
	}
	if status == .Ok
	{
		status = p.simulation_solve(simulation, dt, dispatcher);
	}
	if status != .Ok
	{
		return status;
	}
	if simulation.restitution.committed != state.committed_before
	{
		state.premature_commits += 1;
	}
	if state.mode == .Fail_After_Solve
	{
		return .Invalid_Argument;
	}
	if state.mode == .Twice
	{
		status = p.simulation_solve(simulation, dt, dispatcher);
		if status != .Ok
		{
			return status;
		}
		if simulation.restitution.committed != state.committed_before
		{
			state.premature_commits += 1;
		}
	}
	simulation.step_index += 1;
	return .Ok;
}

@(test)
restitution_public_controls_custom_steps_and_success_only_history :: proc (t: ^testing.T)
{
	state: Restitution_Test_Step = {mode=.Fail_After_Solve};
	description := small_world_description();
	description.gravity = {};
	description.damping = {};
	description.timestepper = {step=restitution_test_custom_step, user_context=&state};
	world: e.World;
	if !testing.expect_value(t, e.world_init(&world, description), e.Status.Ok)
	{
		return;
	}
	defer e.world_destroy(&world);
	simulation, _ := e.world_borrow_simulation(&world);
	shape, _ := e.shape_add(&world, e.sphere(1));
	inertia, _ := e.shape_inertia(e.sphere(1), 1);
	_, status := e.static_add(&world, e.static_body(shape, e.pose()), .None);
	testing.expect_value(t, status, e.Status.Ok);
	body, body_status := e.body_add(&world, e.body_dynamic(shape, inertia, e.pose({0, 2, 0}), e.velocity({0, -10, 0}), e.body_activity(-1, 255)));
	testing.expect_value(t, body_status, e.Status.Ok);
	reference, _ := p.collidable_reference_body(.Dynamic, body);
	configuration := e.restitution_configuration_default();
	configuration.fallback = {0.5, 1};
	testing.expect_value(t, e.world_enable_restitution(&world, configuration), e.Status.Ok);
	configuration.fallback = {1, 0};
	value, read_status := e.restitution_get(&world, reference);
	testing.expect_value(t, read_status, e.Status.Ok);
	testing.expect_value(t, value, e.Restitution_Settings{0.5, 1});
	testing.expect_value(t, e.restitution_set(&world, reference, {1, 2}), e.Status.Ok);
	testing.expect_value(t, e.restitution_remove(&world, reference), e.Status.Ok);
	testing.expect_value(t, e.world_step(&world, 1.0/60), e.Status.Invalid_Argument);
	testing.expect_value(t, simulation.step_index, u64(0));
	testing.expect_value(t, simulation.restitution.committed, state.committed_before);
	testing.expect_value(t, simulation.restitution.pairs[state.committed_before].count, 0);
	testing.expect_value(t, simulation.restitution.state, p.Restitution_Preparation_State.Idle);
	// physics already ran. failed publication does not promise a velocity rollback
	actual, _ := e.body_get(&world, body);
	testing.expect(t, abs(actual.velocity.linear.y-5) < 0.0001);
	testing.expect_value(t, e.body_set_pose(&world, body, e.pose({0, 2, 0})), e.Status.Ok);
	testing.expect_value(t, e.body_set_velocity(&world, body, e.velocity({0, -10, 0})), e.Status.Ok);
	state.mode = .Twice;
	testing.expect_value(t, e.world_step(&world, 1.0/60), e.Status.Ok);
	testing.expect_value(t, state.premature_commits, 0);
	testing.expect_value(t, simulation.step_index, u64(1));
	testing.expect_value(t, simulation.restitution.pairs[simulation.restitution.committed].count, 1);
	actual, _ = e.body_get(&world, body);
	testing.expect(t, abs(actual.velocity.linear.y-5) < 0.0002);
	testing.expect_value(t, e.world_disable_restitution(&world), e.Status.Ok);
}

@(test)
restitution_zero_selection_and_triggers_keep_ordinary_response :: proc (t: ^testing.T)
{
	for trigger_mode in 0 ..< 2
	{
		world: e.World;
		description := small_world_description();
		description.gravity = {};
		description.damping = {};
		if !testing.expect_value(t, e.world_init(&world, description), e.Status.Ok)
		{
			return;
		}
		simulation, _ := e.world_borrow_simulation(&world);
		shape, _ := e.shape_add(&world, e.sphere(1));
		inertia, _ := e.shape_inertia(e.sphere(1), 1);
		ground, _ := e.static_add(&world, e.static_body(shape, e.pose()), .None);
		body, _ := e.body_add(&world, e.body_dynamic(shape, inertia, e.pose({0, 1.9, 0}), e.velocity({0, -10, 0}), e.body_activity(-1, 255)));
		configuration := e.restitution_configuration_default();
		if trigger_mode == 1
		{
			configuration.fallback = {1, 0};
			testing.expect_value(t, e.world_enable_triggers(&world), e.Status.Ok);
			reference, _ := p.collidable_reference_static(ground);
			testing.expect_value(t, e.trigger_set(&world, reference), e.Status.Ok);
		}
		testing.expect_value(t, e.world_enable_restitution(&world, configuration), e.Status.Ok);
		testing.expect_value(t, e.world_step(&world, 1.0/60), e.Status.Ok);
		testing.expect_value(t, simulation.restitution.targets.length, 0);
		testing.expect_value(t, simulation.restitution.blocks.length, 0);
		if trigger_mode == 1
		{
			actual, _ := e.body_get(&world, body);
			testing.expect_value(t, actual.velocity.linear.y, f32(-10));
			events: [1]e.Trigger_Event;
			count, _, status := e.trigger_events_drain(&world, events[:]);
			testing.expect_value(t, status, e.Status.Ok);
			testing.expect_value(t, count, 1);
		}
		testing.expect_value(t, e.world_destroy(&world), e.Status.Ok);
	}
}

@(test)
restitution_owned_warm_steps_and_failure_cleanup :: proc (t: ^testing.T)
{
	tracker: allocation_test_tracker;
	world: e.World;
	description := small_world_description();
	description.gravity = {};
	description.damping = {};
	description.threading.worker_count = 2;
	description.allocator = allocation_test_allocator(&tracker);
	if !testing.expect_value(t, e.world_init_with_allocation_scope(&world, description, .All_Owned), e.Status.Ok)
	{
		return;
	}
	simulation, _ := e.world_borrow_simulation(&world);
	shape, _ := e.shape_add(&world, e.sphere(1));
	inertia, _ := e.shape_inertia(e.sphere(1), 1);
	_, status := e.static_add(&world, e.static_body(shape, e.pose()), .None);
	testing.expect_value(t, status, e.Status.Ok);
	_, status = e.body_add(&world, e.body_dynamic(shape, inertia, e.pose({0, 2, 0}), e.velocity({0, -10, 0}), e.body_activity(-1, 255)));
	testing.expect_value(t, status, e.Status.Ok);
	configuration := e.restitution_configuration_default();
	configuration.fallback = {0.5, 1};
	testing.expect_value(t, e.world_enable_restitution(&world, configuration), e.Status.Ok);
	testing.expect_value(t, e.world_step(&world, 1.0/60), e.Status.Ok);
	testing.expect(t, simulation.restitution.targets.length > 0);
	requests := tracker.requests;
	tracker.fail_from = requests+1;
	for _ in 0 ..< 32
	{
		testing.expect_value(t, e.world_step(&world, 1.0/60), e.Status.Ok);
	}
	testing.expect_value(t, tracker.requests, requests);
	testing.expect_value(t, e.world_destroy(&world), e.Status.Ok);
	allocation_test_empty(t, &tracker);
}

@(test)
restitution_parallel_responsibility_rebuild_preserves_target_ranges :: proc (t: ^testing.T)
{
	N :: 4097;
	for workers in ([2]i32{2, 4})
	{
		description := small_world_description();
		description.gravity = {0, -6, 0};
		description.damping = {};
		description.threading.worker_count = workers;
		description.capacity.bodies = N;
		description.capacity.statics = N;
		description.capacity.constraints = N*3;
		description.capacity.pairs = N;
		description.capacity.broad_phase_candidates = N*2;
		world: e.World;
		if !testing.expect_value(t, e.world_init(&world, description), e.Status.Ok)
		{
			return;
		}
		simulation, _ := e.world_borrow_simulation(&world);
		type_id, valid := register_velocity_target_constraint(t, &world);
		if !valid
		{
			_ = e.world_destroy(&world);
			return;
		}
		shape, _ := e.shape_add(&world, e.sphere(1));
		inertia, _ := e.shape_inertia(e.sphere(1), 1);
		bodies: [N]e.Body_Handle;
		for &body, index in bodies
		{
			_, status := e.static_add(&world, e.static_body(shape, e.pose({f32(index)*5, 0, 0})), .None);
			testing.expect_value(t, status, e.Status.Ok);
			body, status = e.body_add(&world, e.body_dynamic(shape, inertia, e.pose({f32(index)*5, 2, 0}), e.velocity({0, -10, 0}), e.body_activity(-1, 255)));
			testing.expect_value(t, status, e.Status.Ok);
			for _ in 0 ..< 2
			{
				target: Velocity_Target_Constraint;
				_, status = e.custom_constraint_add_typed(&world, bodies[index:index+1], type_id, &target);
				testing.expect_value(t, status, e.Status.Ok);
			}
		}
		configuration := e.restitution_configuration_default();
		configuration.pair_capacity = N;
		configuration.fallback = {0.5, 1};
		testing.expect_value(t, e.world_enable_restitution(&world, configuration), e.Status.Ok);
		if testing.expect_value(t, e.world_step(&world, 1.0/60), e.Status.Ok)
		{
			threshold := p.SOLVER_INTEGRATION_PARALLEL_BASE_THRESHOLD+int(workers)*p.SOLVER_INTEGRATION_PARALLEL_PER_WORKER_THRESHOLD;
			testing.expect(t, p.solver_integration_parallel_constraint_count(&simulation.solver) > threshold);
			for body in bodies
			{
				actual, status := e.body_get(&world, body);
				testing.expect_value(t, status, e.Status.Ok);
				testing.expect(t, abs(actual.velocity.linear.y-5.05) < 0.0003);
			}
		}
		testing.expect_value(t, e.world_destroy(&world), e.Status.Ok);
	}
}

@(test)
restitution_two_dynamic_masses_preserve_momentum_in_both_pair_orders :: proc (t: ^testing.T)
{
	for reverse in 0 ..< 2
	{
		for coefficient in ([2]f32{0.5, 1})
		{
			world: e.World;
			description := small_world_description();
			description.gravity={};
			description.damping={};
			description.threading.worker_count=2;
			testing.expect_value(t, e.world_init(&world, description), e.Status.Ok);
			shape, _ := e.shape_add(&world, e.sphere(1));
			first_inertia, _ := e.shape_inertia(e.sphere(1), 1);
			second_inertia, _ := e.shape_inertia(e.sphere(1), 2);
			a, b:e.Body_Handle;
			for index in 0 ..< 2
			{
				if index == reverse
				{
					a, _=e.body_add(&world, e.body_dynamic(shape, first_inertia, e.pose({0, 2, 0}), e.velocity({0, -10, 0}), e.body_activity(-1, 255)));
				}
				else
				{
					b, _=e.body_add(&world, e.body_dynamic(shape, second_inertia, e.pose(), {}, e.body_activity(-1, 255)));
				}
			}
			config:=e.restitution_configuration_default();
			config.fallback={coefficient, 1};
			testing.expect_value(t, e.world_enable_restitution(&world, config), e.Status.Ok);
			if testing.expect_value(t, e.world_step(&world, 1.0/64), e.Status.Ok)
			{
				first, _:=e.body_get(&world, a);
				second, _:=e.body_get(&world, b);
				testing.expect(t, abs(first.velocity.linear.y-(-10*(1-2*coefficient)/3))<0.0003);
				testing.expect(t, abs(second.velocity.linear.y-(-10*(1+coefficient)/3))<0.0003);
				testing.expect(t, abs(first.velocity.linear.y+2*second.velocity.linear.y+10)<0.0003);
			}
			testing.expect_value(t, e.world_destroy(&world), e.Status.Ok);
		}
	}
}

@(test)
restitution_zero_coefficient_is_identical_to_unselected_response :: proc (t: ^testing.T)
{
	worlds:[2]e.World;
	bodies:[2]e.Body_Handle;
	for &world, index in worlds
	{
		description:=small_world_description();
		description.gravity={0, -6, 0};
		description.damping={};
		testing.expect_value(t, e.world_init(&world, description), e.Status.Ok);
		shape, _:=e.shape_add(&world, e.sphere(1));
		inertia, _:=e.shape_inertia(e.sphere(1), 1);
		_, _=e.static_add(&world, e.static_body(shape, e.pose()), .None);
		bodies[index], _=e.body_add(&world, e.body_dynamic(shape, inertia, e.pose({0, 2, 0}), e.velocity({0, -10, 0}), e.body_activity(-1, 255)));
		if index==1
		{
			testing.expect_value(t, e.world_enable_restitution(&world), e.Status.Ok);
		}
	}
	for _ in 0 ..< 24
	{
		for &world in worlds
		{
			testing.expect_value(t, e.world_step(&world, 1.0/64), e.Status.Ok);
		}
		a, _:=e.body_get(&worlds[0], bodies[0]);
		b, _:=e.body_get(&worlds[1], bodies[1]);
		testing.expect_value(t, a.pose, b.pose);
		testing.expect_value(t, a.velocity, b.velocity);
	}
	simulation, _:=e.world_borrow_simulation(&worlds[1]);
	testing.expect(t, simulation.restitution.targets.memory==nil && simulation.restitution.bundles.memory==nil);
	for &world in worlds
	{
		testing.expect_value(t, e.world_destroy(&world), e.Status.Ok);
	}
}

@(test)
restitution_genuine_reimpacts_decay_and_settle_below_threshold :: proc (t: ^testing.T)
{
	world:e.World;
	description:=small_world_description();
	description.gravity={0, -9.81, 0};
	description.damping={};
	description.threading.worker_count=2;
	testing.expect_value(t, e.world_init(&world, description), e.Status.Ok);
	shape, _:=e.shape_add(&world, e.sphere(1));
	inertia, _:=e.shape_inertia(e.sphere(1), 1);
	_, _=e.static_add(&world, e.static_body(shape, e.pose()), .None);
	body, _:=e.body_add(&world, e.body_dynamic(shape, inertia, e.pose({0, 4, 0}), {}, e.body_activity(-1, 255)));
	config:=e.restitution_configuration_default();
	config.fallback={0.65, 1};
	testing.expect_value(t, e.world_enable_restitution(&world, config), e.Status.Ok);
	previous:f32;
	previous_bounce:f32=100;
	bounce_count:=0;
	for step in 0 ..< 400
	{
		if !testing.expect_value(t, e.world_step(&world, 1.0/120), e.Status.Ok)
		{
			break;
		}
		state, _:=e.body_get(&world, body);
		current:=state.velocity.linear.y;
		if previous < -1 && current > 0.7
		{
			testing.expect(t, current < previous_bounce);
			previous_bounce=current;
			bounce_count+=1;
		}
		if step>=340
		{
			testing.expect(t, abs(current)<0.25);
		}
		previous=current;
	}
	testing.expect(t, bounce_count>=2);
	testing.expect_value(t, e.world_destroy(&world), e.Status.Ok);
}

// exercise type boundaries, uneven ranges, nonzero work-array offsets and all
// lanes independently of the selected solver's currently preferred partition
@(test)
restitution_work_partition_lookup_matches_linear_reference :: proc (t: ^testing.T)
{
	blocks: [8]p.Solver_Work_Block = {
		{type_batch_index=0, start_bundle=0, end_bundle=99},
		{type_batch_index=0, start_bundle=0, end_bundle=1},
		{type_batch_index=0, start_bundle=1, end_bundle=3},
		{type_batch_index=2, start_bundle=0, end_bundle=4},
		{type_batch_index=2, start_bundle=4, end_bundle=5},
		{type_batch_index=2, start_bundle=5, end_bundle=9},
		{type_batch_index=5, start_bundle=0, end_bundle=1},
		{type_batch_index=7, start_bundle=0, end_bundle=99},
	};
	batches: [1]p.Constraint_Batch;
	batches[0].type_id_to_batch_index[0] = 0;
	batches[0].type_id_to_batch_index[1] = 1;
	batches[0].type_id_to_batch_index[2] = 2;
	batches[0].type_id_to_batch_index[3] = 4;
	batches[0].type_id_to_batch_index[4] = 5;
	batches[0].type_id_to_batch_index[5] = 7;
	batches[0].work_block_start = 1;
	batches[0].work_block_count = 6;
	simulation: ^p.Simulation = new(p.Simulation);
	defer mem.free(simulation);
	simulation.solver.active_set.batches = {memory=&batches[0], length=len(batches)};
	simulation.solver.work_blocks = {memory=&blocks[0], length=len(blocks)};
	simulation.solver.fallback_batch_index = 1;
	storage: p.Restitution_Storage = {simulation=simulation, normal_work_count=7};
	for type_id in i32(0) ..< 6
	{
		for constraint_index in i32(0) ..< 80
		{
			expected: int = -1;
			for index in int(1) ..< 7
			{
				block: ^p.Solver_Work_Block = &blocks[index];
				bundle: i32 = constraint_index/util.PRODUCTION_LANE_COUNT;
				if block.type_batch_index == batches[0].type_id_to_batch_index[type_id] &&
				bundle >= block.start_bundle && bundle < block.end_bundle
				{
					expected = index;
					break;
				}
			}
			location: p.Constraint_Location = {batch_index=0, type_id=type_id, index_in_type_batch=constraint_index};
			testing.expect_value(t, p.restitution_pair_block(&storage, location), expected);
		}
	}
	batches[0].work_block_count = 0;
	testing.expect_value(t, p.restitution_pair_block(&storage, {}), -1);
	for type_id in p.CONTACT_1_ONE_BODY_TYPE_ID ..= p.CONTACT_4_NONCONVEX_TYPE_ID
	{
		testing.expect_value(t, p.restitution_pair_block(&storage, {batch_index=1, type_id=i32(type_id)}), 7+int(type_id));
	}
}
