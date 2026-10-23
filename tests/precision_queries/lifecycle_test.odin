package precision_queries_tests

import "core:math"
import "core:testing"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

static_mesh_reference_bounds :: proc "contextless" (
	shape: rawptr, orientation: util.Quaternion, registry: ^physics.Shape_Registry,
) -> (physics.Shape_Bounds, physics.Physics_Status)
{
	return physics.mesh_bounds_proc(shape, orientation, registry);
}

static_mesh_failed_bounds :: proc "contextless" (
	shape: rawptr, orientation: util.Quaternion, registry: ^physics.Shape_Registry,
) -> (physics.Shape_Bounds, physics.Physics_Status)
{
	return {}, .Invalid_Argument;
}

expect_static_mesh_ray_recomputation :: proc (
	t: ^testing.T, scene: ^Scene, handle: physics.Static_Handle,
	segment: physics.World_Precision_Segment, expected_position: entasis.Global_Position,
)
{
	reference: physics.Collidable_Reference;
	status: entasis.Status;
	reference, status = physics.collidable_reference_static(handle);
	testing.expect_value(t, status, entasis.Status.Ok);
	stack: [256]i32;
	closest: [2]entasis.Global_Ray_Hit;
	all: [2][4]entasis.Global_Ray_Hit;
	counts: [2]int;
	batch: ^physics.Shape_Batch = &scene.simulation.shapes.batches[physics.MESH_TYPE_ID];
	for pass in 0 ..< 2
	{
		// the wrapper forces the existing registered-callback route without
		// changing its numerical operations or the target's geometry
		batch.metadata.bounds = physics.mesh_bounds_proc if pass == 0 else static_mesh_reference_bounds;
		closest[pass], status = entasis.ray_cast_global_closest_with_context(&scene.query, segment, stack[:]);
		testing.expect_value(t, status, entasis.Status.Ok);
		state: entasis.Query_Hit_State;
		state, status = entasis.ray_cast_global_any_with_context(&scene.query, segment, stack[:]);
		testing.expect_value(t, status, entasis.Status.Ok);
		testing.expect_value(t, state, entasis.Query_Hit_State.Hit);
		counts[pass], status = entasis.ray_cast_global_all_with_context(&scene.query, segment, all[pass][:], stack[:]);
		testing.expect_value(t, status, entasis.Status.Ok);
		testing.expect_value(t, counts[pass], 1);
	}
	batch.metadata.bounds = physics.mesh_bounds_proc;
	testing.expect_value(t, closest[0], closest[1]);
	testing.expect_value(t, all[0][0], all[1][0]);
	testing.expect_value(t, closest[0].collidable, reference);
	expect_position(t, closest[0].location, expected_position);
}

@(test)
global_native_static_bounds_reuse_preserves_mutations_and_admission :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene, 8);
	defer scene_dispose(t, &scene);
	triangles: [2]physics.Triangle = {
		{{-1, 0, -1}, {1, 0, -1}, {-1, 0, 1}},
		{{1, 0, -1}, {1, 0, 1}, {-1, 0, 1}},
	};
	scales: [3]f32 = {1, 2, 4};
	shapes: [3]physics.Typed_Index;
	status: entasis.Status;
	positions: [3]entasis.Global_Position = {{1e9+0.125, 1e9+0.25, -1e9-0.375}, {1e9+8.125, 1e9+0.25, -1e9-0.375}, {1e9+16.125, 1e9+0.25, -1e9-0.375}};
	descriptions: [3]entasis.Static_Description;
	segments: [3]physics.World_Precision_Segment;
	expected_positions: [3]entasis.Global_Position;
	for scale, index in scales
	{
		mesh: physics.Mesh;
		testing.expect_value(t, physics.mesh_create(&mesh, &triangles[0], len(triangles), {scale, 1, 1}, &scene.pool), physics.Physics_Status.Ok);
		shapes[index], status = physics.shape_registry_add(&scene.simulation.shapes, physics.MESH_TYPE_ID, &mesh);
		testing.expect_value(t, status, entasis.Status.Ok);
		descriptions[index] = entasis.static_body(shapes[index], entasis.pose());
		x: f64 = 0.75*f64(scale);
		segments[index] = translated_segment({start={x, 4, 0.125}, end={x, -4, 0.125}}, positions[index]);
		expected_positions[index] = {positions[index].x+x, positions[index].y, positions[index].z+0.125};
	}
	handles: [3]entasis.Static_Handle;
	written: int;
	written, status = entasis.static_add_batch_global(&scene.world, descriptions[:], handles[:], positions[:], .None);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect_value(t, written, 3);
	for phase in 0 ..< 3
	{
		if phase == 1
		{
			testing.expect_value(t, physics.statics_ensure_capacity(&scene.simulation.statics, 256), physics.Physics_Status.Ok);
		}
		if phase == 2
		{
			testing.expect_value(t, physics.statics_resize(&scene.simulation.statics, 4), physics.Physics_Status.Ok);
		}
		for handle, index in handles
		{
			expect_static_mesh_ray_recomputation(t, &scene, handle, segments[index], expected_positions[index]);
		}
	}
	testing.expect_value(t, entasis.static_remove(&scene.world, handles[0], .None), entasis.Status.Ok);
	_, status = entasis.static_get_global_pose(&scene.world, handles[0]);
	testing.expect_value(t, status, entasis.Status.Not_Found);
	removed_reference: physics.Collidable_Reference;
	removed_reference, _ = physics.collidable_reference_static(handles[0]);
	_, _, status = physics.simulation_query_global_target(scene.simulation, removed_reference);
	testing.expect_value(t, status, entasis.Status.Not_Found);
	expect_static_mesh_ray_recomputation(t, &scene, handles[2], segments[2], expected_positions[2]);
	handles[0], status = entasis.static_add_global(&scene.world, descriptions[0], positions[0], .None);
	testing.expect_value(t, status, entasis.Status.Ok);
	expect_static_mesh_ray_recomputation(t, &scene, handles[0], segments[0], expected_positions[0]);
	rotation: util.Quaternion = util.quaternion_from_axis_angle({0, 1, 0}, 0.5);
	testing.expect_value(t, entasis.static_set_global_pose(&scene.world, handles[0], {position=positions[0], orientation=rotation}, .None), entasis.Status.Ok);
	point: util.Vector3 = util.quaternion_transform({0.75, 0, 0.75}, rotation);
	segments[0] = translated_segment({start={f64(point.x), 4, f64(point.z)}, end={f64(point.x), -4, f64(point.z)}}, positions[0]);
	expected_positions[0] = {positions[0].x+f64(point.x), positions[0].y, positions[0].z+f64(point.z)};
	expect_static_mesh_ray_recomputation(t, &scene, handles[0], segments[0], expected_positions[0]);
	testing.expect_value(t, entasis.static_set_shape(&scene.world, handles[0], shapes[1], .None), entasis.Status.Ok);
	point = util.quaternion_transform({1.5, 0, 0.25}, rotation);
	segments[0] = translated_segment({start={f64(point.x), 4, f64(point.z)}, end={f64(point.x), -4, f64(point.z)}}, positions[0]);
	expected_positions[0] = {positions[0].x+f64(point.x), positions[0].y, positions[0].z+f64(point.z)};
	expect_static_mesh_ray_recomputation(t, &scene, handles[0], segments[0], expected_positions[0]);
	index: int;
	index, _ = physics.statics_resolve(&scene.simulation.statics, handles[0]);
	scene.simulation.statics.statics.memory[index].pose.orientation = util.quaternion_identity();
	testing.expect_value(t, entasis.static_update_bounds(&scene.world, handles[0]), entasis.Status.Ok);
	segments[0] = translated_segment({start={0.25, 4, 0.125}, end={0.25, -4, 0.125}}, positions[0]);
	expected_positions[0] = {positions[0].x+0.25, positions[0].y, positions[0].z+0.125};
	expect_static_mesh_ray_recomputation(t, &scene, handles[0], segments[0], expected_positions[0]);
	// the failed refresh must discard the old zero-height local bounds
	// even after canonical callback admission is restored
	scene.simulation.statics.statics.memory[index].pose.orientation = util.quaternion_from_axis_angle({1, 0, 0}, 0.5);
	batch: ^physics.Shape_Batch = &scene.simulation.shapes.batches[physics.MESH_TYPE_ID];
	batch.metadata.bounds = static_mesh_failed_bounds;
	testing.expect_value(t, entasis.static_update_bounds(&scene.world, handles[0]), entasis.Status.Invalid_Argument);
	stack: [256]i32;
	_, status = entasis.ray_cast_global_closest_with_context(&scene.query, segments[0], stack[:]);
	testing.expect_value(t, status, entasis.Status.Invalid_Argument);
	batch.metadata.bounds = physics.mesh_bounds_proc;
	expected_positions[0].y = positions[0].y-0.125*math.tan(f64(0.5));
	expect_static_mesh_ray_recomputation(t, &scene, handles[0], segments[0], expected_positions[0]);
	testing.expect_value(t, entasis.static_update_bounds(&scene.world, handles[0]), entasis.Status.Ok);
	testing.expect_value(t, physics.broad_phase_update_bounds(&scene.simulation.broad_phase), physics.Physics_Status.Ok);
	expect_static_mesh_ray_recomputation(t, &scene, handles[0], segments[0], expected_positions[0]);
	positions[0].x += 0.125;
	testing.expect_value(t, entasis.static_set_global_pose(&scene.world, handles[0], {position=positions[0], orientation=util.quaternion_identity()}, .None), entasis.Status.Ok);
	segments[0] = translated_segment({start={0.25, 4, 0.125}, end={0.25, -4, 0.125}}, positions[0]);
	expected_positions[0] = {positions[0].x+0.25, positions[0].y, positions[0].z+0.125};
	expect_static_mesh_ray_recomputation(t, &scene, handles[0], segments[0], expected_positions[0]);
	frame: entasis.Origin_Frame;
	testing.expect_value(t, entasis.world_shift_origin(&scene.world, &frame, {1e9, 1e9, -1e9}), entasis.Status.Ok);
	shift: entasis.Global_Position = {-frame.origin.x, -frame.origin.y, -frame.origin.z};
	segments[0] = translated_segment(segments[0], shift);
	expected_positions[0] = {expected_positions[0].x+shift.x, expected_positions[0].y+shift.y, expected_positions[0].z+shift.z};
	expect_static_mesh_ray_recomputation(t, &scene, handles[0], segments[0], expected_positions[0]);
	// warm both paths before checking the reserved repeated-query interval
	allocated: i64 = scene.tracker.current_memory_allocated;
	for _ in 0 ..< 8
	{
		expect_static_mesh_ray_recomputation(t, &scene, handles[0], segments[0], expected_positions[0]);
	}
	testing.expect_value(t, scene.tracker.current_memory_allocated, allocated);
	// low-level detached statics deliberately have no published cache
	testing.expect_value(t, entasis.world_clear(&scene.world), entasis.Status.Ok);
	shape: physics.Typed_Index = add_shape(t, &scene, .Mesh);
	description: physics.Static_Description = {shape=shape, pose={orientation=util.quaternion_identity()}};
	handles[0], status = physics.statics_add(&scene.simulation.statics, &description);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect_value(t, physics.statics_set_global_position(&scene.simulation.statics, handles[0], {0.125, 0.25, -0.375}), physics.Physics_Status.Ok);
	bounds: physics.Tree_Bounds;
	bounds, status = physics.broad_phase_shape_bounds(&scene.simulation.shapes, shape, {0.125, 0.25, -0.375}, util.quaternion_identity());
	testing.expect_value(t, status, entasis.Status.Ok);
	reference: physics.Collidable_Reference;
	reference, _ = physics.collidable_reference_static(handles[0]);
	_, status = physics.broad_phase_add_leaf(&scene.simulation.broad_phase, .Static, reference, bounds);
	testing.expect_value(t, status, entasis.Status.Ok);
	segments[0] = translated_segment({start={0.25, 4, 0.125}, end={0.25, -4, 0.125}}, {0.125, 0.25, -0.375});
	expect_static_mesh_ray_recomputation(t, &scene, handles[0], segments[0], {0.375, 0.25, -0.25});
}

@(test)
global_query_pose_lifecycle_sleep_compaction_reuse_and_rebase :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	shape: physics.Typed_Index = add_shape(t, &scene, .Box);
	positions: [4]physics.Global_Position = {
		{1e9 + 0.125, 1e9 + 0.25, -1e9 - 0.375},
		{1e9 + 0.125, 1e9 + 4.25, -1e9 - 0.375},
		{1e9 + 0.125, 1e9 + 8.25, -1e9 - 0.375},
		{1e9 + 0.125, 1e9 + 12.25, -1e9 - 0.375},
	};
	bodies: [3]physics.Body_Handle;
	bodies[0] = add_body(t, &scene, shape, positions[0]);
	bodies[1] = add_body(t, &scene, shape, positions[1], .Kinematic);
	bodies[2] = add_body(t, &scene, shape, positions[2]);
	stationary: physics.Static_Handle = add_static(t, &scene, shape, positions[3]);
	references: [4]physics.Collidable_Reference;
	for body, index in bodies
	{
		references[index], _ = physics.collidable_reference_create(.Kinematic if index == 1 else .Dynamic, int(body.value));
	}
	references[3], _ = physics.collidable_reference_create(.Static, int(stationary.value));
	exported: [4]entasis.Global_Pose;
	frame: physics.Origin_Frame;
	stack: [128]i32;
	for phase in 0 ..< 4
	{
		if phase == 1
		{
			sleep_body(t, &scene, bodies[0]);
			sleep_body(t, &scene, bodies[2]);
		}
		if phase == 2
		{
			testing.expect_value(t, physics.island_awakener_awaken_body(&scene.simulation.awakener, bodies[0]), physics.Physics_Status.Ok);
			testing.expect_value(t, entasis.world_shift_origin(&scene.world, &frame, {1e9, 1e9, -1e9}), entasis.Status.Ok);
		}
		if phase == 3
		{
			testing.expect_value(t, entasis.body_remove(&scene.world, bodies[0]), entasis.Status.Ok);
			_, removed_status := entasis.body_get_global_pose(&scene.world, bodies[0]);
			testing.expect_value(t, removed_status, entasis.Status.Not_Found);
			local: physics.Global_Position = {positions[0].x - frame.origin.x, positions[0].y - frame.origin.y, positions[0].z - frame.origin.z};
			replacement: physics.Body_Handle = add_body(t, &scene, shape, local);
			testing.expect_value(t, replacement, bodies[0]);
			bodies[0] = replacement;
		}
		count: int;
		status: entasis.Status;
		count, status = entasis.world_export_global_poses(&scene.world, references[:], exported[:], frame);
		testing.expect_value(t, status, entasis.Status.Ok);
		testing.expect_value(t, count, 4);
		for position, index in positions
		{
			expect_position(t, exported[index].position, position, 0);
			local: physics.Global_Position = {position.x - frame.origin.x, position.y - frame.origin.y, position.z - frame.origin.z};
			segment: physics.World_Precision_Segment = translated_segment({start={-2, 0, 0}, end={2, 0, 0}}, local);
			hit: entasis.Global_Ray_Hit;
			hit, status = entasis.ray_cast_global_closest_with_context(&scene.query, segment, stack[:]);
			testing.expect_value(t, status, entasis.Status.Ok);
			testing.expect_value(t, hit.collidable, references[index]);
			expect_position(t, hit.location, {local.x - 0.5, local.y, local.z}, 0);
		}
	}
	pose: entasis.Global_Pose;
	status: entasis.Status;
	pose, status = entasis.static_get_global_pose(&scene.world, stationary);
	testing.expect_value(t, status, entasis.Status.Ok);
	expect_position(t, pose.position, {positions[3].x - frame.origin.x, positions[3].y - frame.origin.y, positions[3].z - frame.origin.z}, 0);
}
