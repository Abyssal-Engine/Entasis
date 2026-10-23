package large_world_precision

import "core:math"
import entasis "entasis:entasis"
import cooking "entasis:entasis_cooking"
import support "../benchmark_support"

// holdouts simulate 1,000 active bodies at 1,000 km. the mixed
// fixture contains 250 each of box, sphere, hull and compound, with a mesh floor
holdout_options :: proc(workers: int, output: string, precision_case: Precision_Case) -> support.Options
{
	spacing: f32 = 1.02;
	if precision_case == .Holdout_Mixed
	{
		spacing = 1.25;
	}
	return {
		workload=.Container, shape=.Box, static_shape=.Box,
		shape_size={1, 1, 1}, density=1, layout_scale=1,
		steps=MEASURED_STEPS, timestep_hz=60, warmup_steps=WARMUP_STEPS,
		velocity_iterations=1, substeps=4, sleep=.Disabled,
		grid={10, 10, 10}, spacing={spacing, spacing, spacing},
		spawn_height=8, container_size={29, 24.5, 29}, worker_count=workers,
		output=output, population_count=1_005, body_count=1_000, static_count=5,
	};
}

holdout_mesh_floor :: proc(owner: ^Benchmark_Owner) -> (entasis.Shape_Handle, entasis.Status)
{
	ctx: cooking.Cooking_Context;
	status := cooking.cooking_context_init(&ctx, POOL_MINIMUM_BLOCK_SIZE);
	if status != .Ok
	{
		return {}, status;
	}
	defer cooking.cooking_context_destroy(&ctx);
	triangles := [2]entasis.Triangle{
		entasis.triangle({-14.5, 0, -14.5}, {14.5, 0, -14.5}, {14.5, 0, 14.5}),
		entasis.triangle({-14.5, 0, -14.5}, {14.5, 0, 14.5}, {-14.5, 0, 14.5}),
	};
	mesh, mesh_status := cooking.cook_mesh(&ctx, triangles[:]);
	if mesh_status != .Ok
	{
		return {}, mesh_status;
	}
	defer cooking.cooked_mesh_destroy(&mesh);
	return cooking.cooked_mesh_import(&owner.world, &mesh);
}

holdout_build_mixed :: proc(owner: ^Benchmark_Owner) -> entasis.Status
{
	floor_shape, status := holdout_mesh_floor(owner);
	if status != .Ok
	{
		return status;
	}
	side, side_status := entasis.shape_add(&owner.world, entasis.box(1, 24.5, 29));
	front, front_status := entasis.shape_add(&owner.world, entasis.box(29, 24.5, 1));
	if side_status != .Ok
	{
		return side_status;
	}
	if front_status != .Ok
	{
		return front_status;
	}
	static_shapes := [5]entasis.Shape_Handle{floor_shape, side, side, front, front};
	static_positions := [5]entasis.Vector3{{0, 0, 0}, {-14.5, 11.75, 0}, {14.5, 11.75, 0}, {0, 11.75, -14.5}, {0, 11.75, 14.5}};
	for position, index in static_positions
	{
		status = add_static_fixture(owner, static_shapes[index], position);
		if status != .Ok
		{
			return status;
		}
	}
	shapes: [4]entasis.Shape_Handle;
	inertias: [4]entasis.Body_Inertia;
	shapes[0], status = entasis.shape_add(&owner.world, entasis.box(1, 1, 1));
	if status != .Ok
	{
		return status;
	}
	shapes[1], status = entasis.shape_add(&owner.world, entasis.sphere(0.5));
	if status != .Ok
	{
		return status;
	}
	shapes[2], status = support.register_uncached_shape(&owner.world, .Hull, {1, 1, 1});
	if status != .Ok
	{
		return status;
	}
	child, child_status := entasis.shape_add(&owner.world, entasis.box(0.4, 0.8, 0.8));
	if child_status != .Ok
	{
		return child_status;
	}
	children := [2]entasis.Compound_Child{
		entasis.compound_child(child, entasis.pose({-0.25, 0, 0})),
		entasis.compound_child(child, entasis.pose({0.25, 0, 0})),
	};
	shapes[3], status = entasis.shape_import_compound(&owner.world, children[:]);
	if status != .Ok
	{
		return status;
	}
	for shape, index in shapes[:3]
	{
		inertias[index], status = entasis.shape_registered_inertia(&owner.world, shape, 1);
		if status != .Ok
		{
			return status;
		}
	}
	masses := [2]f32{0.5, 0.5};
	inertias[3], status = entasis.compound_inertia_weighted(&owner.world, entasis.compound_builder(children[:], masses[:]));
	if status != .Ok
	{
		return status;
	}
	for index in 0 ..< 1_000
	{
		kind := index%4;
		position := entasis.Vector3{
			(f32(index%10)-4.5)*1.25,
			8+f32(index/100)*1.25,
			(f32((index/10)%10)-4.5)*1.25,
		};
		global := global_from_local_fixture(owner, position);
		owner.expected_global_positions[index] = global;
		description := entasis.body_dynamic(shapes[kind], inertias[kind], entasis.pose(), {}, entasis.body_activity(-1, 32));
		description.collidable.maximum_speculative_margin = f32(math.F32_MAX);
		owner.bodies[index], status = candidate_body_add(owner, description, global);
		if status != .Ok
		{
			return status;
		}
	}
	return .Ok;
}
