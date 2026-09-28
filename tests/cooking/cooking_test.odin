package cooking_tests

import "core:testing"
import entasis "entasis:entasis"
import cooking "entasis:entasis_cooking"
import physics "entasis:entasis_physics"

cooking_world_description :: proc () -> entasis.World_Description
{
	description := entasis.world_description_default();
	description.gravity = {};
	description.threading.worker_count = 1;
	return description;
}

@(test)
cooked_hull_import_copies_topology_and_inertia :: proc(t: ^testing.T)
{
	ctx: cooking.Cooking_Context;
	if !testing.expect_value(t, cooking.cooking_context_init(&ctx), entasis.Status.Ok)
	{
		return;
	}
	defer cooking.cooking_context_destroy(&ctx);

	points := [8]entasis.Vector3{
		{-1, -1, -1}, {1, -1, -1}, {-1, 1, -1}, {1, 1, -1},
		{-1, -1, 1}, {1, -1, 1}, {-1, 1, 1}, {1, 1, 1},
	};
	cooked, cook_status := cooking.cook_hull(&ctx, points[:]);
	if !testing.expect_value(t, cook_status, entasis.Status.Ok)
	{
		return;
	}
	defer cooking.cooked_hull_destroy(&cooked);

	direct_inertia, direct_status := physics.convex_hull_inertia(&cooked.hull, 3);
	if !testing.expect_value(t, direct_status, entasis.Status.Ok)
	{
		return;
	}

	world: entasis.World;
	if !testing.expect_value(t, entasis.world_init(&world, cooking_world_description()), entasis.Status.Ok)
	{
		return;
	}
	defer entasis.world_destroy(&world);

	handle, import_status := cooking.cooked_hull_import(&world, &cooked);
	if !testing.expect_value(t, import_status, entasis.Status.Ok)
	{
		return;
	}
	registered_inertia, registered_status := entasis.shape_registered_inertia(&world, handle, 3);
	testing.expect_value(t, registered_status, entasis.Status.Ok);
	testing.expect_value(t, registered_inertia, direct_inertia);
	testing.expect_value(t, entasis.shape_remove(&world, handle), entasis.Status.Ok);
}

@(test)
cooked_mesh_and_compound_import_support_world_queries :: proc(t: ^testing.T)
{
	ctx: cooking.Cooking_Context;
	if !testing.expect_value(t, cooking.cooking_context_init(&ctx), entasis.Status.Ok)
	{
		return;
	}
	defer cooking.cooking_context_destroy(&ctx);

	triangles := [2]entasis.Triangle{
		entasis.triangle({-2, 0, -2}, {2, 0, -2}, {2, 0, 2}),
		entasis.triangle({-2, 0, -2}, {2, 0, 2}, {-2, 0, 2}),
	};
	mesh, mesh_status := cooking.cook_mesh(&ctx, triangles[:]);
	if !testing.expect_value(t, mesh_status, entasis.Status.Ok)
	{
		return;
	}
	defer cooking.cooked_mesh_destroy(&mesh);

	children := [2]cooking.Cooked_Compound_Child{
		{local_pose=entasis.pose({-1, 0, 0}), shape_slot=0},
		{local_pose=entasis.pose({1, 0, 0}), shape_slot=1},
	};
	compound, compound_status := cooking.cook_compound(&ctx, children[:]);
	if !testing.expect_value(t, compound_status, entasis.Status.Ok)
	{
		return;
	}
	defer cooking.cooked_compound_destroy(&compound);

	world: entasis.World;
	if !testing.expect_value(t, entasis.world_init(&world, cooking_world_description()), entasis.Status.Ok)
	{
		return;
	}
	defer entasis.world_destroy(&world);

	mesh_handle, import_mesh_status := cooking.cooked_mesh_import(&world, &mesh);
	if !testing.expect_value(t, import_mesh_status, entasis.Status.Ok)
	{
		return;
	}
	_, static_status := entasis.static_add(
		&world, entasis.static_body(mesh_handle, entasis.pose({0, -1, 0})), .None,
	);
	if !testing.expect_value(t, static_status, entasis.Status.Ok)
	{
		return;
	}
	hit, ray_status := entasis.ray_cast_closest(
		&world, entasis.ray({0, 2, 0}, {0, -1, 0}, 10),
	);
	testing.expect_value(t, ray_status, entasis.Status.Ok);
	testing.expect(t, hit.t > 2 && hit.t < 4);

	sphere_handle, sphere_status := entasis.shape_add(&world, entasis.sphere(0.5));
	if !testing.expect_value(t, sphere_status, entasis.Status.Ok)
	{
		return;
	}
	box_handle, box_status := entasis.shape_add(&world, entasis.box(1, 1, 1));
	if !testing.expect_value(t, box_status, entasis.Status.Ok)
	{
		return;
	}
	compound_handle, import_compound_status := cooking.cooked_compound_import(
		&world, &compound, []entasis.Shape_Handle{sphere_handle, box_handle},
	);
	if !testing.expect_value(t, import_compound_status, entasis.Status.Ok)
	{
		return;
	}
	_, compound_static_status := entasis.static_add(
		&world, entasis.static_body(compound_handle, entasis.pose({0, 2, 0})), .None,
	);
	testing.expect_value(t, compound_static_status, entasis.Status.Ok);
}

@(test)
cooking_context_rejects_destroy_with_live_assets :: proc(t: ^testing.T)
{
	ctx: cooking.Cooking_Context;
	if !testing.expect_value(t, cooking.cooking_context_init(&ctx), entasis.Status.Ok)
	{
		return;
	}
	points := [4]entasis.Vector3{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
	cooked, status := cooking.cook_hull(&ctx, points[:]);
	if !testing.expect_value(t, status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect_value(t, cooking.cooking_context_destroy(&ctx), entasis.Status.Shape_In_Use);
	testing.expect_value(t, cooking.cooked_hull_destroy(&cooked), entasis.Status.Ok);
	testing.expect_value(t, cooking.cooking_context_destroy(&ctx), entasis.Status.Ok);
}
