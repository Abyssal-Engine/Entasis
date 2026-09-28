# Cooking

Use `entasis:entasis_cooking` to prepare convex hulls, meshes, and compounds without mutating a live world

```odin
import entasis "entasis:entasis"
import cooking "entasis:entasis_cooking"
```

Cooking has no scheduler or global state. Run it synchronously, on an application worker, or in offline tooling

## Context lifecycle

The fragments below belong inside an application procedure, with the imports above. Initialize the context before creating assets and schedule its cleanup before asset cleanup, so Odin's reverse-order `defer` releases assets first

```odin
ctx: cooking.Cooking_Context;
status := cooking.cooking_context_init(&ctx);
if status != .Ok
{
	return;
}
defer cooking.cooking_context_destroy(&ctx);
```

A context owns one `Buffer_Pool`. Use `cooking_context_init_with_allocator` for [full allocator ownership](reference/WORLD.md#explicit-allocation-scope)

`cooking_context_clear` and `cooking_context_destroy` return `Shape_In_Use` while any cooked asset owned by the context remains live

## Supported outputs

| Output       | Cooking work                                                      | Import work                                                           |
| ------------ | ----------------------------------------------------------------- | --------------------------------------------------------------------- |
| Convex hull  | Builds hull topology from a point cloud                           | Copies prepared hull data into the world and registers a shape        |
| Mesh         | Builds triangle bounds and the acceleration tree                  | Copies triangles, bounds, and the tree into world storage             |
| Compound     | Copies portable children that refer to caller-defined shape slots | Resolves slots through world shape handles and registers the compound |
| Big compound | Builds a tree over child bounds                                   | Resolves slots and imports the prepared tree                          |

## Convex hull flow

This fragment requires an initialized `ctx`, an idle destination `world` and a `points` slice of `entasis.Vector3`. It returns on failure to the containing procedure, whose deferred cleanup must release any resources already initialized

```odin
cooked, status := cooking.cook_hull(&ctx, points);
if status != .Ok
{
	return;
}
defer cooking.cooked_hull_destroy(&cooked);

shape, import_status := cooking.cooked_hull_import(&world, &cooked);
if import_status != .Ok
{
	return;
}
```

Import uses the prepared hull without rebuilding it

## Mesh flow

Use the hull sequence above with `cook_mesh`, `cooked_mesh_import` and `cooked_mesh_destroy`. Input is a triangle slice. The cooked mesh mass-property procedures provide closed-solid or triangle-soup mass data before import

## Compound slots

Cooked compounds are portable across worlds because children store shape-table slots instead of live `Shape_Handle` values

This fragment requires an initialized `ctx`, an idle `world` and live `box_shape` and `sphere_shape` handles registered in that world. Use a separate scope from the hull fragment because both declare `cooked` and `status`

```odin
children := []cooking.Cooked_Compound_Child{
	{local_pose=entasis.pose({0, 0, 0}), shape_slot=0},
	{local_pose=entasis.pose({1, 0, 0}), shape_slot=1},
};

cooked, status := cooking.cook_compound(&ctx, children);
if status != .Ok
{
	return;
}
defer cooking.cooked_compound_destroy(&cooked);

shape_slots := []entasis.Shape_Handle{box_shape, sphere_shape};
compound, import_status := cooking.cooked_compound_import(&world, &cooked, shape_slots);
if import_status != .Ok
{
	return;
}
```

Every referenced slot must exist in the import table and contain a live shape from the destination world

## Ownership and concurrency

- Input slices remain caller owned and are copied during cooking
- Cooked assets belong to exactly one cooking context, kept at a stable address
- Only one worker may use a context or its assets at a time
- Destroy every cooked asset before clearing or destroying its context
- Import runs while the destination world is idle on its owner thread
- Import copies data and does not transfer or consume the cooked asset
- Imported shapes belong to the world and use normal shared-shape lifetime rules
- Cooking may overlap a world step only when the context, assets, and inputs are independent of that world

## Background cooking

[`background_cooking`](../../examples/headless/background_cooking/main.odin) shows cooking on an application thread and importing after the worker completes

The example stores the context, asset and status in one shared `Cook_Job`. The worker finishes before the simulation thread reads the result. Keep that job at a stable address through import and cleanup. Destroy the cooked asset before its context. Odin runs `defer` statements in reverse order, so declare context cleanup before asset cleanup

## Saving prepared shapes

Cooked objects hold prepared collision data in memory. Writing their raw bytes to disk does not create a supported asset file

Save the original points, triangles, or compound description in your application's asset format, then cook and import them when loading. Entasis does not currently provide a versioned file format for cooked objects
