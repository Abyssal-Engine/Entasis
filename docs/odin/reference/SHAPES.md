# Shapes

Register reusable collision geometry, then reference it from bodies, statics and compound children. Use cooking for prepared hull, mesh and compound assets

For every public declaration, see the [Shapes section of the API index](API-INDEX.md#shapes)

## Shape handles and ownership

A `Shape_Handle` identifies one registered shape in one world

Bodies, statics, and compound children reference registered shapes. `shape_remove` succeeds only when no live world object or compound still references the shape

Shape registry slots may be reused after removal. Treat removed handles as stale and do not persist them across world clear or destruction

`world_clear` removes every registered shape. `world_destroy` releases the complete registry

## Built-in primitive shapes

| Shape    | Builder                            | Main parameters                         |
| -------- | ---------------------------------- | --------------------------------------- |
| Sphere   | `sphere`                           | Radius                                  |
| Box      | `box`, `box_half_extents`          | Full dimensions or half extents         |
| Capsule  | `capsule`, `capsule_half_length`   | Radius and full or half cylinder length |
| Cylinder | `cylinder`, `cylinder_half_length` | Radius and full or half length          |
| Triangle | `triangle`                         | Three local-space vertices              |

Use `shape_validate` before registration when asset validation must be separate from mutation

Register through `shape_add`. Use `shape_add_typed` only for a type already registered through the advanced shape registry

## Inertia and mass

`shape_inertia` computes inertia from a standalone built-in shape description

`shape_registered_inertia` computes inertia from a live registered shape and supports registered complex shapes

Both require positive mass for dynamic-body mass properties. Kinematic bodies use locked inverse inertia and do not need finite dynamic mass

## Shape inspection

| API                      | Use                                                                 |
| ------------------------ | ------------------------------------------------------------------- |
| `shape_type_id`          | Read the public type ID from a handle                               |
| `shape_inspect`          | Return stable metadata for a registered shape                       |
| `shape_bounds`           | Compute local or transformed bounds through the registered type     |
| `shape_ray`              | Test one registered shape directly                                  |
| `shape_borrow_raw`       | Borrow the raw registered shape pointer and type metadata           |
| `shape_borrow_typed`     | Borrow a pointer after checking the expected type                   |
| `shape_remove_recursive` | Remove a shape and recursively release eligible referenced children |

Raw and typed borrows are read-only, owner-thread pointers for use while the world is idle. They transfer no ownership

Shape removal, `world_clear`, and `world_destroy` invalidate them. Shape storage growth or resizing can also relocate a value while its handle remains valid, including when another shape of the same type is added or imported

Reacquire borrowed pointers after shape registration or import, `world_ensure_capacity`, `world_resize`, or any other operation that can resize the shape's storage. An idle world and a valid handle do not guarantee a stable address

## Compounds

Compounds represent the union of their child geometry

`Compound_Builder` binds caller-owned child and mass arrays without allocating hidden builder storage

Use these operations with the builder's caller-owned child and mass storage:

- `compound_builder`
- `compound_center_of_mass`
- `compound_inertia_weighted`
- `compound_inertia_weighted_recenter`
- `compound_build_dynamic`
- `big_compound_build_dynamic`

A normal compound scans its children directly. A big compound stores a bounds tree for larger child counts

Building a registered compound retains its child shapes. Remove the compound before removing a referenced child, or use `shape_remove_recursive` when recursive removal matches application ownership

[`compounds`](../../../examples/headless/compounds/main.odin) shows both compound forms

### Compound hierarchies

Registered compounds and big compounds can contain registered convex shapes, other compounds, big compounds and meshes. Descendants form an immutable shared graph. Registration retains each child and rejects unsupported nonconvex custom children or hierarchy depth above 32. A convex-only compound has depth zero, and each parent containing a nonconvex child adds one level

Collision and query traversal preserve the top-level child identity while descending into nested geometry. Required custom collision and sweep routes must still be registered. Mesh descendants retain surface-intersection semantics. Nesting does not turn a mesh into a filled volume

`shape_remove_recursive` removes the requested root and eligible reachable descendants, retrying shared descendants after their reachable parents are removed. Shapes retained by another body, static or parent remain registered. Caller scratch must cover the reachable set

Assign sensor behavior to a body or static instance through [mixed solid and trigger parts](COLLISION.md#mixed-solid-and-trigger-parts). Shared hierarchy geometry carries no per-instance role or user ID

## Convex hulls

Runtime hull paths accept point clouds and build immutable topology before registration

For asset pipelines, follow the [cook/import flow](../COOKING.md#convex-hull-flow). [`convex_hulls`](../../../examples/headless/convex_hulls/main.odin) applies it to a dynamic body

## Meshes

Meshes store triangles, per-triangle bounds, and an acceleration tree

Collision, queries and sensors use those triangle surfaces, including for closed meshes. An object entirely inside a closed shell need not intersect its surface. Closed-mesh mass and inertia calculations do not change these collision or query semantics

Use the cooking package for prepared import. `shape_import_mesh` is the runtime import boundary for compatible prepared data

Mass helpers distinguish closed-solid and open triangle-soup assumptions:

- `mesh_closed_center_of_mass`
- `mesh_open_center_of_mass`
- `mesh_closed_inertia`
- `mesh_open_inertia`

[`meshes`](../../../examples/headless/meshes/main.odin) shows cooking, import, static registration, ray queries, and dynamic interaction

## Cooked imports

| API                                | Imported asset                                              |
| ---------------------------------- | ----------------------------------------------------------- |
| `shape_import_convex_hull`         | Prepared convex hull data                                   |
| `shape_import_mesh`                | Prepared mesh and bounds tree                               |
| `shape_import_compound`            | Portable child array resolved through world shape handles   |
| `shape_import_big_compound`        | Portable child array with a bounds tree built during import |
| `shape_import_big_compound_cooked` | Prepared big-compound children and tree                     |

`Compound_Child` and `compound_child` describe world-local compound children using registered shape handles

For context-owned cooked values and portable shape slots, use [Odin cooking](../COOKING.md)

## Custom shapes

Custom shape integration has three separate registrations:

1. Shape operations through `Custom_Shape_Registration`
2. Collision routes through `Collision_Task_Registration`
3. Sweep routes through `Sweep_Task_Registration` when world sweeps must support the shape

Allocate IDs with `custom_shape_next_type_id`, configure through `custom_shape_registration`, then call `custom_shape_register` before the first world step

Register every required collision pair and sweep route before creating live content that depends on them. Built-in routes cannot be replaced

Custom callbacks define bounds, inertia, ray, support mapping, and disposal behavior. The world owns a successfully registered custom shape instance until removal or world destruction

[`custom_shape`](../../../examples/headless/custom_shape/main.odin) implements one custom convex shape and its collision route

## Contextual shape callbacks

Use `custom_shape_registration_contextual` and `custom_shape_register_contextual` when shape callbacks need per-world application context. Add instances with `custom_shape_add`. Ordinary bodies, statics, inspection and removal work with the resulting type. Register the type and its collision/sweep routes before stepping

Callbacks use `proc "contextless"` and receive the borrowed `user_context`, original shape payload and a callback-local `Shape_Access`. Bounds, inertia, ray, support, sweep-support and disposal each have a distinct callback type. The constructor defaults sweep support to ordinary support and disposal to a no-op for payloads without nested storage. A manually assembled descriptor must supply all required callbacks

### Storage and lifetime

Follow [contextual registration lifetime](CALLBACKS-STAGES.md#contextual-registration-lifetime). Pointers inside a shape payload also remain application owned. Keep their data valid through instance disposal

Payload alignment must be a power of two no greater than 128 bytes. Invalid registration or allocation failure does not consume the next type ID. Custom application child references are not retained automatically. Built-in compound children use the engine's reference counting

`custom_shape_add_raw` copies an exact registered byte count into aligned instance storage, including from an unaligned input pointer. It rejects built-in type IDs. `custom_shape_data` returns a read-only borrowed payload description. Reacquire it after mutation. `custom_shape_inertia` invokes the instance's registered inertia callback while the world is idle

### Child access and errors

`Shape_Access` provides checked, read-only child operations:

- `shape_access_resolve`
- `shape_access_compute_bounds`, `shape_access_compute_inertia`
- `shape_access_ray_test`
- `shape_access_support`, `shape_access_sweep_support`

Resolved values report the original payload address, size, alignment, type and batch classification. The scope and payload views expire when the callback returns. They must not be retained, used to mutate the registry or shared as another caller's scratch

Concurrent queries invoke callbacks on their own caller threads. Keep callback tables immutable and synchronize mutable shared user data. Failed bounds/inertia/support output is discarded. Failed ray output is a miss with the original error. Failed disposal leaves instance removal retryable. Application side effects inside callbacks are not rolled back

For complete registration and callback examples, see [`contextual_shape_test.odin` in the online full repository](https://github.com/Abyssal-Engine/Entasis/blob/main/tests/public_api/contextual_shape_test.odin)

## Contextual collision and sweep tasks

Use `collision_task_register_contextual` or `sweep_task_register_contextual` when a route needs borrowed `user_context`. Register both shape types first and install required pairs before the first step. Collision descriptors require scalar and wide callbacks. Sweep descriptors require top-level and child callbacks. Invalid or duplicate registration consumes no route, and built-in routes cannot be replaced

Task bindings follow the shared [contextual registration lifetime](CALLBACKS-STAGES.md#contextual-registration-lifetime)

Collision callbacks receive registration data as their first argument. Wide callbacks receive an eight-lane bundle with the registered gather, manifold, flip and scatter conventions. Input/output pointers are borrowed only for that invocation

Sweep callbacks receive registration data separately from the query's child filter and filter data. A flipped route swaps shapes and poses, remaps child indices passed to the filter, then returns normals and child indices in caller order. Child callbacks follow the same convention. Callback failure propagates without unregistering the route

For simultaneous calls, use [independent query contexts](QUERIES.md#independently-owned-query-contexts) and follow [callback synchronization](CALLBACKS-STAGES.md#callback-ownership). Registrations remain immutable while queries execute

## Fixed capacities

Each world has 128 shape type slots. The 9 built-in types occupy IDs 0-8, leaving **119 custom shape types**, assigned IDs 9-127

Collision-task and sweep-task registries each have 128 entries, including built-in registrations. These limits count distinct types and registered routes, not the number of shapes in a scene

Use `custom_shape_next_type_id` to obtain a caller-reserved ID. Register custom types and routes in the same order in worlds that share type identities. Exceeding a registry's capacity fails. It does not replace an existing registration
