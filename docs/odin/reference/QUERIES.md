# Queries

Use scene queries to search world collidables, or distance queries to compare supplied shapes and poses. All queries are synchronous. The caller owns results and prevents concurrent world mutation

For every public declaration, see the [Queries section of the API index](API-INDEX.md#queries)

## Query model

Inputs, outputs and callback context remain caller owned. Results are pointer-free values. Their embedded handles follow [world handle lifetime](../API.md#handles)

Queries can inspect dynamic and kinematic bodies normally between simulation steps. Calling `world_step` and then a query on the same thread needs no additional synchronization because both calls finish their work before returning. If the application launches queries on other threads, it must wait for them to finish before stepping or modifying that world

## Filters

`Query_Filter` combines a mobility mask with optional top-level and child callbacks

The zero filter includes every mobility class. `include` is applied first and `exclude` second. Both are mobility-class masks. Reject an individual collidable through the allow callback

Use:

- `query_filter_all` to include every public collidable class
- `query_filter_mobility` to select dynamic, kinematic, static, or combined masks
- `Query_Allow_Proc` for top-level collidable filtering
- `Query_Allow_Child_Proc` for child filtering inside compounds

Callbacks execute synchronously on the query caller thread. Callback context stays valid until the query returns. Callback synchronization and allocation are the caller's responsibility

## Rays

| API                | Result                                          |
| ------------------ | ----------------------------------------------- |
| `ray_cast_any`     | Whether any accepted collidable is hit          |
| `ray_cast_closest` | Closest accepted `Ray_Hit`                      |
| `ray_cast_all`     | Accepted hits written into caller-owned storage |
| `ray_query`        | Advanced collector-based traversal              |

Construct the ray with `ray`

The direction is not normalized automatically. Hit `t` parameterizes `origin + direction * t`. It is a distance only when the direction has unit length

On a miss, `ray_cast_any` returns `false, Ok`, while `ray_cast_closest` returns `Not_Found`. `ray_cast_all` returns the written prefix count and reports `Capacity_Missing` on overflow

`Ray_Hit` contains collidable identity, child index, ray parameter, point, and normal according to the queried shape path

[`ray_queries`](../../../examples/headless/ray_queries/main.odin) shows all three stable ray helpers

## Sweeps

A sweep moves one registered shape through the world

| API                    | Result                                          |
| ---------------------- | ----------------------------------------------- |
| `sweep_closest`        | Closest hit using the stable helper             |
| `sweep_any`            | Whether any accepted hit occurs                 |
| `sweep_all`            | Accepted hits written into caller-owned storage |
| `sweep_query_advanced` | Collector-based sweep using explicit callbacks  |

The swept `Shape_Handle` must remain live for the complete call. World targets are treated as stationary. `sweep_closest` returns `Not_Found` when no impact occurs

`Sweep_Settings` controls sweep convergence. Its zero value selects scale-derived defaults. `Sweep_Hit_State` contains `Miss` and `Hit`

[`sweep_queries`](../../../examples/headless/sweep_queries/main.odin) shows the stable closest path. [`character_controller`](../../../examples/headless/character_controller/main.odin) uses sweeps for a kinematic controller

## Overlaps and volumes

`overlap_all` performs exact shape overlap tests and returns a contact manifold for each accepted collidable

`volume_all` tests broad-phase bounds against an axis-aligned `Bounding_Box`. It does not perform exact shape intersection

Both return the written prefix count and `Capacity_Missing` when the output is too small. The Odin helpers do not return the total required count

Grow application storage and retry against an unchanged world when complete output is required

## Result ordering

All-hit ray, sweep, overlap, and volume results follow traversal or discovery order without a global sort. Order may change with world topology or traversal implementation. Compare contents as sets or multisets when order is irrelevant

This does not change closest-hit selection, any-hit behavior, or input-order batch results

## Heterogeneous batches

`Query` is a fixed-layout tagged query value. Build entries with:

- `query_ray_any`
- `query_ray_closest`
- `query_ray_all`
- `query_sweep_closest`
- `query_overlap_all`
- `query_volume_all`

`Query_Output` selects caller-owned output ranges for all-hit operations. `Query_Result` stores one pointer-free result per input query

Build optional scratch with `query_scratch`, then call `query_batch`

`query_batch`:

- Processes observable results in input order
- Writes one result per input query
- Uses caller-selected subranges for all-hit output
- Attempts every query and returns the first non-`Ok` result status
- Never allocates hidden result arrays

[`batched_queries`](../../../examples/headless/batched_queries/main.odin) shows mixed rays and volume queries

## Scratch and allocation

Built-in closest rays use bounded stack scratch and do not borrow pool buffers

All-hit queries and mixed batches may borrow temporary world-pool buffers and grow the pool when required. Borrowed buffers are returned before the call completes

Pool borrowing mutates shared pool state even when existing capacity is sufficient

## Batch concurrency

This section applies to query calls that the application launches on multiple threads, not to the physics workers coordinated internally by `world_step`

Concurrent Odin `query_batch` calls on the same world are supported only when all of these conditions hold:

- Every query is a callback-free `.Ray_Closest`
- The world has no live custom shapes
- No thread steps or modifies the world until all queries finish
- Every caller has separate input and result storage

Serialize batches containing any other query kind when they share the world pool

Callback-bearing queries and worlds with live custom shapes are outside this concurrency guarantee. The caller must synchronize callback state and world access

For concurrent query families beyond that closest-ray batch path, use [independently owned contexts](#independently-owned-query-contexts)

## Direct collision versus scene query

Use `collision_query` when testing two registered shapes supplied directly by the caller

Use ray, sweep, overlap, or volume queries when traversing current world collidables

Direct shape-pair queries are documented in [collision](COLLISION.md#direct-collision-queries)

## Independently owned query contexts

`Query_Context` binds to a world and owns a private collision batcher and traversal pool. Create it with `query_context_init`, optionally supplying an exclusive borrowed `Buffer_Pool`. Configure capacity with `Query_Context_Description` or `query_context_description_default`. `query_context_reserve` replaces scratch only after successful allocation. Reservation does not guarantee allocation-free execution beyond the selected capacity

Use the `*_with_context` siblings of ray, sweep, overlap, volume, heterogeneous-batch and direct-collision operations. Results, filter order and output-prefix semantics match ordinary queries. Do not run simultaneous calls on the same context. Different contexts may query the same world concurrently while no thread steps or modifies it. The caller coordinates these jobs and any shared callback data. A query context provides private scratch, not a lock on the world

Context-based ray and sweep any-hit calls return `Query_Hit_State` (`.Miss` or `.Hit`) with a status. Existing context-free any-hit calls retain their boolean return contracts

A context may not be used recursively, reserved or destroyed while executing. Custom callbacks must support concurrent reads of their shared data

A context survives steps and clear/repopulation between query intervals and does not retain epoch-local views. Its pool cannot be the world pool, a worker pool or another attached context's pool. A borrowed pool remains alive after context destruction. Destroy every attached context before destroying the world. Reserve/destroy only while the world and context are idle

## Closest-point, distance and separation

These native operations use registered shape handles and caller-supplied world poses. They do not search the whole world or modify it. `collidable_shape_pose` resolves a known body/static, including sleeping bodies, once into a shape/pose value snapshot. Its `_with_context` variant provides the same operation under a caller-exclusive `Query_Context`. Snapshots do not extend shape/handle lifetimes

| Operation             | Result and supported geometry                                                                                                                                   |
| --------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `shape_closest_point` | Input point and closest witness on a convex solid, compound union or two-sided mesh surface. Inside a closed convex solid, both witnesses equal the input       |
| `shape_distance`      | Two closest witnesses and nonnegative distance. Convex and existing Compound/Big_Compound/Mesh leaves are supported, including composite pairs                  |
| `shape_penetration`   | Fixed-orientation convex-pair separation normal and positive depth, or a separated/touching result with zero depth. Both roots must be convex                   |
| `shape_depenetrate`   | A verified correction for convex A against convex/composite B. A full relevant-leaf recheck must succeed. This is not a globally shortest composite translation |

For one shape-pair query:

1. Register the required shapes and keep their handles live throughout the call
2. Use `distance_query_settings_default()` or validated explicit tolerances and iteration limits
3. Reserve distance scratch before penetration or depenetration. Point/distance queries need no reservation
4. Call the selected operation and inspect its `Status` before using the result. Apply a returned correction through the body's normal pose API only if the application wants to move it

Every operation has a `_with_context` counterpart taking the context instead of `World`. Built-in analytic paths and registered native/contextual convex support are available. Unsupported nonconvex custom payloads fail rather than substituting their AABB. Mesh distance is two-sided. Mesh correction applies the registered collision route's sidedness. Neither operation interprets a closed triangle mesh as a filled volume

`Shape_Distance_Result.geometry` contains `state`, `point_a`, `point_b`, `normal`, `distance`, `depth` and numerical `iterations`. Its `child_a`/`child_b` are -1 for convex roots or the stored child/triangle index. Exact equal-distance leaf results use child-index tie breaking. The normal points from B to A, so convex separation translates A by `normal * depth`. Distance-only intersection reports `Intersecting` and does not imply a penetration depth. Touching/inside witnesses need not have a unique normal. `Shape_Correction_Result` contains final `state`, accumulated `translation` and correction `iterations`

For [nested compound hierarchies](SHAPES.md#compound-hierarchies), result child indices identify the top-level child containing the selected leaf. Queries include both solid and sensor parts as geometry and do not alter part or trigger event history

`distance_query_settings_default()` sets finite positive absolute/relative tolerances and a bounded iteration limit. Geometry is tolerance-bounded f32. These f32 entry points round absolute witness positions at large coordinates. Use the [global variants](#high-query-coordinates) when those positions need f64 resolution. The iteration budget bounds each numerical leaf solve and, for depenetration, the number of correction passes. Traversing many leaves is not globally limited to that number of instructions or support calls. A separated result is `Ok`, not `Not_Found`

| Result status      | Handling                                                                                             |
| ------------------ | ---------------------------------------------------------------------------------------------------- |
| `Ok`               | Read the geometric state. Separation is a successful query result                                    |
| `No_Convergence`   | The numerical budget was exhausted or the solve cycled. No successful partial correction is returned |
| `Capacity_Missing` | Reserve sufficient scratch while idle, then retry                                                    |
| Other failure      | Preserve the input-validation, callback or route error. Do not treat it as a geometric miss          |

### Distance scratch and batching

Point/distance queries need no reservation. Before penetration or correction, call `distance_query_reserve(world, distance_query_capacity_default())` or `distance_query_reserve_with_context(context, capacity)` while idle. Default capacity is 128 vertices, 256 faces and 384 edges. Capacity bounds storage separately from iteration limits. These distance-family queries do not allocate scratch while executing. Even an analytic penetration call requires reservation. Equal/smaller reservations reuse current buffers. Genuine failed growth retains the last usable scratch

World distance scratch uses the world pool. Context distance scratch uses the context's private or exclusively borrowed pool. Clear retains capacity and destruction releases it. The [query-context lifetime and concurrency rules](#independently-owned-query-contexts) also apply to distance calls. Queries do not alter trigger or restitution history

`distance_query_batch` and `_with_context` use dedicated `Distance_Query` and `Distance_Query_Result` spans. Initialize each input's settings with `distance_query_settings_default()`. `Closest_Point` uses `point`, `shape_b` and `pose_b`. Other kinds use both shapes/poses. Inputs and outputs must not alias. Output capacity is preflighted before writes. An insufficient span returns `Invalid_Argument`. Every input is attempted in order and receives its own status. The call returns the first failure. Output entries beyond the input count remain untouched

### Native separation example

This fragment requires an initialized idle `world`, a registered `box_shape` and imports for `entasis` and `core:fmt`. The shape's size determines whether the two supplied poses overlap. Check the returned geometric state rather than assuming a penetration

```odin
status := entasis.distance_query_reserve(&world);
if status != .Ok
{
    return;
}
result, query_status := entasis.shape_penetration(
    &world, box_shape, entasis.pose(), box_shape, entasis.pose({1.5, 0, 0}),
);
if query_status != .Ok
{
    return;
}
fmt.println(result.geometry.state, result.geometry.normal, result.geometry.depth);
// move A by normal * depth if desired, then requery to check separation
```

### Early-out geometric overlap

`overlap_any(world, shape, pose, filter)` and `overlap_any_with_context(context, shape, pose, filter)` return `(Overlap_State, Status)`. `Intersecting` requires at least one accepted contact with depth >= 0. A miss is `Separated, Ok`. AABB coincidence, speculative negative depths and callback errors are not successful intersections. A missing task returns `Not_Found`, rather than masquerading as a miss. World poses must be finite and have a unit quaternion

Traversal stops after the first accepted collidable. Convex pairs use the scalar registered route. Composite/mesh pairs finish that parent's normal child reduction before terminating. The operation does not promise to skip every remaining child of that parent. Existing instance/child detection filters apply. Triggers participate as ordinary geometry and their retained event history is not read or changed. No all-hit result list is built

The world variant uses the established world query scratch. Context variants use independently initialized scratch. Reserve appropriate query traversal/child capacity before parallel calls. Insufficient scratch and task failures retain their status. Unlike `overlap_any`, `overlap_all` can report speculative negative-depth contacts. Apply a depth test when its results must represent geometric intersection

### Numerical budgets for custom shapes

Deep penetration involving smooth custom convex support may require more than the default 128 numerical iterations. Supply a larger explicit budget and matching scratch, for example `{512,1024,1536}` capacity and 512 iterations. The query never silently retries or relaxes tolerance. Bounded exhaustion remains `No_Convergence`, not a partial successful translation

## High query coordinates

The following APIs require mixed64 and an initialized, caller-exclusive `Query_Context`. Positions use the current simulation frame, without an application origin added. The world, shapes and callback registrations must remain immutable for the complete call. Independent callers need separate contexts, pools, traversal stacks and output spans

| Family and entry points | Input and result coordinates |
| --- | --- |
| `ray_cast_global_any_with_context`, `ray_cast_global_closest_with_context`, `ray_cast_global_all_with_context` | `World_Precision_Segment.start/end` are f64 endpoints of a closed segment. `Global_Ray_Hit.t` is a fraction in [0,1], and `location` is f64. Normals and child identities retain the shape-query contract |
| `sweep_global_any_with_context`, `sweep_global_closest_with_context`, `sweep_global_all_with_context` | `Global_Pose.position` is f64. Velocity, maximum time, orientation and local shape math remain f32. `Global_Sweep_Hit.sweep.location` is f64. Targets remain stationary, and all-hit returns the first impact per accepted collidable |
| `overlap_global_any_with_context`, `overlap_global_all_with_context` | The query pose has an f64 position. Manifold offsets remain local f32 values relative to that pose. Any-hit requires a nonnegative-depth contact, while all-hit can include speculative contacts |
| `volume_global_all_with_context` | `Global_Bounds` has f64 endpoints and a closed boundary. Results identify conservative broad-phase candidates, including motion/speculative expansion, without exact shape intersection |
| `shape_closest_point_global_with_context`, `shape_distance_global_with_context`, `shape_penetration_global_with_context`, `shape_depenetrate_global_with_context` | Points and pose positions are f64. Both witnesses in `Global_Shape_Distance_Result` are f64. Distance, depth, normal and the correction translation remain local f32 values |
| `collidable_shape_global_pose_with_context` | Resolve one body/static to its shape and authoritative `Global_Pose`, including sleeping objects, without awakening or borrowing a pose pointer |

Global geometry queries subtract positions in f64 before invoking the local-float shape kernels. Ray callbacks receive a target-centered frame, while sweep/overlap callbacks receive a query-centered frame. Distance callbacks receive the first pose or point at the local origin. Child identities and filtering retain their existing meanings. The returned f64 location does not increase the precision of the shape geometry, motion kernel or solver. Apply a depenetration translation to the authoritative position in f64

Before recurring scene queries, allocate an `[]i32` stack using `global_ray_stack_requirement` for rays or `global_bounds_stack_requirement` for sweeps, overlaps and volumes. The two helpers currently report the same requirement, zero for an empty world. Structural growth can increase it. A short stack is rejected before callbacks or output. Queries do not resize caller storage. Reserve context traversal/child storage separately where needed, and use `distance_query_reserve_with_context` before penetration or depenetration. Point and distance queries need no EPA reservation

Closest ray/sweep misses return `Not_Found`. Any-hit ray/sweep misses return `.Miss, Ok`, and any-overlap misses return `.Separated, Ok`. Any/closest failures publish no partial hit. All-hit operations return the valid written prefix on traversal, callback or capacity failure and leave entries beyond that prefix untouched. Empty output is valid for a miss and returns `Capacity_Missing` when a hit needs storage. Distance separation is a successful `Ok` result. `No_Convergence`, unsupported routes and callback failures remain errors, with no partial witness or correction published

### Global batches

`query_batch_global_with_context` accepts `Global_Query` values tagged by `Global_Query_Kind` and writes one `Global_Query_Result` per input. Only the selected payload is read. `Global_Query_Scratch` supplies disjoint ray/sweep/overlap/volume output spans and one reused traversal stack. All-hit payloads select subranges with `Query_Output`. Keep those ranges disjoint when their results must coexist

`distance_query_batch_global_with_context` uses separate `Global_Distance_Query` and `Global_Distance_Query_Result` spans. Initialize each input's settings explicitly. `Closest_Point` uses `point`, `shape_b` and `pose_b`, while other kinds use both shapes/poses. Distance reservation remains explicit and is never performed by a batch

Both batch operations reject insufficient result capacity with `Invalid_Argument` and write nothing on capacity/context admission failure. Once admitted, they attempt every input in order and return the first non-`Ok` status, including a closest miss. Each result preserves its operation's status and valid-prefix rules. Invalid kinds or output ranges fail only that item. Entries beyond the input count remain untouched, and all input, result, output and scratch spans must be disjoint

Query results are caller-owned values in the frame used for the call. Translate absolute endpoints, poses and witnesses consistently, or discard them, before reusing them after an [origin shift](WORLD.md#precision-profiles-and-origin-frames). Relative normals, manifold offsets and correction vectors do not acquire the frame origin
