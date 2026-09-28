# C API

Initialize a world, register shapes and bodies, then step and query it from the owner thread. The [C/C++ getting-started guide](README.md) covers downloads and linking. Use the sections below for resource lifetime and operation behavior

- [Lifetime and errors](#lifetime-and-errors)
- [Worlds and threading](#worlds-and-threading)
- [Shapes, bodies and constraints](#shapes-bodies-and-constraints)
- [Queries and views](#queries-and-views)
- [Collision and events](#collision-and-events)
- [Cooking](#cooking)
- [Advanced callbacks and stages](#advanced-callbacks-and-stages)

## Headers and references

| Header                  | Contents                                                                           | Reference                               |
| ----------------------- | ---------------------------------------------------------------------------------- | --------------------------------------- |
| `entasis/base.h`        | Versions, status, diagnostics, handles, math, and common constants                 | [Base](reference/BASE.md)               |
| `entasis/world.h`       | Allocators, dispatch, world descriptions, lifecycle, stepping, and statistics      | [World](reference/WORLD.md)             |
| `entasis/shapes.h`      | Shape values, registration, inertia, import, inspection, and removal               | [Shapes](reference/SHAPES.md)           |
| `entasis/bodies.h`      | Bodies, statics, snapshots, mutation, sleeping, and batches                        | [Bodies](reference/BODIES.md)           |
| `entasis/collision.h`   | Layers, materials, policy tables, manifolds, and direct pair tests                 | [Collision](reference/COLLISION.md)     |
| `entasis/constraints.h` | Constraint descriptions, lifecycle, impulses, reactions, breaks, and commands    | [Constraints](reference/CONSTRAINTS.md) |
| `entasis/queries.h`     | Rays, sweeps, geometry distance/separation, overlaps, volumes, filters and batches | [Queries](reference/QUERIES.md)         |
| `entasis/events.h`      | Contacts, triggers, mixed parts, draining, and combined fixed stepping             | [Events](reference/EVENTS.md)           |
| `entasis/views.h`       | Dense views, profile snapshots, and solver statistics                              | [Views](reference/VIEWS.md)             |
| `entasis/properties.h`  | Generation-aware body, static, and collidable property tables                      | [Properties](reference/PROPERTIES.md)   |
| `entasis/entasis.h`     | Runtime umbrella including every stable runtime domain                             | All runtime pages                       |
| `entasis/cooking.h`     | Cooking contexts/assets. Includes the runtime umbrella                             | [Cooking](reference/COOKING.md)         |

The root `entasis.h` and `entasis_cooking.h` compatibility headers forward to the namespaced headers

Every domain header compiles independently as C11 and C++20

## Lifetime and errors

### Status and diagnostics

Checked operations return a fixed-width `entasis_status_t`. Descriptor constructors and simple value helpers return their value directly

`entasis_status_text` returns process-lifetime borrowed UTF-8 bytes through an explicit pointer and byte count. NUL termination is not required

Diagnostics are optional caller-owned plain data. Pass `NULL` when operation-specific detail is not needed

There is no hidden mutable error string

### Opaque owner handles

`entasis_world_t`, `entasis_query_context_t`, `entasis_buffer_pool_t`, `entasis_thread_pool_t`, property-table owners, trackers, cooking contexts, and cooked assets are caller-owned handles around Entasis-managed resources

For every opaque owner:

- Zero initialize before its initialization or creation call
- Keep the ready handle at a stable address
- Do not copy, forge, or independently destroy a copied handle
- Destroy on the owner thread while no operation is active
- Treat a successful destroy as clearing the original handle
- Do not use borrowed pointers after owner clear or destruction

`entasis_world_init` owns its pool and any included dispatcher. `entasis_world_init_with_pool` borrows the supplied pool

Destroy every attached world and query context before clearing or destroying a caller-owned pool. A query context cannot share its borrowed pool with a world or another context

### POD descriptions and copies

Primitive shapes, body and static descriptions, constraint descriptions, filters, materials, and query inputs are fixed-layout POD values

The runtime copies descriptions during the call unless the individual reference states that a table or view remains borrowed

Batch inputs are contiguous caller-owned buffers and are forwarded without per-entry C bridge allocation

### Allocators

`entasis_allocator_t` is the single allocator selection. Nondefault tables require both `allocate` and `deallocate`. `reallocate` is optional. NULL or all-null tables select the runtime heap. Callbacks are copied into stable resource-owned storage. The application retains `user_context` until every dependent resource and worker has been destroyed. Returned storage must satisfy the exact requested size and power-of-two alignment. Return NULL on allocation failure. Allocator callbacks cannot reenter Entasis and follow the [callback boundary](#callback-boundary)

Existing initializers select `ENTASIS_ALLOCATION_LEGACY`. This preserves their previous allocator reach, lazy pool metadata growth and legacy unsized child-free semantics. Set `entasis_world_extensions_t.allocation_scope` to `ENTASIS_ALLOCATION_ALL_OWNED` for full world ownership. Pools, thread pools and cooking contexts have corresponding `*_init_extended` functions with an explicit scope. Unknown scopes and incomplete allocator tables are rejected before allocation

| All_Owned storage                               | Owner                                                                                               |
| ----------------------------------------------- | --------------------------------------------------------------------------------------------------- |
| World record, native world, contextual bindings | The world's copied allocator                                                                        |
| Owned simulation pool and private query scratch | Pool blocks, block pointers, returned IDs and Development tracking use the owning world's allocator |
| Included dispatcher                             | Worker backing storage, worker-pool array, each pool, and fixed thread records use its allocator    |
| Cooking context and assets                      | Context/asset records, geometry and temporary pool buffers use the cooking allocator                |
| Imported geometry                               | Copied into the destination world's pool. Source and destination lifetimes are independent          |

Query contexts inherit their world's scope for storage they own. Borrowed pools and external dispatchers retain their independent allocators, policies and lifetimes. A full-mode world neither upgrades nor destroys them. Properties and contact resources likewise retain their independently selected owners. OS-managed thread stacks/kernel objects and memory explicitly allocated by application callbacks are outside Entasis ownership. Allocator callbacks must support simultaneous growth on separate worker pools. A single pool is still caller-exclusive

Every All_Owned free receives the original allocation pointer, requested byte count, alignment and owner. Optional reallocation preserves `min(old_size,new_size)` bytes on success and leaves the original allocation intact on failure. Without it, Entasis allocates, copies, then frees only after success. Resize-to-zero frees the old allocation. Zero-size allocation acquires no storage. Zeroing operations initialize new storage or only the grown tail. No default-heap fallback follows allocation refusal

All_Owned reserves returned-ID capacity and Development tracking before committing pool blocks. Valid returns therefore cannot allocate, even while the allocator continuously refuses new storage. This costs four bytes per reserved slot plus one Development tracking bit, including minimum capacities and geometric headroom. A one-byte bucket backed by a 128 KiB block needs at least 512 KiB of IDs plus 16 KiB of Development tracking. Legacy does not acquire this eager policy. Pool/cooking clear retains metadata and allocator ownership. Destruction releases it. Allocation failure preserves existing live storage and successful batch prefixes, not transactional rollback of a step. Initialized children and started workers are released before the owning allocator record

### ABI compatibility

`ENTASIS_ABI_VERSION` identifies the public binary interface

Breaking an existing exported signature, public structure layout, enum value, calling convention, or required callback contract requires a new ABI generation. Additive symbols and new independent descriptors do not change the old ABI generation

Product semantic versioning is separate. `entasis_version_current` reports the runtime product version and the generated `ENTASIS_VERSION_*` macros describe the headers

The [ABI manifest in the full repository](https://github.com/Abyssal-Engine/Entasis/blob/main/tools/abi/abi_manifest.json) defines the exported interface and generated reference text

### Native representation boundary

No Odin string, slice, procedure value, allocator value, SIMD type, registry or simulation layout crosses the C boundary. Use the C descriptors and callback views for custom extensions, `entasis_allocator_t` for allocation, and the [dispatcher interface](#world-threading) for worker integration. Low-level simulation and registry objects cannot be borrowed through this API

## Worlds and threading

### Minimal lifecycle

This C11 program creates an empty world, advances it for 120 fixed steps and destroys it. Link it with the runtime library. Exit code zero means initialization, stepping and cleanup all succeeded

```c
#include <entasis/entasis.h>

int main(void)
{
    entasis_world_description_t description = entasis_world_description_default();
    entasis_world_t world = {0};
    entasis_diagnostic_t diagnostic = {0};

    if (entasis_world_init(&world, &description, &diagnostic) != ENTASIS_STATUS_OK)
    {
        return 1;
    }

    entasis_status_t status = ENTASIS_STATUS_OK;
    for (int step = 0; step < 120; ++step)
    {
        status = entasis_world_step(&world, 1.0f / 60.0f, &diagnostic);
        if (status != ENTASIS_STATUS_OK)
        {
            break;
        }
    }

    const entasis_status_t destroyed = entasis_world_destroy(&world, NULL);
    if (status != ENTASIS_STATUS_OK)
    {
        return 2;
    }
    return destroyed == ENTASIS_STATUS_OK ? 0 : 3;
}
```

### World threading

Entasis coordinates its physics workers internally. `entasis_world_step` waits for their work to finish before returning. An application that steps the world and then runs queries on the same thread needs no additional synchronization, even when physics uses multiple workers

The read-phase and query-context rules below apply when the application launches its own concurrent query jobs. Entasis does not automatically coordinate those jobs. The application must wait for them to finish before stepping or modifying the same world

All lifecycle and mutation calls on one world are serialized owner-thread operations

Use `entasis_world_begin_read` and `entasis_world_end_read` around a caller-controlled job barrier for concurrent queries. Initialize one `entasis_query_context_t` per simultaneous caller while the world is idle. The contexts provide rays, sweeps, geometric overlaps, AABB volumes, heterogeneous batches and direct shape collisions with independent scratch

Join every reader before ending the phase. During the phase, reject world mutation, stepping, capacity changes, registration, destruction and cooked-asset import. Read-only snapshots and views do not change the shared access state. The phase is a synchronization contract, not a lock: begin/end belong to the owner, never to an executing callback

The ordinary `entasis_query_batch` is also allowed during a read phase only when the complete batch is valid, callback-free closest rays and the world has no active custom shapes. This eligibility is checked before output writes. Other ordinary query entry points remain serialized. Use their context variants for parallel work

A context belongs to one world until destroyed and can have only one active caller. Initialize/reserve/destroy it outside a read phase. Its optional borrowed pool must be caller-exclusive and cannot be a simulation or worker pool. Reservation is a capacity hint, not a hard allocation budget: growth uses only that context's private pool. Contexts survive intervening steps, growth and clear/repopulation. Destroy contexts before their world. Query filter user data must support simultaneous read callbacks when shared across contexts

`entasis_dispatcher_interface_t` is blocking:

- `worker_count` stays fixed while attached or used by a step
- `dispatch` invokes the work callback once for every index in `[0, worker_count)`
- `dispatch` returns only after all work completes
- A non-OK dispatch result means no worker callback began
- `worker_pool` returns one stable worker-exclusive pool
- Dispatcher callbacks cannot reenter the same world or dispatcher

The interface, callback pointers, context, and worker pools stay valid for the complete attachment or step lifetime

### Fixed stepping, sleeping, and impulses

`entasis_fixed_stepper_t` is caller-owned POD accumulator state. Update executes at most `maximum_steps`, retains the fractional remainder, and reports step count plus interpolation alpha

Body activation queries distinguish active, sleeping, and missing objects. Awakening affects the complete connected sleeping island

Impulse calls change velocity directly and do not implicitly awaken a sleeping body

## Shapes, bodies and constraints

### Shape ownership

Registered shapes belong to one world

Bodies, statics, compounds, and big compounds hold references to their registered shapes. `entasis_shape_remove` returns `ENTASIS_STATUS_SHAPE_IN_USE` while a reference remains

Complex imports copy topology into world-owned storage. Source arrays need to remain valid only for the call unless a specific structure is documented as borrowed

`entasis_shape_remove_recursive` uses caller-owned scratch. It reports required capacity before removal and does not partially remove the tree when scratch is too small

### Body, static, and constraint handles

Removed handles return `ENTASIS_STATUS_NOT_FOUND` from checked operations until a numeric slot is reused

Numeric values may be reused. Clear external mappings and properties when removal succeeds

Batch add, apply, and remove operations preserve input order and stop at the first failure. Earlier successful entries remain applied. The returned count identifies that prefix, so update application mappings for it and retry only the remaining entries

Constraint descriptions are selected by `entasis_constraint_type_id_t`. Description size and stride must match the selected type exactly

### Structural command buffers

`entasis_command_buffer_t` is a non-owning view over caller-owned command, status, and result arrays

- Commands execute in exact input order
- Add-result indices must be in range and unique within each result array
- `out_processed` reports the successful prefix
- The failing entry receives its status when status output is supplied
- Later status entries remain untouched
- The bridge does not allocate or copy the command array

Underlying world storage may still grow during preflight. Reserve capacity before latency-sensitive submission

### Optional body controls

`entasis_body_control_configuration_default` returns the size/version-tagged descriptor for `entasis_world_enable_body_control`. A null configuration selects the same defaults. It is copied before allocation. `entasis_body_control_reserve` grows explicit input/target and damping capacity without changing existing body descriptors. `entasis_world_disable_body_control` releases the optional owner

Preparation failure preserves queued inputs. Executed failed work is consumed rather than replayed. Physics is not rolled back. `ENTASIS_BODY_CONTROL_PRESERVE_SLEEP` keeps untouched sleeping inputs pending without accumulating sleep duration. Mutations require exclusive idle access and reject read phases or callback reentry. Getters accept ordinary read phases. Outputs are written only on success. See [Bodies](reference/BODIES.md) for declarations and field layouts

#### Force and torque

`entasis_body_add_force` and `entasis_body_add_torque` accept `ENTASIS_BODY_INPUT_FORCE` or `ENTASIS_BODY_INPUT_ACCELERATION`, with an explicit `ENTASIS_BODY_CONTROL_WAKE` or `ENTASIS_BODY_CONTROL_PRESERVE_SLEEP` policy. Inputs are world-space, finite and accumulated for one integrated step on a dynamic body. Force scales by inverse mass and duration. Torque scales by current world inverse inertia and duration. Acceleration inputs are mass-independent

`entasis_body_add_force_at_position` freezes the submitted world wrench, including its moment about the current center of mass. Zero input does not allocate or awaken. `entasis_body_clear_inputs` cancels unexecuted input without undoing executed physics. For an immediate velocity change, use the impulse functions instead

#### Damping

`entasis_body_set_damping` sets nonnegative inverse-second rates on a dynamic body, with multiplier `exp(-rate * integrated_dt)`. `ENTASIS_BODY_DAMPING_ADDITIONAL` composes after the installed callback and inputs. `ENTASIS_BODY_DAMPING_OVERRIDE` replaces damping only for the recognized native gravity policies. A selected C custom velocity table is opaque and rejects override mode. Zero additional rates restore normal callback behavior. `entasis_body_get_damping` returns zero additional rates when no setting exists

#### Kinematic targets

`entasis_body_set_kinematic_target` copies a finite normalized pose and derives next-step linear/shortest-arc angular velocity without teleporting. Contacts see those velocities. `entasis_body_get_kinematic_target` reads a pending target. Absent or executed targets return `ENTASIS_STATUS_NOT_FOUND`

Executed targets expire. Derived velocity clears before the next untargeted step unless an explicit body update replaced it. `entasis_body_clear_kinematic_target` cancels pending control and clears expired target-derived velocity. Prepared cancellation restores the previous velocity. Replacing an executed target settles its expired derived velocity before the replacement can be cancelled. Explicit caller velocity edits are preserved. This is not collision-aware character movement. The final positional residual tolerance is 1e-4 world units. Standalone predict/collision/solve use a consistent logical duration

#### Axis locks

`entasis_body_axis_lock_default(reference)` creates a size/version-tagged value. Select `ENTASIS_BODY_LOCK_X`, `ENTASIS_BODY_LOCK_Y` and `ENTASIS_BODY_LOCK_Z` bits independently in `linear_axes` and `angular_axes`. `entasis_body_set_axis_lock`, `entasis_body_get_axis_lock` and `entasis_body_clear_axis_lock` manage one-body constraint rows

Linear axes hold the selected world coordinates of `reference.position`. Angular axes hold orientation relative to `reference.orientation`. The default spring uses 30 Hz and damping ratio 1. These are iterative physical constraints with spring/solver error, not exact bitwise immobility or post-solve velocity clamping. Forces, collisions and explicit joints react through the lock impulses. Unselected axes remain free

Set copies before mutation, rejects unknown bits and invalid poses/springs, and nonzero locks require a dynamic body. The body-control access and output rules above apply. A missing lock returns `ENTASIS_STATUS_NOT_FOUND`. Set/clear and disable may awaken sleeping islands. A zero-axis value removes the lock. Locks use the engine's constraint type 48, leaving caller-custom IDs available
### Breakable joints

Call `entasis_world_enable_joint_breaks`, then `entasis_constraint_set_break_limits` for each watched joint. `ENTASIS_JOINT_BREAK_FORCE` and `ENTASIS_JOINT_BREAK_TORQUE` select independent finite, nonnegative thresholds. A joint breaks only when its physical force or torque is strictly greater than the selected limit. Zero metric flags or `entasis_constraint_clear_break_limits` cancel the watch without removing the joint

`entasis_constraint_reaction` returns the last committed watched sample, its completed step/substep and independent peak force/torque. Forces are world-space and torques are about each body's center of mass. Inspect the reaction state and step: sleeping joints do not produce a new reaction from cached impulses, and failed physics leaves the committed sample unchanged. Getters accept explicit read phases. Enabling, reserving, setting, clearing, draining and disabling require idle exclusive access

Successful physics commits crossed joints for removal. `entasis_constraint_break_events_drain` copies immutable events containing the retired handle, watch lifetime, application ID, exceeded metrics and reaction. A short output writes no events. Allocation failure while awakening a sleeping island can leave publication pending. Retry drain or discard without replaying physics. These calls may complete removal and invalidate borrowed world views. Disabling requires pending delivery to be acknowledged

#### Custom reaction providers

After [custom constraint registration](#custom-constraints), use `entasis_constraint_set_reaction_provider` with a separate `entasis_joint_reaction_provider_t`. Set `struct_size` to `sizeof` the descriptor and `struct_version` to 1. The descriptor is copied into the existing per-world binding. Its `user_context` remains borrowed until replacement, clearing, joint-break disable or world destruction. NULL clears the provider. Replacement or clearing is rejected while that type has watches, preserving the prior provider

The callback receives one scalar lane: one to four body poses, borrowed description bytes, at most 32 solved impulses and the actual substep duration. Fill `body_count` entries of the four-slot output with world-space linear impulses and angular impulses about each center of mass. The runtime divides these impulses by duration to obtain force and torque. All callback views expire on return. The [callback boundary](#callback-boundary) applies, including no world reentry. A callback error, unknown status or nonfinite output fails the step without committing reaction or trigger history

#### Combined fixed-step delivery

Use `entasis_joint_break_stepper_update` when fixed stepping must deliver break events with parent triggers, part triggers or contacts. Joint breaks must be enabled. Enabled parent/part subscriptions participate automatically. A NULL contact tracker requires an empty contact output. The function also works with breaks alone and does not require mixed collider parts

Supply separate arrays for each selected stream and consume each returned written prefix once. `entasis_joint_break_update_result_t` reports completed physics steps, written counts, required capacities and pending stream bits. A successfully completed step consumes accumulator time once even if publication or delivery later fails. Correct capacity or allocation failure and retry with `elapsed=0`. Contact tracker storage may also require `entasis_contact_tracker_ensure_capacity`. Do not replay elapsed time or reset history to bypass pending delivery

See [Constraints](reference/CONSTRAINTS.md) for provider, watch and reaction declarations and [Events](reference/EVENTS.md) for the combined stepper and result layout

## Queries and views

### Queries and output storage

Scene query outputs, filters, callback context, all-hit arrays, and batch scratch are caller owned

A short all-hit output buffer receives the available prefix and returns `ENTASIS_STATUS_CAPACITY_MISSING`. For ray, sweep, overlap, and volume all-hit calls, `out_required` is then `out_written + 1`, a lower bound rather than the total hit count. Grow storage and retry against an unchanged world when complete output is required

Result structs are pointer free. Embedded physics handles retain normal lifetime and reuse behavior

All-hit traversal order is not sorted unless the specific query states otherwise

### Closest-point, distance and separation

The functions in `entasis/queries.h` provide the following operations.
Each has an `entasis_query_context_` counterpart accepting an independently owned
query context instead of a world

| Function                        | Contract                                                                                                                                         |
| ------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------ |
| `entasis_shape_closest_point`   | Input point and nearest witness on a convex solid, compound union or two-sided mesh surface. Inside a convex solid the witnesses equal the input |
| `entasis_shape_distance`        | Closest witnesses and nonnegative distance for existing convex and Compound/Big_Compound/Mesh leaves                                             |
| `entasis_shape_penetration`     | Fixed-orientation convex pair separation normal and depth. Both roots must be convex                                                             |
| `entasis_shape_depenetrate`     | Verified correction for convex A against convex/composite B, not a globally shortest compound translation                                        |
| `entasis_collidable_shape_pose` | A known live body's or static's shape and world-pose snapshot. It does not extend shape or handle lifetime                                       |
| `entasis_overlap_any`           | `ENTASIS_OVERLAP_INTERSECTING` only after a nonnegative-depth geometric contact. A miss is `ENTASIS_OVERLAP_SEPARATED` with `OK`                 |

`entasis_shape_distance_result_t.geometry` holds state, both world-space witnesses,
B-to-A normal, nonnegative distance, penetration depth and numerical iterations.
`child_a/child_b` are -1 for convex roots or the stored child/triangle index.
For convex penetration, move A by `normal * depth`. Distance-only intersection
reports `ENTASIS_DISTANCE_INTERSECTING` without claiming a penetration depth.
`entasis_shape_correction_result_t` instead holds the final state, accumulated
translation and correction-pass count. Exact-distance composite ties use child
indices. Witnesses and normals need not be unique at touching/inside points

A separated result is `OK`, not `NOT_FOUND`. Numerical exhaustion returns
`ENTASIS_STATUS_NO_CONVERGENCE`. Scratch exhaustion returns `CAPACITY_MISSING`.
Other invalid-input, handle, callback and route errors remain distinct. A failed
scalar operation clears its output to an unresolved state, not a successful
partial correction. Settings pointers may be NULL to select documented defaults.
For a batch, initialize each item's settings explicitly using
`entasis_distance_query_settings_default()`

The default is absolute/relative tolerance 1e-5 and 128 iterations. Tolerances
are positive finite values. Relative tolerance must be below one. The iteration
limit is 1..4096 and applies per numerical leaf solve and per correction pass,
not to the total amount of composite traversal. Deep smooth custom support can
require a larger explicit budget and scratch. No query silently increases its
budget, loosens tolerance or returns success on exhaustion. Unsupported custom
nonconvex payloads fail. Their bounds are not treated as geometry. Mesh distance
is two-sided surface distance, while mesh correction uses registered collision
sidedness. Closed-mesh volume containment is not provided

Point and distance calls need no reservation. Before penetration or correction,
call `entasis_distance_query_reserve(world, capacity, diagnostic)` while idle, or
`entasis_query_context_distance_query_reserve` for each selected context. NULL
capacity selects 128 vertices, 256 faces and 384 edges. Query calls do not allocate
geometry storage. Equal/smaller reservations reuse it, and failed growth retains
the last usable buffers. Scratch retains its [allocator ownership](#allocators).
Apply the [query-context and read-phase rules](#world-threading). Callbacks cannot
mutate the world or reenter their own context

`entasis_distance_query_batch` and `entasis_query_context_distance_query_batch`
accept dedicated contiguous input/result spans without extending the existing
heterogeneous query union. They preflight output capacity, alignment, address
arithmetic and disjoint spans before writes. Nonempty spans must be valid caller
storage. Inputs/results/diagnostics must not alias. Every input is attempted in
order, each result carries its status, and the call returns the first failure.
Entries after `count` remain untouched. A zero-count call accepts NULL spans but
still requires a live owner and valid access state. `CLOSEST_POINT` uses `point`
and `shape_b/pose_b`. Other kinds use both shapes/poses

Overlap-any applies existing instance/child filters, includes trigger geometry
by default and never changes trigger/restitution history. It does not allocate
an all-hit list. It stops after the first accepted collidable. Composite/mesh
children finish that parent's existing reduction before termination. Existing
world/private-context collision and traversal scratch must have sufficient
capacity. A callback's `NOT_FOUND` or capacity failure is propagated, not converted
to a successful miss or hit

#### C query example

This complete C11 program registers a unit-radius sphere and compares two instances one unit apart. Link with the runtime library. Success prints a penetrating state and depth near `1`, then returns zero after cleanup

```c
#include <entasis/entasis.h>
#include <stdio.h>

int main(void)
{
    entasis_world_t world = {0};
    entasis_world_description_t description = entasis_world_description_default();
    description.threading.worker_count = 1;
    entasis_status_t status = entasis_world_init(&world, &description, NULL);
    if (status != ENTASIS_STATUS_OK)
    {
        return 1;
    }
    const entasis_sphere_t sphere = entasis_sphere(1);
    entasis_shape_handle_t shape = {0};
    status = entasis_shape_add(&world, ENTASIS_SHAPE_TYPE_SPHERE, &sphere, &shape, NULL);
    if (status == ENTASIS_STATUS_OK)
    {
        status = entasis_distance_query_reserve(&world, NULL, NULL);
    }
    entasis_shape_distance_result_t result = {0};
    if (status == ENTASIS_STATUS_OK)
    {
        const entasis_quaternion_t identity = {0, 0, 0, 1};
        const entasis_rigid_pose_t a = entasis_pose((entasis_vector3_t){0,0,0}, identity);
        const entasis_rigid_pose_t b = entasis_pose((entasis_vector3_t){1,0,0}, identity);
        status = entasis_shape_penetration(&world, shape, a, shape, b, NULL, &result, NULL);
    }
    if (status == ENTASIS_STATUS_OK)
    {
        printf("state=%u depth=%g\n", (unsigned)result.geometry.state, (double)result.geometry.depth);
        /* for Penetrating, translate A by geometry.normal * geometry.depth */
    }
    const entasis_status_t destroyed = entasis_world_destroy(&world, NULL);
    return status == ENTASIS_STATUS_OK && destroyed == ENTASIS_STATUS_OK ? 0 : 1;
}
```

Use this example with C11. The same calls are available from C++20 with its normal
aggregate initialization syntax

### Views and invalidation

Active-body and static views borrow dense world storage

Reacquire views after stepping, body/static/constraint mutation, sleeping migration, awakening, capacity changes or clear. The epoch check can detect invalidation, but does not make an old pointer safe to use. World destruction ends every view's lifetime

Do not retain row or column pointers across an invalidation boundary

### Property tables

Body, static, and collidable property tables own dense caller-visible storage until destroyed

Property tables do not subscribe to world removal. Remove entries after the corresponding physics removal succeeds

Generation-aware keys reject stale table entries after removal, clear, destroy, or reinitialization

A table attached to a live policy or callback context must stay at a stable address until the world no longer uses it

## Collision and events

### Collision policy and events

`entasis_layer_material_policy_t` borrows its collision property table, optional layer matrix, material array, callback pointers, and context

These remain valid until world destruction. Mutate them only while the world is idle and according to the table-specific rules

Contact trackers and user tables are caller-owned resources. Drain events into caller-owned arrays and resolve application identity before physics handles are removed or reused

### Trigger colliders

Include `entasis/entasis.h` or `entasis/events.h`. Use
`entasis_trigger_configuration_default` for versioned capacities,
then `entasis_world_enable_triggers`. Resolve an existing instance without bit
packing using `entasis_body_collidable_reference` or
`entasis_static_collidable_reference` and set its mode. This fragment assumes an initialized idle `world`, enabled triggers and an existing static handle named `sensor`. It belongs in an application function whose error exit cleans up resources it owns

```c
entasis_collidable_reference_t reference;
entasis_trigger_settings_t settings = {0};
settings.user_id = 1001;
if (entasis_static_collidable_reference(&world, sensor, &reference, NULL) != ENTASIS_STATUS_OK)
    return 1;
if (entasis_trigger_set(&world, reference, &settings, NULL) != ENTASIS_STATUS_OK)
    return 1;
```

Settings are copied. `stay` opts into Stay. `static_static` opts into static/static
pairs. Events use `ENTASIS_TRIGGER_ENTER`, `ENTASIS_TRIGGER_STAY` and
`ENTASIS_TRIGGER_EXIT`, with `ENTASIS_TRIGGER_PAIR_A`, `ENTASIS_TRIGGER_PAIR_B`
and `ENTASIS_TRIGGER_PAIR_STAY_REQUESTED` flags. A pair is retained without a solver
constraint. Body integration and explicit joints are unaffected by sensor mode

After a successful step on that world, drain the resulting events. This is an operation fragment, not a separate program. Inspect `status` before consuming the output

```c
entasis_trigger_event_t events[64];
uint64_t written = 0, required = 0;
entasis_status_t status = entasis_trigger_events_drain(
    &world, events, 64, &written, &required, NULL);
/* CAPACITY_MISSING writes no elements. allocate required elements and retry
   this drain, not the completed physics step */
```

A nonempty pending batch blocks the next tracked step before physics work.
`entasis_trigger_stepper_update` reports consumed steps even on notification
backpressure. Its `required` field is zero after all batches are delivered. On
output exhaustion it is the size of the pending batch, not a request to replay
already consumed steps. Any `events_written` prefix is valid and already delivered.
`entasis_trigger_events_discard` explicitly acknowledges a batch.
`entasis_trigger_reset_history` discards history and recovers invalid custom-step
notification state. An empty batch does not stall. Snapshot getters are allowed
inside an existing read phase. Mutations, reserve, drain and stepping are not.
Application callbacks cannot reenter trigger mutation

Configured candidate capacity is processed through bounded scratch batches, not
limited by a hidden one-batch parent count. Actual candidate/pair/child exhaustion
and callback failures propagate normally: no partial event batch is published,
and committed overlaps remain available. Grow capacity with
`entasis_trigger_reserve` for a capacity failure, or correct the callback failure,
before recovery. Earlier physics stages of a failed step are not rolled back

Triggers sample during discrete collision phases. They do not detect continuous
crossing or closed-mesh containment. Sleeping overlap history persists without
waking visitors, and moved static triggers discover sleeping visitors. A parent
pair is deduplicated across compound children and mesh triangles. Existing
detection filters still apply. Sensor pairs create no contact constraints. New static triggers should be added with `ENTASIS_AWAKENING_NONE`
before setting mode. Use copied lifetime tokens and epoch for delayed Exit events,
not a freshly resolved raw handle

Sleeping after integration, including after the final collision stage of a
custom timestep, schedules final geometry for the next trigger sample without
waking the body. Completed-step publication preserves these late notifications.
Supplemental dirty discovery does not duplicate ordinary active pairs. An
unchanged dormant pair invokes no repeated trigger geometry callback

`entasis_trigger_set_user_id` assigns the captured ID of a visitor without changing
its solid/sensor mode. It uses the same per-world lifetime storage, not a global
identity service. A pending event batch keeps the ID sampled before the edit

### Mixed collider parts

Enable triggers, initialize `entasis_collider_part_configuration_default`, select `ENTASIS_COLLIDER_PART_EVENTS_ENABLED` when part notifications are needed, then call `entasis_collider_parts_reserve`. Subscription remains fixed for that storage lifetime. `entasis_collider_part_set/get/remove` manage per-instance top-level overrides. Unspecified parts remain solid, and shared shape data is unchanged. Whole-collidable trigger mode and part overrides are mutually exclusive

Use child index zero for a convex or mesh root. Compound and Big Compound overrides address top-level children, and every nested descendant inherits that child's role. Mesh triangles remain one part. Supported hierarchy depth is bounded to 32 nonconvex descendant levels. Native solid leaves continue physical contact response while sensor leaves produce events without solver contacts

Part identities contain instance, shape incarnation and part serial. Role edits preserve the serial. Removal/rebinding and shape replacement retire it. Events retain copied identities and application IDs after deletion, so a captured raw handle is not a durable lookup. `entasis_collider_parts` lists only configured overrides in child order. `entasis_trigger_part_overlaps` reads committed intersections. Both accept read phases and write no records when output is short

Parent events reduce all qualifying parts to one collidable pair. Part events retain individual pairs. Drain or discard each stream independently. `entasis_collider_part_stepper_update` delivers both streams, and its `_with_contacts` sibling also drains a bound contact tracker before the next step. Part subscription is required by these two entry points. A delivered contact prefix remains delivered if trigger output is short. Retry with `elapsed=0`, preserving each returned prefix. Use the [combined joint-break stepper](#combined-fixed-step-delivery) when breaks are selected too

Exact flags, identities, counts and output-admission rules are in [Events](reference/EVENTS.md)

### Restitution (bounciness)

`entasis_restitution_configuration_default()` initializes the size/version-
tagged configuration. Set `fallback.coefficient` in `[0, 1]`, then call
`entasis_world_enable_restitution`. The fallback threshold defaults to `1` world
unit/second

| Operation                                                                            | Contract                                                                                                                                          |
| ------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------- |
| `entasis_world_enable_restitution` / `entasis_world_disable_restitution`             | Enable copied settings on an idle world. Disable frees optional state. A null configuration selects defaults, with zero coefficient               |
| `entasis_restitution_set` / `entasis_restitution_get` / `entasis_restitution_remove` | Set an instance override, read the effective setting, or remove only the override. Removing an absent override returns `ENTASIS_STATUS_NOT_FOUND` |
| `entasis_restitution_reserve`                                                        | Grow settings/history capacity without changing settings or shrinking existing storage                                                            |

The optional `entasis_restitution_combine_fn` receives copied scalar settings
and an output initialized to the maximum coefficient/threshold. Its borrowed
user context must outlive use. Output must remain finite and within the documented
bounds. It runs on the owner thread before solver workers, cannot reenter world
operations, and follows the [callback boundary](#callback-boundary). The callback
table is copied when enabled

Getters may run within an explicit read phase. Enable/disable, reserve, set and
remove require exclusive Ready access and are rejected from callbacks or read
jobs. `ENTASIS_ALLOCATION_ALL_OWNED` applies to the optional native state and copied
C callback binding. A binding is allocated only when a custom combine callback
is selected. Destruction frees children before their allocator/callback owner

Impact speed is captured after velocity integration and before warmstart.
Approach speed may be retained while speculative contacts brake an incoming
body, but a bounce target is applied only at nonnegative geometric depth and
above the speed threshold. The target is the maximum of penetration recovery
and restitution speed, not their sum. Repeated iterations do not repeat the
impact. Separation and renewed approach can arm another impact. Triggers and
rejected contacts do not bounce. Incompatible custom contact accessors return
`ENTASIS_STATUS_INVALID_DESCRIPTION`. Custom constraints retain their authored response

Complete successful steps commit impact history once. Failed custom steps
discard pending history without rolling back executed physics. Standalone solve
stages commit on success. Multiple solve stages in one custom timestep share
pending history. Settings changes do not wake sleepers. These are discrete
contact-response semantics, not continuous time-of-impact restitution

Disabled worlds allocate no restitution storage. An enabled zero-coefficient
world allocates no packed impact targets. Selected work buffers may grow before
worker dispatch and are reused after warming

## Cooking

The cooking library is separate and depends on the matching runtime library. A private layout check rejects imports between incompatible runtime and cooking builds before accessing world storage. Build and ship them together. Matching public ABI versions alone do not establish this private compatibility

A cooking context owns temporary cooking storage. Every cooked asset belongs to the context that created it

- Destroy cooked assets before clearing or destroying their context
- Import copies prepared topology into the destination world
- Import does not consume the cooked asset
- Imported shapes remain valid after the cooked asset and context are destroyed
- Compound imports borrow shape-slot tables only for the duration of the call

Cooked objects are in-memory resources. Entasis has no versioned file format for saving them. Retain source geometry or your own asset format and cook it when loading

## Advanced callbacks and stages

### Callback boundary

Collision, material, query, and dispatcher callbacks use explicit C calling conventions and caller-owned context

No Odin panic, C++ exception, `longjmp` or foreign unwind may cross the ABI boundary. Callback-local pointers expire on return

World-step collision callbacks may run on physics workers. Query callbacks run synchronously as described by the query reference

Use `entasis_world_init_extended` (or its borrowed-pool sibling) with `entasis_world_extensions_t` to select velocity and contact callbacks. Descriptors are copied into world-owned storage during initialization. Callback user data remains borrowed until disposal. Supplying both an explicit convenience policy and the corresponding extension callback table is rejected. Missing required functions fail before resource callbacks execute. Successfully initialized callback resources are disposed exactly once. Failed initialization does not trigger disposal for that failed resource

Velocity callbacks receive aligned eight-lane scalar-array views through pointers, an active mask, worker index and stable body handles. Do not recover body identity through ordinary world getters during a step. Pose/inertia/dt are read-only, velocity is writable, and inactive lanes remain masked. One foreign call corresponds to one native SIMD bundle, not one body. No native vector aggregate crosses the foreign ABI by value. Unselected extensions leave the existing native gravity and narrow-phase paths intact

Contact callbacks support speculative-margin edits, pair and child filtering, parent/child manifold edits, material edits and optional contact-constraint selection. These callbacks cannot reenter ordinary world operations

### Custom shape payload callbacks

The C API provides custom-shape registration and instance operations, with bounds, inertia, ray, support, sweep-support and optional per-instance disposal callbacks. Start with `entasis_custom_shape_registration_default`, fill the exact payload size/alignment and required functions, then call `entasis_custom_shape_register` before the first successful step. The registration snapshots the descriptor before invoking application allocation callbacks, then stores its copied table. Its `user_context` remains borrowed until world destruction. Independent worlds may reuse the same numeric type ID with different bindings. Null sweep-support selects ordinary support. Null disposal is appropriate only when payloads own no application resources

`entasis_custom_shape_add` copies exactly the registered number of bytes into aligned world-owned storage. Input bytes need not be aligned. Stored alignment must be a power of two from 1 through 128. Successful insertion transfers responsibility for resources described by those bytes to the instance's disposal callback. Failure leaves them with the caller. `entasis_custom_shape_get` copies a shallow payload snapshot and reports its required size. Insufficient capacity writes no payload prefix. This snapshot does not transfer ownership. `entasis_custom_shape_inertia` invokes the registered instance callback while the C world is exclusive

Callbacks receive `entasis_shape_access_t` only for that invocation. Use its checked read-only bounds/inertia/ray/support helpers for built-in or registered child shapes. `entasis_shape_access_custom_data` exposes a custom child's original bytes, size, alignment and type. Built-in private container bytes are not exposed. The scope and its views must not be retained, forged, freed, mutated or moved to another thread. Post-return scope use violates lifetime and is not a supported stale-handle query. No ordinary world getter or mutation may be reentered from the callback. During an explicit read phase, independent query contexts may invoke these callbacks concurrently. Synchronize application data accordingly

Builtin compound/big-compound child handles retain custom instances normally. Arbitrary child handles embedded in application payload bytes are not automatically retained. Manage those application references and avoid cyclic callback delegation. Dispose only the application's nested resources, never the supplied pool-owned payload address. Registrations survive `entasis_world_clear`. Teardown releases their copied tables after all instance disposal callbacks have completed

Shape registration alone does not install pair routes. Register each required custom route explicitly using the task interfaces below. C custom constraints are described below

#### Custom collision and sweep tasks

`entasis_collision_task_registration_default` and `entasis_collision_task_register` install scalar and eight-lane convex-result callbacks for a pair of registered types. `entasis_sweep_task_registration_default` and `entasis_sweep_task_register` install top-level and child sweep callbacks. Register before the first step. Built-in/duplicate routes cannot be replaced. Tables are copied before output writes and application allocation callbacks. The application retains ownership of `user_context`. Bindings survive world clear/reuse and are freed after successful native teardown. A failed registration consumes no route

Wide collision input exposes pointer-based aligned views with one foreign invocation per existing bundle, never one per body. `count` is 1 through 8. Output holds up to four contacts per lane. Each existence mask is exactly -1 or 0, and inactive lanes must have zero masks. Sparse contact slots are allowed. Existing contacts require finite offsets, depths and normals. Callback errors and malformed results are rejected before the native manifold writer. Runtime conversion is field-wise. The C API does not expose native bundle/cache layouts. Built-in triangle payloads, including mesh children, are materialized from the native packed lanes into callback-local read-only scalar storage. Their pointers remain valid for the callback only. This adapter is selected at registration. Native kernels and non-triangle C routes do not pay for its temporary arrays

Every invocation receives a callback-local `entasis_task_access_t`. `entasis_task_shape_access` provides the existing read-only shape operations. `entasis_task_allow_child` calls the current query filter with its own user data, separate from registration data. Registered route flipping remains the native owner's responsibility. Child callbacks have no additional filter because native traversal already performed child filtering

`entasis_task_collide_convex` invokes an existing registered scalar convex route. Direct recursion into the current collision task is rejected. `entasis_task_sweep_convex` invokes the native built-in convex distance helper for sphere through convex-hull payloads, with parent and local poses preserved. It does not redispatch the same custom sweep. Custom geometry can provide its own sweep algorithm or explicitly convert to a supported built-in representation. It must not disguise arbitrary custom bytes as built-in geometry. Neither helper enters ordinary world queries or borrows mutable world scratch

Task scopes and views follow the [shape-access lifetime rules](#custom-shape-payload-callbacks) and [callback boundary](#callback-boundary). Helper calls are same-thread and non-reentrant through the same scope. Callback chains must not contain indirect cycles. Helper input and output storage must not overlap. Apply the [world threading rules](#world-threading) to application data shared across workers or query contexts

`entasis_compound_task_registration_default` and `entasis_collision_task_compound_register` expose the existing callback-free native compound task kinds separately from convex callback tables. The default descriptor has version 1, invalid shape IDs, batch size 16, kind `ENTASIS_COLLISION_TASK_CONVEX_COMPOUND`, and `SUBTASK_GENERATOR | CHILD_ORDER` capabilities. Register the required child-pair callbacks separately. A parent route does not manufacture missing leaf routes

`CONVEX_COMPOUND` requires a registered convex type on side A and an actual built-in Compound, Big_Compound or Mesh on side B. Queries may use either pair order. `COMPOUND_PAIR` requires actual built-in compound storage on both sides. Those built-in pair routes are already installed and cannot be replaced. Giving application payloads a Compound classification does not make them compatible with native compound storage. Such registrations fail rather than reinterpreting the bytes

Batch size is 1 through 32. Allowed capabilities are `SUBTASK_GENERATOR` (required), `CHILD_ORDER` and `MESH_REDUCTION`. Set the last flag for mesh reduction as in the native descriptor. Invalid/high bits are rejected before narrowing. Compound routes use the fixed task registry without an additional binding allocation and follow the registration timing, route protection and lifetime rules above

### Custom constraints

Use `entasis_custom_constraint_next_type_id`, initialize `entasis_custom_constraint_registration_t` with `entasis_custom_constraint_registration_default`, then call `entasis_custom_constraint_register` before adding any constraints or stepping. IDs 56 through 63 are world-local registrations, not limits on instance count. Descriptors and callback pointers are copied before invoking the application allocator. `user_context` stays caller-owned until successful world destruction. Clear/reuse retains registrations

Descriptions are tightly packed `float` data, positive multiples of four bytes and at most 256 bytes. Prestep storage has exactly eight times the scalar description bytes. Impulse storage is a positive multiple of 32 bytes with at most 32 eight-lane fields, matching native sleeping storage. The registration declares arity 1 through 4 and per-body initial/solve access masks. Unused mask entries must be zero. Missing validation/kernel callbacks and unknown mask bits are errors. A NULL incremental callback reuses the main kernel

`entasis_constraint_kernel_view_t` exposes one masked native SIMD invocation. Its body views contain read-only position/orientation/inertia pointers and independently writable velocity pointers. Undeclared fields are NULL. The field targets, prestep, impulses and active mask are 32-byte aligned. Initial access applies to prestep, warmstart and incremental update. Solve access applies to velocity iterations. Every pointer expires on return. Leave inactive lanes unchanged, including prestep and impulses. Sequential fallback executes one active lane at a time in the native ordering when constraints share bodies. Validation runs on add, apply and sleeping-island awakening. It returns an ordinary status, while kernels do not return an error or support unwinding. Callback state must support worker concurrency. No ordinary world API reentry is allowed from these callbacks

`entasis_custom_constraint_add/get/apply` use the native ownership, transfer and validation paths. Caller scalar storage must be four-byte aligned and exactly the registered size. Their batch siblings accept strided descriptions and flattened body handles without allocating adapter arrays. Nonempty strides are at least the scalar size and divisible by four. Spans and products are checked before pointer arithmetic. `out_completed` is required. Preflight errors complete zero items. Execution stops at the first failing item and retains its successful prefix. A failed add writes an invalid handle at that slot and leaves subsequent slots untouched. Empty batches accept NULL buffers but still require a live world, valid registration and the appropriate access state. Inputs, outputs and the completion counter must not overlap. Existing constraint removal, inspection and impulse APIs also support these handles. Read-only get operations can run in a caller-synchronized read phase. Mutation cannot

Registration stores one copied C record and one native contextual binding under the world's allocation scope. It adds no per-body, per-constraint or per-query metadata. `ENTASIS_ALLOCATION_ALL_OWNED` releases these records with exact allocation metadata

### Stages and timestep control

The `entasis_world_stage_*` functions expose sleeping, bound prediction, collision detection, solving and optimization. Individual stages invalidate affected world views but neither increment the completed-step counter nor automatically emit completion callbacks

A custom timestep callback receives an `entasis_step_scope_t` value token. Use only its scope-stage operations while the world is exclusive, and explicitly report completion where required. Nested scope operations, ordinary world mutation and recursive stepping are rejected. A saved token is rejected after callback return while its world remains alive. Using it after world destruction violates handle lifetime. Successful custom callback completion advances the step index once. An error leaves that counter unchanged but does not undo stages already executed

The optional substep scheduler selects positive iteration counts. Nonpositive returns use the configured fixed count. There is no configurable broad-phase scheduler callback

The three `entasis_static_*_filtered` operations expose caller-selected sleeping-island awakening. The filter receives stable body handles and is borrowed only for that call
