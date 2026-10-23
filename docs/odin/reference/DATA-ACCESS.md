# Data access

Use `body_get` or `static_get` to copy one object's state, direct views to iterate dense body storage, and property tables for application data keyed by physics identity. These state copies are not whole-world checkpoints

For every public declaration, see the [Data access section of the API index](API-INDEX.md#data-access)

## Public value types

The facade exposes zero-copy aliases for stable runtime math and state values, including:

- `Vector2`, `Vector3`, `Vector4`
- `Quaternion`, `Matrix3x3`, `Symmetric3x3`
- `Bounding_Box`, `Rigid_Pose`, `Body_Velocity`, `Body_Inertia`
- `Body_Description`, `Body_State`, `Static_Description`, `Static_State`
- `Collidable`, `Continuous_Detection`, `Collidable_Reference`

These aliases use the runtime layout directly. Snapshot types contain no borrowed pointers into a world

## Active body views

`active_body_view` exposes dense active-body columns without copying or transposing data. This fragment requires an initialized idle `world`, with `entasis` and `core:fmt` imported. Finish the loop before stepping or mutating the world

```odin
view, status := entasis.active_body_view(&world);
if status != .Ok
{
	return;
}

for index in 0 ..< view.count
{
	handle := view.handles[index];
	motion := &view.dynamics[index].motion;
	fmt.println(handle.value, motion.pose.position);
}
```

Sleeping bodies are not part of the active view. Use `body_get` for an individual active or sleeping body snapshot

`active_body_row` assembles pointers for one indexed row without allocation

## Static views

`static_view` exposes dense static records and their reverse handle map

Treat static records as read-only. Use static mutation procedures so broad-phase bounds and awakening policy remain correct

`static_view_row` assembles one row without allocation

## View validity

Every direct body and static view captures one world mutation epoch

`body_view_valid` and `static_view_valid` compare the captured epoch and storage identity against the current world

Reacquire views after:

- `world_step` or manual stage execution
- World clear, capacity growth, or resize
- Body, static, or constraint mutation
- Typed batches or command buffers
- Sleeping migration or awakening
- World destruction

A failed structural attempt may also invalidate a view

Read-only shape inspection and ordinary scene queries do not by themselves relocate body or static storage

Never retain row or column pointers across an invalidation boundary

## Supported direct writes

Advanced integrations may write active-body motion pose and velocity directly through `Body_Dynamics`

Do not directly change:

- Handle arrays
- Collidable shape identity or broad-phase index
- Local inertia without matching world inertia
- Static records

Use handle-based procedures for these changes

After an intentional low-level pose write outside the direct motion path, update broad-phase bounds through the documented body or static bounds procedure

## Property tables

Property tables provide dense caller-owned storage keyed by physics identity

| Table                          | Key space                       |
| ------------------------------ | ------------------------------- |
| `Body_Property_Table(T)`       | Body handle numeric space       |
| `Static_Property_Table(T)`     | Static handle numeric space     |
| `Collidable_Property_Table(T)` | Separate body and static spaces |

Use property tables for collision policy, material IDs, entity IDs, gravity, callback routing, or gameplay tags

Lookup uses direct indexing without hashing or world-state copies

## Property lifecycle

A zero initial capacity is valid. Set operations grow geometrically when needed. Get operations never allocate

Property tables do not receive automatic world removal notifications

1. Set the property after object creation succeeds
2. Remove the physics object
3. Remove the property only for the successfully removed object or prefix
4. Clear or destroy the table according to application ownership

Ready property tables must remain at a stable address and must not be copied

## Generation-protected keys

`body_property_key`, `static_property_key`, and `collidable_property_key` capture table identity, table epoch, slot generation, and property presence

The matching `*_property_get_key` rejects stale keys after removal, clear, destroy, or reinitialization

Updating an existing property preserves its key. Growth preserves keys because a key resolves through the table rather than storing a raw value pointer

## Property threading

- Initialize, grow, set, remove, clear, and destroy on the owning thread
- Read-only get operations may run concurrently only while no mutation or growth can occur
- Returned value pointers become invalid after growth, clear, destroy, or reinitialization
- Property mutation does not change physics execution order by itself

Tables copy `T` as raw value data and do not call element destructors. Release resources owned by a stored value before replacing or removing it

## Buffer pools

Use `buffer_pool_init`, `buffer_pool_clear` and `buffer_pool_destroy` for reusable unmanaged storage. The [pool ownership](WORLD.md#pool-ownership) and [allocator scope](WORLD.md#explicit-allocation-scope) contracts define borrowing, clear and teardown

## Low-level interop borrows

The facade exposes advanced idle-world borrows:

- `world_borrow_simulation`
- `world_borrow_pool`
- `world_borrow_dispatcher`

These procedures transfer no ownership and bypass ordinary facade safety boundaries

Use them only on the owner thread while the world is idle. Do not retain returned pointers across mutation, stepping, clear, or destruction

Broad-phase scheduler tuning and other low-level simulation access remain application responsibilities after borrowing

## Precision and borrowed position views

`Body_Dynamics` and static-view records retain their float layouts in both profiles. In mixed64 these positions are rounded caches. Global accessors and ordered global pose export read authority without importing an unchanged cache

Direct writes to active motion positions remain supported, including shapeless bodies. At the next admitted step or standalone stage entry, the engine validates every changed position first, then imports changed caches by exact f32-to-f64 widening. Untouched caches retain their f64 positions. Invalid changed input returns `Invalid_Argument` before any position import and leaves the cache available for correction. Use global setters for edits smaller than the float cache can represent. A successful nonzero shift invalidates borrowed views

## Global authoring snapshots

Mixed64 exposes `Global_Body_State` and `Global_Static_State` with a `Global_Pose` and the ordinary authoring fields. `body_get_global_state` and `static_get_global_state` copy these values without awakening. `body_add_global_state`, `body_apply_global_state`, `static_add_global_state` and `static_apply_global_state` use ordinary complete-description mutation, including its allocation, waking and target-cancellation behavior

`world_export_global_states` and `world_apply_global_states` operate on a `Global_State_Selection` of caller-owned handle spans and a `Global_State_Snapshot` of disjoint value spans. Each value span must hold at least one entry per selected handle. Both operations process four streams in this order:

1. Body authoring states
2. Static authoring states
3. Built-in absolute constraint targets
4. Queued kinematic body targets

All stream capacities are checked before any record is read or applied. A later record failure returns `Global_State_Counts` for the successful prefixes. Export leaves unwritten output untouched, and apply stops before later records. Duplicate handles execute in input order. Apply is not a transaction across records, and an underlying failed mutation can retain its documented wake side effects. Kinematic targets are applied last because complete body replacement cancels queued targets

Constraint-target entries follow the [supported kinds and matching rules](CONSTRAINTS.md#absolute-targets-and-position-space). `body_get_global_target_state` returns `.Absent` for a missing or expired target, including when body controls are disabled, and requires a kinematic body. `body_apply_global_target_state` clears an `.Absent` target or queues a new `.Present` target. Applying `.Present` requires controls to be enabled. It does not restore the target's previous phase, velocity or step state

Snapshots contain authoring values in the simulation frame at capture. Translate every absolute position consistently before applying them after an origin shift. Shape and constraint handles still refer to live objects in the intended world. Create destination objects and enable their optional controls first. There is no identity remapping or restoration of topology, contact/solver history, sleep counters, callbacks or exact replay state. Capture requires an immutable idle world, and apply requires exclusive idle-world ownership

For poses only, `world_export_global_poses` writes `Global_Pose` values in caller-supplied `Collidable_Reference` order, including sleeping bodies and statics. Its optional `Origin_Frame` adds an application origin in f64. Capacity or frame admission failure writes nothing. A later invalid reference or coordinate overflow returns the valid written prefix. Input/output spans must be disjoint, and returned values do not borrow engine storage
