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
