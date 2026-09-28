# Profiling and inspection

Inspect completed-step timings, world counts and solver impulses while the world is idle. Results are copied snapshots unless an operation explicitly returns a borrow

For every public declaration, see the [Profiling and inspection section of the API index](API-INDEX.md#profiling-and-inspection)

## Status values

See [shared status handling](../API.md#status-handling) for `Status`, success/failure helpers and static status text

## Diagnostics

`Diagnostic` records optional failure detail without storing or formatting strings

It contains:

- Result status
- `Diagnostic_Operation`
- Operation-specific integer detail

Use `diagnostic_clear` before reuse and `diagnostic_record` when an integration wants to preserve context around a returned status

Diagnostics are caller owned and do not create global last-error state

## Handle sentinels

See [handle sentinels and reuse](../API.md#handles). A representation check does not establish that an object is live

## Version data

Compile-time product data is exposed through:

- `VERSION_MAJOR`
- `VERSION_MINOR`
- `VERSION_PATCH`
- `VERSION_PRERELEASE`
- `VERSION_STRING`
- `CURRENT_VERSION`
- `version_current`
- `ABI_VERSION`

`ABI_VERSION` identifies the stable C ABI generation and is not the product semantic version

## Enable profiling

Set `World_Description.profiling = true` before `world_init`

Profiling adds timestamp reads around simulation stages and is intended for diagnostics and benchmark fixture validation rather than production timing

Use `world_profile_enabled` to confirm world configuration

## Profile snapshots

`world_profile_snapshot` copies the most recently completed profile into caller-owned `Profile_Snapshot`

The snapshot contains:

- Completed stage trace with a fixed capacity of 64 entries
- Cumulative stage invocation counts
- Cumulative stage durations in nanoseconds
- Current world step index

`profile_stage_text` returns one static name for a `Profile_Stage`

The snapshot is pointer free and allocates nothing. Read it on the owner thread while the world is idle

[`profiling_and_state_export`](../../../examples/headless/profiling_and_state_export/main.odin) shows profile collection with body snapshots

## World statistics

`world_stats` returns pointer-free counts for:

- Active and sleeping bodies
- Sleeping islands
- Statics
- Active and sleeping constraints
- Active and inactive collision pairs
- Registered shape instances and type count
- World step index

`world_solver_stats` reports active batch count, fallback batch index, and fallback constraint count

Both allocate nothing and require an idle owner-thread world

## Constraint impulses

Use:

- `constraint_accumulated_impulses`
- `constraint_accumulated_impulse_magnitude_squared`
- `constraint_accumulated_impulse_magnitude`

The scalar array size depends on the constraint type. Caller output capacity is explicit

Raw accumulated-impulse magnitudes mix type-specific solver coordinates and are not force or torque thresholds. Use [typed joint reactions](CONSTRAINTS.md#typed-joint-reactions) for physical per-body loads on watched constraints

`world_scale_active_accumulated_impulses` scales active warmstart data. `world_scale_accumulated_impulses` scales active and sleeping data

These mutation procedures are advanced owner-thread operations and change subsequent solver warmstarting

## Solver contact data

`solver_contact_data` reconstructs one built-in active or sleeping contact constraint into pointer-free `Solver_Contact_Data`

The result can include:

- Contact constraint kind
- Body or static identities
- Contact offsets, depths, and normals
- Feature IDs
- Contact material data
- Accumulated normal and friction impulses

`MAXIMUM_SOLVER_CONTACT_COUNT` and `MAXIMUM_SOLVER_IMPULSE_COUNT` bound the fixed snapshot storage

The procedure supports built-in solver contact layouts. It does not expose registration for arbitrary custom contact constraint accessors

## Debug rendering

Applications render their own debug views using [shape inspection](SHAPES.md#shape-inspection), body poses, contact data and constraint information. Entasis supplies the physics data without owning a renderer

## Timing rules

Inspection APIs read current world or solver state. Call them after a completed step while the world is idle

The inspection results described here are value copies, so they need no retained low-level pointers. Handles inside those copies still follow normal world lifetime and reuse rules
