# Callbacks and stages

Use built-in policies for gravity and collision filtering. Raw callbacks and custom timesteppers expose finer control over simulation work

For every public declaration, see the [Callbacks and stages section of the API index](API-INDEX.md#callbacks-and-stages)

## Callback ownership

Installed callback tables are copied by value. Their context and any referenced application storage remain borrowed for the complete world lifetime

Keep every context at a stable address from `world_init` until `world_destroy` completes. Stop external jobs using the same context before destruction

Callbacks may run on physics workers. Callback code must synchronize caller-owned shared state and must not perform same-world lifecycle or structural mutation

Temporary pointers supplied to a callback are valid only for that callback invocation

## Contextual registration lifetime

Contextual shape, collision/sweep-task and constraint registrations copy callback addresses and the context pointer into world-owned bindings. Application data remains borrowed until `world_destroy` returns. Removing instances or calling `world_clear` retains registrations. Destruction releases bindings after simulation teardown, including in worlds with a borrowed pool. Register before the first step, plus any earlier insertion restriction documented by the type. Failed registration publishes no binding or ID/route change

## Built-in narrow-phase policies

Use built-in policy builders before raw callback tables when possible

| Policy                                      | Builder                                                   |
| ------------------------------------------- | --------------------------------------------------------- |
| Allow every pair with one constant material | `default_narrow_policy`, `narrow_policy_default`          |
| Collision layers, masks, and material table | `layer_material_policy`, `narrow_policy_layers_materials` |

`narrow_policy_default_stored_material` retrieves the stored material only when a callback table exactly matches the default policy builder

Install the resulting table through `world_description_set_callbacks`

## Built-in pose policies

| Policy                                                 | Context builder            | Callback builder           |
| ------------------------------------------------------ | -------------------------- | -------------------------- |
| Uniform gravity and damping                            | `uniform_gravity_policy`   | `pose_policy_uniform`      |
| Uniform gravity with explicit angular integration mode | `uniform_gravity_policy`   | `pose_policy_uniform_mode` |
| Inverse-square radial gravity                          | `planetary_gravity_policy` | `pose_policy_planetary`    |
| Per-body gravity from a property table                 | `per_body_gravity_policy`  | `pose_policy_per_body`     |

The policy context stays caller owned and must outlive the world

[`per_body_gravity`](../../../examples/headless/per_body_gravity/main.odin), [`planetary_gravity`](../../../examples/headless/planetary_gravity/main.odin), and [`gyroscope`](../../../examples/headless/gyroscope/main.odin) show the built-in variants

## Raw callback tables

`Narrow_Callbacks` and `Pose_Callbacks` expose the runtime callback layout directly

Use raw callbacks only when the built-in policies cannot express the required behavior. The application then owns:

- Pair and child filtering behavior
- Contact material selection and manifold configuration
- Pose preparation and velocity integration behavior
- Thread safety of callback context
- Allocation and synchronization cost inside callbacks

`Callback_Table_State` distinguishes empty and configured tables where the facade needs to validate installation state

## Stage-completion callbacks

`Timestep_Callbacks` receives stable completion points from the default timestepper

Build the table with `timestep_callbacks` and install it through `world_description_set_timestep`

A stage callback uses the contextless calling convention and receives explicit caller context. Entasis does not allocate to invoke it. Application callback code determines its own allocation cost

Do not call owner-thread structural operations from the callback. Use it to observe stage completion or publish application work that does not reenter the same world

## Custom timestepper

`Timestepper` lets an advanced integration replace the complete default step sequence

Build it with `timestepper` and install it through `world_description_set_timestep`

A zero custom timestepper keeps the default stage order. A custom timestepper assumes responsibility for calling the required stage operations in a valid order and for preserving callback and dispatcher contracts

The custom callback receives a low-level `Simulation` in the `Stepping` state. Use the low-level simulation stages on that supplied simulation. The facade `world_stage_*` procedures below require an idle world and cannot be called from the custom timestep callback

## Manual stages

The facade exposes these stage procedures:

1. `world_stage_sleep`
2. `world_stage_predict_bounds`
3. `world_stage_collision_detection`
4. `world_stage_solve`
5. `world_stage_optimize`

Each procedure requires a ready idle world and may use the initialized dispatcher or an explicit dispatcher

Sleep, predict, collision, and solve invalidate direct views. Optimize performs incremental solver and tree maintenance

Manual stages let the application choose the sequence explicitly. Ordinary applications should call `world_step`

## Reentrancy

Do not reenter the same world from:

- Narrow-phase callbacks
- Pose callbacks
- Stage callbacks
- Custom timestep callbacks
- External dispatcher work procedures
- Custom shape collision or sweep callbacks
- Custom constraint kernels

Record intent into caller-owned worker-safe storage, then apply it after the active step or query returns to the owner thread

## Body-control composition

Body controls preserve the installed velocity callback's preparation/disposal lifetime, active masks and body identities. They call it for ordinary lanes before applying queued inputs and damping. Target-driven kinematic lanes use derived velocities. See [damping composition](BODIES-STATICS.md#per-body-damping), [input/target lifetime](BODIES-STATICS.md#step-and-target-lifetime) and [axis locks](BODIES-STATICS.md#physical-axis-locks) for their behavior under default, manual and custom steps
