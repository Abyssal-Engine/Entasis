# Collision

Use collision policies for solid contacts, restitution for bounce, and triggers for non-solving overlap events. Direct shape-pair queries and contact tracking are described below

For every public declaration, see the [Collision section of the API index](API-INDEX.md#collision)

## Built-in collision policy

The built-in layer and material policy combines three caller-owned data sources:

1. `Collision_Property_Table` keyed by collidable identity
2. `Layer_Matrix` defining allowed layer pairs
3. `Material_Table` defining contact material records

Build the policy with `layer_material_policy`, then install it through `narrow_policy_layers_materials` and `world_description_set_callbacks`

[`materials_and_filtering`](../../../examples/headless/materials_and_filtering/main.odin) shows the complete setup

## Layers and masks

`Collision_Layer` is a compact index in `[0, 63]`. `Layer_Mask` stores the allowed target layers for one collidable

Use:

- `collision_layer`
- `layer_mask`, `layer_mask_none`, `layer_mask_all`
- `layer_mask_add`, `layer_mask_remove`, `layer_mask_contains`
- `collision_filter`, `collision_filter_default`

`Collision_Filter` combines one layer with one mask. `collision_filter_allows` checks the symmetric filter relationship directly

## Layer matrix

`Layer_Matrix` stores one 64-bit row per layer

- `layer_matrix_none` starts with every pair denied
- `layer_matrix_all` starts with every pair allowed
- `layer_matrix_allow` and `layer_matrix_deny` update a symmetric pair
- `layer_matrix_set` writes one explicit relationship
- `layer_matrix_allows` reads the current decision

The matrix stays caller owned and must remain valid while the installed policy can access it

## Materials

`Material` identifies friction, maximum recovery velocity, and spring response used by the built-in material policy

Use:

- `material` or `material_default`
- `material_id` and `material_id_is_valid`
- `material_table`
- `material_table_get`
- `material_to_contact`
- `material_combine_default`

A `Material_Table` is a caller-owned borrowed slice. Material IDs index directly into that slice

Custom material combination uses `Material_Combine_Proc` and caller-owned context

## Restitution

Enable bounce once on an idle world with `world_enable_restitution` and `restitution_configuration_default()`. Set `configuration.fallback.coefficient` before enabling, or call `restitution_set` for selected body/static collidables. Defaults are coefficient `0` and threshold `1` world unit/second

| Setting or operation               | Behavior                                                            |
| ---------------------------------- | ------------------------------------------------------------------- |
| `Restitution_Settings.coefficient` | Normal rebound ratio in `[0, 1]`                                    |
| `Restitution_Settings.threshold`   | Finite nonnegative minimum approach speed, in world units/second    |
| `restitution_get`                  | Effective setting, including fallback when no override exists       |
| `restitution_remove`               | Remove an override. `Not_Found` if none exists                      |
| `restitution_reserve`              | Grow collidable-setting and pair-history capacity without shrinking |
| `world_disable_restitution`        | Release optional restitution storage                                |

Pair combination takes the larger coefficient and threshold by default. A custom `Restitution_Combine_Proc` runs synchronously on the owner thread during solve preparation. The configuration is copied. Its user data remains borrowed until disable or world destruction. The callback must return normally and must not mutate the same world

Settings are per collidable, independent of material IDs and shared shapes. Setting changes do not awaken sleepers. Removal, handle reuse and world clear retire the affected settings/history. All mutators require an idle world

### Impact response

Impact speed is captured after velocity integration and before warmstart. A speculative contact may retain incoming speed while braking the body, but bounce begins only when geometric depth becomes nonnegative. Departure clears that retained approach. Separation followed by another approach can create a later impact. Resting contacts do not repeatedly bounce

A substep retains one bounce target across its solver iterations. Positive recovery and bounce targets combine by maximum, not addition. Impact response uses rigid normal mass. Friction and ordinary non-impact spring response retain their material settings. Timestep, contact approximation and other constraints affect the observed rebound. Restitution does not provide continuous time-of-impact resolution

Built-in convex and nonconvex contacts support this response, including sequential fallback and moving kinematics. Rejected contacts and triggers do not bounce. An incompatible custom contact accessor returns `Invalid_Description`. Custom constraint kernels retain their authored behavior

### History, capacity and custom steps

Successful complete steps commit impact history once. Failed custom steps discard pending history without rolling back executed physics. A standalone successful `world_stage_solve` commits its own history. Solve stages within a custom timestep share pending history until the timestep completes

A settings/history capacity failure returns `Capacity_Missing`. Reserve the required capacity before retrying. Target buffers may grow during solve preparation and are reused afterward. Disabled restitution allocates no optional storage. Enabled worlds with only zero coefficients need settings/history storage but no packed bounce-target arrays

## Collision properties

`Collision_Properties` stores one collidable's filter and material ID

`Collision_Property_Table` owns separate dense body and static tables so equal numeric body and static handles do not collide

Follow [property-table lifetime](DATA-ACCESS.md#property-lifecycle) when bodies or statics are added, removed or cleared. Batch set/remove operations stop at the first failure and report how many earlier entries remain applied

## Raw narrow-phase callbacks

Install `narrow_policy_default` for one constant material, or use [raw narrow-phase callbacks](CALLBACKS-STAGES.md#raw-callback-tables) for custom pair/child filtering and manifold configuration. Follow [callback ownership and threading](CALLBACKS-STAGES.md#callback-ownership)

## Direct collision queries

`collision_query` tests two registered shapes directly from caller-supplied poses without inserting either shape as a body or static

`collision_query_batch` executes direct pair tests in input order and writes pointer-free `Collision_Query_Result` entries into caller-owned output

These calls require live registered shapes and an idle world. Results remain valid after the call, while embedded handles follow normal handle lifetime rules

[`direct_collision_queries`](../../../examples/headless/direct_collision_queries/main.odin) combines direct pair tests with world sweeps

## Contact tracking

`Contact_Tracker` compares consecutive completed narrow-phase contact samples to produce begin, persist, and end events

Lifecycle:

1. Initialize tracker storage
2. Bind it to an initialized world before the first tracked step
3. Step the world
4. Drain or discard events after every successful step
5. Unbind or destroy the tracker before destroying its world
6. Destroy any retained tracker storage after it is no longer needed

`contact_events_drain` copies events into caller-owned storage. `contact_events_discard` advances pair history without copying events. Draining ends retired pair lifetimes first, then visits current contacts in pair-cache discovery order. Array order is not a stable identity

`contact_tracker_ensure_capacity` grows tracker storage explicitly. `contact_tracker_capacity` reports the maximum retained pair count, not the output event-buffer capacity

## Contact user data

`Contact_User_Table` associates caller-defined values with bodies and statics for event routing

Body and static handle spaces stay separate. The caller owns the table and synchronizes its entries with object lifetime using the [property rules](DATA-ACCESS.md#property-lifecycle). Contact user data does not change collision behavior

## Event lifetime

`Contact_Event` is pointer free. Its body or static handles are snapshots of the identities observed by the step

Resolve application entity IDs before external mappings are removed or reused. Do not assume an event handle remains live after structural changes following the step

[`overlap_and_events`](../../../examples/headless/overlap_and_events/main.odin) shows overlap results and tracked contact events

## Trigger colliders

Triggers report overlap transitions without creating physical contact response. Mode belongs to the body or static instance, so a shared shape may be used by both solids and sensors. Explicit joints attached to a sensor body remain physical

1. Enable storage with `world_enable_triggers`
2. Obtain a reference with `body_collidable_reference` or `static_collidable_reference`
3. Select sensor mode with `trigger_set(world, reference, {user_id=..., stay=.Disabled})`
4. Step the world, then drain or discard each nonempty trigger event batch

`trigger_remove` restores solid mode without deleting the collidable. `world_disable_triggers` releases trigger storage and rejects an undrained nonempty event batch. Use owner-thread operations while the world is idle

### Detection and filtering

Sensors use discrete collision-phase geometry, before solving. A contact must have nonnegative geometric depth. Speculative proximity alone is not an overlap. Compound children and mesh triangles reduce to one parent pair

Dynamic, kinematic, sensor/sensor and sleeping interactions are supported. Static/static detection requires `static_static=.Enabled` on at least one sensor. Mesh triggers detect triangle-surface intersection, not closed-mesh volume containment. Continuous crossings are unsupported. Use [part roles](#mixed-solid-and-trigger-parts) to combine solid and sensor geometry in one collidable

The usual `allow` and `allow_child` filters apply. Sensor pairs do not use material selection, contact-constraint selection or physical-contact notification as their response. Worker callbacks must obey the [callback threading and reentry contract](CALLBACKS-STAGES.md#callback-ownership). Wide custom collision tasks receive their registered custom payloads, not a built-in packed layout inferred from their contents

Queries include sensors as geometry by default and do not change trigger history. Use query filters when a query should exclude them

### Drain events and preserve identity

`trigger_events_drain` copies Enter, Exit and selected Stay events to caller storage. Each event includes a completed step number, owner epoch, lifetime tokens, collidable references and copied application values. Published values survive endpoint removal

If output is too small, drain returns zero written and the exact required count. Grow the output and retry **without stepping again**. A nonempty undrained batch blocks the next step. `trigger_events_discard` acknowledges the batch without copying. Empty batches need no acknowledgement. `trigger_overlaps` copies retained pairs without advancing history

Raw handles can be reused. Route a delayed Exit through its lifetime token and copied application identity instead of resolving a possibly reused handle. `trigger_set_user_id` attaches an ID to a visitor without making it a sensor. Editing an ID affects future snapshots. It does not restart overlap lifetime or alter a pending batch. World clear and `trigger_reset_history` invalidate the owner epoch

### Sleeping and changed geometry

For a new static sensor, use `static_add(..., .None)` before `trigger_set` to avoid an initial overlap wake-up. Moving or removing a registered static sensor does not awaken sleepers. Converting a solid to a sensor retires its contact constraints, including sleeping contacts, while retaining explicit joints

Sleeping bodies remain discoverable without being awakened. Unchanged dormant pairs retain their history. A custom timestep that sleeps bodies after its final collision stage leaves their changed poses pending for the next sample

Public mutation operations notify trigger tracking. After changing application filter policy, call `trigger_mark_filters_dirty`. After a supported direct geometry edit, call `trigger_mark_geometry_dirty`. Unannounced raw topology or handle-pool writes require `trigger_reset_history`

### Reserve capacity and recover from failure

`Trigger_Configuration` reserves retained pairs, candidates per worker and child scratch separately. `trigger_reserve` reuses sufficient storage and preserves committed history if growth fails. Child capacity must cover the children generated within one collision batch

Exhausted pair, candidate or child capacity returns `Capacity_Missing` without publishing a partial event batch or replacing committed overlap history. Increase the relevant reservation before retrying. Earlier physics stages may already have executed. Failure preserves tracking history, not a rollback of world state

Reserved trigger detection and event storage are reused across steps. Other physics stages may still grow their own scratch. No trigger storage is allocated before enabling the feature

### Fixed-step updates and custom timesteps

`trigger_stepper_update` accepts elapsed time and caller event storage. Its result contains `completed_steps`, `events_written`, `required`, `alpha` and `notification_pending` (`.Ready` or `.Pending`). When output fills, `required` is the exact pending-batch size in addition to the delivered `events_written` prefix. It is zero when no batch is pending

Supply elapsed time once. If output exhaustion follows a successful physics step, that step's time was already consumed. Retry with zero new elapsed time or drain explicitly. Contact trackers are a separate stream. Use the [part/contact stepper](#mixed-event-delivery) or [joint-break stepper](CONSTRAINTS.md#combined-fixed-step-delivery) when their selected features must drain together. Otherwise call `world_step` and drain each stream before another step

Custom timesteppers must execute collision detection and preserve the step-index contract. The last successful collision stage supplies the sample, published only after the complete step succeeds. A successful custom step without a sample consumes its physics time but records a notification error. Drain/update reports the error and further tracked steps remain blocked until `trigger_reset_history`. Failed steps publish no transitions. Manual stages alone do not publish a completed-step event batch

## Mixed solid and trigger parts

A body or static may combine physical and sensor parts without duplicating its shape or body. Enable triggers first, then call `collider_parts_reserve` with `collider_part_configuration_default()`. Set `event_subscription=.Enabled` before the first reservation if part events are needed. Subscription is fixed for that binding lifetime

Reserve `instance_capacity` for all participating collidable identities, including unconfigured solid partners whose endpoints are recorded in part events

`collider_part_set` assigns `Collider_Part_Settings` to one top-level child of a compound or big compound. A primitive, custom convex shape or mesh has one part at index zero. A nested child subtree shares its top-level part's role and identity. Unconfigured parts are solid. Shared geometry can have different settings on different instances

| Operation                | Behavior                                                                                         |
| ------------------------ | ------------------------------------------------------------------------------------------------ |
| `collider_part_set`      | Set `.Solid` or `.Trigger`, a copied `user_id`, and optional `.Stay` or `.Static_Static` flags   |
| `collider_part_get`      | Read a configured override and its identity                                                      |
| `collider_parts`         | Copy configured overrides in top-level child order, with explicit required capacity              |
| `collider_part_remove`   | Remove the override and restore the default solid role                                           |
| `collider_parts_reserve` | Grow instance, part, pair and per-worker observation storage without shrinking committed history |

Whole-collidable `trigger_set` and configured part overrides are mutually exclusive. Switching roles retires affected physical contact state, while explicit joints and the body's authored mass and inertia remain unchanged. Solid/solid children produce contacts and children involving a sensor produce overlap observations. The usual parent and child filters still apply

Moving or removing a static tests its solid-part bounds against the candidate body's solid-part bounds, using the static's old/new pose union. An unconfigured collidable retains whole-shape bounds. Sensor-only bounds on either mixed endpoint do not cause awakening. Shape replacement retires the old instance's overrides, so the replacement starts solid. Removal and handle reuse also retire old identities

### Mixed event delivery

Parent trigger events reduce sensor overlap to one collidable pair. With part subscription enabled, `trigger_part_events_drain` also reports top-level part pairs, and `trigger_part_overlaps` copies their retained overlap state. `trigger_part_events_discard` acknowledges a pending part batch without copying. Endpoint snapshots include the instance, incarnation, part serial, child index and copied user ID. Use these snapshots when routing delayed events after removal or handle reuse

Parent and part batches must both be acknowledged before another tracked step. Insufficient output returns zero written and the required batch size without consuming that stream. `trigger_reset_history` clears both histories and advances the epoch. Disabling triggers rejects a nonempty pending parent or part batch

`collider_part_stepper_update` delivers parent and part streams. `collider_part_stepper_update_with_contacts` adds a bound `Contact_Tracker`. Both require part-event subscription. Their results report delivered prefixes, required capacities, pending streams, completed steps and interpolation alpha. Parent and part output capacities are checked together before either batch is acknowledged. Contact output may already contain a delivered prefix when another stream blocks

Consume each returned prefix once, grow the indicated output or tracker history capacity, then retry with `elapsed=0`. A tracker must belong to this world and must not have skipped a completed step. Capacity failure preserves pending notifications, not a rollback of completed physics. For automatic joint breaks as well, use the [combined fixed-step delivery](CONSTRAINTS.md#combined-fixed-step-delivery) operation

[`triggers`](../../../examples/headless/triggers/main.odin) shows whole-collidable sensors, a mixed body with a nested sensor subtree and contact/part event delivery
