# Engine integration

On the simulation thread, apply gameplay changes, step the world, then read bodies and run queries. Copy the completed physics state into presentation or ECS storage before the next mutation. If the application launches its own query jobs, wait for them before modifying the world again

## Keep one explicit physics owner

Store the world and its integration state in one subsystem owned by the simulation thread:

```text
Physics_State
  world
  fixed_stepper
  optional thread_pool or external dispatcher
  body and static property tables
  contact tracker
  ECS-to-physics and physics-to-ECS mappings
```

Do not copy a ready world, pool, thread pool, contact tracker, or property table. Keep each at a stable address until destroyed

## Initialization order

1. Create caller-owned pools, dispatchers, policy contexts, property tables, and trackers
2. Build `World_Description` from `world_description_default`
3. Install callback, threading, profiling, or timestep policy
4. Initialize the world
5. Register custom shape, constraint, collision, and sweep types before the first step
6. Register shared built-in or imported shapes
7. Create statics, bodies, constraints, and application mappings

## Fixed update flow

Use a fixed physics duration even when render frame time varies:

1. Convert input and gameplay changes into physics commands
2. Apply structural commands while the world is idle
3. Apply direct velocity, impulse, or pose changes that belong before the step
4. Call `fixed_stepper_update` or one or more explicit `world_step` calls
5. Drain selected contact, trigger, part and joint-break events after each completed step
6. Reacquire direct views and copy or consume the current physics state
7. Interpolate presentation state with the fixed-step alpha

[`fixed_step_loop`](../../examples/headless/fixed_step_loop/main.odin) contains the accumulator example. Submit [continuous forces and targets](reference/BODIES-STATICS.md#optional-body-controls) for each physics step that should consume them. When gameplay needs to submit new inputs between catch-up steps, drive explicit `world_step` calls

Choose a fixed-step operation that delivers the event streams you use: [`trigger_stepper_update`](reference/COLLISION.md#fixed-step-updates-and-custom-timesteps), the [mixed part/contact operations](reference/COLLISION.md#mixed-event-delivery), or [`joint_break_stepper_update`](reference/CONSTRAINTS.md#combined-fixed-step-delivery). If event storage fills, consume only the entries reported as delivered, grow storage and retry with zero elapsed time as those guides describe. Do not advance physics again for a step that already completed

## ECS identity mapping

Store application entity IDs separately from [physics handles](API.md#handles). After a successful removal, update both direction mappings and the matching [property-table entry](reference/DATA-ACCESS.md#property-lifecycle). Generation-protected property keys protect table lookups, not arbitrary world-handle storage

## Structural command ingestion

Gather worker-produced changes into application-owned arrays. Merge them into the required game order and apply them on the world owner thread through [typed batches or a command buffer](reference/STRUCTURAL-OPERATIONS.md). Earlier successful entries remain applied if a later entry fails. Update application state only for that successful prefix, using the returned count

## Direct state synchronization

After stepping, use `active_body_view` to iterate active bodies or `body_get` to copy one body's state, including a sleeping body. Neither operation saves a whole-world checkpoint. Copy presentation state before the next mutation. See [view validity](reference/DATA-ACCESS.md#view-validity) and [supported direct writes](reference/DATA-ACCESS.md#supported-direct-writes) before writing through borrowed storage

## Job-system integration

The included thread pool is the simplest multithreaded path. Set `description.threading.worker_count` before `world_init`

Use `Dispatcher_Interface` when the application supplies worker threads. It must block until all worker callbacks complete and provide worker-exclusive pools. Follow the [dispatcher contract](reference/WORLD.md#threading-selection) and validate it with `dispatcher_interface_validate` before installing it. [`external_dispatcher`](../../examples/headless/external_dispatcher/main.odin) shows both a custom interface and the included thread pool adapter

## Callback ownership

Store gravity, layer, material and other callback data beside the physics subsystem. Install [built-in policies](reference/CALLBACKS-STAGES.md#built-in-pose-policies) where possible. Follow [callback ownership](reference/CALLBACKS-STAGES.md#callback-ownership) for raw callbacks and shared worker data

## Queries and events

Place queries between world mutations. For parallel jobs, assign each caller an independent [query context](reference/QUERIES.md#independently-owned-query-contexts) and join them before stepping again. Route [contact events](reference/COLLISION.md#contact-tracking), [trigger and part events](reference/COLLISION.md#drain-events-and-preserve-identity), and [joint-break events](reference/CONSTRAINTS.md#removal-and-notification-retry) through their captured identities before reusing application mappings

## Cooking and streaming

Cook geometry on an application worker, join that work, then import while the destination world is idle. Follow [the cooking handoff and cleanup sequence](COOKING.md#background-cooking)

## Shutdown order

1. Stop jobs, queries, callbacks, and cooking imports that can touch the subsystem
2. Drain or discard application event queues
3. Unbind or destroy contact trackers attached to the world
4. Destroy every query context attached to the world
5. Destroy the world
6. Release external mappings, property tables, and callback-policy storage
7. Destroy application-owned dispatchers and thread pools
8. Destroy caller-owned buffer pools after all attached worlds and contexts
9. Destroy cooked assets, then their cooking contexts
