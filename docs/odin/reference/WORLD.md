# World

Create a world, configure its storage and workers, then advance it with a fixed physics duration

For every public declaration, see the [World section of the API index](API-INDEX.md#world)

## Lifecycle

A world is a caller-owned opaque handle for one simulation

Follow the [complete minimal program](../GETTING-STARTED.md#smallest-complete-world) for initialization, one step and destruction

The handle must start zero initialized, stay at a stable address while ready, and remain on its owner thread for lifecycle and structural operations

Do not copy a ready `World`. Do not destroy it while a step, query, callback, or borrowed view is active

## World descriptions

Start from `world_description_default` and change only the required policy

| Field                | Purpose                                                                             |
| -------------------- | ----------------------------------------------------------------------------------- |
| `profiling`          | Enables per-stage timestamp collection for completed steps                          |
| `gravity`            | Uniform gravity used by the default pose policy                                     |
| `damping`            | Per-second linear and angular damping fractions in `[0, 1]`                         |
| `capacity`           | Initial storage hints that may grow after initialization                            |
| `solve`              | Velocity iterations, substeps, fallback batch threshold, and optional scheduler     |
| `threading`          | Caller-only execution, included worker pool, or external dispatcher                 |
| `allocator`          | Legacy: facade resource block. All_Owned: owned world/pool/worker/extension storage |
| `narrow_callbacks`   | Copied narrow-phase callbacks with borrowed context                                 |
| `pose_callbacks`     | Copied pose callbacks with borrowed context                                         |
| `timestepper`        | Optional caller-owned complete timestep implementation                              |
| `timestep_callbacks` | Optional stage-completion callback table                                            |

The default description uses gravity `{0, -9.81, 0}`, linear and angular damping `0.01`, one worker, no profiling, and the runtime heap allocator for the facade block

Callback and timestep policy installation is covered in [callbacks and stages](CALLBACKS-STAGES.md)

## Pool ownership

`world_init` creates and owns an internal `Buffer_Pool`

`world_init_with_pool` borrows a caller-owned pool. The world returns its buffers during destruction but never destroys the supplied pool

Destroy every world attached to a caller-owned pool before `buffer_pool_clear` or `buffer_pool_destroy`

## Threading selection

Entasis coordinates its physics workers internally. `world_step` waits for their work to finish before returning. An application that calls `world_step` and then runs queries on the same thread needs no additional synchronization, even when physics uses multiple workers

If the application launches its own query jobs on separate threads, it must wait for those jobs to finish before stepping or modifying the same world. Entasis does not automatically coordinate those application-owned jobs. See [query concurrency](QUERIES.md#batch-concurrency) for supported operations and query-context ownership

| Configuration                              | Result                                                              |
| ------------------------------------------ | ------------------------------------------------------------------- |
| `worker_count = 1`, no external dispatcher | The caller thread performs the step                                 |
| `worker_count > 1`, no external dispatcher | The world creates and owns an included dispatcher                   |
| `external_dispatcher != nil`               | The world borrows the external dispatcher and uses its worker count |

`worker_count` includes the caller and supports 1-64 workers (`MAXIMUM_WORKER_COUNT`)

An external `Dispatcher_Interface` must synchronously invoke the supplied work procedure once for each worker index in `[0, worker_count)` and return only after all workers complete

`world_step_external` supplies a dispatcher for one step without changing world ownership. [`external_dispatcher`](../../../examples/headless/external_dispatcher/main.odin) shows the complete contract

## Direct stepping

`world_step` advances exactly the supplied positive duration. It does not accumulate render frame time or choose a timestep

The initialized dispatcher is used unless an explicit dispatcher path is selected

A successful step may:

- Integrate bodies
- Update broad-phase and narrow-phase state
- Solve constraints
- Move sleeping and awakened bodies between sets
- Run installed callbacks
- Update profiling data when enabled
- Invalidate direct body and static views

## Fixed-step accumulation

`Fixed_Stepper` is caller-owned accumulator state. Create it once, retain it between updates and call `fixed_stepper_update` each frame. The fragment assumes an initialized `world` and nonnegative `frame_elapsed` in seconds

```odin
stepper := entasis.fixed_stepper(1.0 / 60.0, 8);
steps, alpha, status := entasis.fixed_stepper_update(
	&stepper,
	&world,
	frame_elapsed,
);
```

`maximum_steps` limits catch-up work in one update. When the backlog exceeds that limit, whole excess steps are dropped while the fractional remainder is retained

Check `status` before using the result as a successful update. A failed `world_step` is not subtracted from this accumulator, but any physics already performed is not rolled back. Do not blindly retry failed physics. `alpha` is the remaining accumulator fraction used for presentation interpolation

For optional event streams, use the [combined fixed-step helpers](../INTEGRATION.md#fixed-update-flow), which distinguish completed physics from pending notification delivery

Use [`fixed_step_loop`](../../../examples/headless/fixed_step_loop/main.odin) as the complete example

## Substeps

Use `solve_description_substeps` for one fixed velocity-iteration count per substep

```odin
description := entasis.world_description_default();
description.solve = entasis.solve_description_substeps(
	substeps=4,
	velocity_iterations=4,
);
```

Use `solve_description_substep_scheduler` only when the iteration count must vary by substep. The scheduler callback and context stay caller owned

The default timestepper runs collision detection once per outer step. Solver substeps update contacts, solve constraints, and integrate bodies without rerunning full collision detection

Substeps add solver and integration work. They are not a replacement for fixed frame-time accumulation

## Capacity management

`Capacity_Hints` controls initial storage, not ordinary fixed limits

- `world_ensure_capacity` grows storage to satisfy new hints and never shrinks it
- `world_resize` moves retained storage toward the supplied hints and may grow or shrink it
- `world_clear` removes world contents and accumulated step state while retaining allocated capacity

These operations require an idle owner-thread world and invalidate direct body and static views

Constraint-batch capacity is derived from `fallback_batch_threshold + 1`. A zero `collision_child_pairs` hint derives its initial value from `pairs`

Custom type registration has separate limits: see [shapes](SHAPES.md#fixed-capacities) and [constraints](CONSTRAINTS.md#custom-constraints)

Body and constraint capacities do not reserve every contact, query, optional-feature or worker buffer. Reserve the resources used by the application through their own APIs. Changed contacts or topology can require growth after warmup

## Failure and rollback

Initialization validates the complete description before committing a ready world. Partial world-owned resources are released when initialization fails

A failed initialization leaves the caller's zero world reusable. A successful `world_destroy` resets the handle to zero. Destroying an already disposed world reports `Disposed`

A failed step is different from a failed initialization: physics already executed is not rolled back. A notification or capacity failure can follow successful physics work. Do not resubmit elapsed time for that completed work. Follow [body-control consumption](BODIES-STATICS.md#step-and-target-lifetime) and [notification retry](CONSTRAINTS.md#combined-fixed-step-delivery) for the operation's recovery procedure

## Explicit allocation scope

Ordinary initialization uses `.Legacy`: `World_Description.allocator` owns the facade resource block, while pools and workers use their established allocators. Select `.All_Owned` when the supplied allocator must cover all storage owned by that resource

| Resource                            | Initialization                                                      |
| ----------------------------------- | ------------------------------------------------------------------- |
| World and its included pool/workers | `world_init_with_allocation_scope(&world, description, .All_Owned)` |
| World borrowing a pool              | `world_init_with_pool_and_allocation_scope`                         |
| Independently owned buffer pool     | `buffer_pool_init_with_allocator`                                   |
| Independently owned thread pool     | `thread_pool_init_with_allocator`                                   |
| Cooking context                     | `cooking_context_init_with_allocator` in `entasis_cooking`          |

Full ownership includes pool metadata and blocks, included workers and thread records, private query scratch and contextual registrations. A borrowed pool or dispatcher keeps its own allocator and lifetime. A nil allocator procedure selects the runtime heap allocator

Keep allocator context alive until destruction completes. The allocator must accept the original size and alignment on Free, support concurrent calls from separate worker pools, and avoid reentering the resource it serves

In full mode, pool growth reserves the metadata needed to return its blocks, so valid pool returns do not allocate. Pool clear releases blocks while retaining that metadata. Destroy releases both. This does not make every warmed physics operation allocation-free: independent scratch may still need to grow

A pool's backing-block byte count is not total world memory. Separate scratch, pool metadata, allocator overhead and thread resources must be accounted for separately

Destroy attached query contexts before the world and destroy borrowed resources only after all users have stopped
