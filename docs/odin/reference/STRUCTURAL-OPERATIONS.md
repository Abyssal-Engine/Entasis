# Structural operations

Apply body, static, shape and constraint changes while the world is idle. Batch and command operations preserve input order and report partial completion

For every public declaration, see the [Structural operations section of the API index](API-INDEX.md#structural-operations)

## Choose the operation path

| Workload                                                      | Best path                                                        |
| ------------------------------------------------------------- | ---------------------------------------------------------------- |
| One isolated object                                           | Single body, static, shape, or constraint procedure              |
| Many objects using the same operation                         | Typed batch procedure                                            |
| Mixed body and static operations that must preserve one order | `Command_Buffer`                                                 |
| Changes produced by parallel gameplay jobs                    | Gather into caller-owned storage, then apply on the owner thread |

No structural path is safe during a world step, query, callback, or active borrowed-view use

## Typed batches

The body, static, and shape batch APIs process input in exact order

| Group   | APIs                                                             |
| ------- | ---------------------------------------------------------------- |
| Bodies  | `body_add_batch`, `body_apply_batch`, `body_remove_batch`        |
| Statics | `static_add_batch`, `static_apply_batch`, `static_remove_batch`  |
| Shapes  | `shape_add_batch`, `shape_add_batch_typed`, `shape_remove_batch` |

A batch stops at the first failure. Earlier successful entries remain applied, and the returned count identifies that prefix. After correcting the failure, retry only the remaining entries

For add operations, caller-owned handle output must have at least the input length. Entries at and after the committed count remain invalid

Batch procedures do not create hidden command storage. Input arrays, output arrays, and result counts belong to the caller

[`box_pile_batch`](../../../examples/headless/box_pile_batch/main.odin) shows body batch creation

## Shape batches

`shape_add_batch` accepts one homogeneous built-in primitive type and validates the complete description layout before registration

`shape_add_batch_typed` is the advanced path for cooked or custom registered shape types. Successfully committed prefix entries transfer their internal shape ownership to the world registry. Unprocessed suffix entries remain caller owned

`shape_remove_batch` stops at the first referenced, missing, or otherwise invalid shape

## Command buffers

`Command_Buffer` is a non-owning view over caller-owned arrays

It can bind:

- Tagged `Command` entries
- Optional per-command `Status` output
- Body-add result slots
- Static-add result slots

Construct commands with:

- `command_body_add`
- `command_body_apply`
- `command_body_remove`
- `command_static_add`
- `command_static_apply`
- `command_static_remove`

Create the view with `command_buffer`, then call `world_apply_commands`

Commands run in exact input order. Add commands write their returned handle to the caller-selected result slot

## Partial failure

`world_apply_commands` attempts commands in order and records per-command status when status storage is supplied

Use the returned completed count to update application mappings only for successful commands. The command buffer is not transactional: a later failure does not undo earlier changes

Result slots that were not reached remain unchanged or hold the caller's initial sentinel value according to the supplied storage

Use one of these application policies:

1. Accept the committed prefix and retry or reject the suffix
2. Validate application-side preconditions before submission
3. Build explicit compensating commands when rollback is required by game logic

Rollback, when required, belongs to the application

## Ordering and determinism

Input order defines observable structural order for typed batches and command buffers

This matters when:

- Numeric handles are allocated and reused
- Static mutations wake overlapping islands
- Body removal deletes connected constraints
- Application property tables and entity mappings are updated
- Contact tracking observes object lifetime changes between steps

Keep command ordering explicit in the application rather than depending on worker completion order

## View invalidation

Structural operations can relocate dense storage. Reacquire direct views after mutation, including a failed attempt that advances the view epoch during partial allocation or mutation

The [view validity rules](DATA-ACCESS.md#view-validity) list invalidating operations and explain epoch checks. Finish using a borrowed view before submitting structural work

## Property-table synchronization

After successful removals, apply the [property-table lifecycle rules](DATA-ACCESS.md#property-lifecycle) only to the committed prefix

## Threading

Collect commands from worker jobs into thread-local or partitioned caller-owned arrays. Merge them into the required application order, then submit from the world owner thread while the world is idle

Do not call structural APIs from narrow-phase callbacks, pose callbacks, stage callbacks, dispatched physics work, or concurrent scene queries
