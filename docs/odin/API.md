# Odin API

Import `entasis:entasis` for world, body, shape, constraint and query operations

Start with [getting started](GETTING-STARTED.md) for a complete program or [engine integration](INTEGRATION.md) for an application update loop

## Status handling

Mutating operations return `Status`. Checked value operations return `(value, Status)` or a result struct with an accompanying status

Given a returned `status`, with `entasis` and `core:fmt` imported, report a failure before using the operation's output:

```odin
if entasis.status_failed(status)
{
	fmt.println(entasis.status_text(status));
}
```

`status_ok` and `status_failed` test a result. `status_text` returns static allocation-free text. Public operations report invalid input through status values instead of panicking

For optional operation-specific detail, see [diagnostics](reference/PROFILING-INSPECTION.md#diagnostics)

## Handles

`Body_Handle`, `Static_Handle`, `Constraint_Handle`, and `Shape_Handle` identify objects owned by one world

The `*_handle_invalid` helpers create invalid sentinels. `*_handle_is_valid` checks only that representation. It does not prove that a handle is live in a particular world

Body, static, and constraint numeric values may be reused after removal. Clear application mappings and property-table entries when removal succeeds

## Ownership and lifetime

Keep ready worlds, pools and contexts at stable addresses. Do not copy them. Ownership and release order are defined with each resource: [worlds/pools](reference/WORLD.md#pool-ownership), [shapes](reference/SHAPES.md#shape-handles-and-ownership), [views/property tables](reference/DATA-ACCESS.md), [query contexts](reference/QUERIES.md#independently-owned-query-contexts) and [cooked assets](COOKING.md#ownership-and-concurrency)

## Threading

One owner thread controls world lifecycle and structural mutation

`world_step` may dispatch internal work through the included dispatcher or a caller-owned blocking dispatcher. The call does not return until all dispatched workers complete

The ordinary sequence is to step, read body state and run queries on the owner thread. Queries are synchronous. Physics worker count does not require extra application synchronization in this sequence

For application-owned parallel query jobs, follow [query concurrency](reference/QUERIES.md#batch-concurrency). Do not step or modify the same world until those jobs finish

Callbacks may run on physics workers. Callback context must remain valid for the world lifetime and must obey the callback-specific synchronization rules

Do not reenter the same world from a callback. Collect requests and apply them after the active call returns

## Allocation boundaries

See [world capacity](reference/WORLD.md#capacity-management), [allocator scope](reference/WORLD.md#explicit-allocation-scope) and [query scratch](reference/QUERIES.md#scratch-and-allocation). Custom-type limits are listed with [shapes](reference/SHAPES.md#fixed-capacities) and [constraints](reference/CONSTRAINTS.md#custom-constraints)

## Structural mutation

Batch and command-buffer operations can stop after applying earlier entries. Use their returned count to update mappings and retry only the unprocessed entries, as described in [structural operations](reference/STRUCTURAL-OPERATIONS.md). Reacquire borrowed state according to [view validity](reference/DATA-ACCESS.md#view-validity)

## API reference

The [documentation map](../README.md#api-reference) lists domain guides. The [declaration index](reference/API-INDEX.md) lists symbols and source locations
