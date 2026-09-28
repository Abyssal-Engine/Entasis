# Documentation

Start with [Odin](odin/GETTING-STARTED.md) to compile Entasis into your application, or [C/C++](c/README.md) to use the prebuilt SDK or build libraries from source

## Build and run

- [Building Entasis](BUILDING.md): downloads, prerequisites and shared build settings
- Odin: [Linux](odin/BUILDING-LINUX.md) and [Windows](odin/BUILDING-WINDOWS.md) example and test commands
- C ABI source builds: [Linux](c/BUILDING-LINUX.md) and [Windows](c/BUILDING-WINDOWS.md)
- Prebuilt C SDK: use the README at the root of your extracted package. Its [consumer guide](../sdk/README.md) and [requirements](../sdk/BUILDING.md) are also available as repository templates
- [Headless examples](odin/EXAMPLES.md): choose a program by physics task
- [Physics viewer](../tools/physics_viewer/README.md): build the optional full-checkout tool for live examples, saved benchmark results and recording playback

## Integrate physics

- [Odin API](odin/API.md): status values, handles, ownership and threading
- [C API](c/API.md): initialization, resource lifetime, errors and feature usage
- [Engine integration](odin/INTEGRATION.md): fixed updates, body mappings, application jobs and shutdown
- [Cooking collision assets](odin/COOKING.md): prepare hulls, meshes and compounds, then import them into a world

## API reference

Odin references explain operations and their requirements. C references list declarations, fields and per-call behavior. Use the [Odin declaration index](odin/reference/API-INDEX.md) for symbol lookup or the [C header map](c/API.md#headers-and-references) to select an include

| Topic | Odin | C/C++ |
| --- | --- | --- |
| Status, diagnostics and versions | [Inspection](odin/reference/PROFILING-INSPECTION.md) | [Base](c/reference/BASE.md) |
| Worlds, stepping, workers and capacity | [World](odin/reference/WORLD.md) | [World](c/reference/WORLD.md) |
| Bodies, statics and motion controls | [Bodies and statics](odin/reference/BODIES-STATICS.md) | [Bodies](c/reference/BODIES.md) |
| Shapes and inertia | [Shapes](odin/reference/SHAPES.md) | [Shapes](c/reference/SHAPES.md) |
| Joints, motors, reactions and breaking | [Constraints](odin/reference/CONSTRAINTS.md) | [Constraints](c/reference/CONSTRAINTS.md) |
| Filters, materials, contacts and triggers | [Collision](odin/reference/COLLISION.md) | [Collision](c/reference/COLLISION.md), [events](c/reference/EVENTS.md) |
| Rays, sweeps, overlaps and distance | [Queries](odin/reference/QUERIES.md) | [Queries](c/reference/QUERIES.md) |
| Batches and deferred commands | [Structural operations](odin/reference/STRUCTURAL-OPERATIONS.md) | [Command buffers](c/API.md#structural-command-buffers) |
| State views and application properties | [Data access](odin/reference/DATA-ACCESS.md) | [Views](c/reference/VIEWS.md), [properties](c/reference/PROPERTIES.md) |
| Callbacks and manual stages | [Callbacks and stages](odin/reference/CALLBACKS-STAGES.md) | [Callbacks](c/API.md#callback-boundary) |
| Profiling and solver inspection | [Profiling](odin/reference/PROFILING-INSPECTION.md#enable-profiling) | [Views and profiling](c/reference/VIEWS.md) |
| Cooking and imports | [Cooking](odin/COOKING.md) | [Cooking](c/reference/COOKING.md) |

## Benchmarks

[Benchmarks](../BENCHMARK.md) contains dated measurements, machine and workload definitions, reproduction commands and links to detailed reports

## Support and project information

- [Platform support and limitations](LIMITS.md): tested targets, missing capabilities, fixed bounds and unsupported combinations
- [Change history](../CHANGELOG.md)
- [Project overview](../README.md)
- [License](../LICENSE) and [third-party attribution](../NOTICE)
