# Odin documentation

Run the falling-box example, then import Entasis into your application using [Getting started](odin/GETTING-STARTED.md). The guides below apply to the source included in this package

## Start here

- [Getting started](odin/GETTING-STARTED.md): run a falling box, set up imports and create a world
- [Build settings](BUILDING.md): prerequisites, tool selection and profiles
- [Linux commands](odin/BUILDING-LINUX.md) and [Windows commands](odin/BUILDING-WINDOWS.md): native example builds and runs
- [Odin API](odin/API.md): status, handles, ownership and threading
- [Engine integration](odin/INTEGRATION.md): initialization, fixed updates, jobs and shutdown
- [Cooking assets](odin/COOKING.md): prepare and import geometry
- [Headless examples](odin/EXAMPLES.md): complete programs grouped by task

## API reference

Use the [declaration index](odin/reference/API-INDEX.md) for symbol lookup

| Domain | Reference |
| --- | --- |
| World lifecycle, stepping and capacity | [World](odin/reference/WORLD.md) |
| Bodies, statics and motion | [Bodies and statics](odin/reference/BODIES-STATICS.md) |
| Shapes and inertia | [Shapes](odin/reference/SHAPES.md) |
| Joints, motors and automatic breaking | [Constraints](odin/reference/CONSTRAINTS.md) |
| Filters, materials, contacts and triggers | [Collision](odin/reference/COLLISION.md) |
| Rays, sweeps, overlaps and distance | [Queries](odin/reference/QUERIES.md) |
| Batches and deferred commands | [Structural operations](odin/reference/STRUCTURAL-OPERATIONS.md) |
| Dense views and properties | [Data access](odin/reference/DATA-ACCESS.md) |
| Callbacks and manual stages | [Callbacks and stages](odin/reference/CALLBACKS-STAGES.md) |
| Profiling, statistics and versions | [Profiling and inspection](odin/reference/PROFILING-INSPECTION.md) |

## Project information

- [Package overview](../README.md)
- [Full repository](https://github.com/Abyssal-Engine/Entasis) for tests, benchmarks and maintainer tools
- [Benchmarks](https://github.com/Abyssal-Engine/Entasis/blob/main/BENCHMARK.md) for dated measurements and reproduction instructions requiring the full checkout
- [Platform support and limitations](LIMITS.md)
- [Change history](../CHANGELOG.md)
- [License](../LICENSE) and [attribution](../NOTICE)
