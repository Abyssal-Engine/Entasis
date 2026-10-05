<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="docs/assets/entasis_logo_white.png">
    <source media="(prefers-color-scheme: light)" srcset="docs/assets/entasis_logo_black.png">
    <img alt="Entasis" src="docs/assets/entasis_logo_black.png" width="568">
  </picture>
</p>

Entasis is a high-performance, data-oriented physics engine written in Odin for 3D games. It provides rigid-body simulation, collision detection, constraints and scene queries through a direct Odin API and a C ABI for C11 and C++20

The engine uses SIMD and multithreaded execution on Windows and Linux AMD64. Odin games compile the engine source directly. C and C++ games can use shared or static libraries from source builds or a prebuilt C SDK

[![Discord community](https://img.shields.io/badge/Discord-Join%20community-5865F2?style=flat&logo=discord&logoColor=white)](https://discord.gg/34NY53KNB)

## Features

- **[Bodies and motion](docs/odin/reference/BODIES-STATICS.md):** dynamic and kinematic bodies, static colliders, shapeless bodies, sleeping and awakening
- **[Collision shapes](docs/odin/reference/SHAPES.md):** primitives, convex hulls, nested compounds and triangle meshes, with inertia calculation and custom shape registration
- **[Joints and constraints](docs/odin/reference/CONSTRAINTS.md):** ball sockets, hinges, distance and angular constraints, motors, servos, typed reactions, automatic force/torque breaking and custom constraint types
- **[Body controls](docs/odin/reference/BODIES-STATICS.md#optional-body-controls):** forces, torques, per-body damping, kinematic targets and physical axis locks
- **[Collision response and events](docs/odin/reference/COLLISION.md):** layers and masks, friction and contact materials, restitution, contact tracking, trigger Enter/Exit/Stay events and mixed solid/sensor parts
- **[Scene queries](docs/odin/reference/QUERIES.md):** rays, sweeps, overlaps, closest points, distance, penetration and separation, query batches and independent query contexts
- **[Simulation control](docs/odin/reference/WORLD.md):** fixed stepping, solver substeps, capacity reservation, included or game-owned worker dispatchers, and [CCD](docs/odin/reference/BODIES-STATICS.md#continuous-collision-detection) for fast-moving bodies
- **[Engine integration](docs/odin/INTEGRATION.md):** structural batches, deferred commands, dense state views, generation-aware properties, callback policies and profiling
- **[Collision asset cooking](docs/odin/COOKING.md):** prepare hulls, meshes and compounds outside a live world, then import them for simulation

Both language APIs expose these features. See [platform support and limitations](docs/LIMITS.md) for tested targets, fixed bounds and unsupported combinations

## Get started

Download a package from [Releases](https://github.com/Abyssal-Engine/Entasis/releases) or clone the repository:

- **Odin source** (`odin-source`): engine source and Odin examples. Compile the engine directly with your game
- **C SDK** (`c-sdk`): prebuilt runtime and cooking libraries, headers, CMake configuration and C/C++ examples for your platform. No Odin compiler required
- **GitHub checkout**: the [full repository](https://github.com/Abyssal-Engine/Entasis), including both APIs, examples, tests, benchmarks and build tools

The supported platforms are Linux and Windows on AMD64, the x86-64 CPU architecture. Binaries require the `x86-64-v3` instruction-set level, including AVX2

For Odin, extract `odin-source` or open the full checkout. Install the [platform prerequisites](docs/BUILDING.md#requirements), then run the falling-box example from its root. The scripts obtain the declared compiler when it is missing

```sh
# Linux
./scripts/linux/examples/run_examples.sh --package falling_box
```

```powershell
# Windows PowerShell 7
& .\scripts\windows\examples\run_examples.ps1 -Package falling_box
```

Success prints the box's settled height near `0.5` and `RUN_EXAMPLES_OK`. The executable is under `build/examples/`. Continue with [Odin getting started](docs/odin/GETTING-STARTED.md) to import the engine into your game

The full checkout also includes an optional [physics viewer](tools/physics_viewer/README.md) for live examples, saved benchmark results and recording playback. It uses the same headless cases. Graphics dependencies apply to visual tools, and LZ4 is also required by benchmark producers built with recording On. Ordinary engine and headless example builds need neither

For C/C++, extract the platform `c-sdk` and follow its root README to run the supplied consumers. See [C/C++ getting started](docs/c/README.md) for library selection, or [Building Entasis](docs/BUILDING.md) to build libraries from the full checkout

## API

### Odin

Import Entasis into your game. The engine compiles directly with your Odin code, with no separate library to build

This quick overview creates a floor, drops a 1 kg box from five units above it and prints its position after three seconds

```odin
package main

import "core:fmt"
import entasis "entasis:entasis"

main :: proc()
{
    world: entasis.World
    entasis.world_init(&world, entasis.world_description_default())
    defer entasis.world_destroy(&world)

    status: entasis.Status

    // a floor with its top at y = 0
    floor_shape: entasis.Shape_Handle
    floor_shape, status = entasis.shape_add(&world, entasis.box(20, 1, 20))

    floor: entasis.Static_Handle
    floor, status = entasis.static_add(
        &world, entasis.static_body(floor_shape, entasis.pose({0, -0.5, 0})),
    )

    // a 1 kg box, five units above the floor
    box_shape: entasis.Shape_Handle
    box_shape, status = entasis.shape_add(&world, entasis.box(1, 1, 1))

    inertia: entasis.Body_Inertia
    inertia, status = entasis.shape_registered_inertia(&world, box_shape, mass=1)

    body: entasis.Body_Handle
    body, status = entasis.body_add(
        &world, entasis.body_dynamic(box_shape, inertia, entasis.pose({0, 5, 0})),
    )

    // simulate three seconds at 60 Hz. gravity is enabled by default
    for step: int = 0; step < 180; step += 1
    {
        entasis.world_step(&world, 1.0 / 60.0)
    }

    state: entasis.Body_State
    state, status = entasis.body_get(&world, body)
    fmt.println("Box position:", state.pose.position)
}
```

The world owns its simulation storage and registered shapes. `world_destroy` releases them. The same API provides [batched body creation](docs/odin/reference/STRUCTURAL-OPERATIONS.md#typed-batches), [scene queries](docs/odin/reference/QUERIES.md) and [constraints](docs/odin/reference/CONSTRAINTS.md)

Run the supplied [falling-box example](examples/headless/falling_box/main.odin) or use [Odin getting started](docs/odin/GETTING-STARTED.md) for imports and error handling

### C and C++

The C ABI is split into focused headers under `include/entasis/`. This C11 overview runs the same scene. `NULL` disables optional diagnostic output

```c
#include <entasis/entasis.h>
#include <stdio.h>

int main(void)
{
    entasis_world_t world = {0};
    entasis_world_description_t description = entasis_world_description_default();
    entasis_world_init(&world, &description, NULL);

    const entasis_quaternion_t identity = {0, 0, 0, 1};

    // a floor with its top at y = 0
    entasis_box_t floor_box = entasis_box(20, 1, 20);
    entasis_shape_handle_t floor_shape;
    entasis_shape_add(&world, ENTASIS_SHAPE_TYPE_BOX, &floor_box, &floor_shape, NULL);

    entasis_static_description_t floor = entasis_static_body(
        floor_shape, entasis_pose((entasis_vector3_t){0, -0.5f, 0}, identity),
        entasis_ccd_discrete());
    entasis_static_handle_t floor_handle;
    entasis_static_add(&world, &floor, ENTASIS_AWAKENING_OVERLAPS, &floor_handle, NULL);

    // a 1 kg box, five units above the floor
    entasis_box_t box = entasis_box(1, 1, 1);
    entasis_shape_handle_t box_shape;
    entasis_shape_add(&world, ENTASIS_SHAPE_TYPE_BOX, &box, &box_shape, NULL);

    entasis_body_inertia_t inertia;
    entasis_shape_inertia(ENTASIS_SHAPE_TYPE_BOX, &box, 1.0f, &inertia);

    entasis_body_description_t body_description = entasis_body_dynamic(
        box_shape, inertia, entasis_pose((entasis_vector3_t){0, 5, 0}, identity),
        (entasis_body_velocity_t){0}, entasis_body_activity_default());
    entasis_body_handle_t body;
    entasis_body_add(&world, &body_description, &body, NULL);

    // simulate three seconds at 60 Hz. gravity is enabled by default
    for (int step = 0; step < 180; ++step)
    {
        entasis_world_step(&world, 1.0f / 60.0f, NULL);
    }

    entasis_body_state_t state;
    entasis_body_get(&world, body, &state, NULL);
    printf("Box position: %g, %g, %g\n",
           state.pose.position.x, state.pose.position.y, state.pose.position.z);

    entasis_world_destroy(&world, NULL);
    return 0;
}
```

Link this example against the runtime library (`Entasis::runtime` with CMake). Registered shapes belong to the world and are released by `entasis_world_destroy`. See the [C lifecycle guide](docs/c/API.md#minimal-lifecycle) for error handling

Offline hull, mesh, and compound cooking uses the separate cooking library and header:

```c
#include <entasis/cooking.h>
```

See [C and C++ getting started](docs/c/README.md) for integration choices and library targets

## Performance

Published measurements on Windows 11 and Intel Core i7-13700K, using Release speed optimization and `x86-64-v3`. Workers include the caller, with no CPU affinity pinning. Each dated report identifies its workload, compiler and measurement date

The September 27, 2026 simulation baseline uses Odin `dev-2026-09-nightly:a2fb372` and the default **4x1** solver profile: four velocity iterations and one substep per 60 Hz step. Sleeping and recording are disabled. Values are medians of five process mean step times, in milliseconds per step. Lower is better

| Workload                                                                                 | 1 worker | 8 workers | 24 workers |
| :--------------------------------------------------------------------------------------- | -------: | --------: | ---------: |
| [Box Container](results/container/box-4x1-awake/2026-09-27/README.md)                       |   10.347 |     2.872 |      2.738 |
| [Contact Islands](results/contact_islands/box-4x1-awake/2026-09-27/README.md)               |    8.781 |     2.677 |      2.432 |
| [Full Pyramid](results/pyramid/box-4x1-awake/2026-09-27/README.md)                          |   28.109 |     6.510 |      6.156 |

The [Ragdoll Stair-Tumble baseline](results/ragdoll_stair_tumble/ap5-20260913/README.md) from September 13, 2026 uses Odin `dev-2026-09:c59786ce8`: **6.662 / 2.133 / 2.270 ms per step** at 1 / 8 / 24 workers, using the same median-of-five statistic

Query results from September 10, 2026 use Odin `dev-2026-09:c59786ce8` and are in millions of queries per second. Each value is the median of five process throughput measurements. Higher is better

| Workload                                                                                  | 1 worker | 8 workers | 24 workers |
| :---------------------------------------------------------------------------------------- | -------: | --------: | ---------: |
| [Closest-ray batches](results/spatial_query_batch/cooperative-wait-20260910-d2/README.md) |   10.235 |    76.118 |    136.786 |
| [Mixed spatial queries](results/spatial_query_trace/2026-09-10/README.md)                 |   11.839 |    84.145 |    157.396 |

Each linked report includes raw samples and all twelve tested worker counts from 1 to 24. See [benchmark methodology](BENCHMARK.md) for workloads, timed regions and the alternative 2x2 and 1x4 profiles. These timings do not establish a physical-accuracy ranking

## Documentation

- [Documentation](docs/README.md): getting started, build instructions and API references by task and language
- [Headless examples](docs/odin/EXAMPLES.md): programs covering queries, vehicles, ragdolls, mixed colliders, cooking and custom physics
- [Engine integration](docs/odin/INTEGRATION.md): initialization, fixed updates, ECS mappings, jobs and shutdown

## License

Entasis is created and maintained by **abyssmadeuspart** and distributed under the [Apache License 2.0](LICENSE)

The low-level simulation core began as an Odin port of [BEPUphysics2](https://github.com/bepu/bepuphysics2), created by Ross Nordby. Its body and shape architecture, broad and narrow phases, constraint solver, queries and supporting utilities grew from that implementation. Entasis has since been reorganized, optimized and extended as its own engine

Special thanks to Ross Nordby for creating BEPUphysics2 and releasing it under the Apache License 2.0. BEPUphysics2 is copyright Bepu Entertainment LLC. The derived portions remain under the same license. See [NOTICE](NOTICE) for attribution
