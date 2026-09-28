# Headless examples

Each linked package is a complete executable under `examples/headless`. Choose a task below. The example runner builds the selected package before running it

From the full checkout or extracted Odin package root, after installing the [platform prerequisites](../BUILDING.md#requirements), run one example:

```sh
# Linux
./scripts/linux/examples/run_examples.sh --package falling_box --configuration development
```

```powershell
# Windows PowerShell 7
& .\scripts\windows\examples\run_examples.ps1 -Package falling_box -Configuration Development
```

The executable is written under `build/examples/development/`. The falling box prints its settled height near `0.5`, followed by `RUN_EXAMPLES_OK`. Replace `falling_box` with any package name below

Each package keeps its authored setup, step/operation, checks and teardown in adjacent `case.odin`, with `main.odin` driving the standalone sequence. In the full checkout, `tools/physics_viewer` imports these same cases for finite paused inspection and interactive controls. Its graphical chooser presents 13 motion examples and 13 diagnostic stepping examples; nine correctness-first examples remain on the headless and explicit capture routes. It never reconstructs benchmark simulations. Headless examples need neither the workspace nor its graphics/compression dependencies

## Create and update a world

| Example                                                                        | Task                                                     |
| ------------------------------------------------------------------------------ | -------------------------------------------------------- |
| [`minimal_world`](../../examples/headless/minimal_world/main.odin)             | World initialization, one step, and teardown             |
| [`falling_box`](../../examples/headless/falling_box/main.odin)                 | Separate floor and box shapes, a falling body and its final position |
| [`box_pile_batch`](../../examples/headless/box_pile_batch/main.odin)           | Bulk body creation in input order                        |
| [`bulk_body_update`](../../examples/headless/bulk_body_update/main.odin)       | Dense active-body iteration and direct motion updates    |
| [`external_dispatcher`](../../examples/headless/external_dispatcher/main.odin) | External blocking dispatcher integration                 |
| [`fixed_step_loop`](../../examples/headless/fixed_step_loop/main.odin)         | Frame-time accumulation and interpolation alpha          |

## Contacts, filtering and queries

| Example                                                                                  | Task                                                                                                      |
| ---------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------- |
| [`materials_and_filtering`](../../examples/headless/materials_and_filtering/main.odin)   | Collision layers, material tables, and built-in narrow policy                                             |
| [`ray_queries`](../../examples/headless/ray_queries/main.odin)                           | Any, closest, and all-hit rays                                                                            |
| [`sweep_queries`](../../examples/headless/sweep_queries/main.odin)                       | Registered-shape sweep against world collidables                                                          |
| [`overlap_and_events`](../../examples/headless/overlap_and_events/main.odin)             | Overlap collection and tracked contact events                                                             |
| [`sleep_and_awaken`](../../examples/headless/sleep_and_awaken/main.odin)                 | Automatic sleep and explicit island awakening                                                             |
| [`continuous_collision`](../../examples/headless/continuous_collision/main.odin)         | Fast projectile against a thin wall: continuous versus discrete collision, with a positive contact margin |
| [`batched_queries`](../../examples/headless/batched_queries/main.odin)                   | Heterogeneous rays and volume queries in one input order                                                  |
| [`impulses_solver_contacts`](../../examples/headless/impulses_solver_contacts/main.odin) | Body impulses, contact events, and accumulated solver impulse inspection                                  |
| [`direct_collision_queries`](../../examples/headless/direct_collision_queries/main.odin) | Direct registered-shape pair tests and world sweeps                                                       |
| [`triggers`](../../examples/headless/triggers/main.odin)                                 | Whole-collidable sensors, nested mixed solid/sensor parts and combined contact/part event delivery        |

## Body motion and constraint recipes

| Example                                                                          | Task                                                                       |
| -------------------------------------------------------------------------------- | -------------------------------------------------------------------------- |
| [`kinematic_platform`](../../examples/headless/kinematic_platform/main.odin)     | Kinematic motion driving a dynamic body                                    |
| [`constraints`](../../examples/headless/constraints/main.odin)                   | A basic two-body constraint                                                |
| [`substepping`](../../examples/headless/substepping/main.odin)                   | Multiple solver substeps per world step                                    |
| [`per_body_gravity`](../../examples/headless/per_body_gravity/main.odin)         | Dense gravity data gathered by body handle                                 |
| [`character_controller`](../../examples/headless/character_controller/main.odin) | Capsule ground probe and swept horizontal movement against static geometry |
| [`vehicle`](../../examples/headless/vehicle/main.odin)                           | Wheel bodies, servos, hinges, motors, layers, and materials                |
| [`cloth`](../../examples/headless/cloth/main.odin)                               | A constraint grid using shapeless bodies                                   |
| [`rope`](../../examples/headless/rope/main.odin)                                 | Distance and twist constraints with enumeration                            |
| [`ragdoll`](../../examples/headless/ragdoll/main.odin)                           | Articulated bodies, self-filtering, and constraint inspection              |
| [`planetary_gravity`](../../examples/headless/planetary_gravity/main.odin)       | Inverse-square radial gravity policy                                       |
| [`gyroscope`](../../examples/headless/gyroscope/main.odin)                       | Angular integration mode selection                                         |
| [`tank_controller`](../../examples/headless/tank_controller/main.odin)           | Tracked vehicle constraints, collision layers, and material policy         |

`character_controller` is a small application recipe: it probes the ground, sweeps a kinematic capsule toward a wall and applies the permitted pose change. It does not supply a production character-controller API, stair climbing, sliding or dynamic support handling. Vehicle, tank, cloth, rope and ragdoll examples likewise assemble bodies and constraints in application code

## Shapes and asset preparation

| Example                                                                      | Task                                                     |
| ---------------------------------------------------------------------------- | -------------------------------------------------------- |
| [`compounds`](../../examples/headless/compounds/main.odin)                   | Dynamic compounds and big compounds                      |
| [`convex_hulls`](../../examples/headless/convex_hulls/main.odin)             | Convex hull creation and registered inertia              |
| [`meshes`](../../examples/headless/meshes/main.odin)                         | Static mesh collision and ray access                     |
| [`custom_shape`](../../examples/headless/custom_shape/main.odin)             | Custom convex shape registration and collision routing   |
| [`background_cooking`](../../examples/headless/background_cooking/main.odin) | Cooking on an application worker and owner-thread import |

## Custom constraints and inspection

| Example                                                                                      | Task                                                     |
| -------------------------------------------------------------------------------------------- | -------------------------------------------------------- |
| [`custom_constraint`](../../examples/headless/custom_constraint/main.odin)                   | Custom constraint registration and wide kernel callbacks |
| [`profiling_and_state_export`](../../examples/headless/profiling_and_state_export/main.odin) | Per-stage profile snapshot and pointer-free state reads  |

## Build and run

Follow [Linux builds](BUILDING-LINUX.md#build-examples) or [Windows builds](BUILDING-WINDOWS.md#build-examples) for the package and configuration options. Use the [online full-repository benchmark guide](https://github.com/Abyssal-Engine/Entasis/blob/main/BENCHMARK.md) for performance measurements

<a id="features-documented-in-the-reference"></a>

For APIs without a standalone example, use the [domain reference map](../README.md#api-reference)
