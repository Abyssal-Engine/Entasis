# Platform support and limitations

Entasis supports Windows and Linux on AMD64 CPUs meeting the full `x86-64-v3` instruction-set requirement. The limits below distinguish missing features from capacity, numerical and compatibility restrictions

## Platforms and CPUs

| Target        | Qualified environment          |
| ------------- | ------------------------------ |
| Windows AMD64 | Windows 11 25H2                |
| Linux AMD64   | Ubuntu 24.04.3 with glibc 2.39 |

- CPUs must support the full `x86-64-v3` instruction set, including AVX2. Checking for AVX2 alone is not sufficient. There is no fallback for CPUs below this baseline
- macOS, ARM64 including Apple Silicon, browser/WebAssembly and 32-bit targets have no supported build or SDK
- The tested environments above are not minimum-version guarantees. Other Windows versions, Linux distributions and libc versions have not been qualified

See [Building Entasis](BUILDING.md) for source-build prerequisites. SDK consumer requirements are in the package's README and build guide

## Missing features

- **Continuous sensor crossings:** triggers detect overlap at collision samples. An object can pass through a sensor between samples without an event. See [trigger detection](odin/reference/COLLISION.md#detection-and-filtering)
- **Character controller:** there is no built-in controller with stair climbing, wall sliding and ground handling. The [character-controller example](../examples/headless/character_controller/main.odin) demonstrates ground probing and swept movement using existing queries
- **Whole-world snapshots:** there is no built-in capture/restore of an exact running simulation. Recreating application objects, shapes, bodies and constraints does not restore all solver and feature history
- **Cooked asset persistence:** cooking and import operate in memory. There is no built-in cooked-asset byte format for saving and reloading prepared shapes. See [saving prepared shapes](odin/COOKING.md#saving-prepared-shapes)
- **Solid-volume mesh queries:** closed-mesh containment is not provided. Current [mesh queries](odin/reference/SHAPES.md#meshes) and mesh sensors operate on triangle surfaces

## Fixed capacities

- **Physics workers:** 1-64 per world, including the caller, for both included and application-owned dispatchers. See [threading configuration](odin/reference/WORLD.md#threading-selection)
- **Compound hierarchy depth:** at most 32. A convex-only compound has depth zero, and each parent containing a nonconvex child adds one level. See [hierarchy ownership and depth](odin/reference/SHAPES.md#compound-hierarchies)
- **Custom registrations:** shape types, collision/sweep routes and constraint registrations have fixed capacities. These count extension types, routes and callback storage, not scene bodies or shape instances. See [shape and route capacities](odin/reference/SHAPES.md#fixed-capacities) and [constraint capacities](odin/reference/CONSTRAINTS.md#custom-constraints)

## Accuracy and sampling

- **Coordinate precision:** positions, velocities and collision geometry use 32-bit floating-point values. Precision decreases farther from the origin. There is no double-precision simulation mode
- **Collision sampling:** the default timestep performs full collision detection once per world step. Solver substeps refine solving and integration without adding collision or trigger samples. See [substeps](odin/reference/WORLD.md#substeps)
- **Continuous collision and restitution:** continuous collision detection uses sweep estimates followed by contact generation, not exact time-of-impact simulation. Restitution uses the resulting contact solve rather than exact impact-time response. See [continuous collision settings](odin/reference/BODIES-STATICS.md#continuous-collision-detection) and [restitution](odin/reference/COLLISION.md#restitution)
- **Composite separation:** a successful correction is checked within tolerance but is not guaranteed to be the shortest translation. Numerical iteration budgets do not provide one total work bound across all composite leaves. See [distance and separation queries](odin/reference/QUERIES.md#closest-point-distance-and-separation)
- **Joint break timing:** thresholds are sampled after solved substeps, but crossed joints remain active until the complete step succeeds and pending removal can finish. See [removal and notification](odin/reference/CONSTRAINTS.md#removal-and-notification-retry)

## Feature compatibility

- **Custom geometry:** distance queries do not support arbitrary nonconvex custom payloads. C custom collision callbacks return convex manifolds, and compound traversal uses engine compound or mesh storage. See [custom shapes](odin/reference/SHAPES.md#custom-shapes) and the [C representation boundary in the online full repository](https://github.com/Abyssal-Engine/Entasis/blob/main/docs/c/API.md#native-representation-boundary)
- **Custom velocity callbacks:** per-body Additional damping is supported, but Override is limited to recognized native gravity policies. See [per-body damping](odin/reference/BODIES-STATICS.md#per-body-damping)

## Allocation and failure guarantees

Warmup and body/constraint reservation alone do not guarantee allocation-free stepping when contacts, topology or enabled features change. Resources have separate reservation requirements. See [capacity management](odin/reference/WORLD.md#capacity-management) and [query scratch](odin/reference/QUERIES.md#distance-scratch-and-batching)

A failed step does not automatically undo physics already executed. Notification or capacity failures can occur after successful physics work. See [failure and recovery](odin/reference/WORLD.md#failure-and-rollback) before retrying
