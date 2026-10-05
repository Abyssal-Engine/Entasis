# Entasis roadmap

Updated: 2026-10-05

The proposed development order starts with collision geometry and queries, then gameplay, reusable assets, simulation snapshots, large-world precision and destruction. All milestones are planned, with no scheduled release dates

For features available today, see the [project overview](README.md#features) and [current limitations](docs/LIMITS.md)

## Milestones

| Order | Milestone | Intended outcome |
| --- | --- | --- |
| 1 | Collision geometry and queries | Faster hull and query paths, closed-mesh queries and more predictable memory use |
| 2 | Character controller and gameplay | Capsule character movement, better fast-moving collision and sensor detection, and finer control over bodies and joints |
| 3 | Reusable collision assets | Direct use of application-owned composite geometry and saved prepared shapes |
| 4 | Simulation snapshots | Capture and restore a running simulation within the same process and engine build |
| 5 | Large-world precision | A coordinate approach chosen from measured physics accuracy, query accuracy, simulation time and memory use |
| 6 | Destruction | Offline fracture preparation and runtime fracturing, with detached pieces simulated as rigid bodies |

## 1. Collision geometry and queries

Start with the geometry and query paths that the later features will use

- Speed up convex hull construction and collision detection across shared and distinct hulls, including hulls with larger faces
- Improve bounding volume hierarchies (BVHs), the spatial trees used to find potential collisions and query hits. Reduce build/update costs and speed up rays, sweeps and overlaps, both individually and in batches
- Cover supported workloads with capacity reservations and remove temporary-storage requirements for penetration and separation queries between two spheres or a sphere and a triangle
- Add point containment and convex-shape overlap queries for closed meshes, including cavities and boundary contact
- Limit the work spent separating a shape from compound geometry, and recheck corrections against the compound's other shapes

## 2. Character controller and gameplay

Use the collision and query foundation to support common movement and interaction needs

### Character controller

Add an optional capsule character controller using scene queries, with application-owned movement state. Support wall sliding and corner handling, stair climbing with headroom checks, slope limits, following nearby walkable ground and moving-platform support

### Collision and body controls

- Improve fast-moving collision response by checking collisions more often within a simulation step
- Detect sensor crossings between collision checks, initially for convex shapes moving in straight lines without rotating
- Restrict selected translation and rotation axes directly in the solver
- Let custom velocity callbacks use per-body damping without applying it twice
- Stop joint impulses in later solver substeps once a break threshold has been crossed

## 3. Reusable collision assets

Reduce repeated geometry preparation and the need to copy application geometry into engine-owned compounds

- Support queries and collisions against fixed collections of application-owned shapes
- Save and reload prepared hulls, meshes and compounds

Custom geometry and prepared-asset storage are separate features. Saved assets cover built-in hulls, meshes and compounds, while applications own file access and loading of child shapes. Native custom geometry comes first, followed by its C interface when the native result is useful

## 4. Simulation snapshots

Develop snapshots after the selected motion, collision-sampling and storage changes settle, so capture and restore cover the resulting simulation state

Capture bodies, shapes, constraints and the internal history needed to continue simulation. Restore them within the same process and engine build, including contact history, sleeping state and object identities

Start with identical serial replay after restoring worlds that use built-in geometry, default engine callbacks and engine-owned resources. Parallel replay needs separate verification. Application state and portable save files remain separate from this feature

## 5. Large-world precision

Compare origin shifting, higher-precision coordinates and separate local worlds for independent areas. Measure physics and query accuracy far from the origin alongside simulation time and memory use

This milestone ends with a choice of coordinate approach for the supported world layouts. No precision strategy is selected yet

## 6. Destruction

Prepare fractured objects offline, including their pieces, collision shapes and breakable connections, so games can load assets that can break during simulation

Support runtime fracturing that generates fragments and their collision geometry when an object breaks. Release detached pieces as independent rigid bodies, whether they were prepared offline or created at runtime
