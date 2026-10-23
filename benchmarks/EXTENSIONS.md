# Native extension benchmarks

[Benchmarks and published results](../BENCHMARK.md)

These Odin workloads run through the native Windows benchmark commands. Sizes and measured operations are fixed in source. `-Components Common` selects the original component set. Current custom scenes use 1x4, so historical 4x1 samples require matching solver settings before comparison. `-Components All` adds the feature groups below. The CSV `benchmark_components` field records the compiled selection. Feature timings remain separate from `physics_elapsed_ms`

| Package             | Common workload                                                  | Additional `all` groups                                                                               |
| ------------------- | ---------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------- |
| `custom_extensions` | Custom constraint arities 1-4 and sequential fallback            | Body controls, watched joints and native/C gravity callbacks              |
| `query_extensions`  | Four shape routes x eight public query operations                | Closest point, shape distance, depenetration, and overlap-any on the same four routes                 |
| `contact_islands`   | Existing contact-islands scene                                   | Restitution, whole-collider triggers, mixed solid/sensor parts, nested compounds and sleeping sensors |
| `world_lifecycle`   | Initialization, growth, cooking/import, clear/reuse, destruction | No component option                                                                                   |

## Custom constraints and body controls

The common custom scenes each contain 1,024 custom-sphere bodies with sleeping disabled, custom pose/material callbacks, and registered custom-versus-floor collision tasks. A disposable 30-step world precedes 300 measured steps in a fresh world. Every scene uses one velocity iteration per substep and four substeps. Arity three includes a partial bundle. Sequential fallback uses three one-body constraints per body and threshold one. Untimed checks cover body motion, contact coverage, and fallback placement

Body-control measurements use five separate 1,024-body scenes with the same warmup and measurement counts. All bodies remain separated, gravity and world damping are zero, and sleeping is disabled. The disabled and enabled-empty scenes move at constant velocity. Force-with-damping submits force each step and installs per-body damping during setup. Kinematic targets advance by one step at unit speed. Axis locks constrain linear Y and all angular axes while repeated force and torque exercise physical solving. X remains free

Axis locks are compliant springs. Their final velocity is checked against the isolated scalar response `v_next = s*(v+a*dt) - (1-s)*p*error`, where `s = 1/(1+omega*dt*(omega*dt+2*zeta))` and `p = min(omega/(omega*dt+2*zeta), 1/dt)`. One representative body's state is captured before the final submission, outside timing. Four scalar updates at `h=dt/4` advance positions, orientations and velocities from that captured state. Independent binary32 rational trigonometry retains the production approximation's quaternion rounding near identity. The oracle permits scaled float roundoff and retains the 0.002 position/quaternion-component bounds. This checks sustained spring response without assuming zero instantaneous angular velocity

`bc01_*_submission_ms` measures public force, torque, or target submission. `bc01_*_step_ms` measures `world_step`. Disabled and enabled-empty scenes have zero submissions. Preparing caller target poses, reservation, motion/constraint validation, pool accounting, and output are outside timing. Engine target preparation remains inside the measured `world_step`. Checksums and submission counts accompany each group. The new groups do not alter common custom-constraint totals

## Gravity policies and C callbacks

`custom_extensions` with `Components All` also measures 2,051 shapeless dynamic bodies, 30 disposable warmup steps and 256 steps in a fresh measured world. Every case uses 60 Hz, explicit 1x4 with fallback threshold 64, disabled sleeping and unconstrained substep integration. Unit inverse mass/inertia, no contacts and no constraints isolate actual public policy/callback execution. Setup, policy tables, allocation, validation and teardown stay outside the step timer

Both profiles include native planetary gravity, native per-body gravity, and each policy with body controls enabled and one live override damping binding. A uniform-gravity control case covers its undamped override kernel too. Override rates match the policies' damping, and the scalar recurrence uses their actual exponential multipliers. Mixed64 adds matched native global planetary cases at zero and `{1e9,-1e9,1e9}`. Independent scalar radial acceleration and four-update recurrences check each body's position, linear/angular velocity, population and finite checksums. Native pool-byte totals must remain unchanged across measured steps

`counted_uniform` gives each of its 2,051 bodies constant Y acceleration followed by linear-velocity halving in each of four substeps. An independent scalar recurrence and exact per-body callback counts are checked outside timing

The C components import `entasis_c` and call its generated public exports. Both profiles execute the actual exported per-body policy. Mixed64 also initializes a world with a real C global callback descriptor and authors high positions through the global body API. Worker-local counters observe active visits, partial masks and positions that the float cache cannot retain. These measurements cover the production bridge in the native image, including masking, and do not measure shared-DLL loading or call overhead

Setup reserves 4,096 body slots and matching pair/constraint capacities before timing. `gravity_*_step_ms` and `c_gravity_*_step_ms` remain separate component costs. The CSV records precision, actual 1x4 settings, counts and checksums. Global-only components and columns are omitted in float32. C `*_world_owned_bytes` and `*_worker_owned_bytes` count live All_Owned allocator bytes, including metadata. Their allocation request counts and live bytes must stay unchanged during stepping, and all client allocations must be freed after teardown. Native `*_pool_bytes` fields count pool blocks instead

## Queries

Convex, compound, mesh, and native-custom worlds each perform closest ray, filtered ray, all-hit ray, closest sweep, all-hit sweep, geometric overlap, direct collision, and two-pair collision-batch operations. Each route/operation uses 128 warmup calls and 8,192 measured calls. Results are stored during execution and validated afterward

Closest-point, distance, depenetration and overlap-any measurements run afterward, with separate elapsed fields and reserved distance-query scratch. Query calls are serial. Configured worker counts do not establish parallel query scaling

Selection and repetition controls are described under [selected Windows query measurements](../BENCHMARK.md#selected-windows-query-measurements)

### Selected hierarchy ray cells

Windows `query_extensions` accepts `-QueryCase hierarchy -QueryOperation ray-direct|ray-any|ray-closest|ray-all -QueryCell mesh-128|mesh-8192|nested-mesh-128|nested-mesh-8192|box-1|hull-32|hull-256`, with `-Components All`. Each Run executes exactly one cell. Missing, duplicate, cross-case and group/repetition combinations reject before execution

Cells prepare 1,536 rays with 50% hits, then execute eight passes, totaling 12,288 queries. Workers are 1, 8, 16, 24 or 32 and partition disjoint ranges over one immutable world. Each worker owns a context. WN includes the caller. The direct entry uses `shape_ray`, and world entries use their corresponding context API. Nested All returns one closest hit per outer child, with storage for both children

The timer includes dispatch, completion, traversal, geometry and output writes. Construction, 128 warmup calls per worker, a 1,536-call independent oracle pass and result/checksum checking are outside timing. CSV rows identify the selected case/operation/cell, precision, components, actual input/pass/query/warmup/oracle counts and hits. The reporter rejects mixed identities/counts and retains historical control rows. It reports throughput and only reports scaling when a W1 lane is present

```powershell
.\scripts\windows\benchmarks\build_benchmarks.ps1 -Configuration Release -Precision Float32 -Package query_extensions -Components All
.\scripts\windows\benchmarks\run_benchmark.ps1 -Configuration Release -Precision Float32 -Package query_extensions -Components All -QueryCase hierarchy -QueryOperation ray-direct -QueryCell mesh-8192 -Runs 1 -Workers 16 -ProcessorAffinity 4294967295 -TimeoutSeconds 120
```

Build prepares the image without running a workload. Output identity is `build/benchmark-results/windows_amd64/query_extensions/hierarchy.<operation>.<cell>/[mixed64/]release/<attempt>/`. Control remains the default, with its existing output identity

The compound cells are `compound-8`, `big-128`, `big-8192`, `nested-compound-8` and `nested-big-8192`. Flat cells admit only `ray-direct`, while nested cells admit all four ray operations

These cells register one Box and place 8, 128 or 8,192 children on spacing-two X/Z grids, alternating identity and Y-rotated child orientations. The root is translated and Y-rotated. Nested cells contain two copies at Y=0/-2. Even rays select child `((i/2)*73)%child_count`, and paired odd rays shift local X by `4*columns`. The independent oracle expects t=4, normal=(0,1,0), the selected flat child or outer child zero, and nested All's second hit at t=6/child one. Other misses return Ok with zero results, while Closest misses return Not_Found

These five cells require an explicit `-TimeoutSeconds` so Run retains producer logs and checks exactly one `WORKER_AFFINITY` record per active worker. Before warmup, each actual worker thread binds to the corresponding prefix of LP order `18,16,20,22,24,26,28,30,0,2,4,6,8,10,12,14,19,17,21,23,25,27,29,31,1,3,5,7,9,11,13,15`. A second native affinity call verifies the one-bit mask, and records retain native thread IDs. The caller's previous mask is restored and background workers are shut down after use. Existing Mesh/Box cells retain their placement behavior

The compound comparison on the verified Ryzen 9 7940HX uses process masks W1=262144, W8=1431633920, W16=1431655765, W24=4294923605 and W32=4294967295, identical for both images and independent controls. These placements do not establish optimal speed or reserve CPUs, and require fresh topology inspection after hardware changes. Query, warmup and oracle counts and the CSV schema remain the same as the existing hierarchy cells

The hull cells are regular Y prisms with 32 vertices/18 faces or 256 vertices/130 faces. Both precisions admit all four operations. Rays use direction (0.125,-1,0.0625) and maximum t=8. Hit rays enter the top plane at t=4 with normal (0,1,0), while paired rays shifted by four X units miss. The independent oracle expects 6,144 hits/checksum. Hull import, preparation, warmup and checking remain outside timing. Hull cells retain process eligibility without compound per-thread pinning

### Selected global ray cells

Windows `query_extensions` All admits `-Precision Mixed64 -QueryCase global -QueryOperation ray-any|ray-closest|ray-all -QueryCell mesh-8192-near|mesh-8192-far|mesh-8192-local|box-near|box-far|box-local`, with workers 1/8/16/24/32. Select exactly one cell per Run, excluding group/repetition selectors. Other precisions, geometries and operations reject before launch

Each cell contains 1,024 static instances of one shared 8,192-triangle grid or unit box, rotated 180 degrees about Y on a 32x32 grid. Near/far origins use offsets 1e3/1e9 with binary fractional positions and submillimeter Y distinctions. Local cells translate the same scene to zero and call legacy local context rays, providing a shared local-frame control for both offsets. Half the prepared rays hit and half miss in gaps. Independent analytic checks cover status, cardinality, triangle/target identity, normal and exact high-coordinate location

Work is 1,536 inputs x eight passes = 12,288 queries, with 128 warmups per worker and a separate untimed 1,536-input oracle pass. WN includes the caller, with private contexts, stacks and output ranges reading one immutable world. The timer includes dispatch/completion, the query entry and observation writes. Construction, cooking, insertion, reservation, warmup, oracles and CSV/checksum work are outside timing. Expected hits/checksum are 6,144. The reporter shows actual-work throughput and only computes scaling when the dataset has W1

```powershell
.\scripts\windows\benchmarks\run_benchmark.ps1 -Configuration Release -Precision Mixed64 -Package query_extensions -Components All -QueryCase global -QueryOperation ray-closest -QueryCell mesh-8192-far -Runs 1 -Workers 1 -ProcessorAffinity 4294967295 -TimeoutSeconds 120
```

Outputs use `build/benchmark-results/windows_amd64/query_extensions/global.<operation>.<cell>/mixed64/release/<attempt>/`. Build the matching prepared All producer/reporter first through the existing Build entry. Run performs no implicit build


### Selected hierarchy distance, point, correction and sweep cells

Windows `query_extensions` All admits the following bounded operation/cell pairs in both precision profiles. The `global` case admits only the first row in mixed64, through the actual global context entries at X offset 1e9

| Case | Operations | Cells |
| --- | --- | --- |
| hierarchy, global | `closest-point`, `distance`, `depenetration` | `nested-query-64`, `query-box-1` |
| hierarchy | `sweep-any`, `sweep-closest`, `sweep-all` | `compound4-nested64`, `compound4-mesh128`, `compound4-nested64-angular`, `compound4-mesh128-angular`, `query-box-1` |

`nested-query-64` and the corresponding sweep target contain eight Compound rows of eight Box children in Big_Compounds, with 4-unit spacing. Distance uses an eight-Sphere query union, while point and correction use a point and one radius-0.25 Sphere respectively. Sweep uses four radius-0.25 Spheres against 64 Boxes or a Compound containing a cooked 128-triangle 8-by-8 plane. The target pose is (16,-8,32) with an exact 180-degree Y rotation. The single-leaf control uses one Box and a Sphere query. Setup imports, static insertion, context/dispatcher creation, reservation and input preparation precede all measurements

Sweep velocity is the rotated (0.25,-4,0.125), maximum time 1. Angular cells also rotate at one radian/second around the query children's X axis. Sphere centers lie on that axis, so the independent geometry still gives the plane/Box entry times 0.9375/0.8125. Half the sweeps miss. Point/distance inputs alternate two separated heights, and correction alternates a 0.125 Y correction with an already separated position. Untimed oracles check statuses, counts, distance, witnesses, normals and child IDs for distance/point, correction translations/state/pass count, and sweep brackets/normals for hit payloads. Angular motion and hierarchy enumeration remain in the production query path

Workers 1/8/16/24/32 each own a context, input partition and output partition, including caller zero. These cells use process eligibility without per-thread pinning. Each Run has 1,536 inputs, eight measured passes (12,288 calls), 128 untimed warmup calls per worker and a separate 1,536-call untimed oracle. The query timer includes dispatch/completion, existing context/API admission and required result writes. Inputs/worlds are immutable and queries do not reset world state. Validation runs before and after timing. Result count/checksum is 12,288 for distance/point/correction and 6,144 for sweeps. A result count is a resolved result, including a separated correction

Use the existing Build and Run entry points, for example:

```powershell
.\scripts\windows\benchmarks\build_benchmarks.ps1 -Configuration Release -Precision Float32 -Package query_extensions -Components All
.\scripts\windows\benchmarks\run_benchmark.ps1 -Configuration Release -Precision Float32 -Package query_extensions -Components All -QueryCase hierarchy -QueryOperation distance -QueryCell nested-query-64 -Runs 1 -Workers 1 -ProcessorAffinity 4294967295 -TimeoutSeconds 120
```

Result identity is `<case>.<operation>.<cell>/[mixed64/]release/<attempt>/` beneath the existing query_extensions result owner. Failed pending output and child statuses remain available

### Selected direct collision batch cells

Windows query_extensions All admits `-QueryCase collision-batch -QueryOperation collision` with exactly one fixed `-QueryCell`. Both float32 and mixed64 support workers 1/8/16/24/32. Cells are `box-native-1`, `box-native-8`, `box-native-32`, `box-native-256` and their `box-context-*` equivalents, `box-native-scalar`, `box-context-scalar`, `sphere-native-256`, `sphere-context-256`, `sphere-native-scalar` and `sphere-context-scalar`. Scalar cells partition the same inputs in groups of 256 and call the corresponding scalar facade

Each cell prepares 24,576 axis-aligned shape pairs, with alternating operand order and 50% separated inputs. Box cells use boxes of dimensions 2 and 1. Sphere cells use radius 0.5 against a dimension-2 box. Hit separation is 1.25 and miss separation is 4, yielding 12,288 hits with depth 0.25. Independent geometry checks verify status, count, normal, offset and finite bounded contacts. Complete scalar manifold parity is checked before and after measurement

Each worker owns a context and disjoint contiguous input/result ranges, partitioned by whole batches. One complete selected batch per worker warms the route. One untimed pass checks all inputs and one measured pass executes 24,576 pairs. Dispatch, completion, facade calls and required result writes are timed. Construction, context allocation/reservation, scalar/independent oracles and final comparison/reduction are outside timing. Ordinary collision cells admit only immutable canonical convex routes and borrow no shared collision scratch. W1 uses the same dispatcher boundary with caller zero. A process mask controls eligibility without pinning individual workers

Build preparation uses `.\scripts\windows\benchmarks\build_benchmarks.ps1 -Configuration Release -Precision Float32 -Package query_extensions -Components All`. Run uses `.\scripts\windows\benchmarks\run_benchmark.ps1 -Configuration Release -Precision Float32 -Package query_extensions -Components All -QueryCase collision-batch -QueryOperation collision -QueryCell box-context-256 -Runs 1 -Workers 1 -ProcessorAffinity 4294967295 -TimeoutSeconds 120`. The 120-second invocation ceiling covers producer and reporter. Results retain actual pair/warmup/oracle counts under `query_extensions/collision-batch.collision.<cell>/[mixed64/]release/<attempt>/`

The scalar collision holdouts `box-triangle-scalar` and `box-hull-scalar` use the same pair counts and native scalar call with batch-256 partitioning. They use a large plane Triangle against a dimension-2 Box, or a cube Hull with half extent 1 against a dimension-1 Box. Triangle hit separation is 0.75 and Hull hit separation 1.25, with separated inputs at 4. Independent geometry checks use depth 0.25, the expected axial normal, contact-plane/surface membership and the existing 1e-5 numerical bound. These cells do not batch Triangle or Hull kernels

## Watched joints

Watched-joint measurements use 1,024 independent ball-socket pairs and 2,048 shapeless bodies with zero gravity and damping, disabled sleep, one velocity iteration per substep and four substeps at 60 Hz. Each group runs 30 warmup steps and 300 measured steps. Poses and opposing velocities are restored outside timing

The five groups have tracking disabled, tracking enabled without watches, 64 sparse watches spread across solver bundles, 1,024 watches, or 1,024 joints that break each step. Sparse and dense watches use high finite limits. Breaking joints use a zero force limit and are recreated outside the next timed step

`bj01_*_step_ms` records watched-joint steps, including `world_step`, reaction capture and deferred removal. Drain and recreation have separate fields. Untimed checks use an independent one-dimensional scalar ball-socket recurrence across all four substeps. It retains warmstart impulses across frame resets, checks each signed impulse divided by the substep duration, and selects the earliest strongest sample. Force/torque peaks, sample identity/duration, joint survival, ordered breaks and distinct watch lifetimes retain their existing tolerances. These groups follow the body-control measurements and do not change historical custom-constraint totals

## Contacts, restitution, and triggers

Restitution uses separate 640-body impact groups at 0%, 10%, and 100% selection, four warmup impacts and 24 measured impacts. Separation and impact resets occur outside timing. Bounce and contact counts accompany the elapsed times

Whole-collider trigger measurements use 320 isolated static/dynamic sphere pairs: 640 collidables total. Sensor selection covers both endpoints of 0, 32, or 320 pairs, giving 0%, 10%, and 100% sensor coverage. Four warmup transitions precede 24 measured transitions, alternating entry and exit. Resetting body positions is untimed. Stepping and event draining have separate timers. Untimed checks require the expected event identities/counts, physical constraints only for solid pairs, and no solver response on sensors. Sensor groups intentionally have different collision behavior and are reported separately

Mixed-collider measurements use 320 compound/static pairs with shared geometry and per-instance sensor selection. Separate groups cover all-solid compounds, whole-collider triggers, flat mixed compounds, nested mixed compounds and sleeping sensor overlaps. Flat compounds contain two spheres. Each nested top-level child contains two sphere descendants, while events retain the top-level part identity

Four warmup transitions precede 24 measured transitions. Pose and velocity resets occur outside timing. Step, parent-event drain and part-event drain are timed separately. Selected streams must each deliver 3,840 Enters and 3,840 Exits. Physical velocities are checked against an untimed scene containing only the matching solid geometry

The sleeping group first settles all 320 bodies within 321 untimed setup steps, stopping once all bodies sleep, then measures 300 unchanged dormant steps and the transition sequence separately. Only statics move during transitions, and sensor-only overlaps must not wake the bodies. Failed settling, event identity or response checks fail the sample

## Lifecycle and memory

One disposable cycle precedes 20 measured cycles. Worlds grow from capacity hints of 128 to 2,048 bodies and constraints, import fixed cooked geometry, clear/repopulate, and destroy their resources. Each component and cycle retains its elapsed time. Geometry preparation, status validation, and pool accounting are untimed. Worker count affects resource creation/destruction here. The fixture performs no simulation dispatch

Pool byte fields describe allocated world, ordinary worker, or cooking pool blocks where named. They do not measure process RSS, all allocator traffic, or the trigger system's private allocations. Timing totals and process-run medians do not establish frame p99 or performance acceptance

## Windows commands

From the full repository root in PowerShell 7, with the [Windows build prerequisites](../docs/odin/BUILDING-WINDOWS.md):

```powershell
& .\scripts\windows\benchmarks\build_benchmarks.ps1 -Configuration Release -Package custom_extensions -Components All
& .\scripts\windows\benchmarks\run_benchmark.ps1 -Package custom_extensions -Configuration Release -Components All -Runs 5 -Workers '1,2,3,4,5,6,8,10,12,14,16,24' -TimeoutSeconds 900
```

Use the same commands for the other packages. Contact-islands runs use `-Shape Box -Variant box-default`. Lifecycle omits `-Components`. The timeout bounds the complete package sample matrix and report generation. Builds occur before measurement. Compare retained common results with matching common inputs, and review enabled costs separately using the raw CSV and reporter output

Runs write CSVs and a report under `build/benchmark-results/windows_amd64/<package>/`. Contact-islands results include the variant directory. Each attempt uses a unique `<attempt>.pending/` directory and publishes as `<attempt>/` after success. Failed attempts and earlier completed runs are retained. See [reproduction and promotion](../BENCHMARK.md#reproduction) for output handling
