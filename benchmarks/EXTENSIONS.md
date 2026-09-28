# Native extension benchmarks

[Benchmarks and published results](../BENCHMARK.md)

These Odin workloads run through the native Windows benchmark commands. Sizes and measured operations are fixed in source. `-Components Common` preserves the workloads used for the original-engine baseline. `-Components All` adds the feature groups below. The CSV `benchmark_components` field records the compiled selection. Feature timings remain separate from `physics_elapsed_ms`

| Package             | Common workload                                                  | Additional `all` groups                                                                               |
| ------------------- | ---------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------- |
| `custom_extensions` | Custom constraint arities 1-4 and sequential fallback            | Body controls and watched joints, including empty, sparse, dense and breaking selections              |
| `query_extensions`  | Four shape routes x eight public query operations                | Closest point, shape distance, depenetration, and overlap-any on the same four routes                 |
| `contact_islands`   | Existing contact-islands scene                                   | Restitution, whole-collider triggers, mixed solid/sensor parts, nested compounds and sleeping sensors |
| `world_lifecycle`   | Initialization, growth, cooking/import, clear/reuse, destruction | No component option                                                                                   |

## Custom constraints and body controls

The common custom scenes each contain 1,024 custom-sphere bodies with sleeping disabled, custom pose/material callbacks, and registered custom-versus-floor collision tasks. A disposable 30-step world precedes 300 measured steps in a fresh world. Every scene uses four velocity iterations and one substep. Arity three includes a partial bundle. Sequential fallback uses three one-body constraints per body and threshold one. Untimed checks cover body motion, contact coverage, and fallback placement

Body-control measurements use five separate 1,024-body scenes with the same warmup and measurement counts. All bodies remain separated, gravity and world damping are zero, and sleeping is disabled. The disabled and enabled-empty scenes move at constant velocity. Force-with-damping submits force each step and installs per-body damping during setup. Kinematic targets advance by one step at unit speed. Axis locks constrain linear Y and all angular axes while repeated force and torque exercise physical solving. X remains free

Axis locks are compliant springs. Their final velocity is checked against the isolated scalar response `v_next = s*(v+a*dt) - (1-s)*p*error`, where `s = 1/(1+omega*dt*(omega*dt+2*zeta))` and `p = min(omega/(omega*dt+2*zeta), 1/dt)`. One representative body's state is captured before the final submission, outside timing. Standard scalar `acos` uses the stored quaternion. The oracle permits scaled float roundoff and retains the 0.002 position/quaternion-component bounds. This checks sustained spring response without assuming zero instantaneous angular velocity

`bc01_*_submission_ms` measures public force, torque, or target submission. `bc01_*_step_ms` measures `world_step`. Disabled and enabled-empty scenes have zero submissions. Preparing caller target poses, reservation, motion/constraint validation, pool accounting, and output are outside timing. Engine target preparation remains inside the measured `world_step`. Checksums and submission counts accompany each group. The new groups do not alter common custom-constraint totals

## Queries

Convex, compound, mesh, and native-custom worlds each perform closest ray, filtered ray, all-hit ray, closest sweep, all-hit sweep, geometric overlap, direct collision, and two-pair collision-batch operations. Each route/operation uses 128 warmup calls and 8,192 measured calls. Results are stored during execution and validated afterward

Closest-point, distance, depenetration and overlap-any measurements run afterward, with separate elapsed fields and reserved distance-query scratch. Query calls are serial. Configured worker counts do not establish parallel query scaling

## Watched joints

Watched-joint measurements use 1,024 independent ball-socket pairs and 2,048 shapeless bodies with zero gravity and damping, disabled sleep, four velocity iterations and one substep at 60 Hz. Each group runs 30 warmup steps and 300 measured steps. Poses and opposing velocities are restored outside timing

The five groups have tracking disabled, tracking enabled without watches, 64 sparse watches spread across solver bundles, 1,024 watches, or 1,024 joints that break each step. Sparse and dense watches use high finite limits. Breaking joints use a zero force limit and are recreated outside the next timed step

`bj01_*_step_ms` records watched-joint steps, including `world_step`, reaction capture and deferred removal. Drain and recreation have separate fields. Untimed checks cover reaction force against the body's velocity change, joint survival, ordered break identities and distinct watch lifetimes. These groups follow the body-control measurements and do not change historical custom-constraint totals

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
