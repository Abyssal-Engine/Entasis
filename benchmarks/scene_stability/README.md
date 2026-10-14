# Resting box scene stability

This observer uses the existing `contact_islands` and `pyramid` constructors and the ordinary `world_step` path. It admits box shapes, zero warmup and no pyramid projectiles. It leaves fixture geometry, material, damping and Passive collision detection unchanged

Build with `scripts/windows/benchmarks/build_benchmarks.ps1 -Configuration Release -Package scene_stability`. The Common component define is selected internally for the imported islands fixture

Run only after explicit workload approval with `scripts/windows/benchmarks/run_scene_stability.ps1 -Package contact_islands` or `-Package pyramid`. Defaults are 18000 steps, 60 Hz, 1x4, sleeping disabled and observations every 60 steps. Initial state, step 1 and final state are always retained. `ProjectileCount` is accepted only for pyramid and must be zero

Each island is reported separately against its initial stack and own floor footprint. The CSV includes expected/world/observed population, finite-state errors, escaped and below-floor bodies, mean height retention, lateral drift, drops/displacements, rotated-box floor penetration, apex and maximum speed. Pure body-state calculations are shared with the wall observer in `benchmarks/scene_observation`

Initial poses are captured once. Observations use fixed storage and have no physics timing score. The final snapshot includes all poses and velocities. Verification defaults to On and assesses every completed step, independently of `SampleEvery`. The physical fields summarize the whole fixture and repeat on each island diagnostic row. The evolving fields accompany each diagnostic row, with complete coverage and final outcome in the final row. `-Verify Off` records Unassessed. Physical Fail retains the outputs and returns status 2

Output paths must be fresh. Run reads a prepared binary and performs no build or compiler discovery. Timeouts preserve partial observations and logs. The four observation cases and ten qualification cases classify explicit states without creating or stepping a world. [Physical qualification](../../BENCHMARK.md#physical-qualification) defines the fixed-box criterion, supported fixtures and coverage windows
