# Pyramid wall

The prepared wall contains 180 rows and 16,290 unit cubes, with identity rotations, zero initial velocities, unit density and Discrete collision detection. The original floor has half extents 500, 1, 500 and its top is y=0

Build with `scripts/windows/benchmarks/build_benchmarks.ps1 -Configuration Release -Package pyramid_wall`. Select a common `HarnessRoot` to compile this fixture against another folder's own engine

Run only after explicit workload approval with `scripts/windows/benchmarks/run_pyramid_wall.ps1`. Defaults are 18000 steps, 60 Hz, one velocity iteration, four substeps, sleeping disabled, friction 0.5, spring 30 Hz, damping ratio 1 and recovery velocity 2. Smaller walls are rejected

The CSV records initial state, step 1, every sample interval and final state. It separates mean center height, maximum center height and apex, and includes lateral and out-of-plane drift, drop/displacement counts, rotated floor penetration, invalid states, escaped bodies and maximum speed. The final snapshot contains all body poses and velocities

`physics_elapsed_ms` accumulates only `world_step` calls. Construction, observations, CSV output and snapshots are outside that region. A 300-step performance sample scores `(elapsed_ms_at_300 - elapsed_ms_at_60) / 240`. Physical observations are a separate result. Successful exit alone establishes neither stability nor performance acceptance

Both output paths must be fresh. Timeouts retain partial CSV and process logs. Process logs record duration, available peak working set and inherited placement. `RecycleDistance` accepts only 0.05, preserving the supplied candidates' delivered default without optional API calls
