# Public ray-batch benchmark

This benchmark measures `entasis.query_batch` against the same scene and ray inputs as `spatial_query_trace`: 50,000 closest rays per frame, submitted in batches of 256, with ten warmup frames and 100 measured frames

Each worker processes a contiguous input range. Timing includes query submission, output initialization and writes, hit consumption, and worker dispatch. Scene creation, worker storage allocation, warmup, and scalar validation are outside timing. Worker storage is allocated once and reused

Every result is compared against a scalar query before and after measurement: hit or miss, target, child, distance, position, and normal. Floating-point comparisons use `0.0002 * max(1, abs(a), abs(b))`. The CSV records the validation checksum and total measured time in `ray_elapsed_ms`:

- Milliseconds per frame: `ray_elapsed_ms / measured_batches`
- Million rays per second: `ray_query_count * measured_batches / (ray_elapsed_ms * 1000)`

The separate mixed-query benchmark measures 100,000 queries per frame, combining rays, sphere casts and overlaps. See [published query results](../../BENCHMARK.md#query-results) for both measurements

## Run on Windows

From the full repository root in PowerShell 7, with the [Windows build prerequisites](../../docs/odin/BUILDING-WINDOWS.md) and the declared native compiler selection:

```powershell
& .\scripts\windows\benchmarks\build_benchmarks.ps1 -Package spatial_query_batch -Configuration Release
& .\scripts\windows\benchmarks\run_benchmark.ps1 `
    -Package spatial_query_batch -Configuration Release `
    -Runs 5 -Workers '1,2,3,4,5,6,8,10,12,14,16,24' `
    -TimeoutSeconds 600
```

Build compiles the benchmark and reporter. Run uses those prepared binaries and runs 60 independent processes sequentially without requiring a compiler. Rebuild explicitly after source or toolchain changes. Release uses `-o:speed`, `x86-64-v3`, and disabled bounds checks and assertions. Worker counts include the calling thread. Windows chooses placement without affinity pinning. Select counts within the available logical processors on another host

The recorded September 10 five-run matrices took about 10.4 seconds each for execution and reporting, excluding compilation. The 600-second timeout is a safety ceiling for that work. Success publishes CSVs and a report to `build/benchmark-results/windows_amd64/spatial_query_batch/release/<attempt>/`. Failure retains partial output in `<attempt>.pending/` and preserves earlier complete results. Save failed output before rerunning

For Linux commands and result promotion, see [Benchmarks](../../BENCHMARK.md#linux-runs-and-result-promotion)
