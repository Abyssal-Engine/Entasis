# Contact islands, 4x1

- Configuration: `release`
- Run date (UTC): `2026-09-27`
- Odin compiler: `dev-2026-09-nightly:a2fb372`

Variant: `box-4x1-awake`

Windows 11, Intel Core i7-13700K, 24 logical processors. Workers include the caller, with unpinned placement

10,000 boxes in 50 islands with 50 statics, common components. 30 warmup steps, then 300 measured steps in the same world using summed native-step timing. 4 velocity iterations per substep and 1 substep per 60 Hz step, sleeping disabled and recording off

Median of five process mean step times, in milliseconds per step. Lower is better

| Workers | Median ms/step |
| ------: | -------------: |
|       1 |       8.780762 |
|       2 |       5.662878 |
|       3 |       4.481518 |
|       4 |       3.831368 |
|       5 |       3.365430 |
|       6 |       3.094042 |
|       8 |       2.676649 |
|      10 |       2.740410 |
|      12 |       2.803748 |
|      14 |       2.740498 |
|      16 |       2.663297 |
|      24 |       2.431708 |

[Raw worker CSVs](./) | [Workload and timing](../../../../BENCHMARK.md#workloads-and-timing) | [Reproduction](../../../../BENCHMARK.md#reproduction)
