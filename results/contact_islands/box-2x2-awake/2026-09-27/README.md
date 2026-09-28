# Contact islands, 2x2

- Configuration: `release`
- Run date (UTC): `2026-09-27`
- Odin compiler: `dev-2026-09-nightly:a2fb372`

Variant: `box-2x2-awake`

Windows 11, Intel Core i7-13700K, 24 logical processors. Workers include the caller, with unpinned placement

10,000 boxes in 50 islands with 50 statics, common components. 30 warmup steps, then 300 measured steps in the same world using summed native-step timing. 2 velocity iterations per substep and 2 substeps per 60 Hz step, sleeping disabled and recording off

Median of five process mean step times, in milliseconds per step. Lower is better

| Workers | Median ms/step |
| ------: | -------------: |
|       1 |       9.639913 |
|       2 |       5.900524 |
|       3 |       4.665508 |
|       4 |       3.883836 |
|       5 |       3.476117 |
|       6 |       3.071698 |
|       8 |       2.547864 |
|      10 |       2.701932 |
|      12 |       2.743643 |
|      14 |       2.670759 |
|      16 |       2.558937 |
|      24 |       2.235791 |

[Raw worker CSVs](./) | [Workload and timing](../../../../BENCHMARK.md#workloads-and-timing) | [Reproduction](../../../../BENCHMARK.md#reproduction)
