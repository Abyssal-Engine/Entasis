# Contact islands, 1x4

- Configuration: `release`
- Run date (UTC): `2026-09-27`
- Odin compiler: `dev-2026-09-nightly:a2fb372`

Variant: `box-1x4-awake`

Windows 11, Intel Core i7-13700K, 24 logical processors. Workers include the caller, with unpinned placement

10,000 boxes in 50 islands with 50 statics, common components. 30 warmup steps, then 300 measured steps in the same world using summed native-step timing. 1 velocity iteration per substep and 4 substeps per 60 Hz step, sleeping disabled and recording off

Median of five process mean step times, in milliseconds per step. Lower is better

| Workers | Median ms/step |
| ------: | -------------: |
|       1 |      10.036811 |
|       2 |       6.216592 |
|       3 |       4.870748 |
|       4 |       4.154191 |
|       5 |       3.615868 |
|       6 |       3.238050 |
|       8 |       2.698150 |
|      10 |       2.897382 |
|      12 |       3.034015 |
|      14 |       2.972960 |
|      16 |       2.824561 |
|      24 |       2.476813 |

[Raw worker CSVs](./) | [Workload and timing](../../../../BENCHMARK.md#workloads-and-timing) | [Reproduction](../../../../BENCHMARK.md#reproduction)
