# Box container, 2x2

- Configuration: `release`
- Run date (UTC): `2026-09-27`
- Odin compiler: `dev-2026-09-nightly:a2fb372`

Variant: `box-2x2-awake`

Windows 11, Intel Core i7-13700K, 24 logical processors. Workers include the caller, with unpinned placement

10,000 boxes and five statics. 30 warmup steps in a disposable world, then 300 measured steps in a fresh world using whole-loop timing. 2 velocity iterations per substep and 2 substeps per 60 Hz step, sleeping disabled and recording off

Median of five process mean step times, in milliseconds per step. Lower is better

| Workers | Median ms/step |
| ------: | -------------: |
|       1 |      12.761452 |
|       2 |       7.624662 |
|       3 |       6.078345 |
|       4 |       5.169785 |
|       5 |       4.493097 |
|       6 |       4.107351 |
|       8 |       3.545752 |
|      10 |       3.664513 |
|      12 |       3.812391 |
|      14 |       3.771288 |
|      16 |       3.672388 |
|      24 |       3.304977 |

[Raw worker CSVs](./) | [Workload and timing](../../../../BENCHMARK.md#workloads-and-timing) | [Reproduction](../../../../BENCHMARK.md#reproduction)
