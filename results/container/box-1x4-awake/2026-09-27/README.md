# Box container, 1x4

- Configuration: `release`
- Run date (UTC): `2026-09-27`
- Odin compiler: `dev-2026-09-nightly:a2fb372`

Variant: `box-1x4-awake`

Windows 11, Intel Core i7-13700K, 24 logical processors. Workers include the caller, with unpinned placement

10,000 boxes and five statics. 30 warmup steps in a disposable world, then 300 measured steps in a fresh world using whole-loop timing. 1 velocity iteration per substep and 4 substeps per 60 Hz step, sleeping disabled and recording off

Median of five process mean step times, in milliseconds per step. Lower is better

| Workers | Median ms/step |
| ------: | -------------: |
|       1 |      12.748119 |
|       2 |       7.809493 |
|       3 |       6.102437 |
|       4 |       5.251157 |
|       5 |       4.653638 |
|       6 |       4.150579 |
|       8 |       3.586410 |
|      10 |       3.850454 |
|      12 |       4.004593 |
|      14 |       3.943334 |
|      16 |       3.768768 |
|      24 |       3.331770 |

[Raw worker CSVs](./) | [Workload and timing](../../../../BENCHMARK.md#workloads-and-timing) | [Reproduction](../../../../BENCHMARK.md#reproduction)
