# Box container, 4x1

- Configuration: `release`
- Run date (UTC): `2026-09-27`
- Odin compiler: `dev-2026-09-nightly:a2fb372`

Variant: `box-4x1-awake`

Windows 11, Intel Core i7-13700K, 24 logical processors. Workers include the caller, with unpinned placement

10,000 boxes and five statics. 30 warmup steps in a disposable world, then 300 measured steps in a fresh world using whole-loop timing. 4 velocity iterations per substep and 1 substep per 60 Hz step, sleeping disabled and recording off

Median of five process mean step times, in milliseconds per step. Lower is better

| Workers | Median ms/step |
| ------: | -------------: |
|       1 |      10.347018 |
|       2 |       6.313046 |
|       3 |       4.867344 |
|       4 |       4.153797 |
|       5 |       3.715510 |
|       6 |       3.349080 |
|       8 |       2.872174 |
|      10 |       3.034352 |
|      12 |       3.182395 |
|      14 |       3.148892 |
|      16 |       2.972920 |
|      24 |       2.738088 |

[Raw worker CSVs](./) | [Workload and timing](../../../../BENCHMARK.md#workloads-and-timing) | [Reproduction](../../../../BENCHMARK.md#reproduction)
