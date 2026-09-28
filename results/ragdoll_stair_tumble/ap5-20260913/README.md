# Ragdoll Stair-Tumble

- Configuration: `release`
- Run date (UTC): `2026-09-13`
- Odin compiler: `dev-2026-09:c59786ce8`

Windows 11, Intel Core i7-13700K, 24 logical processors. Workers include the caller, with unpinned placement

512 fifteen-body ragdolls, 7,168 ball sockets, 64 stairs and four catch-basin statics. 30 warmup steps in a disposable world, then 600 measured steps in a fresh world. 4x1 solver at 60 Hz

Median of five process mean step times, in milliseconds per step. Lower is better

| Workers | Median ms/step |
| ------: | -------------: |
|       1 |       6.662286 |
|       2 |       4.326049 |
|       3 |       3.518923 |
|       4 |       2.993225 |
|       5 |       2.638809 |
|       6 |       2.461467 |
|       8 |       2.132619 |
|      10 |       2.243594 |
|      12 |       2.295174 |
|      14 |       2.287193 |
|      16 |       2.259717 |
|      24 |       2.270396 |

[Raw worker CSVs](./) | [Workload and timing](../../../BENCHMARK.md#workloads-and-timing) | [Reproduction](../../../BENCHMARK.md#reproduction)
