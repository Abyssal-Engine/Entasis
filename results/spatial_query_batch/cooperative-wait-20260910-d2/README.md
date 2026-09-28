# Closest-ray batches

- Configuration: `release`
- Run date (UTC): `2026-09-10`
- Odin compiler: `dev-2026-09:c59786ce8`

Windows 11, Intel Core i7-13700K, 24 logical processors. Workers include the caller, with unpinned placement

50,000 closest rays per frame in batches of 256. Ten warmup frames, then 100 measured frames

Median of five process throughput measurements, in millions of rays per second. Higher is better

| Workers | Median M rays/s |
| ------: | --------------: |
|       1 |       10.235456 |
|       2 |       20.424895 |
|       3 |       30.232598 |
|       4 |       39.885226 |
|       5 |       49.702827 |
|       6 |       59.085103 |
|       8 |       76.117983 |
|      10 |       64.728438 |
|      12 |       72.432800 |
|      14 |       84.365119 |
|      16 |       96.633296 |
|      24 |      136.785807 |

[Raw worker CSVs](./) | [Workload and timing](../../../BENCHMARK.md#workloads-and-timing) | [Reproduction](../../../BENCHMARK.md#reproduction)
