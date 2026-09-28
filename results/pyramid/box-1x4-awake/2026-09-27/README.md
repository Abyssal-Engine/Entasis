# Full pyramid, 1x4

- Configuration: `release`
- Run date (UTC): `2026-09-27`
- Odin compiler: `dev-2026-09-nightly:a2fb372`

Variant: `box-1x4-awake`

Windows 11, Intel Core i7-13700K, 24 logical processors. Workers include the caller, with unpinned placement

16,206 boxes, four sphere projectiles and one floor. No warmup and 600 measured steps using summed native-step timing, including projectile launch at step 120. 1 velocity iteration per substep and 4 substeps per 60 Hz step, sleeping disabled and recording off

Median of five process mean step times, in milliseconds per step. Lower is better

| Workers | Median ms/step |
| ------: | -------------: |
|       1 |      35.620657 |
|       2 |      19.827994 |
|       3 |      14.938140 |
|       4 |      12.501634 |
|       5 |      10.767997 |
|       6 |       9.555969 |
|       8 |       8.005019 |
|      10 |       8.933427 |
|      12 |       8.981039 |
|      14 |       8.759080 |
|      16 |       8.417103 |
|      24 |       7.485239 |

[Raw worker CSVs](./) | [Workload and timing](../../../../BENCHMARK.md#workloads-and-timing) | [Reproduction](../../../../BENCHMARK.md#reproduction)

Post-impact dispersal beyond the finite floor does not establish a physical-accuracy ranking
