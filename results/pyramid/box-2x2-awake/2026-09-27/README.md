# Full pyramid, 2x2

- Configuration: `release`
- Run date (UTC): `2026-09-27`
- Odin compiler: `dev-2026-09-nightly:a2fb372`

Variant: `box-2x2-awake`

Windows 11, Intel Core i7-13700K, 24 logical processors. Workers include the caller, with unpinned placement

16,206 boxes, four sphere projectiles and one floor. No warmup and 600 measured steps using summed native-step timing, including projectile launch at step 120. 2 velocity iterations per substep and 2 substeps per 60 Hz step, sleeping disabled and recording off

Median of five process mean step times, in milliseconds per step. Lower is better

| Workers | Median ms/step |
| ------: | -------------: |
|       1 |      31.440918 |
|       2 |      16.968097 |
|       3 |      13.126816 |
|       4 |      10.815479 |
|       5 |       9.373924 |
|       6 |       8.339649 |
|       8 |       7.240036 |
|      10 |       7.804723 |
|      12 |       7.805863 |
|      14 |       7.585698 |
|      16 |       7.304892 |
|      24 |       6.624692 |

[Raw worker CSVs](./) | [Workload and timing](../../../../BENCHMARK.md#workloads-and-timing) | [Reproduction](../../../../BENCHMARK.md#reproduction)

Post-impact dispersal beyond the finite floor does not establish a physical-accuracy ranking
