# Full pyramid, 4x1

- Configuration: `release`
- Run date (UTC): `2026-09-27`
- Odin compiler: `dev-2026-09-nightly:a2fb372`

Variant: `box-4x1-awake`

Windows 11, Intel Core i7-13700K, 24 logical processors. Workers include the caller, with unpinned placement

16,206 boxes, four sphere projectiles and one floor. No warmup and 600 measured steps using summed native-step timing, including projectile launch at step 120. 4 velocity iterations per substep and 1 substep per 60 Hz step, sleeping disabled and recording off

Median of five process mean step times, in milliseconds per step. Lower is better

| Workers | Median ms/step |
| ------: | -------------: |
|       1 |      28.108629 |
|       2 |      15.243167 |
|       3 |      11.852377 |
|       4 |       9.819090 |
|       5 |       8.564632 |
|       6 |       7.640974 |
|       8 |       6.509744 |
|      10 |       6.991403 |
|      12 |       7.109722 |
|      14 |       7.017965 |
|      16 |       6.751774 |
|      24 |       6.155543 |

[Raw worker CSVs](./) | [Workload and timing](../../../../BENCHMARK.md#workloads-and-timing) | [Reproduction](../../../../BENCHMARK.md#reproduction)

Post-impact dispersal beyond the finite floor does not establish a physical-accuracy ranking
