# Mixed spatial queries

- Configuration: `release`
- Run date (UTC): `2026-09-10`
- Odin compiler: `dev-2026-09:c59786ce8`

Windows 11, Intel Core i7-13700K, 24 logical processors. Workers include the caller, with unpinned placement

10,000 static boxes and 100,000 queries per batch: 50,000 closest rays, 25,000 sphere casts and 25,000 overlap queries. Ten warmup batches, then 100 measured batches

Median of five process throughput measurements, in millions of queries per second. Higher is better

| Workers | Median M queries/s |
| ------: | -----------------: |
|       1 |          11.839490 |
|       2 |          23.492984 |
|       3 |          34.454779 |
|       4 |          46.067898 |
|       5 |          55.747730 |
|       6 |          65.585554 |
|       8 |          84.144628 |
|      10 |          72.491346 |
|      12 |          83.453506 |
|      14 |          92.670333 |
|      16 |         106.287207 |
|      24 |         157.396040 |

[Raw worker CSVs](./) | [Workload and timing](../../../BENCHMARK.md#workloads-and-timing) | [Reproduction](../../../BENCHMARK.md#reproduction)
