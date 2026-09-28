#!/usr/bin/env bash
set -euo pipefail
preparation_start="$(date +%s%3N)"
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/../lib/common.sh"
source "$ROOT/scripts/linux/benchmarks/build_support.sh"

package=""
package_count=0
runs=""
runs_count=0
workers_csv=""
workers_count=0
configuration="release"
configuration_count=0
declare -A workload_options=()
workload_arguments=()
reproduction_options=()
variant=""
record=off
compression=lz4
memory_mib=512
declare -A recording_seen=()
while (($# > 0)); do
  case "$1" in
    --record|--compression|--memory-mib)
      [[ -z "${recording_seen[$1]+present}" ]] || fail_usage "$1 may be supplied only once"
      (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || fail_usage "$1 requires a value"
      recording_seen[$1]=1
      case "$1" in
        --record) record="$2" ;;
        --compression) compression="$2" ;;
        --memory-mib) memory_mib="$2" ;;
      esac
      shift 2
      ;;
    --package)
      ((package_count += 1))
      ((package_count <= 1)) || fail_usage "--package may be supplied only once"
      (($# >= 2)) || fail_usage "--package requires a value"
      package="$2"
      shift 2
      ;;
    --runs)
      ((runs_count += 1))
      ((runs_count <= 1)) || fail_usage "--runs may be supplied only once"
      (($# >= 2)) || fail_usage "--runs requires a value"
      runs="$2"
      shift 2
      ;;
    --workers)
      ((workers_count += 1))
      ((workers_count <= 1)) || fail_usage "--workers may be supplied only once"
      (($# >= 2)) || fail_usage "--workers requires a value"
      workers_csv="$2"
      shift 2
      ;;
    --configuration)
      ((configuration_count += 1))
      ((configuration_count <= 1)) || fail_usage "--configuration may be supplied only once"
      (($# >= 2)) || fail_usage "--configuration requires a value"
      configuration="$2"
      shift 2
      ;;
    --shape)
      [[ -z "${workload_options[shape]+present}" ]] || fail_usage "--shape may be supplied only once"
      (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || fail_usage "--shape requires a value"
      workload_options[shape]="$2"
      reproduction_options+=("$1" "$2")
      workload_arguments+=("--shape=$2")
      shift 2
      ;;
    --static-shape)
      [[ -z "${workload_options[static-shape]+present}" ]] || fail_usage "--static-shape may be supplied only once"
      (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || fail_usage "--static-shape requires a value"
      workload_options[static-shape]="$2"
      reproduction_options+=("$1" "$2")
      workload_arguments+=("--static-shape=$2")
      shift 2
      ;;
    --shape-size)
      [[ -z "${workload_options[shape-size]+present}" ]] || fail_usage "--shape-size may be supplied only once"
      (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || fail_usage "--shape-size requires a value"
      workload_options[shape-size]="$2"
      reproduction_options+=("$1" "$2")
      workload_arguments+=("--shape-size=$2")
      shift 2
      ;;
    --density)
      [[ -z "${workload_options[density]+present}" ]] || fail_usage "--density may be supplied only once"
      (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || fail_usage "--density requires a value"
      workload_options[density]="$2"
      reproduction_options+=("$1" "$2")
      workload_arguments+=("--density=$2")
      shift 2
      ;;
    --layout-scale)
      [[ -z "${workload_options[layout-scale]+present}" ]] || fail_usage "--layout-scale may be supplied only once"
      (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || fail_usage "--layout-scale requires a value"
      workload_options[layout-scale]="$2"
      reproduction_options+=("$1" "$2")
      workload_arguments+=("--layout-scale=$2")
      shift 2
      ;;
    --steps)
      [[ -z "${workload_options[steps]+present}" ]] || fail_usage "--steps may be supplied only once"
      (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || fail_usage "--steps requires a value"
      workload_options[steps]="$2"
      reproduction_options+=("$1" "$2")
      workload_arguments+=("--steps=$2")
      shift 2
      ;;
    --timestep-hz)
      [[ -z "${workload_options[timestep-hz]+present}" ]] || fail_usage "--timestep-hz may be supplied only once"
      (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || fail_usage "--timestep-hz requires a value"
      workload_options[timestep-hz]="$2"
      reproduction_options+=("$1" "$2")
      workload_arguments+=("--timestep-hz=$2")
      shift 2
      ;;
    --warmup-steps)
      [[ -z "${workload_options[warmup-steps]+present}" ]] || fail_usage "--warmup-steps may be supplied only once"
      (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || fail_usage "--warmup-steps requires a value"
      workload_options[warmup-steps]="$2"
      reproduction_options+=("$1" "$2")
      workload_arguments+=("--warmup-steps=$2")
      shift 2
      ;;
    --velocity-iterations)
      [[ -z "${workload_options[velocity-iterations]+present}" ]] || fail_usage "--velocity-iterations may be supplied only once"
      (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || fail_usage "--velocity-iterations requires a value"
      workload_options[velocity-iterations]="$2"
      reproduction_options+=("$1" "$2")
      workload_arguments+=("--velocity-iterations=$2")
      shift 2
      ;;
    --substeps)
      [[ -z "${workload_options[substeps]+present}" ]] || fail_usage "--substeps may be supplied only once"
      (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || fail_usage "--substeps requires a value"
      workload_options[substeps]="$2"
      reproduction_options+=("$1" "$2")
      workload_arguments+=("--substeps=$2")
      shift 2
      ;;
    --sleep)
      [[ -z "${workload_options[sleep]+present}" ]] || fail_usage "--sleep may be supplied only once"
      (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || fail_usage "--sleep requires a value"
      workload_options[sleep]="$2"
      reproduction_options+=("$1" "$2")
      workload_arguments+=("--sleep=$2")
      shift 2
      ;;
    --grid)
      [[ -z "${workload_options[grid]+present}" ]] || fail_usage "--grid may be supplied only once"
      (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || fail_usage "--grid requires a value"
      workload_options[grid]="$2"
      reproduction_options+=("$1" "$2")
      workload_arguments+=("--grid=$2")
      shift 2
      ;;
    --spacing)
      [[ -z "${workload_options[spacing]+present}" ]] || fail_usage "--spacing may be supplied only once"
      (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || fail_usage "--spacing requires a value"
      workload_options[spacing]="$2"
      reproduction_options+=("$1" "$2")
      workload_arguments+=("--spacing=$2")
      shift 2
      ;;
    --spawn-height)
      [[ -z "${workload_options[spawn-height]+present}" ]] || fail_usage "--spawn-height may be supplied only once"
      (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || fail_usage "--spawn-height requires a value"
      workload_options[spawn-height]="$2"
      reproduction_options+=("$1" "$2")
      workload_arguments+=("--spawn-height=$2")
      shift 2
      ;;
    --container-size)
      [[ -z "${workload_options[container-size]+present}" ]] || fail_usage "--container-size may be supplied only once"
      (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || fail_usage "--container-size requires a value"
      workload_options[container-size]="$2"
      reproduction_options+=("$1" "$2")
      workload_arguments+=("--container-size=$2")
      shift 2
      ;;
    --island-grid)
      [[ -z "${workload_options[island-grid]+present}" ]] || fail_usage "--island-grid may be supplied only once"
      (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || fail_usage "--island-grid requires a value"
      workload_options[island-grid]="$2"
      reproduction_options+=("$1" "$2")
      workload_arguments+=("--island-grid=$2")
      shift 2
      ;;
    --island-spacing)
      [[ -z "${workload_options[island-spacing]+present}" ]] || fail_usage "--island-spacing may be supplied only once"
      (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || fail_usage "--island-spacing requires a value"
      workload_options[island-spacing]="$2"
      reproduction_options+=("$1" "$2")
      workload_arguments+=("--island-spacing=$2")
      shift 2
      ;;
    --floor-size)
      [[ -z "${workload_options[floor-size]+present}" ]] || fail_usage "--floor-size may be supplied only once"
      (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || fail_usage "--floor-size requires a value"
      workload_options[floor-size]="$2"
      reproduction_options+=("$1" "$2")
      workload_arguments+=("--floor-size=$2")
      shift 2
      ;;
    --rows)
      [[ -z "${workload_options[rows]+present}" ]] || fail_usage "--rows may be supplied only once"
      (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || fail_usage "--rows requires a value"
      workload_options[rows]="$2"
      reproduction_options+=("$1" "$2")
      workload_arguments+=("--rows=$2")
      shift 2
      ;;
    --projectile-count)
      [[ -z "${workload_options[projectile-count]+present}" ]] || fail_usage "--projectile-count may be supplied only once"
      (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || fail_usage "--projectile-count requires a value"
      workload_options[projectile-count]="$2"
      reproduction_options+=("$1" "$2")
      workload_arguments+=("--projectile-count=$2")
      shift 2
      ;;
    --launch-step)
      [[ -z "${workload_options[launch-step]+present}" ]] || fail_usage "--launch-step may be supplied only once"
      (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || fail_usage "--launch-step requires a value"
      workload_options[launch-step]="$2"
      reproduction_options+=("$1" "$2")
      workload_arguments+=("--launch-step=$2")
      shift 2
      ;;
    --projectile-radius)
      [[ -z "${workload_options[projectile-radius]+present}" ]] || fail_usage "--projectile-radius may be supplied only once"
      (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || fail_usage "--projectile-radius requires a value"
      workload_options[projectile-radius]="$2"
      reproduction_options+=("$1" "$2")
      workload_arguments+=("--projectile-radius=$2")
      shift 2
      ;;
    --projectile-density)
      [[ -z "${workload_options[projectile-density]+present}" ]] || fail_usage "--projectile-density may be supplied only once"
      (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || fail_usage "--projectile-density requires a value"
      workload_options[projectile-density]="$2"
      reproduction_options+=("$1" "$2")
      workload_arguments+=("--projectile-density=$2")
      shift 2
      ;;
    --projectile-center)
      [[ -z "${workload_options[projectile-center]+present}" ]] || fail_usage "--projectile-center may be supplied only once"
      (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || fail_usage "--projectile-center requires a value"
      workload_options[projectile-center]="$2"
      reproduction_options+=("$1" "$2")
      workload_arguments+=("--projectile-center=$2")
      shift 2
      ;;
    --projectile-spacing)
      [[ -z "${workload_options[projectile-spacing]+present}" ]] || fail_usage "--projectile-spacing may be supplied only once"
      (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || fail_usage "--projectile-spacing requires a value"
      workload_options[projectile-spacing]="$2"
      reproduction_options+=("$1" "$2")
      workload_arguments+=("--projectile-spacing=$2")
      shift 2
      ;;
    --projectile-velocity)
      [[ -z "${workload_options[projectile-velocity]+present}" ]] || fail_usage "--projectile-velocity may be supplied only once"
      (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || fail_usage "--projectile-velocity requires a value"
      workload_options[projectile-velocity]="$2"
      reproduction_options+=("$1" "$2")
      workload_arguments+=("--projectile-velocity=$2")
      shift 2
      ;;
    --variant)
      [[ -z "${workload_options[variant]+present}" ]] || fail_usage "--variant may be supplied only once"
      (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || fail_usage "--variant requires a value"
      workload_options[variant]="$2"
      reproduction_options+=("$1" "$2")
      variant="$2"
      shift 2
      ;;
    *) fail_usage "unknown option: $1" ;;
  esac
done

((package_count == 1)) || fail_usage "missing required --package <package>"
((runs_count == 1)) || fail_usage "missing required --runs <n>"
((workers_count == 1)) || fail_usage "missing required --workers <csv>"
benchmark_exists "$package" || fail_usage "unknown benchmark package: $package"
validate_positive_int32 "$runs" || fail_usage "runs must be an integer in 1..2147483647"
[[ "$workers_csv" =~ ^[1-9][0-9]*(,[1-9][0-9]*)*$ ]] || fail_usage "invalid worker list: $workers_csv"
case "$configuration" in
  development|release) ;;
  "") fail_usage "--configuration requires a non-empty value" ;;
  *) fail_usage "unknown configuration: $configuration" ;;
esac
IFS=',' read -r -a workers <<< "$workers_csv"
declare -A seen_workers=()
for worker_count in "${workers[@]}"; do
  validate_worker_count "$worker_count" || fail_usage "worker $worker_count exceeds the $AVAILABLE_WORKER_COUNT CPUs available to this process"
  [[ -z "${seen_workers[$worker_count]:-}" ]] || fail_usage "duplicate worker count: $worker_count"
  seen_workers[$worker_count]=1
done
case "$package" in
  container|contact_islands|pyramid)
    [[ -n "${workload_options[shape]:-}" && -n "$variant" ]] || fail_usage "--shape and --variant are required"
    ;;
  *) ((${#workload_options[@]} == 0)) || fail_usage "shape/configuration/variant options require container, contact_islands or pyramid" ;;
esac
if [[ -n "$variant" ]]; then
  [[ "$variant" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || fail_usage "invalid variant: $variant"
fi
number_pattern='[+-]?([0-9]+([.][0-9]*)?|[.][0-9]+)([eE][+-]?[0-9]+)?'
for option in "${!workload_options[@]}"; do
  value="${workload_options[$option]}"
  case "$option" in
    shape) [[ "$value" =~ ^(box|sphere|capsule|cylinder|hull)$ ]] || fail_usage "invalid --shape" ;;
    static-shape) [[ "$value" =~ ^(box|hull)$ ]] || fail_usage "invalid --static-shape" ;;
    sleep) [[ "$value" =~ ^(enabled|disabled)$ ]] || fail_usage "invalid --sleep" ;;
    variant) ;;
    grid) [[ "$value" =~ ^[0-9]+,[0-9]+,[0-9]+$ ]] || fail_usage "invalid --grid" ;;
    island-grid) [[ "$value" =~ ^[0-9]+,[0-9]+$ ]] || fail_usage "invalid --island-grid" ;;
    steps|timestep-hz|warmup-steps|velocity-iterations|substeps|rows|projectile-count|launch-step)
      [[ "$value" =~ ^[0-9]+$ ]] || fail_usage "invalid --$option" ;;
    shape-size|spacing|container-size|floor-size|projectile-center|projectile-spacing|projectile-velocity)
      [[ "$value" =~ ^$number_pattern,$number_pattern,$number_pattern$ ]] || fail_usage "invalid --$option" ;;
    island-spacing) [[ "$value" =~ ^$number_pattern,$number_pattern$ ]] || fail_usage "invalid --$option" ;;
    *) [[ "$value" =~ ^$number_pattern$ ]] || fail_usage "invalid --$option" ;;
  esac
  case "$option" in
    container-size) [[ "$package" == container ]] || fail_usage "--$option is container-only" ;;
    island-grid|island-spacing) [[ "$package" == contact_islands ]] || fail_usage "--$option is contact_islands-only" ;;
    floor-size) [[ "$package" != container ]] || fail_usage "--floor-size is not a container option" ;;
    grid|spacing|spawn-height) [[ "$package" != pyramid ]] || fail_usage "--$option is not a pyramid option" ;;
    rows|projectile-*|launch-step) [[ "$package" == pyramid ]] || fail_usage "--$option is pyramid-only" ;;
  esac
done
[[ "$record" == off || "$record" == on ]] || fail_usage "--record must be off or on"
[[ "$compression" == lz4 || "$compression" == lz4hc ]] || fail_usage "invalid compression"
[[ "$memory_mib" =~ ^[0-9]{1,5}$ ]] && ((10#$memory_mib >= 64 && 10#$memory_mib <= 16384)) || fail_usage "invalid memory budget"
recording_capable=unavailable
case "$package" in
  container|contact_islands|pyramid|ragdoll_stair_tumble|noncontact_constraint_mix|noncontact_fallback_smoke) recording_capable=available ;;
esac
[[ "$record" == off || "$recording_capable" == available ]] || fail_usage "recording unsupported for this package"
if [[ "$record" == off && ( -n "${recording_seen[--compression]+present}" || -n "${recording_seen[--memory-mib]+present}" ) ]]; then
  fail_usage "recording settings require --record on"
fi
binary="$(benchmark_binary "$package" "$configuration" "$record")"
reporter="$(benchmark_reporter_binary "$configuration")"
build_command="./scripts/linux/benchmarks/build_benchmarks.sh --package $package --configuration $configuration --record $record"
[[ -x "$binary" ]] || fail_usage "missing benchmark binary: $binary. Build with: $build_command"
[[ -x "$reporter" ]] || fail_usage "missing benchmark reporter: $reporter. Build with: $build_command"
compiler_version="Unavailable (reused producer binary)"
operating_system="$(uname -srmo)"
processor="$(awk -F ': ' '/^model name/{print $2; exit}' /proc/cpuinfo)"
[[ -n "$processor" ]] || fail_usage "required processor provenance is unavailable"
logical_processors="$AVAILABLE_WORKER_COUNT"
result_package="$package"
if [[ -n "$variant" ]]; then result_package+="/$variant"; fi
result_parent="$ROOT/build/benchmark-results/linux_amd64/$result_package/$configuration"
mkdir -p "$result_parent"
pending="$(mktemp -d "$result_parent/$(date -u +%Y%m%dT%H%M%SZ).XXXXXXXX.pending")"
current="${pending%.pending}"
export MALLOC_ARENA_MAX=1
printf 'BENCHMARK_RUN_CONFIG package=%s configuration=%s pending=%s\n' \
  "$package" "$configuration" "$pending"
printf 'BENCHMARK_PREPARED elapsed_ms=%s\n' "$(( $(date +%s%3N) - preparation_start ))"
workload_start="$(date +%s%3N)"
for worker_count in "${workers[@]}"; do
  output="$pending/workers-${worker_count}.csv"
  for ((run_index = 1; run_index <= runs; ++run_index)); do
    printf '[benchmark] %s workers=%s run=%s/%s\n' "$package" "$worker_count" "$run_index" "$runs"
    recording_arguments=()
    if [[ "$recording_capable" == available ]]; then
      recording_arguments+=("--record=$record")
      if [[ "$record" == on ]]; then
        recording_arguments+=("--recording-output=$pending/workers-$worker_count-sample-$run_index.epr" "--compression=$compression" "--memory-mib=$memory_mib")
      fi
    fi
    "$binary" --worker-count="$worker_count" --output="$output" "${workload_arguments[@]}" "${recording_arguments[@]}"
    if [[ "$recording_capable" == available ]]; then
      header="$(head -n 1 "$output")"
      header="${header%$'\r'}"
      [[ "$header" == *,recording_path,recording_component,recording_mode,timing_method ]] || { printf 'producer recording capability missing\n' >&2; exit 1; }
      [[ "$(wc -l < "$output")" -eq $((run_index + 1)) ]] || { printf 'unexpected sample count\n' >&2; exit 1; }
      last_row="$(tail -n 1 "$output")"
      last_row="${last_row%$'\r'}"
      actual_mode="${last_row%,*}"; actual_mode="${actual_mode##*,}"
      [[ "$actual_mode" == "$record" ]] || { printf 'producer recording mode mismatch\n' >&2; exit 1; }
    fi
    if [[ "$record" == on ]]; then
      actual_path="${last_row%,*,*,*}"; actual_path="${actual_path##*,}"
      [[ "$actual_path" == "workers-$worker_count-sample-$run_index.epr" ]] || { printf 'recording does not belong to this sample\n' >&2; exit 1; }
      [[ -f "$pending/workers-$worker_count-sample-$run_index.epr" ]] || { printf 'missing finalized recording\n' >&2; exit 1; }
    fi
  done
done
canonical_command="./scripts/linux/benchmarks/run_benchmark.sh --package $package --runs $runs --workers $workers_csv --configuration $configuration"
canonical_command+=" --record $record"
if [[ "$record" == on ]]; then canonical_command+=" --compression $compression --memory-mib $memory_mib"; fi
for option in "${reproduction_options[@]}"; do
  printf -v quoted_option '%q' "$option"
  canonical_command+=" $quoted_option"
done
report_options=()
if [[ -n "$variant" ]]; then report_options+=(--variant "$variant"); fi
report_start="$(date +%s%3N)"
printf 'BENCHMARK_STAGE stage=report workload_ms=%s\n' "$(( report_start - workload_start ))"
"$reporter" "${report_options[@]}" \
  --source "$pending" \
  --package "$package" \
  --configuration "$configuration" \
  --runs "$runs" \
  --workers "$workers_csv" \
  --compiler-version "$compiler_version" \
  --operating-system "$operating_system" \
  --processor "$processor" \
  --logical-processors "$logical_processors" \
  --command "$canonical_command"
printf 'BENCHMARK_REPORT elapsed_ms=%s\n' "$(( $(date +%s%3N) - report_start ))"
printf 'BENCHMARK_ELAPSED milliseconds=%s\n' "$(( $(date +%s%3N) - workload_start ))"
commit_benchmark_result "$pending" "$current"
printf 'BENCHMARK_OK result=%s report=%s/README.md\n' "$current" "$current"
