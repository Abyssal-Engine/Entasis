#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/../lib/common.sh"
source "$ROOT/scripts/linux/benchmarks/build_support.sh"

odin=""
odin_count=0
configuration="release"
configuration_count=0
package=""
package_count=0
record="off"
record_count=0
while (($# > 0)); do
  case "$1" in
    --record)
      ((record_count += 1))
      ((record_count == 1)) || fail_usage "--record may be supplied only once"
      (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || fail_usage "--record requires a value"
      record="$2"
      shift 2
      ;;
    --odin)
      ((odin_count += 1))
      ((odin_count <= 1)) || fail_usage "--odin may be supplied only once"
      (($# >= 2)) || fail_usage "--odin requires a value"
      [[ -n "$2" && "$2" != --* ]] || fail_usage "--odin requires a non-empty value"
      odin="$2"
      shift 2
      ;;
    --configuration)
      ((configuration_count += 1))
      ((configuration_count <= 1)) || fail_usage "--configuration may be supplied only once"
      (($# >= 2)) || fail_usage "--configuration requires a value"
      configuration="$2"
      shift 2
      ;;
    --package)
      ((package_count += 1))
      ((package_count <= 1)) || fail_usage "--package may be supplied only once"
      (($# >= 2)) || fail_usage "--package requires a value"
      package="$2"
      shift 2
      ;;
    *) fail_usage "unknown option: $1" ;;
  esac
done
case "$record" in
  off|on) ;;
  *) fail_usage "--record must be off or on" ;;
esac
if [[ "$record" == on ]]; then
  case "$package" in
    container|contact_islands|pyramid|ragdoll_stair_tumble|noncontact_constraint_mix|noncontact_fallback_smoke) ;;
    *) fail_usage "recording requires one eligible explicit package" ;;
  esac
fi
case "$configuration" in
  development|release) ;;
  "") fail_usage "--configuration requires a non-empty value" ;;
  *) fail_usage "unknown configuration: $configuration" ;;
esac
if ((package_count > 0)); then benchmark_exists "$package" || fail_usage "unknown benchmark package: $package"; fi
require_odin "$odin"
packages=("${BENCHMARK_PACKAGES[@]}")
if ((package_count > 0)); then packages=("$package"); fi
printf 'BENCHMARK_BUILD_CONFIG configuration=%s packages=%s\n' "$configuration" "$(IFS=,; printf '%s' "${packages[*]}")"
for selected_package in "${packages[@]}"; do
  build_benchmark "$selected_package" "$configuration" "$record"
done
build_benchmark_reporter "$configuration"
printf 'BUILD_BENCHMARKS_OK\n'
