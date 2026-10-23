#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/../lib/common.sh"
source "$ROOT/scripts/linux/benchmarks/build_support.sh"

precision="float32"
precision_candidate=""
declare -A precision_seen=()
odin=""
odin_count=0
configuration="release"
configuration_count=0
package=""
package_count=0
record="off"
record_count=0
components="all"
components_count=0
while (($# > 0)); do
  case "$1" in
    --precision|--precision-candidate)
      option="$1"
      [[ -z "${precision_seen[$option]:-}" ]] || fail_usage "repeated option: $option"
      precision_seen[$option]=1
      (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || fail_usage "$option requires a value"
      if [[ "$option" == --precision ]]; then precision="$2"; else precision_candidate="$2"; fi
      shift 2
      ;;
    --components)
      ((components_count += 1))
      ((components_count == 1)) || fail_usage "--components may be supplied only once"
      (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || fail_usage "--components requires a value"
      components="$2"
      shift 2
      ;;
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
case "$precision" in float32|mixed64) ;; *) fail_usage "--precision must be float32 or mixed64" ;; esac
if [[ "$package" == large_world_precision ]]; then
  case "$precision_candidate" in baseline|origin) selected_precision=float32 ;; mixed64) selected_precision=mixed64 ;; *) fail_usage "precision package requires a supported candidate" ;; esac
  [[ -z "${precision_seen[--precision]:-}" || "$precision" == "$selected_precision" ]] || fail_usage "precision conflicts with candidate"
  precision="$selected_precision"
  ((components_count == 0)) || fail_usage "precision package does not accept components"
else
  [[ -z "$precision_candidate" ]] || fail_usage "--precision-candidate requires large_world_precision"
fi
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
[[ "$components" == common || "$components" == all ]] || fail_usage "--components must be common or all"
if ((components_count > 0)); then [[ "$package" == contact_islands ]] || fail_usage "--components requires contact_islands"; fi
require_odin "$odin"
packages=("${BENCHMARK_PACKAGES[@]}")
if ((package_count > 0)); then packages=("$package"); fi
printf 'BENCHMARK_BUILD_CONFIG configuration=%s packages=%s\n' "$configuration" "$(IFS=,; printf '%s' "${packages[*]}")"
for selected_package in "${packages[@]}"; do
  build_benchmark "$selected_package" "$configuration" "$record" "$components" "$precision" "$precision_candidate"
done
build_benchmark_reporter "$configuration"
printf 'BUILD_BENCHMARKS_OK\n'
