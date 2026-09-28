#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
ODIN_TARGET="linux_amd64"
ODIN_MICROARCH="x86-64-v3"

fail_usage() {
  printf 'error: %s\n' "$*" >&2
  exit 2
}

detect_available_worker_count() {
  local detected=1
  if command -v nproc >/dev/null 2>&1; then
    detected="$(nproc)"
  elif command -v getconf >/dev/null 2>&1; then
    detected="$(getconf _NPROCESSORS_ONLN 2>/dev/null || printf '1')"
  fi
  if ! [[ "$detected" =~ ^[1-9][0-9]*$ ]]; then detected=1; fi
  if ((detected > 64)); then detected=64; fi
  printf '%s\n' "$detected"
}

AVAILABLE_WORKER_COUNT="$(detect_available_worker_count)"
ODIN_TEST_THREADS="$AVAILABLE_WORKER_COUNT"
ODIN_THREAD_COUNT="$AVAILABLE_WORKER_COUNT"
if ((ODIN_THREAD_COUNT > 8)); then ODIN_THREAD_COUNT=8; fi

BENCHMARK_PACKAGES=(
  awakening_duplicate_sets
  container
  contact_churn_grid_4k
  noncontact_constraint_mix
  noncontact_fallback_smoke
  shape_mixed_bounds
  spatial_query_trace
  spatial_query_batch
  contact_islands
  ragdoll_stair_tumble
  pyramid
)
TEST_PACKAGES=(
  root
  benchmark_support
  benchmark_report
  physics_visuals
  cooking
  bodies
  broadphase
  collections
  collision_batching
  collision_pairs
  contact_optimization
  constraints
  intrinsics
  islands
  layout
  narrowphase_integration
  queries
  public_api
  release_parity
  shapes
  simulation
  solver_kernels
  sweeps
  tasking_multi
  tasking_single
  threading
  trees
  utilities_bundles
  utilities_math
  utilities_memory
)
CODEGEN_PACKAGES=(
  bodies/codegen
  simulation/codegen
  solver_kernels/codegen
)

require_odin() {
  source "$ROOT/scripts/linux/lib/toolchain_common.sh"
  toolchain_odin "$1"
}

test_scope_to_package() {
  case "$1" in
    root|benchmark-support|benchmark-report|physics-visuals|physics-viewer|cooking|bodies|broadphase|collections|collision-batching|collision-pairs|contact-optimization|constraints|intrinsics|islands|layout|narrowphase-integration|queries|public-api|release-parity|shapes|simulation|solver-kernels|sweeps|tasking-multi|tasking-single|threading|trees|utilities-bundles|utilities-math|utilities-memory)
      printf '%s\n' "${1//-/_}"
      ;;
    *) return 1 ;;
  esac
}

test_package_path() {
  if [[ "$1" == root ]]; then
    printf '%s/tests\n' "$ROOT"
  elif [[ "$1" == benchmark_support ]]; then
    printf '%s/benchmarks/benchmark_support\n' "$ROOT"
  elif [[ "$1" == benchmark_report ]]; then
    printf '%s/tools/benchmark_report\n' "$ROOT"
  elif [[ "$1" == physics_viewer ]]; then
    printf '%s/tools/physics_viewer\n' "$ROOT"
  else
    printf '%s/tests/%s\n' "$ROOT" "$1"
  fi
}

safe_name() {
  printf '%s' "$1" | tr '/\\' '__'
}

benchmark_exists() {
  local candidate
  for candidate in "${BENCHMARK_PACKAGES[@]}"; do
    [[ "$candidate" == "$1" ]] && return 0
  done
  return 1
}

example_packages() {
  local path package
  local -a packages=()
  for path in "$ROOT"/examples/headless/*/main.odin; do
    [[ -f "$path" ]] || continue
    package="${path%/main.odin}"
    package="${package##*/}"
    [[ "$package" =~ ^[a-z][a-z0-9_]*$ ]] || fail_usage "invalid example package path: $path"
    packages+=("$package")
  done
  ((${#packages[@]} > 0)) || fail_usage "no example packages in $ROOT/examples/headless"
  printf '%s\n' "${packages[@]}" | LC_ALL=C sort
}

example_exists() {
  [[ "$1" =~ ^[a-z][a-z0-9_]*$ && -f "$ROOT/examples/headless/$1/main.odin" ]]
}

example_binary() {
  local package="$1"
  local configuration="$2"
  case "$configuration" in
    development|release) printf '%s/build/examples/%s/%s' "$ROOT" "$configuration" "$package" ;;
    *) fail_usage "unknown build configuration: $configuration" ;;
  esac
}

validate_positive_integer() {
  [[ "$1" =~ ^[1-9][0-9]*$ ]]
}

validate_positive_int32() {
  local value="$1"
  validate_positive_integer "$value" || return 1
  if ((${#value} < 10)); then
    return 0
  fi
  ((${#value} == 10)) || return 1
  [[ "$value" < "2147483647" || "$value" == "2147483647" ]]
}

validate_worker_count() {
  validate_positive_integer "$1" && ((10#$1 <= AVAILABLE_WORKER_COUNT))
}

benchmark_result_state() {
  local path="$1"
  if [[ ! -e "$path" && ! -L "$path" ]]; then
    printf 'absent\n'
    return 0
  fi
  if [[ -d "$path" && -s "$path/README.md" ]]; then
    local lane_path
    lane_path="$(find "$path" -maxdepth 1 -type f -name 'workers-*.csv' -size +0c -print -quit)"
    if [[ -n "$lane_path" ]]; then
      printf 'complete\n'
      return 0
    fi
  fi
  printf 'incomplete\n'
}

commit_benchmark_result() {
  local pending="$1"
  local completed="$2"
  [[ "$(benchmark_result_state "$pending")" == complete ]] || {
    printf 'Incomplete pending benchmark result: %s\n' "$pending" >&2
    return 1
  }
  [[ ! -e "$completed" && ! -L "$completed" ]] || {
    printf 'Completed benchmark destination already exists: %s\n' "$completed" >&2
    return 1
  }
  mv -T -- "$pending" "$completed"
}

validate_test_selector_list() {
  [[ "$1" =~ ^[A-Za-z_][A-Za-z0-9_]*(\.[A-Za-z_][A-Za-z0-9_]*)?(,[A-Za-z_][A-Za-z0-9_]*(\.[A-Za-z_][A-Za-z0-9_]*)?)*$ ]]
}

validate_test_thread_count() {
  validate_positive_integer "$1" && ((10#$1 <= AVAILABLE_WORKER_COUNT))
}

set_odin_profile() {
  local configuration="$1"
  case "$configuration" in
    development)
      ODIN_PROFILE_ARGUMENTS=(-debug -o:none -source-code-locations:normal)
      ;;
    release)
      ODIN_PROFILE_ARGUMENTS=(-o:speed -no-bounds-check -disable-assert -source-code-locations:none)
      ;;
    *) fail_usage "unknown build configuration: $configuration" ;;
  esac
}

configure_odin_profile() {
  local artifact_group="$1"
  local package="$2"
  local configuration="$3"
  local output_dir="$ROOT/build/$artifact_group/$configuration"
  mkdir -p "$output_dir"
  ODIN_PROFILE_OUTPUT="$output_dir/$(safe_name "$package")"
  rm -f "$ODIN_PROFILE_OUTPUT" "$ODIN_PROFILE_OUTPUT.o"
  set_odin_profile "$configuration"
  printf 'BUILD_PROFILE package=%s configuration=%s executable=%s symbols=embedded\n' \
    "$package" "$configuration" "$ODIN_PROFILE_OUTPUT"
}

run_test_package() {
  local package="$1"
  local configuration="$2"
  local test_names="${3:-}"
  local test_threads="${4:-$ODIN_TEST_THREADS}"
  local path
  path="$(test_package_path "$package")"
  configure_odin_profile tests "$package" "$configuration"
  printf '[test] %s\n' "$package"
  local arguments=(
    test "$path"
    -out:"$ODIN_PROFILE_OUTPUT"
    -keep-executable
    -collection:entasis="$ROOT/src"
    -target:"$ODIN_TARGET"
    -microarch:"$ODIN_MICROARCH"
    "${ODIN_PROFILE_ARGUMENTS[@]}"
    -vet
    -warnings-as-errors
    -thread-count:"$ODIN_THREAD_COUNT"
    -linker:lld
    -define:ODIN_TEST_THREADS="$test_threads"
    -define:ODIN_TEST_FANCY=false
  )
  if [[ -n "$test_names" ]]; then
    arguments+=( -define:ODIN_TEST_NAMES="$test_names" )
  fi
  if [[ "$package" == physics_viewer ]]; then
    timeout --foreground 180 "$ODIN_BIN" "${arguments[@]}"
  else
    "$ODIN_BIN" "${arguments[@]}"
  fi
}

run_codegen_package() {
  local package="$1"
  local configuration="$2"
  configure_odin_profile codegen "$package" "$configuration"
  printf '[codegen] %s\n' "$package"
  "$ODIN_BIN" build "$ROOT/tests/$package" \
    -out:"$ODIN_PROFILE_OUTPUT" \
    -collection:entasis="$ROOT/src" \
    -target:"$ODIN_TARGET" \
    -microarch:"$ODIN_MICROARCH" \
    "${ODIN_PROFILE_ARGUMENTS[@]}" \
    -vet \
    -warnings-as-errors \
    -thread-count:"$ODIN_THREAD_COUNT" \
    -linker:lld
  "$ODIN_PROFILE_OUTPUT"
}
