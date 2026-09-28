#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/../lib/common.sh"

package=""
package_count=0
name=""
name_count=0
variant=""
variant_count=0
run_directory=""
run_directory_count=0
while (($# > 0)); do
  case "$1" in
    --run-directory)
      ((run_directory_count += 1))
      ((run_directory_count == 1)) || fail_usage "--run-directory may be supplied only once"
      (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || fail_usage "--run-directory requires a value"
      run_directory="$2"
      shift 2
      ;;
    --package)
      ((package_count += 1))
      ((package_count <= 1)) || fail_usage "--package may be supplied only once"
      (($# >= 2)) || fail_usage "--package requires a value"
      [[ "$2" != --* ]] || fail_usage "--package requires a value"
      package="$2"
      shift 2
      ;;
    --name)
      ((name_count += 1))
      ((name_count <= 1)) || fail_usage "--name may be supplied only once"
      (($# >= 2)) || fail_usage "--name requires a value"
      [[ "$2" != --* ]] || fail_usage "--name requires a value"
      name="$2"
      shift 2
      ;;
    --variant)
      ((variant_count += 1))
      ((variant_count == 1)) || fail_usage "--variant may be supplied only once"
      (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || fail_usage "--variant requires a value"
      variant="$2"
      shift 2
      ;;
    *) fail_usage "unknown option: $1" ;;
  esac
done
((run_directory_count == 1)) || fail_usage "missing required --run-directory"
((package_count == 1)) || fail_usage "missing required --package <package>"
((name_count == 1)) || fail_usage "missing required --name <name>"
benchmark_exists "$package" || fail_usage "unknown benchmark package: $package"
case "$package" in
  container|contact_islands|pyramid)
    [[ "$variant" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || fail_usage "--variant is required and must be a safe name" ;;
  *) ((variant_count == 0)) || fail_usage "--variant requires a configurable workload" ;;
esac
[[ "$name" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || fail_usage "invalid result name: $name"
[[ "$name" != "." && "$name" != ".." ]] || fail_usage "invalid result name: $name"

result_package="$package"
if [[ -n "$variant" ]]; then result_package+="/$variant"; fi
destination_parent="$ROOT/results/$result_package"
destination="$destination_parent/$name"
[[ ! -e "$destination" && ! -L "$destination" ]] || { printf 'error: promotion destination already exists: %s\n' "$destination" >&2; exit 1; }
source_result="$(realpath -m -- "$run_directory")"
release_root="$ROOT/build/benchmark-results/linux_amd64/$result_package/release"
[[ "${source_result%/*}" == "$release_root" && "$source_result" != *.pending && "$source_result" != *.failed ]] || fail_usage "--run-directory must select a completed Release run for this package and variant"
[[ -d "$source_result" && ! -L "$source_result" ]] || { printf 'error: complete Release run not found: %s\n' "$source_result" >&2; exit 1; }
[[ -f "$source_result/README.md" && ! -L "$source_result/README.md" ]] || { printf 'error: complete Release run not found: %s\n' "$source_result" >&2; exit 1; }
shopt -s nullglob
lane_files=("$source_result"/workers-*.csv)
shopt -u nullglob
(( ${#lane_files[@]} > 0 )) || { printf 'error: complete Release run not found: %s\n' "$source_result" >&2; exit 1; }
for path in "$source_result" "$destination"; do
  cursor="$path"
  while [[ "$cursor" != "$ROOT" && "$cursor" != / ]]; do
    [[ ! -L "$cursor" ]] || { printf 'error: promotion path is linked: %s\n' "$cursor" >&2; exit 1; }
    cursor="${cursor%/*}"
  done
done
for lane in "${lane_files[@]}"; do
  [[ ! -L "$lane" ]] || { printf 'error: promotion lane is linked: %s\n' "$lane" >&2; exit 1; }
done
recordings=()
declare -A seen_recordings=()
for lane in "${lane_files[@]}"; do
  # recording fields are appended by the native producers and cannot contain commas
  header="$(head -n 1 "$lane")"
  header="${header%$'\r'}"
  if [[ "$header" == *,recording_path,recording_component,recording_mode,timing_method ]]; then
    while IFS= read -r row; do
      row="${row%$'\r'}"
      mode="${row%,*}"; mode="${mode##*,}"
      path="${row%,*,*,*}"; path="${path##*,}"
      [[ -n "$path" ]] || continue
      [[ "$mode" == on && "$path" =~ ^[A-Za-z0-9_.-]+$ && "$path" != . && "$path" != .. && "$path" != *.partial ]] || { printf 'invalid recording association\n' >&2; exit 1; }
      [[ -z "${seen_recordings[$path]+present}" && -f "$source_result/$path" && ! -L "$source_result/$path" ]] || { printf 'missing, linked or duplicate recording: %s\n' "$path" >&2; exit 1; }
      seen_recordings[$path]=1
      recordings+=("$source_result/$path")
    done < <(tail -n +2 "$lane")
  elif [[ "$header" == *recording_path* ]]; then
    printf 'unsupported recording column layout\n' >&2
    exit 1
  fi
done
mkdir -p "$destination_parent"
mkdir "$destination"
if ! cp -p -- "$source_result/README.md" "${lane_files[@]}" "${recordings[@]}" "$destination/"; then
  rm -rf "$destination"
  exit 1
fi
printf 'BENCHMARK_PROMOTED source=%s destination=%s\n' "$source_result" "$destination"
