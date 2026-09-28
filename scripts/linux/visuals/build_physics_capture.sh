#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/../lib/common.sh"
configuration=development
source_root="$ROOT"
source_label=working-tree
output_directory=""
declare -A seen=()
while (($#)); do
  key="$1"
  [[ -z "${seen[$key]:-}" ]] || fail_usage "duplicate option: $key"
  seen[$key]=1
  (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || fail_usage "$key requires a value"
  case "$key" in
    --configuration) configuration="$2" ;;
    --source-root) source_root="$2" ;;
    --source-label) source_label="$2" ;;
    --output-directory) output_directory="$2" ;;
    *) fail_usage "unknown option: $key" ;;
  esac
  shift 2
done
[[ "$configuration" == development || "$configuration" == release ]] || fail_usage 'configuration must be development|release'
[[ "$source_label" =~ ^[a-zA-Z0-9._\ -]{1,120}$ ]] || fail_usage 'source label must be 1..120 plain ASCII characters'
[[ "$source_root" == /* ]] || source_root="$ROOT/$source_root"
source_root="$(realpath -e -- "$source_root")" || fail_usage 'source root does not exist'
[[ -d "$source_root/src/entasis" ]] || fail_usage 'source root has no src/entasis'
if [[ "$source_root" != "$ROOT" ]]; then
  [[ -n "${seen[--source-label]:-}" && -n "$output_directory" ]] || fail_usage 'alternate source requires source-label and output-directory'
  differences=0
  while IFS= read -r -d '' path; do
    relative="${path#"$ROOT/"}"
    if ! cmp -s -- "$path" "$source_root/$relative"; then
      printf 'Fixture differs: %s\n' "$relative" >&2
      differences=1
    fi
  done < <(find "$ROOT/examples/headless" -type f -name '*.odin' -print0)
  while IFS= read -r -d '' path; do
    relative="${path#"$source_root/"}"
    if [[ ! -f "$ROOT/$relative" ]]; then printf 'Extra fixture: %s\n' "$relative" >&2; differences=1; fi
  done < <(find "$source_root/examples/headless" -type f -name '*.odin' -print0)
  ((differences == 0)) || fail_usage 'alternate source fixtures differ'
fi
[[ -n "$output_directory" ]] || output_directory="build/physics-capture/linux/$configuration/current"
[[ "$output_directory" == /* ]] || output_directory="$ROOT/$output_directory"
[[ ! -e "$output_directory" && ! -L "$output_directory" ]] || fail_usage "output directory already exists: $output_directory"
require_odin ""
set_odin_profile "$configuration"
mkdir -p -- "$output_directory"
timeout --foreground 600 "$ODIN_BIN" build "$ROOT/tools/physics_capture" \
  -out:"$output_directory/physics_capture" -collection:entasis="$source_root/src" \
  -define:ENTASIS_REPLAY_SOURCE="$source_label" -define:ENTASIS_VISUAL_CONFIGURATION="$configuration" \
  -target:"$ODIN_TARGET" -microarch:"$ODIN_MICROARCH" "${ODIN_PROFILE_ARGUMENTS[@]}" \
  -vet -warnings-as-errors -thread-count:"$ODIN_THREAD_COUNT" -linker:lld \
  > >(tee "$output_directory/build.log") 2> >(tee "$output_directory/build.stderr.log" >&2)
printf 'PHYSICS_CAPTURE_BUILD_OK directory=%s\n' "$output_directory"
