#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/../lib/common.sh"
build_directory=""
output=""
selection=""
timeout_seconds=120
arguments=()
declare -A seen=()
while (($#)); do
  key="$1"
  [[ -z "${seen[$key]:-}" ]] || fail_usage "duplicate option: $key"
  seen[$key]=1
  (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || fail_usage "$key requires a value"
  value="$2"
  case "$key" in
    --build-directory) build_directory="$value" ;;
    --output) output="$value" ;;
    --scenario|--recipe)
      [[ -z "$selection" ]] || fail_usage 'choose exactly one scenario or recipe'
      selection="$key"
      if [[ "$key" == --recipe && "$value" != /* ]]; then value="$ROOT/$value"; fi
      arguments+=("$key=$value") ;;
    --frames)
      [[ "$value" =~ ^[1-9][0-9]{0,6}$ ]] && ((value <= 1000000)) || fail_usage 'frames must be 1..1000000'
      arguments+=("$key=$value") ;;
    --compression)
      [[ "$value" == lz4 || "$value" == lz4hc ]] || fail_usage 'compression must be lz4|lz4hc'
      arguments+=("$key=$value") ;;
    --memory-mib)
      [[ "$value" =~ ^[0-9]{2,5}$ ]] && ((10#$value >= 64 && 10#$value <= 16384)) || fail_usage 'memory-mib must be 64..16384'
      arguments+=("$key=$value") ;;
    --timeout-seconds)
      [[ "$value" =~ ^[1-9][0-9]{0,3}$ ]] && ((value <= 3600)) || fail_usage 'timeout must be 1..3600 seconds'
      timeout_seconds="$value" ;;
    *) fail_usage "unknown option: $key" ;;
  esac
  shift 2
done
[[ -n "$build_directory" && -n "$output" && -n "$selection" ]] || fail_usage 'build-directory, output and scenario or recipe are required'
[[ "$build_directory" == /* ]] || build_directory="$ROOT/$build_directory"
[[ "$output" == /* ]] || output="$ROOT/$output"
[[ -x "$build_directory/physics_capture" ]] || fail_usage "missing capture executable: $build_directory/physics_capture"
for path in "$output" "$output.partial" "$output.log" "$output.stderr.log"; do
  [[ ! -e "$path" && ! -L "$path" ]] || fail_usage "output already exists: $path"
done
[[ -d "$(dirname "$output")" ]] || fail_usage 'output parent directory must exist'
cd "$ROOT"
set +e
timeout --foreground --kill-after=5 "$timeout_seconds" "$build_directory/physics_capture" "${arguments[@]}" "--output=$output" \
  > "$output.log" 2> "$output.stderr.log"
status=$?
set -e
cat -- "$output.log"
cat -- "$output.stderr.log" >&2
if ((status == 124 || status == 137)); then printf 'Capture incomplete: timeout, retained %s.partial and logs\n' "$output" >&2; fi
exit "$status"
