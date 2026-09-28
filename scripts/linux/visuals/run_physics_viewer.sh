#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/../lib/common.sh"
configuration=development
selection=""
screenshot=""
arguments=()
declare -A seen=()
while (($#)); do
  key="$1"
  [[ -z "${seen[$key]:-}" ]] || fail_usage "duplicate option: $key"
  seen[$key]=1
  (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || fail_usage "$key requires a value"
  value="$2"
  case "$key" in
    --configuration)
      [[ "$value" == development || "$value" == release ]] || fail_usage 'configuration must be development|release'
      configuration="$value" ;;
    --scenario|--recipe|--replay|--results)
      [[ -z "$selection" ]] || fail_usage 'choose only one input selection'
      selection="$key" ;;
    --compare|--package|--screenshot) ;;
    --frame)
      [[ "$value" =~ ^[0-9]{1,7}$ ]] && ((10#$value <= 1000000)) || fail_usage 'frame must be 0..1000000' ;;
    --memory-mib)
      [[ "$value" =~ ^[0-9]{2,5}$ ]] && ((10#$value >= 64 && 10#$value <= 16384)) || fail_usage 'memory-mib must be 64..16384' ;;
    --window-size)
      [[ "$value" =~ ^([0-9]{3,4}),([0-9]{3,4})$ ]] || fail_usage 'window-size must be WIDTH,HEIGHT'
      ((10#${BASH_REMATCH[1]} >= 640 && 10#${BASH_REMATCH[1]} <= 7680 && 10#${BASH_REMATCH[2]} >= 480 && 10#${BASH_REMATCH[2]} <= 4320)) || fail_usage 'window-size outside bounds' ;;
    *) fail_usage "unknown option: $key" ;;
  esac
  case "$key" in
    --recipe|--replay|--results|--compare|--screenshot)
      [[ "$value" == /* ]] || value="$ROOT/$value"
      if [[ "$key" == --screenshot ]]; then screenshot="$value"; fi ;;
  esac
  [[ "$key" == --configuration ]] || arguments+=("$key=$value")
  shift 2
done
[[ -z "${seen[--compare]:-}" || "$selection" == --replay ]] || fail_usage 'compare requires replay'
[[ -z "${seen[--package]:-}" || "$selection" == --results ]] || fail_usage 'package requires results'
[[ -z "${seen[--frame]:-}" || "$selection" == --replay || -n "$screenshot" ]] || fail_usage 'frame requires replay or screenshot'
exe="$ROOT/build/physics-viewer/linux/$configuration/physics_viewer"
[[ -x "$exe" ]] || fail_usage "build viewer first: $exe"
if [[ -n "$screenshot" ]]; then
  [[ ! -e "$screenshot" && ! -L "$screenshot" ]] || fail_usage 'screenshot exists'
  [[ -d "$(dirname "$screenshot")" ]] || fail_usage 'screenshot parent must exist'
  cd "$ROOT"
  exec timeout --foreground --kill-after=5 60 "$exe" "${arguments[@]}"
fi
cd "$ROOT"
exec "$exe" "${arguments[@]}"
