#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/../lib/common.sh"
configuration=development
if (($#)); then
  (($# == 2)) && [[ "$1" == --configuration && ( "$2" == development || "$2" == release ) ]] || fail_usage 'usage: build_physics_viewer.sh [--configuration development|release]'
  configuration="$2"
fi
require_odin ""
configure_odin_profile physics-viewer/linux physics_viewer "$configuration"
timeout --foreground 600 "$ODIN_BIN" build "$ROOT/tools/physics_viewer" \
  -out:"$ODIN_PROFILE_OUTPUT" -collection:entasis="$ROOT/src" \
  -define:ENTASIS_VISUAL_CONFIGURATION="$configuration" -define:ENTASIS_BENCHMARK_COMPONENTS=all \
  -target:"$ODIN_TARGET" -microarch:"$ODIN_MICROARCH" "${ODIN_PROFILE_ARGUMENTS[@]}" \
  -vet -warnings-as-errors -thread-count:"$ODIN_THREAD_COUNT" -linker:lld
printf 'PHYSICS_VIEWER_BUILD_OK executable=%s\n' "$ODIN_PROFILE_OUTPUT"
