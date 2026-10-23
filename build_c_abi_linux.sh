#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/scripts/linux/lib/common.sh"
configuration="release" odin="" precision="float32"
declare -A seen=()
while (($#)); do
  option="$1"
  case "$option" in --configuration|--odin|--precision) ;; *) fail_usage "unknown option: $option" ;; esac
  [[ -z "${seen[$option]:-}" ]] || fail_usage "repeated option: $option"
  seen[$option]=1
  (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || fail_usage "$option requires a value"
  case "$option" in --configuration) configuration="$2" ;; --odin) odin="$2" ;; --precision) precision="$2" ;; esac
  shift 2
done
case "$configuration" in development|release) ;; *) fail_usage "--configuration must be development or release" ;; esac
case "$precision" in float32|mixed64) ;; *) fail_usage "--precision must be float32 or mixed64" ;; esac
require_odin "$odin"
cd "$ROOT"
"$ROOT/scripts/linux/abi/generate_abi.sh" --mode check --odin "$ODIN_BIN"
BUILD_DIR="$ROOT/build/c_abi/linux/$configuration"
if [[ "$precision" == mixed64 ]]; then BUILD_DIR+="/mixed64"; fi
mkdir -p "$BUILD_DIR"
profile=(-debug -o:none -source-code-locations:normal)
if [[ "$configuration" == release ]]; then profile=(-o:speed -no-bounds-check -disable-assert -source-code-locations:none); fi
common=(-define:ENTASIS_PRECISION="$precision" -collection:entasis="$ROOT/src" -target:"$ODIN_TARGET" -microarch:"$ODIN_MICROARCH" -no-entry-point -thread-count:"$ODIN_THREAD_COUNT" -vet -warnings-as-errors)
printf 'C_ABI_BUILD_PROFILE configuration=%s root=%s target=%s microarch=%s\n' "$configuration" "$BUILD_DIR" "$ODIN_TARGET" "$ODIN_MICROARCH"
export_directory="$ROOT/generated/abi"
if [[ "$precision" == mixed64 ]]; then export_directory+="/mixed64"; fi
for library in entasis entasis_cooking; do
  rm -f "$BUILD_DIR/lib$library.so" "$BUILD_DIR/lib$library.a" "$BUILD_DIR/lib$library.so.o" "$BUILD_DIR/lib$library.a.o"
  "$ODIN_BIN" build "$ROOT/src/${library}_c" "${common[@]}" "${profile[@]}" -build-mode:dll -linker:lld -out:"$BUILD_DIR/lib$library.so" "-extra-linker-flags:-Wl,--version-script=$export_directory/$library.map"
  "$ODIN_BIN" build "$ROOT/src/${library}_c" "${common[@]}" "${profile[@]}" -build-mode:lib -out:"$BUILD_DIR/lib$library.a"
done
for library in runtime cooking; do
  binary=libentasis.so
  if [[ "$library" == cooking ]]; then binary=libentasis_cooking.so; fi
  "$ODIN_BIN" run "$ROOT/tools/abi/verify_exports" -target:"$ODIN_TARGET" -microarch:"$ODIN_MICROARCH" -linker:lld -vet -warnings-as-errors -- --precision "$precision" "$library" "$BUILD_DIR/$binary"
done
printf 'C_ABI_BUILD_OK configuration=%s root=%s\n' "$configuration" "$BUILD_DIR"
