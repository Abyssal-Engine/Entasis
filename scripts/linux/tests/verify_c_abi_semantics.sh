#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/../lib/common.sh"
configuration="" compiler=clang
scope=all odin=""
declare -A seen=()
while (($#)); do
  option="$1"
  case "$option" in --configuration|--compiler|--scope|--odin) ;; *) fail_usage "unknown option: $option" ;; esac
  [[ -z "${seen[$option]:-}" ]] || fail_usage "repeated option: $option"
  seen[$option]=1
  (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || fail_usage "$option requires a value"
  case "$option" in --configuration) configuration="$2" ;; --compiler) compiler="$2" ;; --scope) scope="$2" ;; --odin) odin="$2" ;; esac
  shift 2
done
case "$configuration" in development|release) ;; *) fail_usage "--configuration must be development or release" ;; esac
case "$compiler" in gcc|clang|all) ;; *) fail_usage "--compiler must be gcc, clang, or all" ;; esac
case "$scope" in all|queries) ;; *) fail_usage "--scope must be all or queries" ;; esac
BUILD_DIR="$ROOT/build/c_abi/linux/$configuration"
for library in libentasis.so libentasis.a libentasis_cooking.so libentasis_cooking.a; do
  [[ -f "$BUILD_DIR/$library" ]] || fail_usage "missing $BUILD_DIR/$library"
done
source "$ROOT/scripts/linux/lib/toolchain_common.sh"
lanes=("$compiler")
if [[ "$compiler" == all ]]; then lanes=(gcc clang); fi
require_odin "$odin"
toolchain_compilers "$compiler"
profile=(-debug -o:none -source-code-locations:normal)
if [[ "$configuration" == release ]]; then profile=(-o:speed -no-bounds-check -disable-assert -source-code-locations:none); fi
families=(semantic scene_semantic dynamics_semantic queries_semantic custom_shapes_semantic custom_tasks_semantic custom_constraints_semantic extensions_semantic)
consumer_arguments=()
if [[ "$scope" == queries ]]; then families=(queries_semantic); consumer_arguments=(--queries-only); fi
for lane in "${lanes[@]}"; do
  if [[ "$lane" == gcc ]]; then CC="$GCC_BIN"; else CC="$LLVM_ROOT/bin/clang"; fi
  OUT_DIR="$BUILD_DIR/semantic-parity/$scope/$lane"
  mkdir -p "$OUT_DIR"
  printf 'C_ABI_SEMANTIC_LANE configuration=%s compiler=%s scope=%s\n' "$configuration" "$lane" "$scope"
  for family in "${families[@]}"; do
    "$ODIN_BIN" build "$ROOT/tests/c_abi/${family}_direct_odin.odin" -file -collection:entasis="$ROOT/src" -target:"$ODIN_TARGET" -microarch:"$ODIN_MICROARCH" "${profile[@]}" -thread-count:"$ODIN_THREAD_COUNT" -linker:lld -vet -warnings-as-errors -out:"$OUT_DIR/$family-direct-odin"
    "$OUT_DIR/$family-direct-odin" "${consumer_arguments[@]}" > "$OUT_DIR/$family-direct.txt"
    for linkage in shared static; do
      libraries=(-L"$BUILD_DIR" -Wl,-rpath,"$BUILD_DIR" -lentasis -ldl -lpthread -lm)
      if [[ "$linkage" == static ]]; then libraries=("$BUILD_DIR/libentasis.a" -ldl -lpthread -lm); fi
      if [[ "$family" == custom_tasks_semantic || "$family" == extensions_semantic ]]; then
        if [[ "$linkage" == shared ]]; then
          libraries=(-L"$BUILD_DIR" -Wl,-rpath,"$BUILD_DIR" -lentasis -lentasis_cooking -ldl -lpthread -lm)
        else
          libraries=("$BUILD_DIR/libentasis_cooking.a" "$BUILD_DIR/libentasis.a" -Wl,--allow-multiple-definition -ldl -lpthread -lm)
        fi
      fi
      printf 'C_ABI_SEMANTIC compiler=%s family=%s linkage=%s\n' "$lane" "$family" "$linkage"
      "$CC" -std=c11 -O2 -Wall -Wextra -Werror -pedantic -I"$ROOT/include" -I"$ROOT/tests/c_abi" "$ROOT/tests/c_abi/${family}_c11.c" "${libraries[@]}" -o "$OUT_DIR/$family-c-$linkage"
      "$OUT_DIR/$family-c-$linkage" "${consumer_arguments[@]}" > "$OUT_DIR/$family-c-$linkage.txt"
      diff -u "$OUT_DIR/$family-direct.txt" "$OUT_DIR/$family-c-$linkage.txt"
      if [[ "$family" == queries_semantic ]]; then
        printf 'C_ABI_SEMANTIC compiler=%s family=%s linkage=%s context=enabled\n' "$lane" "$family" "$linkage"
        "$CC" -std=c11 -O2 -Wall -Wextra -Werror -pedantic -DENTASIS_TEST_QUERY_CONTEXT -I"$ROOT/include" -I"$ROOT/tests/c_abi" "$ROOT/tests/c_abi/${family}_c11.c" "${libraries[@]}" -o "$OUT_DIR/$family-context-$linkage"
        "$OUT_DIR/$family-context-$linkage" "${consumer_arguments[@]}" > "$OUT_DIR/$family-context-$linkage.txt"
        diff -u "$OUT_DIR/$family-direct.txt" "$OUT_DIR/$family-context-$linkage.txt"
      fi
    done
    printf 'C_ABI_SEMANTIC_PASS compiler=%s family=%s\n' "$lane" "$family"
  done
done
printf 'C_ABI_SEMANTICS_OK configuration=%s compiler=%s scope=%s\n' "$configuration" "$compiler" "$scope"
