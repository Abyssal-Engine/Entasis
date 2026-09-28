#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/../lib/common.sh"
configuration="" compiler=clang scope=all

declare -A seen=()
while (($#)); do
  option="$1"
  case "$option" in --configuration|--compiler|--scope) ;; *) fail_usage "unknown option: $option" ;; esac
  [[ -z "${seen[$option]:-}" ]] || fail_usage "repeated option: $option"
  seen[$option]=1
  (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || fail_usage "$option requires a value"
  case "$option" in --configuration) configuration="$2" ;; --compiler) compiler="$2" ;; --scope) scope="$2" ;;  esac
  shift 2
done
case "$configuration" in development|release) ;; *) fail_usage "--configuration must be development or release" ;; esac
case "$compiler" in gcc|clang|all) ;; *) fail_usage "--compiler must be gcc, clang, or all" ;; esac

case "$scope" in smoke|all) ;; *) fail_usage "--scope must be smoke or all" ;; esac

BUILD_DIR="$ROOT/build/c_abi/linux/$configuration"
for library in libentasis.so libentasis.a libentasis_cooking.so libentasis_cooking.a; do
  [[ -f "$BUILD_DIR/$library" ]] || fail_usage "missing $BUILD_DIR/$library"
done
source "$ROOT/scripts/linux/lib/toolchain_common.sh"
toolchain_compilers "$compiler"
lanes=("$compiler")
if [[ "$compiler" == all ]]; then lanes=(gcc clang); fi
for lane in "${lanes[@]}"; do
  if [[ "$lane" == gcc ]]; then CC="$GCC_BIN"; CXX="$GXX_BIN"; else CC="$LLVM_ROOT/bin/clang"; CXX="$LLVM_ROOT/bin/clang++"; fi
  OUT_DIR="$BUILD_DIR/consumers/$lane"
  mkdir -p "$OUT_DIR"
  printf 'C_ABI_CONSUMER_LANE configuration=%s compiler=%s executable=%s\n' "$configuration" "$lane" "$CC"
COMMON_C_FLAGS=(-std=c11 -O2 -Wall -Wextra -Werror -pedantic -I"$ROOT/include" -I"$ROOT/tests/c_abi")
COMMON_CXX_FLAGS=(-std=c++20 -O2 -Wall -Wextra -Werror -pedantic -I"$ROOT/include" -I"$ROOT/tests/c_abi")
COMMON_STATIC_LIBS=(-ldl -lpthread -lm)

public_headers=(
    entasis/base.h
    entasis/collision.h
    entasis/world.h
    entasis/shapes.h
    entasis/bodies.h
    entasis/constraints.h
    entasis/queries.h
    entasis/events.h
    entasis/views.h
    entasis/properties.h
    entasis/entasis.h
    entasis/cooking.h
    entasis.h
    entasis_cooking.h
)
if [[ "$scope" == all ]]; then
for header in "${public_headers[@]}"; do
    safe_name="${header//\//_}"
    printf '#include <%s>\nint main(void) { return 0; }\n' "$header" \
        > "$OUT_DIR/header-$safe_name.c"
    "$CC" "${COMMON_C_FLAGS[@]}" -c "$OUT_DIR/header-$safe_name.c" \
        -o "$OUT_DIR/header-$safe_name.o"

    printf '#include <%s>\nint main() { return 0; }\n' "$header" \
        > "$OUT_DIR/header-$safe_name.cpp"
    "$CXX" "${COMMON_CXX_FLAGS[@]}" -c "$OUT_DIR/header-$safe_name.cpp" \
        -o "$OUT_DIR/header-$safe_name-cpp.o"
done

"$CC" "${COMMON_C_FLAGS[@]}" -c "$ROOT/tests/c_abi/include_c11.c" -o "$OUT_DIR/include_c11.o"
"$CXX" "${COMMON_CXX_FLAGS[@]}" -c "$ROOT/tests/c_abi/include_cpp20.cpp" -o "$OUT_DIR/include_cpp20.o"

fi

runtime_c_tests=(smoke_c11 lifecycle_c11 dispatcher_c11 policies_c11 scene_lifecycle_c11 layer_material_policy_c11 shape_body_static_c11 constraints_c11 dynamics_c11 properties_c11 queries_events_views_c11 extensions_c11 custom_shapes_c11 custom_tasks_c11 custom_constraints_c11 triggers_c11)
[[ "$scope" != smoke ]] || runtime_c_tests=(smoke_c11)
for test_name in "${runtime_c_tests[@]}"; do
    printf 'C_ABI_CONSUMER test=%s lane=%s scope=%s\n' "$test_name" "$lane" "$scope"
    source_file="$ROOT/tests/c_abi/$test_name.c"
    shared_binary="$OUT_DIR/$test_name-shared"
    static_binary="$OUT_DIR/$test_name-static"

    "$CC" "${COMMON_C_FLAGS[@]}" "$source_file" \
        -L"$BUILD_DIR" -Wl,-rpath,"$BUILD_DIR" -lentasis -lpthread -lm \
        -o "$shared_binary"
    "$shared_binary"

    "$CC" "${COMMON_C_FLAGS[@]}" "$source_file" \
        "$BUILD_DIR/libentasis.a" "${COMMON_STATIC_LIBS[@]}" \
        -o "$static_binary"
    "$static_binary"
done

runtime_cpp_tests=(smoke_cpp20 constraints_dynamics_cpp20 queries_properties_cpp20 extensions_cpp20 custom_shapes_cpp20 custom_tasks_cpp20 custom_constraints_cpp20 triggers_cpp20)
[[ "$scope" != smoke ]] || runtime_cpp_tests=(smoke_cpp20)
for test_name in "${runtime_cpp_tests[@]}"; do
    printf 'C_ABI_CONSUMER test=%s lane=%s scope=%s\n' "$test_name" "$lane" "$scope"
    source_file="$ROOT/tests/c_abi/$test_name.cpp"
    shared_binary="$OUT_DIR/$test_name-shared"
    static_binary="$OUT_DIR/$test_name-static"

    "$CXX" "${COMMON_CXX_FLAGS[@]}" "$source_file" \
        -L"$BUILD_DIR" -Wl,-rpath,"$BUILD_DIR" -lentasis -lpthread -lm \
        -o "$shared_binary"
    "$shared_binary"

    "$CXX" "${COMMON_CXX_FLAGS[@]}" "$source_file" \
        "$BUILD_DIR/libentasis.a" "${COMMON_STATIC_LIBS[@]}" \
        -o "$static_binary"
    "$static_binary"
done

    printf 'C_ABI_CONSUMER test=smoke_cooking_c11 lane=%s scope=%s\n' "$lane" "$scope"
    "$CC" "${COMMON_C_FLAGS[@]}" "$ROOT/tests/c_abi/smoke_cooking_c11.c" \
        -L"$BUILD_DIR" -Wl,-rpath,"$BUILD_DIR" -lentasis_cooking \
        -o "$OUT_DIR/smoke_cooking-shared"
    "$OUT_DIR/smoke_cooking-shared"

    "$CC" "${COMMON_C_FLAGS[@]}" "$ROOT/tests/c_abi/smoke_cooking_c11.c" \
        "$BUILD_DIR/libentasis_cooking.a" "${COMMON_STATIC_LIBS[@]}" \
        -o "$OUT_DIR/smoke_cooking-static"
    "$OUT_DIR/smoke_cooking-static"

    [[ "$scope" != smoke ]] || continue
    cooking_tests=(cooking_assets_c11 compound_tasks_c11 allocators_c11 extensions_semantic_c11 triggers_tasks_c11)
    for test_name in "${cooking_tests[@]}"; do
        printf 'C_ABI_CONSUMER test=%s lane=%s scope=%s\n' "$test_name" "$lane" "$scope"
        binary_name="${test_name%_c11}"
        "$CC" "${COMMON_C_FLAGS[@]}" "$ROOT/tests/c_abi/$test_name.c" \
            -L"$BUILD_DIR" -Wl,-rpath,"$BUILD_DIR" \
            -lentasis -lentasis_cooking -ldl -lpthread -lm \
            -o "$OUT_DIR/$binary_name-shared"
        "$OUT_DIR/$binary_name-shared"

        "$CC" "${COMMON_C_FLAGS[@]}" "$ROOT/tests/c_abi/$test_name.c" \
            "$BUILD_DIR/libentasis_cooking.a" "$BUILD_DIR/libentasis.a" \
            -Wl,--allow-multiple-definition "${COMMON_STATIC_LIBS[@]}" -o "$OUT_DIR/$binary_name-static"
        "$OUT_DIR/$binary_name-static"
    done

done
printf 'C_ABI_CONSUMERS_OK configuration=%s compiler=%s scope=%s\n' "$configuration" "$compiler" "$scope"
