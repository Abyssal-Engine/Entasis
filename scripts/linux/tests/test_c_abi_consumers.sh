#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/../lib/common.sh"
configuration="" compiler=clang scope=all precision=float32 baseline_root="" compatibility_fixtures=cumulative compatibility_layout=different

declare -A seen=()
while (($#)); do
  option="$1"
  case "$option" in --configuration|--compiler|--scope|--precision|--baseline-root|--compatibility-fixtures|--compatibility-layout) ;; *) fail_usage "unknown option: $option" ;; esac
  [[ -z "${seen[$option]:-}" ]] || fail_usage "repeated option: $option"
  seen[$option]=1
  (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || fail_usage "$option requires a value"
  case "$option" in --configuration) configuration="$2" ;; --compiler) compiler="$2" ;; --scope) scope="$2" ;; --precision) precision="$2" ;; --baseline-root) baseline_root="$2" ;; --compatibility-fixtures) compatibility_fixtures="$2" ;; --compatibility-layout) compatibility_layout="$2" ;;  esac
  shift 2
done
case "$configuration" in development|release) ;; *) fail_usage "--configuration must be development or release" ;; esac
case "$compiler" in gcc|clang|all) ;; *) fail_usage "--compiler must be gcc, clang, or all" ;; esac

case "$scope" in smoke|all|callbacks|large-worlds|compatibility) ;; *) fail_usage "--scope must be smoke, all, callbacks, large-worlds, or compatibility" ;; esac

case "$precision" in float32|mixed64) ;; *) fail_usage "--precision must be float32 or mixed64" ;; esac
case "$compatibility_fixtures" in original|cumulative) ;; *) fail_usage "--compatibility-fixtures must be original or cumulative" ;; esac
case "$compatibility_layout" in matching|different) ;; *) fail_usage "--compatibility-layout must be matching or different" ;; esac
if [[ "$scope" == compatibility ]]; then
  [[ -n "$baseline_root" && "$baseline_root" != /* && "$baseline_root" != *:* && "$baseline_root" != *\\* ]] || fail_usage "compatibility requires a repository-relative --baseline-root"
  baseline_root="$(realpath -e "$ROOT/$baseline_root")"
  [[ "$baseline_root" == "$ROOT/"* && -d "$baseline_root" ]] || fail_usage "baseline must remain inside the repository"
else
  for option in --baseline-root --compatibility-fixtures --compatibility-layout; do
    [[ -z "${seen[$option]:-}" ]] || fail_usage "$option is only valid for compatibility"
  done
fi
BUILD_DIR="$ROOT/build/c_abi/linux/$configuration"
if [[ "$precision" == mixed64 ]]; then BUILD_DIR+="/mixed64"; fi
for library in libentasis.so libentasis.a libentasis_cooking.so libentasis_cooking.a; do
  [[ -f "$BUILD_DIR/$library" ]] || fail_usage "missing $BUILD_DIR/$library"
done
source "$ROOT/scripts/linux/lib/toolchain_common.sh"
toolchain_compilers "$compiler"
lanes=("$compiler")
if [[ "$compiler" == all ]]; then lanes=(gcc clang); fi
# freeze the pre-precision cumulative membership for retained consumer delivery
legacy_runtime_c=(smoke_c11 lifecycle_c11 dispatcher_c11 policies_c11 scene_lifecycle_c11 layer_material_policy_c11 shape_body_static_c11 constraints_c11 dynamics_c11 properties_c11 queries_events_views_c11 extensions_c11 custom_shapes_c11 custom_tasks_c11 custom_constraints_c11 triggers_c11)
legacy_runtime_cpp=(smoke_cpp20 constraints_dynamics_cpp20 queries_properties_cpp20 extensions_cpp20 custom_shapes_cpp20 custom_tasks_cpp20 custom_constraints_cpp20 triggers_cpp20)
legacy_cooking=(smoke_cooking_c11 cooking_assets_c11 compound_tasks_c11 allocators_c11 extensions_semantic_c11 triggers_tasks_c11)

run_selected_shared() {
  local binary="$1" pair_root="$2" output="$3"
  shift 3
  local dynamic interpreter library resolved listing
  dynamic="$(readelf -d "$binary")" || return
  [[ "$dynamic" == *'(RUNPATH)'* && "$dynamic" != *'(RPATH)'* ]] || { printf 'unsupported ELF search path: %s\n' "$binary" >&2; return 2; }
  interpreter="$(readelf -l "$binary" | sed -n 's/.*Requesting program interpreter: \(.*\)]/\1/p')"
  [[ "$interpreter" == /* && -x "$interpreter" ]] || { printf 'unsupported ELF interpreter: %s\n' "$binary" >&2; return 2; }
  listing="$(env -u LD_PRELOAD -u LD_AUDIT LD_LIBRARY_PATH="$pair_root" "$interpreter" --list "$binary")" || return
  printf '%s\n' "$listing" > "$output.loader.txt"
  while IFS= read -r library; do
    [[ "$library" == *entasis* ]] || continue
    case "$library" in libentasis.so|libentasis_cooking.so) ;; *) printf 'unsupported Entasis dependency: %s\n' "$library" >&2; return 2 ;; esac
    [[ -f "$pair_root/$library" ]] || return 2
    resolved="$(printf '%s\n' "$listing" | awk -v name="$library" '$1 == name && $2 == "=>" {print $3}')"
    [[ "$resolved" == "$pair_root/$library" ]] || { printf 'incorrect loader selection: %s -> %s\n' "$library" "$resolved" >&2; return 2; }
    printf 'C_ABI_COMPATIBILITY_LOADER binary=%s library=%s resolved=%s\n' "$binary" "$library" "$resolved"
  done < <(printf '%s\n' "$dynamic" | sed -n 's/.*(NEEDED).*\[\(.*\)\]/\1/p')
  env -u LD_PRELOAD -u LD_AUDIT LD_LIBRARY_PATH="$pair_root" "$binary" "$@" > "$output" 2>&1
}

if [[ "$scope" == compatibility ]]; then
  baseline_build="$baseline_root/build/c_abi/linux/$configuration"
  for library in libentasis.so libentasis.a libentasis_cooking.so libentasis_cooking.a; do
    [[ -f "$baseline_build/$library" ]] || fail_usage "missing retained library: $baseline_build/$library"
  done
  if [[ "$compatibility_fixtures" == original ]]; then
    legacy_runtime_c=("${legacy_runtime_c[@]:0:11}")
    legacy_runtime_cpp=("${legacy_runtime_cpp[@]:0:3}")
    legacy_cooking=(smoke_cooking_c11 cooking_assets_c11)
  fi
  for lane in "${lanes[@]}"; do
    for fixture in "${legacy_runtime_c[@]}" "${legacy_runtime_cpp[@]}" "${legacy_cooking[@]}"; do
      binary_name="$fixture"
      extension=c
      [[ "$fixture" != *_cpp20 ]] || extension=cpp
      for cooking in "${legacy_cooking[@]}"; do [[ "$cooking" != "$fixture" ]] || binary_name="${fixture%_c11}"; done
      [[ -f "$baseline_root/tests/c_abi/$fixture.$extension" && -x "$baseline_build/consumers/$lane/$binary_name-shared" ]] || fail_usage "missing retained fixture or executable: $lane/$fixture"
    done
  done
  failures=0
  for lane in "${lanes[@]}"; do
    if [[ "$lane" == gcc ]]; then cc="$GCC_BIN"; cxx="$GXX_BIN"; else cc="$LLVM_ROOT/bin/clang"; cxx="$LLVM_ROOT/bin/clang++"; fi
    lane_root="$BUILD_DIR/consumers/compatibility/$lane"
    mkdir -p "$lane_root/current-pair"
    cp "$BUILD_DIR/libentasis.so" "$BUILD_DIR/libentasis_cooking.so" "$lane_root/current-pair/"
    pair_root="$(realpath "$lane_root/current-pair")"
    for fixture in "${legacy_runtime_c[@]}" "${legacy_runtime_cpp[@]}" "${legacy_cooking[@]}"; do
      binary_name="$fixture" extension=c compile="$cc" standard=c11
      if [[ "$fixture" == *_cpp20 ]]; then extension=cpp; compile="$cxx"; standard=c++20; fi
      libraries=("$BUILD_DIR/libentasis.a")
      for cooking in "${legacy_cooking[@]}"; do
        if [[ "$cooking" == "$fixture" ]]; then
          binary_name="${fixture%_c11}"
          libraries=("$BUILD_DIR/libentasis_cooking.a" "$BUILD_DIR/libentasis.a" -Wl,--allow-multiple-definition)
        fi
      done
      retained="$baseline_build/consumers/$lane/$binary_name-shared"
      if run_selected_shared "$retained" "$pair_root" "$lane_root/$fixture-shared.txt"; then
        printf 'C_ABI_COMPATIBILITY_PASS lane=%s fixture=%s linkage=shared\n' "$lane" "$fixture"
      else failures=$((failures + 1)); fi
      binary="$lane_root/$fixture-static"
      if "$compile" "-std=$standard" -O2 -Wall -Wextra -Werror -pedantic -I"$baseline_root/include" -I"$baseline_root/tests/c_abi" "$baseline_root/tests/c_abi/$fixture.$extension" "${libraries[@]}" -ldl -lpthread -lm -o "$binary" > "$binary.build.txt" 2>&1 && "$binary" > "$binary.run.txt" 2>&1; then
        printf 'C_ABI_COMPATIBILITY_PASS lane=%s fixture=%s linkage=static\n' "$lane" "$fixture"
      else failures=$((failures + 1)); fi
    done
    for pair in old-old new-new old-new new-old; do
      runtime_root="$baseline_build" cooking_root="$baseline_build"
      [[ "$pair" != new-* ]] || runtime_root="$BUILD_DIR"
      [[ "$pair" != *-new ]] || cooking_root="$BUILD_DIR"
      expected=reject
      if [[ "$pair" == old-old || "$pair" == new-new || "$compatibility_layout" == matching ]]; then expected=accept; fi
      pair_root="$lane_root/$pair"
      mkdir -p "$pair_root"
      pair_root="$(realpath "$pair_root")"
      cp "$runtime_root/libentasis.so" "$pair_root/"
      cp "$cooking_root/libentasis_cooking.so" "$pair_root/"
      binary="$pair_root/check"
      if "$cc" -std=c11 -O2 -Wall -Wextra -Werror -pedantic -I"$baseline_root/include" -I"$baseline_root/tests/c_abi" "$ROOT/tests/c_abi/runtime_cooking_compatibility_c11.c" -L"$pair_root" -Wl,-rpath,"$pair_root" -lentasis -lentasis_cooking -ldl -lpthread -lm -o "$binary" > "$pair_root/build.txt" 2>&1 && run_selected_shared "$binary" "$pair_root" "$pair_root/run.txt" "$expected"; then
        printf 'C_ABI_COMPATIBILITY_PAIR lane=%s pair=%s expected=%s status=passed\n' "$lane" "$pair" "$expected"
      else
        failures=$((failures + 1))
        printf 'C_ABI_COMPATIBILITY_PAIR lane=%s pair=%s expected=%s status=failed\n' "$lane" "$pair" "$expected"
      fi
    done
  done
  ((failures == 0)) || { printf 'C_ABI_COMPATIBILITY_FAILED count=%s\n' "$failures" >&2; exit 1; }
  printf 'C_ABI_COMPATIBILITY_OK precision=%s compiler=%s layout=%s\n' "$precision" "$compiler" "$compatibility_layout"
  exit 0
fi

for lane in "${lanes[@]}"; do
  if [[ "$lane" == gcc ]]; then CC="$GCC_BIN"; CXX="$GXX_BIN"; else CC="$LLVM_ROOT/bin/clang"; CXX="$LLVM_ROOT/bin/clang++"; fi
  OUT_DIR="$BUILD_DIR/consumers/$lane"
  mkdir -p "$OUT_DIR"
  printf 'C_ABI_CONSUMER_LANE configuration=%s compiler=%s executable=%s\n' "$configuration" "$lane" "$CC"
COMMON_C_FLAGS=(-std=c11 -O2 -Wall -Wextra -Werror -pedantic -I"$ROOT/include" -I"$ROOT/tests/c_abi")
COMMON_CXX_FLAGS=(-std=c++20 -O2 -Wall -Wextra -Werror -pedantic -I"$ROOT/include" -I"$ROOT/tests/c_abi")
if [[ "$precision" == mixed64 ]]; then COMMON_C_FLAGS+=(-DENTASIS_PRECISION_MIXED64=1); COMMON_CXX_FLAGS+=(-DENTASIS_PRECISION_MIXED64=1); fi
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
runtime_c_tests+=(origin_precision_c11)
if [[ "$precision" == mixed64 ]]; then runtime_c_tests+=(global_precision_c11); fi
[[ "$scope" != smoke ]] || runtime_c_tests=(smoke_c11)
if [[ "$scope" == large-worlds ]]; then runtime_c_tests=(origin_precision_c11); if [[ "$precision" == mixed64 ]]; then runtime_c_tests+=(global_precision_c11); fi; fi
if [[ "$scope" == callbacks ]]; then runtime_c_tests=(policies_c11 layer_material_policy_c11 extensions_c11); fi
for test_name in "${runtime_c_tests[@]}"; do
    fixture_args=()
    if [[ "$scope" == callbacks && "$test_name" == extensions_c11 ]]; then fixture_args=(callbacks); fi
    printf 'C_ABI_CONSUMER test=%s lane=%s scope=%s\n' "$test_name" "$lane" "$scope"
    source_file="$ROOT/tests/c_abi/$test_name.c"
    shared_binary="$OUT_DIR/$test_name-shared"
    static_binary="$OUT_DIR/$test_name-static"

    "$CC" "${COMMON_C_FLAGS[@]}" "$source_file" \
        -L"$BUILD_DIR" -Wl,-rpath,"$BUILD_DIR" -lentasis -lpthread -lm \
        -o "$shared_binary"
    "$shared_binary" "${fixture_args[@]}"

    "$CC" "${COMMON_C_FLAGS[@]}" "$source_file" \
        "$BUILD_DIR/libentasis.a" "${COMMON_STATIC_LIBS[@]}" \
        -o "$static_binary"
    "$static_binary" "${fixture_args[@]}"
done

runtime_cpp_tests=(smoke_cpp20 constraints_dynamics_cpp20 queries_properties_cpp20 extensions_cpp20 custom_shapes_cpp20 custom_tasks_cpp20 custom_constraints_cpp20 triggers_cpp20)
runtime_cpp_tests+=(origin_precision_cpp20)
if [[ "$precision" == mixed64 ]]; then runtime_cpp_tests+=(global_precision_cpp20); fi
[[ "$scope" != smoke ]] || runtime_cpp_tests=(smoke_cpp20)
if [[ "$scope" == large-worlds ]]; then runtime_cpp_tests=(origin_precision_cpp20); if [[ "$precision" == mixed64 ]]; then runtime_cpp_tests+=(global_precision_cpp20); fi; fi
if [[ "$scope" == callbacks ]]; then runtime_cpp_tests=(extensions_cpp20); fi
for test_name in "${runtime_cpp_tests[@]}"; do
    fixture_args=()
    if [[ "$scope" == callbacks && "$test_name" == extensions_cpp20 ]]; then fixture_args=(callbacks); fi
    printf 'C_ABI_CONSUMER test=%s lane=%s scope=%s\n' "$test_name" "$lane" "$scope"
    source_file="$ROOT/tests/c_abi/$test_name.cpp"
    shared_binary="$OUT_DIR/$test_name-shared"
    static_binary="$OUT_DIR/$test_name-static"

    "$CXX" "${COMMON_CXX_FLAGS[@]}" "$source_file" \
        -L"$BUILD_DIR" -Wl,-rpath,"$BUILD_DIR" -lentasis -lpthread -lm \
        -o "$shared_binary"
    "$shared_binary" "${fixture_args[@]}"

    "$CXX" "${COMMON_CXX_FLAGS[@]}" "$source_file" \
        "$BUILD_DIR/libentasis.a" "${COMMON_STATIC_LIBS[@]}" \
        -o "$static_binary"
    "$static_binary" "${fixture_args[@]}"
done

    [[ "$scope" != large-worlds && "$scope" != callbacks ]] || continue
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
