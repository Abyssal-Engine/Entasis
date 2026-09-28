#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/../lib/toolchain_common.sh"
compiler=clang compiler_count=0
while (($#)); do
  case "$1" in
    --compiler)
      ((compiler_count += 1))
      ((compiler_count == 1)) || toolchain_usage_error "repeated --compiler"
      (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || toolchain_usage_error "--compiler requires a value"
      compiler="$2"; shift 2 ;;
    *) toolchain_usage_error "unknown option: $1" ;;
  esac
done
case "$compiler" in gcc|clang|all) ;; *) toolchain_usage_error "--compiler must be gcc, clang, or all" ;; esac
[[ -f "$TOOLCHAIN_ROOT/lib/cmake/Entasis/EntasisConfig.cmake" ]] || toolchain_usage_error "run this command from an extracted Entasis SDK"
toolchain_setup c-abi-sdk "$compiler"
lanes=("$compiler")
if [[ "$compiler" == all ]]; then lanes=(gcc clang); fi
for lane in "${lanes[@]}"; do
  if [[ "$lane" == gcc ]]; then cc="$GCC_BIN"; cxx="$GXX_BIN"; else cc="$LLVM_ROOT/bin/clang"; cxx="$LLVM_ROOT/bin/clang++"; fi
  build="$TOOLCHAIN_ROOT/build/examples/$lane"
  printf 'SDK_EXAMPLE_LANE compiler=%s root=%s\n' "$lane" "$TOOLCHAIN_ROOT"
  "$CMAKE_BIN" -S "$TOOLCHAIN_ROOT/examples" -B "$build" -G Ninja -DCMAKE_BUILD_TYPE=Release \
    "-DEntasis_DIR=$TOOLCHAIN_ROOT/lib/cmake/Entasis" "-DCMAKE_C_COMPILER=$cc" "-DCMAKE_CXX_COMPILER=$cxx" "-DCMAKE_MAKE_PROGRAM=$NINJA_BIN"
  "$CMAKE_BIN" --build "$build" --parallel 4
  "$CTEST_BIN" --test-dir "$build" --output-on-failure --no-tests=error
done
printf 'SDK_EXAMPLES_OK compiler=%s\n' "$compiler"
