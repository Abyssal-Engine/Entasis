#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/../lib/toolchain_common.sh"
compiler=clang archive=""
declare -A seen=()
while (($#)); do
  option="$1"
  case "$option" in --compiler|--archive) ;; *) toolchain_usage_error "unknown option: $option" ;; esac
  [[ -z "${seen[$option]:-}" ]] || toolchain_usage_error "repeated option: $option"
  seen[$option]=1
  (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || toolchain_usage_error "$option requires a value"
  case "$option" in --compiler) compiler="$2" ;; --archive) archive="$2" ;; esac
  shift 2
done
case "$compiler" in gcc|clang|all) ;; *) toolchain_usage_error "--compiler must be gcc, clang, or all" ;; esac
if [[ -z "$archive" ]]; then
  version="$(sed -n 's/^#define ENTASIS_VERSION_STRING "\(.*\)"/\1/p' "$TOOLCHAIN_ROOT/include/entasis/base.h")"
  archive="$TOOLCHAIN_ROOT/build/sdk/linux/Entasis-$version-linux-x86_64-v3-c-sdk.tar.gz"
fi
[[ "$archive" == /* && -f "$archive" ]] || toolchain_usage_error "archive must name an existing absolute path: $archive"
PKG_CONFIG_BIN="$(toolchain_system pkg-config)"
toolchain_setup c-abi-sdk "$compiler"
mkdir -p "$TOOLCHAIN_ROOT/build/sdk-tests/linux"
extraction="$(mktemp -d "$TOOLCHAIN_ROOT/build/sdk-tests/linux/consumer with spaces.XXXXXX")"
toolchain_extract "$archive" "$extraction"
roots=("$extraction"/*)
((${#roots[@]} == 1)) && [[ -d "${roots[0]}" ]] || toolchain_usage_error "expected one SDK archive root"
sdk="${roots[0]}"
# reuse the source command's selected consumer tools without copying local machine paths into the archive
declare -A selection=()
if [[ -f "$TOOLCHAIN_ROOT/toolchains.local" ]]; then toolchain_read_settings "$TOOLCHAIN_ROOT/toolchains.local" selection; fi
{
  printf 'linux-amd64.cache_root=%s\n' "${selection[linux-amd64.cache_root]:-$TOOLCHAIN_ROOT/build/toolchains/prebuilt}"
  printf 'linux-amd64.roots.cmake=%s\n' "$(dirname "$(dirname "$CMAKE_BIN")")"
  printf 'linux-amd64.roots.ninja=%s\n' "$(dirname "$NINJA_BIN")"
  [[ "$compiler" == gcc ]] || printf 'linux-amd64.roots.llvm=%s\n' "$LLVM_ROOT"
  printf 'linux-amd64.system.pkg-config=%s\n' "$PKG_CONFIG_BIN"
  if [[ "$compiler" != clang ]]; then
    printf 'linux-amd64.system.gcc=%s\nlinux-amd64.system.g++=%s\n' "$GCC_BIN" "$GXX_BIN"
  fi
} > "$sdk/toolchains.local"
# exercise actual find_package selection without compiling another consumer
version_tests="$extraction/version-selection"
mkdir "$version_tests"
cat > "$version_tests/CMakeLists.txt" <<'CMAKE'
cmake_minimum_required(VERSION 3.19)
project(EntasisVersionSelection NONE)
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/package")
set(ENTASIS_VERSION "${PROBE_PACKAGE_VERSION}")
configure_file("${PROBE_TEMPLATE}" "${CMAKE_BINARY_DIR}/package/EntasisConfigVersion.cmake" @ONLY)
file(WRITE "${CMAKE_BINARY_DIR}/package/EntasisConfig.cmake" "set(ENTASIS_SELECTION_PROBE 1)\n")
separate_arguments(request NATIVE_COMMAND "${PROBE_REQUEST}")
find_package(Entasis ${request} QUIET CONFIG PATHS "${CMAKE_BINARY_DIR}/package" NO_DEFAULT_PATH)
if(PROBE_EXPECT EQUAL 1)
    if(NOT Entasis_FOUND OR NOT ENTASIS_SELECTION_PROBE)
        message(FATAL_ERROR "Expected package selection: ${PROBE_PACKAGE_VERSION} / ${PROBE_REQUEST}")
    endif()
elseif(Entasis_FOUND)
    message(FATAL_ERROR "Unexpected package selection: ${PROBE_PACKAGE_VERSION} / ${PROBE_REQUEST}")
endif()
CMAKE
while IFS='|' read -r name package_version request expected; do
  "$CMAKE_BIN" -S "$version_tests" -B "$version_tests/$name" \
    "-DPROBE_TEMPLATE=$TOOLCHAIN_ROOT/sdk/cmake/EntasisConfigVersion.cmake.in" \
    "-DPROBE_PACKAGE_VERSION=$package_version" "-DPROBE_REQUEST=$request" "-DPROBE_EXPECT=$expected"
  printf 'SDK_VERSION_SELECTION_OK case=%s\n' "$name"
done <<'CASES'
exact|1.2.3|1.2.3 EXACT|1
newer|1.2.3|1.2.4|0
older-major|1.2.3|0.9|0
older-same|1.2.3|1.1|1
exact-mismatch|1.2.3|1.1 EXACT|0
range-included|1.2.3|1.0...1.2.3|1
range-excluded|1.2.3|1.0...<1.2.3|0
next-major-excluded|1.2.3|1.0...<2|1
cross-major|1.2.3|1.0...2|0
prerelease-stable|1.2.3-rc.1|1.2.3|0
prerelease-unversioned|1.2.3-rc.1||1
CASES
cd "$extraction"
"$sdk/scripts/linux/sdk/run_sdk_examples.sh" --compiler "$compiler"
export PKG_CONFIG_LIBDIR="$sdk/lib/pkgconfig"
unset PKG_CONFIG_PATH
lanes=("$compiler")
if [[ "$compiler" == all ]]; then lanes=(gcc clang); fi
for lane in "${lanes[@]}"; do
  if [[ "$lane" == gcc ]]; then cc="$GCC_BIN"; cxx="$GXX_BIN"; else cc="$LLVM_ROOT/bin/clang"; cxx="$LLVM_ROOT/bin/clang++"; fi
  build="$extraction/pkgconfig-$lane"
  mkdir "$build"
  for language in c cpp; do
    compile="$cc" standard=c11
    if [[ "$language" == cpp ]]; then compile="$cxx"; standard=c++20; fi
    for linkage in shared static; do
      flags_file="$build/$language-$linkage.flags"
      if [[ "$linkage" == static ]]; then "$PKG_CONFIG_BIN" --cflags --libs --static entasis-cooking > "$flags_file"; else "$PKG_CONFIG_BIN" --cflags --libs entasis-cooking > "$flags_file"; fi
      if [[ "$linkage" == static ]]; then
        sed -i -e 's/\(^\|[[:space:]]\)-lentasis_cooking\([[:space:]]\|$\)/\1-l:libentasis_cooking.a\2/g' \
          -e 's/\(^\|[[:space:]]\)-lentasis\([[:space:]]\|$\)/\1-l:libentasis.a\2/g' "$flags_file"
      fi
      binary="$build/$language-$linkage"
      "$compile" "-std=$standard" "$sdk/examples/minimal.$language" "@$flags_file" "-Wl,-rpath,$sdk/lib" -o "$binary"
      "$binary"
      readelf -d "$binary" > "$binary.dependencies.txt"
      if [[ "$linkage" == static ]] && grep -E 'NEEDED.*libentasis' "$binary.dependencies.txt"; then
        printf 'static consumer depends on Entasis shared libraries: %s\n' "$binary" >&2; exit 1
      fi
    done
  done
  for binary in "$sdk/build/examples/$lane/entasis_c_static" "$sdk/build/examples/$lane/entasis_cpp_static"; do
    if readelf -d "$binary" | grep -E 'NEEDED.*libentasis'; then printf 'unexpected shared dependency: %s\n' "$binary" >&2; exit 1; fi
  done
done
readelf --version-info "$sdk/lib/libentasis.so" "$sdk/lib/libentasis_cooking.so" > "$extraction/glibc-versions.txt"
printf 'SDK_CONSUMERS_OK compiler=%s extracted=%s\n' "$compiler" "$sdk"
