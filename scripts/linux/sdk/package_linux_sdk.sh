#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
output_root="$ROOT/build/sdk/linux"
if (($#)); then
  (($# == 2)) && [[ "$1" == --output-root && -n "$2" && "$2" != --* ]] ||
    { printf 'usage: package_linux_sdk.sh [--output-root <path>]\n' >&2; exit 2; }
  output_root="$2"
fi
[[ "$output_root" == /* ]] || output_root="$ROOT/$output_root"
output_root="$(realpath -m "$output_root")"
BUILD_DIR="$ROOT/build/c_abi/linux/release"
version="$(sed -n 's/^#define ENTASIS_VERSION_STRING "\(.*\)"/\1/p' "$ROOT/include/entasis/base.h")"
[[ "$version" =~ ^[0-9]+\.[0-9]+\.[0-9]+(-[A-Za-z0-9.-]+)?$ ]] || { printf 'invalid product version\n' >&2; exit 2; }
install_name="Entasis-$version-linux-x86_64-v3-c-sdk"
install_root="$output_root/$install_name"
archive="$output_root/$install_name.tar.gz"
for output in "$install_root" "$archive"; do
  [[ ! -e "$output" && ! -L "$output" ]] || { printf 'SDK output collision: %s\n' "$output" >&2; exit 1; }
done
required=(include/entasis.h include/entasis_cooking.h include/entasis sdk/examples sdk/README.md sdk/cmake/EntasisConfig.cmake sdk/cmake/EntasisConfigVersion.cmake.in sdk/linux/pkgconfig/entasis.pc.in sdk/linux/pkgconfig/entasis-cooking.pc.in sdk/BUILDING.md docs/c LICENSE NOTICE tools/toolchains.lock)
bootstrap=(scripts/linux/toolchains/setup_toolchains.sh scripts/linux/lib/toolchain_common.sh scripts/linux/sdk/run_sdk_examples.sh)
required+=("${bootstrap[@]}")
for source in "${required[@]}"; do [[ -e "$ROOT/$source" ]] || { printf 'missing package source: %s\n' "$source" >&2; exit 1; }; done
libraries=(libentasis.so libentasis.a libentasis_cooking.so libentasis_cooking.a)
for library in "${libraries[@]}"; do [[ -f "$BUILD_DIR/$library" ]] || { printf 'missing Release library: %s\n' "$BUILD_DIR/$library" >&2; exit 1; }; done
mkdir -p "$install_root/include/entasis" "$install_root/lib/pkgconfig" "$install_root/lib/cmake/Entasis" "$install_root/share/doc/Entasis/c" "$install_root/share/licenses/Entasis" "$install_root/examples" "$install_root/scripts/linux" "$install_root/tools"
cp "$ROOT/include/entasis.h" "$ROOT/include/entasis_cooking.h" "$install_root/include/"
cp "$ROOT/include/entasis/"*.h "$install_root/include/entasis/"
for library in "${libraries[@]}"; do cp "$BUILD_DIR/$library" "$install_root/lib/"; done
cp "$ROOT/sdk/examples/"* "$install_root/examples/"
cp "$ROOT/sdk/README.md" "$install_root/README.md"
cp "$ROOT/sdk/cmake/EntasisConfig.cmake" "$install_root/lib/cmake/Entasis/"
cp "$ROOT/sdk/BUILDING.md" "$install_root/share/doc/Entasis/BUILDING.md"
cp -R "$ROOT/docs/c/." "$install_root/share/doc/Entasis/c/"
cp "$ROOT/LICENSE" "$ROOT/NOTICE" "$install_root/share/licenses/Entasis/"
for source in "${bootstrap[@]}"; do
  mkdir -p "$install_root/$(dirname "$source")"
  cp "$ROOT/$source" "$install_root/$source"
done
sed "s/@ENTASIS_VERSION@/$version/g" "$ROOT/sdk/cmake/EntasisConfigVersion.cmake.in" > "$install_root/lib/cmake/Entasis/EntasisConfigVersion.cmake"
for template in "$ROOT/sdk/linux/pkgconfig/"*.pc.in; do
  name="${template##*/}"
  sed "s/@ENTASIS_VERSION@/$version/g" "$template" > "$install_root/lib/pkgconfig/${name%.in}"
done
awk -F= '$1 ~ /^(llvm|cmake|ninja|icu70)\./ && $1 !~ /\.hosts\.windows-amd64\./' "$ROOT/tools/toolchains.lock" > "$install_root/tools/toolchains.lock"
tar -czf "$archive" -C "$output_root" "$install_name"
tar -tzf "$archive" > /dev/null
printf 'LINUX_SDK_OK tree=%s archive=%s\n' "$install_root" "$archive"
