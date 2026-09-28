#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
output_root="$ROOT/build/releases"
if (($#)); then
  (($# == 2)) && [[ "$1" == --output-root && -n "$2" && "$2" != --* ]] ||
    { printf 'usage: scripts/linux/release/release_linux.sh [--output-root <path>]\n' >&2; exit 2; }
  output_root="$2"
fi
[[ "$output_root" == /* ]] || output_root="$ROOT/$output_root"
output_root="$(realpath -m "$output_root")"
version="$(sed -n 's/^#define ENTASIS_VERSION_STRING "\(.*\)"/\1/p' "$ROOT/include/entasis/base.h")"
[[ "$version" =~ ^[0-9]+\.[0-9]+\.[0-9]+(-[A-Za-z0-9.-]+)?$ ]] || { printf 'invalid product version\n' >&2; exit 2; }
mkdir -p "$ROOT/build/release-work/linux"
work="$(mktemp -d "$ROOT/build/release-work/linux/$(date -u +%Y%m%dT%H%M%SZ).XXXXXX")"
attempt="${work##*/}"
stage=setup
trap 'code=$?; if ((code != 0)); then printf "RELEASE_FAILED platform=linux stage=%s exit=%s work=%s\n" "$stage" "$code" "$work" >&2; fi' EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
exec > >(tee "$work/release.log") 2>&1
printf '[release] platform=linux version=%s work=%s\n' "$version" "$work"
source "$ROOT/scripts/linux/lib/toolchain_common.sh"
toolchain_setup c-abi-source all
stage=build
"$ROOT/build_c_abi_linux.sh" --configuration release
stage=sdk-package
"$ROOT/scripts/linux/sdk/package_linux_sdk.sh" --output-root "$work/sdk"
stage=source-package
"$ROOT/scripts/linux/release/package_source.sh" --output-root "$work/source"
sdk="$work/sdk/Entasis-$version-linux-x86_64-v3-c-sdk.tar.gz"
source_archive="$work/source/Entasis-$version-odin-source.tar.gz"
stage=sdk-consumers
"$ROOT/scripts/linux/sdk/test_sdk_consumers.sh" --compiler all --archive "$sdk"
stage=source-example
extraction="$work/source with spaces"
mkdir "$extraction"
tar -xzf "$source_archive" -C "$extraction"
source_root="$extraction/Entasis-$version-odin-source"
declare -A selection=()
if [[ -f "$ROOT/toolchains.local" ]]; then toolchain_read_settings "$ROOT/toolchains.local" selection; fi
{
  printf 'linux-amd64.roots.odin=%s\nlinux-amd64.roots.llvm=%s\n' "$(dirname "$ODIN_BIN")" "$LLVM_ROOT"
  printf 'linux-amd64.cache_root=%s\n' "${selection[linux-amd64.cache_root]:-$ROOT/build/toolchains/prebuilt}"
} > "$source_root/toolchains.local"
for example in falling_box convex_hulls; do
  (cd "$extraction" && "$source_root/scripts/linux/examples/run_examples.sh" --package "$example" --configuration release)
done
stage=complete
parent="$output_root/$version/linux"
mkdir -p "$parent"
pending="$(mktemp -d "$parent/.$attempt.XXXXXX")"
cp "$sdk" "$source_archive" "$pending/"
cmp "$sdk" "$pending/${sdk##*/}"
cmp "$source_archive" "$pending/${source_archive##*/}"
completed="$parent/$attempt"
[[ ! -e "$completed" ]] || { printf 'release output collision: %s\n' "$completed" >&2; exit 1; }
mv -T "$pending" "$completed"
printf 'RELEASE_OK platform=linux version=%s directory=%s\n' "$version" "$completed"
for asset in "$completed"/*; do printf 'asset=%s\n' "$asset"; done
