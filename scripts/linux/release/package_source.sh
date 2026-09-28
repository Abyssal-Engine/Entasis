#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
output_root="$ROOT/build/packages/source/linux"
if (($#)); then
  (($# == 2)) && [[ "$1" == --output-root && -n "$2" && "$2" != --* ]] ||
    { printf 'usage: package_source.sh [--output-root <path>]\n' >&2; exit 2; }
  output_root="$2"
fi
[[ "$output_root" == /* ]] || output_root="$ROOT/$output_root"
output_root="$(realpath -m "$output_root")"
version="$(sed -n 's/^#define ENTASIS_VERSION_STRING "\(.*\)"/\1/p' "$ROOT/include/entasis/base.h")"
[[ "$version" =~ ^[0-9]+\.[0-9]+\.[0-9]+(-[A-Za-z0-9.-]+)?$ ]] || { printf 'invalid product version\n' >&2; exit 2; }
name="Entasis-$version-odin-source"
tree="$output_root/$name"
archive="$output_root/$name.tar.gz"
for output in "$tree" "$archive"; do
  [[ ! -e "$output" && ! -L "$output" ]] || { printf 'source output collision: %s\n' "$output" >&2; exit 1; }
done
roots=(src/entasis src/entasis_cooking src/entasis_physics src/entasis_utilities examples/headless docs/odin)
files=(CHANGELOG.md LICENSE NOTICE docs/LIMITS.md build_odin_linux.sh build_odin_windows.ps1
  scripts/linux/examples/run_examples.sh scripts/windows/examples/run_examples.ps1
  scripts/linux/lib/common.sh scripts/linux/lib/toolchain_common.sh
  scripts/windows/lib/common.ps1 scripts/windows/lib/toolchain_common.ps1
  scripts/linux/toolchains/setup_toolchains.sh scripts/windows/toolchains/setup_toolchains.ps1 tools/toolchains.lock
  sdk/odin/README.md sdk/odin/BUILDING.md sdk/odin/DOCUMENTATION.md)
declare -A installed=([sdk/odin/README.md]=README.md [sdk/odin/BUILDING.md]=docs/BUILDING.md
  [sdk/odin/DOCUMENTATION.md]=docs/README.md)
# Git owns current membership, including unstaged additions and removals
excludes=(':(exclude)**/AGENTS.md' ':(exclude)**/Plan.*.md' ':(exclude)**/toolchains.local'
  ':(exclude)**/*.log' ':(exclude)**/*.stdout.txt' ':(exclude)**/*.stderr.txt')
selected="$(git -C "$ROOT" -c core.quotePath=false ls-files --cached --others --exclude-standard -- "${roots[@]}" "${files[@]}" "${excludes[@]}")"
deleted="$(git -C "$ROOT" -c core.quotePath=false ls-files --deleted -- "${roots[@]}" "${files[@]}" "${excludes[@]}")"
mapfile -t files < <({
  LC_ALL=C comm -23 <(printf '%s\n' "$selected" | LC_ALL=C sort -u) <(printf '%s\n' "$deleted" | LC_ALL=C sort -u)
  printf '%s\n' "${files[@]}"
} | LC_ALL=C sort -u)
for file in "${files[@]}"; do
  [[ -f "$ROOT/$file" && ! -L "$ROOT/$file" ]] || { printf 'missing/nonregular source input: %s\n' "$file" >&2; exit 1; }
done
mkdir -p "$tree"
for file in "${files[@]}"; do
  destination="$tree/${installed[$file]:-$file}"
  mkdir -p "$(dirname "$destination")"
  cp "$ROOT/$file" "$destination"
  if [[ "$file" == *.sh ]]; then chmod 755 "$destination"; else chmod 644 "$destination"; fi
done
tar -czf "$archive" -C "$output_root" "$name"
tar -tzf "$archive" > /dev/null
tar -dzf "$archive" -C "$output_root"
for file in "${files[@]}"; do cmp "$ROOT/$file" "$tree/${installed[$file]:-$file}"; done
printf 'SOURCE_PACKAGE_OK platform=linux files=%s archive=%s\n' "${#files[@]}" "$archive"
