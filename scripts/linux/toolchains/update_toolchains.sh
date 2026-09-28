#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/../lib/toolchain_common.sh"

tool="" mode=update version=""
declare -A seen=()
while (($#)); do
  option="$1"
  case "$option" in --tool|--mode|--version) ;; *) toolchain_usage_error "unknown option: $option" ;; esac
  [[ -z "${seen[$option]:-}" ]] || toolchain_usage_error "repeated option: $option"
  seen[$option]=1
  (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || toolchain_usage_error "$option requires a value"
  case "$option" in --tool) tool="$2" ;; --mode) mode="$2" ;; --version) version="$2" ;; esac
  shift 2
done
case "$tool" in odin|llvm|cmake|ninja|all) ;; *) toolchain_usage_error "--tool must be odin, llvm, cmake, ninja, or all" ;; esac
case "$mode" in check|update) ;; *) toolchain_usage_error "--mode must be check or update" ;; esac
if [[ -n "$version" ]]; then
  [[ "$tool" != all ]] || toolchain_usage_error "--version requires one tool"
  if [[ "$tool" == odin ]]; then
    [[ "$version" =~ ^dev-[0-9]{4}-[0-9]{2}$ ]] || toolchain_usage_error "invalid Odin release"
  else
    [[ "$version" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]] || toolchain_usage_error "invalid release version"
  fi
fi
toolchain_odin ""
mkdir -p "$TOOLCHAIN_ROOT/build/tools" "$TOOLCHAIN_ROOT/build/toolchains"
updater="$TOOLCHAIN_ROOT/build/tools/update-toolchain-release"
"$ODIN_BIN" build "$TOOLCHAIN_ROOT/tools/toolchains/update_release" -out:"$updater" \
  -target:linux_amd64 -microarch:x86-64-v3 -linker:lld -vet -warnings-as-errors
work="$(mktemp -d "$TOOLCHAIN_ROOT/build/toolchains/.update-XXXXXX")"
trap 'status=$?; if ((status == 0)); then rm -rf "$work"; else printf "UPDATE_FAILED partial=%s\n" "$work" >&2; fi' EXIT
selected=("$tool")
if [[ "$tool" == all ]]; then selected=(odin llvm cmake ninja); fi
path="$TOOLCHAIN_ROOT/tools/toolchains.lock"
declare -A entries=()
toolchain_read_settings "$path" entries
for name in "${selected[@]}"; do
  case "$name" in
    odin) repository=odin-lang/Odin prefix="" ;;
    llvm) repository=llvm/llvm-project prefix=llvmorg- ;;
    cmake) repository=Kitware/CMake prefix=v ;;
    ninja) repository=ninja-build/ninja prefix=v ;;
  esac
  route=releases/latest
  if [[ -n "$version" ]]; then route="releases/tags/$prefix$version"; fi
  url="https://api.github.com/repos/$repository/$route"
  printf 'RELEASE_METADATA %s\n' "$url" >&2
  curl --fail --silent --show-error --location --connect-timeout 30 --max-time 30 \
    -H 'Accept: application/vnd.github+json' -H 'User-Agent: Entasis-toolchains' \
    --url "$url" --output "$work/$name.json"
  "$updater" "$name" "$work/$name.json" > "$work/$name.entry"
  declare -A replacement=()
  toolchain_read_settings "$work/$name.entry" replacement
  printf 'TOOLCHAIN_RELEASE tool=%s old=%s selected=%s mode=%s\n' \
    "$name" "${entries[$name.release]:-absent}" "${replacement[$name.release]}" "$mode"
done
if [[ "$mode" == update ]]; then
  pending="$path.pending"
  (set -o noclobber; : > "$pending") || toolchain_error "Tool declarations already have a writer: $path"
  cp "$path" "$work/current"
  for name in "${selected[@]}"; do
    awk -F= -v prefix="$name." 'index($1, prefix) != 1' "$work/current" > "$work/next"
    cat "$work/$name.entry" >> "$work/next"
    mv "$work/next" "$work/current"
  done
  cat "$work/current" > "$pending"
  mv -f "$pending" "$path"
fi
printf 'TOOLCHAIN_UPDATE_OK mode=%s\n' "$mode"
