#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/../lib/toolchain_common.sh"

usage="" compiler=clang cache_root=""
declare -A seen=()
while (($#)); do
  option="$1"
  case "$option" in --usage|--compiler|--cache-root) ;; *) toolchain_usage_error "unknown option: $option" ;; esac
  [[ -z "${seen[$option]:-}" ]] || toolchain_usage_error "repeated option: $option"
  seen[$option]=1
  (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || toolchain_usage_error "$option requires a value"
  case "$option" in
    --usage) usage="$2" ;;
    --compiler) compiler="$2" ;;
    --cache-root) cache_root="$2" ;;
  esac
  shift 2
done
case "$usage" in odin|c-abi-source|c-abi-sdk) ;; *) toolchain_usage_error "--usage must be odin, c-abi-source, or c-abi-sdk" ;; esac
case "$compiler" in gcc|clang|all) ;; *) toolchain_usage_error "--compiler must be gcc, clang, or all" ;; esac
[[ -z "$cache_root" || "$cache_root" == /* ]] || toolchain_usage_error "--cache-root must be absolute"
[[ "$usage" == c-abi-sdk || -z "${seen[--compiler]:-}" ]] || toolchain_usage_error "--compiler applies only to c-abi-sdk"
toolchain_setup "$usage" "$compiler" "$cache_root"
printf 'TOOLCHAIN_SETUP_OK usage=%s\n' "$usage"
