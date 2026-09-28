#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/../lib/common.sh"
mode="" odin=""
declare -A seen=()
while (($#)); do
  option="$1"
  case "$option" in --mode|--odin) ;; *) fail_usage "unknown option: $option" ;; esac
  [[ -z "${seen[$option]:-}" ]] || fail_usage "repeated option: $option"
  seen[$option]=1
  (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || fail_usage "$option requires a value"
  case "$option" in --mode) mode="$2" ;; --odin) odin="$2" ;; esac
  shift 2
done
case "$mode" in check|write) ;; *) fail_usage "--mode must be check or write" ;; esac
require_odin "$odin"
cd "$ROOT"
"$ODIN_BIN" run "$ROOT/tools/abi/generate_abi" -target:"$ODIN_TARGET" -microarch:"$ODIN_MICROARCH" -linker:lld -vet -warnings-as-errors -- "--$mode"
