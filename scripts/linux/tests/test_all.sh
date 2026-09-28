#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/../lib/common.sh"
odin=""
odin_count=0
configuration=""
configuration_count=0
while (($# > 0)); do
  case "$1" in
    --odin)
      ((odin_count += 1))
      ((odin_count <= 1)) || fail_usage "--odin may be supplied only once"
      (($# >= 2)) || fail_usage "--odin requires a value"
      odin="$2"
      shift 2
      ;;
    --configuration)
      ((configuration_count += 1))
      ((configuration_count <= 1)) || fail_usage "--configuration may be supplied only once"
      (($# >= 2)) || fail_usage "--configuration requires a value"
      configuration="$2"
      shift 2
      ;;
    *) fail_usage "unknown option: $1" ;;
  esac
done
if ((odin_count > 0)); then [[ -n "$odin" ]] || fail_usage "--odin requires a non-empty value"; fi
if ((configuration_count == 0)); then configuration="all"; fi
case "$configuration" in
  development|release|all) ;;
  "") fail_usage "--configuration requires a non-empty value" ;;
  *) fail_usage "unknown configuration: $configuration" ;;
esac
require_odin "$odin"

if [[ "$configuration" == all ]]; then
  configurations=(development release)
else
  configurations=("$configuration")
fi
for selected_configuration in "${configurations[@]}"; do
  for package in "${TEST_PACKAGES[@]}"; do
    run_test_package "$package" "$selected_configuration"
  done
  for package in "${CODEGEN_PACKAGES[@]}"; do
    run_codegen_package "$package" "$selected_configuration"
  done
done
printf 'TEST_ALL_OK\n'
