#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/../lib/common.sh"
odin=""
odin_count=0
configuration=""
configuration_count=0
start_package=""
start_package_count=0
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
    --start-package)
      ((start_package_count += 1))
      ((start_package_count <= 1)) || fail_usage "--start-package may be supplied only once"
      (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || fail_usage "--start-package requires a non-empty value"
      start_package="$2"
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
packages=("${TEST_PACKAGES[@]}" "${CODEGEN_PACKAGES[@]}")
start_index=0
if ((start_package_count > 0)); then
  start_index=-1
  for package_index in "${!packages[@]}"; do
    if [[ "${packages[$package_index]}" == "$start_package" ]]; then start_index="$package_index"; break; fi
  done
  ((start_index >= 0)) || fail_usage "unknown start package: $start_package"
fi
require_odin "$odin"

if [[ "$configuration" == all ]]; then
  configurations=(development release)
else
  configurations=("$configuration")
fi
for selected_configuration in "${configurations[@]}"; do
  printf 'TEST_ALL_CONFIG configuration=%s start_package=%s\n' "$selected_configuration" "${packages[$start_index]}"
  for ((package_index = start_index; package_index < ${#packages[@]}; package_index++)); do
    package="${packages[$package_index]}"
    if ((package_index < ${#TEST_PACKAGES[@]})); then
      run_test_package "$package" "$selected_configuration"
    else
      run_codegen_package "$package" "$selected_configuration"
    fi
  done
  start_index=0
done
printf 'TEST_ALL_OK\n'
