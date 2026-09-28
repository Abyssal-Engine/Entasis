#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/../lib/common.sh"

odin=""
odin_count=0
configuration="development"
configuration_count=0
package=""
package_count=0
skip_build="disabled"
skip_build_count=0
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
    --package)
      ((package_count += 1))
      ((package_count <= 1)) || fail_usage "--package may be supplied only once"
      (($# >= 2)) || fail_usage "--package requires a value"
      package="$2"
      shift 2
      ;;
    --skip-build)
      ((skip_build_count += 1))
      ((skip_build_count <= 1)) || fail_usage "--skip-build may be supplied only once"
      skip_build="enabled"
      shift
      ;;
    *) fail_usage "unknown option: $1" ;;
  esac
done
if ((odin_count > 0)); then [[ -n "$odin" ]] || fail_usage "--odin requires a non-empty value"; fi
case "$configuration" in
  development|release|all) ;;
  "") fail_usage "--configuration requires a non-empty value" ;;
  *) fail_usage "unknown configuration: $configuration" ;;
esac
if ((package_count > 0)); then example_exists "$package" || fail_usage "unknown example package: $package"; fi
if ((package_count > 0)); then
  packages=("$package")
else
  package_names="$(example_packages)"
  mapfile -t packages <<< "$package_names"
fi
require_odin "$odin"
configurations=("$configuration")
if [[ "$configuration" == all ]]; then configurations=(development release); fi

if [[ "$skip_build" == disabled ]]; then
  build_arguments=(--odin "$ODIN_BIN" --configuration "$configuration")
  if ((package_count > 0)); then build_arguments+=(--package "$package"); fi
  "$ROOT/build_odin_linux.sh" "${build_arguments[@]}"
fi

for selected_configuration in "${configurations[@]}"; do
  for selected_package in "${packages[@]}"; do
    binary="$(example_binary "$selected_package" "$selected_configuration")"
    [[ -x "$binary" ]] || fail_usage "missing example binary: $binary"
    printf '[example run] %s configuration=%s executable=%s\n' \
      "$selected_package" "$selected_configuration" "$binary"
    "$binary"
  done
done
printf 'RUN_EXAMPLES_OK\n'
