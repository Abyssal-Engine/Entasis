#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/scripts/linux/lib/common.sh"

odin=""
odin_count=0
configuration="development"
configuration_count=0
package=""
package_count=0
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
printf 'EXAMPLE_BUILD_CONFIG configuration=%s packages=%s\n' "$configuration" "$(IFS=,; printf '%s' "${packages[*]}")"

build_example() {
  local selected_package="$1"
  local selected_configuration="$2"
  configure_odin_profile examples "$selected_package" "$selected_configuration"
  printf '[example build] %s configuration=%s executable=%s\n' \
    "$selected_package" "$selected_configuration" "$ODIN_PROFILE_OUTPUT"
  "$ODIN_BIN" build "$ROOT/examples/headless/$selected_package" \
    -out:"$ODIN_PROFILE_OUTPUT" \
    -collection:entasis="$ROOT/src" \
    -target:"$ODIN_TARGET" \
    -microarch:"$ODIN_MICROARCH" \
    "${ODIN_PROFILE_ARGUMENTS[@]}" \
    -vet \
    -warnings-as-errors \
    -thread-count:"$ODIN_THREAD_COUNT" \
    -linker:lld
}

for selected_configuration in "${configurations[@]}"; do
  for selected_package in "${packages[@]}"; do
    build_example "$selected_package" "$selected_configuration"
  done
done
printf 'BUILD_EXAMPLES_OK\n'
