#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/../lib/common.sh"

odin=""
odin_count=0
scope=""
scope_count=0
configuration=""
configuration_count=0
tests=""
tests_count=0
threads=""
threads_count=0
while (($# > 0)); do
  case "$1" in
    --odin)
      ((odin_count += 1))
      ((odin_count <= 1)) || fail_usage "--odin may be supplied only once"
      (($# >= 2)) || fail_usage "--odin requires a value"
      odin="$2"
      shift 2
      ;;
    --scope)
      ((scope_count += 1))
      ((scope_count <= 1)) || fail_usage "--scope may be supplied only once"
      (($# >= 2)) || fail_usage "--scope requires a value"
      scope="$2"
      shift 2
      ;;
    --configuration)
      ((configuration_count += 1))
      ((configuration_count <= 1)) || fail_usage "--configuration may be supplied only once"
      (($# >= 2)) || fail_usage "--configuration requires a value"
      configuration="$2"
      shift 2
      ;;
    --tests)
      ((tests_count += 1))
      ((tests_count <= 1)) || fail_usage "--tests may be supplied only once"
      (($# >= 2)) || fail_usage "--tests requires a value"
      tests="$2"
      shift 2
      ;;
    --threads)
      ((threads_count += 1))
      ((threads_count <= 1)) || fail_usage "--threads may be supplied only once"
      (($# >= 2)) || fail_usage "--threads requires a value"
      threads="$2"
      shift 2
      ;;
    *) fail_usage "unknown option: $1" ;;
  esac
done
if ((odin_count > 0)); then [[ -n "$odin" ]] || fail_usage "--odin requires a non-empty value"; fi
[[ -n "$scope" ]] || fail_usage "missing required --scope <scope>"
package="$(test_scope_to_package "$scope")" || fail_usage "unknown test scope: $scope"
if ((configuration_count == 0)); then configuration="development"; fi
case "$configuration" in
  development|release) ;;
  "") fail_usage "--configuration requires a non-empty value" ;;
  *) fail_usage "unknown configuration: $configuration" ;;
esac
if ((tests_count > 0)); then
  validate_test_selector_list "$tests" || fail_usage "invalid test selector list: $tests"
fi
if ((threads_count == 0)); then
  threads="$ODIN_TEST_THREADS"
else
  validate_test_thread_count "$threads" || fail_usage "invalid test thread count: $threads"
fi
require_odin "$odin"
test_display="${tests:-all}"
printf 'TEST_CONFIG scope=%s tests=%s threads=%s\n' "$scope" "$test_display" "$threads"
run_test_package "$package" "$configuration" "$tests" "$threads"
printf 'TEST_OK scope=%s\n' "$scope"
