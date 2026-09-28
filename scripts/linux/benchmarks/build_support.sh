benchmark_binary() {
  local package="$1"
  local configuration="${2:-release}"
  if [[ "${3:-off}" == on ]]; then
    printf '%s/build/benchmarks/recorded/%s/%s' "$ROOT" "$configuration" "$package"
    return
  fi
  case "$configuration" in
    development) printf '%s/build/benchmarks/development/%s' "$ROOT" "$package" ;;
    release) printf '%s/build/benchmarks/entasis_%s' "$ROOT" "$package" ;;
    *) fail_usage "unknown build configuration: $configuration" ;;
  esac
}

benchmark_reporter_binary() {
  local configuration="$1"
  printf '%s/build/tools/benchmark-report/%s/benchmark_report' "$ROOT" "$configuration"
}

invoke_benchmark_build() {
  local package="$1" output="$2" started
  shift 2
  started="$(date +%s%3N)"
  local temporary="$output.building" argument
  local -a arguments=()
  mkdir -p "$(dirname "$output")"
  rm -f "$output" "$temporary"
  for argument in "$@"; do
    if [[ "$argument" == "-out:$output" ]]; then argument="-out:$temporary"; fi
    arguments+=("$argument")
  done
  printf 'BENCHMARK_STAGE stage=build package=%s output=%s\n' "$package" "$output"
  "$ODIN_BIN" "${arguments[@]}"
  mv -- "$temporary" "$output"
  printf 'BENCHMARK_BUILT package=%s elapsed_ms=%s\n' "$package" "$(( $(date +%s%3N) - started ))"
}

build_benchmark() {
  local package="$1" configuration="${2:-release}" record="${3:-off}" output revision
  benchmark_exists "$package" || fail_usage "unknown benchmark package: $package"
  output="$(benchmark_binary "$package" "$configuration" "$record")"
  set_odin_profile "$configuration"
  local -a arguments=(build "$ROOT/benchmarks/$package" "-out:$output"
    "-collection:entasis=$ROOT/src" "-target:$ODIN_TARGET" "-microarch:$ODIN_MICROARCH"
    "${ODIN_PROFILE_ARGUMENTS[@]}")
  if [[ "$record" == on ]]; then
    revision="$(git -C "$ROOT" rev-parse HEAD)"
    if [[ -n "$(git -C "$ROOT" status --porcelain --untracked-files=normal)" ]]; then revision+="+dirty"; fi
    arguments+=(-define:ENTASIS_BENCHMARK_RECORDING=true
      "-define:ENTASIS_BENCHMARK_SOURCE_REVISION=$revision" "-define:ENTASIS_BENCHMARK_CONFIGURATION=$configuration")
  fi
  arguments+=(-vet -warnings-as-errors "-thread-count:$ODIN_THREAD_COUNT" -linker:lld)
  invoke_benchmark_build "$package" "$output" "${arguments[@]}"
}

build_benchmark_reporter() {
  local configuration="$1" output
  output="$(benchmark_reporter_binary "$configuration")"
  set_odin_profile "$configuration"
  invoke_benchmark_build benchmark_report "$output" build "$ROOT/tools/benchmark_report" "-out:$output" \
    "-target:$ODIN_TARGET" "-microarch:$ODIN_MICROARCH" "${ODIN_PROFILE_ARGUMENTS[@]}" \
    -vet -warnings-as-errors "-thread-count:$ODIN_THREAD_COUNT" -linker:lld
}
