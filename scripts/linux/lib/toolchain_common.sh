#!/usr/bin/env bash

TOOLCHAIN_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"

toolchain_usage_error() {
  printf 'error: %s\n' "$*" >&2
  exit 2
}

toolchain_error() {
  printf 'error: %s\n' "$*" >&2
  exit 1
}

# values after the first equals sign are literal text, never evaluated as shell code
toolchain_read_settings() {
  local settings_file="$1" line key value
  local -n settings_output="$2"
  settings_output=()
  while IFS= read -r line || [[ -n "$line" ]]; do
    line="${line%$'\r'}"
    [[ -z "$line" || "$line" == \#* ]] && continue
    [[ "$line" == *=* ]] || toolchain_usage_error "expected key=value in $settings_file"
    key="${line%%=*}" value="${line#*=}"
    [[ "$key" =~ ^[a-zA-Z0-9_+.-]+$ && ! -v "settings_output[$key]" ]] ||
      toolchain_usage_error "invalid or repeated setting $key in $settings_file"
    settings_output["$key"]="$value"
  done < "$settings_file"
}

toolchain_extract() {
  local archive="$1" destination="$2"
  mkdir -p "$destination" || return $?
  case "$archive" in
    *.deb)
      timeout 60 dpkg-deb --fsys-tarfile "$archive" |
        timeout 900 tar --extract --file=- --directory="$destination" --no-same-owner --no-same-permissions
      ;;
    *.zip) timeout 900 unzip -q "$archive" -d "$destination" ;;
    *) timeout 900 tar --extract --file="$archive" --directory="$destination" --no-same-owner --no-same-permissions ;;
  esac
}

toolchain_host() {
  [[ "$(uname -s)" == Linux && "$(uname -m)" == x86_64 ]] || toolchain_usage_error "native Linux AMD64 is required"
}

toolchain_inspect() (
  local root="$1" tool="$2" version="$3" commit="$4" cache="$5"
  local name soname output libraries="" runtime
  local -a executables
  case "$tool" in
    odin) executables=(odin) ;;
    llvm) executables=(bin/clang bin/clang++ bin/ld.lld bin/llvm-readobj) ;;
    cmake) executables=(bin/cmake bin/ctest) ;;
    ninja) executables=(ninja) ;;
    icu70)
      executables=()
      for name in icudata icuuc icui18n; do executables+=("usr/lib/x86_64-linux-gnu/lib$name.so.$version"); done
      ;;
    *) toolchain_usage_error "unknown tool: $tool" ;;
  esac
  for name in "${executables[@]}"; do
    [[ -f "$root/$name" && "$(head -c 4 "$root/$name")" == $'\x7fELF' ]] ||
      toolchain_error "Required native ELF executable is missing/incompatible: $root/$name"
  done
  if [[ "$tool" == odin ]]; then
    for name in base core vendor; do
      [[ -d "$root/$name" ]] || toolchain_error "Incomplete Odin distribution: $root/$name"
    done
  fi
  if [[ "$tool" == icu70 ]]; then
    for name in "${executables[@]}"; do
      soname="${name%.so.*}.so.${version%%.*}"
      [[ "$(readlink -f "$root/$soname")" == "$(readlink -f "$root/$name")" ]] ||
        toolchain_error "Required ICU 70 SONAME link is missing: $soname"
    done
    exit 0
  fi
  local argument=--version
  [[ "$tool" != odin ]] || argument=version
  output="$(timeout 30 "$root/${executables[0]}" "$argument")" || exit $?
  output="${output%%$'\n'*}"
  if [[ "$tool" == odin ]]; then
    [[ "$output" =~ version[[:space:]]([^[:space:]]+):([0-9a-f]{7,40}) ]] ||
      toolchain_error "Odin declaration mismatch: $output"
    [[ ( "${BASH_REMATCH[1]}" == "$version" || "${BASH_REMATCH[1]}" == "$version-nightly" ) &&
      "$commit" == "${BASH_REMATCH[2]}"* ]] || toolchain_error "Odin declaration mismatch: $output"
  else
    local version_pattern="(^|[^0-9.])${version//./\\.}([^0-9.]|$)"
    [[ "$output" =~ $version_pattern ]] || toolchain_error "$tool declaration mismatch: $output"
  fi
  if [[ "$tool" == llvm ]]; then
    runtime="$(toolchain_resolve icu70 "" "$cache")" || exit $?
    libraries="$runtime/usr/lib/x86_64-linux-gnu"
    export LD_LIBRARY_PATH="$libraries${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  fi
  for name in "${executables[@]:1}"; do
    timeout 30 "$root/$name" --version >/dev/null || toolchain_error "Required tool cannot run: $root/$name"
  done
  printf 'TOOLCHAIN tool=%s version=%s root=%s\n' "$tool" "$output" "$root" >&2
  [[ -z "$libraries" ]] || printf '%s\n' "$libraries"
  exit 0
)

toolchain_resolve() (
  local tool="$1" external="${2:-}" cache="${3:-}" prefix field root libraries
  local -A entries=() selection=() identity=()
  toolchain_read_settings "$TOOLCHAIN_ROOT/tools/toolchains.lock" entries || exit $?
  if [[ -f "$TOOLCHAIN_ROOT/toolchains.local" ]]; then
    toolchain_read_settings "$TOOLCHAIN_ROOT/toolchains.local" selection || exit $?
  fi
  external="${external:-${selection[linux-amd64.roots.$tool]:-}}"
  cache="${cache:-${selection[linux-amd64.cache_root]:-$TOOLCHAIN_ROOT/build/toolchains/prebuilt}}"
  [[ "$cache" == /* && ( -z "$external" || "$external" == /* ) ]] ||
    toolchain_usage_error "Tool roots and cache_root must be absolute paths"
  prefix="$tool.hosts.linux-amd64"
  local release="${entries[$tool.release]:-}" version="${entries[$tool.version]:-}" commit="${entries[$tool.commit]:-}"
  local asset="${entries[$prefix.asset]:-}" url="${entries[$prefix.url]:-}"
  local digest="${entries[$prefix.sha256]:-}" size="${entries[$prefix.size]:-}"
  [[ "$release" =~ ^[a-zA-Z0-9._-]+$ && -n "$version" && -n "$asset" && "$asset" != */* &&
    "$asset" != . && "$asset" != .. && -n "$url" && "$digest" =~ ^[0-9a-f]{64}$ && "$size" =~ ^[0-9]+$ ]] ||
    toolchain_usage_error "Incomplete or invalid declared asset: $tool"
  root="${external:-$cache/$tool/$release/linux-amd64}"
  if [[ -n "$external" || -e "$root" ]]; then
    if [[ -z "$external" ]]; then
      toolchain_read_settings "$root/.entasis-toolchain" identity || exit $?
      for field in asset url sha256 size; do
        [[ "${identity[$field]:-}" == "${entries[$prefix.$field]}" ]] ||
          toolchain_error "Cache identity differs from declared asset: $root"
      done
    fi
    libraries="$(toolchain_inspect "$root" "$tool" "$version" "$commit" "$cache")" || exit $?
  else
    [[ "$tool" != icu70 ]] || command -v dpkg-deb >/dev/null ||
      toolchain_error "dpkg-deb is required to extract the declared ICU runtime"
    local parent="${root%/*}" lock="$root.lock" work archive candidate
    mkdir -p "$parent" || exit $?
    (set -o noclobber; printf '%s\n' "$BASHPID" > "$lock") ||
      toolchain_error "Toolchain installation already has a writer: $root"
    work=""
    trap 'status=$?; if ((status != 0)); then printf "INSTALL_FAILED partial=%s\n" "$work" >&2; fi; rm -f "$lock"; exit "$status"' EXIT
    work="$(mktemp -d "$parent/.install-XXXXXX")" || exit $?
    archive="$work/$asset"
    printf 'DOWNLOAD tool=%s release=%s bytes=%s destination=%s partial=%s\n' "$tool" "$release" "$size" "$root" "$work" >&2
    curl --fail --location --silent --show-error --connect-timeout 30 --max-time 900 --speed-limit 1 --speed-time 30 \
      --output "$archive" --url "$url" || exit $?
    [[ "$(stat -c %s "$archive")" == "$size" ]] || toolchain_error "Archive size or SHA-256 mismatch: $archive"
    local actual
    actual="$(sha256sum "$archive")" || exit $?
    [[ "${actual:0:64}" == "$digest" ]] || toolchain_error "Archive size or SHA-256 mismatch: $archive"
    printf 'EXTRACTING %s\n' "$archive" >&2
    toolchain_extract "$archive" "$work/extracted" || exit $?
    candidate="$work/extracted"
    if [[ "$tool" != icu70 ]]; then
      shopt -s nullglob dotglob
      local -a children=("$candidate"/*)
      if ((${#children[@]} == 1)) && [[ -d "${children[0]}" ]]; then candidate="${children[0]}"; fi
    fi
    libraries="$(toolchain_inspect "$candidate" "$tool" "$version" "$commit" "$cache")" || exit $?
    for field in asset url sha256 size; do
      printf '%s=%s\n' "$field" "${entries[$prefix.$field]}"
    done > "$candidate/.entasis-toolchain" || exit $?
    mv -T "$candidate" "$root" || exit $?
    rm -rf "$work" || exit $?
  fi
  printf '%s\n' "$root"
  [[ "$tool" != llvm ]] || printf '%s\n' "$libraries"
  exit 0
)

toolchain_use() {
  local tool="$1" resolved
  resolved="$(toolchain_resolve "$tool" "${2:-}" "${3:-}")" || return $?
  case "$tool" in
    odin) ODIN_BIN="$resolved/odin"; export ODIN_ROOT="$resolved" ;;
    llvm)
      LLVM_ROOT="${resolved%%$'\n'*}"
      export LD_LIBRARY_PATH="${resolved#*$'\n'}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
      export PATH="$LLVM_ROOT/bin:$PATH"
      ;;
    cmake) CMAKE_BIN="$resolved/bin/cmake"; CTEST_BIN="$resolved/bin/ctest" ;;
    ninja) NINJA_BIN="$resolved/ninja" ;;
  esac
}

toolchain_odin() {
  toolchain_host
  local explicit="$1" cache="${2:-}"
  if [[ -n "$explicit" ]]; then
    [[ "$explicit" == /* && "${explicit##*/}" == odin && -f "$explicit" ]] || toolchain_usage_error "--odin must name an existing absolute native odin executable"
    explicit="$(dirname "$explicit")"
  fi
  toolchain_use odin "$explicit" "$cache"
  toolchain_use llvm "" "$cache"
  [[ -f /usr/include/stdio.h ]] || toolchain_usage_error "Linux C development headers are required (libc6-dev on Ubuntu)"
}

toolchain_compilers() {
  local compiler="$1" cache="${2:-}"
  toolchain_host
  case "$compiler" in
    gcc|all)
      GCC_BIN="$(toolchain_system gcc)"
      GXX_BIN="$(toolchain_system g++)"
      ;;
  esac
  case "$compiler" in
    clang|all) toolchain_use llvm "" "$cache" ;;
  esac
}

toolchain_system() {
  local name="$1" selected=""
  local -A settings=()
  if [[ -f "$TOOLCHAIN_ROOT/toolchains.local" ]]; then
    toolchain_read_settings "$TOOLCHAIN_ROOT/toolchains.local" settings || return $?
    selected="${settings[linux-amd64.system.$name]:-}"
  fi
  if [[ -z "$selected" ]]; then
    selected="$(command -v "$name")" || toolchain_usage_error "required native system tool is missing: $name"
  fi
  [[ "$selected" == /* && -x "$selected" && "$selected" != *.exe ]] || toolchain_usage_error "invalid native system tool $name: $selected"
  printf '%s\n' "$selected"
}

toolchain_setup() {
  local usage="$1" compiler="$2" cache="${3:-}"
  case "$usage" in
    odin|c-abi-source) toolchain_odin "" "$cache" ;;
    c-abi-sdk)
      toolchain_compilers "$compiler" "$cache"
      toolchain_use cmake "" "$cache"
      toolchain_use ninja "" "$cache"
      ;;
  esac
}
