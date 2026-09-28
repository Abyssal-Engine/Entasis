#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/../lib/toolchain_common.sh"
scope=all
if (($#)); then
  (($# == 2)) && [[ "$1" == --scope ]] || toolchain_usage_error "expected --scope all|fixtures"
  scope="$2"
fi
case "$scope" in all|fixtures) ;; *) toolchain_usage_error "--scope must be all or fixtures" ;; esac
toolchain_host
source_root="$TOOLCHAIN_ROOT"
mkdir -p "$source_root/build/toolchain-tests"
test_root="$(mktemp -d "$source_root/build/toolchain-tests/linux.XXXXXX")/provisioning with spaces"
mkdir -p "$test_root/scripts/linux/lib" "$test_root/scripts/linux/toolchains" "$test_root/tools" "$test_root/fixture"
cp "$source_root/scripts/linux/lib/toolchain_common.sh" "$test_root/scripts/linux/lib/"
cp "$source_root/scripts/linux/toolchains/setup_toolchains.sh" "$test_root/scripts/linux/toolchains/"
declare -A entries=()
toolchain_read_settings "$source_root/tools/toolchains.lock" entries

write_fixture_lock() {
  local key
  for key in "${!entries[@]}"; do printf '%s=%s\n' "$key" "${entries[$key]}"; done > "$test_root/tools/toolchains.lock"
}

select_fixture_archive() {
  local tool="$1" archive="$2" digest
  digest="$(sha256sum "$archive")"
  entries[$tool.hosts.linux-amd64.asset]="${archive##*/}"
  entries[$tool.hosts.linux-amd64.url]="file://${archive// /%20}"
  entries[$tool.hosts.linux-amd64.sha256]="${digest:0:64}"
  entries[$tool.hosts.linux-amd64.size]="$(stat -c %s "$archive")"
}

native_ninja="$(command -v ninja)" || toolchain_usage_error "The Linux provisioning fixtures require native Ninja"
ninja_version="$("$native_ninja" --version)"
[[ "$ninja_version" =~ [0-9]+(\.[0-9]+)+ ]] || toolchain_error "Cannot read fixture Ninja version"
ninja_version="${BASH_REMATCH[0]}"
cp "$native_ninja" "$test_root/fixture/ninja"
(cd "$test_root/fixture" && zip -q "$test_root/fixture.zip" ninja)
entries[ninja.release]=fixture
entries[ninja.version]="$ninja_version"
select_fixture_archive ninja "$test_root/fixture.zip"
write_fixture_lock
cp "$test_root/tools/toolchains.lock" "$test_root/fixture.lock"
source "$test_root/scripts/linux/lib/toolchain_common.sh"
installed="$(toolchain_resolve ninja)"
mv "$test_root/fixture.zip" "$test_root/fixture.offline"
[[ "$(toolchain_resolve ninja)" == "$installed" ]]
[[ "$(toolchain_resolve ninja "$installed")" == "$installed" ]]
entries[ninja.version]=0.0.0
write_fixture_lock
mkdir "$test_root/wrong-host"
printf 'MZ incompatible host' > "$test_root/wrong-host/ninja"
if toolchain_resolve ninja "$installed" > "$test_root/mismatch.log" 2>&1; then
  toolchain_error "mismatched external root unexpectedly passed"
fi
if toolchain_resolve ninja "$test_root/wrong-host" > "$test_root/host.log" 2>&1; then
  toolchain_error "incompatible native executable unexpectedly passed"
fi
toolchain_read_settings "$test_root/fixture.lock" entries
write_fixture_lock
if toolchain_resolve ninja "$test_root/missing" > "$test_root/missing.log" 2>&1; then
  toolchain_error "missing external root unexpectedly passed"
fi
entries[ninja.release]=corrupt
entries[ninja.hosts.linux-amd64.url]="file://${test_root// /%20}/fixture.offline"
printf -v entries[ninja.hosts.linux-amd64.sha256] '%064d' 0
write_fixture_lock
if toolchain_resolve ninja > "$test_root/digest.log" 2>&1; then
  toolchain_error "corrupt digest unexpectedly passed"
fi
[[ -x "$installed/ninja" ]]
printf x > "$test_root/rejected"
tar -czf "$test_root/invalid.tar.gz" --transform='s|^rejected$|../rejected|' \
  -C "$test_root/fixture" ninja -C "$test_root" rejected
entries[ninja.release]=extraction-failure
select_fixture_archive ninja "$test_root/invalid.tar.gz"
write_fixture_lock
if toolchain_resolve ninja > "$test_root/extraction.log" 2>&1; then
  toolchain_error "invalid extraction unexpectedly passed"
fi
partial=("$test_root/build/toolchains/prebuilt/ninja/extraction-failure/".install-*/extracted/ninja)
[[ -f "${partial[0]}" && -x "$installed/ninja" ]]
toolchain_read_settings "$test_root/fixture.lock" entries
entries[ninja.release]=writer-exclusion
entries[ninja.hosts.linux-amd64.url]="file://${test_root// /%20}/fixture.offline"
write_fixture_lock
writer_root="$test_root/build/toolchains/prebuilt/ninja/writer-exclusion/linux-amd64"
writer_lock="$writer_root.lock"
mkdir -p "${writer_root%/*}"
(set -o noclobber; printf '%s\n' "$BASHPID" > "$writer_lock")
writer_token="$(< "$writer_lock")"
if toolchain_resolve ninja > "$test_root/concurrent.log" 2>&1; then
  toolchain_error "concurrent writer unexpectedly passed"
fi
grep -Fq "Toolchain installation already has a writer: $writer_root" "$test_root/concurrent.log"
[[ ! -e "$writer_root" && -f "$writer_lock" && "$(< "$writer_lock")" == "$writer_token" ]]
rm "$writer_lock"
[[ "$(toolchain_resolve ninja)" == "$writer_root" && -x "$writer_root/ninja" && ! -e "$writer_lock" ]]
toolchain_read_settings "$test_root/fixture.lock" entries
write_fixture_lock
[[ "$(toolchain_resolve ninja)" == "$installed" ]]
for arguments in '--unknown' '--usage' '--usage invalid' '--usage odin --usage odin' '--usage odin --compiler gcc'; do
  set +e
  "$test_root/scripts/linux/toolchains/setup_toolchains.sh" $arguments > "$test_root/parser.log" 2>&1
  status=$?
  set -e
  [[ "$status" == 2 ]] || toolchain_error "parser returned $status: $arguments"
done
set +e
"$test_root/scripts/linux/toolchains/setup_toolchains.sh" --usage "" > "$test_root/empty.log" 2>&1
status=$?
set -e
[[ "$status" == 2 ]]
package="$test_root/icu-package"
libraries="$package/usr/lib/x86_64-linux-gnu"
mkdir -p "$libraries" "$package/DEBIAN" "$test_root/fixture-llvm/bin"
printf 'Package: entasis-runtime-fixture\nVersion: 70.1-2\nArchitecture: amd64\nMaintainer: Entasis\nDescription: private loader fixture\n' > "$package/DEBIAN/control"
printf 'int entasis_fixture_value(void) { return 70; }\n' > "$test_root/runtime.c"
for name in icudata icuuc icui18n; do
  gcc -shared -fPIC "$test_root/runtime.c" "-Wl,-soname,lib$name.so.70" -o "$libraries/lib$name.so.70.1"
  ln -s "lib$name.so.70.1" "$libraries/lib$name.so.70"
done
dpkg-deb --build --nocheck --root-owner-group "$package" "$test_root/icu-fixture.deb"
select_fixture_archive icu70 "$test_root/icu-fixture.deb"
entries[llvm.version]="$ninja_version"
write_fixture_lock
cp "$native_ninja" "$test_root/fixture-llvm/bin/clang"
printf '#include <stdio.h>\nint entasis_fixture_value(void);\nint main(void) { if (entasis_fixture_value() != 70) return 1; puts("%s"); return 0; }\n' "$ninja_version" > "$test_root/runtime.c"
gcc "$test_root/runtime.c" "$libraries/libicuuc.so.70.1" -o "$test_root/fixture-llvm/bin/ld.lld"
ln -s ld.lld "$test_root/fixture-llvm/bin/clang++"
ln -s ld.lld "$test_root/fixture-llvm/bin/llvm-readobj"
(
  unset LD_LIBRARY_PATH
  if "$test_root/fixture-llvm/bin/ld.lld" --version > "$test_root/runtime-missing.log" 2>&1; then
    toolchain_error "unselected private runtime unexpectedly loaded"
  fi
  toolchain_use llvm "$test_root/fixture-llvm" "$test_root/runtime cache"
  "$LLVM_ROOT/bin/ld.lld" --version
  mv "$test_root/icu-fixture.deb" "$test_root/icu-fixture.offline"
  toolchain_use llvm "$test_root/fixture-llvm" "$test_root/runtime cache"
  "$LLVM_ROOT/bin/ld.lld" --version
)
printf 'TOOLCHAIN_TESTS_OK cases=cold,warm-offline,external,missing,digest,extraction-preservation,concurrent-writer,parser,private-runtime-cold,private-runtime-offline root=%s\n' "$test_root"
[[ "$scope" == all ]] || exit 0

official_root="$source_root/build/toolchain-tests/official-linux"
mkdir -p "$official_root/scripts/linux/lib" "$official_root/scripts/linux/toolchains" "$official_root/tools"
cp "$source_root/scripts/linux/lib/toolchain_common.sh" "$official_root/scripts/linux/lib/"
cp "$source_root/scripts/linux/toolchains/setup_toolchains.sh" "$official_root/scripts/linux/toolchains/setup_toolchains.sh"
cp "$source_root/tools/toolchains.lock" "$official_root/tools/"
source "$official_root/scripts/linux/lib/toolchain_common.sh"
toolchain_use odin
printf 'OFFICIAL_ODIN_SETUP_OK root=%s\n' "$official_root"
