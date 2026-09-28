#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
linux_release="" windows_release="" dry_run=0
declare -A seen=()
usage() { printf 'usage: scripts/linux/release/publish_release_linux.sh --linux-release <directory> --windows-release <directory> [--dry-run]\n' >&2; exit 2; }
while (($#)); do
  option="$1"
  [[ -n "$option" && -z "${seen[$option]:-}" ]] || usage
  seen[$option]=1
  case "$option" in
    --dry-run) dry_run=1; shift ;;
    --linux-release|--windows-release)
      (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || usage
      if [[ "$option" == --linux-release ]]; then linux_release="$2"; else windows_release="$2"; fi
      shift 2 ;;
    *) usage ;;
  esac
done
[[ -n "$linux_release" && -n "$windows_release" ]] || usage
stage=inputs work="" release_url=""
trap 'code=$?; printf "PUBLISH_FAILED stage=%s code=%s work=%s release=%s\n" "$stage" "$code" "$work" "$release_url" >&2; exit "$code"' ERR
fail() { printf '%s\n' "$*" >&2; return 1; }
for tool in git gh tar gzip unzip base64; do command -v "$tool" >/dev/null || fail "Missing publishing prerequisite: $tool"; done
[[ "$linux_release" == /* ]] || linux_release="$ROOT/$linux_release"
[[ "$windows_release" == /* ]] || windows_release="$ROOT/$windows_release"
linux_release="$(realpath -e "$linux_release")"
windows_release="$(realpath -e "$windows_release")"
version="$(sed -n 's/^#define ENTASIS_VERSION_STRING "\(.*\)"/\1/p' "$ROOT/include/entasis/base.h")"
manifest_version="$(sed -n 's/^[[:space:]]*"version_string": "\([^"]*\)".*/\1/p' "$ROOT/tools/abi/abi_manifest.json")"
[[ "$version" =~ ^(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)(-[A-Za-z0-9.-]+)?$ && "$version" == "$manifest_version" ]] || fail 'Manifest/header product version mismatch or invalid version'
tag="v$version" title="Entasis $version" prerelease=false
[[ "$version" != *-* ]] || prerelease=true
origin="$(git -C "$ROOT" remote get-url origin)"
case "$origin" in
  https://github.com/*) repo="${origin#https://github.com/}" ;;
  git@github.com:*) repo="${origin#git@github.com:}" ;;
  ssh://git@github.com/*) repo="${origin#ssh://git@github.com/}" ;;
  *) fail 'origin must be a standard GitHub HTTPS or SSH URL' ;;
esac
repo="${repo%.git}"
[[ "$repo" =~ ^[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+$ ]] || fail 'Invalid GitHub origin repository'
commit="$(git -C "$ROOT" rev-parse HEAD)"
mkdir -p "$ROOT/build/release-work/publish"
work="$(mktemp -d "$ROOT/build/release-work/publish/linux.XXXXXX")"
assets=("$linux_release/Entasis-$version-linux-x86_64-v3-c-sdk.tar.gz" "$linux_release/Entasis-$version-odin-source.tar.gz"
  "$windows_release/Entasis-$version-windows-x86_64-v3-c-sdk.zip" "$windows_release/Entasis-$version-odin-source.zip")
for archive in "${assets[@]}"; do
  [[ -f "$archive" && ! -L "$archive" ]] || fail "Missing/nonregular archive: $archive"
done
for directory in "$linux_release" "$windows_release"; do
  mapfile -t entries < <(find "$directory" -mindepth 1 -maxdepth 1 -printf '%f\n')
  ((${#entries[@]} == 2)) || fail "Expected exactly two release files in $directory"
done
stage=archives
for i in "${!assets[@]}"; do
  archive="${assets[$i]}" destination="$work/archive-$i"
  name="${archive##*/}" name="${name%.tar.gz}" name="${name%.zip}"
  mkdir "$destination"
  if [[ "$archive" == *.tar.gz ]]; then
    tar -tzf "$archive" > "$work/entries-$i"
    tar -tvzf "$archive" > "$work/types-$i"
    if grep -qv '^[-d]' "$work/types-$i"; then fail "Unsupported archive entry type: $archive"; fi
  else
    unzip -Z1 "$archive" > "$work/entries-$i"
    unzip -Z -l "$archive" > "$work/types-$i"
    if grep -q '^l' "$work/types-$i"; then fail "Unsupported archive symlink: $archive"; fi
  fi
  while IFS= read -r entry; do
    [[ "$entry" == "$name/"* && "$entry" != *\\* && "/$entry/" != */../* ]] || fail "Invalid archive entry: $entry"
  done < "$work/entries-$i"
  if [[ "$archive" == *.tar.gz ]]; then tar -xzf "$archive" --no-same-owner -C "$destination"; else unzip -q "$archive" -d "$destination"; fi
  payload="$destination/$name"
  if [[ "$name" == *-odin-source ]]; then
    embedded="$(sed -n 's/^VERSION_STRING :: "\([^"]*\)";\r\{0,1\}$/\1/p' "$payload/src/entasis/version.odin")"
  else
    embedded="$(sed -n 's/^#define ENTASIS_VERSION_STRING "\(.*\)"/\1/p' "$payload/include/entasis/base.h")"
  fi
  [[ "$embedded" == "$version" ]] || fail "Embedded product version mismatch: $archive"
done
source_linux="$work/archive-1/Entasis-$version-odin-source"
source_windows="$work/archive-3/Entasis-$version-odin-source"
(cd "$source_linux" && find . -type f -printf '%P\n' | LC_ALL=C sort) > "$work/source-linux.files"
(cd "$source_windows" && find . -type f -printf '%P\n' | LC_ALL=C sort) > "$work/source-windows.files"
cmp "$work/source-linux.files" "$work/source-windows.files"
declare -A source_paths=([README.md]=sdk/odin/README.md [docs/BUILDING.md]=sdk/odin/BUILDING.md
  [docs/README.md]=sdk/odin/DOCUMENTATION.md)
while IFS= read -r file; do
  source_path="${source_paths[$file]:-$file}"
  expected="$(git -C "$ROOT" rev-parse "$commit:$source_path")"
  actual="$(git -C "$ROOT" hash-object --path "$source_path" "$source_linux/$file")"
  [[ "$actual" == "$expected" ]] || fail "Linux source differs from checkout commit: $file"
  actual="$(git -C "$ROOT" hash-object --path "$source_path" "$source_windows/$file")"
  [[ "$actual" == "$expected" ]] || fail "Windows source differs from checkout commit: $file"
done < "$work/source-linux.files"
for file in README.md docs/README.md docs/BUILDING.md docs/LIMITS.md CHANGELOG.md LICENSE NOTICE \
  src/entasis/version.odin src/entasis/package.odin src/entasis_cooking/cooking.odin \
  build_odin_linux.sh build_odin_windows.ps1 scripts/linux/examples/run_examples.sh scripts/windows/examples/run_examples.ps1 \
  tools/toolchains.lock docs/odin/GETTING-STARTED.md docs/odin/BUILDING-LINUX.md docs/odin/BUILDING-WINDOWS.md \
  docs/odin/EXAMPLES.md examples/headless/falling_box/main.odin examples/headless/convex_hulls/main.odin; do
  [[ -f "$source_linux/$file" ]] || fail "Missing required source file: $file"
done
stage=notes
awk -v heading="## $version" '
  { sub(/\r$/, "") }
  /^## / { active=($0==heading); if(active) count++; next }
  active { print }
  END { if(count!=1) exit 1 }
' "$source_linux/CHANGELOG.md" > "$work/notes.raw"
sed -e ':a' -e '/^[[:space:]]*$/{$d;N;ba;}' -e '$s/[[:space:]]*$//' "$work/notes.raw" > "$work/changelog.md"
if ! grep -q '[[:alnum:]]' "$work/changelog.md" || grep -Eiq '^[[:space:]-]*(TODO|TBD|Initial version of Entasis)[[:space:]]*$' "$work/changelog.md"; then fail "Missing substantive changelog notes for $version"; fi
download="https://github.com/$repo/releases/download/$tag"
{
  printf '%s\n' '## Downloads' '' \
    "- Native Odin: [ZIP]($download/Entasis-$version-odin-source.zip) or [TAR.GZ]($download/Entasis-$version-odin-source.tar.gz), equivalent alternatives for either supported host. Compile the source into your application without an Entasis C ABI library" \
    "- Prebuilt C ABI for C, C++ and other languages using C interop: [Windows AMD64]($download/Entasis-$version-windows-x86_64-v3-c-sdk.zip) or [Linux AMD64]($download/Entasis-$version-linux-x86_64-v3-c-sdk.tar.gz), both requiring x86-64-v3 CPUs" \
    "- Full repository: GitHub's automatic [source ZIP](https://github.com/$repo/archive/refs/tags/$tag.zip) or [source TAR.GZ](https://github.com/$repo/archive/refs/tags/$tag.tar.gz), including tests and maintainer tools" \
    '' \
    '## Release changes'
  cat "$work/changelog.md"
} > "$work/notes.md"
stage=remote-preflight
gh auth status --hostname github.com > "$work/auth.txt" 2>&1
gh api "repos/$repo/git/ref/tags/$tag" --jq '[.object.type,.object.sha]|@tsv' > "$work/tag.txt"
IFS=$'\t' read -r object_type remote_commit < "$work/tag.txt"
while [[ "$object_type" == tag ]]; do
  gh api "repos/$repo/git/tags/$remote_commit" --jq '[.object.type,.object.sha]|@tsv' > "$work/tag.txt"
  IFS=$'\t' read -r object_type remote_commit < "$work/tag.txt"
done
[[ "$object_type" == commit && "$remote_commit" == "$commit" ]] || fail "Remote $tag must resolve to checkout commit $commit"
query=".[] | select(.tag_name == \"$tag\") | [.id,.draft,.prerelease,.html_url,.name,(.body // \"\" | @base64),([.assets[] | [.id,.name,.state] | @tsv] | join(\"\\n\") | @base64)] | @tsv"
gh api --paginate "repos/$repo/releases?per_page=100" --jq "$query" > "$work/release.tsv"
release_id="" remote_assets=() remote_names=()
if [[ -s "$work/release.tsv" ]]; then
  IFS=$'\t' read -r release_id remote_draft remote_prerelease release_url remote_title notes64 assets64 < "$work/release.tsv"
  [[ "$remote_draft" == true ]] || fail "Release already published: $release_url"
  [[ "$remote_title" == "$title" && "$remote_prerelease" == "$prerelease" ]] || fail 'Existing draft metadata differs'
  printf '%s' "$notes64" | base64 -d > "$work/remote-notes.md"
  [[ "$(cat "$work/remote-notes.md")" == "$(cat "$work/notes.md")" ]] || fail 'Existing draft notes differ'
  if [[ -n "$assets64" ]]; then mapfile -t remote_assets < <(printf '%s' "$assets64" | base64 -d); fi
fi
declare -A expected_names=() existing_names=()
for asset in "${assets[@]}"; do expected_names["${asset##*/}"]="$asset"; done
for asset in "${remote_assets[@]}"; do
  IFS=$'\t' read -r asset_id name asset_state <<< "$asset"
  [[ -n "${expected_names[$name]:-}" && -z "${existing_names[$name]:-}" ]] || fail "Unexpected/duplicate remote asset: $name"
  [[ "$asset_state" != starter ]] || fail "Incomplete draft asset: repo=$repo tag=$tag id=$asset_id name=$name state=$asset_state draft=$release_url. Remove this incomplete asset from the draft through GitHub's maintainer interface, then rerun the same publisher command."
  existing_names[$name]=1
  remote_names+=("$name")
done
if ((${#remote_names[@]})); then
  mkdir "$work/existing"
  gh release download "$tag" --repo "$repo" --dir "$work/existing"
  for name in "${remote_names[@]}"; do cmp "${expected_names[$name]}" "$work/existing/$name"; done
fi
printf 'PUBLISH_INPUT repo=%s commit=%s tag=%s title=%s prerelease=%s\n' "$repo" "$commit" "$tag" "$title" "$prerelease"
printf 'asset=%s\n' "${assets[@]}"
cat "$work/notes.md"
if ((dry_run)); then printf '\nPUBLISH_PREVIEW_OK work=%s\n' "$work"; exit 0; fi
stage=draft
if [[ -z "$release_id" ]]; then
  gh release create "$tag" --repo "$repo" --verify-tag --draft --title "$title" --notes-file "$work/notes.md" --prerelease="$prerelease"
  release_url="https://github.com/$repo/releases/tag/$tag"
fi
stage=upload
missing=()
for asset in "${assets[@]}"; do [[ -n "${existing_names[${asset##*/}]:-}" ]] || missing+=("$asset"); done
if ((${#missing[@]})); then gh release upload "$tag" "${missing[@]}" --repo "$repo"; fi
stage=remote-assets
mkdir "$work/uploaded"
gh release download "$tag" --repo "$repo" --dir "$work/uploaded"
mapfile -t uploaded < <(find "$work/uploaded" -mindepth 1 -maxdepth 1 -type f -printf '%f\n')
((${#uploaded[@]} == ${#assets[@]})) || fail 'Remote asset count differs'
for asset in "${assets[@]}"; do cmp "$asset" "$work/uploaded/${asset##*/}"; done
stage=publish
latest=()
[[ "$prerelease" != true ]] || latest=(--latest=false)
gh release edit "$tag" --repo "$repo" --draft=false --verify-tag "${latest[@]}"
stage=readback
gh release view "$tag" --repo "$repo" --json tagName,name,body,isDraft,isPrerelease,assets,url --jq '[.tagName,.name,.isDraft,.isPrerelease,.url,(.body|@base64),([.assets[].name]|sort|join("\n")|@base64)]|@tsv' > "$work/published.tsv"
IFS=$'\t' read -r published_tag published_title published_draft published_prerelease release_url notes64 assets64 < "$work/published.tsv"
[[ "$published_tag" == "$tag" && "$published_title" == "$title" && "$published_draft" == false && "$published_prerelease" == "$prerelease" ]] || fail 'Published release metadata differs'
[[ "$(printf '%s' "$notes64" | base64 -d)" == "$(cat "$work/notes.md")" ]] || fail 'Published release notes differ'
[[ "$(printf '%s' "$assets64" | base64 -d)" == "$(printf '%s\n' "${!expected_names[@]}" | LC_ALL=C sort)" ]] || fail 'Published asset names differ'
printf 'PUBLISH_OK tag=%s url=%s work=%s\n' "$tag" "$release_url" "$work"
