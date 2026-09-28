param()
$linuxRelease = ""
$windowsRelease = ""
$dryRun = 0
$seen = @{}
for ($index = 0; $index -lt $args.Count; $index++)
{
    $option = [string]$args[$index]
    if ($seen.ContainsKey($option))
    {
        [Console]::Error.WriteLine("Repeated argument: $option")
        exit 2
    }
    $seen[$option] = 1
    if ($option -ieq "-DryRun")
    {
        $dryRun = 1
        continue
    }
    if ($option -notin @("-LinuxRelease", "-WindowsRelease") -or $index + 1 -ge $args.Count -or
        [string]::IsNullOrWhiteSpace([string]$args[$index + 1]) -or ([string]$args[$index + 1]).StartsWith("-"))
    {
        [Console]::Error.WriteLine("usage: scripts/windows/release/publish_release_windows.ps1 -LinuxRelease <directory> -WindowsRelease <directory> [-DryRun]")
        exit 2
    }
    $index++
    if ($option -ieq "-LinuxRelease")
    {
        $linuxRelease = [string]$args[$index]
    }
    else
    {
        $windowsRelease = [string]$args[$index]
    }
}
if ($linuxRelease.Length -eq 0 -or $windowsRelease.Length -eq 0)
{
    [Console]::Error.WriteLine("Both -LinuxRelease and -WindowsRelease are required")
    exit 2
}
$ErrorActionPreference = "Stop"
$root = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot "../../.."))
$stage = "inputs"
$work = ""
$releaseUrl = ""
$script:publishExit = 1
function Invoke-PublishTool([string]$Tool, [string[]]$Arguments)
{
    & $Tool @Arguments
    [int]$code = $LASTEXITCODE
    if ($code -ne 0)
    {
        $script:publishExit = $code
        throw "Command failed ($code): $Tool $($Arguments -join ' ')"
    }
}
function Assert-PublishBytes([string]$First, [string]$Second)
{
    if ((Get-FileHash -LiteralPath $First -Algorithm SHA256).Hash -cne
        (Get-FileHash -LiteralPath $Second -Algorithm SHA256).Hash)
    {
        throw "File bytes differ: $First and $Second"
    }
}
try
{
    foreach ($tool in @("git", "gh", "tar.exe"))
    {
        $null = Get-Command $tool -CommandType Application -ErrorAction Stop
    }
    $linuxRelease = [IO.Path]::GetFullPath($linuxRelease, $root)
    $windowsRelease = [IO.Path]::GetFullPath($windowsRelease, $root)
    $manifest = Get-Content -LiteralPath (Join-Path $root "tools/abi/abi_manifest.json") -Raw | ConvertFrom-Json
    $version = [string]$manifest.product.version_string
    $header = Get-Content -LiteralPath (Join-Path $root "include/entasis/base.h") -Raw
    if ($version -cnotmatch '^(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)(-[A-Za-z0-9.-]+)?$' -or
        $header -cnotmatch ('(?m)^#define ENTASIS_VERSION_STRING "' + [regex]::Escape($version) + '"\r?$'))
    {
        throw "Invalid version or manifest/header version mismatch"
    }
    $tag = "v$version"
    $title = "Entasis $version"
    $prerelease = if ($version.Contains("-"))
    {
        "true"
    }
    else
    {
        "false"
    }
    $origin = [string](Invoke-PublishTool "git" @("-C", $root, "remote", "get-url", "origin"))
    if ($origin -cmatch '^(?:https://github\.com/|git@github\.com:|ssh://git@github\.com/)([A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+)$')
    {
        $repo = $Matches[1] -creplace '\.git$', ''
    }
    else
    {
        throw "origin must be a standard GitHub HTTPS or SSH URL"
    }
    $commit = [string](Invoke-PublishTool "git" @("-C", $root, "rev-parse", "HEAD"))
    $work = Join-Path $root ("build/release-work/publish/windows." + [Guid]::NewGuid().ToString("N"))
    $null = New-Item -ItemType Directory -Path $work
    $assets = @(
        (Join-Path $linuxRelease "Entasis-$version-linux-x86_64-v3-c-sdk.tar.gz"),
        (Join-Path $linuxRelease "Entasis-$version-odin-source.tar.gz"),
        (Join-Path $windowsRelease "Entasis-$version-windows-x86_64-v3-c-sdk.zip"),
        (Join-Path $windowsRelease "Entasis-$version-odin-source.zip")
    )
    foreach ($archive in $assets)
    {
        $item = Get-Item -LiteralPath $archive
        if ($item.PSIsContainer -or ($item.Attributes -band [IO.FileAttributes]::ReparsePoint))
        {
            throw "Nonregular release asset: $archive"
        }
    }
    foreach ($directory in @($linuxRelease, $windowsRelease))
    {
        if (@(Get-ChildItem -LiteralPath $directory -Force).Count -ne 2)
        {
            throw "Expected exactly two release files in $directory"
        }
    }
    $stage = "archives"
    for ($index = 0; $index -lt $assets.Count; $index++)
    {
        $archive = $assets[$index]
        $name = [IO.Path]::GetFileName($archive) -creplace '\.(tar\.gz|zip)$', ''
        $destination = Join-Path $work "archive-$index"
        $null = New-Item -ItemType Directory -Path $destination
        if ($archive.EndsWith(".tar.gz", [StringComparison]::Ordinal))
        {
            $entries = @(Invoke-PublishTool "tar.exe" @("-tzf", $archive))
            $types = @(Invoke-PublishTool "tar.exe" @("-tvzf", $archive))
            if ($types -cmatch '^[^-d]')
            {
                throw "Unsupported TAR entry type: $archive"
            }
        }
        else
        {
            $zip = [IO.Compression.ZipFile]::OpenRead($archive)
            try
            {
                $entries = @($zip.Entries | ForEach-Object { $_.FullName })
                foreach ($entry in $zip.Entries)
                {
                    if ((($entry.ExternalAttributes -shr 16) -band 0xF000) -eq 0xA000)
                    {
                        throw "Unsupported ZIP symlink: $archive"
                    }
                }
            }
            finally
            {
                $zip.Dispose()
            }
        }
        foreach ($entry in $entries)
        {
            if (-not $entry.StartsWith("$name/", [StringComparison]::Ordinal) -or
                $entry.Contains('\') -or "/$entry/" -cmatch '/\.\./')
            {
                throw "Invalid archive entry: $entry"
            }
        }
        if ($archive.EndsWith(".tar.gz", [StringComparison]::Ordinal))
        {
            Invoke-PublishTool "tar.exe" @("-xzf", $archive, "-C", $destination)
        }
        else
        {
            [IO.Compression.ZipFile]::ExtractToDirectory($archive, $destination)
        }
        if ($name.EndsWith("-odin-source", [StringComparison]::Ordinal))
        {
            $embedded = Get-Content -LiteralPath (Join-Path $destination "$name/src/entasis/version.odin") -Raw
            $versionPattern = '(?m)^VERSION_STRING :: "' + [regex]::Escape($version) + '";\r?$'
        }
        else
        {
            $embedded = Get-Content -LiteralPath (Join-Path $destination "$name/include/entasis/base.h") -Raw
            $versionPattern = '(?m)^#define ENTASIS_VERSION_STRING "' + [regex]::Escape($version) + '"\r?$'
        }
        if ($embedded -cnotmatch $versionPattern)
        {
            throw "Embedded product version mismatch: $archive"
        }
    }
    $sourceLinux = Join-Path $work "archive-1/Entasis-$version-odin-source"
    $sourceWindows = Join-Path $work "archive-3/Entasis-$version-odin-source"
    $files = @(Get-ChildItem -LiteralPath $sourceLinux -File -Recurse -Force | ForEach-Object { [IO.Path]::GetRelativePath($sourceLinux, $_.FullName).Replace('\', '/') } | Sort-Object -CaseSensitive)
    $windowsFiles = @(Get-ChildItem -LiteralPath $sourceWindows -File -Recurse -Force | ForEach-Object { [IO.Path]::GetRelativePath($sourceWindows, $_.FullName).Replace('\', '/') } | Sort-Object -CaseSensitive)
    if (($files -join "`n") -cne ($windowsFiles -join "`n"))
    {
        throw "Source archive file sets differ"
    }
    $sourcePaths = @{
        "README.md" = "sdk/odin/README.md"
        "docs/BUILDING.md" = "sdk/odin/BUILDING.md"
        "docs/README.md" = "sdk/odin/DOCUMENTATION.md"
    }
    foreach ($file in $files)
    {
        $sourcePath = if ($sourcePaths.ContainsKey($file))
        {
            $sourcePaths[$file]
        }
        else
        {
            $file
        }
        $expected = [string](Invoke-PublishTool "git" @("-C", $root, "rev-parse", "${commit}:$sourcePath"))
        $actual = [string](Invoke-PublishTool "git" @("-C", $root, "hash-object", "--path", $sourcePath, (Join-Path $sourceLinux $file)))
        if ($actual -cne $expected)
        {
            throw "Linux source differs from checkout commit: $file"
        }
        $actual = [string](Invoke-PublishTool "git" @("-C", $root, "hash-object", "--path", $sourcePath, (Join-Path $sourceWindows $file)))
        if ($actual -cne $expected)
        {
            throw "Windows source differs from checkout commit: $file"
        }
    }
    foreach ($file in @("README.md", "docs/README.md", "docs/BUILDING.md", "docs/LIMITS.md", "CHANGELOG.md", "LICENSE", "NOTICE",
        "src/entasis/version.odin", "src/entasis/package.odin", "src/entasis_cooking/cooking.odin",
        "build_odin_linux.sh", "build_odin_windows.ps1", "scripts/linux/examples/run_examples.sh", "scripts/windows/examples/run_examples.ps1",
        "tools/toolchains.lock", "docs/odin/GETTING-STARTED.md", "docs/odin/BUILDING-LINUX.md", "docs/odin/BUILDING-WINDOWS.md",
        "docs/odin/EXAMPLES.md", "examples/headless/falling_box/main.odin", "examples/headless/convex_hulls/main.odin"))
    {
        if (-not (Test-Path -LiteralPath (Join-Path $sourceLinux $file) -PathType Leaf))
        {
            throw "Missing required source file: $file"
        }
    }
    $stage = "notes"
    $changelog = Get-Content -LiteralPath (Join-Path $sourceLinux "CHANGELOG.md") -Raw
    $sections = [regex]::Matches($changelog, '(?ms)^## ' + [regex]::Escape($version) + '\r?\n(.*?)(?=^## |\z)')
    if ($sections.Count -ne 1)
    {
        throw "Expected one changelog section for $version"
    }
    $notes = $sections[0].Groups[1].Value.Replace("`r`n", "`n").TrimEnd()
    if ($notes -notmatch '[A-Za-z0-9]' -or $notes -match '(?im)^[\s-]*(TODO|TBD|Initial version of Entasis)\s*$')
    {
        throw "Missing substantive changelog notes for $version"
    }
    $download = "https://github.com/$repo/releases/download/$tag"
    $selection = @(
        '## Downloads', '',
        "- Native Odin: [ZIP]($download/Entasis-$version-odin-source.zip) or [TAR.GZ]($download/Entasis-$version-odin-source.tar.gz), equivalent alternatives for either supported host. Compile the source into your application without an Entasis C ABI library",
        "- Prebuilt C ABI for C, C++ and other languages using C interop: [Windows AMD64]($download/Entasis-$version-windows-x86_64-v3-c-sdk.zip) or [Linux AMD64]($download/Entasis-$version-linux-x86_64-v3-c-sdk.tar.gz), both requiring x86-64-v3 CPUs",
        "- Full repository: GitHub's automatic [source ZIP](https://github.com/$repo/archive/refs/tags/$tag.zip) or [source TAR.GZ](https://github.com/$repo/archive/refs/tags/$tag.tar.gz), including tests and maintainer tools",
        '',
        '## Release changes', ''
    ) -join "`n"
    $notes = $selection + $notes
    $notesFile = Join-Path $work "notes.md"
    [IO.File]::WriteAllText($notesFile, $notes + "`n")
    $stage = "remote-preflight"
    Invoke-PublishTool "gh" @("auth", "status", "--hostname", "github.com")
    $reference = (@(Invoke-PublishTool "gh" @("api", "repos/$repo/git/ref/tags/$tag")) -join "`n") | ConvertFrom-Json
    while ($reference.object.type -ceq "tag")
    {
        $reference = (@(Invoke-PublishTool "gh" @("api", "repos/$repo/git/tags/$($reference.object.sha)")) -join "`n") | ConvertFrom-Json
    }
    if ($reference.object.type -cne "commit" -or $reference.object.sha -cne $commit)
    {
        throw "Remote $tag must resolve to checkout commit $commit"
    }
    $releaseJson = @(Invoke-PublishTool "gh" @("api", "--paginate", "repos/$repo/releases?per_page=100", "--jq", ".[] | select(.tag_name == `"$tag`")")) -join "`n"
    $release = if ($releaseJson.Length -gt 0)
    {
        $releaseJson | ConvertFrom-Json
    }
    else
    {
        $null
    }
    $expectedNames = [Collections.Generic.Dictionary[string, string]]::new([StringComparer]::Ordinal)
    foreach ($asset in $assets)
    {
        $expectedNames.Add([IO.Path]::GetFileName($asset), $asset)
    }
    $existingNames = [Collections.Generic.HashSet[string]]::new([StringComparer]::Ordinal)
    if ($null -ne $release)
    {
        $releaseUrl = [string]$release.html_url
        if (-not $release.draft)
        {
            throw "Release already published: $releaseUrl"
        }
        if ($release.name -cne $title -or $release.prerelease.ToString().ToLowerInvariant() -cne $prerelease -or
            ([string]$release.body).Replace("`r`n", "`n").TrimEnd() -cne $notes)
        {
            throw "Existing draft metadata or notes differs"
        }
        foreach ($asset in $release.assets)
        {
            if (-not $expectedNames.ContainsKey($asset.name) -or -not $existingNames.Add($asset.name))
            {
                throw "Unexpected/duplicate remote asset: $($asset.name)"
            }
            if ($asset.state -ceq "starter")
            {
                throw "Incomplete draft asset: repo=$repo tag=$tag id=$($asset.id) name=$($asset.name) state=$($asset.state) draft=$releaseUrl. Remove this incomplete asset from the draft through GitHub's maintainer interface, then rerun the same publisher command."
            }
        }
        if ($existingNames.Count -gt 0)
        {
            $existing = Join-Path $work "existing"
            $null = New-Item -ItemType Directory -Path $existing
            Invoke-PublishTool "gh" @("release", "download", $tag, "--repo", $repo, "--dir", $existing)
            foreach ($name in $existingNames)
            {
                Assert-PublishBytes $expectedNames[$name] (Join-Path $existing $name)
            }
        }
    }
    Write-Host "PUBLISH_INPUT repo=$repo commit=$commit tag=$tag title=$title prerelease=$prerelease"
    foreach ($asset in $assets)
    {
        Write-Host "asset=$asset"
    }
    Write-Host $notes
    if ($dryRun -eq 1)
    {
        Write-Host "PUBLISH_PREVIEW_OK work=$work"
        exit 0
    }
    $stage = "draft"
    if ($null -eq $release)
    {
        Invoke-PublishTool "gh" @("release", "create", $tag, "--repo", $repo, "--verify-tag", "--draft", "--title", $title, "--notes-file", $notesFile, "--prerelease=$prerelease")
        $releaseUrl = "https://github.com/$repo/releases/tag/$tag"
    }
    $stage = "upload"
    $missing = @($assets | Where-Object { -not $existingNames.Contains([IO.Path]::GetFileName($_)) })
    if ($missing.Count -gt 0)
    {
        Invoke-PublishTool "gh" (@("release", "upload", $tag) + $missing + @("--repo", $repo))
    }
    $stage = "remote-assets"
    $uploaded = Join-Path $work "uploaded"
    $null = New-Item -ItemType Directory -Path $uploaded
    Invoke-PublishTool "gh" @("release", "download", $tag, "--repo", $repo, "--dir", $uploaded)
    if (@(Get-ChildItem -LiteralPath $uploaded -Force).Count -ne $assets.Count)
    {
        throw "Remote asset count differs"
    }
    foreach ($asset in $assets)
    {
        Assert-PublishBytes $asset (Join-Path $uploaded ([IO.Path]::GetFileName($asset)))
    }
    $stage = "publish"
    $latest = @()
    if ($prerelease -eq "true")
    {
        $latest = @("--latest=false")
    }
    Invoke-PublishTool "gh" (@("release", "edit", $tag, "--repo", $repo, "--draft=false", "--verify-tag") + $latest)
    $stage = "readback"
    $published = (@(Invoke-PublishTool "gh" @("release", "view", $tag, "--repo", $repo, "--json", "tagName,name,body,isDraft,isPrerelease,assets,url")) -join "`n") | ConvertFrom-Json
    if ($published.tagName -cne $tag -or $published.name -cne $title -or $published.isDraft -or
        $published.isPrerelease.ToString().ToLowerInvariant() -cne $prerelease -or
        ([string]$published.body).Replace("`r`n", "`n").TrimEnd() -cne $notes -or
        @($published.assets).Count -ne $assets.Count)
    {
        throw "Published release metadata differs"
    }
    foreach ($asset in $published.assets)
    {
        if (-not $expectedNames.ContainsKey($asset.name))
        {
            throw "Published asset names differ"
        }
    }
    $releaseUrl = [string]$published.url
    Write-Host "PUBLISH_OK tag=$tag url=$releaseUrl work=$work"
}
catch
{
    [Console]::Error.WriteLine("PUBLISH_FAILED stage=$stage code=$script:publishExit work=$work release=$releaseUrl`n$_")
    exit $script:publishExit
}
