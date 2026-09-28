param()
$outputRoot = "build/packages/source/windows"
if ($args.Count -gt 0)
{
    if ($args.Count -ne 2 -or $args[0] -ine "-OutputRoot" -or
        [string]::IsNullOrWhiteSpace([string]$args[1]) -or ([string]$args[1]).StartsWith("-"))
    {
        [Console]::Error.WriteLine("usage: package_source.ps1 [-OutputRoot <path>]")
        exit 2
    }
    $outputRoot = [string]$args[1]
}
$ErrorActionPreference = "Stop"
$root = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot "../../.."))
$outputRoot = [IO.Path]::GetFullPath($outputRoot, $root)
$manifest = Get-Content -LiteralPath (Join-Path $root "tools/abi/abi_manifest.json") -Raw | ConvertFrom-Json
$version = [string]$manifest.product.version_string
if ($version -notmatch '^[0-9]+\.[0-9]+\.[0-9]+(?:-[A-Za-z0-9.-]+)?$')
{
    throw "Invalid product version: $version"
}
$name = "Entasis-$version-odin-source"
$tree = Join-Path $outputRoot $name
$archivePath = Join-Path $outputRoot "$name.zip"
foreach ($path in @($tree, $archivePath))
{
    if (Test-Path -LiteralPath $path)
    {
        throw "Source output collision: $path"
    }
}
$roots = @("src/entasis", "src/entasis_cooking", "src/entasis_physics", "src/entasis_utilities", "examples/headless", "docs/odin")
$explicit = @("CHANGELOG.md", "LICENSE", "NOTICE", "docs/LIMITS.md", "build_odin_linux.sh", "build_odin_windows.ps1",
    "scripts/linux/examples/run_examples.sh", "scripts/windows/examples/run_examples.ps1",
    "scripts/linux/lib/common.sh", "scripts/linux/lib/toolchain_common.sh",
    "scripts/windows/lib/common.ps1", "scripts/windows/lib/toolchain_common.ps1",
    "scripts/linux/toolchains/setup_toolchains.sh", "scripts/windows/toolchains/setup_toolchains.ps1", "tools/toolchains.lock",
    "sdk/odin/README.md", "sdk/odin/BUILDING.md", "sdk/odin/DOCUMENTATION.md")
$installed = @{
    "sdk/odin/README.md" = "README.md"
    "sdk/odin/BUILDING.md" = "docs/BUILDING.md"
    "sdk/odin/DOCUMENTATION.md" = "docs/README.md"
}
$excludes = @(':(exclude)**/AGENTS.md', ':(exclude)**/Plan.*.md', ':(exclude)**/toolchains.local',
    ':(exclude)**/*.log', ':(exclude)**/*.stdout.txt', ':(exclude)**/*.stderr.txt')
$tracked = @(& git -C $root -c core.quotePath=false ls-files --cached --others --exclude-standard -- @roots @explicit @excludes)
if ($LASTEXITCODE -ne 0)
{
    exit $LASTEXITCODE
}
$deleted = @(& git -C $root -c core.quotePath=false ls-files --deleted -- @roots @explicit @excludes)
if ($LASTEXITCODE -ne 0)
{
    exit $LASTEXITCODE
}
$files = @(@($tracked | Where-Object { $deleted -cnotcontains $_ }) + $explicit | Sort-Object -Unique -CaseSensitive)
foreach ($file in $files)
{
    $source = Get-Item -LiteralPath (Join-Path $root $file)
    if ($source.PSIsContainer -or ($source.Attributes -band [IO.FileAttributes]::ReparsePoint))
    {
        throw "Nonregular source input: $file"
    }
}
$null = New-Item -ItemType Directory -Path $tree
$sources = @{}
foreach ($file in $files)
{
    $relative = if ($installed.ContainsKey($file))
    {
        $installed[$file]
    }
    else
    {
        $file
    }
    $sources[$relative] = $file
    $destination = Join-Path $tree $relative
    $null = New-Item -ItemType Directory -Force -Path (Split-Path $destination -Parent)
    [IO.File]::Copy((Join-Path $root $file), $destination)
}
$archive = [IO.Compression.ZipFile]::Open($archivePath, [IO.Compression.ZipArchiveMode]::Create)
try
{
    foreach ($file in $files)
    {
        $relative = if ($installed.ContainsKey($file))
        {
            $installed[$file]
        }
        else
        {
            $file
        }
        $entry = [IO.Compression.ZipFileExtensions]::CreateEntryFromFile($archive, (Join-Path $tree $relative), "$name/$relative", [IO.Compression.CompressionLevel]::Optimal)
        # regular Unix files: 0100755 for shell scripts, 0100644 for other payloads
        [int]$mode = if ($file.EndsWith(".sh", [StringComparison]::Ordinal))
        {
            33261
        }
        else
        {
            33188
        }
        $entry.ExternalAttributes = $mode -shl 16
    }
}
finally
{
    $archive.Dispose()
}
# .NET writes the current host as creator. mark our Unix mode fields as Unix
# in each central header (PKWARE APPNOTE 4.3.12 / 4.4.2). no source bytes change
$stream = [IO.File]::Open($archivePath, [IO.FileMode]::Open, [IO.FileAccess]::ReadWrite)
try
{
    $reader = [IO.BinaryReader]::new($stream, [Text.Encoding]::UTF8, $true)
    $null = $stream.Seek(-22, [IO.SeekOrigin]::End)
    $end = $reader.ReadBytes(22)
    if ([BitConverter]::ToUInt32($end, 0) -ne 0x06054b50 -or
        [BitConverter]::ToUInt16($end, 20) -ne 0 -or
        [BitConverter]::ToUInt16($end, 10) -ne $files.Count -or
        [BitConverter]::ToUInt32($end, 16) -eq [uint32]::MaxValue)
    {
        throw "Source ZIP exceeds the single-volume ZIP32 metadata boundary"
    }
    [long]$position = [BitConverter]::ToUInt32($end, 16)
    foreach ($file in $files)
    {
        $stream.Position = $position
        $header = $reader.ReadBytes(46)
        if ([BitConverter]::ToUInt32($header, 0) -ne 0x02014b50)
        {
            throw "Invalid ZIP central header at $position"
        }
        $stream.Position = $position + 5
        $stream.WriteByte(3)
        $position += 46 + [BitConverter]::ToUInt16($header, 28) +
            [BitConverter]::ToUInt16($header, 30) + [BitConverter]::ToUInt16($header, 32)
    }
    $reader.Dispose()
}
finally
{
    $stream.Dispose()
}
$archive = [IO.Compression.ZipFile]::OpenRead($archivePath)
try
{
    foreach ($entry in $archive.Entries)
    {
        $stream = $entry.Open()
        try
        {
            $hash = [Convert]::ToHexString([Security.Cryptography.SHA256]::HashData($stream))
            $file = $entry.FullName.Substring($name.Length + 1)
            if ($hash -ne (Get-FileHash -LiteralPath (Join-Path $root $sources[$file]) -Algorithm SHA256).Hash)
            {
                throw "Archive differs from selected source: $file"
            }
        }
        finally
        {
            $stream.Dispose()
        }
    }
}
finally
{
    $archive.Dispose()
}
Write-Host "SOURCE_PACKAGE_OK platform=windows files=$($files.Count) archive=$archivePath"
