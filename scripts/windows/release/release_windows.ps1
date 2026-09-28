param()
$outputRoot = "build/releases"
if ($args.Count -gt 0)
{
    if ($args.Count -ne 2 -or $args[0] -ine "-OutputRoot" -or
        [string]::IsNullOrWhiteSpace([string]$args[1]) -or ([string]$args[1]).StartsWith("-"))
    {
        [Console]::Error.WriteLine("usage: scripts/windows/release/release_windows.ps1 [-OutputRoot <path>]")
        exit 2
    }
    $outputRoot = [string]$args[1]
}
$ErrorActionPreference = "Stop"
$releaseRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot "../../.."))
$outputRoot = [IO.Path]::GetFullPath($outputRoot, $releaseRoot)
$manifest = Get-Content -LiteralPath (Join-Path $releaseRoot "tools/abi/abi_manifest.json") -Raw | ConvertFrom-Json
$version = [string]$manifest.product.version_string
if ($version -notmatch '^[0-9]+\.[0-9]+\.[0-9]+(?:-[A-Za-z0-9.-]+)?$')
{
    throw "Invalid product version: $version"
}
$attempt = [DateTime]::UtcNow.ToString("yyyyMMddTHHmmssZ") + "." + [Guid]::NewGuid().ToString("N")
$work = Join-Path $releaseRoot "build/release-work/windows/$attempt"
$null = New-Item -ItemType Directory -Path $work
$stage = "setup"
$script:releaseExitCode = 1
$null = Start-Transcript -LiteralPath (Join-Path $work "release.log")

function Invoke-ReleaseStage([string]$Owner, [string[]]$Arguments)
{
    # a separate native shell contains child scripts' exit statements
    & (Join-Path $PSHOME "pwsh.exe") -NoProfile -File $Owner @Arguments
    [int]$code = $LASTEXITCODE
    if ($code -ne 0)
    {
        $script:releaseExitCode = $code
        throw "Command failed ($code): $Owner"
    }
}

try
{
    Write-Host "[release] platform=windows version=$version work=$work"
    . (Join-Path $releaseRoot "scripts/windows/lib/toolchain_common.ps1")
    $odin = Resolve-ToolchainOdin
    $stage = "build"
    Invoke-ReleaseStage (Join-Path $releaseRoot "build_c_abi_windows.ps1") @("-Configuration", "Release")
    $stage = "sdk-package"
    Invoke-ReleaseStage (Join-Path $releaseRoot "scripts/windows/sdk/package_windows_sdk.ps1") @("-OutputRoot", (Join-Path $work "sdk"))
    $stage = "source-package"
    Invoke-ReleaseStage (Join-Path $releaseRoot "scripts/windows/release/package_source.ps1") @("-OutputRoot", (Join-Path $work "source"))
    $sdk = Join-Path $work "sdk/Entasis-$version-windows-x86_64-v3-c-sdk.zip"
    $sourceArchive = Join-Path $work "source/Entasis-$version-odin-source.zip"
    $stage = "sdk-consumers"
    Invoke-ReleaseStage (Join-Path $releaseRoot "scripts/windows/sdk/test_sdk_consumers.ps1") @("-Compiler", "All", "-Archive", $sdk)
    $stage = "source-example"
    $extraction = Join-Path $work "source with spaces"
    [IO.Compression.ZipFile]::ExtractToDirectory($sourceArchive, $extraction)
    $sourceRoot = Join-Path $extraction "Entasis-$version-odin-source"
    $local = Read-ToolchainLocal
    $selection = @{
        roots = @{ odin = Split-Path $odin -Parent }
        cache_root = if ($local.ContainsKey("cache_root"))
        {
            $local.cache_root
        }
        else
        {
            Join-Path $releaseRoot "build/toolchains/prebuilt"
        }
    }
    if ($local.ContainsKey("system"))
    {
        $selection.system = $local.system
    }
    ConvertTo-ToolchainLines @{ "windows-amd64" = $selection } | Set-Content -LiteralPath (Join-Path $sourceRoot "toolchains.local")
    Push-Location $extraction
    try
    {
        foreach ($example in @("falling_box", "convex_hulls"))
        {
            Invoke-ReleaseStage (Join-Path $sourceRoot "scripts/windows/examples/run_examples.ps1") @("-Package", $example, "-Configuration", "Release")
        }
    }
    finally
    {
        Pop-Location
    }
    $stage = "complete"
    $parent = Join-Path $outputRoot "$version/windows"
    $pending = Join-Path $parent ".$attempt"
    $completed = Join-Path $parent $attempt
    $null = New-Item -ItemType Directory -Path $pending
    foreach ($asset in @($sdk, $sourceArchive))
    {
        $destination = Join-Path $pending ([IO.Path]::GetFileName($asset))
        [IO.File]::Copy($asset, $destination)
        if ((Get-FileHash -LiteralPath $asset -Algorithm SHA256).Hash -ne (Get-FileHash -LiteralPath $destination -Algorithm SHA256).Hash)
        {
            throw "Release copy differs: $destination"
        }
    }
    [IO.Directory]::Move($pending, $completed)
    Write-Host "RELEASE_OK platform=windows version=$version directory=$completed"
    foreach ($asset in Get-ChildItem -LiteralPath $completed -File)
    {
        Write-Host "asset=$($asset.FullName)"
    }
}
catch
{
    [Console]::Error.WriteLine($_.ToString())
    [Console]::Error.WriteLine("RELEASE_FAILED platform=windows stage=$stage exit=$script:releaseExitCode work=$work")
    exit $script:releaseExitCode
}
finally
{
    $null = Stop-Transcript
}
