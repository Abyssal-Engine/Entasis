[CmdletBinding(PositionalBinding = $false)]
param(
    [ValidateSet("All", "Fixtures")]
    [string]$Scope = "All"
)
. (Join-Path $PSScriptRoot "../lib/toolchain_common.ps1")
Assert-ToolchainHost
$nativeNinja = Join-Path (Resolve-Toolchain "ninja") "ninja.exe"
$sourceRoot = $script:ToolchainRoot
$testRoot = Join-Path $sourceRoot ("build/toolchain-tests/windows-" + [Guid]::NewGuid().ToString("N") + "/provisioning with spaces")
$null = New-Item -ItemType Directory -Path (Join-Path $testRoot "scripts/windows/lib"), (Join-Path $testRoot "scripts/windows/toolchains"), (Join-Path $testRoot "tools"), (Join-Path $testRoot "fixture")
Copy-Item -LiteralPath (Join-Path $sourceRoot "scripts/windows/lib/toolchain_common.ps1") -Destination (Join-Path $testRoot "scripts/windows/lib")
Copy-Item -LiteralPath (Join-Path $sourceRoot "scripts/windows/toolchains/setup_toolchains.ps1") -Destination (Join-Path $testRoot "scripts/windows/toolchains")
Copy-Item -LiteralPath $nativeNinja -Destination (Join-Path $testRoot "fixture/ninja.exe")
$archive = Join-Path $testRoot "fixture.zip"
[IO.Compression.ZipFile]::CreateFromDirectory((Join-Path $testRoot "fixture"), $archive)
$entries = Read-ToolchainData (Join-Path $sourceRoot "tools/toolchains.lock")
$entry = $entries.ninja
$entry.release = "fixture"
$asset = @{
    asset = "fixture.zip"
    url = ([Uri]$archive).AbsoluteUri
    sha256 = (Get-FileHash -LiteralPath $archive -Algorithm SHA256).Hash.ToLowerInvariant()
    size = (Get-Item -LiteralPath $archive).Length
}
$entry.hosts["windows-amd64"] = $asset
$lockPath = Join-Path $testRoot "tools/toolchains.lock"
ConvertTo-ToolchainLines $entries | Set-Content -LiteralPath $lockPath
. (Join-Path $testRoot "scripts/windows/lib/toolchain_common.ps1")
$installed = Resolve-Toolchain "ninja"
Move-Item -LiteralPath $archive -Destination "$archive.offline"
if ((Resolve-Toolchain "ninja") -ne $installed -or (Resolve-Toolchain "ninja" $installed) -ne $installed)
{
    throw "Warm/offline or external selection changed the installation"
}
$declaredVersion = $entry.version
$entry.version = "0.0.0"
ConvertTo-ToolchainLines $entries | Set-Content -LiteralPath $lockPath
$wrongHost = Join-Path $testRoot "wrong-host"
$null = New-Item -ItemType Directory -Path $wrongHost
[IO.File]::WriteAllBytes((Join-Path $wrongHost "ninja.exe"), [byte[]]@(127, 69, 76, 70))
foreach ($root in @($installed, $wrongHost))
{
    $status = "UnexpectedSuccess"
    try
    {
        $null = Resolve-Toolchain "ninja" $root
    }
    catch
    {
        $status = "Rejected"
        $_ | Out-String | Add-Content -LiteralPath (Join-Path $testRoot "mismatch.log")
    }
    if ($status -ne "Rejected")
    {
        throw "Incompatible external root unexpectedly passed: $root"
    }
}
$entry.version = $declaredVersion
ConvertTo-ToolchainLines $entries | Set-Content -LiteralPath $lockPath
$status = "UnexpectedSuccess"
try
{
    $null = Resolve-Toolchain "ninja" (Join-Path $testRoot "missing")
}
catch
{
    $status = "Rejected"
    $_ | Out-String | Set-Content -LiteralPath (Join-Path $testRoot "missing.log")
}
if ($status -ne "Rejected")
{
    throw "Missing external root unexpectedly passed"
}
$entry.release = "corrupt"
$asset.url = ([Uri]"$archive.offline").AbsoluteUri
$asset.sha256 = "0" * 64
ConvertTo-ToolchainLines $entries | Set-Content -LiteralPath $lockPath
$status = "UnexpectedSuccess"
try
{
    $null = Resolve-Toolchain "ninja"
}
catch
{
    $status = "Rejected"
    $_ | Out-String | Set-Content -LiteralPath (Join-Path $testRoot "digest.log")
}
if ($status -ne "Rejected")
{
    throw "Corrupt digest unexpectedly passed"
}
$invalid = Join-Path $testRoot "invalid.zip"
$invalidArchive = [IO.Compression.ZipFile]::Open($invalid, [IO.Compression.ZipArchiveMode]::Create)
try
{
    $null = [IO.Compression.ZipFileExtensions]::CreateEntryFromFile($invalidArchive, $nativeNinja, "ninja.exe")
    $entryStream = $invalidArchive.CreateEntry("../rejected").Open()
    $entryStream.Dispose()
}
finally
{
    $invalidArchive.Dispose()
}
$entry.release = "extraction-failure"
$asset.url = ([Uri]$invalid).AbsoluteUri
$asset.sha256 = (Get-FileHash -LiteralPath $invalid -Algorithm SHA256).Hash.ToLowerInvariant()
$asset.size = (Get-Item -LiteralPath $invalid).Length
ConvertTo-ToolchainLines $entries | Set-Content -LiteralPath $lockPath
$status = "UnexpectedSuccess"
try
{
    $null = Resolve-Toolchain "ninja"
}
catch
{
    $status = "Rejected"
    $_ | Out-String | Set-Content -LiteralPath (Join-Path $testRoot "extraction.log")
}
if ($status -ne "Rejected")
{
    throw "Invalid extraction unexpectedly passed"
}
$partialFiles = @(Get-ChildItem -Path (Join-Path $testRoot "build/toolchains/prebuilt/ninja/extraction-failure/.install-*/extracted/ninja.exe"))
if ($partialFiles.Count -ne 1)
{
    throw "Expected the failed extraction to retain its first extracted member"
}
$entry.release = "writer-exclusion"
$asset.url = ([Uri]"$archive.offline").AbsoluteUri
$asset.sha256 = (Get-FileHash -LiteralPath "$archive.offline" -Algorithm SHA256).Hash.ToLowerInvariant()
$asset.size = (Get-Item -LiteralPath "$archive.offline").Length
ConvertTo-ToolchainLines $entries | Set-Content -LiteralPath $lockPath
$writerRoot = Join-Path $testRoot "build/toolchains/prebuilt/ninja/writer-exclusion/windows-amd64"
$writerLock = "$writerRoot.lock"
$null = New-Item -ItemType Directory -Path (Split-Path $writerRoot -Parent)
$lock = [IO.File]::Open($writerLock, [IO.FileMode]::CreateNew, [IO.FileAccess]::Write, [IO.FileShare]::None)
try
{
    $status = "UnexpectedSuccess"
    try
    {
        $null = Resolve-Toolchain "ninja"
    }
    catch
    {
        $_ | Out-String | Set-Content -LiteralPath (Join-Path $testRoot "concurrent.log")
        if ($_.Exception.InnerException -isnot [IO.IOException] -or
            -not $_.Exception.Message.StartsWith('Exception calling "Open"') -or
            -not $_.Exception.Message.Contains($writerLock))
        {
            throw
        }
        $status = "Rejected"
    }
    if ($status -ne "Rejected")
    {
        throw "Concurrent writer unexpectedly passed"
    }
    if ((Test-Path -LiteralPath $writerRoot) -or -not (Test-Path -LiteralPath $writerLock) -or
        -not $lock.CanWrite)
    {
        throw "Blocked installation changed the installation root or held lock"
    }
}
finally
{
    $lock.Dispose()
    Remove-Item -LiteralPath $writerLock
}
if ((Resolve-Toolchain "ninja") -ne $writerRoot -or
    -not (Test-Path -LiteralPath (Join-Path $writerRoot "ninja.exe")) -or
    (Test-Path -LiteralPath $writerLock))
{
    throw "Installation after releasing the writer lock did not complete"
}
$entry.release = "fixture"
$entry.hosts["windows-amd64"] = Read-ToolchainData (Join-Path $installed ".entasis-toolchain")
ConvertTo-ToolchainLines $entries | Set-Content -LiteralPath $lockPath
if ((Resolve-Toolchain "ninja") -ne $installed)
{
    throw "Failed installation changed the working entry"
}
Write-Host "TOOLCHAIN_TESTS_OK cases=cold,warm-offline,external,missing,digest,extraction-preservation,concurrent-writer root=$testRoot"

if ($Scope -eq "Fixtures")
{
    return
}

$officialRoot = Join-Path $sourceRoot "build/toolchain-tests/official-windows"
$null = New-Item -ItemType Directory -Force -Path (Join-Path $officialRoot "scripts/windows/lib"), (Join-Path $officialRoot "scripts/windows/toolchains"), (Join-Path $officialRoot "tools")
Copy-Item -LiteralPath (Join-Path $sourceRoot "scripts/windows/lib/toolchain_common.ps1") -Destination (Join-Path $officialRoot "scripts/windows/lib") -Force
Copy-Item -LiteralPath (Join-Path $sourceRoot "scripts/windows/toolchains/setup_toolchains.ps1") -Destination (Join-Path $officialRoot "scripts/windows/toolchains") -Force
Copy-Item -LiteralPath (Join-Path $sourceRoot "tools/toolchains.lock") -Destination (Join-Path $officialRoot "tools") -Force
& (Join-Path $officialRoot "scripts/windows/toolchains/setup_toolchains.ps1") -Usage Odin
if ($LASTEXITCODE -ne 0)
{
    exit $LASTEXITCODE
}
Write-Host "OFFICIAL_ODIN_SETUP_OK root=$officialRoot"
