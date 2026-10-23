param()
$outputRoot = "build/sdk/windows"
$precision = "Float32"
$seen = @{}
for ($index = 0; $index -lt $args.Count; $index += 2)
{
    $option = [string]$args[$index]
    if ($option -notin @("-OutputRoot", "-Precision") -or $seen.ContainsKey($option) -or
        $index + 1 -ge $args.Count -or [string]::IsNullOrWhiteSpace([string]$args[$index + 1]) -or
        ([string]$args[$index + 1]).StartsWith("-"))
    {
        [Console]::Error.WriteLine("Invalid or repeated option: $option")
        exit 2
    }
    $seen[$option] = $true
    if ($option -eq "-OutputRoot")
    {
        $outputRoot = [string]$args[$index + 1]
    }
    else
    {
        $precision = [string]$args[$index + 1]
    }
}
if ($precision -notin @("Float32", "Mixed64"))
{
    [Console]::Error.WriteLine("Precision must be Float32 or Mixed64")
    exit 2
}
$highDefine = 0
$suffix = ""
if ($precision -eq "Mixed64")
{
    $highDefine = 1
    $suffix = "-mixed64"
    $outputRoot = Join-Path $outputRoot "mixed64"
}
. (Join-Path $PSScriptRoot "../lib/c_abi_windows_common.ps1")
$outputRoot = [IO.Path]::GetFullPath($outputRoot, $script:Root)
$manifest = Get-Content -LiteralPath (Join-Path $script:Root "tools/abi/abi_manifest.json") -Raw | ConvertFrom-Json
$version = [string]$manifest.product.version_string
if ($version -notmatch '^[0-9]+\.[0-9]+\.[0-9]+(?:-[A-Za-z0-9.-]+)?$')
{
    throw "Invalid product.version_string: $version"
}
$buildRoot = Get-CAbiBuildRoot "Release" $precision
Assert-CAbiBuildArtifacts $buildRoot "Release"
$installName = "Entasis-$version-windows-x86_64-v3-c-sdk$suffix"
$installRoot = Join-Path $outputRoot $installName
$archivePath = Join-Path $outputRoot "$installName.zip"
foreach ($path in @($installRoot, $archivePath))
{
    if (Test-Path -LiteralPath $path)
    {
        throw "SDK output collision: $path"
    }
}
$required = @("include/entasis.h", "include/entasis_cooking.h", "include/entasis", "sdk/examples", "sdk/README.md", "sdk/cmake/EntasisConfig.cmake", "sdk/cmake/EntasisConfigVersion.cmake.in", "sdk/BUILDING.md", "docs/c", "LICENSE", "NOTICE", "tools/toolchains.lock")
$bootstrap = @("scripts/windows/toolchains/setup_toolchains.ps1", "scripts/windows/lib/toolchain_common.ps1", "scripts/windows/sdk/run_sdk_examples.ps1")
$required += $bootstrap
foreach ($source in $required)
{
    if (-not (Test-Path -LiteralPath (Join-Path $script:Root $source)))
    {
        throw "Missing package source: $source"
    }
}
foreach ($directory in @("include/entasis", "bin", "lib/cmake/Entasis", "examples", "scripts/windows", "tools", "share/doc/Entasis/c", "share/licenses/Entasis"))
{
    $null = New-Item -ItemType Directory -Force -Path (Join-Path $installRoot $directory)
}
Copy-Item -LiteralPath (Join-Path $script:Root "include/entasis.h"), (Join-Path $script:Root "include/entasis_cooking.h") -Destination (Join-Path $installRoot "include")
Copy-Item -Path (Join-Path $script:Root "include/entasis/*.h") -Destination (Join-Path $installRoot "include/entasis")
foreach ($name in @("entasis", "entasis_cooking"))
{
    Copy-Item -LiteralPath (Join-Path $buildRoot "$name.dll") -Destination (Join-Path $installRoot "bin")
    Copy-Item -LiteralPath (Join-Path $buildRoot "$name.lib"), (Join-Path $buildRoot "${name}_static.lib") -Destination (Join-Path $installRoot "lib")
}
Copy-Item -Path (Join-Path $script:Root "sdk/examples/*") -Destination (Join-Path $installRoot "examples")
Copy-Item -LiteralPath (Join-Path $script:Root "sdk/README.md") -Destination (Join-Path $installRoot "README.md")
$config = Get-Content -LiteralPath (Join-Path $script:Root "sdk/cmake/EntasisConfig.cmake") -Raw
[IO.File]::WriteAllText((Join-Path $installRoot "lib/cmake/Entasis/EntasisConfig.cmake"),
    $config.Replace("@ENTASIS_PRECISION@", $precision.ToLowerInvariant()).Replace("@ENTASIS_PRECISION_MIXED64@", [string]$highDefine))
$template = Get-Content -LiteralPath (Join-Path $script:Root "sdk/cmake/EntasisConfigVersion.cmake.in") -Raw
[IO.File]::WriteAllText((Join-Path $installRoot "lib/cmake/Entasis/EntasisConfigVersion.cmake"), $template.Replace("@ENTASIS_VERSION@", $version))
Copy-Item -LiteralPath (Join-Path $script:Root "sdk/BUILDING.md") -Destination (Join-Path $installRoot "share/doc/Entasis/BUILDING.md")
Copy-Item -Path (Join-Path $script:Root "docs/c/*") -Destination (Join-Path $installRoot "share/doc/Entasis/c") -Recurse
Copy-Item -LiteralPath (Join-Path $script:Root "LICENSE"), (Join-Path $script:Root "NOTICE") -Destination (Join-Path $installRoot "share/licenses/Entasis")
foreach ($source in $bootstrap)
{
    $destination = Join-Path $installRoot $source
    $null = New-Item -ItemType Directory -Force -Path (Split-Path $destination -Parent)
    Copy-Item -LiteralPath (Join-Path $script:Root $source) -Destination $destination
}
$entries = Read-ToolchainData (Join-Path $script:Root "tools/toolchains.lock")
$sdkEntries = [ordered]@{}
foreach ($name in @("llvm", "cmake", "ninja"))
{
    $entry = $entries[$name]
    $entry.hosts = @{ "windows-amd64" = $entry.hosts["windows-amd64"] }
    $sdkEntries[$name] = $entry
}
ConvertTo-ToolchainLines $sdkEntries | Set-Content -LiteralPath (Join-Path $installRoot "tools/toolchains.lock")
[IO.Compression.ZipFile]::CreateFromDirectory($installRoot, $archivePath, [IO.Compression.CompressionLevel]::Optimal, $true)
$archive = [IO.Compression.ZipFile]::OpenRead($archivePath)
try
{
    foreach ($entry in $archive.Entries)
    {
        if ($entry.Name.Length -eq 0)
        {
            continue
        }
        $stream = $entry.Open()
        try
        {
            $actual = [Convert]::ToHexString([Security.Cryptography.SHA256]::HashData($stream))
            $source = Join-Path $outputRoot $entry.FullName
            if ($actual -ne (Get-FileHash -LiteralPath $source -Algorithm SHA256).Hash)
            {
                throw "Archive content differs from install tree: $($entry.FullName)"
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
Write-Host "WINDOWS_SDK_OK tree=$installRoot archive=$archivePath"
