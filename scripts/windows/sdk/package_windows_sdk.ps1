param()
$outputRoot = "build/sdk/windows"
if ($args.Count -gt 0)
{
    if ($args.Count -ne 2 -or $args[0] -ine "-OutputRoot" -or
        [string]::IsNullOrWhiteSpace([string]$args[1]) -or ([string]$args[1]).StartsWith("-"))
    {
        [Console]::Error.WriteLine("usage: package_windows_sdk.ps1 [-OutputRoot <path>]")
        exit 2
    }
    $outputRoot = [string]$args[1]
}
. (Join-Path $PSScriptRoot "../lib/c_abi_windows_common.ps1")
$outputRoot = [IO.Path]::GetFullPath($outputRoot, $script:Root)
$manifest = Get-Content -LiteralPath (Join-Path $script:Root "tools/abi/abi_manifest.json") -Raw | ConvertFrom-Json
$version = [string]$manifest.product.version_string
if ($version -notmatch '^[0-9]+\.[0-9]+\.[0-9]+(?:-[A-Za-z0-9.-]+)?$')
{
    throw "Invalid product.version_string: $version"
}
$buildRoot = Get-CAbiBuildRoot "Release"
Assert-CAbiBuildArtifacts $buildRoot "Release"
$installName = "Entasis-$version-windows-x86_64-v3-c-sdk"
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
Copy-Item -LiteralPath (Join-Path $script:Root "sdk/cmake/EntasisConfig.cmake") -Destination (Join-Path $installRoot "lib/cmake/Entasis")
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
