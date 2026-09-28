[CmdletBinding(PositionalBinding = $false)]
param(
    [ValidateSet("Msvc", "ClangCl", "All")]
    [string]$Compiler = "Msvc",
    [ValidateNotNullOrEmpty()]
    [string]$Archive,
    [ValidateNotNullOrEmpty()]
    [string]$VsDevCmd,
    [ValidateNotNullOrEmpty()]
    [string]$ClangClExe,
    [ValidateNotNullOrEmpty()]
    [string]$CMakeExe,
    [ValidateNotNullOrEmpty()]
    [string]$NinjaExe
)
. (Join-Path $PSScriptRoot "../lib/toolchain_common.ps1")
if ($Archive.Length -eq 0)
{
    $manifest = Get-Content -LiteralPath (Join-Path $script:ToolchainRoot "tools/abi/abi_manifest.json") -Raw | ConvertFrom-Json
    $Archive = Join-Path $script:ToolchainRoot "build/sdk/windows/Entasis-$($manifest.product.version_string)-windows-x86_64-v3-c-sdk.zip"
}
if (-not [IO.Path]::IsPathFullyQualified($Archive) -or -not (Test-Path -LiteralPath $Archive -PathType Leaf))
{
    [Console]::Error.WriteLine("Archive must name an existing absolute path: $Archive")
    exit 2
}
Import-ToolchainVsEnvironment $VsDevCmd
$cmake = Resolve-ToolchainExecutable "cmake" "bin/cmake.exe" $CMakeExe
$ninja = Resolve-ToolchainExecutable "ninja" "ninja.exe" $NinjaExe
$roots = @{
    cmake = Split-Path (Split-Path $cmake -Parent) -Parent
    ninja = Split-Path $ninja -Parent
}
if ($Compiler -in @("ClangCl", "All"))
{
    $clang = Resolve-ToolchainExecutable "llvm" "bin/clang-cl.exe" $ClangClExe
    $roots.llvm = Split-Path (Split-Path $clang -Parent) -Parent
}
$extraction = Join-Path $script:ToolchainRoot ("build/sdk-tests/windows/sdk " + [IO.Path]::GetRandomFileName())
$null = New-Item -ItemType Directory -Path $extraction
[IO.Compression.ZipFile]::ExtractToDirectory($Archive, $extraction)
$children = @(Get-ChildItem -LiteralPath $extraction)
if ($children.Count -ne 1 -or -not $children[0].PSIsContainer)
{
    throw "Expected one SDK archive root"
}
$sdk = $children[0].FullName
$selection = @{ roots = $roots }
if ($VsDevCmd.Length -gt 0)
{
    $selection.system = @{ vsdevcmd = $VsDevCmd }
}
else
{
    $local = Read-ToolchainLocal
    if ($local.ContainsKey("system") -and $local.system.ContainsKey("vsdevcmd"))
    {
        $selection.system = @{ vsdevcmd = $local.system.vsdevcmd }
    }
}
ConvertTo-ToolchainLines @{ "windows-amd64" = $selection } | Set-Content -LiteralPath (Join-Path $sdk "toolchains.local")
# exercise actual find_package selection without compiling another consumer
$versionTests = Join-Path $extraction "version-selection"
$null = New-Item -ItemType Directory -Path $versionTests
$versionProject = @'
cmake_minimum_required(VERSION 3.19)
project(EntasisVersionSelection NONE)
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/package")
set(ENTASIS_VERSION "${PROBE_PACKAGE_VERSION}")
configure_file("${PROBE_TEMPLATE}" "${CMAKE_BINARY_DIR}/package/EntasisConfigVersion.cmake" @ONLY)
file(WRITE "${CMAKE_BINARY_DIR}/package/EntasisConfig.cmake" "set(ENTASIS_SELECTION_PROBE 1)\n")
separate_arguments(request NATIVE_COMMAND "${PROBE_REQUEST}")
find_package(Entasis ${request} QUIET CONFIG PATHS "${CMAKE_BINARY_DIR}/package" NO_DEFAULT_PATH)
if(PROBE_EXPECT EQUAL 1)
    if(NOT Entasis_FOUND OR NOT ENTASIS_SELECTION_PROBE)
        message(FATAL_ERROR "Expected package selection: ${PROBE_PACKAGE_VERSION} / ${PROBE_REQUEST}")
    endif()
elseif(Entasis_FOUND)
    message(FATAL_ERROR "Unexpected package selection: ${PROBE_PACKAGE_VERSION} / ${PROBE_REQUEST}")
endif()
'@
[IO.File]::WriteAllText((Join-Path $versionTests "CMakeLists.txt"), $versionProject)
$versionCases = @(
    @("exact", "1.2.3", "1.2.3 EXACT", 1),
    @("newer", "1.2.3", "1.2.4", 0),
    @("older-major", "1.2.3", "0.9", 0),
    @("older-same", "1.2.3", "1.1", 1),
    @("exact-mismatch", "1.2.3", "1.1 EXACT", 0),
    @("range-included", "1.2.3", "1.0...1.2.3", 1),
    @("range-excluded", "1.2.3", "1.0...<1.2.3", 0),
    @("next-major-excluded", "1.2.3", "1.0...<2", 1),
    @("cross-major", "1.2.3", "1.0...2", 0),
    @("prerelease-stable", "1.2.3-rc.1", "1.2.3", 0),
    @("prerelease-unversioned", "1.2.3-rc.1", "", 1)
)
foreach ($case in $versionCases)
{
    & $cmake -S $versionTests -B (Join-Path $versionTests $case[0]) `
        "-DPROBE_TEMPLATE=$(Join-Path $script:ToolchainRoot 'sdk/cmake/EntasisConfigVersion.cmake.in')" `
        "-DPROBE_PACKAGE_VERSION=$($case[1])" "-DPROBE_REQUEST=$($case[2])" "-DPROBE_EXPECT=$($case[3])"
    if ($LASTEXITCODE -ne 0)
    {
        exit $LASTEXITCODE
    }
    Write-Host "SDK_VERSION_SELECTION_OK case=$($case[0])"
}
Push-Location $extraction
try
{
    & (Join-Path $sdk "scripts/windows/sdk/run_sdk_examples.ps1") -Compiler $Compiler
    if ($LASTEXITCODE -ne 0)
    {
        exit $LASTEXITCODE
    }
    $dumpbin = (Get-Command dumpbin.exe -CommandType Application -ErrorAction Stop).Source
    $lanes = if ($Compiler -eq "All")
    {
        @("msvc", "clangcl")
    }
    else
    {
        @($Compiler.ToLowerInvariant())
    }
    foreach ($lane in $lanes)
    {
        foreach ($name in @("entasis_c_static", "entasis_cpp_static"))
        {
            $binary = Join-Path $sdk "build/examples/$lane/$name.exe"
            $imports = @(& $dumpbin /nologo /imports $binary)
            if ($LASTEXITCODE -ne 0)
            {
                exit $LASTEXITCODE
            }
            $imports | Set-Content -LiteralPath "$binary.imports.txt"
            if ($imports -match '(?i)entasis(?:_cooking)?\.dll')
            {
                throw "Static consumer imports an Entasis DLL: $binary"
            }
        }
    }
}
finally
{
    Pop-Location
}
Write-Host "SDK_CONSUMERS_OK compiler=$Compiler extracted=$sdk"
