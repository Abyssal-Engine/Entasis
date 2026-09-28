[CmdletBinding(PositionalBinding = $false)]
param(
    [ValidateSet("Msvc", "ClangCl", "All")]
    [string]$Compiler = "Msvc",
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
if (-not (Test-Path -LiteralPath (Join-Path $script:ToolchainRoot "lib/cmake/Entasis/EntasisConfig.cmake")))
{
    throw "Run this command from an extracted Entasis SDK"
}
Import-ToolchainVsEnvironment $VsDevCmd
$cmake = Resolve-ToolchainExecutable "cmake" "bin/cmake.exe" $CMakeExe
$ctest = Join-Path (Split-Path $cmake -Parent) "ctest.exe"
$ninja = Resolve-ToolchainExecutable "ninja" "ninja.exe" $NinjaExe
$clang = ""
if ($Compiler -in @("ClangCl", "All"))
{
    $clang = Resolve-ToolchainExecutable "llvm" "bin/clang-cl.exe" $ClangClExe
}
$lanes = if ($Compiler -eq "All")
{
    @("Msvc", "ClangCl")
}
else
{
    @($Compiler)
}
foreach ($lane in $lanes)
{
    $cc = if ($lane -eq "Msvc")
    {
        (Get-Command cl.exe -CommandType Application).Source
    }
    else
    {
        $clang
    }
    $build = Join-Path $script:ToolchainRoot "build/examples/$($lane.ToLowerInvariant())"
    Write-Host "SDK_EXAMPLE_LANE compiler=$lane root=$script:ToolchainRoot"
    & $cmake -S (Join-Path $script:ToolchainRoot "examples") -B $build -G Ninja -DCMAKE_BUILD_TYPE=Release `
        "-DEntasis_DIR=$(Join-Path $script:ToolchainRoot 'lib/cmake/Entasis')" "-DCMAKE_C_COMPILER=$cc" "-DCMAKE_CXX_COMPILER=$cc" "-DCMAKE_MAKE_PROGRAM=$ninja"
    if ($LASTEXITCODE -ne 0)
    {
        exit $LASTEXITCODE
    }
    & $cmake --build $build --parallel 4
    if ($LASTEXITCODE -ne 0)
    {
        exit $LASTEXITCODE
    }
    & $ctest --test-dir $build --output-on-failure --no-tests=error
    if ($LASTEXITCODE -ne 0)
    {
        exit $LASTEXITCODE
    }
}
Write-Host "SDK_EXAMPLES_OK compiler=$Compiler"
