[CmdletBinding(PositionalBinding = $false)]
param(
    [ValidateNotNullOrEmpty()]
    [string]$OdinExe,
    [ValidateNotNullOrEmpty()]
    [string]$VsDevCmd,
    [ValidateNotNullOrEmpty()]
    [string]$ClangClExe,
    [Parameter(Mandatory = $true)]
    [ValidateSet("Development", "Release")]
    [string]$Configuration,
    [ValidateSet("Msvc", "ClangCl", "All")]
    [string]$Compiler = "Msvc",
    [ValidateSet("All", "Queries", "Custom-Shapes", "Custom-Tasks", "Custom-Constraints", "Extensions")]
    [string]$Scope = "All"
)
. (Join-Path $PSScriptRoot "../lib/c_abi_windows_common.ps1")

$odin = Resolve-ToolchainOdin $OdinExe $VsDevCmd
$clangCl = ""
if ($Compiler -in @("ClangCl", "All"))
{
    $clangCl = Resolve-ToolchainExecutable "llvm" "bin/clang-cl.exe" $ClangClExe
}
$buildRoot = Get-CAbiBuildRoot $Configuration
Assert-CAbiBuildArtifacts $buildRoot $Configuration

$msvc = (Get-Command "cl.exe" -CommandType Application -ErrorAction Stop).Source
$lanes = switch ($Compiler)
{
    "Msvc"
    {
        @("msvc")
    }
    "ClangCl"
    {
        @("clang-cl")
    }
    "All"
    {
        @("msvc", "clang-cl")
    }
}
$testRoot = Join-Path $script:Root "tests\c_abi"
$includeRoot = Join-Path $script:Root "include"
$runtimeImport = Join-Path $buildRoot "entasis.lib"
$runtimeStatic = Join-Path $buildRoot "entasis_static.lib"
$families = @("semantic", "scene_semantic", "dynamics_semantic", "queries_semantic", "custom_shapes_semantic", "custom_tasks_semantic", "custom_constraints_semantic", "extensions_semantic")
$consumerArguments = @()
$parityRoot = Join-Path $buildRoot "semantic-parity"
if ($Scope -eq "Queries")
{
    $families = @("queries_semantic")
    $consumerArguments = @("--queries-only")
    $parityRoot = Join-Path $parityRoot "queries"
}

if ($Scope -eq "Extensions")
{
    $families = @("extensions_semantic")
    $parityRoot = Join-Path $parityRoot "extensions"
}
if ($Scope -eq "Custom-Constraints")
{
    $families = @("custom_constraints_semantic")
    $parityRoot = Join-Path $parityRoot "custom-constraints"
}
if ($Scope -eq "Custom-Tasks")
{
    $families = @("custom_tasks_semantic")
    $parityRoot = Join-Path $parityRoot "custom-tasks"
}
if ($Scope -eq "Custom-Shapes")
{
    $families = @("custom_shapes_semantic")
    $parityRoot = Join-Path $parityRoot "custom-shapes"
}

function Assert-EqualFiles([string]$Expected, [string]$Actual, [string]$Owner)
{
    $expectedBytes = [System.IO.File]::ReadAllBytes($Expected)
    $actualBytes = [System.IO.File]::ReadAllBytes($Actual)
    $expectedText = [System.Convert]::ToBase64String($expectedBytes)
    $actualText = [System.Convert]::ToBase64String($actualBytes)
    if (-not $expectedText.Equals($actualText, [System.StringComparison]::Ordinal))
    {
        throw "$Owner output differs from direct Odin: $Actual"
    }
}

function Invoke-SharedCaptured([string]$Binary, [string]$OutputPath)
{
    $previousPath = $env:PATH
    try
    {
        $env:PATH = "$buildRoot;$previousPath"
        Invoke-CapturedToFile $Binary $consumerArguments $OutputPath
    }
    finally
    {
        $env:PATH = $previousPath
    }
}

foreach ($lane in $lanes)
{
    $laneName = $lane
    $compilerExe = if ($laneName -eq "msvc")
    {
        $msvc
    }
    else
    {
        $clangCl
    }
    $laneRoot = Join-Path $parityRoot $laneName
    Remove-Item -LiteralPath $laneRoot -Recurse -Force -ErrorAction SilentlyContinue
    New-Item -ItemType Directory -Path $laneRoot | Out-Null
    $commonCFlags = @("/nologo", "/std:c11", "/O2", "/W4", "/WX", "/I$includeRoot", "/I$testRoot")
    if ($laneName -eq "msvc")
    {
        $commonCFlags += "/experimental:c11atomics"
    }
    Write-Host "C_ABI_SEMANTIC_LANE configuration=$($Configuration.ToLowerInvariant()) compiler=$laneName scope=$($Scope.ToLowerInvariant()) executable=$compilerExe"

    foreach ($family in $families)
    {
        $directSource = Join-Path $testRoot "${family}_direct_odin.odin"
        $cSource = Join-Path $testRoot "${family}_c11.c"
        $directBinary = Join-Path $laneRoot "$family-direct-odin.exe"
        $directOutput = Join-Path $laneRoot "$family-direct.txt"
        $sharedBinary = Join-Path $laneRoot "$family-c-shared.exe"
        $sharedOutput = Join-Path $laneRoot "$family-c-shared.txt"
        $staticBinary = Join-Path $laneRoot "$family-c-static.exe"
        $staticOutput = Join-Path $laneRoot "$family-c-static.txt"
        $directArguments = @(
            "build", $directSource, "-file", "-out:$directBinary",
            "-collection:entasis=$(Join-Path $script:Root 'src')",
            "-target:$script:OdinTarget", "-microarch:$script:OdinMicroarch",
            "-thread-count:$script:OdinThreadCount", "-linker:lld", "-vet", "-warnings-as-errors"
        )
        if ($Configuration -eq "Development")
        {
            $directArguments += @("-debug", "-o:none", "-source-code-locations:normal", "-pdb-name:$([System.IO.Path]::ChangeExtension($directBinary, '.pdb'))")
        }
        else
        {
            $directArguments += @("-o:speed", "-no-bounds-check", "-disable-assert", "-source-code-locations:none")
        }
        Invoke-Checked $odin $directArguments
        Invoke-CapturedToFile $directBinary $consumerArguments $directOutput

        $sharedLibraries = @($runtimeImport)
        $staticLibraries = @($runtimeStatic)
        if ($family -in @("custom_tasks_semantic", "extensions_semantic"))
        {
            $sharedLibraries += (Join-Path $buildRoot "entasis_cooking.lib")
            $staticLibraries += @((Join-Path $buildRoot "entasis_cooking_static.lib"), "/FORCE:MULTIPLE", "/IGNORE:4006")
        }
        Invoke-Checked $compilerExe ($commonCFlags + @($cSource, "/Fo$(Join-Path $laneRoot "$family-c-shared.obj")", "/Fe:$sharedBinary", "/link") + $sharedLibraries)
        Invoke-SharedCaptured $sharedBinary $sharedOutput
        Assert-EqualFiles $directOutput $sharedOutput "$laneName $family shared"

        Invoke-Checked $compilerExe ($commonCFlags + @($cSource, "/Fo$(Join-Path $laneRoot "$family-c-static.obj")", "/Fe:$staticBinary", "/link") + $staticLibraries)
        Invoke-CapturedToFile $staticBinary $consumerArguments $staticOutput
        Assert-EqualFiles $directOutput $staticOutput "$laneName $family static"
        if ($family -eq "queries_semantic")
        {
            $contextShared = Join-Path $laneRoot "$family-context-shared.exe"
            $contextStatic = Join-Path $laneRoot "$family-context-static.exe"
            $contextSharedOutput = Join-Path $laneRoot "$family-context-shared.txt"
            $contextStaticOutput = Join-Path $laneRoot "$family-context-static.txt"
            Invoke-Checked $compilerExe ($commonCFlags + @("/DENTASIS_TEST_QUERY_CONTEXT", $cSource, "/Fo$(Join-Path $laneRoot "$family-context-shared.obj")", "/Fe:$contextShared", "/link", $runtimeImport))
            Invoke-SharedCaptured $contextShared $contextSharedOutput
            Assert-EqualFiles $directOutput $contextSharedOutput "$laneName $family context shared"
            Invoke-Checked $compilerExe ($commonCFlags + @("/DENTASIS_TEST_QUERY_CONTEXT", $cSource, "/Fo$(Join-Path $laneRoot "$family-context-static.obj")", "/Fe:$contextStatic", "/link", $runtimeStatic))
            Invoke-CapturedToFile $contextStatic $consumerArguments $contextStaticOutput
            Assert-EqualFiles $directOutput $contextStaticOutput "$laneName $family context static"
        }
        Write-Host "C_ABI_SEMANTIC_PASS compiler=$laneName family=$family"
    }
}
Write-Host "C_ABI_SEMANTICS_OK configuration=$($Configuration.ToLowerInvariant()) compiler=$($Compiler.ToLowerInvariant()) scope=$($Scope.ToLowerInvariant())"
