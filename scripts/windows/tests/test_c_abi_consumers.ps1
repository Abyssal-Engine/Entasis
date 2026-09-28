[CmdletBinding(PositionalBinding = $false)]
param(
    [ValidateNotNullOrEmpty()]
    [string]$VsDevCmd,
    [ValidateNotNullOrEmpty()]
    [string]$ClangClExe,
    [Parameter(Mandatory = $true)]
    [ValidateSet("Development", "Release")]
    [string]$Configuration,
    [ValidateSet("Msvc", "ClangCl", "All")]
    [string]$Compiler = "Msvc",
    [ValidateSet("Smoke", "All", "Lifecycle", "Queries", "Callbacks", "Stages", "Memory", "Custom-Shapes", "Custom-Tasks", "Custom-Constraints", "Extensions", "Allocators", "Triggers", "Body-Control", "Restitution", "Compatibility")]
    [string]$Scope = "All",
    [ValidateNotNullOrEmpty()]
    [string]$BaselineRoot,
    [ValidateSet("Original", "Cumulative")]
    [string]$CompatibilityFixtures = "Original"
)
. (Join-Path $PSScriptRoot "../lib/c_abi_windows_common.ps1")

$buildRoot = Get-CAbiBuildRoot $Configuration
if ($Scope -eq "Compatibility")
{
    if ([string]::IsNullOrWhiteSpace($BaselineRoot) -or [IO.Path]::IsPathRooted($BaselineRoot))
    {
        [Console]::Error.WriteLine("Compatibility requires a repository-relative -BaselineRoot")
        exit 2
    }
    $BaselineRoot = [IO.Path]::GetFullPath((Join-Path $script:Root $BaselineRoot))
    $separator = [IO.Path]::DirectorySeparatorChar
    if (-not $BaselineRoot.StartsWith($script:Root + $separator, [StringComparison]::OrdinalIgnoreCase) -or
        -not (Test-Path -LiteralPath $BaselineRoot -PathType Container) -or
        $BaselineRoot -eq $buildRoot -or
        $BaselineRoot.StartsWith($buildRoot + $separator, [StringComparison]::OrdinalIgnoreCase) -or
        $buildRoot.StartsWith($BaselineRoot + $separator, [StringComparison]::OrdinalIgnoreCase))
    {
        [Console]::Error.WriteLine("Baseline must be inside the checkout and must not overlap candidate outputs")
        exit 2
    }
    for ($item = Get-Item -LiteralPath $BaselineRoot; $item.FullName -ne $script:Root; $item = Get-Item -LiteralPath (Split-Path $item.FullName -Parent))
    {
        if (($item.Attributes -band [IO.FileAttributes]::ReparsePoint) -ne 0)
        {
            [Console]::Error.WriteLine("Baseline path must not traverse a reparse point")
            exit 2
        }
    }
}
elseif ($PSBoundParameters.ContainsKey("BaselineRoot"))
{
    [Console]::Error.WriteLine("-BaselineRoot is only valid for Compatibility")
    exit 2
}
if ($Scope -ne "Compatibility" -and $PSBoundParameters.ContainsKey("CompatibilityFixtures"))
{
    [Console]::Error.WriteLine("-CompatibilityFixtures is only valid for Compatibility")
    exit 2
}
Assert-CAbiBuildArtifacts $buildRoot $Configuration

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
$includeRoot = Join-Path $script:Root "include"
$testRoot = Join-Path $script:Root "tests\c_abi"
$runtimeImport = Join-Path $buildRoot "entasis.lib"
$runtimeStatic = Join-Path $buildRoot "entasis_static.lib"
$cookingImport = Join-Path $buildRoot "entasis_cooking.lib"
$cookingStatic = Join-Path $buildRoot "entasis_cooking_static.lib"
$publicHeaders = @(
    "entasis/base.h", "entasis/collision.h", "entasis/world.h", "entasis/shapes.h",
    "entasis/bodies.h", "entasis/constraints.h", "entasis/queries.h", "entasis/events.h",
    "entasis/views.h", "entasis/properties.h", "entasis/entasis.h", "entasis/cooking.h",
    "entasis.h", "entasis_cooking.h"
)
$runtimeCTests = @(
    "smoke_c11", "lifecycle_c11", "dispatcher_c11", "policies_c11",
    "scene_lifecycle_c11", "layer_material_policy_c11", "shape_body_static_c11",
    "constraints_c11", "dynamics_c11",
    "properties_c11", "queries_events_views_c11", "extensions_c11", "custom_shapes_c11", "custom_tasks_c11", "custom_constraints_c11", "triggers_c11"
)
$runtimeCppTests = @("smoke_cpp20", "constraints_dynamics_cpp20", "queries_properties_cpp20", "extensions_cpp20", "custom_shapes_cpp20", "custom_tasks_cpp20", "custom_constraints_cpp20", "triggers_cpp20")
$cookingTests = @("smoke_cooking_c11", "cooking_assets_c11", "compound_tasks_c11", "allocators_c11", "extensions_semantic_c11", "triggers_tasks_c11")
if ($Scope -eq "Smoke")
{
    $runtimeCTests = @("smoke_c11")
    $runtimeCppTests = @("smoke_cpp20")
    $cookingTests = @("smoke_cooking_c11")
}

switch ($Scope)
{
    "Triggers"
    {
        $runtimeCTests = @("triggers_c11")
        $runtimeCppTests = @("triggers_cpp20")
        $cookingTests = @("triggers_tasks_c11")
    }
    "Allocators"
    {
        $runtimeCTests = @()
        $runtimeCppTests = @()
        $cookingTests = @("allocators_c11")
    }
    "Extensions"
    {
        $runtimeCTests = @("extensions_c11")
        $runtimeCppTests = @("extensions_cpp20")
        $cookingTests = @("extensions_semantic_c11")
    }
    "Lifecycle"
    {
        $runtimeCTests = @("lifecycle_c11", "dispatcher_c11", "scene_lifecycle_c11")
        $runtimeCppTests = @()
        $cookingTests = @("smoke_cooking_c11", "cooking_assets_c11")
    }
    "Queries"
    {
        $runtimeCTests = @("queries_events_views_c11")
        $runtimeCppTests = @("queries_properties_cpp20")
        $cookingTests = @()
    }
    "Callbacks"
    {
        $runtimeCTests = @("policies_c11", "layer_material_policy_c11", "extensions_c11")
        $runtimeCppTests = @("extensions_cpp20")
        $cookingTests = @()
    }
    "Stages"
    {
        $runtimeCTests = @("extensions_c11")
        $runtimeCppTests = @()
        $cookingTests = @()
    }
    "Body-Control"
    {
        $runtimeCTests = @("extensions_c11")
        $runtimeCppTests = @("extensions_cpp20")
        $cookingTests = @()
    }
    "Restitution"
    {
        $runtimeCTests = @()
        $runtimeCppTests = @("extensions_cpp20")
        $cookingTests = @()
    }
    "Custom-Shapes"
    {
        $runtimeCTests = @("custom_shapes_c11")
        $runtimeCppTests = @("custom_shapes_cpp20")
        $cookingTests = @()
    }
    "Custom-Constraints"
    {
        $runtimeCTests = @("custom_constraints_c11")
        $runtimeCppTests = @("custom_constraints_cpp20")
        $cookingTests = @()
    }
    "Custom-Tasks"
    {
        $runtimeCTests = @("custom_tasks_c11")
        $runtimeCppTests = @("custom_tasks_cpp20")
        $cookingTests = @("compound_tasks_c11")
    }
    "Memory"
    {
        $runtimeCTests = @("lifecycle_c11")
        $runtimeCppTests = @()
        $cookingTests = @("allocators_c11")
    }
}

if ($Scope -eq "Compatibility")
{
    # reject candidate-side junctions too, before deleting any old output tree
    foreach ($lane in $lanes)
    {
        for ($path = Join-Path $buildRoot "consumers/compatibility/$lane"; $path -ne $script:Root; $path = Split-Path $path -Parent)
        {
            if ((Test-Path -LiteralPath $path) -and ((Get-Item -LiteralPath $path).Attributes -band [IO.FileAttributes]::ReparsePoint) -ne 0)
            {
                throw "Compatibility output path must not traverse a reparse point"
            }
        }
    }
    $includeRoot = Join-Path $BaselineRoot "include"
    $testRoot = Join-Path $BaselineRoot "tests/c_abi"
    $baselineBuild = Join-Path $BaselineRoot "build/c_abi/windows/$($Configuration.ToLowerInvariant())"
    [string]$baselineConsumerRoot = Join-Path $baselineBuild "consumers"
    if ($CompatibilityFixtures -eq "Cumulative")
    {
        $baselineConsumerRoot = Join-Path $baselineConsumerRoot "all"
    }
    else
    {
        $runtimeCTests = @(
            "smoke_c11", "lifecycle_c11", "dispatcher_c11", "policies_c11", "scene_lifecycle_c11",
            "layer_material_policy_c11", "shape_body_static_c11", "constraints_c11", "dynamics_c11",
            "properties_c11", "queries_events_views_c11"
        )
        $runtimeCppTests = @("smoke_cpp20", "constraints_dynamics_cpp20", "queries_properties_cpp20")
        $cookingTests = @("smoke_cooking_c11", "cooking_assets_c11")
    }
    foreach ($file in @("entasis.dll", "entasis.lib", "entasis_cooking.dll", "entasis_cooking.lib"))
    {
        if (-not (Test-Path -LiteralPath (Join-Path $baselineBuild $file) -PathType Leaf))
        {
            throw "Missing retained library: $file"
        }
    }
    foreach ($file in @((Join-Path $includeRoot "entasis.h"), (Join-Path $includeRoot "entasis_cooking.h"), (Join-Path $testRoot "test_support.h")))
    {
        if (-not (Test-Path -LiteralPath $file -PathType Leaf))
        {
            throw "Missing retained input: $file"
        }
    }
    foreach ($testName in ($runtimeCTests + $runtimeCppTests + $cookingTests))
    {
        $extension = if ($testName.EndsWith("_cpp20"))
        {
            "cpp"
        }
        else
        {
            "c"
        }
        $source = Join-Path $testRoot "$testName.$extension"
        if (-not (Test-Path -LiteralPath $source -PathType Leaf))
        {
            throw "Missing retained fixture: $source"
        }
        foreach ($lane in $lanes)
        {
            $retained = Join-Path $baselineConsumerRoot "$lane/$testName-shared.exe"
            if (-not (Test-Path -LiteralPath $retained -PathType Leaf))
            {
                throw "Missing retained executable: $retained"
            }
        }
    }
    if (-not (Test-Path -LiteralPath (Join-Path $script:Root "tests/c_abi/runtime_cooking_compatibility_c11.c") -PathType Leaf))
    {
        throw "Missing runtime/cooking compatibility fixture"
    }
}
# all argument and retained-input checks precede provisioning and output mutation
$clangCl = ""
if ($Compiler -in @("ClangCl", "All"))
{
    $clangCl = Resolve-ToolchainExecutable "llvm" "bin/clang-cl.exe" $ClangClExe
}
Import-ToolchainVsEnvironment $VsDevCmd
$msvc = (Get-Command "cl.exe" -CommandType Application -ErrorAction Stop).Source

function Copy-RetainedConsumer([string]$TestName, [string]$Destination)
{
    $source = Join-Path $baselineConsumerRoot "$lane/$TestName-shared.exe"
    Copy-Item -LiteralPath $source -Destination $Destination
}

$compatibilityFailures = [System.Collections.Generic.List[string]]::new()
$script:compatibilityExitCode = 0
function Invoke-Consumer([string]$Binary, [string[]]$Arguments, [string]$Mode = "Shared")
{
    $previousPath = $env:PATH
    try
    {
        if ($Mode -eq "Shared")
        {
            $env:PATH = "$buildRoot;$previousPath"
        }
        $global:LASTEXITCODE = 0
        & $Binary @Arguments
        $exitCode = $LASTEXITCODE
        if ($Scope -ne "Compatibility")
        {
            if ($exitCode -ne 0)
            {
                [Console]::Error.WriteLine("Command failed ($exitCode): $Binary $($Arguments -join ' ')")
                exit $exitCode
            }
            return
        }
        $resultMode = if ($Mode -eq "Shared")
        {
            "retained-shared"
        }
        else
        {
            "original-header-static"
        }
        $fixture = "test=$testName lane=$lane mode=$resultMode"
        if ($exitCode -ne 0)
        {
            $compatibilityFailures.Add($fixture)
            if ($script:compatibilityExitCode -eq 0)
            {
                $script:compatibilityExitCode = $exitCode
            }
            [Console]::Error.WriteLine("C_ABI_COMPATIBILITY_FAILURE $fixture exit=$exitCode")
        }
        else
        {
            Write-Host "C_ABI_COMPATIBILITY $fixture"
        }
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
    $laneRoot = Join-Path $buildRoot "consumers\$($Scope.ToLowerInvariant())\$laneName"
    Remove-Item -LiteralPath $laneRoot -Recurse -Force -ErrorAction SilentlyContinue
    New-Item -ItemType Directory -Path $laneRoot | Out-Null
    if ($Scope -eq "Compatibility")
    {
        # Windows searches the executable directory before PATH. put only the
        # matching candidate pair beside copied old executables, never beside
        # the original reference executables
        foreach ($library in @("entasis.dll", "entasis_cooking.dll"))
        {
            $sourceLibrary = Join-Path $buildRoot $library
            $copyLibrary = Join-Path $laneRoot $library
            Copy-Item -LiteralPath $sourceLibrary -Destination $copyLibrary
        }
    }
    $commonCFlags = @("/nologo", "/std:c11", "/O2", "/W4", "/WX", "/I$includeRoot", "/I$testRoot")
    $commonCppFlags = @("/nologo", "/std:c++20", "/O2", "/W4", "/WX", "/EHsc", "/I$includeRoot", "/I$testRoot")
    if ($laneName -eq "msvc")
    {
        $commonCFlags += "/experimental:c11atomics"
    }
    Write-Host "C_ABI_CONSUMER_LANE configuration=$($Configuration.ToLowerInvariant()) compiler=$laneName executable=$compilerExe"

    if ($Scope -eq "All")
    {
        foreach ($header in $publicHeaders)
        {
            $safeName = $header.Replace('/', '__')
            $cSource = Join-Path $laneRoot "header-$safeName.c"
            $cppSource = Join-Path $laneRoot "header-$safeName.cpp"
            [System.IO.File]::WriteAllText($cSource, "#include <$header>`r`nint main(void)`r`n{`r`n    return 0;`r`n}`r`n", [System.Text.Encoding]::ASCII)
            [System.IO.File]::WriteAllText($cppSource, "#include <$header>`r`nint main()`r`n{`r`n    return 0;`r`n}`r`n", [System.Text.Encoding]::ASCII)
            Invoke-Checked $compilerExe ($commonCFlags + @("/c", $cSource, "/Fo$(Join-Path $laneRoot "header-$safeName.obj")"))
            Invoke-Checked $compilerExe ($commonCppFlags + @("/c", $cppSource, "/Fo$(Join-Path $laneRoot "header-$safeName-cpp.obj")"))
        }
        Invoke-Checked $compilerExe ($commonCFlags + @("/c", (Join-Path $testRoot "include_c11.c"), "/Fo$(Join-Path $laneRoot 'include_c11.obj')"))
        Invoke-Checked $compilerExe ($commonCppFlags + @("/c", (Join-Path $testRoot "include_cpp20.cpp"), "/Fo$(Join-Path $laneRoot 'include_cpp20.obj')"))

    }
    foreach ($testName in $runtimeCTests)
    {
        $consumerArguments = @()
        if ($testName -in @("extensions_c11", "extensions_cpp20") -and $Scope -in @("Stages", "Callbacks", "Body-Control", "Restitution"))
        {
            $consumerArguments = @($Scope.ToLowerInvariant())
        }
        if ($Scope -eq "Extensions" -and $testName -in @("extensions_c11", "extensions_cpp20"))
        {
            $consumerArguments = @("joint-breaks")
        }
        if (($Scope -eq "Queries" -and $testName -in @("queries_events_views_c11", "queries_properties_cpp20")) -or
            ($Scope -eq "Memory" -and $testName -eq "lifecycle_c11"))
        {
            $consumerArguments = @($Scope.ToLowerInvariant())
        }
        $source = Join-Path $testRoot "$testName.c"
        $sharedBinary = Join-Path $laneRoot "$testName-shared.exe"
        $staticBinary = Join-Path $laneRoot "$testName-static.exe"
        if ($Scope -eq "Compatibility")
        {
            Copy-RetainedConsumer $testName $sharedBinary
        }
        else
        {
            Invoke-Checked $compilerExe ($commonCFlags + @($source, "/Fo$(Join-Path $laneRoot "$testName-shared.obj")", "/Fe:$sharedBinary", "/link", $runtimeImport))
        }
        Invoke-Consumer $sharedBinary $consumerArguments
        Invoke-Checked $compilerExe ($commonCFlags + @($source, "/Fo$(Join-Path $laneRoot "$testName-static.obj")", "/Fe:$staticBinary", "/link", $runtimeStatic))
        Invoke-Consumer $staticBinary $consumerArguments "Static"
    }
    foreach ($testName in $runtimeCppTests)
    {
        $consumerArguments = @()
        if ($testName -in @("extensions_c11", "extensions_cpp20") -and $Scope -in @("Stages", "Callbacks", "Body-Control", "Restitution"))
        {
            $consumerArguments = @($Scope.ToLowerInvariant())
        }
        if ($Scope -eq "Extensions" -and $testName -in @("extensions_c11", "extensions_cpp20"))
        {
            $consumerArguments = @("joint-breaks")
        }
        if (($Scope -eq "Queries" -and $testName -in @("queries_events_views_c11", "queries_properties_cpp20")) -or
            ($Scope -eq "Memory" -and $testName -eq "lifecycle_c11"))
        {
            $consumerArguments = @($Scope.ToLowerInvariant())
        }
        $source = Join-Path $testRoot "$testName.cpp"
        $sharedBinary = Join-Path $laneRoot "$testName-shared.exe"
        $staticBinary = Join-Path $laneRoot "$testName-static.exe"
        if ($Scope -eq "Compatibility")
        {
            Copy-RetainedConsumer $testName $sharedBinary
        }
        else
        {
            Invoke-Checked $compilerExe ($commonCppFlags + @($source, "/Fo$(Join-Path $laneRoot "$testName-shared.obj")", "/Fe:$sharedBinary", "/link", $runtimeImport))
        }
        Invoke-Consumer $sharedBinary $consumerArguments
        Invoke-Checked $compilerExe ($commonCppFlags + @($source, "/Fo$(Join-Path $laneRoot "$testName-static.obj")", "/Fe:$staticBinary", "/link", $runtimeStatic))
        Invoke-Consumer $staticBinary $consumerArguments "Static"
    }
    foreach ($testName in $cookingTests)
    {
        $consumerArguments = @()
        $source = Join-Path $testRoot "$testName.c"
        $sharedBinary = Join-Path $laneRoot "$testName-shared.exe"
        $staticBinary = Join-Path $laneRoot "$testName-static.exe"
        if ($Scope -eq "Compatibility")
        {
            Copy-RetainedConsumer $testName $sharedBinary
        }
        else
        {
            Invoke-Checked $compilerExe ($commonCFlags + @(
                $source, "/Fo$(Join-Path $laneRoot "$testName-shared.obj")", "/Fe:$sharedBinary",
                "/link", $cookingImport, $runtimeImport
            ))
        }
        Invoke-Consumer $sharedBinary $consumerArguments
        # Odin emits imported package objects into both archives. the cooking C
        # fixture also calls runtime ABI symbols, so both archives are required.
        # select one copy of their identically built package objects at link time
        Invoke-Checked $compilerExe ($commonCFlags + @(
            $source, "/Fo$(Join-Path $laneRoot "$testName-static.obj")", "/Fe:$staticBinary",
            "/link", $runtimeStatic, $cookingStatic, "/FORCE:MULTIPLE", "/IGNORE:4006"
        ))
        Invoke-Consumer $staticBinary $consumerArguments "Static"
    }
    if ($Scope -eq "Compatibility")
    {
        foreach ($pair in @("old-old", "new-new", "old-new", "new-old"))
        {
            $runtimeRoot = if ($pair.StartsWith("new-"))
            {
                $buildRoot
            }
            else
            {
                $baselineBuild
            }
            $cookingRoot = if ($pair.EndsWith("-new"))
            {
                $buildRoot
            }
            else
            {
                $baselineBuild
            }
            $expected = if ($pair -in @("old-old", "new-new"))
            {
                "accept"
            }
            else
            {
                "reject"
            }
            $pairRoot = Join-Path $laneRoot $pair
            $null = New-Item -ItemType Directory -Path $pairRoot
            Copy-Item -LiteralPath (Join-Path $runtimeRoot "entasis.dll") -Destination $pairRoot
            Copy-Item -LiteralPath (Join-Path $cookingRoot "entasis_cooking.dll") -Destination $pairRoot
            $binary = Join-Path $pairRoot "check.exe"
            Invoke-Checked $compilerExe ($commonCFlags + @(
                (Join-Path $script:Root "tests/c_abi/runtime_cooking_compatibility_c11.c"),
                "/Fo$(Join-Path $pairRoot 'check.obj')", "/Fe:$binary", "/link",
                (Join-Path $runtimeRoot "entasis.lib"), (Join-Path $cookingRoot "entasis_cooking.lib")
            ))
            Invoke-Checked $binary @($expected)
            Write-Host "C_ABI_COMPATIBILITY_PAIR lane=$lane pair=$pair expected=$expected"
        }
    }
}
if ($compatibilityFailures.Count -ne 0)
{
    [Console]::Error.WriteLine("C_ABI_COMPATIBILITY_FAILED count=$($compatibilityFailures.Count) fixtures=$($compatibilityFailures -join ', ')")
    exit $script:compatibilityExitCode
}
Write-Host "C_ABI_CONSUMERS_OK configuration=$($Configuration.ToLowerInvariant()) compiler=$($Compiler.ToLowerInvariant()) scope=$($Scope.ToLowerInvariant())"
