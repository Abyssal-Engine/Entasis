[CmdletBinding(PositionalBinding = $false)]
param(
    [ValidateNotNullOrEmpty()]
    [string]$OdinExe,
    [ValidateSet("Development", "Release")]
    [string]$Configuration = "Release",
    [ValidateSet(
        "awakening_duplicate_sets", "container", "contact_churn_grid_4k",
        "noncontact_constraint_mix", "noncontact_fallback_smoke", "shape_mixed_bounds", "spatial_query_trace", "spatial_query_batch",
        "custom_extensions", "query_extensions", "world_lifecycle",
        "contact_islands", "ragdoll_stair_tumble", "pyramid", "pyramid_wall", "scene_stability"
    )]
    [string]$Package,
    [string]$Components = "All",
    [ValidateSet("Executable", "Assembly")]
    [string]$Emit = "Executable",
    [string]$Record = "Off",
    [ValidateNotNullOrEmpty()]
    [string]$HarnessRoot
)
. (Join-Path $PSScriptRoot "../lib/common.ps1")
. (Join-Path $PSScriptRoot "build_support.ps1")
if ($Components -notin @("Common", "All"))
{
    [Console]::Error.WriteLine("Components must be Common or All.")
    exit 2
}
if ($PSBoundParameters.ContainsKey("Components") -and $PSBoundParameters.ContainsKey("Package") -and
    $Package -notin $script:BenchmarkComponentPackages)
{
    [Console]::Error.WriteLine("Components is not applicable to $Package.")
    exit 2
}
if ($Emit -eq "Assembly" -and ($Configuration -ne "Release" -or -not $PSBoundParameters.ContainsKey("Package")))
{
    [Console]::Error.WriteLine("Assembly output requires Release and one explicit Package.")
    exit 2
}
if ($Record -notin @("Off", "On") -or
    ($Record -eq "On" -and ($Package -notin $script:BenchmarkRecordingPackages -or $Emit -ne "Executable")))
{
    [Console]::Error.WriteLine("Record must be Off or On; On requires one eligible package and executable output.")
    exit 2
}
[string]$script:BenchmarkHarnessRoot = $script:Root
if ($PSBoundParameters.ContainsKey("HarnessRoot"))
{
    if (-not [IO.Path]::IsPathFullyQualified($HarnessRoot) -or -not (Test-Path -LiteralPath $HarnessRoot -PathType Container))
    {
        [Console]::Error.WriteLine("HarnessRoot must be an existing absolute native project root.")
        exit 2
    }
    $script:BenchmarkHarnessRoot = [IO.Path]::GetFullPath($HarnessRoot)
}
[string[]]$required = @("benchmarks/benchmark_support", "tools/benchmark_report")
if ($PSBoundParameters.ContainsKey("Package"))
{
    $required += "benchmarks/$Package"
}
foreach ($directory in $required)
{
    if (-not (Test-Path -LiteralPath (Join-Path $script:BenchmarkHarnessRoot $directory) -PathType Container))
    {
        [Console]::Error.WriteLine("HarnessRoot is missing $directory.")
        exit 2
    }
}
[Diagnostics.Stopwatch]$script:BenchmarkBuildClock = [Diagnostics.Stopwatch]::StartNew()
$odin = Resolve-OdinBinary $OdinExe
$packages = $script:BenchmarkPackages
if ($PSBoundParameters.ContainsKey("Package"))
{
    $packages = @($Package)
}
Invoke-BoundedProcess $odin @("version") 600
Write-Host "BENCHMARK_BUILD_INPUTS engine=$(Join-Path $script:Root 'src') harness=$script:BenchmarkHarnessRoot compiler=$odin target=$script:OdinTarget microarch=$script:OdinMicroarch deadline_seconds=600"
Write-Host "BENCHMARK_BUILD_CONFIG configuration=$($Configuration.ToLowerInvariant()) packages=$($packages -join ',') emit=$($Emit.ToLowerInvariant()) components=$($Components.ToLowerInvariant())"

foreach ($selectedPackage in $packages)
{
    Build-Benchmark $odin $selectedPackage $Configuration $Emit $Components $Record
}
if ($Emit -eq "Executable")
{
    Build-BenchmarkReporter $odin $Configuration
}
Write-Host "BUILD_BENCHMARKS_OK"
