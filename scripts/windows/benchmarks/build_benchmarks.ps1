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
        "contact_islands", "ragdoll_stair_tumble", "pyramid"
    )]
    [string]$Package,
    [string]$Components = "All",
    [ValidateSet("Executable", "Assembly")]
    [string]$Emit = "Executable",
    [string]$Record = "Off"
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
$odin = Resolve-OdinBinary $OdinExe
$packages = $script:BenchmarkPackages
if ($PSBoundParameters.ContainsKey("Package"))
{
    $packages = @($Package)
}
Invoke-Checked $odin @("version")
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
