[CmdletBinding(PositionalBinding = $false)]
param(
    [ValidateNotNullOrEmpty()]
    [string]$OdinExe,
    [Parameter(Mandatory = $true)]
    [ValidateSet(
        "root", "scene-stability", "benchmark-support", "benchmark-report", "physics-visuals", "cooking", "bodies", "broadphase", "collections", "collision-batching",
        "collision-pairs", "contact-optimization", "constraints", "intrinsics", "islands", "layout",
        "narrowphase-integration", "queries", "public-api", "release-parity", "shapes",
        "simulation", "solver-kernels", "sweeps", "tasking-multi", "tasking-single",
        "threading", "trees", "utilities-bundles", "utilities-math", "utilities-memory"
    )]
    [string]$Scope,
    [ValidateSet("Development", "Release")]
    [string]$Configuration = "Development",
    [string]$Tests,
    [string]$Threads
)
. (Join-Path $PSScriptRoot "../lib/common.ps1")
$testNames = ""
$testDisplay = "all"
if ($PSBoundParameters.ContainsKey("Tests"))
{
    $testNames = Resolve-TestSelectorList $Tests
    $testDisplay = $testNames
}
$testThreads = $script:TestThreads
if ($PSBoundParameters.ContainsKey("Threads"))
{
    $testThreads = Resolve-TestThreadCount $Threads
}
$odin = Resolve-OdinBinary $OdinExe
$package = Get-TestPackageForScope $Scope
Write-Host "TEST_CONFIG scope=$Scope tests=$testDisplay threads=$testThreads"
Invoke-TestPackage $odin $package $Configuration $testNames $testThreads
Write-Host "TEST_OK scope=$Scope"
