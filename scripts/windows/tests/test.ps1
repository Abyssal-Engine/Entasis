[CmdletBinding(PositionalBinding = $false)]
param(
    [ValidateNotNullOrEmpty()]
    [string]$OdinExe,
    [Parameter(Mandatory = $true)]
    [ValidateSet(
        "precision-bounds", "precision-broadphase", "precision-storage", "precision-collisions", "precision-constraints", "precision-targets", "precision-mutations", "precision-callbacks", "precision-queries", "precision-snapshots", "benchmark-precision",
        "root", "scene-stability", "benchmark-support", "benchmark-report", "benchmark-query", "physics-visuals", "physics-viewer", "cooking", "bodies", "broadphase", "collections", "collision-batching",
        "collision-pairs", "contact-optimization", "constraints", "intrinsics", "islands", "layout",
        "narrowphase-integration", "queries", "public-api", "release-parity", "shapes",
        "simulation", "solver-kernels", "sweeps", "tasking-multi", "tasking-single",
        "threading", "trees", "utilities-bundles", "utilities-math", "utilities-memory"
    )]
    [string]$Scope,
    [ValidateSet("Development", "Release")]
    [string]$Configuration = "Development",
    [string]$Tests,
    [string]$Threads,
    [ValidateSet("Float32", "Mixed64")]
    [string]$Precision = "Float32",
    [ValidateSet("Off", "On")]
    [string]$MathChecks = "Off",
    [ValidateSet("baseline", "origin", "mixed64")]
    [string]$PrecisionCandidate
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
if ($Scope -eq "benchmark-precision")
{
    if (-not $PSBoundParameters.ContainsKey("PrecisionCandidate"))
    {
        [Console]::Error.WriteLine("benchmark-precision requires -PrecisionCandidate")
        exit 2
    }
    [string]$candidatePrecision = if ($PrecisionCandidate -eq "mixed64")
    {
        "Mixed64"
    }
    else
    {
        "Float32"
    }
    if ($PSBoundParameters.ContainsKey("Precision") -and $Precision -ne $candidatePrecision)
    {
        [Console]::Error.WriteLine("Conflicting precision and candidate")
        exit 2
    }
    $Precision = $candidatePrecision
}
elseif ($PSBoundParameters.ContainsKey("PrecisionCandidate"))
{
    [Console]::Error.WriteLine("-PrecisionCandidate is only valid for benchmark-precision")
    exit 2
}
if ($Scope.StartsWith("precision-") -and $Precision -ne "Mixed64")
{
    [Console]::Error.WriteLine("Precision scopes require Mixed64")
    exit 2
}
$odin = Resolve-OdinBinary $OdinExe
$package = Get-TestPackageForScope $Scope
Write-Host "TEST_CONFIG scope=$Scope tests=$testDisplay threads=$testThreads"
Invoke-TestPackage $odin $package $Configuration $testNames $testThreads $Precision $MathChecks $PrecisionCandidate
Write-Host "TEST_OK scope=$Scope"
