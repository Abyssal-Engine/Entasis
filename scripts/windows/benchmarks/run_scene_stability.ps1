[CmdletBinding(PositionalBinding = $false)]
param(
    [Parameter(Mandatory = $true)][ValidateSet("container", "contact_islands", "pyramid", "solver_joint_angular")][string]$Package,
    [ValidateSet("Development", "Release")][string]$Configuration = "Release",
    [ValidateRange(1, 64)][int]$WorkerCount = 1,
    [ValidateRange(1, 1000000)][int]$Steps = 18000,
    [ValidateSet(0)][int]$WarmupSteps = 0,
    [ValidateRange(1, 1000000)][int]$VelocityIterations = 1,
    [ValidateRange(1, 1000000)][int]$Substeps = 4,
    [ValidateRange(1, 1000000)][int]$TimestepHz = 60,
    [ValidateRange(1, 1000000)][int]$SampleEvery = 60,
    [ValidateSet("enabled", "disabled")][string]$Sleep = "disabled",
    [ValidateSet("box")][string]$Shape = "box",
    [ValidateSet(0)][int]$ProjectileCount = 0,
    [string]$Scenario,
    [string]$AngularMode,
    [ValidateRange(1, 16384)][int]$Chains = 8,
    [ValidateRange(1, 128)][int]$Links = 8,
    [ValidateRange(1, 10000)][int]$MassRatio = 1,
    [ValidateRange(1, 2147483)][int]$TimeoutSeconds = 1800,
    [ValidateSet("On", "Off")][string]$Verify = "On",
    [string]$Output,
    [string]$Snapshot
)
. (Join-Path $PSScriptRoot "../lib/common.ps1")
. (Join-Path $PSScriptRoot "build_support.ps1")
$Sleep = $Sleep.ToLowerInvariant()
$Verify = $Verify.ToLowerInvariant()
$Package = $Package.ToLowerInvariant()
$Shape = $Shape.ToLowerInvariant()
if ($Package -ne "pyramid" -and $PSBoundParameters.ContainsKey("ProjectileCount"))
{
    [Console]::Error.WriteLine("ProjectileCount applies only to pyramid.")
    exit 2
}
if ($Package -eq "solver_joint_angular")
{
    if ($Scenario -cnotin @("hinge", "weld", "mixed") -or $AngularMode -cnotin @("momentum", "gyro") -or
        $Chains * ($Links + 1) -gt 1000000 -or $Steps -gt 100000 -or $VelocityIterations -ne 1 -or
        $Substeps -ne 4 -or $TimestepHz -ne 60 -or $Sleep -ne "disabled" -or
        $PSBoundParameters.ContainsKey("Shape") -or $PSBoundParameters.ContainsKey("Verify") -or
        ($PSBoundParameters.ContainsKey("SampleEvery") -and $SampleEvery -ne 30))
    {
        [Console]::Error.WriteLine("Angular quality requires its fixed 1x4/60Hz/sleep-off inputs, 30-step cadence, Scenario and AngularMode")
        exit 2
    }
    $SampleEvery = 30
}
else
{
    foreach ($option in @("Scenario", "AngularMode", "Chains", "Links", "MassRatio"))
    {
        if ($PSBoundParameters.ContainsKey($option))
        {
            [Console]::Error.WriteLine("$option requires solver_joint_angular")
            exit 2
        }
    }
}
[string]$binary = Get-BenchmarkBinary "scene_stability" $Configuration
[string]$stamp = [DateTime]::UtcNow.ToString("yyyyMMddTHHmmssfffffffZ")
if (-not $PSBoundParameters.ContainsKey("Output"))
{
    $Output = Join-Path $script:Root "build/benchmark-results/windows_amd64/scene_stability/$Package-$stamp.csv"
}
if (-not $PSBoundParameters.ContainsKey("Snapshot"))
{
    $Snapshot = "$Output.snapshot.csv"
}
$Output = [IO.Path]::GetFullPath($Output, $script:Root)
$Snapshot = [IO.Path]::GetFullPath($Snapshot, $script:Root)
[string[]]$destinations = @($Output, $Snapshot, "$Output.stdout.txt", "$Output.stderr.txt", "$Output.timeout.txt")
for ([int]$index = 0; $index -lt $destinations.Length; $index++)
{
    if (Test-Path -LiteralPath $destinations[$index])
    {
        [Console]::Error.WriteLine("Output, Snapshot and process logs must be fresh paths.")
        exit 2
    }
    for ([int]$other = 0; $other -lt $index; $other++)
    {
        if ([StringComparer]::OrdinalIgnoreCase.Equals($destinations[$index], $destinations[$other]))
        {
            [Console]::Error.WriteLine("Output, Snapshot and process logs must be distinct paths.")
            exit 2
        }
    }
}
if (-not (Test-Path -LiteralPath $binary -PathType Leaf))
{
    [Console]::Error.WriteLine("Prepared stability binary is missing: $binary. Use benchmark Build explicitly.")
    exit 2
}
Assert-WorkerCount $WorkerCount
[string[]]$arguments = @("--package=$Package", "--worker-count=$WorkerCount", "--steps=$Steps",
    "--warmup-steps=$WarmupSteps", "--velocity-iterations=$VelocityIterations", "--substeps=$Substeps",
    "--timestep-hz=$TimestepHz", "--sample-every=$SampleEvery", "--sleep=$Sleep",
    "--output=$Output", "--snapshot=$Snapshot")
if ($Package -eq "solver_joint_angular")
{
    $arguments += @("--scenario=$Scenario", "--angular-mode=$AngularMode", "--chains=$Chains", "--links=$Links", "--mass-ratio=$MassRatio")
}
else
{
    $arguments += @("--shape=$Shape", "--verify=$Verify")
}
if ($Package -eq "pyramid")
{
    $arguments += "--projectile-count=$ProjectileCount"
}
foreach ($path in @($Output, $Snapshot))
{
    New-Item -ItemType Directory -Force -Path (Split-Path -Parent $path) | Out-Null
}
Write-Host "STABILITY_RUN engine=$script:Root binary=$binary package=$Package output=$Output snapshot=$Snapshot timeout_seconds=$TimeoutSeconds"
Invoke-BoundedProcess $binary $arguments $TimeoutSeconds $Output "scene-stability"
