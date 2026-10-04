[CmdletBinding(PositionalBinding = $false)]
param(
    [Parameter(Mandatory = $true)][ValidateSet("contact_islands", "pyramid")][string]$Package,
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
    [ValidateRange(1, 2147483)][int]$TimeoutSeconds = 1800,
    [string]$Output,
    [string]$Snapshot
)
. (Join-Path $PSScriptRoot "../lib/common.ps1")
. (Join-Path $PSScriptRoot "build_support.ps1")
$Sleep = $Sleep.ToLowerInvariant()
$Package = $Package.ToLowerInvariant()
$Shape = $Shape.ToLowerInvariant()
if ($Package -ne "pyramid" -and $PSBoundParameters.ContainsKey("ProjectileCount"))
{
    [Console]::Error.WriteLine("ProjectileCount applies only to pyramid.")
    exit 2
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
    "--timestep-hz=$TimestepHz", "--sample-every=$SampleEvery", "--sleep=$Sleep", "--shape=$Shape",
    "--output=$Output", "--snapshot=$Snapshot")
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
