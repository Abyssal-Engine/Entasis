[CmdletBinding(PositionalBinding = $false)]
param(
    [ValidateSet("Development", "Release")][string]$Configuration = "Release",
    [ValidateRange(1, 2147483)][int]$TimeoutSeconds = 1800,
    [ValidateSet(180)][int]$Rows = 180,
    [ValidateRange(1, 1000000)][int]$Steps = 18000,
    [ValidateRange(1, 64)][int]$WorkerCount = 1,
    [ValidateRange(1, 1000000)][int]$VelocityIterations = 1,
    [ValidateRange(1, 1000000)][int]$Substeps = 4,
    [ValidateRange(1, 1000000)][int]$TimestepHz = 60,
    [ValidateRange(1, 1000000)][int]$SampleEvery = 60,
    [ValidateSet("enabled", "disabled")][string]$Sleep = "disabled",
    [ValidateRange(0, 1000000)][double]$Friction = 0.5,
    [ValidateRange(0.000001, 1000000)][double]$Hertz = 30,
    [ValidateRange(0.000001, 1000000)][double]$DampingRatio = 1,
    [ValidateRange(0, 1000000)][double]$Recovery = 2,
    [ValidateSet(0.05)][double]$RecycleDistance = 0.05,
    [string]$Output,
    [string]$Snapshot
)
. (Join-Path $PSScriptRoot "../lib/common.ps1")
. (Join-Path $PSScriptRoot "build_support.ps1")
$Sleep = $Sleep.ToLowerInvariant()
[string]$binary = Get-BenchmarkBinary "pyramid_wall" $Configuration
[string]$stamp = [DateTime]::UtcNow.ToString("yyyyMMddTHHmmssfffffffZ")
if (-not $PSBoundParameters.ContainsKey("Output"))
{
    $Output = Join-Path $script:Root "build/benchmark-results/windows_amd64/pyramid_wall/$stamp.csv"
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
    [Console]::Error.WriteLine("Prepared wall binary is missing: $binary. Use benchmark Build explicitly.")
    exit 2
}
Assert-WorkerCount $WorkerCount
[string[]]$arguments = @("--rows=$Rows", "--steps=$Steps", "--worker-count=$WorkerCount",
    "--velocity-iterations=$VelocityIterations", "--substeps=$Substeps", "--timestep-hz=$TimestepHz",
    "--sample-every=$SampleEvery", "--sleep=$Sleep", "--output=$Output", "--snapshot=$Snapshot")
foreach ($name in @("Friction", "Hertz", "DampingRatio", "Recovery"))
{
    [string]$key = switch ($name)
    {
        "DampingRatio"
        {
            "damping-ratio"
        }
        default
        {
            $name.ToLowerInvariant()
        }
    }
    [double]$value = Get-Variable -Name $name -ValueOnly
    $arguments += "--$key=$($value.ToString([Globalization.CultureInfo]::InvariantCulture))"
}
foreach ($path in @($Output, $Snapshot))
{
    New-Item -ItemType Directory -Force -Path (Split-Path -Parent $path) | Out-Null
}
Write-Host "WALL_RUN engine=$script:Root binary=$binary output=$Output snapshot=$Snapshot rows=$Rows timeout_seconds=$TimeoutSeconds"
Invoke-BoundedProcess $binary $arguments $TimeoutSeconds $Output "pyramid-wall"
