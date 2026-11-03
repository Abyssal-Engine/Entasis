[CmdletBinding(PositionalBinding = $false)]
param(
    [string]$Origin,
    [string]$OutputRoot,
    [string]$ProcessorAffinity,
    [string]$TimeoutSeconds
)
. (Join-Path $PSScriptRoot "../lib/common.ps1")
. (Join-Path $PSScriptRoot "build_support.ps1")
. (Join-Path $PSScriptRoot "process_support.ps1")

try
{
    [int]$seconds = 0
    [uint64]$mask = 0
    if ($Origin -cnotin @("accepted", "r02-pairs") -or
        -not [IO.Path]::IsPathFullyQualified($OutputRoot) -or
        $TimeoutSeconds -cnotmatch '^[1-9][0-9]*$' -or
        -not [int]::TryParse($TimeoutSeconds, [ref]$seconds) -or $seconds -gt 3600 -or
        $ProcessorAffinity -cnotmatch '^[1-9][0-9]*$' -or
        -not [uint64]::TryParse($ProcessorAffinity, [ref]$mask))
    {
        throw "Capture requires Origin accepted|r02-pairs, an absolute OutputRoot, positive decimal ProcessorAffinity and TimeoutSeconds 1..3600"
    }
    if ([IntPtr]::Size -ne 8)
    {
        throw "Capture affinity requires a 64-bit host"
    }
    [Diagnostics.Process]$owner = [Diagnostics.Process]::GetCurrentProcess()
    try
    {
        [uint64]$eligible = [BitConverter]::ToUInt64([BitConverter]::GetBytes($owner.ProcessorAffinity.ToInt64()), 0)
    }
    finally
    {
        $owner.Dispose()
    }
    if (($mask -band $eligible) -ne $mask)
    {
        throw "Capture mask must be a subset of eligible processors: $eligible"
    }
    $OutputRoot = [IO.Path]::GetFullPath($OutputRoot)
    foreach ($suffix in @("step150.bin", "step300.bin", "capture.stdout.txt", "capture.stderr.txt", "capture.timeout.txt", "capture.observation.txt"))
    {
        if (Test-Path -LiteralPath (Join-Path $OutputRoot "$Origin.$suffix"))
        {
            throw "Capture output already exists: $Origin.$suffix"
        }
    }
    [string]$binary = Get-BenchmarkBinary "solver_contact_capture" "Release"
    if (-not (Test-Path -LiteralPath $binary -PathType Leaf))
    {
        throw "Missing prepared capture image: $binary. Use the public Build command for solver_contact_capture"
    }
    Initialize-BenchmarkProcessAffinity
}
catch
{
    [Console]::Error.WriteLine($_.Exception.Message)
    exit 2
}

New-Item -ItemType Directory -Force -Path $OutputRoot | Out-Null
[Diagnostics.Stopwatch]$clock = [Diagnostics.Stopwatch]::StartNew()
[int]$exitCode = Invoke-BenchmarkChild $binary @("--origin=$Origin", "--output-root=$OutputRoot") (Join-Path $OutputRoot "$Origin.capture") "capture origin=$Origin worker=1" $clock ([long]$seconds * 1000) $mask
if ($exitCode -eq 0)
{
    foreach ($step in @(150, 300))
    {
        [string]$inputPath = Join-Path $OutputRoot "$Origin.step$step.bin"
        if (-not (Test-Path -LiteralPath $inputPath -PathType Leaf) -or (Get-Item -LiteralPath $inputPath).Length -eq 0)
        {
            [Console]::Error.WriteLine("Capture completed without required input: $inputPath")
            exit 2
        }
    }
    Write-Host "SOLVER_CONTACT_CAPTURE_OK origin=$Origin output=$OutputRoot"
}
exit $exitCode
