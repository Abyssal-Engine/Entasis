$ErrorActionPreference = "Stop"
function Stop-CaptureUsage([string]$Message)
{
    [Console]::Error.WriteLine("error: $Message")
    exit 2
}
$options = @{}
for ($index = 0; $index -lt $args.Count; $index += 2)
{
    $key = ([string]$args[$index]).ToLowerInvariant()
    if ($key -notin @("-builddirectory", "-scenario", "-recipe", "-output", "-frames", "-timeoutseconds", "-compression", "-memorymib") -or $options.ContainsKey($key))
    {
        Stop-CaptureUsage "unknown or duplicate option: $key"
    }
    if ($index + 1 -ge $args.Count -or [string]::IsNullOrWhiteSpace($args[$index + 1]) -or ([string]$args[$index + 1]).StartsWith("-"))
    {
        Stop-CaptureUsage "$key requires a value"
    }
    $options[$key] = [string]$args[$index + 1]
}
if (-not $options.ContainsKey("-builddirectory") -or -not $options.ContainsKey("-output") -or
    $options.ContainsKey("-scenario") -eq $options.ContainsKey("-recipe"))
{
    Stop-CaptureUsage "BuildDirectory, Output and exactly one Scenario or Recipe are required"
}
foreach ($key in @("-frames", "-timeoutseconds"))
{
    if (-not $options.ContainsKey($key))
    {
        continue
    }
    $maximum = 1000000
    if ($key -eq "-timeoutseconds")
    {
        $maximum = 3600
    }
    if ($options[$key] -notmatch '^[1-9][0-9]{0,6}$' -or [int]$options[$key] -gt $maximum)
    {
        Stop-CaptureUsage "$key must be 1..$maximum"
    }
}
if ($options.ContainsKey("-compression") -and $options["-compression"] -cnotin @("lz4", "lz4hc"))
{
    Stop-CaptureUsage "compression must be lz4|lz4hc"
}
if ($options.ContainsKey("-memorymib") -and
    ($options["-memorymib"] -notmatch '^[0-9]{2,5}$' -or [int]$options["-memorymib"] -lt 64 -or [int]$options["-memorymib"] -gt 16384))
{
    Stop-CaptureUsage "MemoryMiB must be 64..16384"
}
. (Join-Path $PSScriptRoot "../lib/common.ps1")
foreach ($key in @("-builddirectory", "-output", "-recipe"))
{
    if ($options.ContainsKey($key) -and -not [IO.Path]::IsPathRooted($options[$key]))
    {
        $options[$key] = Join-Path $script:Root $options[$key]
    }
}
$output = $options["-output"]
$exe = Join-Path $options["-builddirectory"] "physics_capture.exe"
if (-not (Test-Path -LiteralPath $exe -PathType Leaf))
{
    Stop-CaptureUsage "missing capture executable: $exe"
}
foreach ($path in @($output, "$output.partial", "$output.log", "$output.stderr.log"))
{
    if (Test-Path -LiteralPath $path)
    {
        Stop-CaptureUsage "output already exists: $path"
    }
}
if (-not (Test-Path -LiteralPath (Split-Path $output -Parent) -PathType Container))
{
    Stop-CaptureUsage "output parent directory must exist"
}
$timeoutSeconds = 120
if ($options.ContainsKey("-timeoutseconds"))
{
    $timeoutSeconds = [int]$options["-timeoutseconds"]
}
$start = [Diagnostics.ProcessStartInfo]::new($exe)
$start.WorkingDirectory = $script:Root
$start.UseShellExecute = $false
$start.RedirectStandardOutput = $true
$start.RedirectStandardError = $true
foreach ($key in @("-scenario", "-recipe", "-output", "-frames", "-compression", "-memorymib"))
{
    if ($options.ContainsKey($key))
    {
        $argument = "-$key"
        if ($key -eq "-memorymib")
        {
            $argument = "--memory-mib"
        }
        $start.ArgumentList.Add("$argument=$($options[$key])")
    }
}
$stdout = [IO.File]::Open("$output.log", [IO.FileMode]::CreateNew)
$stderr = [IO.File]::Open("$output.stderr.log", [IO.FileMode]::CreateNew)
$process = [Diagnostics.Process]::Start($start)
$status = 0
try
{
    $outTask = $process.StandardOutput.BaseStream.CopyToAsync($stdout)
    $errTask = $process.StandardError.BaseStream.CopyToAsync($stderr)
    if (-not $process.WaitForExit($timeoutSeconds * 1000))
    {
        $process.Kill($true)
        $process.WaitForExit()
        $status = 124
        [Console]::Error.WriteLine("Capture incomplete: timeout, partial output and logs retained at $output")
    }
    else
    {
        $status = $process.ExitCode
    }
    [void]$outTask.GetAwaiter().GetResult()
    [void]$errTask.GetAwaiter().GetResult()
}
finally
{
    if (-not $process.HasExited)
    {
        $process.Kill($true)
        $process.WaitForExit()
    }
    $process.Dispose()
    $stdout.Dispose()
    $stderr.Dispose()
}
Get-Content -LiteralPath "$output.log"
Get-Content -LiteralPath "$output.stderr.log" | ForEach-Object { [Console]::Error.WriteLine($_) }
exit $status
