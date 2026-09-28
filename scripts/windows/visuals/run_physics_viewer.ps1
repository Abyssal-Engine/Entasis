$ErrorActionPreference = "Stop"
function Stop-ViewerUsage([string]$Message)
{
    [Console]::Error.WriteLine("error: $Message")
    exit 2
}
$names = @{
    "-configuration" = "configuration"
    "-scenario" = "scenario"
    "-recipe" = "recipe"
    "-replay" = "replay"
    "-results" = "results"
    "-compare" = "compare"
    "-package" = "package"
    "-frame" = "frame"
    "-screenshot" = "screenshot"
    "-memorymib" = "memory-mib"
    "-windowsize" = "window-size"
}
$options = @{}
for ($index = 0; $index -lt $args.Count; $index += 2)
{
    $key = ([string]$args[$index]).ToLowerInvariant()
    if (-not $names.ContainsKey($key) -or $options.ContainsKey($key))
    {
        Stop-ViewerUsage "unknown or duplicate option: $key"
    }
    if ($index + 1 -ge $args.Count -or [string]::IsNullOrWhiteSpace($args[$index + 1]) -or ([string]$args[$index + 1]).StartsWith("-"))
    {
        Stop-ViewerUsage "$key requires a value"
    }
    $options[$key] = [string]$args[$index + 1]
}
$configuration = "development"
if ($options.ContainsKey("-configuration"))
{
    $configuration = $options["-configuration"].ToLowerInvariant()
}
if ($configuration -notin @("development", "release"))
{
    Stop-ViewerUsage "configuration must be development|release"
}
$selections = @($options.Keys | Where-Object { $_ -in @("-scenario", "-recipe", "-replay", "-results") })
if ($selections.Count -gt 1 -or ($options.ContainsKey("-compare") -and -not $options.ContainsKey("-replay")) -or
    ($options.ContainsKey("-package") -and -not $options.ContainsKey("-results")))
{
    Stop-ViewerUsage "conflicting selection, compare requires replay, package requires results"
}
if ($options.ContainsKey("-frame") -and
    ($options["-frame"] -notmatch '^[0-9]{1,7}$' -or [int]$options["-frame"] -gt 1000000 -or
        (-not $options.ContainsKey("-replay") -and -not $options.ContainsKey("-screenshot"))))
{
    Stop-ViewerUsage "frame must be 0..1000000 and requires replay or screenshot"
}
if ($options.ContainsKey("-memorymib") -and
    ($options["-memorymib"] -notmatch '^[0-9]{2,5}$' -or [int]$options["-memorymib"] -lt 64 -or [int]$options["-memorymib"] -gt 16384))
{
    Stop-ViewerUsage "MemoryMiB must be 64..16384"
}
if ($options.ContainsKey("-windowsize"))
{
    if ($options["-windowsize"] -notmatch '^([0-9]{3,4}),([0-9]{3,4})$' -or
        [int]$Matches[1] -lt 640 -or [int]$Matches[1] -gt 7680 -or [int]$Matches[2] -lt 480 -or [int]$Matches[2] -gt 4320)
    {
        Stop-ViewerUsage "WindowSize must be 640..7680 by 480..4320"
    }
}
. (Join-Path $PSScriptRoot "../lib/common.ps1")
$exe = Join-Path $script:Root "build/physics-viewer/windows/$configuration/physics_viewer.exe"
if (-not (Test-Path -LiteralPath $exe -PathType Leaf))
{
    Stop-ViewerUsage "build viewer first: $exe"
}
$arguments = @()
foreach ($key in $options.Keys)
{
    if ($key -eq "-configuration")
    {
        continue
    }
    $value = $options[$key]
    if ($key -in @("-recipe", "-replay", "-results", "-compare", "-screenshot") -and -not [IO.Path]::IsPathRooted($value))
    {
        $value = Join-Path $script:Root $value
    }
    if ($key -eq "-screenshot" -and ((Test-Path -LiteralPath $value) -or -not (Test-Path -LiteralPath (Split-Path $value -Parent) -PathType Container)))
    {
        Stop-ViewerUsage "screenshot must be new and its parent must exist"
    }
    $arguments += "--$($names[$key])=$value"
}
Push-Location -LiteralPath $script:Root
try
{
if ($options.ContainsKey("-screenshot"))
{
    Invoke-BoundedProcess $exe $arguments 60
}
else
{
    Invoke-Checked $exe $arguments
}

}
finally
{
    Pop-Location
}
