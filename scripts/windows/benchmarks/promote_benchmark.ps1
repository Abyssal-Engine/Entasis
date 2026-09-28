param(
    [Parameter(Mandatory = $true)]
    [string]$Package,
    [Parameter(Mandatory = $true)]
    [ValidateNotNullOrEmpty()]
    [string]$Name,
    [string]$Variant,
    [Parameter(Mandatory = $true)]
    [string]$RunDirectory
)
. (Join-Path $PSScriptRoot "../lib/common.ps1")
try
{
    Assert-BenchmarkPackage $Package
}
catch
{
    [Console]::Error.WriteLine($_.Exception.Message)
    exit 2
}
if ($Name -notmatch '^[A-Za-z0-9][A-Za-z0-9._-]*$' -or $Name -eq "." -or $Name -eq "..")
{
    throw "Invalid result name: $Name"
}
$resultPackage = $Package
if ($Package -in @("container", "contact_islands", "pyramid"))
{
    if ($Variant -cnotmatch '^[A-Za-z0-9][A-Za-z0-9._-]*$')
    {
        throw "Variant is required and must be a safe name."
    }
    $resultPackage = "$Package\$Variant"
}
elseif ($PSBoundParameters.ContainsKey("Variant"))
{
    throw "Variant requires a configurable workload."
}
$destinationParent = Join-Path $script:Root "results\$resultPackage"
$destination = Join-Path $destinationParent $Name
$destinationItem = Get-Item -LiteralPath $destination -Force -ErrorAction SilentlyContinue
if ($null -ne $destinationItem)
{
    throw "Promotion destination already exists: $destination"
}
$releaseRoot = [IO.Path]::GetFullPath((Join-Path $script:Root "build\benchmark-results\windows_amd64\$resultPackage\release"))
$source = [IO.Path]::GetFullPath($RunDirectory)
if ((Split-Path -Parent $source) -ne $releaseRoot -or $source.EndsWith(".pending") -or $source.EndsWith(".failed"))
{
    [Console]::Error.WriteLine("RunDirectory must select a completed Release run for this package and variant")
    exit 2
}
foreach ($path in @($source, $destination))
{
    $cursor = $path
    while ($cursor.Length -gt $script:Root.Length)
    {
        $item = Get-Item -LiteralPath $cursor -Force -ErrorAction SilentlyContinue
        if ($null -ne $item -and ($item.Attributes -band [IO.FileAttributes]::ReparsePoint) -ne 0)
        {
            throw "Promotion path is linked: $cursor"
        }
        $cursor = Split-Path -Parent $cursor
    }
}
$report = Join-Path $source "README.md"
$laneFiles = @(Get-ChildItem -LiteralPath $source -File -Filter "workers-*.csv" -ErrorAction SilentlyContinue)
if (-not (Test-Path -LiteralPath $source -PathType Container) -or
    -not (Test-Path -LiteralPath $report -PathType Leaf) -or
    $laneFiles.Count -eq 0)
{
    throw "Complete Release run not found: $source"
}
foreach ($item in @(Get-ChildItem -LiteralPath $source -Recurse -Force))
{
    if (($item.Attributes -band [IO.FileAttributes]::ReparsePoint) -ne 0)
    {
        throw "Promotion source contains a link: $($item.FullName)"
    }
}
$recordings = [Collections.Generic.HashSet[string]]::new([StringComparer]::OrdinalIgnoreCase)
foreach ($lane in $laneFiles)
{
    foreach ($row in @(Import-Csv -LiteralPath $lane.FullName))
    {
        if ($row.PSObject.Properties.Name -contains "recording_path" -and $row.recording_path)
        {
            if ($row.recording_mode -ne "on" -or $row.recording_path -cnotmatch '^[A-Za-z0-9_.-]+$' -or
                $row.recording_path -in @(".", "..") -or $row.recording_path.EndsWith(".partial"))
            {
                throw "Invalid declared recording in $($lane.FullName)"
            }
            $recording = Join-Path $source $row.recording_path
            if (-not (Test-Path -LiteralPath $recording -PathType Leaf))
            {
                throw "Missing declared recording: $recording"
            }
            if (-not $recordings.Add($recording))
            {
                throw "Recording associated with multiple samples: $recording"
            }
        }
    }
}
New-Item -ItemType Directory -Force -Path $destinationParent | Out-Null
try
{
    New-Item -ItemType Directory -Path $destination | Out-Null
    [string[]]$publishFiles = @($report) + @($laneFiles.FullName) + @($recordings)
    Copy-Item -LiteralPath $publishFiles -Destination $destination -ErrorAction Stop
}
catch
{
    Remove-Item -LiteralPath $destination -Recurse -Force -ErrorAction SilentlyContinue
    throw
}
Write-Host "BENCHMARK_PROMOTED source=$source destination=$destination"
