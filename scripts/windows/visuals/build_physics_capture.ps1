$ErrorActionPreference = "Stop"
function Stop-CaptureBuildUsage([string]$Message)
{
    [Console]::Error.WriteLine("error: $Message")
    exit 2
}
$options = @{}
for ($index = 0; $index -lt $args.Count; $index += 2)
{
    $key = ([string]$args[$index]).ToLowerInvariant()
    if ($key -notin @("-configuration", "-sourceroot", "-sourcelabel", "-outputdirectory") -or $options.ContainsKey($key))
    {
        Stop-CaptureBuildUsage "unknown or duplicate option: $key"
    }
    if ($index + 1 -ge $args.Count -or [string]::IsNullOrWhiteSpace($args[$index + 1]) -or ([string]$args[$index + 1]).StartsWith("-"))
    {
        Stop-CaptureBuildUsage "$key requires a value"
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
    Stop-CaptureBuildUsage "configuration must be development|release"
}
$sourceLabel = "working-tree"
if ($options.ContainsKey("-sourcelabel"))
{
    $sourceLabel = $options["-sourcelabel"]
}
if ($sourceLabel -cnotmatch '^[a-zA-Z0-9._ -]{1,120}$')
{
    Stop-CaptureBuildUsage "source label must be 1..120 plain ASCII characters"
}
. (Join-Path $PSScriptRoot "../lib/common.ps1")
$sourceRoot = $script:Root
if ($options.ContainsKey("-sourceroot"))
{
    $sourceRoot = $options["-sourceroot"]
    if (-not [IO.Path]::IsPathRooted($sourceRoot))
    {
        $sourceRoot = Join-Path $script:Root $sourceRoot
    }
    $sourceRoot = [IO.Path]::GetFullPath($sourceRoot)
}
if (-not (Test-Path -LiteralPath (Join-Path $sourceRoot "src/entasis") -PathType Container))
{
    Stop-CaptureBuildUsage "source root has no src/entasis"
}
if ($sourceRoot -ne $script:Root)
{
    if (-not $options.ContainsKey("-sourcelabel") -or -not $options.ContainsKey("-outputdirectory"))
    {
        Stop-CaptureBuildUsage "alternate source requires SourceLabel and OutputDirectory"
    }
    $currentFiles = @{}
    $selectedFiles = @{}
    foreach ($folder in @("examples/headless"))
    {
        foreach ($file in Get-ChildItem -LiteralPath (Join-Path $script:Root $folder) -Recurse -Filter *.odin -File)
        {
            $currentFiles[$file.FullName.Substring($script:Root.Length + 1)] = $file.FullName
        }
        foreach ($file in Get-ChildItem -LiteralPath (Join-Path $sourceRoot $folder) -Recurse -Filter *.odin -File)
        {
            $selectedFiles[$file.FullName.Substring($sourceRoot.Length + 1)] = $file.FullName
        }
    }
    $differences = 0
    foreach ($path in $currentFiles.Keys)
    {
        if (-not $selectedFiles.ContainsKey($path) -or
            [IO.File]::ReadAllText($currentFiles[$path]) -cne [IO.File]::ReadAllText($selectedFiles[$path]))
        {
            [Console]::Error.WriteLine("Fixture differs: $path")
            $differences += 1
        }
    }
    foreach ($path in $selectedFiles.Keys)
    {
        if (-not $currentFiles.ContainsKey($path))
        {
            [Console]::Error.WriteLine("Extra fixture: $path")
            $differences += 1
        }
    }
    if ($differences -ne 0)
    {
        Stop-CaptureBuildUsage "alternate source fixtures differ"
    }
}
$outputDirectory = "build/physics-capture/windows/$configuration/current"
if ($options.ContainsKey("-outputdirectory"))
{
    $outputDirectory = $options["-outputdirectory"]
}
if (-not [IO.Path]::IsPathRooted($outputDirectory))
{
    $outputDirectory = Join-Path $script:Root $outputDirectory
}
if (Test-Path -LiteralPath $outputDirectory)
{
    Stop-CaptureBuildUsage "output directory already exists: $outputDirectory"
}
$odin = Resolve-OdinBinary ""
New-Item -ItemType Directory -Path $outputDirectory | Out-Null
$executable = Join-Path $outputDirectory "physics_capture.exe"
$profile = @("-debug", "-o:none", "-source-code-locations:normal", "-pdb-name:$(Join-Path $outputDirectory 'physics_capture.pdb')")
if ($configuration -eq "release")
{
    $profile = @("-o:speed", "-no-bounds-check", "-disable-assert", "-source-code-locations:none")
}
$arguments = @(
    "build", (Join-Path $script:Root "tools/physics_capture"), "-out:$executable",
    "-collection:entasis=$(Join-Path $sourceRoot 'src')", "-define:ENTASIS_REPLAY_SOURCE=$sourceLabel",
    "-define:ENTASIS_VISUAL_CONFIGURATION=$configuration",
    "-target:$script:OdinTarget", "-microarch:$script:OdinMicroarch",
    "-vet", "-warnings-as-errors", "-thread-count:$script:OdinThreadCount", "-linker:lld"
) + $profile
Invoke-BoundedProcess $odin $arguments 600
Write-Host "PHYSICS_CAPTURE_BUILD_OK directory=$outputDirectory"
