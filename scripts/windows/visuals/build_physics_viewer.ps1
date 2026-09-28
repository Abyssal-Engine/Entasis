$ErrorActionPreference = "Stop"
$configuration = "development"
if ($args.Count -ne 0)
{
    if ($args.Count -ne 2 -or $args[0] -cne "-Configuration" -or $args[1] -notin @("development", "release"))
    {
        [Console]::Error.WriteLine("usage: build_physics_viewer.ps1 [-Configuration development|release]")
        exit 2
    }
    $configuration = ([string]$args[1]).ToLowerInvariant()
}
. (Join-Path $PSScriptRoot "../lib/common.ps1")
. (Join-Path $PSScriptRoot "../lib/toolchain_common.ps1")
$started = [Diagnostics.Stopwatch]::StartNew()
$odin = Resolve-OdinBinary ""
Initialize-OdinProfile "physics-viewer/windows" "physics_viewer" $configuration
$arguments = @(
    "build", (Join-Path $script:Root "tools/physics_viewer"), "-out:$script:OdinProfileOutput",
    "-collection:entasis=$(Join-Path $script:Root 'src')",
    "-define:ENTASIS_VIEWER_POWERSHELL=$([Diagnostics.Process]::GetCurrentProcess().MainModule.FileName)",
    "-define:ENTASIS_VISUAL_CONFIGURATION=$configuration", "-define:ENTASIS_BENCHMARK_COMPONENTS=all",
    "-target:$script:OdinTarget", "-microarch:$script:OdinMicroarch",
    "-vet", "-warnings-as-errors", "-thread-count:$script:OdinThreadCount", "-linker:lld"
) + $script:OdinProfileArguments
$remaining = 600 - [int][Math]::Ceiling($started.Elapsed.TotalSeconds)
if ($remaining -le 0)
{
    exit 124
}
Invoke-BoundedProcess $odin $arguments $remaining
Write-Host "PHYSICS_VIEWER_BUILD_OK executable=$script:OdinProfileOutput"
