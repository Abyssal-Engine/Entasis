$ErrorActionPreference = "Stop"
$script:Root = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot "../../.."))
$script:OdinTarget = "windows_amd64"
$script:OdinMicroarch = "x86-64-v3"
$script:AvailableWorkerCount = [Math]::Min([Math]::Max([Environment]::ProcessorCount, 1), 64)
$script:TestThreads = $script:AvailableWorkerCount
$script:OdinThreadCount = [Math]::Min($script:AvailableWorkerCount, 8)
$script:BenchmarkPackages = @(
    "awakening_duplicate_sets",
    "container",
    "contact_churn_grid_4k",
    "noncontact_constraint_mix",
    "noncontact_fallback_smoke",
    "shape_mixed_bounds", "spatial_query_trace", "spatial_query_batch",
    "custom_extensions", "query_extensions", "world_lifecycle",
    "contact_islands", "ragdoll_stair_tumble", "pyramid"
)
$script:BenchmarkRecordingPackages = @("container", "contact_islands", "pyramid", "ragdoll_stair_tumble", "noncontact_constraint_mix", "noncontact_fallback_smoke")
$script:BenchmarkComponentPackages = @("custom_extensions", "query_extensions", "contact_islands")
$script:TestPackages = @(
    "root", "benchmark_support", "benchmark_report", "physics_visuals", "cooking", "bodies", "broadphase", "collections", "collision_batching",
    "collision_pairs", "contact_optimization", "constraints", "intrinsics", "islands", "layout",
    "narrowphase_integration", "queries", "public_api", "release_parity", "shapes",
    "simulation", "solver_kernels", "sweeps", "tasking_multi", "tasking_single",
    "threading", "trees", "utilities_bundles", "utilities_math", "utilities_memory"
)
$script:CodegenPackages = @(
    "bodies/codegen", "simulation/codegen", "solver_kernels/codegen"
)
$script:TestScopeToPackage = @{
    "root" = "root"
    "benchmark-support" = "benchmark_support"
    "benchmark-report" = "benchmark_report"
    "physics-visuals" = "physics_visuals"
    "cooking" = "cooking"
    "bodies" = "bodies"
    "broadphase" = "broadphase"
    "collections" = "collections"
    "collision-batching" = "collision_batching"
    "collision-pairs" = "collision_pairs"
    "contact-optimization" = "contact_optimization"
    "constraints" = "constraints"
    "intrinsics" = "intrinsics"
    "islands" = "islands"
    "layout" = "layout"
    "narrowphase-integration" = "narrowphase_integration"
    "queries" = "queries"
    "public-api" = "public_api"
    "release-parity" = "release_parity"
    "shapes" = "shapes"
    "simulation" = "simulation"
    "solver-kernels" = "solver_kernels"
    "sweeps" = "sweeps"
    "tasking-multi" = "tasking_multi"
    "tasking-single" = "tasking_single"
    "threading" = "threading"
    "trees" = "trees"
    "utilities-bundles" = "utilities_bundles"
    "utilities-math" = "utilities_math"
    "utilities-memory" = "utilities_memory"
}

function Resolve-OdinBinary([string]$OdinExe)
{
    . (Join-Path $PSScriptRoot "toolchain_common.ps1")
    return (Resolve-ToolchainOdin $OdinExe)
}

function Invoke-Checked([string]$FilePath, [string[]]$Arguments)
{
    $global:LASTEXITCODE = 0
    & $FilePath @Arguments
    $exitCode = $LASTEXITCODE
    if ($exitCode -ne 0)
    {
        [Console]::Error.WriteLine("Command failed ($exitCode): $FilePath $($Arguments -join ' ')")
        exit $exitCode
    }
}

function Invoke-BoundedProcess([string]$FilePath, [string[]]$Arguments, [int]$TimeoutSeconds)
{
    $start = [Diagnostics.ProcessStartInfo]::new($FilePath)
    $start.UseShellExecute = $false
    $start.WorkingDirectory = $script:Root
    foreach ($argument in $Arguments)
    {
        $start.ArgumentList.Add($argument)
    }
    $process = [Diagnostics.Process]::Start($start)
    try
    {
        if (-not $process.WaitForExit($TimeoutSeconds * 1000))
        {
            $process.Kill($true)
            $process.WaitForExit()
            [Console]::Error.WriteLine("Process timed out after $TimeoutSeconds seconds: $FilePath")
            exit 124
        }
        if ($process.ExitCode -ne 0)
        {
            exit $process.ExitCode
        }
    }
    finally
    {
        if (-not $process.HasExited)
        {
            $process.Kill($true)
            $process.WaitForExit()
        }
        $process.Dispose()
    }
}

function Get-TestPackagePath([string]$Package)
{
    if ($Package -eq "root")
    {
        return (Join-Path $script:Root "tests")
    }
    if ($Package -eq "benchmark_report")
    {
        return (Join-Path $script:Root "tools\benchmark_report")
    }
    if ($Package -eq "benchmark_support")
    {
        return (Join-Path $script:Root "benchmarks\benchmark_support")
    }
    return (Join-Path $script:Root "tests\$Package")
}

function Get-TestPackageForScope([string]$Scope)
{
    if (-not $script:TestScopeToPackage.ContainsKey($Scope))
    {
        throw "Unknown test scope: $Scope"
    }
    return $script:TestScopeToPackage[$Scope]
}

function Get-SafeName([string]$Value)
{
    return ($Value -replace '[\\/]', '__')
}

function Assert-BenchmarkPackage([string]$Package)
{
    if ($script:BenchmarkPackages -notcontains $Package)
    {
        throw "Unknown benchmark package: $Package"
    }
}

function Get-ExamplePackages
{
    [string]$root = Join-Path $script:Root "examples/headless"
    [System.IO.DirectoryInfo[]]$directories = @(Get-ChildItem -LiteralPath $root -Directory)
    [string[]]$packages = @(
        foreach ($directory in $directories)
        {
            [string]$source = Join-Path $directory.FullName "main.odin"
            if (Test-Path -LiteralPath $source -PathType Leaf)
            {
                if ($directory.Name -cnotmatch '^[a-z][a-z0-9_]*$')
                {
                    [Console]::Error.WriteLine("Invalid example package path: $source")
                    exit 2
                }
                $directory.Name
            }
        }
    )
    if ($packages.Count -eq 0)
    {
        [Console]::Error.WriteLine("No example packages in $root")
        exit 2
    }
    [Array]::Sort($packages, [StringComparer]::Ordinal)
    return $packages
}

function Assert-ExamplePackage([string]$Package)
{
    if ($Package -cnotmatch '^[a-z][a-z0-9_]*$' -or
        -not (Test-Path -LiteralPath (Join-Path $script:Root "examples/headless/$Package/main.odin") -PathType Leaf))
    {
        [Console]::Error.WriteLine("Unknown example package: $Package")
        exit 2
    }
}

function Assert-WorkerCount([int]$WorkerCount)
{
    if ($WorkerCount -lt 1 -or $WorkerCount -gt $script:AvailableWorkerCount)
    {
        throw "Worker count $WorkerCount exceeds the $script:AvailableWorkerCount CPUs available to this process."
    }
}

function Get-BenchmarkResultState([string]$Path)
{
    $item = Get-Item -LiteralPath $Path -Force -ErrorAction SilentlyContinue
    if ($null -eq $item)
    {
        return "Absent"
    }
    if (-not $item.PSIsContainer)
    {
        return "Incomplete"
    }
    $report = Join-Path $Path "README.md"
    $lanes = @(Get-ChildItem -LiteralPath $Path -File -Filter "workers-*.csv" -ErrorAction Stop)
    if ((Test-Path -LiteralPath $report -PathType Leaf) -and
        (Get-Item -LiteralPath $report -ErrorAction Stop).Length -gt 0 -and
        $lanes.Count -gt 0 -and
        ($lanes | Where-Object Length -gt 0).Count -gt 0)
    {
        return "Complete"
    }
    return "Incomplete"
}

function Complete-BenchmarkResultCommit([string]$Pending, [string]$Completed)
{
    if ((Get-BenchmarkResultState $Pending) -ne "Complete")
    {
        throw "Incomplete pending benchmark result: $Pending"
    }
    [IO.Directory]::Move($Pending, $Completed)
}

function Resolve-TestSelectorList([string]$Tests)
{
    if ($Tests -notmatch '^[A-Za-z_][A-Za-z0-9_]*(\.[A-Za-z_][A-Za-z0-9_]*)?(,[A-Za-z_][A-Za-z0-9_]*(\.[A-Za-z_][A-Za-z0-9_]*)?)*$')
    {
        throw "Invalid test selector list: $Tests"
    }
    return $Tests
}

function Resolve-TestThreadCount([string]$Threads)
{
    if ($Threads -notmatch '^[1-9][0-9]*$')
    {
        throw "Test thread count must be a positive integer: $Threads"
    }
    $threadCount = [int]::Parse($Threads, [Globalization.CultureInfo]::InvariantCulture)
    if ($threadCount -gt $script:AvailableWorkerCount)
    {
        throw "Test thread count $threadCount exceeds the $script:AvailableWorkerCount CPUs available to this process."
    }
    return $threadCount
}

function Get-ExampleBinary([string]$Package, [string]$Configuration)
{
    switch ($Configuration)
    {
        "Development"
        {
            return (Join-Path $script:Root "build\examples\development\$Package.exe")
        }
        "Release"
        {
            return (Join-Path $script:Root "build\examples\release\$Package.exe")
        }
        default
        {
            throw "Unknown build configuration: $Configuration"
        }
    }
}

function Initialize-OdinProfile([string]$ArtifactGroup, [string]$Package, [string]$Configuration)
{
    $configurationName = $Configuration.ToLowerInvariant()
    $profileBuild = Join-Path $script:Root "build\$ArtifactGroup\$configurationName"
    New-Item -ItemType Directory -Force -Path $profileBuild | Out-Null
    $script:OdinProfileOutput = Join-Path $profileBuild "$(Get-SafeName $Package).exe"
    $script:OdinProfilePdb = [System.IO.Path]::ChangeExtension($script:OdinProfileOutput, ".pdb")
    Remove-Item -LiteralPath $script:OdinProfileOutput, "$script:OdinProfileOutput.obj", $script:OdinProfilePdb -Force -ErrorAction SilentlyContinue
    switch ($Configuration)
    {
        "Development"
        {
            $script:OdinProfileArguments = @(
                "-debug", "-o:none", "-source-code-locations:normal",
                "-pdb-name:$script:OdinProfilePdb"
            )
            $symbols = $script:OdinProfilePdb
        }
        "Release"
        {
            $script:OdinProfileArguments = @(
                "-o:speed", "-no-bounds-check", "-disable-assert",
                "-source-code-locations:none"
            )
            $symbols = "none"
        }
        default
        {
            throw "Unknown build configuration: $Configuration"
        }
    }
    Write-Host "BUILD_PROFILE package=$Package configuration=$configurationName executable=$script:OdinProfileOutput symbols=$symbols"
}

function Invoke-TestPackage(
    [string]$OdinExe,
    [string]$Package,
    [string]$Configuration,
    [string]$Tests = "",
    [int]$Threads = $script:TestThreads
)
{
    Initialize-OdinProfile "tests" $Package $Configuration
    Write-Host "[test] $Package"
    $arguments = @(
        "test", (Get-TestPackagePath $Package), "-out:$script:OdinProfileOutput", "-keep-executable",
        "-collection:entasis=$(Join-Path $script:Root 'src')", "-target:$script:OdinTarget",
        "-microarch:$script:OdinMicroarch"
    )
    $arguments += $script:OdinProfileArguments
    $arguments += @(
        "-vet",
        "-warnings-as-errors", "-thread-count:$script:OdinThreadCount", "-linker:lld",
        "-define:ODIN_TEST_THREADS=$Threads", "-define:ODIN_TEST_FANCY=false"
    )
    if ($Tests.Length -gt 0)
    {
        $arguments += "-define:ODIN_TEST_NAMES=$Tests"
    }
    Invoke-Checked $OdinExe $arguments
}

function Invoke-CodegenPackage([string]$OdinExe, [string]$Package, [string]$Configuration)
{
    Initialize-OdinProfile "codegen" $Package $Configuration
    Write-Host "[codegen] $Package"
    $arguments = @(
        "build", (Join-Path $script:Root "tests\$Package"), "-out:$script:OdinProfileOutput",
        "-collection:entasis=$(Join-Path $script:Root 'src')", "-target:$script:OdinTarget",
        "-microarch:$script:OdinMicroarch"
    )
    $arguments += $script:OdinProfileArguments
    $arguments += @(
        "-vet", "-warnings-as-errors",
        "-thread-count:$script:OdinThreadCount", "-linker:lld"
    )
    Invoke-Checked $OdinExe $arguments
    Invoke-Checked $script:OdinProfileOutput @()
}
