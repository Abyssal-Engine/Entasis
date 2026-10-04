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
    "root", "scene_stability", "benchmark_support", "benchmark_report", "physics_visuals", "cooking", "bodies", "broadphase", "collections", "collision_batching",
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
    "scene-stability" = "scene_stability"
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

function Invoke-BoundedProcess(
    [string]$FilePath, [string[]]$Arguments, [int]$TimeoutSeconds,
    [string]$LogPrefix = "", [string]$Stage = "process")
{
    [Diagnostics.Stopwatch]$clock = [Diagnostics.Stopwatch]::StartNew()
    [long]$budgetMilliseconds = [long]$TimeoutSeconds * 1000
    [Diagnostics.ProcessStartInfo]$start = [Diagnostics.ProcessStartInfo]::new($FilePath)
    $start.UseShellExecute = $false
    $start.WorkingDirectory = $script:Root
    if ($LogPrefix.Length -gt 0)
    {
        $start.RedirectStandardOutput = $true
        $start.RedirectStandardError = $true
    }
    foreach ($argument in $Arguments)
    {
        $start.ArgumentList.Add($argument)
    }
    [Threading.CancellationTokenSource]$copyCancellation = [Threading.CancellationTokenSource]::new()
    [IO.FileStream]$stdout = $null
    [IO.FileStream]$stderr = $null
    [Threading.Tasks.Task]$stdoutTask = $null
    [Threading.Tasks.Task]$stderrTask = $null
    [Diagnostics.Process]$process = $null
    [int]$exitCode = 124
    [long]$observedPeak = 0
    try
    {
        if ($LogPrefix.Length -gt 0)
        {
            $stdout = [IO.File]::Create("$LogPrefix.stdout.txt")
            $stderr = [IO.File]::Create("$LogPrefix.stderr.txt")
        }
        $process = [Diagnostics.Process]::Start($start)
        if ($LogPrefix.Length -gt 0)
        {
            $stdoutTask = $process.StandardOutput.BaseStream.CopyToAsync($stdout, 4096, $copyCancellation.Token)
            $stderrTask = $process.StandardError.BaseStream.CopyToAsync($stderr, 4096, $copyCancellation.Token)
        }
        while (-not $process.HasExited)
        {
            $observedPeak = Get-ProcessPeakWorkingSet $process $observedPeak
            [long]$remaining = $budgetMilliseconds - $clock.ElapsedMilliseconds
            if ($remaining -le 0)
            {
                break
            }
            [void]$process.WaitForExit([int][Math]::Min($remaining, 100))
        }
        if ($process.HasExited -and $clock.ElapsedMilliseconds -lt $budgetMilliseconds)
        {
            $exitCode = $process.ExitCode
            if ($LogPrefix.Length -gt 0)
            {
                foreach ($task in @($stdoutTask, $stderrTask))
                {
                    [long]$remaining = $budgetMilliseconds - $clock.ElapsedMilliseconds
                    if ($remaining -le 0 -or -not $task.Wait([int][Math]::Min($remaining, [int]::MaxValue)))
                    {
                        $exitCode = 124
                        break
                    }
                }
            }
        }
        if ($exitCode -eq 124)
        {
            [string]$message = "PROCESS_TIMEOUT stage=$Stage timeout_seconds=$TimeoutSeconds elapsed_ms=$($clock.ElapsedMilliseconds)"
            [Console]::Error.WriteLine($message)
            if ($LogPrefix.Length -gt 0)
            {
                [IO.File]::WriteAllText("$LogPrefix.timeout.txt", $message)
            }
        }
    }
    finally
    {
        $copyCancellation.Cancel()
        try
        {
            if ($null -ne $process -and -not $process.HasExited)
            {
                try
                {
                    $process.Kill($true)
                }
                catch [InvalidOperationException]
                {
                    if (-not $process.HasExited)
                    {
                        throw
                    }
                }
                [long]$remaining = $budgetMilliseconds - $clock.ElapsedMilliseconds
                if ($remaining -gt 0)
                {
                    [void]$process.WaitForExit([int][Math]::Min($remaining, [int]::MaxValue))
                }
            }
        }
        finally
        {
            if ($null -ne $process)
            {
                $process.Dispose()
            }
            if ($null -ne $stdout)
            {
                $stdout.Dispose()
            }
            if ($null -ne $stderr)
            {
                $stderr.Dispose()
            }
            $copyCancellation.Dispose()
        }
    }
    if ($LogPrefix.Length -gt 0)
    {
        Write-ProcessObservation $clock $LogPrefix $Stage $observedPeak
        [Console]::Out.Write([IO.File]::ReadAllText("$LogPrefix.stdout.txt"))
        [Console]::Error.Write([IO.File]::ReadAllText("$LogPrefix.stderr.txt"))
    }
    if ($exitCode -ne 0)
    {
        exit $exitCode
    }
}

function Get-ProcessPeakWorkingSet([Diagnostics.Process]$Process, [long]$PreviousPeak)
{
    try
    {
        $Process.Refresh()
        return [Math]::Max($PreviousPeak, $Process.PeakWorkingSet64)
    }
    catch [InvalidOperationException]
    {
        return $PreviousPeak
    }
    catch [ComponentModel.Win32Exception]
    {
        return $PreviousPeak
    }
}

function Write-ProcessObservation(
    [Diagnostics.Stopwatch]$Clock, [string]$LogPrefix, [string]$Stage, [long]$ObservedPeak = 0)
{
    [string]$memoryState = "unavailable"
    [string]$peak = ""
    if ($ObservedPeak -gt 0)
    {
        $memoryState = "available"
        $peak = $ObservedPeak.ToString([Globalization.CultureInfo]::InvariantCulture)
    }
    [Diagnostics.Process]$parent = [Diagnostics.Process]::GetCurrentProcess()
    try
    {
        [string]$placement = "cpu=$([Environment]::GetEnvironmentVariable('PROCESSOR_IDENTIFIER')) logical_cpus=$([Environment]::ProcessorCount) inherited_affinity=$($parent.ProcessorAffinity.ToInt64())"
        [string]$line = "PROCESS_OBSERVATION stage=$Stage elapsed_ms=$($Clock.ElapsedMilliseconds) memory=$memoryState peak_working_set_bytes=$peak peak_scope=observed_while_running $placement"
        [IO.File]::AppendAllText("$LogPrefix.stdout.txt", "$line`n")
    }
    finally
    {
        $parent.Dispose()
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
    if ($Package -in @("benchmark_support", "scene_stability"))
    {
        return (Join-Path $script:Root "benchmarks\$Package")
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
    if ($script:BenchmarkPackages -notcontains $Package -and $Package -notin @("pyramid_wall", "scene_stability"))
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
    if ($Package -eq "scene_stability")
    {
        $arguments += "-define:ENTASIS_BENCHMARK_COMPONENTS=common"
    }
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
