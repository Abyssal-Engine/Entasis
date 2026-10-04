[CmdletBinding(PositionalBinding = $false)]
param(
    [Parameter(Mandatory = $true)]
    [ValidateSet(
        "awakening_duplicate_sets", "container",
        "contact_churn_grid_4k", "noncontact_constraint_mix", "noncontact_fallback_smoke",
        "shape_mixed_bounds", "spatial_query_trace", "spatial_query_batch",
        "custom_extensions", "query_extensions", "world_lifecycle",
        "contact_islands", "ragdoll_stair_tumble", "pyramid"
    )]
    [string]$Package,
    [Parameter(Mandatory = $true)]
    [ValidateRange(1, 2147483647)]
    [int]$Runs,
    [Parameter(Mandatory = $true)]
    [ValidateNotNullOrEmpty()]
    [string]$Workers,
    [ValidateSet("Development", "Release")]
    [string]$Configuration = "Release",
    [string]$Components = "All",
    [string]$Record = "Off",
    [string]$Compression = "lz4",
    [int]$MemoryMiB = 512,
    [string]$Shape,
    [string]$StaticShape,
    [string]$ShapeSize,
    [string]$Density,
    [string]$LayoutScale,
    [string]$Steps,
    [string]$TimestepHz,
    [string]$WarmupSteps,
    [string]$VelocityIterations,
    [string]$Substeps,
    [string]$Sleep,
    [string]$Grid,
    [string]$Spacing,
    [string]$SpawnHeight,
    [string]$ContainerSize,
    [string]$IslandGrid,
    [string]$IslandSpacing,
    [string]$FloorSize,
    [string]$Rows,
    [string]$ProjectileCount,
    [string]$LaunchStep,
    [string]$ProjectileRadius,
    [string]$ProjectileDensity,
    [string]$ProjectileCenter,
    [string]$ProjectileSpacing,
    [string]$ProjectileVelocity,
    [string]$Variant,
    [ValidatePattern('^[1-9][0-9]{0,9}$')]
    [ValidateScript({ [long]$_ -le [int]::MaxValue })]
    [string]$TimeoutSeconds
)
$preparationClock = [Diagnostics.Stopwatch]::StartNew()
. (Join-Path $PSScriptRoot "../lib/common.ps1")
. (Join-Path $PSScriptRoot "build_support.ps1")
if ($Components -notin @("Common", "All"))
{
    [Console]::Error.WriteLine("Components must be Common or All.")
    exit 2
}
if ($PSBoundParameters.ContainsKey("Components") -and $Package -notin $script:BenchmarkComponentPackages)
{
    [Console]::Error.WriteLine("Components is not applicable to $Package.")
    exit 2
}

if ($Record -notin @("Off", "On") -or $Compression -notin @("lz4", "lz4hc") -or $MemoryMiB -lt 64 -or $MemoryMiB -gt 16384 -or
    ($Record -eq "On" -and $Package -notin $script:BenchmarkRecordingPackages) -or
    ($Record -eq "Off" -and ($PSBoundParameters.ContainsKey("Compression") -or $PSBoundParameters.ContainsKey("MemoryMiB"))))
{
    [Console]::Error.WriteLine("Invalid recording options or unsupported package.")
    exit 2
}

function Invoke-BenchmarkChild(
    [string]$FilePath, [string[]]$Arguments, [string]$OutputPrefix, [string]$Stage,
    [Diagnostics.Stopwatch]$Clock, [long]$BudgetMilliseconds)
{
    if ($Clock.ElapsedMilliseconds -ge $BudgetMilliseconds)
    {
        [string]$message = "BENCHMARK_TIMEOUT stage=$Stage elapsed_ms=$($Clock.ElapsedMilliseconds) before_launch"
        [Console]::Error.WriteLine($message)
        [IO.File]::WriteAllText("$OutputPrefix.timeout.txt", $message)
        return 124
    }
    [Diagnostics.ProcessStartInfo]$start = [Diagnostics.ProcessStartInfo]::new()
    $start.FileName = $FilePath
    $start.UseShellExecute = $false
    $start.RedirectStandardOutput = $true
    $start.RedirectStandardError = $true
    foreach ($argument in $Arguments)
    {
        $start.ArgumentList.Add($argument)
    }
    [Diagnostics.Stopwatch]$childClock = [Diagnostics.Stopwatch]::StartNew()
    [Threading.CancellationTokenSource]$copyCancellation = [Threading.CancellationTokenSource]::new()
    [IO.FileStream]$stdout = $null
    [IO.FileStream]$stderr = $null
    [Diagnostics.Process]$process = $null
    [long]$observedPeak = 0
    [int]$exitCode = 124
    try
    {
        $stdout = [IO.File]::Create("$OutputPrefix.stdout.txt")
        $stderr = [IO.File]::Create("$OutputPrefix.stderr.txt")
        $process = [Diagnostics.Process]::Start($start)
        [Threading.Tasks.Task]$stdoutTask = $process.StandardOutput.BaseStream.CopyToAsync($stdout, 4096, $copyCancellation.Token)
        [Threading.Tasks.Task]$stderrTask = $process.StandardError.BaseStream.CopyToAsync($stderr, 4096, $copyCancellation.Token)
        while (-not $process.HasExited)
        {
            $observedPeak = Get-ProcessPeakWorkingSet $process $observedPeak
            [long]$remaining = $BudgetMilliseconds - $Clock.ElapsedMilliseconds
            if ($remaining -le 0)
            {
                break
            }
            [void]$process.WaitForExit([int][Math]::Min($remaining, 100))
        }
        if ($process.HasExited -and $Clock.ElapsedMilliseconds -lt $BudgetMilliseconds)
        {
            $exitCode = $process.ExitCode
            foreach ($task in @($stdoutTask, $stderrTask))
            {
                [long]$remaining = $BudgetMilliseconds - $Clock.ElapsedMilliseconds
                if ($remaining -le 0 -or -not $task.Wait([int][Math]::Min($remaining, [int]::MaxValue)))
                {
                    $exitCode = 124
                    break
                }
            }
        }
        if ($exitCode -eq 124)
        {
            [string]$message = "BENCHMARK_TIMEOUT stage=$Stage elapsed_ms=$($Clock.ElapsedMilliseconds) pid=$($process.Id)"
            [Console]::Error.WriteLine($message)
            [IO.File]::WriteAllText("$OutputPrefix.timeout.txt", $message)
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
                [long]$remaining = $BudgetMilliseconds - $Clock.ElapsedMilliseconds
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
    Write-ProcessObservation $childClock $OutputPrefix $Stage $observedPeak
    [Console]::Out.Write([IO.File]::ReadAllText("$OutputPrefix.stdout.txt"))
    [Console]::Error.Write([IO.File]::ReadAllText("$OutputPrefix.stderr.txt"))
    if ($exitCode -ne 0 -and $exitCode -ne 124)
    {
        [Console]::Error.WriteLine("Command failed ($exitCode): $FilePath stage=$Stage")
    }
    return $exitCode
}

try
{
Assert-BenchmarkPackage $Package
$workloadOptionNames = [ordered]@{
    Shape = "shape"
    StaticShape = "static-shape"
    ShapeSize = "shape-size"
    Density = "density"
    LayoutScale = "layout-scale"
    Steps = "steps"
    TimestepHz = "timestep-hz"
    WarmupSteps = "warmup-steps"
    VelocityIterations = "velocity-iterations"
    Substeps = "substeps"
    Sleep = "sleep"
    Grid = "grid"
    Spacing = "spacing"
    SpawnHeight = "spawn-height"
    ContainerSize = "container-size"
    IslandGrid = "island-grid"
    IslandSpacing = "island-spacing"
    FloorSize = "floor-size"
    Rows = "rows"
    ProjectileCount = "projectile-count"
    LaunchStep = "launch-step"
    ProjectileRadius = "projectile-radius"
    ProjectileDensity = "projectile-density"
    ProjectileCenter = "projectile-center"
    ProjectileSpacing = "projectile-spacing"
    ProjectileVelocity = "projectile-velocity"
}
$nativeOptions = @()
$reportOptions = @()
$resultPackage = $Package
if ($Package -in @("container", "contact_islands", "pyramid"))
{
    if ($Shape -notin @("box", "sphere", "capsule", "cylinder", "hull") -or
        $Variant -cnotmatch '^[A-Za-z0-9][A-Za-z0-9._-]*$')
    {
        throw "New workloads require valid Shape and Variant."
    }
    $resultPackage = "$Package\$Variant"
    $reportOptions = @("--variant", $Variant)
}
else
{
    foreach ($option in @($workloadOptionNames.Keys) + @("Variant"))
    {
        if ($PSBoundParameters.ContainsKey($option))
        {
            throw "Option $option is not applicable to $Package."
        }
    }
}
foreach ($option in $workloadOptionNames.Keys)
{
    if ($PSBoundParameters.ContainsKey($option))
    {
        $value = $PSBoundParameters[$option]
        if ([string]::IsNullOrEmpty($value))
        {
            throw "Option $option requires a value."
        }
        $nativeOptions += "--$($workloadOptionNames[$option])=$value"
    }
}
$workerValues = @()
foreach ($workerText in $Workers.Split(','))
{
    $worker = 0
    if (-not [int]::TryParse($workerText.Trim(), [ref]$worker))
    {
        throw "Invalid worker count: $workerText"
    }
    Assert-WorkerCount $worker
    if ($workerValues -contains $worker)
    {
        throw "Duplicate worker count: $worker"
    }
    $workerValues += $worker
}
if ($workerValues.Count -eq 0)
{
    throw "Workers must not be empty."
}

}
catch
{
    [Console]::Error.WriteLine($_.Exception.Message)
    exit 2
}

$binary = Get-BenchmarkBinary $Package $Configuration $Record $Components
$reporter = Get-BenchmarkReporterBinary $Configuration
$buildCommand = "& .\scripts\windows\benchmarks\build_benchmarks.ps1 -Package $Package -Configuration $Configuration -Record $Record"
if ($Package -in $script:BenchmarkComponentPackages)
{
    $buildCommand += " -Components $Components"
}
if (-not (Test-Path -LiteralPath $binary -PathType Leaf))
{
    throw "Missing benchmark binary: $binary. Build with: $buildCommand"
}
if (-not (Test-Path -LiteralPath $reporter -PathType Leaf))
{
    throw "Missing benchmark reporter: $reporter. Build with: $buildCommand"
}
$compilerVersion = "Unavailable (reused producer binary)"
$operatingSystem = [System.Runtime.InteropServices.RuntimeInformation]::OSDescription.Trim()
$processor = [Environment]::GetEnvironmentVariable("PROCESSOR_IDENTIFIER")
if ([string]::IsNullOrWhiteSpace($operatingSystem) -or [string]::IsNullOrWhiteSpace($processor))
{
    throw "Required host provenance is unavailable."
}
$processor = $processor.Trim()
$configurationName = $Configuration.ToLowerInvariant()
$resultParent = Join-Path $script:Root "build\benchmark-results\windows_amd64\$resultPackage\$configurationName"
$attempt = [DateTime]::UtcNow.ToString("yyyyMMddTHHmmssfffffffZ") + "." + [Guid]::NewGuid().ToString("N")
$pending = Join-Path $resultParent "$attempt.pending"
$current = Join-Path $resultParent $attempt
New-Item -ItemType Directory -Force -Path $resultParent | Out-Null
New-Item -ItemType Directory -Path $pending | Out-Null
Write-Host "BENCHMARK_RUN_CONFIG package=$Package configuration=$configurationName components=$($Components.ToLowerInvariant()) pending=$pending"
$workloadClock = [Diagnostics.Stopwatch]::StartNew()
Write-Host "BENCHMARK_PREPARED elapsed_ms=$($preparationClock.ElapsedMilliseconds)"
$budgetMilliseconds = [long]$TimeoutSeconds * 1000
foreach ($worker in $workerValues)
{
    $output = Join-Path $pending "workers-$worker.csv"
    for ($run = 1; $run -le $Runs; $run++)
    {
        Write-Host "[benchmark] $Package workers=$worker run=$run/$Runs"
        $childArguments = @("--worker-count=$worker", "--output=$output") + $nativeOptions
        if ($Package -in $script:BenchmarkRecordingPackages)
        {
            $childArguments += "--record=$($Record.ToLowerInvariant())"
            if ($Record -eq "On")
            {
                $recordingOutput = Join-Path $pending "workers-$worker-sample-$run.epr"
                $childArguments += @("--recording-output=$recordingOutput", "--compression=$Compression", "--memory-mib=$MemoryMiB")
            }
        }
        if ($PSBoundParameters.ContainsKey("TimeoutSeconds"))
        {
            $exitCode = Invoke-BenchmarkChild $binary $childArguments (Join-Path $pending "workers-$worker-run-$run") "worker=$worker run=$run/$Runs" $workloadClock $budgetMilliseconds
            if ($exitCode -ne 0)
            {
                exit $exitCode
            }
        }
        else
        {
            Invoke-Checked $binary $childArguments
        }
        if ($Package -in $script:BenchmarkRecordingPackages)
        {
            $samples = @(Import-Csv -LiteralPath $output)
            if ($samples.Count -ne $run -or $samples[-1].recording_mode -cne $Record.ToLowerInvariant())
            {
                throw "Recording capability mismatch in $output"
            }
            if ($Record -eq "On" -and ($samples[-1].recording_path -cne [IO.Path]::GetFileName($recordingOutput) -or -not (Test-Path -LiteralPath $recordingOutput -PathType Leaf)))
            {
                throw "Missing or mismatched finalized recording in $output"
            }
        }
        if ($Package -in $script:BenchmarkComponentPackages)
        {
            $samples = @(Import-Csv -LiteralPath $output)
            if ($samples.Count -eq 0 -or $samples[-1].benchmark_components -cne $Components.ToLowerInvariant())
            {
                throw "Benchmark component mismatch: requested $Components; explicitly build the matching component selection. Output: $output"
            }
        }
    }
}
$canonicalCommand = "& .\scripts\windows\benchmarks\run_benchmark.ps1 -Package $Package -Runs $Runs -Workers '$Workers' -Configuration $Configuration"
$canonicalCommand += " -Record $($Record.ToLowerInvariant())"
if ($Record -eq "On")
{
    $canonicalCommand += " -Compression $Compression -MemoryMiB $MemoryMiB"
}
if ($Package -in $script:BenchmarkComponentPackages)
{
    $canonicalCommand += " -Components $($Components.ToLowerInvariant())"
}
if ($PSBoundParameters.ContainsKey("TimeoutSeconds"))
{
    $canonicalCommand += " -TimeoutSeconds $TimeoutSeconds"
}
foreach ($option in @($workloadOptionNames.Keys) + @("Variant"))
{
    if ($PSBoundParameters.ContainsKey($option))
    {
        $literal = ([string]$PSBoundParameters[$option]).Replace("'", "''")
        $canonicalCommand += " -$option '$literal'"
    }
}
$reportArguments = @(
    "--source", $pending,
    "--package", $Package,
    "--configuration", $configurationName,
    "--runs", "$Runs",
    "--workers", $Workers,
    "--compiler-version", $compilerVersion,
    "--operating-system", $operatingSystem,
    "--processor", $processor,
    "--logical-processors", "$script:AvailableWorkerCount",
    "--command", $canonicalCommand
) + $reportOptions
Write-Host "BENCHMARK_STAGE stage=report workload_ms=$($workloadClock.ElapsedMilliseconds)"
$reportClock = [Diagnostics.Stopwatch]::StartNew()
if ($PSBoundParameters.ContainsKey("TimeoutSeconds"))
{
    $exitCode = Invoke-BenchmarkChild $reporter $reportArguments (Join-Path $pending "reporter") "reporter" $workloadClock $budgetMilliseconds
    if ($exitCode -ne 0)
    {
        exit $exitCode
    }
}
else
{
    Invoke-Checked $reporter $reportArguments
}
$workloadClock.Stop()
Write-Host "BENCHMARK_REPORT elapsed_ms=$($reportClock.ElapsedMilliseconds)"
Write-Host "BENCHMARK_ELAPSED milliseconds=$($workloadClock.ElapsedMilliseconds)"
Complete-BenchmarkResultCommit $pending $current
$reportPath = Join-Path $current "README.md"
Write-Host "BENCHMARK_OK result=$current report=$reportPath"
