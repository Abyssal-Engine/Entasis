[CmdletBinding(PositionalBinding = $false)]
param(
    [Parameter(Mandatory = $true)]
    [ValidateSet(
        "awakening_duplicate_sets", "container",
        "contact_churn_grid_4k", "noncontact_constraint_mix", "noncontact_fallback_smoke",
        "shape_mixed_bounds", "spatial_query_trace", "spatial_query_batch",
        "custom_extensions", "query_extensions", "world_lifecycle",
        "contact_islands", "ragdoll_stair_tumble", "pyramid", "large_world_precision"
    )]
    [string]$Package,
    [Parameter(Mandatory = $true)]
    [ValidateRange(1, 2147483647)]
    [int]$Runs,
    [Parameter(Mandatory = $true)]
    [ValidateNotNullOrEmpty()]
    [string]$Workers,
    [string]$ProcessorAffinity,
    [ValidateSet("Development", "Release")]
    [string]$Configuration = "Release",
    [string]$Precision = "Float32",
    [string]$RayPattern = "axial",
    [string]$QueryCase = "control",
    [string]$QueryOperation = "",
    [string]$QueryCell = "",
    [string]$QueryGroup = "",
    [string]$QueryRepetitions = "1",
    [string]$QueryWarmupRepetitions = "1",
    [string]$QueryFamily = "all",
    [string]$WarmupBatches = "10",
    [string]$MeasuredBatches = "100",
    [string]$PrecisionCandidate = "",
    [string]$PrecisionCase = "",
    [string]$SamplingMode = "batch",
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
    [string]$Verify = "On",
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
if ($RayPattern -cnotin @("axial", "oblique") -or
    ($PSBoundParameters.ContainsKey("RayPattern") -and $Package -notin @("spatial_query_batch", "spatial_query_trace")))
{
    [Console]::Error.WriteLine("RayPattern requires axial or oblique and a spatial query package")
    exit 2
}
$queryOptionNames = [ordered]@{
    QueryCase = "query-case"
    QueryOperation = "query-operation"
    QueryCell = "query-cell"
    QueryGroup = "query-group"
    QueryRepetitions = "query-repetitions"
    QueryWarmupRepetitions = "query-warmup-repetitions"
    QueryFamily = "query-family"
    WarmupBatches = "warmup-batches"
    MeasuredBatches = "measured-batches"
}
foreach ($option in $queryOptionNames.Keys)
{
    if (-not $PSBoundParameters.ContainsKey($option))
    {
        continue
    }
    if (($option -in @("QueryCase", "QueryOperation", "QueryCell", "QueryGroup", "QueryRepetitions", "QueryWarmupRepetitions") -and $Package -ne "query_extensions") -or
        ($option -in @("WarmupBatches", "MeasuredBatches") -and $Package -notin @("spatial_query_batch", "spatial_query_trace")) -or
        ($option -eq "QueryFamily" -and $Package -ne "spatial_query_trace"))
    {
        [Console]::Error.WriteLine("$option is not applicable to $Package")
        exit 2
    }
}
if ($QueryCase -ceq "control")
{
    if ($PSBoundParameters.ContainsKey("QueryOperation") -or $PSBoundParameters.ContainsKey("QueryCell"))
    {
        [Console]::Error.WriteLine("Control rejects QueryOperation and QueryCell")
        exit 2
    }
}
else
{
    if ($Package -ne "query_extensions" -or $Components -ine "All" -or
        $Workers -cnotmatch '^(1|8|16|24|32)(,(1|8|16|24|32))*$' -or
        $PSBoundParameters.ContainsKey("QueryGroup") -or $PSBoundParameters.ContainsKey("QueryRepetitions") -or
        $PSBoundParameters.ContainsKey("QueryWarmupRepetitions"))
    {
        [Console]::Error.WriteLine("Select one implemented query cell, All components and workers 1/8/16/24/32, without group/repetition selectors")
        exit 2
    }
    [int]$geometryCell = 0
    if (($QueryOperation -cin @("closest-point", "distance", "depenetration") -and $QueryCell -cin @("nested-query-64", "query-box-1")) -or
        ($QueryCase -ceq "hierarchy" -and $QueryOperation -cin @("sweep-any", "sweep-closest", "sweep-all") -and
            $QueryCell -cin @("compound4-nested64", "compound4-mesh128", "compound4-nested64-angular", "compound4-mesh128-angular", "query-box-1")))
    {
        $geometryCell = 1
    }
    if ($QueryCase -ceq "hierarchy")
    {
        if ($geometryCell -eq 0 -and ($QueryOperation -cnotin @("ray-direct", "ray-any", "ray-closest", "ray-all") -or
            $QueryCell -cnotin @("mesh-128", "mesh-8192", "nested-mesh-128", "nested-mesh-8192", "box-1", "compound-8", "big-128", "big-8192", "nested-compound-8", "nested-big-8192", "hull-32", "hull-256") -or
            ($QueryCell -cin @("compound-8", "big-128", "big-8192") -and $QueryOperation -cne "ray-direct")))
        {
            [Console]::Error.WriteLine("Select one implemented hierarchy operation/cell")
            exit 2
        }
    }
    elseif ($QueryCase -ceq "collision-batch")
    {
        if ($QueryOperation -cne "collision" -or $QueryCell -cnotin @(
            "box-native-1", "box-native-8", "box-native-32", "box-native-256",
            "box-context-1", "box-context-8", "box-context-32", "box-context-256",
            "box-native-scalar", "box-context-scalar", "sphere-native-256", "sphere-context-256",
            "sphere-native-scalar", "sphere-context-scalar", "box-triangle-scalar", "box-hull-scalar"))
        {
            [Console]::Error.WriteLine("Select one implemented collision-batch collision cell")
            exit 2
        }
    }
    elseif ($QueryCase -cne "global" -or $Precision -ine "Mixed64" -or
        ($geometryCell -eq 0 -and ($QueryOperation -cnotin @("ray-any", "ray-closest", "ray-all") -or
            $QueryCell -cnotin @("mesh-8192-near", "mesh-8192-far", "mesh-8192-local", "box-near", "box-far", "box-local"))))
    {
        [Console]::Error.WriteLine("Global cells require Mixed64 and one implemented operation/cell")
        exit 2
    }
}
if ($QueryCell -cin @("compound-8", "big-128", "big-8192", "nested-compound-8", "nested-big-8192") -and
    -not $PSBoundParameters.ContainsKey("TimeoutSeconds"))
{
    [Console]::Error.WriteLine("Compound hierarchy cells require TimeoutSeconds for bounded producer logs and worker-affinity verification")
    exit 2
}
if (($PSBoundParameters.ContainsKey("QueryRepetitions") -or $PSBoundParameters.ContainsKey("QueryWarmupRepetitions")) -and
    -not $PSBoundParameters.ContainsKey("QueryGroup"))
{
    [Console]::Error.WriteLine("Query repetitions require QueryGroup")
    exit 2
}
foreach ($value in @($QueryRepetitions, $QueryWarmupRepetitions, $WarmupBatches, $MeasuredBatches))
{
    [uint64]$count = 0
    if ($value -cnotmatch '^[0-9]+$' -or -not [uint64]::TryParse($value, [ref]$count) -or $count -lt 1 -or $count -gt 1000000)
    {
        [Console]::Error.WriteLine("Query work counts must be decimal integers in 1..1000000")
        exit 2
    }
}
if ($PSBoundParameters.ContainsKey("QueryGroup"))
{
    [string]$operations = "ray_closest|ray_filtered|ray_all|sweep_closest|sweep_all|geometric_overlap|direct_collision|collision_batch"
    if ($Components -eq "All")
    {
        $operations += "|closest_point|shape_distance|depenetration|overlap_any"
    }
    if ($QueryGroup -cnotmatch "^(convex|compound|mesh|native_custom)\.($operations)$")
    {
        [Console]::Error.WriteLine("QueryGroup must name one existing route.operation available in Components")
        exit 2
    }
}
if ($QueryFamily -cnotin @("all", "ray", "sphere-cast", "aabb-presence") -or
    ($RayPattern -eq "oblique" -and $QueryFamily -cnotin @("all", "ray")))
{
    [Console]::Error.WriteLine("Invalid QueryFamily or oblique selection without rays")
    exit 2
}
if ($Verify -notin @("On", "Off") -or
    ($PSBoundParameters.ContainsKey("Verify") -and $Package -notin @("container", "contact_islands", "pyramid")))
{
    [Console]::Error.WriteLine("Verify requires On or Off and a supported dynamics package")
    exit 2
}
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
    [Diagnostics.Stopwatch]$Clock, [long]$BudgetMilliseconds, [uint64]$AffinityMask = 0)
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
        if ($AffinityMask -eq 0)
        {
            $process = [Diagnostics.Process]::Start($start)
        }
        else
        {
            [Diagnostics.Process]$owner = [Diagnostics.Process]::GetCurrentProcess()
            [IntPtr]$originalAffinity = $owner.ProcessorAffinity
            try
            {
                $owner.ProcessorAffinity = [IntPtr]::new([BitConverter]::ToInt64([BitConverter]::GetBytes($AffinityMask), 0))
                $process = [Diagnostics.Process]::Start($start)
            }
            finally
            {
                try
                {
                    $owner.ProcessorAffinity = $originalAffinity
                }
                finally
                {
                    $owner.Dispose()
                }
            }
            [UIntPtr]$processAffinity = [UIntPtr]::Zero
            [UIntPtr]$systemAffinity = [UIntPtr]::Zero
            if (-not [EntasisBenchmarkProcessAffinity]::GetProcessAffinityMask($process.SafeHandle, [ref]$processAffinity, [ref]$systemAffinity))
            {
                throw [ComponentModel.Win32Exception]::new([Runtime.InteropServices.Marshal]::GetLastWin32Error(), "Cannot query producer affinity for pid=$($process.Id).")
            }
            [uint64]$effectiveAffinity = $processAffinity.ToUInt64()
            [string]$placement = "PROCESS_AFFINITY stage=$Stage requested=$AffinityMask effective=$effectiveAffinity pid=$($process.Id)"
            [byte[]]$placementBytes = [Text.Encoding]::UTF8.GetBytes("$placement`n")
            $stdout.Write($placementBytes, 0, $placementBytes.Length)
            $stdout.Flush()
            Write-Host $placement
            if ($effectiveAffinity -ne $AffinityMask)
            {
                throw "Producer affinity differs from requested mask: requested=$AffinityMask effective=$effectiveAffinity"
            }
        }
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
    Verify = "verify"
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
if ($Package -eq "query_extensions" -and $QueryCase -cne "control")
{
    $resultPackage += "/$QueryCase.$QueryOperation.$QueryCell"
}
if ($Package -eq "query_extensions")
{
    $reportOptions += @("--components", $Components.ToLowerInvariant())
}
foreach ($option in $queryOptionNames.Keys)
{
    if ($PSBoundParameters.ContainsKey($option))
    {
        $nativeOptions += "--$($queryOptionNames[$option])=$($PSBoundParameters[$option])"
        $reportOptions += @("--$($queryOptionNames[$option])", [string]$PSBoundParameters[$option])
    }
}
if ($Package -in @("spatial_query_batch", "spatial_query_trace"))
{
    $nativeOptions += "--ray-pattern=$RayPattern"
    $reportOptions += @("--ray-pattern", $RayPattern)
    if (-not $PSBoundParameters.ContainsKey("WarmupBatches"))
    {
        $reportOptions += @("--warmup-batches", "10")
    }
    if (-not $PSBoundParameters.ContainsKey("MeasuredBatches"))
    {
        $reportOptions += @("--measured-batches", "100")
    }
    if ($Package -eq "spatial_query_trace" -and -not $PSBoundParameters.ContainsKey("QueryFamily"))
    {
        $reportOptions += @("--query-family", "all")
    }
    if ($RayPattern -eq "oblique")
    {
        $resultPackage += "/oblique"
    }
}
if ($Precision -notin @("Float32", "Mixed64"))
{
    [Console]::Error.WriteLine("Precision must be Float32 or Mixed64")
    exit 2
}
if ($Package -eq "large_world_precision")
{
    $cases = @("compact-near", "compact-10km", "compact-1000km", "compact-1gm", "shift-sparse", "shift-stress", "distributed", "crossing", "queries-local", "queries-spanning", "holdout-small", "holdout-mixed")
    if (-not $PSBoundParameters.ContainsKey("SamplingMode") -or
        $PrecisionCandidate -cnotin @("baseline", "origin", "mixed64") -or $PrecisionCase -cnotin $cases -or
        $SamplingMode -cnotin @("batch", "instrumented", "engine-only", "memory"))
    {
        [Console]::Error.WriteLine("Precision workloads require an explicit supported candidate, case and sampling mode")
        exit 2
    }
    $selectedPrecision = if ($PrecisionCandidate -eq "mixed64")
    {
        "Mixed64"
    }
    else
    {
        "Float32"
    }
    if ($PSBoundParameters.ContainsKey("Precision") -and $Precision -ne $selectedPrecision)
    {
        [Console]::Error.WriteLine("Precision conflicts with candidate")
        exit 2
    }
    $Precision = $selectedPrecision
    if ($Record -ne "Off" -or $PSBoundParameters.ContainsKey("Components") -or $PSBoundParameters.ContainsKey("Variant"))
    {
        [Console]::Error.WriteLine("Precision workloads require Record Off and no Components or Variant")
        exit 2
    }
    if ($PrecisionCase -in @("distributed", "queries-spanning") -and $PrecisionCandidate -ne "mixed64")
    {
        [Console]::Error.WriteLine("Distributed precision workloads require mixed64")
        exit 2
    }
    if ($PrecisionCandidate -eq "baseline" -and $PrecisionCase -notin @("compact-near", "compact-10km", "compact-1000km", "compact-1gm"))
    {
        [Console]::Error.WriteLine("Baseline requires a compact precision case")
        exit 2
    }
    if ($PrecisionCase -in @("queries-local", "queries-spanning") -and ($SamplingMode -eq "engine-only" -or $Workers -ne "1"))
    {
        [Console]::Error.WriteLine("Precision queries require worker 1 and an end-to-end mode")
        exit 2
    }
    $nativeOptions += @("--precision-case=$PrecisionCase", "--sampling-mode=$SamplingMode")
    $resultPackage = "$Package/$PrecisionCandidate.$PrecisionCase.$SamplingMode"
    $reportOptions += @("--precision-candidate", $PrecisionCandidate, "--precision-case", $PrecisionCase, "--sampling-mode", $SamplingMode)
}
elseif ($PSBoundParameters.ContainsKey("PrecisionCandidate") -or $PSBoundParameters.ContainsKey("PrecisionCase") -or $PSBoundParameters.ContainsKey("SamplingMode"))
{
    [Console]::Error.WriteLine("Precision workload options require large_world_precision")
    exit 2
}
$reportOptions += @("--precision", $Precision.ToLowerInvariant())
if ($Package -in @("container", "contact_islands", "pyramid"))
{
    if ($Shape -notin @("box", "sphere", "capsule", "cylinder", "hull") -or
        $Variant -cnotmatch '^[A-Za-z0-9][A-Za-z0-9._-]*$')
    {
        throw "New workloads require valid Shape and Variant."
    }
    $nativeOptions += "--verify=$($Verify.ToLowerInvariant())"
    $resultPackage = "$Package\$Variant"
    $reportOptions += @("--variant", $Variant)
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
if ($Precision -eq "Mixed64" -and $Package -ne "large_world_precision")
{
    $resultPackage += "/mixed64"
}
foreach ($option in $workloadOptionNames.Keys)
{
    if ($option -ne "Verify" -and $PSBoundParameters.ContainsKey($option))
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
[uint64]$requestedProcessorAffinity = 0
if ($PSBoundParameters.ContainsKey("ProcessorAffinity"))
{
    if ($null -eq ("EntasisBenchmarkProcessAffinity" -as [type]))
    {
        Add-Type -TypeDefinition @'
using System;
using System.Runtime.InteropServices;
using Microsoft.Win32.SafeHandles;
public static class EntasisBenchmarkProcessAffinity
{
    [DllImport("kernel32.dll", SetLastError = true)]
    [return: MarshalAs(UnmanagedType.Bool)]
    public static extern bool GetProcessAffinityMask(SafeProcessHandle process, out UIntPtr processMask, out UIntPtr systemMask);
}
'@
    }
    if ($ProcessorAffinity -cnotmatch '^[1-9][0-9]*$' -or
        -not [uint64]::TryParse($ProcessorAffinity, [ref]$requestedProcessorAffinity))
    {
        throw "ProcessorAffinity must be a positive decimal UInt64 mask."
    }
    if (-not $PSBoundParameters.ContainsKey("TimeoutSeconds"))
    {
        throw "ProcessorAffinity requires an explicit TimeoutSeconds."
    }
    [uint32]$logicalProcessors = (Get-CimInstance -ClassName Win32_ComputerSystem -Property NumberOfLogicalProcessors -OperationTimeoutSec 5).NumberOfLogicalProcessors
    if ($logicalProcessors -eq 0 -or $logicalProcessors -gt 64 -or [IntPtr]::Size -ne 8)
    {
        throw "ProcessorAffinity supports one 64-bit processor group on systems with 1..64 logical processors."
    }
    [Diagnostics.Process]$owner = [Diagnostics.Process]::GetCurrentProcess()
    try
    {
        [uint64]$eligibleAffinity = [BitConverter]::ToUInt64([BitConverter]::GetBytes($owner.ProcessorAffinity.ToInt64()), 0)
    }
    finally
    {
        $owner.Dispose()
    }
    if (($requestedProcessorAffinity -band $eligibleAffinity) -ne $requestedProcessorAffinity)
    {
        throw "ProcessorAffinity must be a subset of the current eligible processor mask: $eligibleAffinity."
    }
    [int]$selectedProcessors = [System.Numerics.BitOperations]::PopCount($requestedProcessorAffinity)
    foreach ($worker in $workerValues)
    {
        if ($worker -gt $selectedProcessors)
        {
            throw "Worker count $worker exceeds the $selectedProcessors processors selected by ProcessorAffinity."
        }
    }
}

}
catch
{
    [Console]::Error.WriteLine($_.Exception.Message)
    exit 2
}

$binary = Get-BenchmarkBinary $Package $Configuration $Record $Components "Off" $Precision $PrecisionCandidate
$reporter = Get-BenchmarkReporterBinary $Configuration
$buildCommand = "& .\scripts\windows\benchmarks\build_benchmarks.ps1 -Package $Package -Configuration $Configuration -Record $Record"
if ($Package -in $script:BenchmarkComponentPackages)
{
    $buildCommand += " -Components $Components"
}
if (-not (Test-Path -LiteralPath $binary -PathType Leaf))
{
    [Console]::Error.WriteLine("Missing benchmark binary: $binary. Build with: $buildCommand")
    exit 2
}
if (-not (Test-Path -LiteralPath $reporter -PathType Leaf))
{
    [Console]::Error.WriteLine("Missing benchmark reporter: $reporter. Build with: $buildCommand")
    exit 2
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
$nativePending = if ($pending.StartsWith('\\'))
{
    '\\?\UNC\' + $pending.Substring(2)
}
else
{
    '\\?\' + $pending
}
New-Item -ItemType Directory -Force -Path $resultParent | Out-Null
New-Item -ItemType Directory -Path $pending | Out-Null
Write-Host "BENCHMARK_RUN_CONFIG package=$Package configuration=$configurationName components=$($Components.ToLowerInvariant()) pending=$pending"
$workloadClock = [Diagnostics.Stopwatch]::StartNew()
Write-Host "BENCHMARK_PREPARED elapsed_ms=$($preparationClock.ElapsedMilliseconds)"
$budgetMilliseconds = [long]$TimeoutSeconds * 1000
foreach ($worker in $workerValues)
{
    $output = Join-Path $nativePending "workers-$worker.csv"
    for ($run = 1; $run -le $Runs; $run++)
    {
        Write-Host "[benchmark] $Package workers=$worker run=$run/$Runs"
        $childArguments = @("--worker-count=$worker", "--output=$output") + $nativeOptions
        if ($Package -in $script:BenchmarkRecordingPackages)
        {
            $childArguments += "--record=$($Record.ToLowerInvariant())"
            if ($Record -eq "On")
            {
                $recordingOutput = Join-Path $nativePending "workers-$worker-sample-$run.epr"
                $childArguments += @("--recording-output=$recordingOutput", "--compression=$Compression", "--memory-mib=$MemoryMiB")
            }
        }
        if ($PSBoundParameters.ContainsKey("TimeoutSeconds"))
        {
            $exitCode = Invoke-BenchmarkChild $binary $childArguments (Join-Path $pending "workers-$worker-run-$run") "worker=$worker run=$run/$Runs" $workloadClock $budgetMilliseconds $requestedProcessorAffinity
            if ($exitCode -ne 0)
            {
                exit $exitCode
            }
            if ($QueryCell -cin @("compound-8", "big-128", "big-8192", "nested-compound-8", "nested-big-8192"))
            {
                [int[]]$workerOrder = @(18, 16, 20, 22, 24, 26, 28, 30, 0, 2, 4, 6, 8, 10, 12, 14,
                    19, 17, 21, 23, 25, 27, 29, 31, 1, 3, 5, 7, 9, 11, 13, 15)
                [string[]]$records = @([IO.File]::ReadLines((Join-Path $pending "workers-$worker-run-$run.stdout.txt")) |
                    Where-Object { $_.StartsWith("WORKER_AFFINITY ") })
                if ($records.Count -ne [int]$worker)
                {
                    throw "Expected one WORKER_AFFINITY record per active worker"
                }
                $workerIds = [Collections.Generic.HashSet[int]]::new()
                $nativeIds = [Collections.Generic.HashSet[uint32]]::new()
                foreach ($workerAffinityRecord in $records)
                {
                    if ($workerAffinityRecord -cnotmatch '^WORKER_AFFINITY cell=([^ ]+) worker=([0-9]+) requested=([0-9]+) effective=([0-9]+) native_tid=([0-9]+)$')
                    {
                        throw "Invalid worker-affinity record: $workerAffinityRecord"
                    }
                    [int]$workerId = [int]$Matches[2]
                    [uint32]$nativeId = [uint32]$Matches[5]
                    if ($Matches[1] -cne $QueryCell -or $workerId -ge [int]$worker -or $nativeId -eq 0 -or
                        -not $workerIds.Add($workerId) -or -not $nativeIds.Add($nativeId))
                    {
                        throw "Invalid cell, worker or native thread identity: $workerAffinityRecord"
                    }
                    [uint64]$expectedMask = [uint64]1 -shl $workerOrder[$workerId]
                    if ([uint64]$Matches[3] -ne $expectedMask -or [uint64]$Matches[4] -ne $expectedMask)
                    {
                        throw "Worker placement differs from declared LP order: $workerAffinityRecord"
                    }
                }
            }
        }
        else
        {
            Invoke-Checked $binary $childArguments
        }
        if ($Package -in @("spatial_query_batch", "spatial_query_trace"))
        {
            $samples = @(Import-Csv -LiteralPath $output)
            if ($samples.Count -ne $run)
            {
                throw "Ray sample count mismatch in $output"
            }
            foreach ($sample in $samples)
            {
                if ($sample.ray_pattern -cne $RayPattern)
                {
                    throw "Ray pattern mismatch: requested $RayPattern. Rebuild the selected producer. Output: $output"
                }
            }
        }
        if ($Package -in @("spatial_query_batch", "spatial_query_trace", "query_extensions"))
        {
            $samples = @(Import-Csv -LiteralPath $output)
            if ($samples.Count -ne $run)
            {
                throw "Query sample count mismatch in $output"
            }
            foreach ($sample in $samples)
            {
                if ($Package -eq "query_extensions")
                {
                    if ($QueryCase -cne "control")
                    {
                        [string]$expectedHits = "6144"
                        if ($QueryOperation -ceq "ray-all" -and $QueryCell.StartsWith("nested-"))
                        {
                            $expectedHits = "12288"
                        }
                        if ($QueryOperation -cin @("closest-point", "distance", "depenetration"))
                        {
                            $expectedHits = "12288"
                        }
                        [string]$expectedInputs = "1536"
                        [string]$expectedPasses = "8"
                        [string]$expectedQueries = "12288"
                        [int]$expectedWarmup = 128 * [int]$sample.worker_count
                        if ($QueryCase -ceq "collision-batch")
                        {
                            $expectedInputs = "24576"
                            $expectedPasses = "1"
                            $expectedQueries = "24576"
                            $expectedHits = "12288"
                            [int]$batchSize = 256
                            if ($QueryCell -cmatch '-(1|8|32)$')
                            {
                                $batchSize = [int]$Matches[1]
                            }
                            $expectedWarmup = $batchSize * [int]$sample.worker_count
                        }
                        if ($sample.query_case -cne $QueryCase -or $sample.query_operation -cne $QueryOperation -or
                            $sample.query_cell -cne $QueryCell -or $sample.input_count -cne $expectedInputs -or
                            $sample.passes -cne $expectedPasses -or $sample.query_count -cne $expectedQueries -or
                            $sample.warmup_queries -cne [string]$expectedWarmup -or
                            $sample.benchmark_components -cne "all" -or $sample.worker_count -cne [string]$worker -or
                            $sample.precision -cne $Precision.ToLowerInvariant() -or $sample.oracle_queries -cne $expectedInputs -or
                            $sample.hit_count -cne $expectedHits -or
                            $sample.checksum -cne $sample.hit_count)
                        {
                            throw "Selected query cell or counts mismatch in $output"
                        }
                    }
                    elseif ($sample.PSObject.Properties.Name -contains "query_case" -and $sample.query_case -cne "control")
                    {
                        throw "Unexpected selected query case in $output"
                    }
                    elseif ($QueryGroup -ne "")
                    {
                        if ($sample.query_group -cne $QueryGroup -or $sample.query_repetitions -ne [string][int]$QueryRepetitions -or
                            $sample.query_warmup_repetitions -ne [string][int]$QueryWarmupRepetitions -or
                            $sample.query_count_per_operation -ne [string]([uint64]8192 * [uint64]$QueryRepetitions))
                        {
                            throw "Focused query settings mismatch in $output"
                        }
                    }
                    elseif ($sample.PSObject.Properties.Name -contains "query_group")
                    {
                        throw "Unexpected focused query result in $output"
                    }
                }
                elseif ($sample.warmup_batches -ne [string][int]$WarmupBatches -or $sample.measured_batches -ne [string][int]$MeasuredBatches)
                {
                    throw "Spatial work count mismatch in $output"
                }
                if ($Package -eq "spatial_query_trace")
                {
                    [string]$emittedFamily = "all"
                    if ($sample.PSObject.Properties.Name -contains "query_family")
                    {
                        $emittedFamily = $sample.query_family
                    }
                    if ($emittedFamily -cne $QueryFamily)
                    {
                        throw "Spatial query family mismatch in $output"
                    }
                }
            }
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
if ($Package -in @("spatial_query_batch", "spatial_query_trace"))
{
    $canonicalCommand += " -RayPattern $RayPattern"
}
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
foreach ($option in @($workloadOptionNames.Keys) + @("Variant", "Precision", "PrecisionCandidate", "PrecisionCase", "SamplingMode", "ProcessorAffinity") + @($queryOptionNames.Keys))
{
    if ($PSBoundParameters.ContainsKey($option))
    {
        $literal = ([string]$PSBoundParameters[$option]).Replace("'", "''")
        $canonicalCommand += " -$option '$literal'"
    }
}
$reportArguments = @(
    "--source", $nativePending,
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
