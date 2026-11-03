[CmdletBinding(PositionalBinding = $false)]
param(
    [Parameter(Mandatory = $true)]
    [ValidateSet(
        "awakening_duplicate_sets", "container",
        "contact_churn_grid_4k", "noncontact_constraint_mix", "noncontact_fallback_smoke",
        "shape_mixed_bounds", "spatial_query_trace", "spatial_query_batch",
        "custom_extensions", "query_extensions", "world_lifecycle",
        "contact_islands", "ragdoll_stair_tumble", "pyramid", "large_world_precision", "solver_joint_angular", "solver_fixed_work", "solver_contact_work", "simulation_stage_work"
    )]
    [string]$Package,
    [Parameter(Mandatory = $true)]
    [ValidateRange(1, 2147483647)]
    [int]$Runs,
    [Parameter(Mandatory = $true)]
    [ValidateNotNullOrEmpty()]
    [string]$Workers,
    [string]$ProcessorAffinity,
    [ValidateNotNullOrEmpty()]
    [string]$BaselineRoot,
    [string]$ContactInput,
    [ValidateSet("Stop", "Continue")]
    [string]$FailurePolicy = "Stop",
    [ValidateSet("Development", "Release")]
    [string]$Configuration = "Release",
    [ValidateSet("Caller", "Mailbox")]
    [string]$QueryExecution = "Caller",
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
    [ValidateSet("conserving", "convex", "cached-convex", "nonconvex", "joints", "nonconserving", "pose", "constrained-final", "free-final", "kinematic", IgnoreCase = $false)][string]$Case,
    [ValidateSet("tree-update", "pair-traversal", "narrow-phase", "collision", IgnoreCase = $false)][string]$Stage,
    [ValidateSet("container", "islands", IgnoreCase = $false)][string]$Fixture,
    [ValidateSet("steady", "churn", IgnoreCase = $false)][string]$Motion,
    [ValidateRange(8, 1000000)][int]$BodyCount,
    [ValidateSet("enabled", "disabled", IgnoreCase = $false)][string]$UnconstrainedSubsteps,
    [ValidateSet("full", "partial", "sparse", "isotropic", IgnoreCase = $false)][string]$Layout,
    [ValidateRange(1, 100000)][int]$Passes = 512,
    [ValidateRange(8, 65536)][int]$Bundles = 4096,
    [ValidateRange(0, 1000)][int]$WarmupPasses = 4,
    [string]$Scenario,
    [string]$AngularMode,
    [ValidateRange(1, 16384)][int]$Chains = 128,
    [ValidateRange(1, 128)][int]$Links = 8,
    [ValidateRange(1, 10000)][int]$MassRatio = 1,
    [ValidatePattern('^[1-9][0-9]{0,9}$')]
    [ValidateScript({ [long]$_ -le [int]::MaxValue })]
    [string]$TimeoutSeconds
)
$preparationClock = [Diagnostics.Stopwatch]::StartNew()
. (Join-Path $PSScriptRoot "../lib/common.ps1")
. (Join-Path $PSScriptRoot "build_support.ps1")
. (Join-Path $PSScriptRoot "process_support.ps1")
if ($FailurePolicy -eq "Continue" -and -not $PSBoundParameters.ContainsKey("TimeoutSeconds"))
{
    [Console]::Error.WriteLine("FailurePolicy Continue requires an explicit TimeoutSeconds")
    exit 2
}
if ($PSBoundParameters.ContainsKey("QueryExecution") -and $Package -ne "spatial_query_batch")
{
    [Console]::Error.WriteLine("QueryExecution requires spatial_query_batch")
    exit 2
}
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
[hashtable]$requestedMetadata = @{}
foreach ($option in $workloadOptionNames.Keys)
{
    $requestedMetadata[$option] = $workloadOptionNames[$option]
}
$requestedMetadata.Case = "case"
$requestedMetadata.Layout = "layout"
$requestedMetadata.AngularMode = "angular-mode"
$requestedMetadata.Scenario = "scenario"
$requestedMetadata.Chains = "chains"
$requestedMetadata.Links = "links"
$requestedMetadata.MassRatio = "mass-ratio"
$requestedMetadata.Passes = "passes"
$requestedMetadata.Bundles = "bundles"
$requestedMetadata.WarmupPasses = "warmup-passes"
$requestedMetadata.Stage = "stage"
$requestedMetadata.Fixture = "fixture"
$requestedMetadata.Motion = "motion"
$requestedMetadata.BodyCount = "bodies"
$requestedMetadata.UnconstrainedSubsteps = "unconstrained-substeps"
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
if ($Package -eq "spatial_query_batch")
{
    $reportOptions += @("--query-execution", $QueryExecution.ToLowerInvariant())
    if ($QueryExecution -eq "Mailbox")
    {
        $resultPackage += "/mailbox"
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
elseif ($Package -eq "solver_contact_work")
{
    if ($Precision -ne "Float32" -or $Record -ne "Off" -or
        -not $PSBoundParameters.ContainsKey("ProcessorAffinity") -or
        -not $PSBoundParameters.ContainsKey("TimeoutSeconds") -or
        $Workers -cnotmatch '^[1-9][0-9]*$' -or
        -not [IO.Path]::IsPathFullyQualified($ContactInput) -or
        -not (Test-Path -LiteralPath $ContactInput -PathType Leaf))
    {
        throw "Contact work requires one worker, Float32, Record Off, ProcessorAffinity, TimeoutSeconds and an existing absolute ContactInput file"
    }
    foreach ($option in @($workloadOptionNames.Keys) + @("Variant", "Scenario", "AngularMode", "Chains", "Links", "MassRatio", "Case", "Layout", "Passes", "Bundles", "WarmupPasses"))
    {
        if ($PSBoundParameters.ContainsKey($option))
        {
            throw "Option $option does not apply to contact work"
        }
    }
    $ContactInput = [IO.Path]::GetFullPath($ContactInput)
    $nativeOptions += @("--input=$ContactInput", "--processor-affinity=$ProcessorAffinity")
}
elseif ($Package -eq "simulation_stage_work")
{
    if ([string]::IsNullOrEmpty($Stage) -or [string]::IsNullOrEmpty($Fixture) -or [string]::IsNullOrEmpty($Motion) -or
        $Precision -ne "Float32" -or $Record -ne "Off" -or $Workers -cnotmatch '^[1-9][0-9]*$' -or
        -not $PSBoundParameters.ContainsKey("ProcessorAffinity") -or -not $PSBoundParameters.ContainsKey("TimeoutSeconds"))
    {
        throw "Stage work requires Stage, Fixture, Motion, Float32, one worker, ProcessorAffinity, TimeoutSeconds and Record Off"
    }
    foreach ($option in @($workloadOptionNames.Keys) + @("Variant", "Case", "Layout", "Passes", "Bundles", "WarmupPasses", "Scenario", "AngularMode", "Chains", "Links", "MassRatio", "UnconstrainedSubsteps", "BodyCount"))
    {
        if ($PSBoundParameters.ContainsKey($option))
        {
            throw "Option $option does not apply to stage work"
        }
    }
    $nativeOptions += @("--stage=$Stage", "--fixture=$Fixture", "--motion=$Motion")
    $resultPackage = "$Package/$Stage.$Fixture.$Motion"
}
elseif ($Package -eq "shape_mixed_bounds")
{
    foreach ($option in @($workloadOptionNames.Keys) + @("Variant"))
    {
        if ($PSBoundParameters.ContainsKey($option))
        {
            throw "Option $option does not apply to bounds work"
        }
    }
    if ($PSBoundParameters.ContainsKey("BodyCount"))
    {
        $nativeOptions += "--body-count=$BodyCount"
        $resultPackage = "$Package/bodies-$BodyCount"
    }
}
elseif ($Package -eq "solver_fixed_work")
{
    if ([string]::IsNullOrEmpty($Case) -or $Record -ne "Off" -or
        ($Case -ne "conserving" -and $Precision -ne "Float32"))
    {
        throw "Fixed work requires a supported Case, Record Off and Float32 for new cases"
    }
    foreach ($option in @($workloadOptionNames.Keys) + @("Variant"))
    {
        if ($PSBoundParameters.ContainsKey($option) -and -not ($Case -eq "cached-convex" -and $option -in @("VelocityIterations", "Substeps")))
        {
            throw "Option $option does not apply to fixed work"
        }
    }
    if ($Case -ne "conserving")
    {
        if (-not $PSBoundParameters.ContainsKey("Passes"))
        {
            $Passes = 100
        }
        if (-not $PSBoundParameters.ContainsKey("WarmupPasses"))
        {
            $WarmupPasses = 10
        }
        if (-not $PSBoundParameters.ContainsKey("Bundles"))
        {
            $Bundles = 1250
        }
    }
    $nativeOptions += @("--case=$Case", "--passes=$Passes", "--warmup-passes=$WarmupPasses")
    if ($Case -in @("conserving", "nonconserving", "pose", "constrained-final", "free-final", "kinematic"))
    {
        if ($Layout -notin @("full", "partial", "sparse", "isotropic"))
        {
            throw "Integration cases require a supported Layout"
        }
        $nativeOptions += "--layout=$Layout"
    }
    elseif ($PSBoundParameters.ContainsKey("Layout"))
    {
        throw "Layout does not apply to native graph cases"
    }
    if ($Case -in @("pose", "constrained-final", "free-final") -and $Layout -notin @("full", "isotropic"))
    {
        throw "Final pose cases process the full real population and require full or isotropic Layout"
    }
    if ($Case -in @("conserving", "nonconserving", "pose", "constrained-final", "free-final", "joints"))
    {
        if ($AngularMode -cnotin @("nonconserving", "momentum", "gyro") -or
            ($Case -in @("conserving", "joints") -and $AngularMode -eq "nonconserving") -or
            ($Case -eq "nonconserving" -and $AngularMode -ne "nonconserving"))
        {
            throw "Select an AngularMode applicable to this fixed case"
        }
        $nativeOptions += "--angular-mode=$AngularMode"
    }
    elseif ($PSBoundParameters.ContainsKey("AngularMode"))
    {
        throw "AngularMode does not apply to this fixed case"
    }
    if ($Case -eq "joints")
    {
        if ($Scenario -cnotin @("hinge", "weld", "mixed"))
        {
            throw "Fixed joints require Scenario hinge, weld or mixed"
        }
        if ($PSBoundParameters.ContainsKey("Bundles"))
        {
            throw "Joints uses Chains/Links rather than Bundles"
        }
        if (-not $PSBoundParameters.ContainsKey("Chains"))
        {
            $Chains = 1000
        }
        if (-not $PSBoundParameters.ContainsKey("Links"))
        {
            $Links = 9
        }
        if ($Chains * ($Links + 1) -gt 1000000)
        {
            throw "Joint population exceeds 1000000"
        }
        $nativeOptions += @("--scenario=$Scenario", "--chains=$Chains", "--links=$Links", "--mass-ratio=$MassRatio")
    }
    else
    {
        foreach ($option in @("Scenario", "Chains", "Links", "MassRatio"))
        {
            if ($PSBoundParameters.ContainsKey($option))
            {
                throw "$option requires fixed joints"
            }
        }
        $nativeOptions += "--bundles=$Bundles"
    }
    if ($Case -eq "cached-convex")
    {
        if (-not $PSBoundParameters.ContainsKey("VelocityIterations"))
        {
            $VelocityIterations = "2"
        }
        if (-not $PSBoundParameters.ContainsKey("Substeps"))
        {
            $Substeps = "2"
        }
        if ("$VelocityIterations/$Substeps" -cnotin @("2/2", "4/1"))
        {
            throw "Cached convex requires 2x2 or 4x1"
        }
        $nativeOptions += @("--velocity-iterations=$VelocityIterations", "--substeps=$Substeps")
    }
    if ($Case -eq "free-final")
    {
        if ([string]::IsNullOrEmpty($UnconstrainedSubsteps))
        {
            throw "Free final requires UnconstrainedSubsteps"
        }
        $nativeOptions += "--unconstrained-substeps=$UnconstrainedSubsteps"
    }
    elseif ($PSBoundParameters.ContainsKey("UnconstrainedSubsteps"))
    {
        throw "UnconstrainedSubsteps requires free-final"
    }
    $resultPackage = "$Package/$Case"
}
elseif ($Package -eq "solver_joint_angular")
{
    if ($Scenario -cnotin @("hinge", "weld", "mixed") -or $AngularMode -cnotin @("momentum", "gyro") -or
        $Chains * ($Links + 1) -gt 1000000 -or $Record -ne "Off")
    {
        [Console]::Error.WriteLine("Angular requires Scenario hinge/weld/mixed, AngularMode momentum/gyro and its bounded population")
        exit 2
    }
    [hashtable]$fixedSettings = @{ WarmupSteps = "0"; VelocityIterations = "1"; Substeps = "4"; TimestepHz = "60"; Sleep = "disabled" }
    foreach ($option in @($workloadOptionNames.Keys) + @("Variant"))
    {
        if (-not $PSBoundParameters.ContainsKey($option))
        {
            continue
        }
        if ($option -eq "Steps")
        {
            if ($Steps -cnotmatch '^[1-9][0-9]{0,5}$' -or [long]$Steps -gt 100000)
            {
                [Console]::Error.WriteLine("Angular Steps must be 1..100000")
                exit 2
            }
        }
        elseif (-not $fixedSettings.ContainsKey($option) -or [string]$PSBoundParameters[$option] -cne $fixedSettings[$option])
        {
            [Console]::Error.WriteLine("Angular option $option conflicts with its fixed workload")
            exit 2
        }
    }
    $nativeOptions += @("--scenario=$Scenario", "--angular-mode=$AngularMode", "--chains=$Chains", "--links=$Links", "--mass-ratio=$MassRatio")
    $resultPackage = "$Package/$Scenario.$AngularMode"
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
foreach ($option in @("Stage", "Fixture", "Motion"))
{
    if ($Package -ne "simulation_stage_work" -and $PSBoundParameters.ContainsKey($option))
    {
        throw "$option requires simulation_stage_work"
    }
}
if ($Package -ne "shape_mixed_bounds" -and $PSBoundParameters.ContainsKey("BodyCount"))
{
    throw "BodyCount requires shape_mixed_bounds"
}
if ($Package -ne "solver_fixed_work" -and $PSBoundParameters.ContainsKey("UnconstrainedSubsteps"))
{
    throw "UnconstrainedSubsteps requires solver_fixed_work/free-final"
}
if ($Package -ne "solver_contact_work" -and $PSBoundParameters.ContainsKey("ContactInput"))
{
    throw "ContactInput requires solver_contact_work"
}
if ($Package -notin @("solver_joint_angular", "solver_fixed_work"))
{
    foreach ($option in @("Scenario", "AngularMode", "Chains", "Links", "MassRatio"))
    {
        if ($PSBoundParameters.ContainsKey($option))
        {
            [Console]::Error.WriteLine("Option $option requires solver_joint_angular")
            exit 2
        }
    }
}
if ($Package -ne "solver_fixed_work")
{
    foreach ($option in @("Case", "Layout", "Passes", "Bundles", "WarmupPasses"))
    {
        if ($PSBoundParameters.ContainsKey($option))
        {
            [Console]::Error.WriteLine("Option $option requires solver_fixed_work")
            exit 2
        }
    }
}
if ($Precision -eq "Mixed64" -and $Package -ne "large_world_precision")
{
    $resultPackage += "/mixed64"
}
foreach ($option in $workloadOptionNames.Keys)
{
    if ($option -ne "Verify" -and $PSBoundParameters.ContainsKey($option) -and
        -not ($Package -eq "solver_fixed_work" -and $Case -eq "cached-convex" -and $option -in @("VelocityIterations", "Substeps")))
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
    Initialize-BenchmarkProcessAffinity
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

$binary = Get-BenchmarkBinary $Package $Configuration $Record $Components "Off" $Precision $PrecisionCandidate $QueryExecution $Case
$reporter = Get-BenchmarkReporterBinary $Configuration
$buildCommand = "& .\scripts\windows\benchmarks\build_benchmarks.ps1 -Package $Package -Configuration $Configuration -Record $Record"
if ($Package -in $script:BenchmarkComponentPackages)
{
    $buildCommand += " -Components $Components"
}
if ($Package -eq "solver_fixed_work")
{
    $buildCommand += " -Case $Case -Precision $Precision"
}
if ($Package -eq "spatial_query_batch")
{
    $buildCommand += " -QueryExecution $QueryExecution -Precision $Precision"
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
[string]$resultLayout = "Single"
[string]$baselineBinary = ""
if ($PSBoundParameters.ContainsKey("BaselineRoot"))
{
    if ($Package -notin @("container", "contact_islands", "pyramid", "solver_joint_angular", "solver_fixed_work", "solver_contact_work", "simulation_stage_work", "shape_mixed_bounds", "query_extensions", "spatial_query_batch", "spatial_query_trace", "world_lifecycle", "awakening_duplicate_sets", "contact_churn_grid_4k", "noncontact_constraint_mix", "noncontact_fallback_smoke") -or $Record -ne "Off" -or
        -not $PSBoundParameters.ContainsKey("TimeoutSeconds") -or
        -not [IO.Path]::IsPathFullyQualified($BaselineRoot) -or
        -not (Test-Path -LiteralPath $BaselineRoot -PathType Container))
    {
        [Console]::Error.WriteLine("BaselineRoot requires an existing absolute root, a supported paired package, Record Off and TimeoutSeconds")
        exit 2
    }
    $BaselineRoot = [IO.Path]::GetFullPath($BaselineRoot)
    $baselineBinary = Join-Path $BaselineRoot ([IO.Path]::GetRelativePath($script:Root, $binary))
    if (-not (Test-Path -LiteralPath $baselineBinary -PathType Leaf))
    {
        [Console]::Error.WriteLine("Missing prepared baseline artifact: $baselineBinary")
        exit 2
    }
    $resultLayout = "Paired"
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
if ($resultLayout -eq "Paired")
{
    New-Item -ItemType Directory -Path (Join-Path $pending "baseline"), (Join-Path $pending "candidate") | Out-Null
}
Write-Host "BENCHMARK_RUN_CONFIG package=$Package configuration=$configurationName components=$($Components.ToLowerInvariant()) failure_policy=$($FailurePolicy.ToLowerInvariant()) pending=$pending"
$workloadClock = [Diagnostics.Stopwatch]::StartNew()
Write-Host "BENCHMARK_PREPARED elapsed_ms=$($preparationClock.ElapsedMilliseconds)"
$budgetMilliseconds = [long]$TimeoutSeconds * 1000
[int]$firstFailureExitCode = 0
[string]$cohortState = "Complete"
foreach ($worker in $workerValues)
{
    for ($run = 1; $run -le $Runs; $run++)
    {
        [string[]]$arms = @("candidate")
        if ($resultLayout -eq "Paired")
        {
            $arms = if ($run % 2 -eq 1)
            {
                @("baseline", "candidate")
            }
            else
            {
                @("candidate", "baseline")
            }
        }
        foreach ($arm in $arms)
        {
            [string]$armPending = if ($resultLayout -eq "Paired")
            {
                Join-Path $pending $arm
            }
            else
            {
                $pending
            }
            [string]$nativeArm = if ($resultLayout -eq "Paired")
            {
                Join-Path $nativePending $arm
            }
            else
            {
                $nativePending
            }
            [string]$armBinary = if ($arm -eq "baseline")
            {
                $baselineBinary
            }
            else
            {
                $binary
            }
            [string]$output = Join-Path $nativeArm "workers-$worker.csv"
            Write-Host "[benchmark] $Package arm=$arm workers=$worker run=$run/$Runs"
            $childArguments = @("--worker-count=$worker", "--output=$output") + $nativeOptions
            if ($Package -in $script:BenchmarkRecordingPackages)
            {
                $childArguments += "--record=$($Record.ToLowerInvariant())"
                if ($Record -eq "On")
                {
                    $recordingOutput = Join-Path $nativeArm "workers-$worker-sample-$run.epr"
                    $childArguments += @("--recording-output=$recordingOutput", "--compression=$Compression", "--memory-mib=$MemoryMiB")
                }
            }
            [int]$exitCode = 0
            try
            {
                if ($PSBoundParameters.ContainsKey("TimeoutSeconds"))
                {
                    $exitCode = Invoke-BenchmarkChild $armBinary $childArguments (Join-Path $armPending "workers-$worker-run-$run") "worker=$worker run=$run/$Runs" $workloadClock $budgetMilliseconds $requestedProcessorAffinity
                    if ($exitCode -ne 0)
                    {
                        if ($FailurePolicy -eq "Stop")
                        {
                            exit $exitCode
                        }
                        if ($firstFailureExitCode -eq 0)
                        {
                            $firstFailureExitCode = $exitCode
                        }
                        if ($exitCode -eq 124 -and $workloadClock.ElapsedMilliseconds -ge $budgetMilliseconds)
                        {
                            [Console]::Error.WriteLine("BENCHMARK_FAILED status=deadline result=$pending exit_code=124")
                            exit 124
                        }
                    }
                    if ($QueryCell -cin @("compound-8", "big-128", "big-8192", "nested-compound-8", "nested-big-8192"))
                    {
                        [int[]]$workerOrder = @(18, 16, 20, 22, 24, 26, 28, 30, 0, 2, 4, 6, 8, 10, 12, 14,
                            19, 17, 21, 23, 25, 27, 29, 31, 1, 3, 5, 7, 9, 11, 13, 15)
                        [string[]]$records = @([IO.File]::ReadLines((Join-Path $armPending "workers-$worker-run-$run.stdout.txt")) |
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
                    Invoke-Checked $armBinary $childArguments
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
                        if ($Package -eq "spatial_query_batch" -and
                            ($sample.query_execution -cne $QueryExecution.ToLowerInvariant() -or
                            $sample.batch_size -ne "256"))
                        {
                            throw "Query execution/submission identity mismatch. Output: $output"
                        }
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
                if ($Package -eq "solver_contact_work")
                {
                    [object[]]$contactSamples = @(Import-Csv -LiteralPath $output)
                    if ($contactSamples.Count -ne $run -or $contactSamples[-1].contact_input -cne $ContactInput -or
                        $contactSamples[-1].processor_affinity -cne $ProcessorAffinity -or
                        $contactSamples[-1].worker_count -cne [string]$worker)
                    {
                        throw "Contact replay input, affinity or append count differs in $output"
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
                if ($resultLayout -eq "Paired")
                {
                    [object[]]$samples = @(Import-Csv -LiteralPath $output)
                    if ($samples.Count -ne $run -or $samples[-1].worker_count -cne [string]$worker)
                    {
                        throw "Paired append count or worker mismatch in $output"
                    }
                    [Collections.Generic.Dictionary[string,string]]$emitted = [Collections.Generic.Dictionary[string,string]]::new()
                    foreach ($field in $samples[-1].benchmark_parameters.Split(';'))
                    {
                        [string[]]$pair = $field.Split('=', 2)
                        if ($pair.Length -ne 2 -or -not $emitted.TryAdd($pair[0], $pair[1]))
                        {
                            throw "Invalid benchmark parameter metadata in $output"
                        }
                    }
                    foreach ($option in $requestedMetadata.Keys)
                    {
                    if (-not $PSBoundParameters.ContainsKey($option))
                    {
                        continue
                    }
                        [string]$key = $requestedMetadata[$option]
                    if (-not $emitted.ContainsKey($key))
                    {
                        throw "Missing requested $key in $output"
                    }
                        [string[]]$expected = ([string]$PSBoundParameters[$option]).Split(',')
                        [string[]]$actual = $emitted[$key].Split(',')
                    if ($expected.Length -ne $actual.Length)
                    {
                        throw "Requested $key differs in $output"
                    }
                        for ([int]$component = 0; $component -lt $expected.Length; $component++)
                        {
                            [double]$expectedNumber = 0
                            [double]$actualNumber = 0
                            if ([double]::TryParse($expected[$component], [Globalization.NumberStyles]::Float, [Globalization.CultureInfo]::InvariantCulture, [ref]$expectedNumber))
                            {
                                if (-not [double]::TryParse($actual[$component], [Globalization.NumberStyles]::Float, [Globalization.CultureInfo]::InvariantCulture, [ref]$actualNumber) -or
                                    [single]$expectedNumber -ne [single]$actualNumber)
                                {
                                    throw "Requested $key differs in $output"
                                }
                            }
                        elseif ($expected[$component] -cne $actual[$component])
                        {
                            throw "Requested $key differs in $output"
                        }
                        }
                    }
                }

                if ($exitCode -ne 0)
                {
                    [object[]]$failedSamples = @(Import-Csv -LiteralPath $output)
                    if ($exitCode -eq 2 -and $failedSamples.Count -eq $run -and
                        (($Package -in @("container", "contact_islands", "pyramid") -and $failedSamples[-1].physical_status -ceq "fail") -or
                         ($Package -in @("solver_contact_work", "solver_fixed_work", "simulation_stage_work") -and $failedSamples[-1].physics_status -ceq "fail")))
                    {
                        if ($cohortState -eq "Complete")
                        {
                            $cohortState = "PhysicalFailed"
                        }
                    }
                    else
                    {
                        $cohortState = "Incomplete"
                    }
                }
            }
            catch
            {
                if ($FailurePolicy -eq "Stop")
                {
                    throw
                }
                if ($firstFailureExitCode -eq 0)
                {
                    $firstFailureExitCode = if ($exitCode -ne 0) { $exitCode } else { 1 }
                }
                $cohortState = "Incomplete"
                [Console]::Error.WriteLine("BENCHMARK_CELL_FAILED arm=$arm workers=$worker run=$run/$Runs exit_code=$exitCode error=$($_.Exception.Message)")
            }
        }
    }
}
if ($cohortState -eq "Incomplete")
{
    [Console]::Error.WriteLine("BENCHMARK_FAILED status=incomplete result=$pending exit_code=$firstFailureExitCode report=unavailable")
    exit $firstFailureExitCode
}
$canonicalCommand = "& .\scripts\windows\benchmarks\run_benchmark.ps1 -Package $Package -Runs $Runs -Workers '$Workers' -Configuration $Configuration"
if ($Package -in @("spatial_query_batch", "spatial_query_trace"))
{
    $canonicalCommand += " -RayPattern $RayPattern"
    if ($Package -eq "spatial_query_batch")
    {
        $canonicalCommand += " -QueryExecution $QueryExecution"
    }
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
foreach ($option in @($workloadOptionNames.Keys) + @("Variant", "Precision", "PrecisionCandidate", "PrecisionCase", "SamplingMode", "ProcessorAffinity", "BaselineRoot", "FailurePolicy", "Scenario", "AngularMode", "Chains", "Links", "MassRatio", "Case", "Layout", "Passes", "Bundles", "WarmupPasses", "ContactInput", "Stage", "Fixture", "Motion", "BodyCount", "UnconstrainedSubsteps") + @($queryOptionNames.Keys))
{
    if ($PSBoundParameters.ContainsKey($option))
    {
        $literal = ([string]$PSBoundParameters[$option]).Replace("'", "''")
        $canonicalCommand += " -$option '$literal'"
    }
}
$reportSource = if ($resultLayout -eq "Paired")
{
    Join-Path $nativePending "candidate"
}
else
{
    $nativePending
}
$reportArguments = @(
    "--source", $reportSource,
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
if ($resultLayout -eq "Paired")
{
    $reportArguments += @("--baseline-source", (Join-Path $nativePending "baseline"))
}
Write-Host "BENCHMARK_STAGE stage=report workload_ms=$($workloadClock.ElapsedMilliseconds)"
$reportClock = [Diagnostics.Stopwatch]::StartNew()
if ($PSBoundParameters.ContainsKey("TimeoutSeconds"))
{
    $exitCode = Invoke-BenchmarkChild $reporter $reportArguments (Join-Path $pending "reporter") "reporter" $workloadClock $budgetMilliseconds
    if ($exitCode -ne 0)
    {
        if ($FailurePolicy -eq "Continue")
        {
            if ($firstFailureExitCode -eq 0 -or $exitCode -eq 124)
            {
                $firstFailureExitCode = $exitCode
            }
            [Console]::Error.WriteLine("BENCHMARK_FAILED status=report_failed result=$pending reporter_exit_code=$exitCode exit_code=$firstFailureExitCode")
            exit $firstFailureExitCode
        }
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
if ($cohortState -eq "PhysicalFailed")
{
    $failedReport = if ($resultLayout -eq "Paired")
    {
        Join-Path $pending "candidate/README.md"
    }
    else
    {
        Join-Path $pending "README.md"
    }
    [Console]::Error.WriteLine("BENCHMARK_FAILED status=physical_failed result=$pending report=$failedReport exit_code=$firstFailureExitCode")
    exit $firstFailureExitCode
}
Complete-BenchmarkResultCommit $pending $current $resultLayout
$reportPath = if ($resultLayout -eq "Paired")
{
    Join-Path $current "candidate/README.md"
}
else
{
    Join-Path $current "README.md"
}
Write-Host "BENCHMARK_OK result=$current report=$reportPath"
