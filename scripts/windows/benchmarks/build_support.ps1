function Get-BenchmarkBinary([string]$Package, [string]$Configuration = "Release", [string]$Record = "Off", [string]$Components = "All", [string]$Symbols = "Off", [string]$Precision = "Float32", [string]$PrecisionCandidate = "", [string]$QueryExecution = "Caller")
{
    if ($Package -eq "large_world_precision")
    {
        return (Join-Path $script:Root "build/benchmarks/large_world_precision/$($Configuration.ToLowerInvariant())/$PrecisionCandidate/entasis_large_world_precision.exe")
    }
    if ($Package -eq "spatial_query_batch" -and $QueryExecution -eq "Mailbox")
    {
        $Package += "_mailbox"
    }
    if ($Precision -eq "Mixed64")
    {
        return (Join-Path $script:Root "build/benchmarks/mixed64/$($Configuration.ToLowerInvariant())/entasis_$Package.exe")
    }
    if ($Package -in $script:BenchmarkComponentPackages)
    {
        $Package += "_$($Components.ToLowerInvariant())"
    }
    if ($Record -eq "On")
    {
        return (Join-Path $script:Root "build\benchmarks\recorded\$($Configuration.ToLowerInvariant())\$Package.exe")
    }
    if ($Symbols -eq "On")
    {
        return (Join-Path $script:Root "build/benchmarks/symbols/release/entasis_$Package.exe")
    }
    switch ($Configuration)
    {
        "Development"
        {
            return (Join-Path $script:Root "build\benchmarks\development\$Package.exe")
        }
        "Release"
        {
            return (Join-Path $script:Root "build\benchmarks\entasis_$Package.exe")
        }
        default
        {
            throw "Unknown build configuration: $Configuration"
        }
    }
}

function Get-BenchmarkReporterBinary([string]$Configuration)
{
    $configurationName = $Configuration.ToLowerInvariant()
    return (Join-Path $script:Root "build\tools\benchmark-report\$configurationName\benchmark_report.exe")
}

function Publish-BenchmarkSymbols([string]$Staging, [string]$Output)
{
    [string[]]$destinations = @($Output, [IO.Path]::ChangeExtension($Output, '.pdb'))
    foreach ($destination in $destinations)
    {
        [string]$candidate = Join-Path $Staging ([IO.Path]::GetFileName($destination))
        if (-not (Test-Path -LiteralPath $candidate -PathType Leaf) -or (Get-Item -LiteralPath $candidate).Length -eq 0)
        {
            throw "Symbol build output is missing or empty: $candidate"
        }
    }
    [string]$previous = Join-Path $Staging 'previous'
    New-Item -ItemType Directory -Path $previous | Out-Null
    [string]$publication = 'SavingPrevious'
    try
    {
        foreach ($destination in $destinations)
        {
            if (Test-Path -LiteralPath $destination)
            {
                Move-Item -LiteralPath $destination -Destination $previous
            }
        }
        $publication = 'Publishing'
        foreach ($destination in $destinations)
        {
            Move-Item -LiteralPath (Join-Path $Staging ([IO.Path]::GetFileName($destination))) -Destination $destination
        }
        $publication = 'Complete'
    }
    catch
    {
        if ($publication -eq 'Publishing')
        {
            foreach ($destination in $destinations)
            {
                if (Test-Path -LiteralPath $destination)
                {
                    Remove-Item -LiteralPath $destination
                }
            }
        }
        foreach ($destination in $destinations)
        {
            [string]$saved = Join-Path $previous ([IO.Path]::GetFileName($destination))
            if (Test-Path -LiteralPath $saved)
            {
                Move-Item -LiteralPath $saved -Destination $destination
            }
        }
        throw
    }
    Remove-Item -LiteralPath $Staging -Recurse
}

function Invoke-BenchmarkBuild([string]$OdinExe, [string]$Package, [string]$Output, [string[]]$Arguments, [string]$Emit = 'Executable', [string]$Symbols = 'Off')
{
    [Diagnostics.Stopwatch]$clock = [Diagnostics.Stopwatch]::StartNew()
    New-Item -ItemType Directory -Force -Path (Split-Path -Parent $Output) | Out-Null
    [string]$temporary = [IO.Path]::ChangeExtension($Output, '.building.exe')
    [string]$staging = ''
    if ($Symbols -eq 'On')
    {
        $staging = Join-Path (Split-Path -Parent $Output) ('.build-' + $Package + '-' + [Guid]::NewGuid().ToString('N'))
        New-Item -ItemType Directory -Path $staging | Out-Null
        $temporary = Join-Path $staging ([IO.Path]::GetFileName($Output))
        [string]$pdb = [IO.Path]::ChangeExtension($Output, '.pdb')
        for ($index = 0; $index -lt $Arguments.Count; $index++)
        {
            if ($Arguments[$index] -eq "-out:$Output")
            {
                $Arguments[$index] = "-out:$temporary"
            }
            elseif ($Arguments[$index] -eq "-pdb-name:$pdb")
            {
                $Arguments[$index] = "-pdb-name:$([IO.Path]::ChangeExtension($temporary, '.pdb'))"
            }
        }
        Write-Host "BENCHMARK_SYMBOL_STAGE directory=$staging"
    }
    elseif ($Emit -eq 'Executable')
    {
        foreach ($selected in @($temporary))
        {
            if (Test-Path -LiteralPath $selected)
            {
                Move-Item -LiteralPath $selected -Destination "$selected.$([DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffffffZ')).failed"
            }
        }
        for ($index = 0; $index -lt $Arguments.Count; $index++)
        {
            if ($Arguments[$index] -eq "-out:$Output")
            {
                $Arguments[$index] = "-out:$temporary"
            }
        }
    }
    Write-Host "BENCHMARK_STAGE stage=build package=$Package output=$Output"
    [int]$remaining = [int][Math]::Floor((600000 - $script:BenchmarkBuildClock.ElapsedMilliseconds) / 1000)
    if ($remaining -le 0)
    {
        [IO.File]::WriteAllText("$Output.build.timeout.txt", "BUILD_TIMEOUT stage=$Package before_launch")
        exit 124
    }
    [string]$logPrefix = "$Output.build.$([DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffffffZ'))"
    Write-Host "BENCHMARK_BUILD_LOG prefix=$logPrefix"
    [IO.File]::WriteAllText("$logPrefix.arguments.txt", ($Arguments -join "`n"))
    Invoke-BoundedProcess $OdinExe $Arguments $remaining $logPrefix "build-$Package"
    if ($Symbols -eq 'On')
    {
        Publish-BenchmarkSymbols $staging $Output
    }
    elseif ($Emit -eq 'Executable')
    {
        Move-Item -LiteralPath $temporary -Destination $Output -Force
    }
    Write-Host "BENCHMARK_BUILT package=$Package elapsed_ms=$($clock.ElapsedMilliseconds)"
}

function Build-Benchmark([string]$OdinExe, [string]$Package, [string]$Configuration = "Release", [string]$Emit = "Executable", [string]$Components = "All", [string]$Record = "Off", [string]$Symbols = "Off", [string]$Precision = "Float32", [string]$PrecisionCandidate = "", [string]$QueryExecution = "Caller")
{
    Assert-BenchmarkPackage $Package
    [string]$output = Get-BenchmarkBinary $Package $Configuration $Record $Components $Symbols $Precision $PrecisionCandidate $QueryExecution
    [string[]]$profileArguments = @("-define:ENTASIS_PRECISION=$($Precision.ToLowerInvariant())")
    if ($Package -eq "spatial_query_batch")
    {
        $profileArguments += "-define:ENTASIS_QUERY_EXECUTION=$($QueryExecution.ToLowerInvariant())"
    }
    switch ($Configuration)
    {
        'Development'
        {
            $profileArguments += @('-debug', '-o:none', '-source-code-locations:normal', "-pdb-name:$([IO.Path]::ChangeExtension($output, '.pdb'))")
        }
        'Release'
        {
            $profileArguments += @('-o:speed', '-no-bounds-check', '-disable-assert', '-source-code-locations:none')
        }
        default
        {
            throw "Unknown build configuration: $Configuration"
        }
    }
    if ($Symbols -eq 'On')
    {
        $profileArguments += @('-debug', "-pdb-name:$([IO.Path]::ChangeExtension($output, '.pdb'))")
    }
    if ($Emit -eq 'Assembly')
    {
        $output = Join-Path $script:Root "build/inspection/$Package/release/$Package.s"
        if ($Package -eq "large_world_precision")
        {
            $output = Join-Path $script:Root "build/inspection/$Package/release/$PrecisionCandidate/$Package.s"
        }
        elseif ($Precision -eq "Mixed64")
        {
            $output = Join-Path $script:Root "build/inspection/$Package/release/mixed64/$Package.s"
        }
        if ($Package -eq "spatial_query_batch" -and $QueryExecution -eq "Mailbox")
        {
            $output = Join-Path (Split-Path -Parent $output) "${Package}_mailbox.s"
        }
        $profileArguments += '-build-mode:assembly'
    }
    if ($Package -eq "large_world_precision")
    {
        $profileArguments += "-define:ENTASIS_PRECISION_CANDIDATE=$PrecisionCandidate"
    }
    if ($Record -eq 'On')
    {
        $profileArguments += '-define:ENTASIS_BENCHMARK_RECORDING=true'
        [string]$revision = (& git -C $script:Root rev-parse HEAD | Out-String).Trim()
        if ($LASTEXITCODE -ne 0)
        {
            throw 'Cannot read producer source revision.'
        }
        [string]$dirty = (& git -C $script:Root status --porcelain --untracked-files=normal | Out-String).Trim()
        if ($LASTEXITCODE -ne 0)
        {
            throw 'Cannot read producer source state.'
        }
        if ($dirty.Length -gt 0)
        {
            $revision += '+dirty'
        }
        $profileArguments += @("-define:ENTASIS_BENCHMARK_SOURCE_REVISION=$revision", "-define:ENTASIS_BENCHMARK_CONFIGURATION=$($Configuration.ToLowerInvariant())")
    }
    if ($Package -in $script:BenchmarkComponentPackages)
    {
        $profileArguments += "-define:ENTASIS_BENCHMARK_COMPONENTS=$($Components.ToLowerInvariant())"
    }
    if ($Package -eq "scene_stability")
    {
        $profileArguments += "-define:ENTASIS_BENCHMARK_COMPONENTS=common"
    }
    [string[]]$arguments = @('build', (Join-Path $script:BenchmarkHarnessRoot "benchmarks/$Package"), "-out:$output",
        "-collection:entasis=$(Join-Path $script:Root 'src')", "-target:$script:OdinTarget", "-microarch:$script:OdinMicroarch") +
        $profileArguments + @('-vet', '-warnings-as-errors', "-thread-count:$script:OdinThreadCount", '-linker:lld')
    Invoke-BenchmarkBuild $OdinExe $Package $output $arguments $Emit $Symbols
}

function Build-BenchmarkReporter([string]$OdinExe, [string]$Configuration)
{
    [string]$output = Get-BenchmarkReporterBinary $Configuration
    [string[]]$profileArguments = if ($Configuration -eq 'Development')
    {
        @('-debug', '-o:none', '-source-code-locations:normal', "-pdb-name:$([IO.Path]::ChangeExtension($output, '.pdb'))")
    }
    else
    {
        @('-o:speed', '-no-bounds-check', '-disable-assert', '-source-code-locations:none')
    }
    [string[]]$arguments = @('build', (Join-Path $script:BenchmarkHarnessRoot 'tools/benchmark_report'), "-out:$output",
        "-target:$script:OdinTarget", "-microarch:$script:OdinMicroarch") + $profileArguments +
        @('-vet', '-warnings-as-errors', "-thread-count:$script:OdinThreadCount", '-linker:lld')
    Invoke-BenchmarkBuild $OdinExe 'benchmark_report' $output $arguments
}
