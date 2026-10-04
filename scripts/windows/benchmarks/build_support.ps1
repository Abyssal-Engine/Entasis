function Get-BenchmarkBinary([string]$Package, [string]$Configuration = "Release", [string]$Record = "Off", [string]$Components = "All")
{
    if ($Package -in $script:BenchmarkComponentPackages)
    {
        $Package += "_$($Components.ToLowerInvariant())"
    }
    if ($Record -eq "On")
    {
        return (Join-Path $script:Root "build\benchmarks\recorded\$($Configuration.ToLowerInvariant())\$Package.exe")
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

function Invoke-BenchmarkBuild([string]$OdinExe, [string]$Package, [string]$Output, [string[]]$Arguments, [string]$Emit = 'Executable')
{
    [Diagnostics.Stopwatch]$clock = [Diagnostics.Stopwatch]::StartNew()
    New-Item -ItemType Directory -Force -Path (Split-Path -Parent $Output) | Out-Null
    [string]$temporary = [IO.Path]::ChangeExtension($Output, '.building.exe')
    if ($Emit -eq 'Executable')
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
    if ($Emit -eq 'Executable')
    {
        Move-Item -LiteralPath $temporary -Destination $Output -Force
    }
    Write-Host "BENCHMARK_BUILT package=$Package elapsed_ms=$($clock.ElapsedMilliseconds)"
}

function Build-Benchmark([string]$OdinExe, [string]$Package, [string]$Configuration = "Release", [string]$Emit = "Executable", [string]$Components = "All", [string]$Record = "Off")
{
    Assert-BenchmarkPackage $Package
    [string]$output = Get-BenchmarkBinary $Package $Configuration $Record $Components
    [string[]]$profileArguments = @()
    switch ($Configuration)
    {
        'Development'
        {
            $profileArguments = @('-debug', '-o:none', '-source-code-locations:normal', "-pdb-name:$([IO.Path]::ChangeExtension($output, '.pdb'))")
        }
        'Release'
        {
            $profileArguments = @('-o:speed', '-no-bounds-check', '-disable-assert', '-source-code-locations:none')
        }
        default
        {
            throw "Unknown build configuration: $Configuration"
        }
    }
    if ($Emit -eq 'Assembly')
    {
        $output = Join-Path $script:Root "build/inspection/$Package/release/$Package.s"
        $profileArguments += '-build-mode:assembly'
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
    Invoke-BenchmarkBuild $OdinExe $Package $output $arguments $Emit
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
