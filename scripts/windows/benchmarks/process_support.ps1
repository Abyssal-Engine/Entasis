# shared process lifetime, affinity and native stream capture for benchmark producers
function Initialize-BenchmarkProcessAffinity
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
    Write-Host "BENCHMARK_CHILD_EXIT stage=$Stage exit_code=$exitCode"
    return $exitCode
}

