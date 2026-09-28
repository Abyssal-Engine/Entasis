. (Join-Path $PSScriptRoot "toolchain_common.ps1")
$ErrorActionPreference = "Stop"
$script:Root = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot "../../.."))
$script:OdinTarget = "windows_amd64"
$script:OdinMicroarch = "x86-64-v3"
$script:OdinThreadCount = [Math]::Min([Math]::Max([Environment]::ProcessorCount, 1), 8)

function Resolve-RequiredFile([string]$PathText, [string]$Owner)
{
    if ([string]::IsNullOrWhiteSpace($PathText))
    {
        throw "$Owner is required."
    }
    if (-not [System.IO.Path]::IsPathRooted($PathText))
    {
        throw "$Owner must be an absolute path: $PathText"
    }
    if (-not (Test-Path -LiteralPath $PathText -PathType Leaf))
    {
        throw "$Owner was not found: $PathText"
    }
    return (Resolve-Path -LiteralPath $PathText).Path
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

function Invoke-CapturedToFile([string]$FilePath, [string[]]$Arguments, [string]$OutputPath)
{
    $start = [Diagnostics.ProcessStartInfo]::new()
    $start.FileName = $FilePath
    $start.UseShellExecute = $false
    $start.RedirectStandardOutput = $true
    $start.RedirectStandardError = $true
    foreach ($argument in $Arguments)
    {
        $start.ArgumentList.Add($argument)
    }
    $process = [Diagnostics.Process]::new()
    $process.StartInfo = $start
    if (-not $process.Start())
    {
        throw "Failed to start: $FilePath"
    }
    $stdoutTask = $process.StandardOutput.ReadToEndAsync()
    $stderrTask = $process.StandardError.ReadToEndAsync()
    $process.WaitForExit()
    $stdout = $stdoutTask.GetAwaiter().GetResult()
    $stderr = $stderrTask.GetAwaiter().GetResult()
    if (-not [string]::IsNullOrEmpty($stderr))
    {
        [Console]::Error.Write($stderr)
    }
    if ($process.ExitCode -ne 0)
    {
        throw "Command failed ($($process.ExitCode)): $FilePath $($Arguments -join ' ')"
    }
    [System.IO.File]::WriteAllText($OutputPath, $stdout, [System.Text.UTF8Encoding]::new($false))
}

function Get-CAbiBuildRoot([string]$Configuration)
{
    return (Join-Path $script:Root "build\c_abi\windows\$($Configuration.ToLowerInvariant())")
}

function Assert-CAbiBuildArtifacts([string]$BuildRoot, [string]$Configuration)
{
    $required = @(
        "entasis.dll",
        "entasis.lib",
        "entasis_static.lib",
        "entasis_cooking.dll",
        "entasis_cooking.lib",
        "entasis_cooking_static.lib"
    )
    foreach ($name in $required)
    {
        $path = Join-Path $BuildRoot $name
        if (-not (Test-Path -LiteralPath $path -PathType Leaf))
        {
            throw "Missing C ABI build artifact: $path"
        }
    }
    foreach ($name in @("entasis.pdb", "entasis_cooking.pdb"))
    {
        $path = Join-Path $BuildRoot $name
        if ($Configuration -eq "Development")
        {
            if (-not (Test-Path -LiteralPath $path -PathType Leaf))
            {
                throw "Missing Development PDB: $path"
            }
        }
        elseif (Test-Path -LiteralPath $path)
        {
            throw "Release build produced a forbidden PDB: $path"
        }
    }
}
