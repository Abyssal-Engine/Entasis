[CmdletBinding(PositionalBinding = $false)]
param(
    [ValidateNotNullOrEmpty()]
    [string]$OdinExe,
    [ValidateSet("Development", "Release", "All")]
    [string]$Configuration = "Development",
    [ValidateNotNullOrEmpty()]
    [string]$Package,
    [switch]$SkipBuild
)
. (Join-Path $PSScriptRoot "../lib/common.ps1")
if ($PSBoundParameters.ContainsKey("Package"))
{
    Assert-ExamplePackage $Package
}
[string[]]$packages = @()
if ($PSBoundParameters.ContainsKey("Package"))
{
    $packages = @($Package)
}
else
{
    $packages = @(Get-ExamplePackages)
}
$odin = Resolve-OdinBinary $OdinExe
$configurations = @($Configuration)
if ($Configuration -eq "All")
{
    $configurations = @("Development", "Release")
}

if (-not $SkipBuild)
{
    $buildArguments = @{
        OdinExe = $odin
        Configuration = $Configuration
    }
    if ($PSBoundParameters.ContainsKey("Package"))
    {
        $buildArguments.Package = $Package
    }
    & (Join-Path $script:Root "build_odin_windows.ps1") @buildArguments
    if ($LASTEXITCODE -ne 0)
    {
        exit $LASTEXITCODE
    }
}

foreach ($selectedConfiguration in $configurations)
{
    foreach ($selectedPackage in $packages)
    {
        $binary = Get-ExampleBinary $selectedPackage $selectedConfiguration
        if (-not (Test-Path -LiteralPath $binary -PathType Leaf))
        {
            throw "Missing example binary: $binary"
        }
        $configurationName = $selectedConfiguration.ToLowerInvariant()
        Write-Host "[example run] $selectedPackage configuration=$configurationName executable=$binary"
        Invoke-Checked $binary @()
    }
}
Write-Host "RUN_EXAMPLES_OK"
