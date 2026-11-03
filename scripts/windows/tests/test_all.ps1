[CmdletBinding(PositionalBinding = $false)]
param(
    [ValidateNotNullOrEmpty()]
    [string]$OdinExe,
    [ValidateSet("Development", "Release", "All")]
    [string]$Configuration = "All",
    [string]$StartPackage
)
. (Join-Path $PSScriptRoot "../lib/common.ps1")
[string[]]$packages = @($script:TestPackages) + @($script:CodegenPackages)
[int]$startIndex = 0
if ($PSBoundParameters.ContainsKey("StartPackage"))
{
    $startIndex = [Array]::IndexOf($packages, $StartPackage)
    if ($startIndex -lt 0)
    {
        [Console]::Error.WriteLine("Unknown or empty start package: $StartPackage")
        exit 2
    }
}
$odin = Resolve-OdinBinary $OdinExe

$configurations = if ($Configuration -eq "All")
{
    @("Development", "Release")
}
else
{
    @($Configuration)
}
foreach ($selectedConfiguration in $configurations)
{
    Write-Host "TEST_ALL_CONFIG configuration=$($selectedConfiguration.ToLowerInvariant()) start_package=$($packages[$startIndex])"
    for ([int]$packageIndex = $startIndex; $packageIndex -lt $packages.Count; $packageIndex++)
    {
        [string]$package = $packages[$packageIndex]
        if ($packageIndex -lt $script:TestPackages.Count)
        {
            Invoke-TestPackage $odin $package $selectedConfiguration
        }
        else
        {
            Invoke-CodegenPackage $odin $package $selectedConfiguration
        }
    }
    $startIndex = 0
}
Write-Host "TEST_ALL_OK"
