[CmdletBinding(PositionalBinding = $false)]
param(
    [ValidateNotNullOrEmpty()]
    [string]$OdinExe,
    [ValidateSet("Development", "Release", "All")]
    [string]$Configuration = "All"
)
. (Join-Path $PSScriptRoot "../lib/common.ps1")
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
    foreach ($package in $script:TestPackages)
    {
        Invoke-TestPackage $odin $package $selectedConfiguration
    }
    foreach ($package in $script:CodegenPackages)
    {
        Invoke-CodegenPackage $odin $package $selectedConfiguration
    }
}
Write-Host "TEST_ALL_OK"
