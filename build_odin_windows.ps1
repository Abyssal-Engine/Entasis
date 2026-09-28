[CmdletBinding(PositionalBinding = $false)]
param(
    [ValidateNotNullOrEmpty()]
    [string]$OdinExe,
    [ValidateSet("Development", "Release", "All")]
    [string]$Configuration = "Development",
    [ValidateNotNullOrEmpty()]
    [string]$Package
)
. (Join-Path $PSScriptRoot "scripts/windows/lib/common.ps1")
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
Write-Host "EXAMPLE_BUILD_CONFIG configuration=$($Configuration.ToLowerInvariant()) packages=$($packages -join ',')"

function Build-Example([string]$SelectedPackage, [string]$SelectedConfiguration)
{
    Initialize-OdinProfile "examples" $SelectedPackage $SelectedConfiguration
    $configurationName = $SelectedConfiguration.ToLowerInvariant()
    Write-Host "[example build] $SelectedPackage configuration=$configurationName executable=$script:OdinProfileOutput"
    $arguments = @(
        "build", (Join-Path $script:Root "examples\headless\$SelectedPackage"),
        "-out:$script:OdinProfileOutput", "-collection:entasis=$(Join-Path $script:Root 'src')",
        "-target:$script:OdinTarget", "-microarch:$script:OdinMicroarch"
    )
    $arguments += $script:OdinProfileArguments
    $arguments += @(
        "-vet", "-warnings-as-errors", "-thread-count:$script:OdinThreadCount", "-linker:lld"
    )
    Invoke-Checked $odin $arguments
}

foreach ($selectedConfiguration in $configurations)
{
    foreach ($selectedPackage in $packages)
    {
        Build-Example $selectedPackage $selectedConfiguration
    }
}
Write-Host "BUILD_EXAMPLES_OK"
