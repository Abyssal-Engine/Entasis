[CmdletBinding(PositionalBinding = $false)]
param(
    [ValidateNotNullOrEmpty()]
    [string]$OdinExe,
    [Parameter(Mandatory = $true)]
    [ValidateSet("Check", "Write")]
    [string]$Mode
)
. (Join-Path $PSScriptRoot "../lib/c_abi_windows_common.ps1")

$odin = Resolve-ToolchainOdin $OdinExe
$generator = Join-Path $script:Root "tools\abi\generate_abi"
if (-not (Test-Path -LiteralPath $generator -PathType Container))
{
    throw "ABI generator package was not found: $generator"
}

$generatorMode = if ($Mode -eq "Write")
{
    "--write"
}
else
{
    "--check"
}
Push-Location $script:Root
try
{
    Invoke-Checked $odin @("run", $generator, "-vet", "-warnings-as-errors", "--", $generatorMode)
}
finally
{
    Pop-Location
}
