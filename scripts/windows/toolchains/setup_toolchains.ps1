[CmdletBinding(PositionalBinding = $false)]
param(
    [Parameter(Mandatory = $true)]
    [ValidateSet("Odin", "CAbiSource", "CAbiSdk")]
    [string]$Usage,
    [ValidateSet("Msvc", "ClangCl", "All")]
    [string]$Compiler = "Msvc",
    [ValidateNotNullOrEmpty()]
    [string]$CacheRoot
)
. (Join-Path $PSScriptRoot "../lib/toolchain_common.ps1")
Assert-ToolchainHost
if ($PSBoundParameters.ContainsKey("Compiler") -and $Usage -ne "CAbiSdk")
{
    [Console]::Error.WriteLine("Compiler applies only to CAbiSdk")
    exit 2
}
if ($CacheRoot.Length -gt 0 -and -not [IO.Path]::IsPathFullyQualified($CacheRoot))
{
    [Console]::Error.WriteLine("CacheRoot must be absolute")
    exit 2
}
if ($Usage -in @("Odin", "CAbiSource"))
{
    $null = Resolve-ToolchainOdin "" "" $CacheRoot
    if ($Usage -eq "CAbiSource")
    {
        $null = Resolve-Toolchain "llvm" "" $CacheRoot
    }
}
else
{
    Import-ToolchainVsEnvironment
    if ($Compiler -in @("ClangCl", "All"))
    {
        $null = Resolve-Toolchain "llvm" "" $CacheRoot
    }
    $null = Resolve-Toolchain "cmake" "" $CacheRoot
    $null = Resolve-Toolchain "ninja" "" $CacheRoot
}
Write-Host "TOOLCHAIN_SETUP_OK usage=$Usage"
