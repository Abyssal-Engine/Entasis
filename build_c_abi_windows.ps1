[CmdletBinding(PositionalBinding = $false)]
param(
    [ValidateNotNullOrEmpty()]
    [string]$OdinExe,
    [ValidateNotNullOrEmpty()]
    [string]$VsDevCmd,
    [ValidateNotNullOrEmpty()]
    [string]$LlvmReadObjExe,
    [ValidateSet("Development", "Release")]
    [string]$Configuration = "Release"
)
. (Join-Path $PSScriptRoot "scripts/windows/lib/c_abi_windows_common.ps1")

$odin = Resolve-ToolchainOdin $OdinExe $VsDevCmd
$llvmReadObj = Resolve-ToolchainExecutable "llvm" "bin/llvm-readobj.exe" $LlvmReadObjExe
$generator = Join-Path $script:Root "tools\abi\generate_abi"
$exportVerifier = Join-Path $script:Root "tools\abi\verify_exports"
if (-not (Test-Path -LiteralPath $generator -PathType Container))
{
    throw "ABI generator package was not found: $generator"
}
if (-not (Test-Path -LiteralPath $exportVerifier -PathType Container))
{
    throw "Export verifier package was not found: $exportVerifier"
}
$runtimeDef = Resolve-RequiredFile (Join-Path $script:Root "generated\abi\entasis.def") "runtime DEF"
$cookingDef = Resolve-RequiredFile (Join-Path $script:Root "generated\abi\entasis_cooking.def") "cooking DEF"


Push-Location $script:Root
try
{
    Invoke-Checked $odin @("run", $generator, "-vet", "-warnings-as-errors", "--", "--check")
}
finally
{
    Pop-Location
}

$buildRoot = Get-CAbiBuildRoot $Configuration
$runtimeDll = Join-Path $buildRoot "entasis.dll"
$runtimeImport = Join-Path $buildRoot "entasis.lib"
$runtimeStatic = Join-Path $buildRoot "entasis_static.lib"
$runtimePdb = Join-Path $buildRoot "entasis.pdb"
$cookingDll = Join-Path $buildRoot "entasis_cooking.dll"
$cookingImport = Join-Path $buildRoot "entasis_cooking.lib"
$cookingStatic = Join-Path $buildRoot "entasis_cooking_static.lib"
$cookingPdb = Join-Path $buildRoot "entasis_cooking.pdb"

New-Item -ItemType Directory -Force -Path $buildRoot | Out-Null
$knownArtifacts = @(
    $runtimeDll, $runtimeImport, $runtimeStatic, $runtimePdb,
    $cookingDll, $cookingImport, $cookingStatic, $cookingPdb,
    "$runtimeDll.obj", "$runtimeStatic.obj", "$cookingDll.obj", "$cookingStatic.obj"
)
Remove-Item -LiteralPath $knownArtifacts -Force -ErrorAction SilentlyContinue

$commonArguments = @(
    "-collection:entasis=$(Join-Path $script:Root 'src')",
    "-target:$script:OdinTarget",
    "-microarch:$script:OdinMicroarch",
    "-no-entry-point",
    "-thread-count:$script:OdinThreadCount",
    "-vet",
    "-warnings-as-errors"
)
if ($Configuration -eq "Development")
{
    $profileArguments = @("-debug", "-o:none", "-source-code-locations:normal")
    $symbolDisplay = "$runtimePdb,$cookingPdb"
}
else
{
    $profileArguments = @("-o:speed", "-no-bounds-check", "-disable-assert", "-source-code-locations:none")
    $symbolDisplay = "none"
}
Write-Host "C_ABI_BUILD_PROFILE configuration=$($Configuration.ToLowerInvariant()) root=$buildRoot symbols=$symbolDisplay"

function Build-CAbiLibrary(
    [string]$Package,
    [string]$Dll,
    [string]$ImportLibrary,
    [string]$StaticLibrary,
    [string]$DefFile,
    [string]$Pdb
)
{
    $dllArguments = @("build", $Package, "-build-mode:dll", "-linker:lld", "-out:$Dll")
    $dllArguments += $commonArguments
    $dllArguments += $profileArguments
    if ($Configuration -eq "Development")
    {
        $dllArguments += "-pdb-name:$Pdb"
    }
    $dllArguments += ("-extra-linker-flags:/DEF:$DefFile /IMPLIB:$ImportLibrary /DEFAULTLIB:libvcruntime.lib /DEFAULTLIB:libucrt.lib")
    Invoke-Checked $odin $dllArguments

    $staticArguments = @("build", $Package, "-build-mode:lib", "-out:$StaticLibrary")
    $staticArguments += $commonArguments
    $staticArguments += $profileArguments
    Invoke-Checked $odin $staticArguments
}

Build-CAbiLibrary (Join-Path $script:Root "src\entasis_c") $runtimeDll $runtimeImport $runtimeStatic $runtimeDef $runtimePdb
Build-CAbiLibrary (Join-Path $script:Root "src\entasis_cooking_c") $cookingDll $cookingImport $cookingStatic $cookingDef $cookingPdb
Assert-CAbiBuildArtifacts $buildRoot $Configuration
Push-Location $script:Root
try
{
    Invoke-Checked $odin @("run", $exportVerifier, "-vet", "-warnings-as-errors", "--", "--format", "coff", "--tool", $llvmReadObj, "runtime", $runtimeDll)
    Invoke-Checked $odin @("run", $exportVerifier, "-vet", "-warnings-as-errors", "--", "--format", "coff", "--tool", $llvmReadObj, "cooking", $cookingDll)
}
finally
{
    Pop-Location
}
Write-Host "C_ABI_BUILD_OK configuration=$($Configuration.ToLowerInvariant()) root=$buildRoot"
