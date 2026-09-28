[CmdletBinding(PositionalBinding = $false)]
param(
    [ValidateNotNullOrEmpty()]
    [string]$OdinExe,
    [ValidateSet("Assembly", "LlvmIr", "All")]
    [string]$View = "All"
)
. (Join-Path $PSScriptRoot "../lib/common.ps1")

$odin = Resolve-OdinBinary $OdinExe
$package = Join-Path $script:Root "benchmarks\container"
$inspectionRoot = Join-Path $script:Root "build\inspection\container\release"
$assemblyOutput = Join-Path $inspectionRoot "assembly\container-release.s"
$llvmOutput = Join-Path $inspectionRoot "llvm-ir\container-release.ll"
$releaseArguments = @(
    "-collection:entasis=$(Join-Path $script:Root 'src')",
    "-target:$script:OdinTarget",
    "-microarch:$script:OdinMicroarch",
    "-o:speed",
    "-no-bounds-check",
    "-disable-assert",
    "-source-code-locations:none",
    "-vet",
    "-warnings-as-errors",
    "-thread-count:$script:OdinThreadCount",
    "-linker:lld"
)

Write-Host "CODEGEN_INSPECTION view=$($View.ToLowerInvariant())"
Invoke-Checked $odin @("version")

if ($View -eq "Assembly" -or $View -eq "All")
{
    $assemblyDirectory = Split-Path -Parent $assemblyOutput
    New-Item -ItemType Directory -Force -Path $assemblyDirectory | Out-Null
    Remove-Item -LiteralPath $assemblyOutput -Force -ErrorAction SilentlyContinue
    $arguments = @("build", $package, "-build-mode:assembly", "-out:$assemblyOutput")
    $arguments += $releaseArguments
    Invoke-Checked $odin $arguments
    if (-not (Test-Path -LiteralPath $assemblyOutput -PathType Leaf))
    {
        throw "Assembly output was not produced: $assemblyOutput"
    }
    Write-Host "CODEGEN_ARTIFACT view=assembly path=$assemblyOutput"
}

if ($View -eq "LlvmIr" -or $View -eq "All")
{
    $llvmDirectory = Split-Path -Parent $llvmOutput
    New-Item -ItemType Directory -Force -Path $llvmDirectory | Out-Null
    Remove-Item -LiteralPath $llvmOutput -Force -ErrorAction SilentlyContinue
    $temporaryDirectory = Join-Path $llvmDirectory (".tmp-" + [Guid]::NewGuid().ToString("N"))
    New-Item -ItemType Directory -Path $temporaryDirectory | Out-Null
    try
    {
        $arguments = @("build", $package, "-build-mode:llvm-ir", "-out:$temporaryDirectory")
        $arguments += $releaseArguments
        Invoke-Checked $odin $arguments
        $llvmFiles = @(Get-ChildItem -LiteralPath $temporaryDirectory -Recurse -File -Filter "*.ll")
        if ($llvmFiles.Count -ne 1)
        {
            throw "Expected exactly one LLVM IR file in $temporaryDirectory; found $($llvmFiles.Count)."
        }
        Move-Item -LiteralPath $llvmFiles[0].FullName -Destination $llvmOutput
    }
    finally
    {
        Remove-Item -LiteralPath $temporaryDirectory -Recurse -Force -ErrorAction SilentlyContinue
    }
    if (-not (Test-Path -LiteralPath $llvmOutput -PathType Leaf))
    {
        throw "LLVM IR output was not produced: $llvmOutput"
    }
    Write-Host "CODEGEN_ARTIFACT view=llvm-ir path=$llvmOutput"
}
