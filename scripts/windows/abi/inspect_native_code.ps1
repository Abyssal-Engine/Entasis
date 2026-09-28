# deliberately parse pairs here: malformed/repeated arguments return 2 before
# toolchain provisioning or output creation
$ErrorActionPreference = "Stop"
function Stop-InspectionUsage([string]$Message)
{
    [Console]::Error.WriteLine("error: $Message")
    exit 2
}
$options = @{}
for ($index = 0; $index -lt $args.Count; $index += 2)
{
    $key = ([string]$args[$index]).ToLowerInvariant()
    if ($key -notin @("-binary", "-symbol") -or $options.ContainsKey($key))
    {
        Stop-InspectionUsage "unknown or repeated option: $($args[$index])"
    }
    if ($index + 1 -ge $args.Count)
    {
        Stop-InspectionUsage "$key requires a value"
    }
    $value = [string]$args[$index + 1]
    if ([string]::IsNullOrWhiteSpace($value) -or $value.StartsWith("-"))
    {
        Stop-InspectionUsage "$key requires a value"
    }
    $options[$key] = $value
}
if (-not $options.ContainsKey("-binary") -or -not $options.ContainsKey("-symbol"))
{
    Stop-InspectionUsage "-Binary and -Symbol are required"
}
$symbol = $options["-symbol"]
if ($symbol -match '[\s\p{Cc}/\\]')
{
    Stop-InspectionUsage "unsupported symbol spelling"
}
$root = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot "../../.."))
$binary = $options["-binary"]
if (-not [IO.Path]::IsPathRooted($binary))
{
    $binary = Join-Path $root $binary
}
try
{
    $image = [IO.Path]::GetFullPath($binary)
}
catch
{
    Stop-InspectionUsage "invalid binary path"
}
if (-not $image.StartsWith($root + [IO.Path]::DirectorySeparatorChar, [StringComparison]::OrdinalIgnoreCase) -or
    -not (Test-Path -LiteralPath $image -PathType Leaf))
{
    Stop-InspectionUsage "binary must be an existing file under the checkout"
}
# do not let a junction/symlink make a lexically in-tree path escape the checkout
for ($item = Get-Item -LiteralPath $image; $item.FullName -ne $root; $item = Get-Item -LiteralPath (Split-Path $item.FullName -Parent))
{
    if (($item.Attributes -band [IO.FileAttributes]::ReparsePoint) -ne 0)
    {
        Stop-InspectionUsage "binary path must not traverse a reparse point"
    }
}
$stem = [IO.Path]::GetFileName($image)
if ($stem -notmatch '^[A-Za-z0-9_.-]+$')
{
    Stop-InspectionUsage "unsafe image filename"
}
$reader = [IO.BinaryReader]::new([IO.File]::OpenRead($image))
try
{
    if ($reader.BaseStream.Length -lt 64 -or $reader.ReadUInt16() -ne 0x5a4d)
    {
        Stop-InspectionUsage "binary must be a native AMD64 PE image"
    }
    $reader.BaseStream.Position = 0x3c
    $peOffset = $reader.ReadUInt32()
    if ($peOffset -gt $reader.BaseStream.Length - 26)
    {
        Stop-InspectionUsage "invalid PE header"
    }
    $reader.BaseStream.Position = $peOffset
    if ($reader.ReadUInt32() -ne 0x4550 -or $reader.ReadUInt16() -ne 0x8664)
    {
        Stop-InspectionUsage "binary must be a native AMD64 PE image"
    }
    $reader.BaseStream.Position = $peOffset + 22
    if (($reader.ReadUInt16() -band 2) -eq 0 -or $reader.ReadUInt16() -ne 0x20b)
    {
        Stop-InspectionUsage "binary must be a linked PE32+ image"
    }
}
finally
{
    $reader.Dispose()
}

$outputName = $symbol
if ($symbol -notmatch '^[A-Za-z_][A-Za-z0-9_]*$' -or $symbol -match '^(?i:CON|PRN|AUX|NUL|COM[1-9]|LPT[1-9])$')
{
    $outputName = "selected-symbol"
}
$outputDirectory = Join-Path $root "build/inspection/native/windows/$stem"
$output = Join-Path $outputDirectory "$outputName.asm"
for ($path = $output; $path -ne $root; $path = Split-Path $path -Parent)
{
    if ((Test-Path -LiteralPath $path) -and ((Get-Item -LiteralPath $path).Attributes -band [IO.FileAttributes]::ReparsePoint) -ne 0)
    {
        Stop-InspectionUsage "inspection output must not traverse a reparse point"
    }
}

. (Join-Path $PSScriptRoot "../lib/toolchain_common.ps1")
$objdump = Resolve-ToolchainExecutable "llvm" "bin/llvm-objdump.exe"
if (-not (Test-Path -LiteralPath $objdump -PathType Leaf))
{
    throw "Selected LLVM has no llvm-objdump: $objdump"
}
function Invoke-InspectionTool([string[]]$ToolArguments, [string]$OutputPath = "")
{
    $start = [Diagnostics.ProcessStartInfo]::new()
    $start.FileName = $objdump
    $start.UseShellExecute = $false
    $start.RedirectStandardOutput = $true
    $start.RedirectStandardError = $true
    $start.StandardOutputEncoding = [Text.UTF8Encoding]::new($false)
    $start.StandardErrorEncoding = [Text.UTF8Encoding]::new($false)
    foreach ($argument in $ToolArguments)
    {
        $start.ArgumentList.Add($argument)
    }
    $process = [Diagnostics.Process]::new()
    $process.StartInfo = $start
    try
    {
        if (-not $process.Start())
        {
            throw "Failed to start selected LLVM"
        }
        $stdoutTask = $process.StandardOutput.ReadToEndAsync()
        $stderrTask = $process.StandardError.ReadToEndAsync()
        $exitStatus = 0
        if (-not $process.WaitForExit(60000))
        {
            $exitStatus = 124
            $process.Kill($true)
            $process.WaitForExit()
        }
        else
        {
            $exitStatus = $process.ExitCode
        }
        $stdout = $stdoutTask.GetAwaiter().GetResult()
        $stderr = $stderrTask.GetAwaiter().GetResult()
        if ($OutputPath.Length -gt 0)
        {
            # replace the output directory entry instead of truncating an
            # existing file, which might be hard-linked to a retained image
            $temporary = Join-Path (Split-Path $OutputPath -Parent) ([IO.Path]::GetRandomFileName())
            try
            {
                [IO.File]::WriteAllText($temporary, "; symbol=$symbol`n; image=$image`n; tool=$objdump`n" + $stdout + $stderr, [Text.UTF8Encoding]::new($false))
                [IO.File]::Move($temporary, $OutputPath, $true)
            }
            finally
            {
                if (Test-Path -LiteralPath $temporary)
                {
                    Remove-Item -LiteralPath $temporary -Force
                }
            }
        }
        if ($stderr.Length -gt 0)
        {
            [Console]::Error.Write($stderr)
        }
        if ($exitStatus -ne 0)
        {
            if ($OutputPath.Length -eq 0)
            {
                [Console]::Error.Write($stdout)
            }
            if ($exitStatus -eq 124)
            {
                [Console]::Error.WriteLine("LLVM inspection exceeded its 60-second ceiling")
            }
            exit $exitStatus
        }
        return $stdout
    }
    finally
    {
        $process.Dispose()
    }
}

# exports are also symbols in stripped PE images. COFF function records, when
# retained, supply non-exported Odin names. only resolve the requested range.
# there is no persisted symbol index and the target image is never rewritten
$metadata = Invoke-InspectionTool @("--syms", "--section-headers", "--private-headers", $image)
if ($metadata -notmatch '(?m)^ImageBase\s+([0-9A-Fa-f]+)\s*$')
{
    throw "Missing PE image base"
}
$imageBase = [Convert]::ToUInt64($Matches[1], 16)
$sections = @{}
foreach ($match in [regex]::Matches($metadata, '(?m)^\s*(\d+)\s+(\S+)\s+([0-9A-Fa-f]+)\s+([0-9A-Fa-f]+)\s+TEXT\s*$'))
{
    $sections[[int]$match.Groups[1].Value + 1] = @{
        Start = [Convert]::ToUInt64($match.Groups[4].Value, 16)
        Size = [Convert]::ToUInt64($match.Groups[3].Value, 16)
    }
}
$records = @([regex]::Matches($metadata, '(?m)^\[\s*\d+\]\(sec\s+(\d+)\).*?\(ty\s+20\).*?0x([0-9A-Fa-f]+)\s+(.+?)\s*$'))
$exports = @([regex]::Matches($metadata, '(?m)^\s*\d+\s+0x([0-9A-Fa-f]+)\s+(\S+)\s*$'))
$addresses = @()
foreach ($record in $records)
{
    $section = $sections[[int]$record.Groups[1].Value]
    if ($null -ne $section -and $record.Groups[3].Value -ceq $symbol)
    {
        $addresses += [uint64]($section.Start + [Convert]::ToUInt64($record.Groups[2].Value, 16))
    }
}
foreach ($record in $exports)
{
    if ($record.Groups[2].Value -ceq $symbol)
    {
        $addresses += [uint64]($imageBase + [Convert]::ToUInt64($record.Groups[1].Value, 16))
    }
}
$addresses = @($addresses | Sort-Object -Unique)
if ($addresses.Count -ne 1)
{
    throw "Unique defined function not found: $symbol"
}
$startAddress = [uint64]$addresses[0]
$containing = @($sections.Values | Where-Object -FilterScript `
{
    $startAddress -ge $_.Start -and $startAddress -lt $_.Start + $_.Size
})
if ($containing.Count -ne 1)
{
    throw "Symbol is not in a unique executable section: $symbol"
}
# like objdump's symbol selector, a leaf without function-size metadata ends at
# the next function symbol, or at the section end. padding may be included.
# this boundary is not reported as an exact native function-size measurement
$stopAddress = [uint64]($containing[0].Start + $containing[0].Size)
foreach ($record in $records)
{
    $section = $sections[[int]$record.Groups[1].Value]
    if ($null -eq $section)
    {
        continue
    }
    $address = [uint64]($section.Start + [Convert]::ToUInt64($record.Groups[2].Value, 16))
    if ($address -gt $startAddress -and $address -lt $stopAddress)
    {
        $stopAddress = $address
    }
}
foreach ($record in $exports)
{
    $address = [uint64]($imageBase + [Convert]::ToUInt64($record.Groups[1].Value, 16))
    if ($address -gt $startAddress -and $address -lt $stopAddress)
    {
        $stopAddress = $address
    }
}
$null = New-Item -ItemType Directory -Force -Path $outputDirectory
Write-Host "NATIVE_INSPECTION image=$image symbol=$symbol output=$output boundary=next-symbol-or-section"
Write-Host "TOOL_PATH=$objdump"
$version = Invoke-InspectionTool @("--version")
Write-Host (($version -split "`n")[0])
$listing = Invoke-InspectionTool @("--disassemble", "--start-address=0x$($startAddress.ToString('x'))", "--stop-address=0x$($stopAddress.ToString('x'))", $image) $output
if ($listing -notmatch '(?m)^\s*[0-9A-Fa-f]+:\s+[0-9A-Fa-f]{2}\s')
{
    throw "No instruction bytes in selected range; see $output"
}
Write-Host "NATIVE_INSPECTION_OK output=$output"
