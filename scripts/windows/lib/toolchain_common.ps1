$ErrorActionPreference = "Stop"
$script:ToolchainRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot "../../.."))


function Read-ToolchainData([string]$Path)
{
    $values = @{}
    foreach ($line in [IO.File]::ReadLines($Path))
    {
        if ($line.Length -eq 0 -or $line.StartsWith("#"))
        {
            continue
        }
        $separator = $line.IndexOf("=")
        if ($separator -le 0 -or $line.Substring(0, $separator) -notmatch '^[a-zA-Z0-9_+.-]+$')
        {
            throw "Expected key=value in $Path"
        }
        $parts = $line.Substring(0, $separator).Split(".")
        $target = $values
        for ($index = 0; $index -lt $parts.Length - 1; $index++)
        {
            $part = $parts[$index]
            if (-not $target.ContainsKey($part))
            {
                $target[$part] = @{}
            }
            if ($target[$part] -isnot [Collections.IDictionary])
            {
                throw "Conflicting setting in $Path"
            }
            $target = $target[$part]
        }
        $key = $parts[-1]
        if ($target.ContainsKey($key))
        {
            throw "Repeated setting in $Path"
        }
        $target[$key] = $line.Substring($separator + 1)
    }
    return $values
}

function ConvertTo-ToolchainLines([Collections.IDictionary]$Values, [string]$Prefix = "")
{
    foreach ($key in ($Values.Keys | Sort-Object))
    {
        $value = $Values[$key]
        if ($value -is [Collections.IDictionary])
        {
            ConvertTo-ToolchainLines $value "$Prefix$key."
        }
        else
        {
            $text = [string]$value
            if ($text.Contains("`n") -or $text.Contains("`r"))
            {
                throw "Setting contains a line break: $Prefix$key"
            }
            "$Prefix$key=$text"
        }
    }
}

function Read-ToolchainLocal
{
    $path = Join-Path $script:ToolchainRoot "toolchains.local"
    if (Test-Path -LiteralPath $path)
    {
        $settings = Read-ToolchainData $path
        if ($settings.ContainsKey("windows-amd64"))
        {
            return $settings["windows-amd64"]
        }
    }
    return @{}
}

function Assert-ToolchainHost
{
    if (-not $IsWindows -or [Runtime.InteropServices.RuntimeInformation]::OSArchitecture -ne "X64")
    {
        throw "Native Windows AMD64 and PowerShell 7 are required"
    }
}

function Assert-ToolchainBundle([string]$Root, [string]$Tool, [hashtable]$Entry)
{
    $names = @(switch ($Tool)
    {
        "odin"
        {
            @("odin.exe")
        }
        "llvm"
        {
            @("bin/clang-cl.exe", "bin/lld-link.exe", "bin/llvm-readobj.exe")
        }
        "cmake"
        {
            @("bin/cmake.exe", "bin/ctest.exe")
        }
        "ninja"
        {
            @("ninja.exe")
        }
    })
    foreach ($name in $names)
    {
        $path = Join-Path $Root $name
        if (-not (Test-Path -LiteralPath $path -PathType Leaf))
        {
            throw "Required native executable missing: $path"
        }
        $stream = [IO.File]::OpenRead($path)
        try
        {
            if ($stream.ReadByte() -ne 77 -or $stream.ReadByte() -ne 90)
            {
                throw "Required Windows PE executable is incompatible: $path"
            }
        }
        finally
        {
            $stream.Dispose()
        }
    }
    if ($Tool -eq "odin")
    {
        foreach ($name in @("base", "core", "vendor", "LLVM-C.dll"))
        {
            if (-not (Test-Path -LiteralPath (Join-Path $Root $name)))
            {
                throw "Incomplete Odin distribution: $Root ($name)"
            }
        }
    }
    $argument = if ($Tool -eq "odin")
    {
        "version"
    }
    else
    {
        "--version"
    }
    $version = @(& (Join-Path $Root $names[0]) $argument)[0]
    if ($LASTEXITCODE -ne 0)
    {
        throw "Version check failed: $Root"
    }
    if ($Tool -eq "odin")
    {
        if ($version -notmatch 'version (\S+):([0-9a-f]{7,40})' -or
            $Matches[1] -notin @($Entry.version, "$($Entry.version)-nightly") -or
            -not $Entry.commit.StartsWith($Matches[2]))
        {
            throw "Odin declaration mismatch: $version"
        }
    }
    elseif ($version -notmatch ("(?<![\d.])" + [regex]::Escape($Entry.version) + "(?![\d.])"))
    {
        throw "$Tool declaration mismatch: $version"
    }
    Write-Host "TOOLCHAIN tool=$Tool version=$version root=$Root"
}

function Receive-ToolchainArchive([hashtable]$Asset, [string]$Destination)
{
    if (([Uri]$Asset.url).IsFile)
    {
        [IO.File]::Copy(([Uri]$Asset.url).LocalPath, $Destination)
        Assert-ToolchainArchive $Asset $Destination
        return
    }
    $client = [Net.Http.HttpClient]::new()
    $client.Timeout = [TimeSpan]::FromSeconds(30)
    $ceiling = [Threading.CancellationTokenSource]::new([TimeSpan]::FromSeconds(900))
    $response = $null
    $inputStream = $null
    $outputStream = $null
    try
    {
        $response = $client.GetAsync($Asset.url, [Net.Http.HttpCompletionOption]::ResponseHeadersRead, $ceiling.Token).GetAwaiter().GetResult()
        $null = $response.EnsureSuccessStatusCode()
        $inputStream = $response.Content.ReadAsStream()
        $outputStream = [IO.File]::Open($Destination, [IO.FileMode]::CreateNew)
        $buffer = [byte[]]::new(1048576)
        $count = 0L
        $clock = [Diagnostics.Stopwatch]::StartNew()
        $reportAt = 0L
        while ($true)
        {
            $readTimeout = [Threading.CancellationTokenSource]::CreateLinkedTokenSource($ceiling.Token)
            try
            {
                $readTimeout.CancelAfter(30000)
                $size = $inputStream.ReadAsync($buffer, 0, $buffer.Length, $readTimeout.Token).GetAwaiter().GetResult()
            }
            finally
            {
                $readTimeout.Dispose()
            }
            if ($size -eq 0)
            {
                break
            }
            $outputStream.Write($buffer, 0, $size)
            $count += $size
            if ($clock.ElapsedMilliseconds - $reportAt -ge 10000)
            {
                Write-Host "DOWNLOADING bytes=$count/$($Asset.size)"
                $reportAt = $clock.ElapsedMilliseconds
            }
        }
    }
    finally
    {
        if ($null -ne $outputStream)
        {
            $outputStream.Dispose()
        }
        if ($null -ne $inputStream)
        {
            $inputStream.Dispose()
        }
        if ($null -ne $response)
        {
            $response.Dispose()
        }
        $ceiling.Dispose()
        $client.Dispose()
    }
    Assert-ToolchainArchive $Asset $Destination
}

function Assert-ToolchainArchive([hashtable]$Asset, [string]$Destination)
{
    if ((Get-Item -LiteralPath $Destination).Length -ne $Asset.size -or
        (Get-FileHash -LiteralPath $Destination -Algorithm SHA256).Hash -ne $Asset.sha256)
    {
        throw "Archive size or SHA-256 mismatch: $Destination"
    }
}

function Expand-ToolchainArchive([string]$Archive, [string]$Destination)
{
    $null = New-Item -ItemType Directory -Path $Destination
    if ($Archive.EndsWith(".zip"))
    {
        $source = [IO.Compression.ZipFile]::OpenRead($Archive)
        $clock = [Diagnostics.Stopwatch]::StartNew()
        $prefix = [IO.Path]::GetFullPath($Destination).TrimEnd([IO.Path]::DirectorySeparatorChar) + [IO.Path]::DirectorySeparatorChar
        try
        {
            foreach ($entry in $source.Entries)
            {
                $path = [IO.Path]::GetFullPath((Join-Path $Destination $entry.FullName))
                if (-not $path.StartsWith($prefix, [StringComparison]::OrdinalIgnoreCase) -or
                    ($entry.ExternalAttributes -shr 16 -band 61440) -eq 40960)
                {
                    throw "Unsafe archive member: $($entry.FullName)"
                }
                if ($entry.Name.Length -eq 0)
                {
                    $null = [IO.Directory]::CreateDirectory($path)
                    continue
                }
                $null = [IO.Directory]::CreateDirectory((Split-Path $path -Parent))
                $inputStream = $entry.Open()
                $outputStream = [IO.File]::Open($path, [IO.FileMode]::CreateNew)
                try
                {
                    $buffer = [byte[]]::new(1048576)
                    while (($count = $inputStream.Read($buffer, 0, $buffer.Length)) -gt 0)
                    {
                        $outputStream.Write($buffer, 0, $count)
                        if ($clock.Elapsed.TotalSeconds -gt 900)
                        {
                            throw "Archive extraction exceeded its 900-second ceiling: $Archive"
                        }
                    }
                }
                finally
                {
                    $outputStream.Dispose()
                    $inputStream.Dispose()
                }
            }
        }
        finally
        {
            $source.Dispose()
        }
    }
    else
    {
        $tar = Join-Path $env:SystemRoot "System32/tar.exe"
        $members = @(& $tar -tf $Archive)
        if ($LASTEXITCODE -ne 0)
        {
            throw "Cannot inspect archive: $Archive"
        }
        foreach ($member in $members)
        {
            if ($member -match '(^[/\\]|(^|[/\\])\.\.([/\\]|$)|:)')
            {
                throw "Unsafe archive member: $member"
            }
        }
        $start = [Diagnostics.ProcessStartInfo]::new($tar)
        $start.UseShellExecute = $false
        foreach ($argument in @("-xf", $Archive, "-C", $Destination))
        {
            $start.ArgumentList.Add($argument)
        }
        $process = [Diagnostics.Process]::Start($start)
        if (-not $process.WaitForExit(900000))
        {
            $process.Kill($true)
            throw "Archive extraction exceeded its 900-second ceiling: $Archive"
        }
        if ($process.ExitCode -ne 0)
        {
            throw "Archive extraction failed: $Archive"
        }
        $process.Dispose()
    }
    $children = @(Get-ChildItem -LiteralPath $Destination)
    if ($children.Count -eq 1 -and $children[0].PSIsContainer)
    {
        return $children[0].FullName
    }
    return $Destination
}

function Resolve-Toolchain([string]$Tool, [string]$Override = "", [string]$CacheRoot = "")
{
    Assert-ToolchainHost
    $entries = Read-ToolchainData (Join-Path $script:ToolchainRoot "tools/toolchains.lock")
    $entry = $entries[$Tool]
    $asset = $entry.hosts["windows-amd64"]
    $local = Read-ToolchainLocal
    $external = $Override
    if ($external.Length -eq 0 -and $local.ContainsKey("roots") -and $local.roots.ContainsKey($Tool))
    {
        $external = $local.roots[$Tool]
    }
    if ($CacheRoot.Length -eq 0)
    {
        $CacheRoot = if ($local.ContainsKey("cache_root"))
        {
            $local.cache_root
        }
        else
        {
            Join-Path $script:ToolchainRoot "build/toolchains/prebuilt"
        }
    }
    if (-not [IO.Path]::IsPathFullyQualified($CacheRoot) -or
        ($external.Length -gt 0 -and -not [IO.Path]::IsPathFullyQualified($external)))
    {
        throw "Tool roots and cache_root must be absolute paths"
    }
    $root = if ($external.Length -gt 0)
    {
        $external
    }
    else
    {
        Join-Path $CacheRoot "$Tool/$($entry.release)/windows-amd64"
    }
    if ($external.Length -gt 0 -or (Test-Path -LiteralPath $root))
    {
        if ($external.Length -eq 0)
        {
            $identity = Read-ToolchainData (Join-Path $root ".entasis-toolchain")
            foreach ($key in $asset.Keys)
            {
                if ($identity[$key] -ne $asset[$key])
                {
                    throw "Cache identity differs from declared asset: $root"
                }
            }
        }
        Assert-ToolchainBundle $root $Tool $entry
        return $root
    }
    $parent = Split-Path $root -Parent
    $null = New-Item -ItemType Directory -Force -Path $parent
    $lockPath = "$root.lock"
    $lock = [IO.File]::Open($lockPath, [IO.FileMode]::CreateNew, [IO.FileAccess]::Write, [IO.FileShare]::None)
    $work = Join-Path $parent (".install-" + [Guid]::NewGuid().ToString("N"))
    try
    {
        $null = New-Item -ItemType Directory -Path $work
        $archive = Join-Path $work $asset.asset
        Write-Host "DOWNLOAD tool=$Tool release=$($entry.release) bytes=$($asset.size) destination=$root partial=$work"
        Receive-ToolchainArchive $asset $archive
        Write-Host "EXTRACTING $archive"
        $candidate = Expand-ToolchainArchive $archive (Join-Path $work "extracted")
        Assert-ToolchainBundle $candidate $Tool $entry
        ConvertTo-ToolchainLines $asset | Set-Content -LiteralPath (Join-Path $candidate ".entasis-toolchain")
        Move-Item -LiteralPath $candidate -Destination $root
        Remove-Item -LiteralPath $work -Recurse -Force
        return $root
    }
    catch
    {
        [Console]::Error.WriteLine("INSTALL_FAILED partial=$work")
        throw
    }
    finally
    {
        $lock.Dispose()
        Remove-Item -LiteralPath $lockPath
    }
}

function Import-ToolchainVsEnvironment([string]$Override = "")
{
    $local = Read-ToolchainLocal
    $devcmd = $Override
    if ($devcmd.Length -eq 0 -and $local.ContainsKey("system"))
    {
        $devcmd = $local.system.vsdevcmd
    }
    if ([string]::IsNullOrEmpty($devcmd))
    {
        $vswhere = Join-Path ${env:ProgramFiles(x86)} "Microsoft Visual Studio/Installer/vswhere.exe"
        if (-not (Test-Path -LiteralPath $vswhere))
        {
            throw "Install Visual Studio C++ build tools, or select system.vsdevcmd in toolchains.local"
        }
        $installation = & $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
        if ($LASTEXITCODE -ne 0 -or [string]::IsNullOrEmpty($installation))
        {
            throw "Visual Studio C++ build tools were not found"
        }
        $devcmd = Join-Path $installation "Common7/Tools/VsDevCmd.bat"
    }
    if (-not [IO.Path]::IsPathFullyQualified($devcmd) -or -not (Test-Path -LiteralPath $devcmd -PathType Leaf))
    {
        throw "Invalid Visual Studio developer command: $devcmd"
    }
    $command = '"{0}" -arch=amd64 -host_arch=amd64 >nul && set' -f $devcmd
    $lines = @(& $env:ComSpec /d /s /c $command)
    if ($LASTEXITCODE -ne 0)
    {
        exit $LASTEXITCODE
    }
    foreach ($line in $lines)
    {
        $separator = $line.IndexOf('=')
        if ($separator -gt 0)
        {
            [Environment]::SetEnvironmentVariable($line.Substring(0, $separator), $line.Substring($separator + 1), "Process")
        }
    }
    Write-Host "VS_ENVIRONMENT compiler=$((Get-Command cl.exe -CommandType Application -ErrorAction Stop).Source)"
}

function Resolve-ToolchainOdin([string]$OdinExe = "", [string]$VsDevCmd = "", [string]$CacheRoot = "")
{
    $override = ""
    if ($OdinExe.Length -gt 0)
    {
        if (-not [IO.Path]::IsPathFullyQualified($OdinExe) -or [IO.Path]::GetFileName($OdinExe) -ne "odin.exe" -or
            -not (Test-Path -LiteralPath $OdinExe -PathType Leaf))
        {
            throw "OdinExe must name an existing absolute native odin.exe"
        }
        $override = Split-Path $OdinExe -Parent
    }
    $root = Resolve-Toolchain "odin" $override $CacheRoot
    $env:ODIN_ROOT = $root
    Import-ToolchainVsEnvironment $VsDevCmd
    return (Join-Path $root "odin.exe")
}

function Resolve-ToolchainExecutable([string]$Tool, [string]$RelativePath, [string]$Override = "")
{
    $external = ""
    if ($Override.Length -gt 0)
    {
        if (-not [IO.Path]::IsPathFullyQualified($Override) -or -not (Test-Path -LiteralPath $Override -PathType Leaf))
        {
            throw "Expected an existing absolute executable: $Override"
        }
        $external = Split-Path $Override -Parent
        if ($RelativePath.StartsWith("bin/"))
        {
            $external = Split-Path $external -Parent
        }
        if ([IO.Path]::GetFullPath((Join-Path $external $RelativePath)) -ne [IO.Path]::GetFullPath($Override))
        {
            throw "Expected $RelativePath within a complete $Tool distribution: $Override"
        }
    }
    return (Join-Path (Resolve-Toolchain $Tool $external) $RelativePath)
}
