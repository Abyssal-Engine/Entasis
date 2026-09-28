[CmdletBinding(PositionalBinding = $false)]
param(
    [Parameter(Mandatory = $true)]
    [ValidateSet("odin", "llvm", "cmake", "ninja", "all")]
    [string]$Tool,
    [ValidateSet("Check", "Update")]
    [string]$Mode = "Update",
    [ValidateNotNullOrEmpty()]
    [string]$Version
)
. (Join-Path $PSScriptRoot "../lib/toolchain_common.ps1")
Assert-ToolchainHost
if ($PSBoundParameters.ContainsKey("Version"))
{
    $pattern = if ($Tool -eq "odin")
    {
        '^dev-\d{4}-\d{2}$'
    }
    else
    {
        '^\d+\.\d+\.\d+$'
    }
    if ($Tool -eq "all" -or $Version -notmatch $pattern)
    {
        [Console]::Error.WriteLine("Version requires one tool and a valid stable/monthly release")
        exit 2
    }
}

function Read-OfficialRelease([string]$Repository, [string]$Route)
{
    Write-Host "RELEASE_METADATA https://api.github.com/repos/$Repository$Route"
    return Invoke-RestMethod -Uri "https://api.github.com/repos/$Repository$Route" -ConnectionTimeoutSeconds 30 -OperationTimeoutSeconds 30 -Headers @{
        Accept = "application/vnd.github+json"
        "User-Agent" = "Entasis-toolchains"
    }
}

$path = Join-Path $script:ToolchainRoot "tools/toolchains.lock"
$entries = Read-ToolchainData $path
$selected = if ($Tool -eq "all")
{
    @("odin", "llvm", "cmake", "ninja")
}
else
{
    @($Tool)
}
foreach ($name in $selected)
{
    $repository = @{
        odin = "odin-lang/Odin"
        llvm = "llvm/llvm-project"
        cmake = "Kitware/CMake"
        ninja = "ninja-build/ninja"
    }[$name]
    $prefix = @{ odin = ""; llvm = "llvmorg-"; cmake = "v"; ninja = "v" }[$name]
    $route = if ($Version.Length -gt 0)
    {
        "/releases/tags/$prefix$Version"
    }
    else
    {
        "/releases/latest"
    }
    $release = Read-OfficialRelease $repository $route
    if ($release.draft -or $release.prerelease)
    {
        throw "Only published stable/monthly releases are supported"
    }
    $tag = $release.tag_name
    $number = $tag.Substring($prefix.Length)
    $assets = switch ($name)
    {
        "odin"
        {
            @("odin-linux-amd64-$tag.tar.gz", "odin-windows-amd64-$tag.zip")
        }
        "llvm"
        {
            @("LLVM-$number-Linux-X64.tar.xz", "clang+llvm-$number-x86_64-pc-windows-msvc.tar.xz")
        }
        "cmake"
        {
            @("cmake-$number-linux-x86_64.tar.gz", "cmake-$number-windows-x86_64.zip")
        }
        "ninja"
        {
            @("ninja-linux.zip", "ninja-win.zip")
        }
    }
    $entry = [ordered]@{
        repository = $repository
        release = $tag
        version = $number
        hosts = [ordered]@{}
    }
    if ($name -eq "odin")
    {
        $entry.commit = (Read-OfficialRelease $repository "/commits/$tag").sha
    }
    $hosts = @("linux-amd64", "windows-amd64")
    for ($index = 0; $index -lt 2; $index++)
    {
        $assetMatches = @($release.assets | Where-Object name -EQ $assets[$index])
        if ($assetMatches.Count -ne 1 -or $assetMatches[0].digest -notmatch '^sha256:[0-9a-f]{64}$')
        {
            throw "Required official asset/digest unavailable: $($assets[$index])"
        }
        $asset = $assetMatches[0]
        if (-not $asset.browser_download_url.StartsWith("https://github.com/$repository/releases/download/"))
        {
            throw "Unexpected official asset URL: $($asset.browser_download_url)"
        }
        $entry.hosts[$hosts[$index]] = [ordered]@{
            asset = $asset.name
            url = $asset.browser_download_url
            sha256 = $asset.digest.Substring(7)
            size = $asset.size
        }
    }
    Write-Host "TOOLCHAIN_RELEASE tool=$name old=$($entries[$name].release) selected=$tag mode=$Mode"
    $entries[$name] = $entry
}
if ($Mode -eq "Update")
{
    $pending = "$path.pending"
    $stream = [IO.File]::Open($pending, [IO.FileMode]::CreateNew)
    $writer = [IO.StreamWriter]::new($stream)
    try
    {
        $current = Read-ToolchainData $path
        foreach ($name in $selected)
        {
            $current[$name] = $entries[$name]
        }
        $writer.WriteLine(((ConvertTo-ToolchainLines $current) -join "`n"))
    }
    finally
    {
        $writer.Dispose()
    }
    [IO.File]::Move($pending, $path, $true)
}
Write-Host "TOOLCHAIN_UPDATE_OK mode=$Mode"
