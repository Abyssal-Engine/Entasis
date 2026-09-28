# Building Entasis

Odin applications compile Entasis source into the application. C and C++ applications link the runtime and optional cooking libraries, either from a prebuilt SDK or a source build

Run checkout commands from the repository root and package commands from the extracted package root

## Choose a build path

Choose a download from [Releases](https://github.com/Abyssal-Engine/Entasis/releases):

- `odin-source`: native Odin source, with equivalent ZIP and TAR.GZ formats for either supported host
- Platform `c-sdk`: prebuilt C ABI libraries for C, C++ and other C-interop languages
- GitHub's automatic source archives: the full repository, including tests, benchmarks and maintainer tools

Extracted packages include their own build guides. For checkout commands, choose your [language and platform](README.md#build-and-run)

## Requirements

Both platforms require AMD64, the x86-64 CPU architecture, with the `x86-64-v3` instruction-set level, including AVX2

### Linux

- Bash, curl, GNU coreutils, tar, gzip, xz, unzip, and `dpkg-deb`
- Native C development headers and libraries, plus binutils
- C++ development headers and libraries for C++ consumers
- GCC/G++ when building the C/C++ consumers with GCC

The build commands locate or download the declared Odin and LLVM distributions. Linux LLVM uses a private pinned ICU runtime. Setup does not change system libraries. WSL uses these Linux commands and produces Linux executables

### Windows

- Native x64 PowerShell 7
- Visual Studio or Build Tools with **Desktop development with C++**
- MSVC x64/x86 tools and a Windows SDK

Scripts discover Visual Studio with `vswhere.exe` and load its x64 developer environment. The C++ toolset and Windows SDK are also needed when selecting clang-cl

### Optional visual tools

Build the optional [physics viewer](../tools/physics_viewer/README.md#build-and-open) from the full checkout. It uses Odin's bundled raylib/raygui and LZ4 bindings. The C SDK and Odin source archives do not include the viewer's sources, commands or binaries

Native Linux requires linkable/runtime `liblz4`, X11/OpenGL development libraries and a working desktop context for the viewer. Headless capture requires LZ4 but no display. Benchmark producers built with recording On also require LZ4. Recording-Off benchmarks and ordinary engine/headless example builds need no graphics or compression dependencies. Visual commands do not install these prerequisites

Benchmark Build compiles the selected producers and reporter. Run uses prepared binaries and requires no compiler. Rebuild explicitly after source or toolchain changes. See [benchmark commands](../BENCHMARK.md#reproduction) or the viewer's [benchmark launch actions](../tools/physics_viewer/README.md#benchmark-launch-and-recordings)

## Toolchain selection

Versions and checksums are declared in [`tools/toolchains.lock`](../tools/toolchains.lock). Normal builds reuse a matching installation or download the declared release. They do not advance versions

Selection order:

1. An explicit tool argument supported by the command
2. A matching entry in the ignored `toolchains.local` file
3. The declared distribution under `build/toolchains/prebuilt`

The first download requires HTTPS access. A matching cached distribution can be reused offline. If the tools are already installed elsewhere, select them in `toolchains.local` before building

Use literal `key=value` entries with absolute paths:

```text
linux-amd64.roots.odin=/opt/odin
linux-amd64.roots.llvm=/opt/llvm
windows-amd64.roots.odin=C:/Tools/Odin
windows-amd64.system.vsdevcmd=C:/BuildTools/Common7/Tools/VsDevCmd.bat
```

Odin and LLVM overrides must select complete distributions matching the declaration. Do not quote or shell-escape values

### Visual Studio selection

Use `windows-amd64.system.vsdevcmd` to select an installed developer command for all Windows flows. C ABI and SDK commands also accept `-VsDevCmd` where listed in their parameters. Odin example and test commands use the shared setting

Selecting a path does not install a missing workload or SDK

## Build profiles

| Profile     | Odin flags                                                              | Purpose                                     |
| ----------- | ----------------------------------------------------------------------- | ------------------------------------------- |
| Development | `-debug -o:none -source-code-locations:normal`                          | Debugging with assertions and bounds checks |
| Release     | `-o:speed -no-bounds-check -disable-assert -source-code-locations:none` | Optimized builds                            |

Supplied builds target `linux_amd64` or `windows_amd64` with `-microarch:x86-64-v3`. Example builds default to Development. C ABI builds default to Release

## Output layout

| Output         | Location                               |
| -------------- | -------------------------------------- |
| Odin examples  | `build/examples/<configuration>/`      |
| Odin tests     | `build/tests/<configuration>/`         |
| Linux C ABI    | `build/c_abi/linux/<configuration>/`   |
| Windows C ABI  | `build/c_abi/windows/<configuration>/` |
| SDK packages   | `build/sdk/<platform>/`                |
| Source package | `build/packages/source/<platform>/`    |

Ordinary builds replace their previous outputs. A successful build does not run the application: use the platform's example runner to check a scene. The example runner builds by default, so a separate build command is unnecessary when the goal is to run an example. Package scripts require a fresh output path

Creating packages requires a full Git checkout. Building an extracted Odin package does not require Git

## Tests

Tests require the full checkout, not an extracted Odin package or C SDK. Use the commands in your [platform guide](README.md#build-and-run). Odin test commands build their selected packages. C ABI consumer and semantic checks require matching libraries to be built first

On Linux, the full Odin suite and the focused `physics-visuals` scope require linkable/runtime LZ4, provided by `liblz4-dev` on Ubuntu, even when no GUI is opened. Graphics libraries and a desktop context are required only when building and opening the viewer

## ABI and documentation generation

The Odin tool in `tools/abi/generate_abi` reads `tools/abi/abi_manifest.json` and produces C headers, exports, Odin bindings, layout assertions, and C reference pages

| Action                           | Linux                                              | Windows PowerShell 7                                   |
| -------------------------------- | -------------------------------------------------- | ------------------------------------------------------ |
| Check generated files            | `./scripts/linux/abi/generate_abi.sh --mode check` | `& .\scripts\windows\abi\generate_abi.ps1 -Mode Check` |
| Regenerate after a manifest edit | `./scripts/linux/abi/generate_abi.sh --mode write` | `& .\scripts\windows\abi\generate_abi.ps1 -Mode Write` |

C ABI builds check these files automatically. Edit the manifest or renderer when changing generated content

The Odin guides and [declaration index](odin/reference/API-INDEX.md) are maintained with their source declarations and examples. `./scripts/format_docs.sh` formats Markdown tables from Bash across root pages, plans, guides and result reports. It has no single-file option

## Update locked tool versions

Check available releases:

```bash
./scripts/linux/toolchains/update_toolchains.sh --tool all --mode check
```

```powershell
& .\scripts\windows\toolchains\update_toolchains.ps1 -Tool All -Mode Check
```

Use `update` instead of `check` to change the declarations. An intentional compiler update requires rebuilding and revalidating the affected source and consumer paths

## Failure recovery

An invalid explicit tool path stops immediately. Correct that selection before retrying

Download and extraction failures report an `INSTALL_FAILED partial=...` directory. Keep it while diagnosing the missing prerequisite, checksum, path, or network failure, then rerun the original command after repair
