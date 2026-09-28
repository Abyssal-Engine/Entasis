# Building the Odin package

Run example build and run commands from the extracted package root. This package compiles Entasis directly into Odin applications and does not include C ABI build scripts, tests or benchmarks

## Requirements

Supported targets are Linux and Windows on AMD64, the x86-64 CPU architecture. Compiled applications require the `x86-64-v3` instruction-set level, including AVX2

### Linux

- Bash, curl, GNU coreutils, tar, gzip, xz, unzip and `dpkg-deb`
- Native C development headers and libraries, plus binutils, for platform linking

Build commands obtain Odin and LLVM. LLVM uses a private pinned ICU runtime. WSL builds Linux executables

### Windows

- Native x64 PowerShell 7
- Visual Studio or Build Tools with **Desktop development with C++**
- MSVC x64/x86 tools and a Windows SDK

Scripts obtain Odin and load the Visual Studio x64 environment for platform linking

## Toolchain selection

Versions and checksums are declared in [`tools/toolchains.lock`](../tools/toolchains.lock)

Selection order:

1. An explicit tool argument supported by the command
2. A matching entry in `toolchains.local` at the package root
3. The declared distribution under `build/toolchains/prebuilt`

The first download requires HTTPS access. Matching cached distributions are reused offline. Select complete distributions matching the declarations, using literal `key=value` entries with absolute paths and no quotes or shell escaping:

```text
linux-amd64.roots.odin=/opt/odin
linux-amd64.roots.llvm=/opt/llvm
windows-amd64.roots.odin=C:/Tools/Odin
windows-amd64.system.vsdevcmd=C:/BuildTools/Common7/Tools/VsDevCmd.bat
```

### Visual Studio selection

Windows scripts discover Visual Studio through `vswhere.exe`. Use `windows-amd64.system.vsdevcmd` to select an installed developer command. Odin example commands do not accept `-VsDevCmd`

## Build profiles

| Profile | Odin flags | Purpose |
| --- | --- | --- |
| Development | `-debug -o:none -source-code-locations:normal` | Debugging with assertions and bounds checks |
| Release | `-o:speed -no-bounds-check -disable-assert -source-code-locations:none` | Optimized builds |

Supplied builds target `linux_amd64` or `windows_amd64` with `-microarch:x86-64-v3`. Examples default to Development. Follow the [Linux](odin/BUILDING-LINUX.md) or [Windows](odin/BUILDING-WINDOWS.md) guide for native build and run commands

Builds replace previous outputs in `build/examples/<configuration>/`. `BUILD_EXAMPLES_OK` reports compilation success. The example runner also executes the scene and prints `RUN_EXAMPLES_OK` on success
