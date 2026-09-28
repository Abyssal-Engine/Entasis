# SDK build settings

These settings apply to the extracted Linux or Windows C SDK. Its root `README.md` contains example commands and CMake integration

## Requirements

The SDK targets AMD64, the x86-64 CPU architecture, with the `x86-64-v3` instruction-set level, including AVX2. Consumers need a C11 or C++20 toolchain. They do not need Odin

### Linux

- Bash, curl, GNU coreutils, tar, gzip, xz, and unzip
- Native C/C++ development headers and libraries, plus binutils
- `dpkg-deb` for the supplied Clang distribution's private ICU dependency
- GCC/G++ when building the consumers with GCC

The scripts obtain declared CMake and Ninja distributions, plus LLVM when selecting Clang. Downloaded LLVM dependencies remain in a private cache

### Windows

- Native x64 PowerShell 7
- Visual Studio or Build Tools with **Desktop development with C++**
- MSVC x64/x86 tools and a Windows SDK

CMake, Ninja, and optional clang-cl are provisioned from the package's tool declarations. clang-cl also requires the installed Visual Studio C++ toolset and Windows SDK

## Toolchain selection

Versions and checksums are declared in `tools/toolchains.lock` at the SDK root

Selection order:

1. An explicit tool argument supported by the command
2. A matching entry in `toolchains.local`
3. The declared distribution under `build/toolchains/prebuilt`

A first download requires HTTPS access. Matching cached distributions are reused offline

Use literal `key=value` entries with absolute paths:

```text
linux-amd64.roots.llvm=/opt/llvm
linux-amd64.system.gcc=/usr/bin/gcc
linux-amd64.system.g++=/usr/bin/g++
windows-amd64.system.vsdevcmd=C:/BuildTools/Common7/Tools/VsDevCmd.bat
```

Portable tools must come from complete distributions matching the declarations. Do not quote or shell-escape values

### Visual Studio selection

The Windows scripts discover Visual Studio through `vswhere.exe` and load an x64 developer environment

Use `windows-amd64.system.vsdevcmd` for a persistent choice. To select one installation for a consumer run:

```powershell
& .\scripts\windows\sdk\run_sdk_examples.ps1 `
    -Compiler Msvc `
    -VsDevCmd 'C:\BuildTools\Common7\Tools\VsDevCmd.bat'
```

These settings select installed components. Missing workloads and SDKs must be installed through Visual Studio Installer

## Package layout

| Content                     | Installed path                                |
| --------------------------- | --------------------------------------------- |
| Public headers              | `include/`                                    |
| Shared and static libraries | `lib/` on Linux, `bin/` and `lib/` on Windows |
| CMake package               | `lib/cmake/Entasis/`                          |
| Linux pkg-config files      | `lib/pkgconfig/`                              |
| Runnable consumers          | `examples/`                                   |
| C ABI documentation         | `share/doc/Entasis/c/`                        |
| Licenses                    | `share/licenses/Entasis/`                     |
