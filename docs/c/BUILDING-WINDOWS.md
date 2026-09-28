# C ABI builds on Windows

Run these commands in PowerShell 7 from the root of the [full repository checkout](https://github.com/Abyssal-Engine/Entasis). They build engine libraries from Odin source and are not available inside the prebuilt C SDK. To use an SDK, follow its root README instead

## Requirements

Use the [Windows prerequisites](../BUILDING.md#windows) and [toolchain selection rules](../BUILDING.md#toolchain-selection)

## Build libraries

Release:

```powershell
& .\build_c_abi_windows.ps1
```

Development:

```powershell
& .\build_c_abi_windows.ps1 -Configuration Development
```

| Library | Shared                | Import library        | Static                       |
| ------- | --------------------- | --------------------- | ---------------------------- |
| Runtime | `entasis.dll`         | `entasis.lib`         | `entasis_static.lib`         |
| Cooking | `entasis_cooking.dll` | `entasis_cooking.lib` | `entasis_cooking_static.lib` |

Outputs are written to `build/c_abi/windows/<configuration>/`. Success prints `C_ABI_BUILD_OK` after checking generated files and exported symbols

Development also produces PDBs

## Test C and C++ consumers

After building Release, start with the runtime and cooking smoke consumers:

```powershell
& .\scripts\windows\tests\test_c_abi_consumers.ps1 -Configuration Release -Compiler Msvc -Scope Smoke
```

For all consumer fixtures and direct-Odin comparisons with both MSVC and clang-cl:

```powershell
& .\scripts\windows\tests\test_c_abi_consumers.ps1 -Configuration Release -Compiler All -Scope All
& .\scripts\windows\tests\verify_c_abi_semantics.ps1 -Configuration Release -Compiler All -Scope All
```

Use `-Compiler Msvc` or `-Compiler ClangCl` for one toolchain. Use `-Configuration Development` after building Development libraries

Consumer tests compile C11/C++20 shared/static programs. `All` also compiles every public header and generated layout assertions. To check one domain, select `Smoke`, `Lifecycle`, `Queries`, `Callbacks`, `Stages`, `Memory`, `Custom-Shapes`, `Custom-Tasks`, `Custom-Constraints`, `Extensions`, `Allocators`, `Triggers`, `Body-Control` or `Restitution`. `Extensions` selects joint reaction/provider groups and the extension semantic C fixture. `Triggers` includes mixed collider parts and nested shapes

Semantic tests compare exact direct-Odin, shared-C and static-C output. They accept `All`, `Queries`, `Custom-Shapes`, `Custom-Tasks`, `Custom-Constraints` or `Extensions`. Query checks include the query-context variant

Success prints `C_ABI_CONSUMERS_OK` or `C_ABI_SEMANTICS_OK`. Executables and comparison output remain under the matching `build/c_abi/windows/<configuration>/` directory

`-Scope Compatibility -BaselineRoot <repository-relative-directory>` uses retained shared executables and separately links retained-header static consumers against current libraries. The baseline must contain matching headers, test sources, libraries and executables. `-CompatibilityFixtures Original` is the default historical fixture set with `build/c_abi/windows/<configuration>/consumers/<lane>/` executables. Select `Cumulative` for the current All fixture set retained under `consumers/all/<lane>/`

Compatibility also checks matching runtime/cooking pairs and rejection of deliberately mixed pairs across the private world-layout change. It cannot replace missing retained shared executables with rebuilt ones. Build and ship runtime and cooking libraries together even when their public ABI versions match

## Package the SDK

After a successful Release build:

```powershell
& .\scripts\windows\sdk\package_windows_sdk.ps1
```

The default destination is `build/sdk/windows/`. Success prints `WINDOWS_SDK_OK` and the archive path. Select another directory with `-OutputRoot <path>`. Existing package outputs are rejected. The packager verifies archived files against the install tree

## Select Visual Studio

Use [shared Visual Studio selection](../BUILDING.md#visual-studio-selection) for a persistent choice, or pass `-VsDevCmd` to the C ABI build and consumer commands for one invocation

## Portable tool overrides

| Command        | Optional parameters           |
| -------------- | ----------------------------- |
| C ABI build    | `-OdinExe`, `-LlvmReadObjExe` |
| Consumer tests | `-ClangClExe`                 |
| Semantic tests | `-OdinExe`, `-ClangClExe`     |
