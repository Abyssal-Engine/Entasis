# Odin builds on Windows

Run native build and example commands in PowerShell 7 from the repository root or extracted Odin package root

## Requirements

Follow the shared [platform prerequisites](../BUILDING.md#windows), [tool selection](../BUILDING.md#toolchain-selection) and [profiles](../BUILDING.md#build-profiles)

## Run a falling box

This command builds the example, creates a world with gravity and a floor, then runs the simulation:

```powershell
& .\scripts\windows\examples\run_examples.ps1 -Package falling_box -Configuration Development
```

Success prints a settled height near `0.5` and `RUN_EXAMPLES_OK`. The executable is `build/examples/development/falling_box.exe`. Choose another scene from the [example catalog](EXAMPLES.md)

## Build examples

Build every example in Development:

```powershell
& .\build_odin_windows.ps1
```

Build one example in Release:

```powershell
& .\build_odin_windows.ps1 -Package falling_box -Configuration Release
```

| Option           | Values                                          | Default                   |
| ---------------- | ----------------------------------------------- | ------------------------- |
| `-Package`       | Any package in [headless examples](EXAMPLES.md) | Every example             |
| `-Configuration` | `Development`, `Release`, `All`                 | `Development`             |
| `-OdinExe`       | Absolute path to a matching Odin executable     | Toolchain selection rules |

Outputs are written to `build/examples/<configuration>/` with the `.exe` suffix. A successful build prints `BUILD_EXAMPLES_OK` without running the scene

## Run examples

Build and run every example in both profiles:

```powershell
& .\scripts\windows\examples\run_examples.ps1 -Configuration All
```

Use `-Package` to select one example. Add `-SkipBuild` only when its executable already matches the current source and requested profile. Otherwise the command builds before running

## Run tests

Tests require the [full repository checkout](https://github.com/Abyssal-Engine/Entasis). The extracted Odin package does not include tests or test scripts. Run the following commands from the full checkout root

Public facade tests:

```powershell
& .\scripts\windows\tests\test.ps1 -Scope public-api -Configuration Development
```

Complete Development and Release test packages plus codegen checks:

```powershell
& .\scripts\windows\tests\test_all.ps1 -Configuration All
```

Use `-Tests` to select named tests and `-Threads` to set the Odin test thread count

## Visual Studio selection

Use the [shared Visual Studio setting](../BUILDING.md#visual-studio-selection). Odin commands do not accept `-VsDevCmd`

## Use Entasis in another repository

Follow [Import Entasis](GETTING-STARTED.md#import-entasis) for the source collection and public package. Apply the target and profile settings from [shared build settings](../BUILDING.md#build-profiles)
