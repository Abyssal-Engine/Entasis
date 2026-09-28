# Odin builds on Linux

Run native build and example commands from the repository root or extracted Odin package root

## Requirements

Follow the shared [platform prerequisites](../BUILDING.md#linux), [tool selection](../BUILDING.md#toolchain-selection) and [profiles](../BUILDING.md#build-profiles)

## Run a falling box

This command builds the example, creates a world with gravity and a floor, then runs the simulation:

```bash
./scripts/linux/examples/run_examples.sh --package falling_box --configuration development
```

Success prints a settled height near `0.5` and `RUN_EXAMPLES_OK`. The executable is `build/examples/development/falling_box`. Choose another scene from the [example catalog](EXAMPLES.md)

## Build examples

Build every example in Development:

```bash
./build_odin_linux.sh
```

Build one example in Release:

```bash
./build_odin_linux.sh --package falling_box --configuration release
```

| Option            | Values                                          | Default                   |
| ----------------- | ----------------------------------------------- | ------------------------- |
| `--package`       | Any package in [headless examples](EXAMPLES.md) | Every example             |
| `--configuration` | `development`, `release`, `all`                 | `development`             |
| `--odin`          | Absolute path to a matching Odin executable     | Toolchain selection rules |

Outputs are written to `build/examples/<configuration>/`. A successful build prints `BUILD_EXAMPLES_OK` without running the scene

## Run examples

Build and run every example in both profiles:

```bash
./scripts/linux/examples/run_examples.sh --configuration all
```

Use `--package` to select one example. Add `--skip-build` only when its executable already matches the current source and requested profile. Otherwise the command builds before running

Run the whole-collider and mixed solid/sensor trigger example in both profiles:

```bash
./scripts/linux/examples/run_examples.sh --configuration all --package triggers
```

## Run tests

Tests require the [full repository checkout](https://github.com/Abyssal-Engine/Entasis). The extracted Odin package does not include tests or test scripts. Run the following commands from the full checkout root

Public facade tests:

```bash
./scripts/linux/tests/test.sh --scope public-api --configuration development
```

Complete Development and Release test packages plus codegen checks:

```bash
./scripts/linux/tests/test_all.sh --configuration all
```

The test runner also accepts `--tests` for a selector list and `--threads` for the Odin test thread count

## Use Entasis in another repository

Follow [Import Entasis](GETTING-STARTED.md#import-entasis) for the source collection and public package. Apply the target and profile settings from [shared build settings](../BUILDING.md#build-profiles)
