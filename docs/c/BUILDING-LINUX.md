# C ABI builds on Linux

Run these commands from the root of the [full repository checkout](https://github.com/Abyssal-Engine/Entasis) on Linux or WSL. They build engine libraries from Odin source and are not available inside the prebuilt C SDK. To use an SDK, follow its root README instead

## Requirements

Use the [Linux prerequisites](../BUILDING.md#linux) and [toolchain selection rules](../BUILDING.md#toolchain-selection)

## Build libraries

Release:

```bash
./build_c_abi_linux.sh
```

Development:

```bash
./build_c_abi_linux.sh --configuration development
```

| Library | Shared                  | Static                 |
| ------- | ----------------------- | ---------------------- |
| Runtime | `libentasis.so`         | `libentasis.a`         |
| Cooking | `libentasis_cooking.so` | `libentasis_cooking.a` |

Outputs are written to `build/c_abi/linux/<configuration>/`. Success prints `C_ABI_BUILD_OK` after checking generated files and exported symbols

## Test C and C++ consumers

After building Release, start with the runtime and cooking smoke consumers:

```bash
./scripts/linux/tests/test_c_abi_consumers.sh --configuration release --compiler clang --scope smoke
```

For all consumer fixtures and direct-Odin comparisons with both Clang and GCC:

```bash
./scripts/linux/tests/test_c_abi_consumers.sh --configuration release --compiler all --scope all
./scripts/linux/tests/verify_c_abi_semantics.sh --configuration release --compiler all --scope all
```

Use `--compiler gcc` or `--compiler clang` for one compiler pair. Use `--configuration development` after building Development libraries

| Command                     | Available scopes | Coverage                                                                                                                                                                                    |
| --------------------------- | ---------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `test_c_abi_consumers.sh`   | `smoke`, `all`   | C11/C++20 shared/static consumers, including custom shapes, tasks, constraints, triggers, allocators and extensions. `all` also compiles each public header and generated layout assertions |
| `verify_c_abi_semantics.sh` | `all`, `queries` | Exact C/direct-Odin output comparison for core, scene, dynamics, queries, custom shapes, custom tasks, custom constraints and extensions, including shared/static query-context variants    |

`smoke` runs only the runtime C11/C++20 and cooking C11 smoke fixtures. `queries` compares only query output, including caller-owned query contexts. The Linux commands have no retained-binary compatibility scope

Success prints `C_ABI_CONSUMERS_OK` or `C_ABI_SEMANTICS_OK`. Consumer executables and comparison output remain under the matching `build/c_abi/linux/<configuration>/` directory

## Package the SDK

After a successful Release build:

```bash
./scripts/linux/sdk/package_linux_sdk.sh
```

The default destination is `build/sdk/linux/`. Success prints `LINUX_SDK_OK` and the archive path. Select another directory with `--output-root <path>`. Existing package outputs are rejected
