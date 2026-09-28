# C and C++ getting started

Entasis supports C11 and C++20 through its runtime and cooking libraries

## Run an SDK example

Download the platform `c-sdk` from [Releases](https://github.com/Abyssal-Engine/Entasis/releases) and extract it. Install the [platform prerequisites](../BUILDING.md#requirements), then run from the extracted SDK root:

```sh
# Linux
./scripts/linux/sdk/run_sdk_examples.sh --compiler clang
```

```powershell
# Windows PowerShell 7
& .\scripts\windows\sdk\run_sdk_examples.ps1 -Compiler Msvc
```

The runner builds four consumers under `build/examples/<compiler>/`: C11 and C++20 with shared and static linkage. Each checks versions, initializes a world and destroys it. Success reports four passing CTest tests. These programs link both runtime and cooking libraries but do not cook geometry

The SDK's root README explains CMake and Linux pkg-config setup. Your application uses `Entasis::runtime` for simulation and `Entasis::cooking` when it also prepares geometry. Use `Entasis::runtime_static` or `Entasis::cooking_static` for static linkage. SDK consumers need a C/C++ compiler, not Odin

## Source build workflow

To build the libraries yourself, obtain the [full checkout](https://github.com/Abyssal-Engine/Entasis) and follow [Linux C ABI builds](BUILDING-LINUX.md) or [Windows C ABI builds](BUILDING-WINDOWS.md). These commands are not included in the prebuilt SDK

Source builds require Odin. The scripts obtain missing declared tools, check generated ABI files, compile shared and static runtime/cooking libraries, and verify exports. Rebuild after source changes before running consumer or semantic tests

Packaging uses the existing Release libraries. It does not rebuild them. The extracted SDK's root `README.md` has consumer commands, CMake targets and Linux pkg-config linking

## Common tasks

| Task                                                   | Guide                                                                                                         |
| ------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------- |
| Initialize, step and destroy a world                   | [Minimal lifecycle](API.md#minimal-lifecycle)                                                                 |
| Supply memory and external workers                     | [Allocators](API.md#allocators), [world threading](API.md#world-threading)                                    |
| Run concurrent queries                                 | [Read phases and query contexts](API.md#world-threading)                                                      |
| Find witnesses, penetration or a separation correction | [Distance queries and C11 example](API.md#closest-point-distance-and-separation)                              |
| Add sensors and consume events                         | [Trigger colliders](API.md#trigger-colliders)                                                                 |
| Mix solid and sensor parts on one instance             | [Mixed collider parts](API.md#mixed-collider-parts)                                                           |
| Inspect joint reactions and break overloaded joints    | [Breakable joints](API.md#breakable-joints), [combined fixed stepping](API.md#combined-fixed-step-delivery)    |
| Add bounce, forces, targets or axis locks              | [Restitution](API.md#restitution-bounciness), [body controls](API.md#optional-body-controls)                  |
| Register application-defined physics                   | [Shapes and collision tasks](API.md#custom-shape-payload-callbacks), [constraints](API.md#custom-constraints) |
| Prepare and import geometry                            | [Cooking ownership](API.md#cooking)                                                                           |

Read [opaque owner lifetimes](API.md#opaque-owner-handles) and [callback rules](API.md#callback-boundary) before retaining handles or passing application data into the library

Use the [header and reference map](API.md#headers-and-references) for exact declarations, fields, return values and failure behavior
