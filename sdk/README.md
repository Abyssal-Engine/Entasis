# Entasis C/C++ SDK

The `c-sdk` provides prebuilt runtime and cooking libraries for C, C++ and other C-interop languages. See [SDK build settings](share/doc/Entasis/BUILDING.md) for requirements, tool selection and installed paths, and the [C and C++ guide](share/doc/Entasis/c/README.md) for API usage

## Start here

Run commands from the extracted SDK root. Both runners use CMake/Ninja in Release mode to build and run four consumers: C11 and C++20, each with shared and static linkage. Each checks library versions, initializes a world and destroys it. They link runtime and cooking libraries but do not cook geometry

Output goes to `build/examples/<compiler>/`. Success reports four passing CTest tests per compiler and `SDK_EXAMPLES_OK`

### Linux

Install the [Linux prerequisites](share/doc/Entasis/BUILDING.md#linux)

```bash
./scripts/linux/sdk/run_sdk_examples.sh --compiler clang
```

Use `--compiler gcc` for the installed GCC/G++ pair or `--compiler all` for both

### Windows

Install the [Windows prerequisites](share/doc/Entasis/BUILDING.md#windows)

```powershell
& .\scripts\windows\sdk\run_sdk_examples.ps1 -Compiler Msvc
```

Use `-Compiler ClangCl` for clang-cl or `-Compiler All` for both. The script loads the Visual Studio x64 environment and copies DLLs beside shared consumers

## CMake integration

```cmake
cmake_minimum_required(VERSION 3.20)
project(my_application LANGUAGES C CXX)

find_package(Entasis CONFIG REQUIRED)

add_executable(my_application main.cpp)
target_compile_features(my_application PRIVATE cxx_std_20)
target_link_libraries(my_application PRIVATE Entasis::runtime)
```

The fragment above assumes your application supplies `main.cpp`. From that application's root, configure CMake with the SDK path:

```text
-DEntasis_DIR=<SDK>/lib/cmake/Entasis
```

Available targets:

| Library              | Shared             | Static                    |
| -------------------- | ------------------ | ------------------------- |
| Runtime              | `Entasis::runtime` | `Entasis::runtime_static` |
| Cooking with runtime | `Entasis::cooking` | `Entasis::cooking_static` |

Cooking prepares hulls, meshes and compounds. Its targets include the runtime dependency. Follow the [matching-library and ownership rules](share/doc/Entasis/c/API.md#cooking)

On Windows, copy `$<TARGET_FILE:Entasis::runtime>` beside a shared consumer. A cooking consumer also needs `$<TARGET_FILE:Entasis::cooking>`. The shipped [CMake example](examples/CMakeLists.txt) shows both shared and static linkage to these libraries. [minimal.c](examples/minimal.c) and [minimal.cpp](examples/minimal.cpp) contain the version and world-lifecycle checks

Stable packages satisfy equal or older requests within the same major version. `EXACT` requires numeric equality. Prerelease packages support only the unversioned lookup shown above

## pkg-config on Linux

Add `<SDK>/lib/pkgconfig` to `PKG_CONFIG_PATH`

```bash
pkg-config --cflags --libs entasis
pkg-config --cflags --libs entasis-cooking
```

For static linking, add `--static` and replace `-lentasis_cooking` and `-lentasis` with absolute paths to `libentasis_cooking.a` and `libentasis.a`, in that order. `--static` supplies private system-library flags and `--allow-multiple-definition` for overlapping support symbols. It does not force archive selection. The CMake static targets handle these requirements automatically

## Rebuild the engine

The SDK runners build consumer applications, not engine libraries. Rebuilding the C ABI requires the [full repository](https://github.com/Abyssal-Engine/Entasis). The included [Linux](share/doc/Entasis/c/BUILDING-LINUX.md) and [Windows](share/doc/Entasis/c/BUILDING-WINDOWS.md) source-build guides describe commands to run from that checkout, not from this SDK

## License

Licensing and attribution are in [LICENSE](share/licenses/Entasis/LICENSE) and [NOTICE](share/licenses/Entasis/NOTICE)
