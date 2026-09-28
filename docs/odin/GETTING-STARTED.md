# Getting started with Odin

Entasis compiles directly into an Odin application. No separate engine library is required for the Odin path

## Requirements

Follow the [shared requirements and toolchain setup](../BUILDING.md#requirements) for your platform

## Run a falling box

From the root of an extracted `odin-source` package or full checkout, run:

```sh
# Linux
./scripts/linux/examples/run_examples.sh --package falling_box --configuration development
```

```powershell
# Windows PowerShell 7
& .\scripts\windows\examples\run_examples.ps1 -Package falling_box -Configuration Development
```

The runner builds [falling_box](../../examples/headless/falling_box/main.odin) into `build/examples/development/` and runs it. Success prints a settled height near `0.5` and `RUN_EXAMPLES_OK`

## Import Entasis

Install the matching Odin distribution described in [toolchain selection](../BUILDING.md#toolchain-selection). Map the repository or extracted Odin package's `src` directory as the `entasis` collection when compiling your application:

```text
-collection:entasis=/absolute/path/to/Entasis/src
```

Then import the public facade:

```odin
import entasis "entasis:entasis"
```

Use the target and optimization settings in [build profiles](../BUILDING.md#build-profiles). For example, from your own application directory containing `main.odin`, a Linux Development build is:

```sh
odin run . -collection:entasis=/absolute/path/to/Entasis/src -target:linux_amd64 -microarch:x86-64-v3 -debug -o:none
```

Replace the collection path with your extracted source location. On Windows use `-target:windows_amd64` from a shell with the Visual Studio x64 environment loaded. This compiles and runs your application, not a separate Entasis library

## Smallest complete world

This program creates an empty world, advances it by one fixed timestep and releases its storage. It prints `minimal world stepped` on success. The supplied [minimal_world](../../examples/headless/minimal_world/main.odin) program demonstrates the same lifecycle

```odin
package main

import "core:fmt"
import "core:os"
import entasis "entasis:entasis"

main :: proc()
{
	world: entasis.World;
	description := entasis.world_description_default();
	init_status := entasis.world_init(&world, description);
	if init_status != .Ok
	{
		fmt.eprintln("world_init failed:", init_status);
		os.exit(1);
	}

	step_status := entasis.world_step(&world, 1.0 / 60.0);
	destroy_status := entasis.world_destroy(&world);
	if step_status != .Ok || destroy_status != .Ok
	{
		fmt.eprintln("world_step:", step_status, "world_destroy:", destroy_status);
		os.exit(1);
	}
	fmt.println("minimal world stepped");
}
```

`World` must start zero initialized, remain at a stable address while ready, and be destroyed after all work using it has stopped

## Choose a package

| Package                     | Purpose                                                                       |
| --------------------------- | ----------------------------------------------------------------------------- |
| `entasis:entasis`           | Stable world, shape, body, constraint, query, callback, and inspection facade |
| `entasis:entasis_cooking`   | Hull, mesh, and compound preparation outside a live world                     |
| `entasis:entasis_physics`   | Advanced low-level simulation access                                          |
| `entasis:entasis_utilities` | Low-level memory, collection, and threading utilities                         |

Start with `entasis:entasis`. Use the lower-level packages only when the facade does not expose the required integration point

## Next steps

Add a [falling body and floor](../../examples/headless/falling_box/main.odin), then follow [engine integration](INTEGRATION.md) for an application loop. The [documentation map](../README.md) links every domain reference and the example catalog
