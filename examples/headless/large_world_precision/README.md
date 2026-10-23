# Large-world coordinates

This example places a body at application position `(1 Gm + 2 mm, -1 Gm, 1 Gm)`, shifts the simulation origin, steps once and checks that the reconstructed application position differs by at most 1 mm on each axis

Float32 subtracts a caller-owned f64 origin before authoring a local body. Mixed64 authors the simulation-frame f64 position directly, queries it with a global ray and exports its pose before shifting

## Run

Run from the repository or extracted Odin source-package root. The runner builds the selected example before executing it

Linux:

```bash
./scripts/linux/examples/run_examples.sh --package large_world_precision --configuration release --precision float32
./scripts/linux/examples/run_examples.sh --package large_world_precision --configuration release --precision mixed64
```

Windows PowerShell 7:

```powershell
& .\scripts\windows\examples\run_examples.ps1 -Package large_world_precision -Configuration Release -Precision Float32
& .\scripts\windows\examples\run_examples.ps1 -Package large_world_precision -Configuration Release -Precision Mixed64
```

Success prints `LARGE_WORLD_PRECISION_OK` with the profile, application position and origin, followed by `RUN_EXAMPLES_OK`. Build requirements are in [Building Entasis](../../../docs/BUILDING.md)

## Coordinate ownership

The engine shifts its positions, built-in absolute targets and bounds. Prepare updates to external absolute state before maintenance, then apply them only after `world_shift_origin` succeeds. This example owns no external gravity center or queued command state

See [world precision](../../../docs/odin/reference/WORLD.md#precision-profiles-and-origin-frames) for shift ownership and [position views and snapshots](../../../docs/odin/reference/DATA-ACCESS.md#precision-and-borrowed-position-views) for f64 authority, float caches and captured state
