# Physics viewer

A small Odin application for live examples, saved benchmark results and recording playback, using raylib/raygui and [DejaVu Sans](assets/DejaVuSans-LICENSE.txt)

The viewer is optional and must be built from the full repository checkout. The C SDK and Odin source archives do not include its sources, commands or binaries

## Build and open

Install the [optional visual prerequisites](../../docs/BUILDING.md#optional-visual-tools), then run from the repository root

```powershell
./scripts/windows/visuals/build_physics_viewer.ps1 -Configuration development
./scripts/windows/visuals/run_physics_viewer.ps1
./scripts/windows/visuals/run_physics_viewer.ps1 -Scenario example/falling_box
```

```sh
./scripts/linux/visuals/build_physics_viewer.sh --configuration development
./scripts/linux/visuals/run_physics_viewer.sh --scenario example/falling_box
```

The viewer opens on Benchmarks by default. `--scenario example/falling_box` (PowerShell `-Scenario example/falling_box`) opens an example directly

## Saved benchmark results

Choose a case and saved run on the Benchmarks tab without simulating physics. Rows show the run date in UTC, name and build. Use All, With recordings or Without recordings to filter the list. A run with at least one remaining associated recording appears under With recordings. Missing recordings and recording Off runs appear under Without recordings. Runs whose recording status cannot be read remain visible under All. Refresh rechecks files changed outside the viewer. Select a worker count, measurement and individual sample, then Play recording when available. Open report opens the detailed report. Back preserves the selected run and sample

Delete all recordings in the top bar cleans recordings across saved runs and cases with one confirmation, regardless of the current filter. It searches `results/`, `build/benchmark-results/windows_amd64/` and `build/benchmark-results/linux_amd64/`. Review the file/run counts, size and scrollable details before confirming. Unreadable runs and unsafe targets are listed as excluded. Pending/failed runs, partial captures and unassociated files are retained

Delete recordings on Results still offers the selected sample or all recordings in that run, including an explicitly opened external run. All deletion scopes retain CSV measurements, reports and original recording/timing conditions. Cancel or Escape before removal changes no files. Stop during removal leaves completed deletions in place. Locked files are reported while cleanup continues with the remaining files. Workspace cleanup and failed per-run cleanup keep their outcomes visible until Close, with every failed filename and reason in the scrollable details. Deleting the last recording moves the run to Without recordings while keeping its numerical Results open

Cleanup opened over setup preserves the form. Cleanup opened over replay pauses it, and cancelling resumes its previous state. Confirmed cleanup closes a replay/comparison view that holds a target recording before removing that file

Recording and Timing identify the saved measurement conditions. Total physics time covers all measured steps, and Mean per step divides it by the saved step count. Missing legacy conditions remain unavailable. Legacy CSVs and replay v1/v2/v3 remain readable

Runs are discovered under the selected package's `build/benchmark-results/<host>/` and `results/` roots on entry or Refresh. The viewer has no filesystem browser. Use CLI `--results` for an explicit path. Raw overhead files require package `c_abi_overhead/native`, `c_abi_overhead/c` or `c_abi_overhead` when the origin is unavailable

## Examples

The Examples tab separates 13 finite motion cases and 13 diagnostics. Selecting an example opens it directly. Motion opens paused with Play/Pause, Step, Reset and speed. Controllable examples offer Scripted/WASD, with a session reset when changing input mode. Diagnostics use Next operation. Multi-trial playback pauses at the next trial's Reset observation before advancing its physics. Examples have no recording controls

Nine correctness-first examples remain available through headless routes and explicit example CLI/capture, outside the graphical chooser. Example recipes are supported by explicit `--recipe PATH`; the viewer has no generic recipe editor

| Input | Action |
| --- | --- |
| Space | Play/pause motion, restart a finished replay, next operation for live diagnostics |
| Right / Left | Next observation / previous recorded observation |
| R / Home | Reset the example or replay |
| End | Last replay frame |
| Escape | Close the open dialog, or return to the list |
| F | Fit visible content |
| Right drag / middle drag / wheel | Look around / pan / zoom |
| Alt + left drag / Shift + right drag | Orbit the surface under the pointer / pan |
| Left click | Focus the scene or comparison pane |
| WASD / Q,E | Move the camera / move down and up |
| Shift | Move the camera faster |
| WASD with Controls set to WASD | Camera-relative character movement or authored vehicle/tank controls |

Text fields, dialogs, lost focus and paused playback block controller input. WASD controls the example during playback when that mode is selected, otherwise it moves the camera. Alt + left drag orbits the surface under the pointer, or the scene centre over empty space. A camera drag stays with its starting pane. Scene clicks focus a pane without selecting objects

Dynamic bodies are orange. Container walls and floor show only outlines so the bodies inside remain visible. Other scenes show solid static geometry

## Benchmark launch and recordings

Run new exposes thread counts, repeats and applicable shape/workload settings. Click the Thread counts summary to open its checkbox grid. Select all selects every available count, Select none clears it, and each tile toggles one count. Each selected count runs separately with the chosen number of repeats. The selection is limited to the available CPUs and the existing 64-worker limit. An empty selection disables both launch actions

Selections apply immediately. Done, Escape or clicking outside closes the chooser and retains them. Arrow keys move focus, Space toggles a count, and Tab/Shift+Tab reach the bulk actions and Done. The form shows the selected counts and repeats before launch. Fresh and reset forms select one worker

The case list and Benchmarks/Examples tabs remain usable during setup. Selecting another case keeps setup open and retains thread counts, repeats and Save replay, while loading that case's workload defaults. Cases without recording support produce numerical results only and retain the Save replay preference for returning to a supported case. Selecting the same case or pressing Run new again keeps the draft. Examples exits setup, and returning to Benchmarks shows saved runs. Navigation never starts a benchmark. Open confirmation dialogs and choosers temporarily own input

Save replay defaults Off for fresh and reset launches. On is available for container, pyramid, ragdoll stair tumble and the two noncontact constraint cases. Contact Islands offers Save baseline replay because its extra component measurements are not recorded. Other cases produce numerical results only

Run again restores known saved workload values and recording mode. Complete any missing legacy settings before launching. Reset defaults restores the package defaults, including recording Off. Static shape accepts Box or Hull, while dynamic Shape offers Box, Sphere, Capsule, Cylinder and Hull. Fields support Ctrl+A/C/X/V, Tab/Shift+Tab, Home/End, arrows and Shift selection. Enter accepts an edit and Escape restores its previous value

- **Build and benchmark:** compile current source, then run the selected workload after successful compilation
- **Benchmark:** run prepared binaries directly. It does not check source changes or build automatically. Choose Build and benchmark after source/toolchain changes or when binaries are missing

Both actions validate the settings and keep the same window open with the selected case, current stage, latest diagnostic and Total elapsed, including host startup and compilation. Cancel or Escape stops the launched work and retains partial output. Closing the application also stops it. GUI launches have no timeout

Success opens the completed result. Failure shows the exit or OS error and diagnostic output, with Back to settings for correction. Open log opens `build/physics-viewer/runs/<attempt>/launch.log`, which contains the full build/run output. Each attempt is retained separately

Recordings capture the actual measured world. Recording can affect cache, memory, total runtime and the timing method, so compare samples with matching conditions. See [playback recording and CSV timing fields](../../BENCHMARK.md#optional-playback-recording) for measurement details

## Replay

Select an individual benchmark sample to replay its associated recording without running physics. A median or sample without an association has no replay. An associated file that has been deleted shows Recording missing and is excluded from the comparison picker. With recordings means file presence, so a corrupt recording can still appear there and can be deleted, but playback admission continues to reject it. With no association, saved Off shows Recording off for supported cases or Replay unsupported for other cases. Saved On shows Not recorded for this measurement, and an unknown saved mode shows Replay unavailable. Replay offers Loop Off/On

Sample Replay and saved-run Compare check the exact worker, sample and measurement association. Linked files, incomplete recordings and worker, measured-extent or supplied-workload mismatches are rejected while keeping the current panes and selected result

Compare requires matching nonempty case IDs, axes and timesteps. Recordings with different settings remain comparable. Panes A/B show whether settings and source metadata match, differ or are unavailable. Hover Settings or Source to inspect recorded values and scroll longer settings. Missing legacy metadata remains unavailable

Linked playback aligns cycle, phase and tick over the common range. Unlink playback to inspect unequal tails. Camera linking is independent. Playback/camera links and Close comparison appear above the scenes

## CLI entry points

Use `--replay`, `--compare`, `--results`, `--scenario` or `--recipe` to open content directly. `--frame`, `--window-size` and `--memory-mib` control the initial frame, window and memory budget. `--screenshot PATH` exports a PNG and exits, refusing an existing destination. There is no GUI screenshot export
