# Entasis Odin source

Compile Entasis directly into your Odin application. No separate Entasis library is required

## Start here

Follow [native build settings](docs/BUILDING.md) for prerequisites and tool selection. Run commands from the extracted package root

### Linux

```bash
./scripts/linux/examples/run_examples.sh --package falling_box --configuration release
```

### Windows

```powershell
& .\scripts\windows\examples\run_examples.ps1 -Package falling_box -Configuration Release
```

These commands build and run a floor-and-box scene. Success prints a settled height near `0.5` and `RUN_EXAMPLES_OK`. Executables remain under `build/examples/release/`. See the [example catalog](docs/odin/EXAMPLES.md) for other scenes

## Import Entasis

Map this package's `src` directory as an Odin collection:

```text
-collection:entasis=/absolute/path/to/Entasis/src
```

Import the public facade:

```odin
import entasis "entasis:entasis"
```

For an application compile command and package choices, see [getting started](docs/odin/GETTING-STARTED.md)

This package contains native engine source and examples. Tests, benchmarks, C ABI sources and maintainer tools require the [full repository](https://github.com/Abyssal-Engine/Entasis)

## Documentation

See the [documentation map](docs/README.md) for guides and API references

## License

See [LICENSE](LICENSE) and [NOTICE](NOTICE)
