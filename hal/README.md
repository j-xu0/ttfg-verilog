# tinystep HAL

Host-side C driver. `inc/` public headers, `src/` platform-free core,
`platform/` one file per target (selected at link time), `test/`, `examples/`.

```sh
cmake -S hal -B hal/build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON   # configure
cmake --build hal/build                                        # build
ctest --test-dir hal/build --output-on-failure                 # test
./hal/build/bringup_example                                   # example
```

Port to a new target by adding `platform/tinystep_platform_<name>.c` and configuring with
`-DTINYSTEP_PLATFORM=<name>` (default `host`). The default host platform is a simulated chip
(`platform/tinystep_platform_host.h`) that speaks the same frame protocol as the RTL, so the
tests run with no hardware.

Settled design decisions are in [DESIGN.md](DESIGN.md).

`compile_commands.json` is generated in `hal/build/` and symlinked to `hal/`; both are gitignored.
