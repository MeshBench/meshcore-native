# Provenance

Binaries released from this repository are built from
[meshcore-dev/MeshCore](https://github.com/meshcore-dev/MeshCore), which is MIT
licensed, copyright Scott Powell / rippleradios.com. Their applications are
compiled **unmodified**; everything this repository adds is the host variant in
`variants/host/` and the bridge in `bridge/`, also MIT.

The SX1262 model is no longer here. It is
[MeshBench/virtual-sx1262](https://github.com/MeshBench/virtual-sx1262), MIT,
vendored as a submodule at `vendor/virtual-sx1262` and compiled into these
binaries. QEMU and Renode load the same model from that repository's own
releases, so a node built here and an emulated board are the same chip from the
same commit: two copies of a chip model drift, and the moment they do, a
comparison between a native node and an emulated one is measuring our code
rather than MeshCore's.

The commit each release was built from is recorded in that release's body.

Third-party code compiled in:

| what | source | licence |
|---|---|---|
| MeshCore | meshcore-dev/MeshCore | MIT |
| Crypto | rweather/arduinolibs | MIT |
| ed25519 | vendored in MeshCore under `lib/ed25519` | as MeshCore ships it |

The Cayenne LPP constants in `variants/host/CayenneLPP.h` and the base64
implementation in `variants/host/base64.hpp` are ours, written against published
formats rather than copied, so that a build does not need the network to resolve
values that have not changed in a decade.

## Why this is a separate repository

The simulator these builds run under has not chosen a licence. Linking MeshCore
constrains that choice, so the linking happens here, under MeshCore's own terms,
and the simulator downloads the result.
