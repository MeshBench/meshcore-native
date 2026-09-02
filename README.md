<picture>
  <source media="(prefers-color-scheme: dark)" srcset="brand/meshbench-banner-1600x400.png">
  <source media="(prefers-color-scheme: light)" srcset="brand/meshbench-banner-1600x400-light.png">
  <img alt="MeshBench: an RF-accurate MeshCore network simulator" src="brand/meshbench-banner-1600x400-light.png">
</picture>

# meshcore-native

Real [MeshCore](https://github.com/meshcore-dev/MeshCore) firmware, compiled for
desktop architectures, with its radio and its console on a socket.

These are not models of MeshCore. They are MeshCore's own applications —
`examples/simple_repeater`, `examples/companion_radio` and the rest — built
unmodified against a host *variant*, so the forwarding policy, the CSMA timing,
the CLI and the preferences are the firmware's and not a reimplementation of it.
A simulator that decides for itself which packets get relayed is answering a
different question from the one anybody asked.

## Where the real ends and the simulation begins

Worth being exact about, because the interesting bugs live on the boundary.

| Layer | What runs |
|---|---|
| MeshCore application and mesh logic | **real**, unmodified |
| MeshCore radio driver — `CustomSX1262`, `RadioLibWrapper` | **real**, unmodified |
| RadioLib 7.6.0 — the version MeshCore pins | **real**, vendored in `vendor/`, unmodified |
| The SX1262 chip | **ours** — `variants/host/VirtualSX1262` |
| Arduino, board, filesystem, RTC, sensors, RNG | **ours** — `variants/host/` |
| The air | whatever drives the bridge |

Nothing in MeshCore is patched. The build points at a checkout and compiles it as
it stands, with the flags its own `platformio.ini` sets — `RADIOLIB_GODMODE`
among them, because its driver reaches into RadioLib's internals.

### The radio, and why it moved

The radio driver used to be ours: `HostRadio`, a hand-written stand-in for the
whole stack. It worked, and it quietly answered questions the firmware should
have answered. MeshCore asks its radio "is another station transmitting?" before
it sends; `HostRadio` decided that, so listen-before-talk was the simulator's
behaviour rather than the firmware's — and two MeshCore versions differing *only*
in their radio driver produced bit-identical results.

Now RadioLib talks to a **virtual SX1262** over a `RadioLibHal`:

- `SimHal` — pins, SPI, time and interrupts. Sixteen methods, all trivial but
  `spiTransfer`, because RadioLib is written to be ported.
- `VirtualSX1262` — the chip: command interpreter, registers, data buffer, and
  the IRQ register, which is the point. `PREAMBLE_DETECTED` and `HEADER_VALID`
  are raised from what is actually on the air, at the instant each becomes true,
  and `CustomSX1262::isReceiving()` reads and times them exactly as on hardware.

Two deliberate departures from silicon:

- **Time is the node's, not the machine's.** `millis()` and `micros()` come from
  simulated time, because a lockstep run is reproducible only if everything the
  firmware can observe comes from the simulation.
- **BUSY is never asserted.** Command *latency* is modelled; the handshake is
  not. RadioLib spins on that line, and a spin that does not advance simulated
  time never ends — a hang the firmware cannot tell apart from a fast chip, and
  worth removing outright.

## The faulty variants

Alongside each release there is a `-faultyirq` build: the same firmware, on a
virtual radio that **misbehaves the way real ones do**.

Real SX1262s sometimes latch their detection interrupts and refuse to clear
them. A driver that trusts those flags then believes the channel is permanently
busy and stops transmitting — the "4 second lock-up" MeshCore's own release
notes describe, and the reason its 1.17 driver stopped trusting the flags and
started timing them out instead.

On a chip that behaves, 1.16 and 1.17 are indistinguishable. That is not a
guess: twelve runs across a 154-node mesh produced byte-identical results,
because the recovery path 1.17 added never had anything to recover from.

So the fault is a build:

    repeater-v1.17.0             a chip that behaves
    repeater-v1.17.0-faultyirq   a chip that latches its detection flags

Making it a variant rather than a runtime switch means an experiment can select
it exactly like any other firmware version, and "does this release survive a
radio that sticks?" becomes a question you can put in a matrix and measure.

The chip also reports what the firmware asked it: how many times the interrupt
register was read, how many of those reads found a busy flag set, how long the
flags were up, and how many preambles it raised. That is what separates "the
mesh is genuinely busy" from "our chip cries busy too readily" — and the second
would look exactly like a finding about the firmware.

## A node is a node

There is no list of supported node types here, and that is deliberate. A
MeshCore node is a radio in a place running an application; whether it is a
repeater, a companion, a room server or a sensor is settled entirely by which
application is linked. So the pipeline reads `examples/` at build time and builds
whatever it finds. When upstream ships a new kind of node, it appears in the next
nightly release without this repository changing.

The same goes for versions: "every previous release" is not a fixed list, it is
whatever upstream has tagged by the time the workflow runs.

## What gets released

| release | tracks |
|---|---|
| `main` | upstream's `main` branch, rebuilt when it moves |
| `dev` | upstream's `dev` branch, marked pre-release |
| `<tag>` | each upstream tag, built once and never rebuilt |

Each release carries one binary per role per platform, named
`meshcore-<role>-<os>-<arch>`:

| | x86 | x64 | arm64 |
|---|---|---|---|
| Linux | ✓ | ✓ | ✓ |
| Windows | ✓ | ✓ | ✓ |
| macOS | — | ✓ | ✓ |

macOS has had no 32-bit userland since Catalina, so there is nothing to build
there.

A release body contains a `meshcore-commit:` line. That is not decoration — the
next run reads it back to decide whether the ref still needs building, which is
what stops a nightly schedule recompiling three tags that have not changed since
they were cut.

## Running one

```
meshcore-simple_repeater-linux-amd64 --bridge 127.0.0.1:9000 --seed 4417 --sf 10 --bw-khz 250 --cr 1
```

The binary connects to the simulator and then does nothing on its own clock. The
simulator owns time: it sends a tick, the node runs `loop()` once per simulated
millisecond and acknowledges. That is what makes a run reproducible from its
seed, and it is why `delay()` here does nothing — a firmware that busy-waited for
`millis()` to move would wait forever.

`--print-airtime N` reports the firmware's own `getEstAirtimeFor(N)` and exits,
so a caller can check that its channel model and this build still agree about how
long a packet occupies the air. Two copies of a formula that nothing compares are
two formulas.

## The host variant

`variants/host/` is a MeshCore board like any other: it provides `target.h`,
the four globals every variant declares, and the platform underneath them.
Nothing in it is role-aware.

What it stands in for, and why it answers the way it does:

- **The radio** is a socket. Transmission reaches the wire immediately and is
  *not* immediately complete — `isSendComplete()` stays false until the simulator
  says the waveform ended, because how long a signal occupied the channel is a
  property of the samples the simulator generated, not of a formula this end
  could evaluate.
- **The clock** is the simulator's. `millis()` reads a variable the bridge writes.
- **The filesystem** is real files under a directory the simulator owns, with the
  block geometry of an nRF52840's `InternalFS`. The identity, preferences and ACL
  a repeater writes are genuinely persisted, so a CLI that changes a setting
  changes something.
- **Randomness** is seeded. Identity generation on hardware samples radio noise,
  which is correct there and the one thing that cannot be replayed.
- **Sensors and I²C** report absent. That is a case the firmware already handles,
  and it is a truthful answer rather than a plausible invented reading.

`roles.d/<role>.flags` is optional per-role build flags. Anything without a file
there builds with the defaults, which is the point.

## Building locally

```
MESHCORE=path/to/MeshCore CRYPTO=path/to/arduinolibs/libraries/Crypto \
  ./build.sh simple_repeater out
```

Cross-compiling is `TARGET_OS`, `TARGET_ARCH`, `CXX`, `CC` and `EXTRA_FLAGS`.

Sources that will not compile for a host are dropped rather than failing the
build — a helper that wants an SPI bus has nothing to say here. If dropping one
leaves the link short of a symbol, that role does not build on that platform for
that version of MeshCore, and the build says so instead of papering over it.

## Licence

MIT. MeshCore is MIT, this links MeshCore, and that is why these builds are
released from here rather than from the simulator that consumes them — see
`NOTICE.md`.
