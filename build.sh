#!/usr/bin/env bash
#
# Note on "${arr[@]+...}": macOS ships bash 3.2, where expanding an empty
# array under `set -u` is an unbound-variable error rather than nothing at
# all. Every array expansion here is guarded for that reason - a Linux-only
# spelling builds fine here and fails only on the Mac.
# Build one MeshCore application for this host.
#
#   MESHCORE=path/to/MeshCore CRYPTO=path/to/Crypto ./build.sh <role> [outdir]
#
# <role> is a directory name under MeshCore's examples/. It is not validated
# against a list, because there is no list: a node is a node, and which
# application it runs is the only thing that makes it a repeater rather than a
# companion. When upstream adds an example, this builds it without being told.
#
# Sources are compiled individually and a source that will not compile for a
# host is dropped rather than failing the build — a board-specific helper that
# wants an SPI bus has nothing to say here. If dropping it leaves the link short
# of a symbol, the role does not build on this host for this version, and that
# is reported as such rather than papered over.
set -uo pipefail

role=${1:-}
out=${2:-build}
if [ -z "$role" ]; then
  echo "usage: MESHCORE=... CRYPTO=... $0 <role> [outdir]" >&2
  exit 2
fi
# The radio model is ours rather than a MeshCore application: it carries its own
# main() and opens neither MeshCore nor Crypto. Demanding two checkouts it never
# reads would make the one build an emulator-only packaging job needs the most
# awkward one to ask for.
if [ "$role" != radioserver ]; then
  : "${MESHCORE:?set MESHCORE to a MeshCore checkout}"
  : "${CRYPTO:?set CRYPTO to arduinolibs/libraries/Crypto}"
fi

root=$(cd "$(dirname "$0")" && pwd)
variant="$root/variants/host"
# The SX1262 model is shared with QEMU, Renode and the simulator, so it lives
# in its own repository rather than in this variant. A submodule keeps one copy
# and one history: two copies of a chip model drift, and the moment they do a
# native node and an emulated one stop being comparable.
vsx="$root/vendor/virtual-sx1262"
if [ ! -f "$vsx/src/VirtualSX1262.cpp" ]; then
  echo "build.sh: vendor/virtual-sx1262 is empty; run: git submodule update --init" >&2
  exit 1
fi
# Globbed rather than listed, so the library splitting a file does not break
# this build with an undefined symbol in a repository its author is not in.
vsx_src=("$vsx"/src/*.cpp)
src="${MESHCORE:-}/examples/$role"
if [ "$role" != radioserver ]; then
  [ -d "$src" ] || { echo "no such role: $role (looked in $MESHCORE/examples)" >&2; exit 2; }
fi

# The target, which is not necessarily this machine. Windows and 32-bit builds
# are produced by cross-compilers on a Linux runner, so os/arch are inputs with
# defaults rather than facts read off uname.
case "$(uname -s)" in
  Linux)  host_os=linux ;;
  Darwin) host_os=darwin ;;
  MINGW*|MSYS*|CYGWIN*) host_os=windows ;;
  *) host_os=unknown ;;
esac
case "$(uname -m)" in
  x86_64|amd64) host_arch=amd64 ;;
  arm64|aarch64) host_arch=arm64 ;;
  i?86) host_arch=386 ;;
  *) host_arch=unknown ;;
esac
os=${TARGET_OS:-$host_os}
arch=${TARGET_ARCH:-$host_arch}
CXX=${CXX:-g++}
CC=${CC:-gcc}
# EXTRA_FLAGS reaches both the compiler and the linker, because the flags that
# select a target — -m32 above all — are wrong in only one of the two.
read -r -a extra_flags <<< "${EXTRA_FLAGS:-}"
if [ "$os" = unknown ] || [ "$arch" = unknown ]; then
  echo "set TARGET_OS and TARGET_ARCH: this machine reports $(uname -s)/$(uname -m)" >&2
  exit 1
fi

exe=""
extra_link=()
if [ "$os" = windows ]; then
  exe=".exe"
  # Winsock is not linked by default, and the runtimes are static so the
  # artefact is one file a user can run rather than one that wants DLLs.
  extra_link=(-lws2_32 -static -static-libgcc -static-libstdc++)
fi
if [ ${#extra_flags[@]} -gt 0 ]; then
  extra_link+=(${extra_flags[@]+"${extra_flags[@]}"})
fi

# The radio model, which every emulated node needs and no native one does.
#
# Built from this tree rather than from the simulator's packaging because the
# chip model lives here: an emulated node and a native one have to be the same
# VirtualSX1262, and compiling both from one checkout is the cheapest way to
# keep them that. It reaches nothing else - no MeshCore, no Crypto, no RadioLib
# - so it is two objects and a link rather than the sweep below.
if [ "$role" = radioserver ]; then
  obj="$out/obj/radioserver"
  mkdir -p "$obj"
  bin="$out/radioserver-$os-$arch$exe"
  rs_flags=("${STD:--std=c++17}" -O2 -w ${extra_flags[@]+"${extra_flags[@]}"})
  rs_objs=()
  for f in "${vsx_src[@]}" "$root/bridge/radioserver.cpp"; do
    o="$obj/$(basename "${f%.cpp}").o"
    if ! "$CXX" "${rs_flags[@]}" -I "$variant" -I "$vsx/src" -c "$f" -o "$o"; then
      echo "build.sh: radioserver: $(basename "$f") did not compile for $os/$arch" >&2
      exit 1
    fi
    rs_objs+=("$o")
  done
  if ! "$CXX" -o "$bin" "${rs_objs[@]}" ${extra_link[@]+"${extra_link[@]}"}; then
    echo "build.sh: radioserver does not link for $os/$arch" >&2
    exit 3
  fi
  echo "$bin"
  exit 0
fi

obj="$out/obj/$role"
mkdir -p "$obj"
bin="$out/meshcore-$role-$os-$arch$exe"

# RadioLib is vendored (MIT, see vendor/RadioLib/VENDORED.md) and compiled in,
# so MeshCore's own radio driver runs against the library it was written for
# rather than against a stand-in of ours.
radiolib="$root/vendor/RadioLib/src"
inc=(-I "$variant" -I "$vsx/src" -I "$MESHCORE/src" -I "$src" -I "$CRYPTO" -I "$MESHCORE/lib/ed25519" -I "$radiolib")
# -O2, not -Os: this build exists to be fast, and it is also the build whose
# results get compared against the emulated one. Optimisation level is exactly
# the kind of difference that would make that comparison meaningless if it
# drifted between machines, so it is pinned here rather than left to the caller.
cxxflags=("${STD:--std=c++17}" -O2 -w -fpermissive -DMESHCORE_HOST_VARIANT=1 -DNRF52_PLATFORM=1 -DENABLE_USB_INTERFACE=1
          # What MeshCore's own platformio.ini sets for every board, and needed
          # here for the same reasons: its driver reaches into RadioLib's
          # internals, which GODMODE makes public, and STATIC_ONLY keeps
          # RadioLib off the heap.
          -DRADIOLIB_GODMODE=1 -DRADIOLIB_STATIC_ONLY=1
          # The radio a board is fitted with. Real variants set these per board;
          # the scenario overrides them at runtime, and RadioLib's begin() wants
          # a starting point.
          #
          # LORA_TX_POWER is also the ceiling. MeshCore defines
          # MAX_LORA_TX_POWER as LORA_TX_POWER unless a variant overrides it,
          # and the companion both reports that value as its maximum and clamps
          # any set-power request to it. At 20 a scenario asking for 22 was
          # silently reduced, with the only sign a status line in the workbench.
          #
          # 22 dBm is what an SX1262 delivers at its pin, and it is within the
          # UK allowance for the 869.4-869.65 MHz band.
          -DLORA_FREQ=869.618 -DLORA_BW=62.5 -DLORA_SF=8 -DLORA_CR=5 -DLORA_TX_POWER=22
          -include "$variant/HostArduino.h" ${extra_flags[@]+"${extra_flags[@]}"})

# A build whose virtual radio misbehaves the way real ones do. Published as its
# own variant - see roles.d/README.md - because "does this firmware survive a
# radio that latches its interrupt flags" is a question about the firmware, and
# an experiment should be able to select it like any other build.
if [ -n "${STUCK_IRQ_MS:-}" ]; then
  cxxflags+=("-DVIRTUAL_SX1262_STUCK_IRQ_MS=$STUCK_IRQ_MS")
fi

# Flags this particular role needs, if any. See roles.d/README.md.
if [ -f "$root/roles.d/$role.flags" ]; then
  while read -r line; do
    case "$line" in ''|'#'*) continue ;; esac
    cxxflags+=("$line")
  done < "$root/roles.d/$role.flags"
fi

# Every .cpp MeshCore ships that could plausibly be host code, plus the chosen
# application. Radio drivers and display drivers are not enumerated away — they
# simply fail to compile and get dropped, which keeps this working when upstream
# moves a file.
#
# Collected with a loop rather than `mapfile`, which is a bash 4 builtin:
# macOS ships bash 3.2 and has neither that nor a reliable process
# substitution here, so the Mac build failed at this line before it compiled
# anything.
candidates=()
while IFS= read -r _f; do
  [ -n "$_f" ] && candidates+=("$_f")
done <<CANDIDATES
$(
  find "$MESHCORE/src" -maxdepth 2 -name '*.cpp' 2>/dev/null
  # The radio driver layer sits a level deeper than the sweep above reaches,
  # and is the whole reason RadioLib is here: this is MeshCore's own driver.
  find "$MESHCORE/src/helpers/radiolib" -maxdepth 1 -name '*.cpp' 2>/dev/null
  # RadioLib: the library itself, its SX126x family, and its own HAL base.
  find "$radiolib" -maxdepth 1 -name '*.cpp' 2>/dev/null
  find "$radiolib/modules/SX126x" -name '*.cpp' 2>/dev/null
  find "$radiolib/protocols/PhysicalLayer" -name '*.cpp' 2>/dev/null
  find "$radiolib/utils" -name '*.cpp' 2>/dev/null
  find "$src" -maxdepth 1 -name '*.cpp' 2>/dev/null
)
CANDIDATES

skipped=()
objs=()
for f in "${candidates[@]}"; do
  o="$obj/$(echo "${f#"$MESHCORE"/}" | tr '/' '_' | sed 's/\.cpp$/.o/')"
  if "$CXX" "${cxxflags[@]}" "${inc[@]}" -c "$f" -o "$o" 2>"$o.log"; then
    objs+=("$o")
  else
    skipped+=("${f#"$MESHCORE"/}")
  fi
done

# The variant and the bridge are ours and must compile; a failure here is a bug
# in this repository rather than an upstream file that does not suit a host.
#
# Test programs in the variant directory are skipped: they carry their own
# main(), so linking one into a role produces "multiple definition of main" and
# takes down every role at once, which reads as the role not porting.
for f in "$variant"/*.cpp "${vsx_src[@]}" "$root/bridge/main.cpp"; do
  case "$(basename "$f")" in *_test.cpp) continue ;; esac
  o="$obj/$(basename "${f%.cpp}").o"
  # The bridge is the one file that includes windows.h - through winsock2.h -
  # and Windows owns the names INPUT and OUTPUT there. It drives no pins, so
  # it asks the variant not to declare them. See variants/host/Arduino.h.
  bridgeflag=()
  case "$f" in */bridge/main.cpp) bridgeflag=(-DMESHCORE_HOST_BRIDGE=1) ;; esac
  if ! "$CXX" "${cxxflags[@]}" ${bridgeflag[@]+"${bridgeflag[@]}"} "${inc[@]}" -c "$f" -o "$o"; then
    echo "build.sh: the host variant itself failed to compile" >&2
    exit 1
  fi
  objs+=("$o")
done

# Crypto and ed25519 are MeshCore's own dependencies, pinned by its lib_deps.
for f in "$CRYPTO"/*.cpp; do
  # RNG.cpp wants an entropy source and somewhere to persist a seed. The variant
  # supplies a seeded one instead, because a run has to be reproducible.
  [ "$(basename "$f")" = "RNG.cpp" ] && continue
  o="$obj/crypto_$(basename "${f%.cpp}").o"
  "$CXX" "${cxxflags[@]}" "${inc[@]}" -c "$f" -o "$o" 2>/dev/null && objs+=("$o")
done
for f in "$MESHCORE"/lib/ed25519/*.c; do
  o="$obj/ed_$(basename "${f%.c}").o"
  "$CC" -std=c11 -O2 -w -I "$MESHCORE/lib/ed25519" -c "$f" -o "$o" 2>/dev/null && objs+=("$o")
done

if ! "$CXX" -o "$bin" "${objs[@]}" ${extra_link[@]+"${extra_link[@]}"} 2>"$obj/link.log"; then
  echo "build.sh: $role does not link for a host on this MeshCore version." >&2
  echo "Unresolved after dropping ${#skipped[@]} board-specific sources:" >&2
  grep -o "undefined reference to \`[^']*'" "$obj/link.log" | sort -u | head -20 >&2
  exit 3
fi

echo "$bin"
