// The radio model, between an emulated MCU and the RF engine.
//
// A native node reaches VirtualSX1262 in process through SimHal. An emulated one
// cannot: its firmware is inside QEMU, so the chip has to sit outside and be
// reachable from both sides at once.
//
//   QEMU  --- SPI transactions --->  this  --- frames and ticks --->  engine
//
// Deliberately the same chip object as the native path. A second model would be
// a second thing to keep in agreement, and the first time the two drifted every
// comparison between a native node and an emulated one would be measuring our
// own code rather than MeshCore's.
//
// One thing is different from a native node and it matters. There, the bridge
// owns the firmware's execution: a tick runs loop() a millisecond at a time and
// nothing else happens in between. Here the firmware runs inside an emulator on
// its own schedule, so SPI transactions arrive whenever QEMU feels like it,
// while ticks arrive from the engine. Both mutate the chip, so both take a lock,
// and the ordering between them is not reproducible the way a native node's is.
// See the note at the bottom.
//
// Usage:
//   radioserver <spi socket> [--bridge host:port]

#include "VirtualSX1262.h"

// Sockets, on the three families of desktop this ships for. The same five-line
// confinement bridge/main.cpp uses, for the same reason: Winsock needs
// initialising, its handles are not file descriptors, and closesocket is not
// close.
//
// Two things here are not in the bridge. Waiting on two sockets at once is
// WSAPoll on Windows and poll everywhere else - same structure, different
// name - and Unix domain sockets exist only on the POSIX side, which is why
// the transport is chosen by the address rather than compiled in.
#ifdef _WIN32
  #include <winsock2.h>
  #include <ws2tcpip.h>
  using sock_t = SOCKET;
  #define BAD_SOCK INVALID_SOCKET
  #define CLOSE_SOCK closesocket
  using pollfd_t = WSAPOLLFD;
  static int pollSockets(pollfd_t* f, int n, int t) { return WSAPoll(f, n, t); }
  static int sockRead(sock_t s, void* p, size_t n) { return recv(s, (char*)p, (int)n, 0); }
  static int sockWrite(sock_t s, const void* p, size_t n) { return send(s, (const char*)p, (int)n, 0); }
  using sockopt_t = char;
  using socklen_compat_t = int;
#else
  #include <arpa/inet.h>
  #include <netdb.h>
  #include <netinet/in.h>
  #include <netinet/tcp.h>
  #include <poll.h>
  #include <sys/socket.h>
  #include <sys/un.h>
  #include <unistd.h>
  using sock_t = int;
  #define BAD_SOCK (-1)
  #define CLOSE_SOCK close
  using pollfd_t = struct pollfd;
  static int pollSockets(pollfd_t* f, int n, int t) { return poll(f, (nfds_t)n, t); }
  static int sockRead(sock_t s, void* p, size_t n) { return (int)read(s, p, n); }
  static int sockWrite(sock_t s, const void* p, size_t n) { return (int)write(s, p, n); }
  using sockopt_t = int;
  using socklen_compat_t = socklen_t;
#endif

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <mutex>
#include <string>
#include <vector>

namespace {

// The QEMU side. Small because it is on the hot loop of every SPI byte.
enum : uint8_t {
  kCsAssert = 0x01,
  kCsRelease = 0x02,
  kXfer = 0x03,
  kReadBusy = 0x04,
  //  Whether DIO1 is asserted. QEMU never needed this - the ESP32 firmware
  //  polls the chip's IRQ register over SPI - but an nRF52 waits on the pin,
  //  and a pin nothing drives is a node that configures its radio and then
  //  sits there for ever.
  kReadIrq = 0x05,
  //  The board's front-end module enable line, with its level in the next
  //  byte. It arrives here rather than over SPI because the firmware drives it
  //  as an ordinary GPIO - the module is beside the radio, not inside it, and
  //  the chip has no idea whether its output reaches an antenna.
  kSetFem = 0x06,
};

// The engine side, shared with the simulator's Go half and with bridge/main.cpp.
constexpr uint8_t kFrame = 0x01;
constexpr uint8_t kTick = 0x02;
constexpr uint8_t kAck = 0x03;
constexpr uint8_t kTxDone = 0x04;
// Console traffic reaches an emulated node over the emulator's own serial
// port, so these two are named here only to be ignored deliberately rather
// than to fall through to the unknown-message path.
constexpr uint8_t kConsoleIn = 0x06;
constexpr uint8_t kChannelBusy = 0x08;
constexpr uint8_t kRadioStats = 0x09;

VirtualSX1262 gChip;
//  MESHCORE_RADIO_TRACE=1 logs every SPI transaction. Off by default: this is
//  on the hot path of every byte.
const bool gTracing = getenv("MESHCORE_RADIO_TRACE") != nullptr;
std::vector<uint8_t> gTrace;
std::mutex gChipMu;          // QEMU and the engine both reach the chip
uint32_t gSimMillis = 0;

bool readAll(sock_t fd, void* buf, size_t n) {
  auto* p = static_cast<uint8_t*>(buf);
  while (n > 0) {
    int got = sockRead(fd, p, n);
    if (got <= 0) return false;
    p += got;
    n -= (size_t)got;
  }
  return true;
}

bool writeAll(sock_t fd, const void* buf, size_t n) {
  const auto* p = static_cast<const uint8_t*>(buf);
  while (n > 0) {
    int put = sockWrite(fd, p, n);
    if (put <= 0) return false;
    p += put;
    n -= (size_t)put;
  }
  return true;
}

bool writeMsg(sock_t fd, uint8_t kind, const uint8_t* p, size_t n) {
  uint8_t hdr[3] = {kind, (uint8_t)(n >> 8), (uint8_t)n};
  if (!writeAll(fd, hdr, 3)) return false;
  return n == 0 || writeAll(fd, p, n);
}

void put32(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16);
  p[2] = (uint8_t)(v >> 8);  p[3] = (uint8_t)v;
}
void put16(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v;
}

// Sixteen bytes of counters, then everything this radio has been configured to
// be. The engine reads it on length, so a peer that stops at sixteen still
// parses - and bridge/main.cpp writes the same payload in the same order,
// because an emulated node and a native one reporting different shapes would
// make every comparison between them a comparison of our own code.
//
// It exists because a board profile is a datasheet claim about hardware and not
// a claim about the firmware running on it. Until the chip reported its own
// state there was no way here to tell a node configured correctly from one that
// was not.
void writeRadioStats(sock_t fd) {
  uint8_t sb[39];
  put32(&sb[0], gChip.irqReads());
  put32(&sb[4], gChip.busyReads());
  put32(&sb[8], gChip.busyMs());
  put32(&sb[12], gChip.spuriousRaises());

  sb[16] = gChip.rxGainReg();
  sb[17] = (uint8_t)gChip.txPowerDbm();
  sb[18] = gChip.femEnabled() ? 1 : 0;
  sb[19] = gChip.mode();
  sb[20] = (uint8_t)gChip.sf();
  sb[21] = (uint8_t)gChip.cr();
  put32(&sb[22], gChip.freqHz());
  // Bandwidth in whole hertz. The chip holds it in kHz as a float because the
  // datasheet's table is fractional - 7.81, 10.42 - and rounding it to kHz here
  // would make two distinct settings look like one.
  put32(&sb[26], (uint32_t)(gChip.bwKHz() * 1000.0f + 0.5f));
  put16(&sb[30], (uint16_t)gChip.preambleSyms());
  put16(&sb[32], gChip.irqMask());
  put16(&sb[34], gChip.irqFlags());
  // Three states, because "has not transmitted" is not "transmitted with the
  // module out": 0 no transmission yet, 1 module out, 2 module in.
  sb[36] = !gChip.hasTransmitted() ? 0 : (gChip.femAtTx() ? 2 : 1);
  // The DIO1 routing mask, which is not the IRQ enable mask above. Reported
  // separately because confusing the two is a fault that has already happened
  // here: the model read the enable mask where SetDioIrqParams gives the routing
  // mask, HeaderValid raised DIO1 part-way through a carrier, and the pin was
  // still high when RxDone arrived - no rising edge for a driver that attaches
  // it on one. Appended, because the host reads this record on length.
  put16(&sb[37], gChip.dio1Mask());

  writeMsg(fd, kRadioStats, sb, sizeof(sb));
}

// Anything the firmware handed its radio goes out to the engine now.
//
// Transmission reaches the channel immediately and is *not* immediately
// complete: the chip stays in transmit until the engine sends kTxDone, exactly
// as a native node does, because that is what stops a node talking over itself.
void drainTx(sock_t bridgeFd) {
  if (bridgeFd == BAD_SOCK || !gChip.hasPendingTx) return;
  gChip.hasPendingTx = false;
  writeMsg(bridgeFd, kFrame, gChip.pendingTx.data(), gChip.pendingTx.size());
}

sock_t dialBridge(const std::string& addr) {
  auto colon = addr.rfind(':');
  if (colon == std::string::npos) {
    fprintf(stderr, "radioserver: --bridge wants host:port, got %s\n", addr.c_str());
    return BAD_SOCK;
  }
  std::string host = addr.substr(0, colon), port = addr.substr(colon + 1);

  addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  addrinfo* res = nullptr;
  if (getaddrinfo(host.c_str(), port.c_str(), &hints, &res) != 0) {
    fprintf(stderr, "radioserver: cannot resolve %s\n", addr.c_str());
    return BAD_SOCK;
  }
  sock_t fd = BAD_SOCK;
  for (addrinfo* a = res; a; a = a->ai_next) {
    fd = ::socket(a->ai_family, a->ai_socktype, a->ai_protocol);
    if (fd == BAD_SOCK) continue;
    if (::connect(fd, a->ai_addr, (socklen_compat_t)a->ai_addrlen) == 0) break;
    CLOSE_SOCK(fd);
    fd = BAD_SOCK;
  }
  freeaddrinfo(res);
  if (fd == BAD_SOCK) {
    fprintf(stderr, "radioserver: cannot reach the engine at %s\n", addr.c_str());
    return BAD_SOCK;
  }
  // Frames are small and latency is the whole game here: a tick that waits on
  // Nagle is a node that answers late for no reason.
  sockopt_t one = 1;
  ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, (const char*)&one, sizeof(one));
  return fd;
}

// One message from the engine.
bool serviceBridge(sock_t fd) {
  uint8_t hdr[3];
  if (!readAll(fd, hdr, 3)) return false;
  const uint8_t kind = hdr[0];
  const size_t n = ((size_t)hdr[1] << 8) | hdr[2];
  std::vector<uint8_t> payload(n);
  if (n && !readAll(fd, payload.data(), n)) return false;

  std::lock_guard<std::mutex> lock(gChipMu);
  switch (kind) {
    case kFrame:
      // A packet the channel delivered. Only CRC-passing frames arrive here,
      // exactly as on hardware: everything else was recorded and withheld.
      gChip.inbox.push_back(std::move(payload));
      break;

    case kTxDone:
      gChip.transmitFinished();
      break;

    case kChannelBusy:
      if (n >= 1) gChip.setChannelBusy(payload[0] != 0);
      break;

    case kTick: {
      if (n != 4) break;
      uint32_t at = ((uint32_t)payload[0] << 24) | ((uint32_t)payload[1] << 16) |
                    ((uint32_t)payload[2] << 8) | payload[3];
      // A millisecond at a time, as a native node does. Stepping rather than
      // jumping is what keeps the chip's own timeouts behaving: a preamble flag
      // that should clear after 66 ms does not, if time arrives in 500 ms
      // lumps.
      while (gSimMillis < at) {
        gSimMillis++;
        gChip.tick(gSimMillis);
        drainTx(fd);
      }
      gChip.tick(gSimMillis);
      drainTx(fd);

      writeRadioStats(fd);
      if (!writeMsg(fd, kAck, payload.data(), 4)) return false;
      break;
    }

    case kConsoleIn:
      // Console input belongs to the firmware's serial port, and an emulated
      // node's serial port is the emulator's, not this socket. Ignoring it is
      // the whole handling: what matters is that it is not fatal. Treating it
      // as unknown killed the radio model the moment anything typed at the
      // fleet, and the node then reported "radio init failed: -2" - chip not
      // found - which points at wiring rather than at a console.
      break;

    default:
      // Skipped rather than fatal. The framing is length-prefixed and the
      // payload has already been read, so an unrecognised kind costs nothing
      // and cannot desynchronise the stream - whereas exiting takes the node
      // down for a message it did not need.
      fprintf(stderr, "radioserver: ignoring engine message 0x%02x (%zu bytes)\n",
              kind, n);
      break;
  }
  return true;
}

// One message from the emulator.
bool serviceQemu(sock_t fd, uint64_t* transactions, uint64_t* bytes) {
  uint8_t tag = 0;
  if (!readAll(fd, &tag, 1)) return false;

  std::lock_guard<std::mutex> lock(gChipMu);
  switch (tag) {
    case kCsAssert:
      gChip.beginTransaction();
      gTrace.clear();
      return true;

    case kCsRelease:
      gChip.endTransaction();
      (*transactions)++;
      //  One line per SPI transaction, opcode first. The point is comparison:
      //  the same chip serves a native node, an emulated ESP32 and an emulated
      //  nRF52, so when one of them fails to bring its radio up, a diff of the
      //  three traces says which command got an answer it did not like.
      if (gTracing && !gTrace.empty()) {
        fprintf(stderr, "spi:");
        for (size_t i = 0; i < gTrace.size() && i < 24; i++) {
          fprintf(stderr, " %02x", gTrace[i]);
        }
        if (gTrace.size() > 24) fprintf(stderr, " ...(%zu)", gTrace.size());
        fprintf(stderr, "\n");
        fflush(stderr);
      }
      return true;

    case kXfer: {
      uint8_t out = 0;
      if (!readAll(fd, &out, 1)) return false;
      uint8_t in = gChip.transferByte(out);
      (*bytes)++;
      if (gTracing) gTrace.push_back(out);
      return writeAll(fd, &in, 1);
    }

    case kReadIrq: {
      uint8_t irq = gChip.irqAsserted() ? 1 : 0;
      return writeAll(fd, &irq, 1);
    }

    case kSetFem: {
      uint8_t level = 0;
      if (!readAll(fd, &level, 1)) return false;
      gChip.setFemEnabled(level != 0);
      return true;
    }

    case kReadBusy: {
      // Always clear, which is what the native path does too: SimHal holds BUSY
      // low and VirtualSX1262 does not model the time a real chip spends
      // digesting a command. Answering differently here would make an emulated
      // node a different radio from a native one, which is the one thing this
      // whole arrangement exists to avoid.
      uint8_t busy = 0;
      return writeAll(fd, &busy, 1);
    }

    default:
      fprintf(stderr, "radioserver: unknown emulator tag 0x%02x\n", tag);
      return false;
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    fprintf(stderr, "usage: %s <spi socket|:port> [--bridge host:port]\n", argv[0]);
    return 2;
  }
  const char* path = argv[1];
  std::string bridgeAddr;
  for (int i = 2; i < argc - 1; i++) {
    if (strcmp(argv[i], "--bridge") == 0) bridgeAddr = argv[i + 1];
  }

#ifdef _WIN32
  WSADATA wsa;
  if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
    fprintf(stderr, "radioserver: Winsock will not start\n");
    return 1;
  }
#else
  // A broken pipe is an emulator that has exited, which is ordinary. Let the
  // read fail and tidy up rather than dying on a signal. Windows has no
  // SIGPIPE: a send to a closed socket returns an error there, which is the
  // behaviour this is asking for.
  ::signal(SIGPIPE, SIG_IGN);
#endif

  // Two ways in, because there are two emulators. QEMU is native and takes a
  // Unix socket; Renode runs on Mono, whose Unix domain socket support has been
  // unreliable for long enough that betting an emulated node on it is a poor
  // trade for one path separator. A leading colon asks for TCP on loopback.
  //
  // Windows has only the TCP half. Its own AF_UNIX exists but mingw has no
  // <sys/un.h> to reach it with, and the simulator already asks for ":0" on
  // Windows for both emulators - so the missing half is unreachable rather
  // than merely untested. Saying so beats a build that silently listens
  // nowhere.
  const bool useTcp = path[0] == ':';
  sock_t srv = BAD_SOCK;
  if (useTcp) {
    const int port = atoi(path + 1);
    srv = ::socket(AF_INET, SOCK_STREAM, 0);
    if (srv == BAD_SOCK) {
      fprintf(stderr, "radioserver: cannot make a socket\n");
      return 1;
    }
    sockopt_t on = 1;
    ::setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, (const char*)&on, sizeof(on));
    sockaddr_in in{};
    in.sin_family = AF_INET;
    in.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    in.sin_port = htons((uint16_t)port);
    if (::bind(srv, (sockaddr*)&in, sizeof(in)) != 0) {
      fprintf(stderr, "radioserver: cannot bind %s\n", path);
      return 1;
    }
    if (::listen(srv, 1) != 0) {
      fprintf(stderr, "radioserver: cannot listen on %s\n", path);
      return 1;
    }
    // The chosen port is printed because port 0 means "any", which is what a
    // harness starting several nodes at once wants: it reads the number back
    // rather than picking one and hoping.
    socklen_compat_t len = sizeof(in);
    if (::getsockname(srv, (sockaddr*)&in, &len) == 0) {
      printf("radioserver: listening on 127.0.0.1:%d\n", ntohs(in.sin_port));
    }
  } else {
#ifdef _WIN32
    fprintf(stderr, "radioserver: this build takes ':port' and not a socket "
                    "path (%s): Windows reaches both emulators over TCP\n", path);
    return 2;
#else
    ::unlink(path);
    srv = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (srv == BAD_SOCK) {
      perror("socket");
      return 1;
    }
    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    if (strlen(path) >= sizeof(addr.sun_path)) {
      fprintf(stderr, "radioserver: socket path too long: %s\n", path);
      return 1;
    }
    strncpy(addr.sun_path, path, sizeof(addr.sun_path) - 1);
    if (::bind(srv, (sockaddr*)&addr, sizeof(addr)) < 0) {
      perror("bind");
      return 1;
    }
    if (::listen(srv, 1) < 0) {
      perror("listen");
      return 1;
    }
    printf("radioserver: listening on %s\n", path);
#endif
  }
  fflush(stdout);

  sock_t bridgeFd = BAD_SOCK;
  if (!bridgeAddr.empty()) {
    bridgeFd = dialBridge(bridgeAddr);
    if (bridgeFd == BAD_SOCK) return 1;
    printf("radioserver: joined the engine at %s\n", bridgeAddr.c_str());
    fflush(stdout);
  } else {
    // Worth saying. Without the engine this chip transmits into nowhere and
    // never receives, so the firmware comes up and then waits for ever on a
    // transmission that cannot complete - which looks like a hang rather than
    // like a missing argument.
    printf("radioserver: no --bridge, so this node is deaf and mute\n");
    fflush(stdout);
  }

  sock_t qemuFd = ::accept(srv, nullptr, nullptr);
  if (qemuFd == BAD_SOCK) {
    fprintf(stderr, "radioserver: the emulator never connected\n");
    return 1;
  }
  printf("radioserver: emulator connected\n");
  fflush(stdout);

  uint64_t transactions = 0, bytes = 0;
  for (;;) {
    pollfd_t fds[2];
    int n = 0;
    fds[n++] = {qemuFd, POLLIN, 0};
    if (bridgeFd != BAD_SOCK) fds[n++] = {bridgeFd, POLLIN, 0};

    if (pollSockets(fds, n, -1) < 0) break;

    if (fds[0].revents & (POLLIN | POLLHUP)) {
      if (!serviceQemu(qemuFd, &transactions, &bytes)) break;
    }
    if (bridgeFd != BAD_SOCK && (fds[1].revents & (POLLIN | POLLHUP))) {
      if (!serviceBridge(bridgeFd)) {
        fprintf(stderr, "radioserver: the engine went away\n");
        break;
      }
    }
  }

  printf("radioserver: %llu transactions, %llu bytes\n",
         (unsigned long long)transactions, (unsigned long long)bytes);
  if (bridgeFd != BAD_SOCK) CLOSE_SOCK(bridgeFd);
  CLOSE_SOCK(qemuFd);
  CLOSE_SOCK(srv);
#ifndef _WIN32
  if (!useTcp) ::unlink(path);
#endif
#ifdef _WIN32
  WSACleanup();
#endif
  return 0;
}

// What this is not, and it is worth being exact.
//
// A native node runs in lockstep: the engine supplies the clock, the bridge runs
// loop() one millisecond at a time, and the same seed gives the same answer
// every time. Here the engine still supplies the clock to the *chip*, so IRQ
// timing and channel state are engine-relative - but the *firmware* runs inside
// an emulator on wall time, so the instant at which it reads a register is not
// reproducible.
//
// The consequence is narrow and real: an emulated node can transmit and receive
// and take part in a mesh, and two runs of one seed will not produce identical
// ledgers. Mixing emulated and native nodes in a measurement therefore costs the
// determinism the native path has. Fixing it means QEMU's -icount with the
// engine driving virtual time, which is the same contract the native bridge
// already implements.
