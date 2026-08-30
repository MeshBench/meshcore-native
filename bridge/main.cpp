// The bridge: main() for a MeshCore application built against the host variant.
//
// A board's runtime calls setup() once and loop() forever, driven by a crystal.
// This calls the same two functions, driven by a socket — the simulator owns the
// clock, the antenna and the console, and this is the only file that knows that.
//
// Nothing here is role-aware, and that is deliberate. A node is a node: a radio
// at a place running an application. Which application — repeater, companion,
// room server, sensor, or something MeshCore has not shipped yet — is settled by
// what gets linked alongside this file, not by anything this file or the
// simulator decides. When upstream adds a new example directory, it builds.
// Sockets, on the three families of desktop this ships for.
//
// Winsock is not a POSIX socket layer with different headers: it needs
// initialising, its handles are not file descriptors, and closesocket is not
// close. Confining that to five lines here is cheaper than the alternative,
// which is a Windows build that silently is not tested.
#ifdef _WIN32
  #include <winsock2.h>
  #include <ws2tcpip.h>
  using sock_t = SOCKET;
  #define BAD_SOCK INVALID_SOCKET
  #define CLOSE_SOCK closesocket
  static int sockRead(sock_t s, void* p, size_t n) { return recv(s, (char*)p, (int)n, 0); }
  static int sockWrite(sock_t s, const void* p, size_t n) { return send(s, (const char*)p, (int)n, 0); }
#else
  #include <arpa/inet.h>
  #include <netinet/in.h>
  #include <netinet/tcp.h>
  #include <sys/socket.h>
  #include <unistd.h>
  using sock_t = int;
  #define BAD_SOCK (-1)
  #define CLOSE_SOCK close
  static int sockRead(sock_t s, void* p, size_t n) { return (int)read(s, p, n); }
  static int sockWrite(sock_t s, const void* p, size_t n) { return (int)write(s, p, n); }
#endif

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <string>
#include <vector>

#include "target.h"
#include "SimHal.h"

// The application's entry points, as on any Arduino target.
void setup();
void loop();

// Defined by the host variant.
extern uint32_t g_sim_millis;
int g_stuck_irq_ms = 0;
extern int g_sf;
extern float g_bwKHz;
extern int g_cr;
extern uint32_t g_identity_seed;

namespace {

// The wire protocol, shared with the simulator's Go side.
constexpr uint8_t kFrame = 0x01;       // a packet, either direction
constexpr uint8_t kTick = 0x02;        // advance simulated time to N ms
constexpr uint8_t kAck = 0x03;         // this node has caught up to N ms
constexpr uint8_t kTxDone = 0x04;      // the waveform has left the antenna
constexpr uint8_t kOriginate = 0x05;   // send a message of the node's own
constexpr uint8_t kConsoleIn = 0x06;   // bytes typed at the node's UART
constexpr uint8_t kConsoleOut = 0x07;  // bytes the node printed
constexpr uint8_t kChannelBusy = 0x08; // is another station on the air here?
constexpr uint8_t kRadioStats = 0x09;  // node -> host: what the chip has seen

sock_t gFd = BAD_SOCK;
std::deque<uint8_t> gConsoleIn;
std::vector<char> gConsoleOut;

bool readAll(sock_t fd, uint8_t* p, size_t n) {
  while (n) {
    int r = sockRead(fd, p, n);
    if (r <= 0) return false;
    p += r;
    n -= (size_t)r;
  }
  return true;
}

bool writeMsg(sock_t fd, uint8_t kind, const uint8_t* p, size_t n) {
  uint8_t hdr[3] = {kind, (uint8_t)(n >> 8), (uint8_t)n};
  if (sockWrite(fd, hdr, 3) != 3) return false;
  while (n) {
    int w = sockWrite(fd, p, n);
    if (w <= 0) return false;
    p += w;
    n -= (size_t)w;
  }
  return true;
}

// The console seam. MeshCore's applications carry their own CLI on Serial, and
// carrying it over the bridge is what makes clicking a node in the simulator
// reach a real command interface rather than a mock of one.
void consoleWrite(const char* p, size_t n) { gConsoleOut.insert(gConsoleOut.end(), p, p + n); }

int consoleRead() {
  if (gConsoleIn.empty()) return -1;
  int c = gConsoleIn.front();
  gConsoleIn.pop_front();
  return c;
}

int consoleAvailable() { return (int)gConsoleIn.size(); }

int consolePeek() { return gConsoleIn.empty() ? -1 : gConsoleIn.front(); }

void flushConsole() {
  if (gConsoleOut.empty()) return;
  writeMsg(gFd, kConsoleOut, (const uint8_t*)gConsoleOut.data(), gConsoleOut.size());
  gConsoleOut.clear();
}

// Anything the application handed its radio goes out now.
//
// Transmission reaches the wire immediately and is *not* immediately complete:
// isSendComplete() stays false until the engine sends kTxDone. The node cannot
// time its own transmission, because how long the signal occupied the channel is
// a property of the samples the engine generated. Computing it here from the
// airtime estimate would replace the simulation with the formula.
void drainTx() {
  auto& chip = sim_hal.chip();
  if (!chip.hasPendingTx) return;
  chip.hasPendingTx = false;
  writeMsg(gFd, kFrame, chip.pendingTx.data(), chip.pendingTx.size());
}

sock_t connectTo(const std::string& addr) {
#ifdef _WIN32
  WSADATA wsa;
  if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) return BAD_SOCK;
#endif
  auto colon = addr.rfind(':');
  if (colon == std::string::npos) {
    fprintf(stderr, "bridge: --bridge wants host:port, got %s\n", addr.c_str());
    return BAD_SOCK;
  }
  std::string host = addr.substr(0, colon);
  int port = atoi(addr.c_str() + colon + 1);

  sock_t fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd == BAD_SOCK) return BAD_SOCK;
  sockaddr_in sa{};
  sa.sin_family = AF_INET;
  sa.sin_port = htons((uint16_t)port);
  if (inet_pton(AF_INET, host.c_str(), &sa.sin_addr) != 1) {
    CLOSE_SOCK(fd);
    fprintf(stderr, "bridge: cannot parse address %s\n", host.c_str());
    return BAD_SOCK;
  }
  if (connect(fd, (sockaddr*)&sa, sizeof sa) != 0) {
    CLOSE_SOCK(fd);
    return BAD_SOCK;
  }
  // Nagle would coalesce a frame with the tick that follows it, which is exactly
  // the latency the lockstep round trip exists to avoid paying.
  int one = 1;
  setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, (const char*)&one, sizeof one);
  return fd;
}

}  // namespace

int main(int argc, char** argv) {
  std::string bridge;
  int printAirtimeFor = -1;
  for (int i = 1; i < argc; i++) {
    auto next = [&]() { return i + 1 < argc ? argv[++i] : ""; };
    if (!strcmp(argv[i], "--bridge")) bridge = next();
    else if (!strcmp(argv[i], "--seed")) g_identity_seed = (uint32_t)strtoul(next(), nullptr, 10);
    else if (!strcmp(argv[i], "--sf")) g_sf = atoi(next());
    else if (!strcmp(argv[i], "--bw-khz")) g_bwKHz = (float)atof(next());
    else if (!strcmp(argv[i], "--cr")) g_cr = atoi(next());
    // Fault injection: how long a raised detection flag refuses to clear.
    // This is the misbehaviour MeshCore 1.17 was written to survive, and
    // without a way to reproduce it the difference between 1.16 and 1.17
    // cannot be observed at all.
    else if (!strcmp(argv[i], "--stuck-irq-ms")) g_stuck_irq_ms = atoi(next());
    else if (!strcmp(argv[i], "--print-airtime")) printAirtimeFor = atoi(next());
  }

  // A self-report, so the simulator can check that this transcription of the
  // airtime formula still agrees with its own. Two copies of a formula that
  // nothing compares are two formulas.
  if (printAirtimeFor >= 0) {
    printf("%u\n", sim_hal.chip().estAirtimeMs(printAirtimeFor));
    return 0;
  }
  if (bridge.empty()) {
    fprintf(stderr,
            "usage: %s --bridge host:port [--seed N] [--sf N] [--bw-khz F] [--cr N]\n"
            "A MeshCore node with its radio and console on a socket.\n",
            argv[0]);
    return 2;
  }

  gFd = connectTo(bridge);
  if (gFd == BAD_SOCK) {
    fprintf(stderr, "bridge: cannot reach the simulator at %s\n", bridge.c_str());
    return 1;
  }
  Serial.attach(consoleWrite, consoleRead, consoleAvailable, consolePeek);

  // Only if asked for on the command line. A build compiled as a faulty
  // variant already has its value, and overwriting it with the flag default
  // silently turned that whole variant back into a well-behaved one.
  if (g_stuck_irq_ms > 0) sim_hal.chip().setStuckIrqMs((uint32_t)g_stuck_irq_ms);
  setup();
  drainTx();
  flushConsole();

  uint8_t hdr[3];
  for (;;) {
    if (!readAll(gFd, hdr, 3)) break;
    uint16_t n = (uint16_t)((hdr[1] << 8) | hdr[2]);
    std::vector<uint8_t> payload(n);
    if (n && !readAll(gFd, payload.data(), n)) break;

    switch (hdr[0]) {
      case kFrame:
        // Queued, not delivered. The application collects it from recvRaw() on
        // its next loop, exactly as it would drain a real radio's FIFO.
        sim_hal.chip().inbox.push_back(std::move(payload));
        break;

      case kTxDone:
        sim_hal.chip().transmitFinished();
        break;

      case kChannelBusy:
        // What listen-before-talk asks the radio, answered by the only thing
        // that knows: the engine, which has every waveform in flight and how
        // strongly each arrives here. Sent immediately before the tick it
        // applies to, so the node reads the channel as it was at that instant
        // rather than as it was a tick ago.
        sim_hal.chip().setChannelBusy(!payload.empty() && payload[0] != 0);
        break;

      case kConsoleIn:
        gConsoleIn.insert(gConsoleIn.end(), payload.begin(), payload.end());
        break;

      case kOriginate:
        // The application owns the mesh instance, not this file, so there is no
        // generic way to make it author a packet — and fabricating one here is
        // exactly what does not work: MeshCore drops what is not a valid packet,
        // correctly, and nothing relays.
        //
        // Said out loud rather than ignored. A silently dropped request looks
        // identical to a message that was sent and reached nobody, which is the
        // single most misleading thing this bridge could do. Originate traffic
        // through the node's own CLI instead — that is what the console is for.
        fprintf(stderr,
                "bridge: this application cannot be asked to originate; "
                "send through its CLI on the console instead\n");
        break;

      case kTick: {
        // Reconcile the node's own clock to network time, then let RadioLib
        // service anything the chip raised. Servicing here rather than from
        // inside the chip means an ISR never runs in the middle of an SPI
        // transaction - which it cannot on hardware either.
        if (n != 4) break;
        uint32_t at = ((uint32_t)payload[0] << 24) | ((uint32_t)payload[1] << 16) |
                      ((uint32_t)payload[2] << 8) | payload[3];
        // One loop per millisecond of simulated time. Stepping rather than
        // jumping is what keeps timeouts, retries and duty-cycle refill behaving
        // as they do on hardware: a node that sees time move in 500 ms jumps
        // takes different branches.
        //
        // A tick that advances no time still gets one pass, which is the only
        // reason for the second branch. Running that pass unconditionally gave
        // the final millisecond of every advancing tick a second one at the
        // same clock value: beginTick, servicePendingIrq, loop and drainTx all
        // ran twice there. Every timer the firmware runs off this clock fired
        // an extra time at each tick boundary, and a pending interrupt was
        // serviced twice, on every tick, for every node.
        if (g_sim_millis < at) {
          while (g_sim_millis < at) {
            g_sim_millis++;
            // The RTC only has second resolution, so it moves once per 1000 of
            // these - but it has to actually move. Without this the comment
            // above was aspirational: rtc_clock stayed at its construction
            // value forever, so two adverts sent seconds apart carried the
            // same embedded timestamp and hashed identically, and the second
            // was silently deduplicated as a repeat of the first.
            if (g_sim_millis % 1000 == 0) {
              rtc_clock.advance(1);
            }
            sim_hal.beginTick(g_sim_millis);
            sim_hal.servicePendingIrq();
            loop();
            drainTx();
          }
        } else {
          sim_hal.beginTick(g_sim_millis);
          sim_hal.servicePendingIrq();
          loop();
          drainTx();
        }
        flushConsole();
        {
          // What the chip has seen, alongside the acknowledgement. Cheap, and
          // it turns "the mesh went quiet" into a question with an answer:
          // was the channel busy, or did our chip only say so?
          //
          // The same payload radioserver sends, in the same order. Two writers
          // of one wire format is the arrangement RadioServerSX1262 was written
          // to avoid, and it is only tolerable while they are kept identical:
          // an emulated node and a native one reporting different shapes would
          // make every comparison between them a comparison of our own code.
          auto& c = sim_hal.chip();
          uint32_t st[4] = {c.irqReads(), c.busyReads(), c.busyMs(), c.spuriousRaises()};
          uint8_t sb[37];
          for (int k = 0; k < 4; k++) {
            sb[k*4+0] = (uint8_t)(st[k] >> 24); sb[k*4+1] = (uint8_t)(st[k] >> 16);
            sb[k*4+2] = (uint8_t)(st[k] >> 8);  sb[k*4+3] = (uint8_t)st[k];
          }
          auto put32 = [&](int at, uint32_t v) {
            sb[at+0] = (uint8_t)(v >> 24); sb[at+1] = (uint8_t)(v >> 16);
            sb[at+2] = (uint8_t)(v >> 8);  sb[at+3] = (uint8_t)v;
          };
          auto put16 = [&](int at, uint16_t v) {
            sb[at+0] = (uint8_t)(v >> 8); sb[at+1] = (uint8_t)v;
          };
          sb[16] = c.rxGainReg();
          sb[17] = (uint8_t)c.txPowerDbm();
          sb[18] = c.femEnabled() ? 1 : 0;
          sb[19] = c.mode();
          sb[20] = (uint8_t)c.sf();
          sb[21] = (uint8_t)c.cr();
          put32(22, c.freqHz());
          put32(26, (uint32_t)(c.bwKHz() * 1000.0f + 0.5f));
          put16(30, (uint16_t)c.preambleSyms());
          put16(32, c.irqMask());
          put16(34, c.irqFlags());
          // Always "not answered" here, never "the module was out".
          //
          // A native node has no front-end module wired: SimHal owns an array
          // of pins and nothing drives an enable line into it. Reporting the
          // line as low would be true of the pin and false about the board, and
          // the engine would dock every native node on a board that has a
          // module for a fault it did not have.
          sb[36] = 0;
          writeMsg(gFd, kRadioStats, sb, sizeof(sb));
        }
        if (!writeMsg(gFd, kAck, payload.data(), 4)) goto done;
        break;
      }

      default:
        goto done;
    }
  }
done:
  fprintf(stderr, "bridge: closed after %u ms\n", g_sim_millis);
  CLOSE_SOCK(gFd);
  return 0;
}
