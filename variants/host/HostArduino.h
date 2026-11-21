// The handful of Arduino functions MeshCore's helpers use that its own test
// mocks do not provide.
//
// Force-included ahead of everything else, because ArduinoHelpers.h calls them
// at namespace scope and there is nowhere later to put them.
#pragma once

// Windows, before anything can include its headers.
//
// This file is force-included ahead of every translation unit, which makes it
// the only place that can set these in time. The bridge reaches windows.h
// through winsock2.h, and without NOMINMAX that header defines min and max as
// macros - which then eat the templates below and produce a parse error deep
// inside winuser.h, a long way from anything to do with either.
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX 1
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN 1
#endif
#endif

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <type_traits>

// Seeded rather than entropic: a run has to be reproducible from its seed, and
// the identity a node generates comes through here.
//
// Its own generator rather than POSIX random(): that function does not exist
// on Windows at all, and where it does exist the sequence is the platform's
// business - so the same seed produced different node identities on Linux and
// macOS, which is not what "reproducible from its seed" is supposed to mean.
// This is xorshift64*, which is small, fast, and identical everywhere.
inline unsigned long long& hostRandomState() {
  static unsigned long long s = 0x9E3779B97F4A7C15ull;
  return s;
}
inline void randomSeed(unsigned long seed) {
  // Never zero: xorshift is stuck there for ever.
  hostRandomState() = (unsigned long long)seed * 0x2545F4914F6CDD1Dull + 0x9E3779B97F4A7C15ull;
}
inline long hostRandom() {
  unsigned long long x = hostRandomState();
  x ^= x >> 12;
  x ^= x << 25;
  x ^= x >> 27;
  hostRandomState() = x;
  // Positive half only: callers treat this as Arduino's random(), which never
  // returns a negative number.
  return (long)((x * 0x2545F4914F6CDD1Dull) >> 33);
}

// Arduino defines these as macros and MeshCore calls them unqualified.
// Templates rather than macros: force-including a `min(a,b)` macro ahead of the
// standard library breaks <limits>, whose numeric_limits<T>::min() then looks
// like a macro call with one argument. That produced 368 errors inside libstdc++
// and none in any MeshCore file, which is a confusing place to start debugging.
//
// Each returns a value, never a reference: when both arguments have one type,
// `decltype(a < b ? a : b)` is a reference to the function's own by-value
// parameter, gone once it returns. std::decay_t keeps the type a macro's
// expression would have, without the reference.
template <class T, class U>
constexpr auto min(T a, U b) -> std::decay_t<decltype(a < b ? a : b)> {
  return a < b ? a : b;
}
template <class T, class U>
constexpr auto max(T a, U b) -> std::decay_t<decltype(a > b ? a : b)> {
  return a > b ? a : b;
}
template <class T, class L, class H>
constexpr auto constrain(T v, L lo, H hi)
    -> std::decay_t<decltype(v < lo ? lo : (v > hi ? hi : v))> {
  return v < lo ? lo : (v > hi ? hi : v);
}

// Arduino's integer-to-string helpers, which MeshCore uses for its CLI replies.
inline char* ltoa(long value, char* out, int base) {
  if (base == 10) {
    sprintf(out, "%ld", value);
  } else if (base == 16) {
    sprintf(out, "%lx", value);
  } else {
    sprintf(out, "%ld", value);
  }
  return out;
}
inline char* itoa(int value, char* out, int base) { return ltoa(value, out, base); }
inline char* utoa(unsigned long value, char* out, int base) {
  sprintf(out, base == 16 ? "%lx" : "%lu", value);
  return out;
}

inline long random() { return hostRandom(); }
inline long random(long howbig) { return howbig <= 0 ? 0 : hostRandom() % howbig; }
inline long random(long howsmall, long howbig) {
  return howbig <= howsmall ? howsmall : howsmall + random(howbig - howsmall);
}

// Serial is the repeater's console. On a board it is a UART; here it is the
// bridge, so the simulator can carry a node's CLI to whoever is looking at it.
//
// This is the whole point of linking the repeater application rather than
// reimplementing it: the CLI a user reaches when they click a repeater is
// MeshCore's own, with its own commands and its own replies.
#include "Arduino.h"

class HostSerial : public Stream {
 public:
  // The simulator swaps these for the bridge's own queues.
  //
  // `avail` is separate from `in` and not optional. Answering "is there a byte"
  // by taking one and looking at it destroys every other character: MeshCore's
  // CLI calls available() before each read(), so `ver` arrived as `e` and the
  // firmware answered "Unknown command" — correctly, about a command nobody
  // typed.
  void attach(void (*out)(const char*, size_t), int (*in)(), int (*avail)(),
              int (*peek)() = nullptr) {
    out_ = out;
    in_ = in;
    avail_ = avail;
    peek_ = peek;
  }

  int available() override { return avail_ ? avail_() : 0; }
  int read() override { return in_ ? in_() : -1; }
  int peek() override { return peek_ ? peek_() : -1; }
  void flush() override {}

  size_t write(uint8_t c) override {
    char ch = (char)c;
    if (out_) out_(&ch, 1);
    return 1;
  }
  size_t write(const uint8_t* buf, size_t len) override {
    if (out_) out_((const char*)buf, len);
    return len;
  }

  // The full Arduino print family, not the subset today's MeshCore happens to
  // call.
  //
  // This shim has to keep compiling against MeshCore versions nobody has
  // written yet — the firmware pipeline syncs upstream and rebuilds without a
  // human — so implementing the platform contract is the job, and implementing
  // exactly what one release uses is how it breaks on the next one.
  size_t print(const char* s) { return s ? write((const uint8_t*)s, strlen(s)) : 0; }
  size_t print(char c) { return write((uint8_t)c); }
  size_t print(int v) { return printf("%d", v); }
  size_t print(unsigned int v) { return printf("%u", v); }
  size_t print(long v) { return printf("%ld", v); }
  size_t print(unsigned long v) { return printf("%lu", v); }
  size_t print(double v) { return printf("%.2f", v); }
  size_t print(float v) { return printf("%.2f", (double)v); }

  template <class T>
  size_t println(T v) {
    return print(v) + print("\r\n");
  }
  size_t println() { return print("\r\n"); }
  size_t printf(const char* fmt, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    if (n <= 0) return 0;
    if (n > (int)sizeof buf - 1) n = (int)sizeof buf - 1;
    return write((const uint8_t*)buf, (size_t)n);
  }

  void begin(unsigned long) {}
  explicit operator bool() const { return true; }

 private:
  void (*out_)(const char*, size_t) = nullptr;
  int (*in_)() = nullptr;
  int (*avail_)() = nullptr;
  int (*peek_)() = nullptr;
};

extern HostSerial Serial;
