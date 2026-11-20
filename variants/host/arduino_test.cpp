// min, max and constrain must return a value, as Arduino's macros give one.
//
// They are templates here, not macros (see HostArduino.h). Declared with
// `-> decltype(a < b ? a : b)`, a call whose two arguments have one type
// returned a reference to the function's own by-value parameter, which is gone
// once it returns. Optimised, `min(frame[2], (uint8_t)64)` in the companion's
// CMD_SET_AUTOADD_CONFIG read whatever was left on the stack and stored 0 as
// the hop limit.
//
//   c++ -std=c++17 -O2 -I variants/host variants/host/arduino_test.cpp -o /tmp/arduino_test
//   /tmp/arduino_test

#include "HostArduino.h"

#include <cstdio>
#include <type_traits>

static int failures = 0;

static void expect(bool ok, const char* what) {
  if (!ok) {
    printf("FAIL %s\n", what);
    failures++;
  }
}

// Inputs the optimiser can't fold, as a frame's bytes are.
volatile uint8_t frame[3] = {0x3a, 0x0e, 0x07};
volatile int lo = 2, hi = 9, v = 12;

int main() {
  uint8_t a = frame[2], b = 64;
  expect(!std::is_reference<decltype(min(a, b))>::value, "min of one type returns a value");
  expect(!std::is_reference<decltype(max(a, b))>::value, "max of one type returns a value");
  expect(!std::is_reference<decltype(constrain(a, a, b))>::value, "constrain of one type returns a value");

  uint8_t hops = min(frame[2], (uint8_t)64);  // as the companion's CMD_SET_AUTOADD_CONFIG calls it
  expect(hops == 7, "min(uint8_t, uint8_t) gives the smaller");
  expect(max(frame[1], frame[2]) == 0x0e, "max(uint8_t, uint8_t) gives the larger");
  int vv = v, l = lo, h = hi;
  expect(constrain(vv, l, h) == 9, "constrain(int, int, int) clamps");
  expect(min(frame[2], 64) == 7, "min of mixed types still works");

  if (failures) return 1;
  printf("ok\n");
  return 0;
}
