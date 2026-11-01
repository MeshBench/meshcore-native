// DIO1 interrupts on its rising edge, as RadioLib attaches it.
//
//   c++ -std=c++17 -I variants/host -I vendor/virtual-sx1262/src -I vendor/virtual-sx1262/include \
//       -I vendor/RadioLib/src \
//       variants/host/dio1_edge_test.cpp vendor/virtual-sx1262/src/*.cpp vendor/RadioLib/src/Hal.cpp \
//       -o /tmp/dio1_edge_test && /tmp/dio1_edge_test

#include "SimHal.h"

#include <cstdio>

SimHal sim_hal;
static int interrupts = 0;
static void isr() { interrupts++; }
static int failures = 0;

static void check(bool ok, const char* what) {
  std::printf("  %-4s %s\n", ok ? "ok" : "FAIL", what);
  if (!ok) failures++;
}

static void spi(std::initializer_list<uint8_t> cmd) {
  uint8_t out[16] = {0}, in[16] = {0};
  size_t n = 0;
  for (uint8_t b : cmd) out[n++] = b;
  sim_hal.spiTransfer(out, n, in);
}

int main() {
  sim_hal.attachInterrupt(SimHal::kPinDio1, isr, 0);
  spi({0x08, 0xFF, 0xFF, 0x00, 0x02, 0, 0, 0, 0});  // everything recorded, RxDone on DIO1
  spi({0x82, 0xFF, 0xFF, 0xFF});                    // continuous receive
  const uint8_t frame[] = {0x15, 0x00, 0x11, 0x22};
  uint32_t ms = 1;
  sim_hal.chip().deliverFrame(frame, sizeof(frame));
  for (int i = 0; i < 5; i++) {
    sim_hal.beginTick(ms++);
    sim_hal.servicePendingIrq();
  }
  check(interrupts == 1, "a line left high interrupts once");
  sim_hal.chip().deliverFrame(frame, sizeof(frame));
  sim_hal.beginTick(ms++);
  sim_hal.servicePendingIrq();
  check(interrupts == 1, "a second packet while it is still high raises nothing");
  spi({0x02, 0xFF, 0xFF});  // ClearIrqStatus
  sim_hal.chip().deliverFrame(frame, sizeof(frame));
  sim_hal.beginTick(ms++);
  sim_hal.servicePendingIrq();
  check(interrupts == 2, "once cleared, the next packet interrupts again");
  return failures ? 1 : 0;
}
