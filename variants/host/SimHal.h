#pragma once

// The hardware RadioLib runs on, when there is no hardware.
//
// RadioLib is written to be ported: everything it needs from a platform is
// behind RadioLibHal, sixteen virtual methods. Implementing them is what lets
// MeshCore's own CustomSX1262 driver - and RadioLib itself - run unmodified
// against a simulated chip, instead of both being replaced by a hand-written
// stub whose behaviour is ours rather than theirs.
//
// Two things here are not obvious and both matter.
//
// **Time is the node's, not the machine's.** millis() and micros() return
// simulated time, because a lockstep simulation is reproducible only if
// everything the firmware can observe comes from the simulation. Wall time
// would make two runs of one seed differ.
//
// **Waiting must always end.** RadioLib spins on the BUSY line and on its own
// timeouts, and a spin that does not advance simulated time never terminates -
// which in this codebase means a frozen window and a control socket that stops
// answering. So every blocking call advances the local clock, and BUSY is never
// asserted: we model command *latency* rather than the BUSY handshake, which
// removes the whole class of hang for something the firmware cannot tell apart.

#include <Hal.h>
#include <stdint.h>

#include "VirtualSX1262.h"

class SimHal : public RadioLibHal {
 public:
  // Levels and modes as the Arduino-flavoured constants RadioLib expects.
  SimHal()
      : RadioLibHal(/*input*/ 0x0, /*output*/ 0x1, /*low*/ 0x0, /*high*/ 0x1,
                    /*rising*/ 0x1, /*falling*/ 0x2) {}

  void init() override {}
  void term() override {}

  void pinMode(uint32_t, uint32_t) override {}
  void digitalWrite(uint32_t, uint32_t) override {}

  uint32_t digitalRead(uint32_t pin) override {
    // BUSY, the only line RadioLib reads. Always ready: see the note above.
    if (pin == kPinBusy) return GpioLevelLow;
    if (pin == kPinDio1) return chip_.irqAsserted() ? GpioLevelHigh : GpioLevelLow;
    return GpioLevelLow;
  }

  void attachInterrupt(uint32_t, void (*cb)(void), uint32_t) override { isr_ = cb; }
  void detachInterrupt(uint32_t) override { isr_ = nullptr; }

  // Every wait advances the node's clock. Nothing here can spin for ever.
  void delay(RadioLibTime_t ms) override { advanceUs(ms * 1000); }
  void delayMicroseconds(RadioLibTime_t us) override { advanceUs(us); }
  void yield() override { advanceUs(50); }

  RadioLibTime_t millis() override { return localUs_ / 1000; }
  RadioLibTime_t micros() override { return localUs_; }

  long pulseIn(uint32_t, uint32_t, RadioLibTime_t) override { return 0; }

  void spiBegin() override {}
  void spiBeginTransaction() override {}
  void spiEndTransaction() override {}
  void spiEnd() override {}

  void spiTransfer(uint8_t* out, size_t len, uint8_t* in) override {
    // A command costs time even though BUSY never rises, so the firmware's own
    // timing arithmetic sees a plausible driver rather than an instant one.
    advanceUs(30 + len * 2);
    chip_.spiTransfer(out, len, in);
    // A command that clears the flags drops the line, so the next one is an edge.
    if (!chip_.irqAsserted()) dio1High_ = false;
  }

  // ---- the simulator's side ----

  VirtualSX1262& chip() { return chip_; }

  // Called by the bridge at the start of each tick. The node's local clock is
  // reconciled to network time here: if the firmware spent longer inside a tick
  // than the tick was worth, the excess carries rather than being forgiven.
  void beginTick(uint32_t networkMs) {
    uint64_t want = (uint64_t)networkMs * 1000;
    if (want > localUs_) localUs_ = want;
    chip_.tick(localUs_ / 1000);
  }

  // Deliver a pending interrupt to RadioLib, if the chip has raised one. Called
  // once per tick rather than from inside the chip, so an ISR never runs in the
  // middle of an SPI transaction the way it cannot on real hardware either.
  //
  // On the rising edge only, as RadioLib attaches DIO1: a line left high by a
  // firmware that has not yet read the chip interrupts once, not every tick, and
  // a second packet landing while it is still high raises nothing.
  void servicePendingIrq() {
    const bool high = chip_.irqAsserted();
    if (isr_ && high && !dio1High_ && !inIsr_) {
      inIsr_ = true;
      isr_();
      inIsr_ = false;
    }
    dio1High_ = high;
  }

  static constexpr uint32_t kPinNss = 1;
  static constexpr uint32_t kPinDio1 = 2;
  static constexpr uint32_t kPinReset = 3;
  static constexpr uint32_t kPinBusy = 4;

 private:
  void advanceUs(uint64_t us) {
    localUs_ += us;
    chip_.tick(localUs_ / 1000);
  }

  VirtualSX1262 chip_;
  void (*isr_)(void) = nullptr;
  bool inIsr_ = false;
  bool dio1High_ = false;
  uint64_t localUs_ = 0;
};

extern SimHal sim_hal;
