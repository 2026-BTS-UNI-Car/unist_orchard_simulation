// Drive outputs (see fw_drive.h).
#include "fw_drive.h"

#include "fw_state.h"

namespace ubts_fw {

namespace {

const uint32_t kRampGridMs = 10;
const uint32_t kRampCapMs = 50;
const uint32_t kZeroDwellMs = 120;
const uint32_t kRampPerS = 255;

void ramp_side(DriveSide &s, uint32_t now) {
  uint32_t el = now - s.ramp_at;
  if (el < kRampGridMs) return;
  s.ramp_at = now;
  if (el > kRampCapMs) el = kRampCapMs;  // a stalled loop never jumps more than 50 ms of ramp
  const int32_t d = s.d;
  const int32_t g = s.g;
  int32_t goal;
  if (d == 0) {
    if (g == 0 || static_cast<uint32_t>(now - s.zero_at) < kZeroDwellMs) {
      s.rem = 0;
      return;
    }
    goal = g;
  } else if (g != 0 && sgn32(g) == sgn32(d)) {
    goal = g;
  } else {
    goal = 0;
  }
  if (d == goal) {
    s.rem = 0;
    return;
  }
  const uint32_t sc = static_cast<uint32_t>(s.rem) + el * kRampPerS;
  const int32_t dl = static_cast<int32_t>(sc / 1000u);
  s.rem = static_cast<uint16_t>(sc % 1000u);
  int32_t d2;
  if (abs32(goal) > abs32(d)) {
    int32_t m = abs32(d) + dl;
    if (m > abs32(goal)) m = abs32(goal);
    d2 = sgn32(goal) * m;
  } else {
    int32_t m = abs32(d) - dl;
    if (m < abs32(goal)) m = abs32(goal);
    d2 = sgn32(d) * m;
  }
  if (d != 0 && d2 == 0) s.zero_at = now;
  s.d = static_cast<int16_t>(d2);
  if (d2 == goal) s.rem = 0;
}

DriveSide &side_of(uint8_t pin_digit) { return fw.side[(pin_digit == 8 || pin_digit == 5) ? 0 : 1]; }

int32_t pin_direction(uint8_t pin_digit) { return (pin_digit == 8 || pin_digit == 3) ? 1 : -1; }

uint8_t magnitude_on_pin(int32_t value, int32_t direction) {
  return sgn32(value) == direction ? static_cast<uint8_t>(abs32(value)) : 0;
}

}  // namespace

void drive_boot(uint32_t now) {
  for (int i = 0; i < 2; ++i) {
    fw.side[i].d = fw.side[i].g = 0;
    fw.side[i].rem = 0;
    fw.side[i].ramp_at = fw.side[i].zero_at = now;
  }
  fw.drive_ever = 0;
  fw.last_drive_nonzero = now;
}

void drive_ramp(uint32_t now) {
  ramp_side(fw.side[0], now);
  ramp_side(fw.side[1], now);
  if (fw.side[0].d != 0 || fw.side[1].d != 0) {
    fw.drive_ever = 1;
    fw.last_drive_nonzero = now;
  }
}

void drive_brake(uint32_t now) {
  for (int i = 0; i < 2; ++i) {
    DriveSide &s = fw.side[i];
    if (s.d != 0) s.zero_at = now;
    s.d = s.g = 0;
    s.rem = 0;
    s.ramp_at = now;
  }
}

void drive_release() { fw.side[0].g = fw.side[1].g = 0; }

void drive_set_targets(int16_t left, int16_t right) {
  fw.side[0].g = left;
  fw.side[1].g = right;
}

bool drive_busy() { return fw.side[0].d != 0 || fw.side[1].d != 0; }

bool drive_moving(uint32_t now) {
  if (fw.side[0].d != 0 || fw.side[1].d != 0) return true;
  return fw.drive_ever && static_cast<uint32_t>(now - fw.last_drive_nonzero) < fw.w_par[kPMovms];
}

void drive_bench_request(uint8_t pin_digit, uint8_t value) {
  DriveSide &s = side_of(pin_digit);
  const int32_t dir = pin_direction(pin_digit);
  if (value > 0) {
    s.g = static_cast<int16_t>(dir * value);
  } else if (sgn32(s.g) == dir) {
    s.g = 0;  // a zero only clears a target in this pin's direction
  }
}

uint8_t drive_pin_duty(uint8_t pin_digit) { return magnitude_on_pin(side_of(pin_digit).d, pin_direction(pin_digit)); }

uint8_t drive_pin_target(uint8_t pin_digit) {
  return magnitude_on_pin(side_of(pin_digit).g, pin_direction(pin_digit));
}

void drive_pin_values(uint8_t *lf, uint8_t *lr, uint8_t *rf, uint8_t *rr) {
  const int32_t l = fw.side[0].d, r = fw.side[1].d;
  *lf = l > 0 ? static_cast<uint8_t>(l) : 0;
  *lr = l < 0 ? static_cast<uint8_t>(-l) : 0;
  *rf = r > 0 ? static_cast<uint8_t>(r) : 0;
  *rr = r < 0 ? static_cast<uint8_t>(-r) : 0;
}

}  // namespace ubts_fw
