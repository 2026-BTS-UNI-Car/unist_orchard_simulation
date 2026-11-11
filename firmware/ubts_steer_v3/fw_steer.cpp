// Steering (see fw_steer.h).
#include "fw_steer.h"

#include "fw_drive.h"
#include "fw_link.h"
#include "fw_state.h"

namespace ubts_fw {

namespace {

const uint32_t kServoDtCapMs = 50;
const uint8_t kPolPwm = 153;        // 60 % (spec 7.10, placeholder P11)
const uint32_t kPolDriveMs = 300;
const uint32_t kPolSettleEndMs = 500;
const uint32_t kPolBackEndMs = 800;
const uint32_t kPolEndMs = 1000;
const int32_t kPolMinCounts = 4;

uint32_t servo_period() { return 1000u / fw.w_par[kPSvhz]; }

// Start of ACTIVE or HOLD. From OFF the pins stay OFF until the first servo update; between ACTIVE and HOLD
// the pins continue (spec 7.1, 7.6).
void enter_servo(uint8_t mode) {
  const bool from_servo = fw.mode == kModeActive || fw.mode == kModeHold;
  fw.mode = mode;
  fw.rrem = 0;
  fw.braked = 0;
  fw.in_db = 0;
  if (!fw.steer_en_prev) fw.last_sign = 0;
  if (!from_servo) {
    fw.servo_live = 0;
    fw.u = 0;
  }
}

int32_t clamp32(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : (v > hi ? hi : v); }

void servo_update(uint32_t dt) {
  const uint16_t *p = fw.w_par;
  const Cal &cal = fw.w_cal;
  const int32_t soft_l = cal.soft_l, soft_r = cal.soft_r;
  const int32_t goal = clamp32(fw.tgt, soft_r, soft_l);
  const int32_t rate = drive_moving(fw.now) ? p[kPRmov] : p[kPRstd];
  if (fw.cmd == goal) {
    fw.rrem = 0;
  } else {
    const int32_t acc = fw.rrem + rate * static_cast<int32_t>(dt);
    const int32_t step = acc / 1000;
    fw.rrem = acc % 1000;
    if (fw.cmd < goal) {
      fw.cmd += step;
      if (fw.cmd > goal) fw.cmd = goal;
    } else {
      fw.cmd -= step;
      if (fw.cmd < goal) fw.cmd = goal;
    }
    if (fw.cmd == goal) fw.rrem = 0;
  }
  const int32_t pos = fw.pos;
  const int32_t e = fw.cmd - pos;
  const int32_t db = p[kPDb];
  int32_t u = 0;
  bool limit_only = false;
  if (fw.braked) {
    if (abs32(e) > 2 * db) {
      fw.braked = 0;  // push again
      fw.in_db = 0;
    } else {
      limit_only = true;
    }
  }
  if (!limit_only) {
    if (abs32(e) <= db) {
      u = 0;
      fw.in_db += static_cast<int32_t>(dt);
      if (fw.in_db >= static_cast<int32_t>(p[kPDwell])) fw.braked = 1;  // brake hold inside the deadband
    } else {
      fw.in_db = 0;
      int32_t mag = (static_cast<int32_t>(p[kPKp]) * abs32(e) + 128) >> 8;
      if (mag < static_cast<int32_t>(p[kPMinp])) mag = p[kPMinp];
      if (mag > static_cast<int32_t>(p[kPMaxp])) mag = p[kPMaxp];
      u = e > 0 ? mag : -mag;
    }
    if (u != 0 && fw.last_sign != 0 && sgn32(u) != fw.last_sign &&
        static_cast<uint32_t>(fw.now - fw.last_nz_ms) < p[kPRev]) {
      u = 0;  // reversal pause
    }
  }
  if (cal_complete(cal)) {  // outward output blocked at the soft limits (spec 7.8)
    if (u > 0 && pos >= soft_l) u = 0;
    if (u < 0 && pos <= soft_r) u = 0;
  }
  if (u != 0) {
    fw.last_sign = static_cast<int8_t>(sgn32(u));
    fw.last_nz_ms = fw.now;
  }
  fw.u = static_cast<int16_t>(u);
}

void set_steer_pins(uint8_t en, uint8_t a, uint8_t b) {
  fw.pin_sen = en;
  fw.pin_sa = a;
  fw.pin_sb = b;
}

}  // namespace

void steer_boot(uint32_t now) {
  fw.mode = kModeOff;
  fw.u = 0;
  fw.servo_live = 0;
  fw.next_servo = now;
  fw.last_servo = now - servo_period();
}

void steer_off() {
  fw.mode = kModeOff;
  fw.u = 0;
  fw.servo_live = 0;
}

void steer_arm() {
  enter_servo(kModeActive);
  fw.tgt = fw.cmd = fw.pos;
}

void steer_hold(uint8_t policy, uint32_t hold_ms) {
  const int32_t cmd_before = fw.cmd;
  enter_servo(kModeHold);
  fw.hold_t0 = fw.now;
  fw.hold_h = hold_ms;
  switch (policy) {
    case kHoldTarget:
      fw.tgt = fw.cmd = cmd_before;  // cmd stops where it is
      break;
    case kHoldCenter:
      fw.tgt = 0;
      fw.cmd = fw.pos;  // then rate-limited toward 0
      break;
    default:
      fw.tgt = fw.cmd = fw.pos;  // hold the measured position
      break;
  }
}

void steer_jog(int16_t pwm, uint16_t ms) {
  fw.mode = kModeJog;
  fw.u = 0;
  fw.servo_live = 0;
  fw.jog_pwm = pwm;
  fw.jog_ms = ms;
  fw.jog_t0 = fw.now;
}

void steer_pol() {
  fw.mode = kModePol;
  fw.u = 0;
  fw.servo_live = 0;
  fw.pol_t0 = fw.now;
  fw.pol_f0 = fw.f;
  fw.pol_f500 = 0;
  fw.pol_have500 = 0;
}

bool steer_busy() { return fw.mode == kModeHold || fw.mode == kModeJog || fw.mode == kModePol; }

bool steer_servo_mode() { return fw.mode == kModeActive || fw.mode == kModeHold; }

void steer_mode_clock() {
  if (fw.mode == kModeHold) {
    if (static_cast<uint32_t>(fw.now - fw.hold_t0) >= fw.hold_h) steer_off();
  } else if (fw.mode == kModeJog) {
    if (static_cast<uint32_t>(fw.now - fw.jog_t0) >= fw.jog_ms) steer_off();
  } else if (fw.mode == kModePol) {
    const uint32_t e = fw.now - fw.pol_t0;
    if (!fw.pol_have500 && e >= kPolSettleEndMs) {
      fw.pol_f500 = fw.f;  // f[t + 500]
      fw.pol_have500 = 1;
    }
    if (e >= kPolEndMs) {
      const int32_t d1 = static_cast<int32_t>(fw.pol_f500) - static_cast<int32_t>(fw.pol_f0);
      const int32_t d2 = static_cast<int32_t>(fw.f) - static_cast<int32_t>(fw.pol_f500);
      const bool ok = abs32(d1) >= kPolMinCounts && abs32(d2) >= kPolMinCounts && sgn32(d1) * sgn32(d2) < 0;
      if (ok) {
        const int32_t mark_dir = sgn32(static_cast<int32_t>(fw.w_cal.raw_l) - static_cast<int32_t>(fw.w_cal.raw_c));
        fw.w_cal.mdir = static_cast<int8_t>(sgn32(d1) * mark_dir);
        fw.w_cal.pol_ok = 1;
        // the driver followed both directions: the POLARITY and DRV_STUCK latches may go (decision 12.1-71)
        fw.latched = static_cast<uint16_t>(fw.latched & ~kFNeedsPolCheck);
      } else {
        fw.w_cal.pol_ok = 0;
      }
      steer_off();
      emit_pol_result(ok, d1, d2);
      emit_cal_line();
    }
  }
}

void steer_output() {
  bool servo_tick = false;
  uint32_t dt = 0;
  if (static_cast<int32_t>(fw.now - fw.next_servo) >= 0) {
    servo_tick = true;
    dt = fw.now - fw.last_servo;
    if (dt > kServoDtCapMs) dt = kServoDtCapMs;
    fw.last_servo = fw.now;
    fw.next_servo += servo_period();
    if (static_cast<int32_t>(fw.now - fw.next_servo) >= 0) fw.next_servo = fw.now + servo_period();
  }
  if (steer_servo_mode() && servo_tick) {
    servo_update(dt);
    fw.servo_live = 1;
  }
  switch (fw.mode) {
    case kModeActive:
    case kModeHold:
      if (!fw.servo_live) {
        set_steer_pins(0, 0, 0);
      } else if (fw.u == 0) {
        set_steer_pins(1, 0, 0);  // BRAKE
      } else {
        const int32_t v = static_cast<int32_t>(fw.u) * fw.w_cal.mdir;
        if (v > 0) {
          set_steer_pins(1, static_cast<uint8_t>(v), 0);
        } else {
          set_steer_pins(1, 0, static_cast<uint8_t>(-v));
        }
      }
      break;
    case kModeJog:
      if (fw.jog_pwm > 0) {
        set_steer_pins(1, static_cast<uint8_t>(fw.jog_pwm), 0);
      } else {
        set_steer_pins(1, 0, static_cast<uint8_t>(-fw.jog_pwm));
      }
      break;
    case kModePol: {
      const uint32_t e = fw.now - fw.pol_t0;
      if (e < kPolDriveMs) {
        set_steer_pins(1, kPolPwm, 0);
      } else if (e >= kPolSettleEndMs && e < kPolBackEndMs) {
        set_steer_pins(1, 0, kPolPwm);
      } else {
        set_steer_pins(0, 0, 0);
      }
      break;
    }
    default:
      set_steer_pins(0, 0, 0);
      break;
  }
}

}  // namespace ubts_fw
