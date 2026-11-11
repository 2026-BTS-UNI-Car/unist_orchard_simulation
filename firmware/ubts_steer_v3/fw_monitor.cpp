// Potentiometer input and fault monitors (see fw_monitor.h).
#include "fw_monitor.h"

#include "fw_hw.h"
#include "fw_state.h"
#include "fw_steer.h"

namespace ubts_fw {

namespace {

int32_t g_fault_arg[16];  // argument of the last trip of each bit (F line)

uint16_t median_now() {
  const uint8_t m = static_cast<uint8_t>(fw.w_par[kPMed]);
  uint16_t tmp[kMedMax];
  for (uint8_t k = 0; k < m; ++k) {
    const uint8_t index = static_cast<uint8_t>((fw.med_pos + kMedMax - 1 - k) % kMedMax);
    tmp[k] = fw.med_buf[index];
  }
  for (uint8_t i = 1; i < m; ++i) {  // insertion sort, at most 9 values
    const uint16_t v = tmp[i];
    uint8_t j = i;
    while (j > 0 && tmp[j - 1] > v) {
      tmp[j] = tmp[j - 1];
      --j;
    }
    tmp[j] = v;
  }
  return tmp[m / 2];
}

// THERMAL bit ring (spec 6.7): one bit per processed tick. The index has its own wrap at kThermalTicks: 2^32 is
// not a multiple of 60000, so tick % 60000 would jump at the u32 wrap of the tick counter (decision 12.1-72).
uint32_t thermal_index_ago(uint32_t ticks_ago) {  // 1 <= ticks_ago <= kThermalTicks
  return (fw.thermal_idx + kThermalTicks - ticks_ago) % kThermalTicks;
}

bool thermal_bit_at(uint32_t index) { return (fw.thermal_bits[index >> 3] >> (index & 7u)) & 1u; }

void set_thermal_bit_at(uint32_t index, bool on) {
  const uint8_t mask = static_cast<uint8_t>(1u << (index & 7u));
  if (on) {
    fw.thermal_bits[index >> 3] = static_cast<uint8_t>(fw.thermal_bits[index >> 3] | mask);
  } else {
    fw.thermal_bits[index >> 3] = static_cast<uint8_t>(fw.thermal_bits[index >> 3] & ~mask);
  }
}

// The p, f and raw rings are powers of two, so (tick - ticks_ago) & mask stays right across the u32 wrap.
int16_t past_pos(uint32_t ticks_ago) { return fw.p_ring[(fw.tick - ticks_ago) & (kPRing - 1u)]; }

// Window guards count processed ticks with a saturating counter, not the wrapping tick number (12.1-72).
bool seen_at_least(uint32_t ticks) { return fw.ticks_seen >= ticks; }

// TRACK progress test (decision 12.1-69): the position moved toward cmd by more than stm within the last stt ticks
// (the motion values of STALL). A servo that lags but follows is not a TRACK fault: with the placeholder P servo the
// lag at the rate limit is about 6 deg, above trk. Too few ticks since boot count as no progress.
bool track_progress(int32_t pos) {
  const uint32_t w = fw.w_par[kPStt];
  if (!seen_at_least(w)) return false;
  const int32_t toward = (pos - past_pos(w)) * sgn32(fw.cmd - pos);
  return toward > static_cast<int32_t>(fw.w_par[kPStm]);
}

void trip(uint16_t *mask, uint16_t bit, int32_t arg) {
  *mask = static_cast<uint16_t>(*mask | bit);
  for (int i = 0; i < 16; ++i) {
    if (bit == (1u << i)) g_fault_arg[i] = arg;
  }
}

// DRV_STUCK observation run (spec 6.7, decision 12.1-70), in a servo mode, after the pins of this tick are chosen.
// A run starts with a BRAKE tick (u = 0, steering EN 1) at position stuck_ref and counts every tick with EN 1
// while the output does not show that the driver follows it:
//   * BRAKE ticks continue the run;
//   * a push (u != 0) that starts while the position is still within db of stuck_ref comes from a new target, not
//     from drift under BRAKE: the run ends (a reversed polarity is the POLARITY case);
//   * after a drift under BRAKE (position more than db from stuck_ref when the push starts) the run goes on as long
//     as the push has not moved the position more than db past stuck_ref in its own direction.
// With the default servo (re-push at 2 db) BRAKE ticks alone hold at most about 3 db of motion, so a stuck driver
// could never move dsm (80) within dst; the run now bridges the re-push.
void stuck_run_step(int32_t u, int32_t db) {
  if (!fw.pin_sen) {
    fw.stuck_phase = kStuckIdle;
    fw.run_stuck = 0;
    return;
  }
  const int32_t moved = static_cast<int32_t>(fw.pos) - fw.stuck_ref;
  if (u == 0) {
    if (fw.stuck_phase == kStuckIdle) {
      fw.stuck_phase = kStuckBrake;
      fw.stuck_ref = fw.pos;
      fw.run_stuck = 0;
    } else if (fw.stuck_phase == kStuckDrift && abs32(moved) <= db) {
      fw.stuck_phase = kStuckBrake;  // back near the start: a later push is judged afresh
    }
    fw.run_stuck = sat_inc16(fw.run_stuck);
    return;
  }
  if (fw.stuck_phase == kStuckBrake && abs32(moved) > db) fw.stuck_phase = kStuckDrift;
  if (fw.stuck_phase == kStuckDrift && moved * sgn32(u) <= db) {
    fw.run_stuck = sat_inc16(fw.run_stuck);
  } else {
    fw.stuck_phase = kStuckIdle;
    fw.run_stuck = 0;
  }
}

}  // namespace

void monitor_boot() {
  fw.med_refill = 1;
  fw.med_pos = 0;
  fw.thermal_window = 1000u * fw.w_par[kPThw];
  fw.thermal_cnt = 0;
  fw.thermal_idx = 0;
  fw.ticks_seen = 0;
  fw.stuck_phase = kStuckIdle;
  fw.stuck_ref = 0;
  for (int i = 0; i < 16; ++i) g_fault_arg[i] = 0;
}

void monitor_adc() {
  const uint16_t raw = hw::adc();  // exactly one analogRead per tick
  fw.raw = raw;
  if (fw.med_refill) {
    for (uint8_t i = 0; i < kMedMax; ++i) fw.med_buf[i] = raw;
    fw.med_pos = 0;
    fw.med_refill = 0;
  } else {
    fw.med_buf[fw.med_pos] = raw;
    fw.med_pos = static_cast<uint8_t>((fw.med_pos + 1) % kMedMax);
  }
  fw.f = median_now();
  fw.pos = raw_to_permille(fw.w_cal, fw.f);
  fw.p_ring[fw.tick & (kPRing - 1u)] = fw.pos;
  fw.f_ring[fw.tick & (kFRing - 1u)] = fw.f;
  fw.raw_ring[fw.tick & (kFRing - 1u)] = raw;
  if (!steer_servo_mode()) fw.tgt = fw.cmd = fw.pos;
}

void monitor_refill_median() {
  for (uint8_t i = 0; i < kMedMax; ++i) fw.med_buf[i] = fw.raw;
}

void monitor_thermal_window_changed() {
  fw.thermal_window = 1000u * fw.w_par[kPThw];
  // bits are recorded for the ticks before this one: count the last min(recorded, window) of them
  const uint32_t m = seen_at_least(fw.thermal_window) ? fw.thermal_window : fw.ticks_seen;
  uint32_t count = 0;
  for (uint32_t a = 1; a <= m; ++a) count += thermal_bit_at(thermal_index_ago(a)) ? 1u : 0u;
  fw.thermal_cnt = count;
}

uint16_t monitor_faults() {
  const uint16_t *p = fw.w_par;
  const uint32_t n = fw.tick;
  const int32_t f = fw.f;
  const int32_t pos = fw.pos;
  uint16_t mask = 0;
  const bool open = f < static_cast<int32_t>(p[kPPvmin]);
  const bool shorted = f > static_cast<int32_t>(p[kPPvmax]);
  if (open) trip(&mask, kFPotOpen, f);
  if (shorted) trip(&mask, kFPotShort, f);
  if (!open && !shorted) {
    const uint32_t jw = p[kPJmpw];
    if (seen_at_least(jw)) {
      const int32_t d = f - static_cast<int32_t>(fw.f_ring[(n - jw) & (kFRing - 1u)]);
      if (abs32(d) > static_cast<int32_t>(p[kPJmp])) trip(&mask, kFPotJump, d);
    }
    const uint32_t w = p[kPNsw];
    if (seen_at_least(w - 1u)) {  // raw[n] of this tick and w - 1 earlier samples (nsw >= 2)
      int64_t s1 = 0, s2 = 0;
      for (uint32_t k = 0; k < w; ++k) {
        const int64_t x = fw.raw_ring[(n - k) & (kFRing - 1u)];
        s1 += x;
        s2 += x * x;
      }
      const int64_t ww = static_cast<int64_t>(w);
      const int64_t num = ww * s2 - s1 * s1;
      const int64_t nsd = p[kPNsd];
      if (100 * num > nsd * nsd * ww * ww) trip(&mask, kFPotNoise, static_cast<int32_t>((100 * num) / (ww * ww)));
    }
  }
  const bool servo = steer_servo_mode();
  // runTrack: ticks in a row with |cmd - P| > trk and no progress toward cmd (decision 12.1-69). cmd is the value
  // before this tick's servo update.
  if (servo && abs32(fw.cmd - pos) > static_cast<int32_t>(p[kPTrk]) && !track_progress(pos)) {
    fw.run_track = sat_inc16(fw.run_track);
  } else {
    fw.run_track = 0;
  }
  if (servo) {
    const uint32_t polt = p[kPPolt];
    if (fw.run_pol >= polt && seen_at_least(polt)) {
      const int32_t dp = pos - past_pos(polt);
      if (dp * fw.run_pol_sign < -static_cast<int32_t>(p[kPPolp])) trip(&mask, kFPolarity, dp);
    }
    const uint32_t dst = p[kPDst];
    if (fw.run_stuck >= dst && seen_at_least(dst)) {
      const int32_t dp = pos - past_pos(dst);
      if (abs32(dp) > static_cast<int32_t>(p[kPDsm])) trip(&mask, kFDrvStuck, dp);
    }
    const uint32_t stt = p[kPStt];
    if (fw.run_stall >= stt && seen_at_least(stt)) {
      const int32_t dp = pos - past_pos(stt);
      if (abs32(dp) < static_cast<int32_t>(p[kPStm])) trip(&mask, kFStall, dp);
    }
    if (fw.run_track >= p[kPTrt]) trip(&mask, kFTrack, fw.cmd - pos);
  }
  if (fw.thermal_cnt > p[kPTha]) trip(&mask, kFThermal, static_cast<int32_t>(fw.thermal_cnt));
  fw.softlimit_bit = (cal_complete(fw.w_cal) && (pos > fw.w_cal.soft_l || pos < fw.w_cal.soft_r)) ? 1 : 0;
  return static_cast<uint16_t>(mask & ~fw.latched);
}

int32_t monitor_fault_arg(uint16_t bit) {
  for (int i = 0; i < 16; ++i) {
    if (bit == (1u << i)) return g_fault_arg[i];
  }
  return 0;
}

bool monitor_thermal_active() { return fw.thermal_cnt > fw.w_par[kPTha]; }

bool monitor_arm_blocked() {
  const uint16_t *p = fw.w_par;
  return fw.f < p[kPPvmin] || fw.f > p[kPPvmax] || monitor_thermal_active();
}

void monitor_end_of_tick() {
  const uint16_t *p = fw.w_par;
  if (steer_servo_mode()) {
    const int32_t u = fw.u;
    const int32_t au = abs32(u);
    const int32_t minp = p[kPMinp] > 0 ? p[kPMinp] : 1;
    const int32_t stp = p[kPStp] > 0 ? p[kPStp] : 1;
    if (au >= minp) {
      const int8_t s = static_cast<int8_t>(sgn32(u));
      if (fw.run_pol > 0 && fw.run_pol_sign == s) {
        fw.run_pol = sat_inc16(fw.run_pol);
      } else {
        fw.run_pol = 1;
        fw.run_pol_sign = s;
      }
    } else {
      fw.run_pol = 0;
    }
    fw.run_stall = au >= stp ? sat_inc16(fw.run_stall) : 0;
    stuck_run_step(u, p[kPDb]);
  } else {
    fw.run_pol = fw.run_stall = fw.run_stuck = 0;
    fw.run_pol_sign = 0;
    fw.stuck_phase = kStuckIdle;
  }
  const int32_t thp = p[kPThp] > 0 ? p[kPThp] : 1;
  const bool hot = abs32(static_cast<int32_t>(fw.pin_sa) - static_cast<int32_t>(fw.pin_sb)) >= thp;
  if (seen_at_least(fw.thermal_window) && thermal_bit_at(thermal_index_ago(fw.thermal_window))) --fw.thermal_cnt;
  set_thermal_bit_at(fw.thermal_idx, hot);
  if (hot) ++fw.thermal_cnt;
  fw.thermal_idx = (fw.thermal_idx + 1u) % kThermalTicks;
  if (fw.ticks_seen != 0xFFFFFFFFu) ++fw.ticks_seen;
}

}  // namespace ubts_fw
