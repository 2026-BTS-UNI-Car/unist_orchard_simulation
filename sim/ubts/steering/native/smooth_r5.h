// SIM model of the SMOOTH hold of the real board firmware r5 (ESP32-S3, BTS7960_Serial_Control.ino with
// CAL_RUNTIME=1 SMOOTH=1, flashed 2026-11-16 23:20, app sha256 f448b2db...) and of the SIM host's ANGLE policy for it
// (docs/steering/SMOOTH_R5_SIM_KO.md).
//
// Firmware mode only: the startup block "steer_law" with kind smooth_r5 (profile schema v5, steer_fw_mode smooth_r5)
// installs smooth_r5_law() as the steering-law hook of the firmware V3 (fw_steer.h steer_law_hook), as abs_hold8.h
// does for the pulse hold. The V3 firmware keeps the link, arming, TTL, watchdog, STOP, drive ramp and fault monitors.
// Without the block the hook stays as it was (null: the V3 law; bit-identical sessions).
//
// Ported from the r5 source (names of the .ino in brackets; counts of the board's 12-bit ADC, ms):
//   [readPotValue] one ADC sample every sample_ms; median of median_of; two medians within pair_tol confirm their
//     mean; r4 slew limit: the confirmed value moves at most max(slew_min, slew_per_ms x ms since the last
//     confirmation) while that confirmation is fresh; no confirmation for more than stale_ms reads -1 (invalid).
//     Plausible: 100 < v < 4000 and hard_right_min - limit_margin < v < hard_left_max + limit_margin.
//   [moveSteeringToAngle] ANGLE a: target = CENTER + lroundf(span * |a| / 20) (float arithmetic), span LEFT - CENTER
//     for a < 0, RIGHT - CENTER otherwise; the same target while holding is ignored; a new target in the running
//     direction while driving (and more than stop_tol away) is taken over without stopping (runs = 1, fresh, the
//     run-time limit restarts); any other target stops the motor (stopMotor(0)) and arms the hold (fresh).
//   [updateSmoothHold] phase 1 (driving): every quiet_ms the driver is switched off and the run uses that quiet read
//     (r5; slew-limited as above against the last quiet read); a run ends (brake, settle) within stop_tol or past
//     the target, or past a CAL mark + guard; no progress_counts of progress for stall_ms raises the PWM floor by
//     boost, still none at pwm_max cancels (STALL); max_run_ms of one run cancels (RUN TIME LIMIT); every
//     ramp_tick_ms PWM = floor + (pwm_max - floor) * min(1, (remaining - stop_tol) / slow), rising at most ramp.
//     Phase 2 (after a run): brake_ms with EN high and both PWM 0 (short brake), then EN low; the rebound peak over
//     the first rebound_window_ms; after settle_ms the settled reading decides (rebound >= rebound_max with the error
//     above restart_tol cancels). Phase 0: the error within stop_tol (or within restart_tol once the target was
//     reached) holds; max_runs runs without reaching the target cancel (NO CONVERGENCE); a target past a CAL mark from
//     a reading at or past it cancels (QUIET OPERATING LIMIT); a reversal waits reverse_pause_ms after the driver went
//     off; else a run starts at pwm_min.
// SIM differences (not in the firmware): the decision loop runs on the V3 servo grid (the profile's servo_hz, 1000 Hz
//   for the smooth profile) instead of every loop pass; the quiet read is split over two servo periods (EN low for
//   one period, then one ADC value of the next period is the quiet read; the firmware switches off for quiet_us and
//   takes the median of five reads in the same pass); the 12-bit value is made from the V3 10-bit model ADC through
//   the V3 calibration marks (as abs_hold8.cc); the SIM ADC path adds the placeholder powered offset and current
//   bursts while the steering driver was powered (adc_* keys, 0 = off).
//
// SIM host policy (the real bridge da39e8a and the executor adapter dad6a93 under SMOOTH): every poll_ms the host
// rounds the executor's goal (command degrees a = -goal_permille * command_max / 1000) half away from zero to step_deg
// and sends it when it differs from the last ANGLE by >= deadband_deg and min_interval_ms has passed (or the board is
// idle). After a hold cancel the cancelled ANGLE is not sent again until the requested angle changes (no force retry).
#ifndef UBTS_STEER_SMOOTH_R5_H
#define UBTS_STEER_SMOOTH_R5_H

#include <stdint.h>

#include "steer_config.h"

namespace ubts_fw {
struct Cal;
}

namespace ubts_steer {

// Cancel reasons (the firmware's 'STEER HOLD CANCELLED: ...' lines).
enum SmoothCancel {
  kSmoothCancelNone = 0,
  kSmoothCancelInvalid = 1,      // ADC STALE/INVALID
  kSmoothCancelStall = 2,        // STALL (no force retry)
  kSmoothCancelRunTime = 3,      // RUN TIME LIMIT
  kSmoothCancelRebound = 4,      // REBOUND (no force retry)
  kSmoothCancelNoConvergence = 5,  // NO CONVERGENCE (no force retry)
  kSmoothCancelQuietLimit = 6    // QUIET OPERATING LIMIT
};

// The board part of the firmware: the steering motor's pins and the SMOOTH hold, without the SIM host and the SIM ADC
// path. counts is the ADC value (12-bit counts) of this pass; the filter takes it when a sample is due.
class SmoothBoard {
 public:
  SmoothBoard();
  void configure(const SmoothLawConfig &cfg);
  // Power-on state without a target (as after a boot or a STOP); the filter primed with counts (a board that has been
  // sampling all along), the driver off long enough for any direction.
  void reset(uint32_t now, int counts);
  // 'ANGLE deg' (command degrees, + right): moveSteeringToAngle(deg < 0, |deg|). False when the firmware refuses it.
  bool angle(uint32_t now, float deg, int counts);
  // 'STOP' / stopMotor(0).
  void stop(uint32_t now);
  // One loop pass (checkSteerTimeout -> updateSmoothHold while a target is active).
  void loop(uint32_t now, int counts);
  // The filtered position (readPotValue) at now, sampling counts when due; -1 = invalid / stale.
  int read_pot(uint32_t now, int counts);

  // Steering driver pins after the pass: EN (both enables), RPWM (F, turns right, counts fall), LPWM (R, left).
  bool en() const { return en_ && !quiet_pending_; }
  int pwm_f() const { return pwm_f_; }
  int pwm_r() const { return pwm_r_; }
  bool braking() const { return en() && pwm_f_ == 0 && pwm_r_ == 0; }
  // Signed output, + toward + permille (left, counts rise): -RPWM or +LPWM while EN is high, else 0.
  int output() const;

  bool target_active() const { return target_active_; }
  int target() const { return target_; }
  int phase() const { return sm_phase_; }
  int runs() const { return runs_; }
  int floor_pwm() const { return floor_; }
  int pwm() const { return sm_pwm_; }
  int quiet_pot() const { return quiet_pot_; }
  uint32_t drive_ms() const { return drive_ms_; }
  uint32_t cancels() const { return cancels_; }
  int last_cancel() const { return last_cancel_; }
  bool quiet_pending() const { return quiet_pending_; }

 private:
  bool plausible(int v) const;
  void reset_filter();
  void deenergize(uint32_t now);
  void stop_motor(uint32_t now);
  void cancel(uint32_t now, int reason);
  void start_run(int direction, int value, uint32_t now);
  void quiet_sample(uint32_t now, int counts);
  void end_run(uint32_t now, int value);
  void write_pwm(int direction, int pwm);
  void update(uint32_t now, int counts);

  SmoothLawConfig cfg_;
  // readPotValue
  bool sample_started_;
  uint32_t sample_ms_;
  int window_[9];
  int window_count_, window_index_;
  int candidate_, confirmed_;
  uint32_t confirmed_ms_;
  // motor 1 (steering) pins and direction
  bool en_;
  int pwm_f_, pwm_r_;
  int direction_;  // motors[0].direction: 1 = F (right), -1 = R (left), 0 = off
  uint32_t steer_off_ms_;
  // hold
  bool target_active_;
  int target_;
  int sm_phase_, sm_dir_, sm_pwm_, floor_;
  uint32_t run_start_, limit_start_, tick_ms_, progress_ms_, settle_start_, drive_ms_;
  int progress_ref_, run_start_pot_, peak_;
  int runs_;
  bool fresh_, braking_;
  int quiet_pot_;
  uint32_t quiet_ms_;
  // SIM only: the quiet read's second servo period is pending (EN low now); cancel bookkeeping for the host
  bool quiet_pending_;
  uint32_t cancels_;
  int last_cancel_;
};

// Install (cfg.present) the hook and reset the law state; without the block nothing changes.
void smooth_r5_install(const SmoothLawConfig &cfg);
void smooth_r5_remove();

// The hook (fw_steer.h SteerLawHook signature).
int16_t smooth_r5_law(uint32_t now, int32_t goal, uint16_t raw, bool fresh, const ubts_fw::Cal &cal,
                      int32_t *cmd_permille, uint8_t *release);

}  // namespace ubts_steer

#endif  // UBTS_STEER_SMOOTH_R5_H
