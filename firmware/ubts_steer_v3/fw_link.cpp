// Serial link (see fw_link.h). Check order of a host line (spec 4.2): long, ctrl, STOP special, crc, cmd, nocrc,
// state, syntax, then the per-command checks. A rejected line changes nothing except the bad-line count and
// the BENCH brake on framing errors.
#include "fw_link.h"

#include <string.h>

#include "fw_cal.h"
#include "fw_config.h"
#include "fw_crc.h"
#include "fw_drive.h"
#include "fw_hw.h"
#include "fw_main.h"
#include "fw_monitor.h"
#include "fw_params.h"
#include "fw_state.h"
#include "fw_steer.h"
#include "fw_text.h"

namespace ubts_fw {

namespace {

enum Reason {
  kRLong = 0, kRCtrl, kRCrc, kRCmd, kRNocrc, kRState, kRSyntax, kRRange, kRBoot, kRNocal, kRCal, kRPol, kRFault,
  kROld, kRBusy, kRWear, kRKey, kRPin
};
const char *const kReasonText[] = {"long", "ctrl",  "crc", "cmd", "nocrc", "state", "syntax", "range", "boot",
                                   "nocal", "cal", "pol", "fault", "old", "busy",  "wear",   "key",   "pin"};

const char *const kFaultName[16] = {"NOCAL",   "POT_OPEN", "POT_SHORT", "POT_JUMP", "POT_NOISE", "POLARITY",
                                    "DRV_STUCK", "SOFTLIMIT", "STALL",   "TRACK",    "THERMAL",   "TTL",
                                    "WATCHDOG", "CRC",      "BOOT",      "RESERVED"};

enum Cmd { kCmdUnknown, kCmdInfo, kCmdPing, kCmdStop, kCmdArm, kCmdDisarm, kCmdC, kCmdCal, kCmdCfg, kCmdBench, kCmdD };

const uint8_t kMaxTokens = 8;
const uint8_t kRxPerTick = 64;
const uint8_t kMaxCommits = 8;

struct Tokens {
  const char *s;
  uint8_t start[kMaxTokens];
  uint8_t len[kMaxTokens];
  uint8_t count;
  bool overflow;  // more than kMaxTokens tokens
  bool empty;     // an empty token (leading, trailing or double space)
  bool star;      // a '*' inside the body
  const char *at(uint8_t i) const { return s + start[i]; }
  bool is(uint8_t i, const char *word) const { return i < count && token_is(at(i), len[i], word); }
};

void split(const char *s, uint8_t n, Tokens *t) {
  t->s = s;
  t->count = 0;
  t->overflow = t->empty = t->star = false;
  uint8_t begin = 0;
  for (uint8_t i = 0; i <= n; ++i) {
    if (i == n || s[i] == ' ') {
      if (t->count < kMaxTokens) {
        t->start[t->count] = begin;
        t->len[t->count] = static_cast<uint8_t>(i - begin);
        ++t->count;
      } else {
        t->overflow = true;
      }
      if (i == begin) t->empty = true;
      begin = static_cast<uint8_t>(i + 1);
    } else if (s[i] == '*') {
      t->star = true;
    }
  }
}

// --- TX ring -------------------------------------------------------------------------------------------------
void push_line(const OutLine &line) {
  if (static_cast<uint16_t>(fw.tx_len + line.len) > kTxRing) {
    if (fw.drop != 0xFFFFu) ++fw.drop;  // whole lines only (spec 5.10)
    return;
  }
  for (uint8_t i = 0; i < line.len; ++i) {
    fw.tx[(fw.tx_head + fw.tx_len) % kTxRing] = static_cast<uint8_t>(line.buf[i]);
    ++fw.tx_len;
  }
}

void reject(uint8_t reason, int32_t q) {
  OutLine line;
  line.clear();
  line.str("N ");
  if (q >= 0) {
    line.u32(static_cast<uint32_t>(q));
  } else {
    line.ch('-');
  }
  line.ch(' ');
  line.str(kReasonText[reason]);
  line.finish();
  push_line(line);
}

void emit_ack_cfg(int key) {
  OutLine line;
  line.clear();
  line.str("A CFG ");
  line.str(kParams[key].name);
  line.ch(' ');
  line.u32(fw.w_par[key]);
  line.finish();
  push_line(line);
}

void emit_ack_jog(int32_t pwm, uint32_t ms) {
  OutLine line;
  line.clear();
  line.str("A JOG ");
  line.i32(pwm);
  line.ch(' ');
  line.u32(ms);
  line.finish();
  push_line(line);
}

void emit_pin_ack(uint8_t digit, uint8_t duty, uint8_t target) {
  OutLine line;
  line.clear();
  line.str("P D");
  line.u32(digit);
  line.ch(' ');
  line.u32(duty);
  line.ch(' ');
  line.u32(target);
  line.finish();
  push_line(line);
}

// --- bad lines and the CRC burst fault (spec 6.6) ---------------------------------------------------------------
void count_bad_line() {
  if (fw.bad_run < 0xFFu) ++fw.bad_run;
  if (fw.bad_run < fw.w_par[kPCrcn]) return;
  drive_brake(fw.now);
  if (fw.mode == kModeActive) {
    steer_hold(kHoldMeasured, fw.w_par[kPHloss]);
  } else if (fw.mode == kModeJog || fw.mode == kModePol) {
    steer_off();
  }
  if (fw.state == kStArmed) clear_c_state();
  fw.state = kStDisarmed;
  if (!(fw.latched & kFCrc)) {
    fw.latched = static_cast<uint16_t>(fw.latched | kFCrc);
    emit_fault_line(kFCrc, fw.w_par[kPCrcn]);
  }
  fw.bad_run = 0;
}

// The byte that makes the line a framing error (spec 2.2, decision 12.1-48): in BENCH the drive stops in this
// tick, as V2 called stopDirections() at the offending byte. The N line and the bad-line count wait for the LF.
void framing_error_seen(uint8_t err) {
  fw.line_err = err;
  if (fw.state == kStBench) drive_brake(fw.now);  // V2 stopDirections
}

void framing_error(uint8_t err) {
  reject(err == kFrameLong ? kRLong : kRCtrl, -1);
  count_bad_line();
}

// --- commands -----------------------------------------------------------------------------------------------
void cmd_stop() {
  switch (fw.state) {
    case kStArmed:
      drive_brake(fw.now);
      steer_hold(static_cast<uint8_t>(fw.w_par[kPStoppol]), fw.w_par[kPHstop]);
      fw.state = kStDisarmed;
      break;
    case kStBench:
      drive_brake(fw.now);
      fw.state = kStDisarmed;
      break;
    default:
      drive_brake(fw.now);
      if (fw.mode == kModeJog || fw.mode == kModePol) steer_off();  // HOLD continues unchanged
      break;
  }
  clear_c_state();
}

void cmd_disarm() {
  switch (fw.state) {
    case kStArmed:
      drive_release();
      steer_hold(kHoldMeasured, fw.w_par[kPHdis]);
      fw.state = kStDisarmed;
      break;
    case kStBench:
      drive_release();
      fw.state = kStDisarmed;
      break;
    default:
      if (fw.mode == kModeJog || fw.mode == kModePol) steer_off();
      break;
  }
  clear_c_state();
}

void cmd_arm(const Tokens &t) {
  uint32_t boot = 0;
  if (t.count != 2 || !parse_hex(t.at(1), t.len[1], 8, &boot)) return reject(kRSyntax, -1);
  if (boot != fw.boot_id) return reject(kRBoot, -1);
  if (fw.mode == kModeJog || fw.mode == kModePol) return reject(kRBusy, -1);
  if (!fw.r_valid) return reject(kRNocal, -1);
  if (!cal_equal(fw.w_cal, fw.r_cal)) return reject(kRCal, -1);
  // pol: no stored polarity check, or a hardware latch (POLARITY, DRV_STUCK) that only a passing CAL POL or a
  // reset clears (decision 12.1-71)
  if (!fw.r_cal.pol_ok || (fw.latched & kFNeedsPolCheck)) return reject(kRPol, -1);
  if (monitor_arm_blocked()) return reject(kRFault, -1);
  fw.state = kStArmed;
  steer_arm();
  fw.latched = static_cast<uint16_t>(fw.latched & ~kFArmClears);
  fw.bad_run = 0;
  clear_c_state();
  fw.last_input = fw.now;
}

void cmd_c(const Tokens &t, int32_t q_token) {
  uint32_t q = 0, ttl = 0;
  int32_t l = 0, r = 0, s = 0;
  if (t.count != 6 || !parse_uint(t.at(1), t.len[1], &q) || !parse_uint(t.at(2), t.len[2], &ttl) ||
      !parse_int(t.at(3), t.len[3], &l) || !parse_int(t.at(4), t.len[4], &r) || !parse_int(t.at(5), t.len[5], &s)) {
    return reject(kRSyntax, q_token);
  }
  if (q > 65535u || ttl < 1u || ttl > fw.w_par[kPTtlmax] || abs32(l) > 255 || abs32(r) > 255 || abs32(s) > 1000) {
    return reject(kRRange, q_token);
  }
  const uint16_t diff = static_cast<uint16_t>(q - fw.last_q);
  if (diff < 1u || diff > 32767u) return reject(kROld, q_token);
  fw.last_q = static_cast<uint16_t>(q);
  drive_set_targets(static_cast<int16_t>(l), static_cast<int16_t>(r));
  fw.tgt = s;
  fw.c_valid = 1;
  fw.c_ms = fw.now;
  fw.c_ttl = static_cast<uint16_t>(ttl);
  fw.ttl_bit = 0;
  fw.last_input = fw.now;
}

uint16_t next_cal_id() {
  uint16_t prev = 0;
  if (fw.r_valid) {
    prev = fw.r_cal_id;
  } else {
    uint8_t rec[kRecordBytes];
    for (uint8_t i = 0; i < kRecordBytes; ++i) rec[i] = hw::eeprom_read(i);
    uint16_t id = 0;
    if (record_tombstone_id(rec, &id)) prev = id;
  }
  return prev == 0xFFFFu ? 1 : static_cast<uint16_t>(prev + 1u);
}

void cal_save() {
  const uint16_t id = next_cal_id();
  uint8_t rec[kRecordBytes];
  record_encode(rec, false, id, fw.w_cal, fw.w_par);
  for (uint8_t i = 0; i < kRecordBytes; ++i) hw::eeprom_write(i, rec[i]);
  ++fw.commits;
  if (!hw::eeprom_commit()) return;  // flash write failed: R stays as it was (real chip only)
  fw.r_valid = 1;
  fw.r_cal_id = id;
  fw.r_cal = fw.w_cal;
  for (int i = 0; i < kParamCount; ++i) fw.r_par[i] = fw.w_par[i];
  fw.r_crc = record_crc(rec);
}

// CAL ERASE (spec 9.5). A failed commit (real chip only) keeps R, W and the NOCAL bit as they were and sends no
// F NOCAL line: the reply is then the cal line alone, still with id=<R's cal_id> (decision 12.1-73, the SAVE rule).
void cal_erase() {
  if (fw.r_valid) {
    uint8_t rec[kRecordBytes];
    for (uint8_t i = 0; i < kRecordBytes; ++i) rec[i] = hw::eeprom_read(i);
    rec[3] = 'X';
    const uint32_t crc = record_crc(rec);
    for (int i = 0; i < 4; ++i) rec[kRecordCrcOffset + i] = static_cast<uint8_t>((crc >> (8 * i)) & 0xFFu);
    hw::eeprom_write(3, rec[3]);
    for (int i = 0; i < 4; ++i) hw::eeprom_write(kRecordCrcOffset + i, rec[kRecordCrcOffset + i]);
    ++fw.commits;
    if (!hw::eeprom_commit()) return;
    fw.r_valid = 0;
  }
  cal_clear(&fw.w_cal);
  fw.latched = static_cast<uint16_t>(fw.latched | kFNocal);
  emit_fault_line(kFNocal, 0);
}

void cmd_cal(const Tokens &t) {
  if (t.count < 2) return reject(kRSyntax, -1);
  if (t.is(1, "GET")) {
    if (t.count != 2) return reject(kRSyntax, -1);
    return emit_cal_line();
  }
  const bool busy = steer_busy() || drive_busy();
  const uint16_t *p = fw.w_par;
  if (t.is(1, "JOG")) {
    int32_t pwm = 0;
    uint32_t ms = 0;
    if (t.count != 4 || !parse_int(t.at(2), t.len[2], &pwm) || !parse_uint(t.at(3), t.len[3], &ms)) {
      return reject(kRSyntax, -1);
    }
    if (pwm == 0 || abs32(pwm) > 255 || ms < 1u || ms > 500u) return reject(kRRange, -1);
    if (busy) return reject(kRBusy, -1);
    if (monitor_arm_blocked()) return reject(kRFault, -1);  // the active conditions of ARM (12.1-68)
    steer_jog(static_cast<int16_t>(pwm), static_cast<uint16_t>(ms));
    return emit_ack_jog(pwm, ms);
  }
  if (t.is(1, "POL")) {
    if (t.count != 2) return reject(kRSyntax, -1);
    if (busy) return reject(kRBusy, -1);
    const Cal &c = fw.w_cal;
    if (c.raw_l == kMarkUnset || c.raw_c == kMarkUnset ||
        abs32(static_cast<int32_t>(c.raw_l) - static_cast<int32_t>(c.raw_c)) < 50) {
      return reject(kRCal, -1);
    }
    if (monitor_arm_blocked()) return reject(kRFault, -1);  // the active conditions of ARM (12.1-68)
    return steer_pol();
  }
  if (t.is(1, "MARK")) {
    if (t.count != 3 || !(t.is(2, "L") || t.is(2, "C") || t.is(2, "R"))) return reject(kRSyntax, -1);
    if (busy) return reject(kRBusy, -1);
    if (fw.f < p[kPCwmin] || fw.f > p[kPCwmax]) return reject(kRCal, -1);
    if (t.is(2, "L")) {
      fw.w_cal.raw_l = fw.f;
    } else if (t.is(2, "C")) {
      fw.w_cal.raw_c = fw.f;
    } else {
      fw.w_cal.raw_r = fw.f;
    }
    return emit_cal_line();
  }
  if (t.is(1, "SOFT")) {
    uint32_t l = 0;
    int32_t r = 0;
    if (t.count != 4 || !parse_uint(t.at(2), t.len[2], &l) || !parse_int(t.at(3), t.len[3], &r)) {
      return reject(kRSyntax, -1);
    }
    if (l < 1u || l > 1000u || r < -1000 || r > -1) return reject(kRRange, -1);
    if (busy) return reject(kRBusy, -1);
    fw.w_cal.soft_l = static_cast<int16_t>(l);
    fw.w_cal.soft_r = static_cast<int16_t>(r);
    return emit_cal_line();
  }
  if (t.is(1, "SAVE")) {
    if (t.count != 2) return reject(kRSyntax, -1);
    if (busy) return reject(kRBusy, -1);
    if (!cal_complete(fw.w_cal) || !marks_in_window(fw.w_cal, p[kPCwmin], p[kPCwmax]) || !params_all_ok(p)) {
      return reject(kRCal, -1);
    }
    if (fw.commits >= kMaxCommits) return reject(kRWear, -1);
    cal_save();
    return emit_cal_line();
  }
  if (t.is(1, "ERASE")) {
    if (t.count != 2) return reject(kRSyntax, -1);
    if (busy) return reject(kRBusy, -1);
    if (fw.commits >= kMaxCommits) return reject(kRWear, -1);
    cal_erase();
    return emit_cal_line();
  }
  reject(kRSyntax, -1);
}

void cmd_cfg(const Tokens &t) {
  uint32_t value = 0;
  if (t.count != 2 && t.count != 3) return reject(kRSyntax, -1);
  if (t.count == 3 && !parse_uint(t.at(2), t.len[2], &value)) return reject(kRSyntax, -1);
  const int key = param_find(t.at(1), t.len[1]);
  if (key < 0) return reject(kRKey, -1);
  if (t.count == 2) return emit_ack_cfg(key);
  if (!param_value_ok(key, value)) return reject(kRRange, -1);
  uint16_t trial[kParamCount];
  for (int i = 0; i < kParamCount; ++i) trial[i] = fw.w_par[i];
  trial[key] = static_cast<uint16_t>(value);
  if (!params_cross_ok(trial)) return reject(kRRange, -1);
  if (steer_busy()) return reject(kRBusy, -1);
  fw.w_par[key] = static_cast<uint16_t>(value);
  if (key == kPMed) monitor_refill_median();
  if (key == kPThw) monitor_thermal_window_changed();
  emit_ack_cfg(key);
}

void cmd_bench(const Tokens &t) {
  uint32_t boot = 0;
  if (t.count != 2 || !parse_hex(t.at(1), t.len[1], 8, &boot)) return reject(kRSyntax, -1);
  if (boot != fw.boot_id) return reject(kRBoot, -1);
  if (steer_busy() || drive_busy()) return reject(kRBusy, -1);
  fw.state = kStBench;
  drive_set_targets(0, 0);
  steer_off();
  fw.last_input = fw.now;
}

void cmd_d(const Tokens &t, uint8_t digit) {
  uint32_t v = 0;
  if (t.count != 2 || !parse_uint(t.at(1), t.len[1], &v)) return reject(kRSyntax, -1);
  if (v > 255u) return reject(kRRange, -1);
  if (digit == 2 || digit == 4 || digit == 6 || digit == 7) return reject(kRPin, -1);
  fw.last_input = fw.now;
  if (digit == 1) return emit_pin_ack(1, 255, 255);  // drive EN stays HIGH ("power locked")
  drive_bench_request(digit, static_cast<uint8_t>(v));
  emit_pin_ack(digit, drive_pin_duty(digit), drive_pin_target(digit));
}

Cmd identify(const Tokens &t, uint8_t *digit) {
  const char *s = t.at(0);
  const uint8_t n = t.len[0];
  if (token_is(s, n, "INFO")) return kCmdInfo;
  if (token_is(s, n, "PING")) return kCmdPing;
  if (token_is(s, n, "STOP")) return kCmdStop;
  if (token_is(s, n, "ARM")) return kCmdArm;
  if (token_is(s, n, "DISARM")) return kCmdDisarm;
  if (token_is(s, n, "C")) return kCmdC;
  if (token_is(s, n, "CAL")) return kCmdCal;
  if (token_is(s, n, "CFG")) return kCmdCfg;
  if (token_is(s, n, "BENCH")) return kCmdBench;
  if (n == 2 && s[0] == 'D' && s[1] >= '0' && s[1] <= '8') {
    *digit = static_cast<uint8_t>(s[1] - '0');
    return kCmdD;
  }
  return kCmdUnknown;
}

bool needs_crc(Cmd c) {
  return c == kCmdArm || c == kCmdDisarm || c == kCmdC || c == kCmdCal || c == kCmdCfg || c == kCmdBench;
}

bool state_allows(Cmd c, const Tokens &t) {
  const bool cal_get = c == kCmdCal && t.is(1, "GET");
  const bool cfg_query = c == kCmdCfg && t.count == 2;
  switch (fw.state) {
    case kStArmed:
      return c == kCmdInfo || c == kCmdPing || c == kCmdStop || c == kCmdDisarm || c == kCmdC || cal_get ||
             cfg_query;
    case kStBench:
      return c == kCmdD || c == kCmdPing || c == kCmdInfo || c == kCmdStop || c == kCmdDisarm || cal_get ||
             cfg_query;
    default:
      return c != kCmdC && c != kCmdD;
  }
}

bool is_stop_special(const char *s, uint8_t n) {
  return (n == 4 && memcmp(s, "STOP", 4) == 0) || (n == 7 && memcmp(s, "STOP*", 5) == 0);
}

void process_line(const char *s, uint8_t n) {
  if (is_stop_special(s, n)) {
    bool good = n == 4;  // a STOP*HH line is good only with the right CRC; never bad
    if (n == 7) {
      const int hi = hex_digit(s[5]), lo = hex_digit(s[6]);
      good = hi >= 0 && lo >= 0 && crc8(reinterpret_cast<const uint8_t *>(s), 4) == ((hi << 4) | lo);
    }
    if (good) fw.bad_run = 0;
    return cmd_stop();
  }
  const bool has_crc = n >= 3 && s[n - 3] == '*';
  uint8_t body = n;
  if (has_crc) {
    const int hi = hex_digit(s[n - 2]), lo = hex_digit(s[n - 1]);
    body = static_cast<uint8_t>(n - 3);
    if (hi < 0 || lo < 0 || crc8(reinterpret_cast<const uint8_t *>(s), body) != ((hi << 4) | lo)) {
      reject(kRCrc, -1);
      return count_bad_line();
    }
  }
  fw.bad_run = 0;  // a good line
  Tokens t;
  split(s, body, &t);
  uint8_t digit = 0;
  const Cmd c = identify(t, &digit);
  int32_t q = -1;
  uint32_t qv = 0;
  if (c == kCmdC && t.count >= 2 && parse_uint(t.at(1), t.len[1], &qv) && qv <= 65535u) q = static_cast<int32_t>(qv);
  if (c == kCmdUnknown) return reject(kRCmd, q);
  if (needs_crc(c) && !has_crc) return reject(kRNocrc, q);
  if (!state_allows(c, t)) return reject(kRState, q);
  if (t.empty || t.star || t.overflow) return reject(kRSyntax, q);
  switch (c) {
    case kCmdInfo:
      if (t.count != 1) return reject(kRSyntax, -1);
      emit_banner();
      return emit_cal_line();
    case kCmdPing:
      if (t.count != 1) return reject(kRSyntax, -1);
      fw.last_input = fw.now;
      return;
    case kCmdStop:
      return reject(kRSyntax, -1);  // a STOP line that is not the special form (e.g. "STOP x")
    case kCmdArm:
      return cmd_arm(t);
    case kCmdDisarm:
      if (t.count != 1) return reject(kRSyntax, -1);
      return cmd_disarm();
    case kCmdC:
      return cmd_c(t, q);
    case kCmdCal:
      return cmd_cal(t);
    case kCmdCfg:
      return cmd_cfg(t);
    case kCmdBench:
      return cmd_bench(t);
    case kCmdD:
      return cmd_d(t, digit);
    default:
      return reject(kRCmd, q);
  }
}

void feed_byte(uint8_t c) {
  if (c == '\n') {
    const uint8_t err = fw.line_err, n = fw.line_len;
    fw.line_err = kFrameOk;
    fw.line_len = 0;
    fw.line_cr = 0;
    if (err != kFrameOk) return framing_error(err);
    if (n == 0) return;  // empty line (LF or CR LF): ignored, no reply, no count, no watchdog refresh
    fw.line[n] = 0;
    return process_line(fw.line, n);
  }
  if (fw.line_err != kFrameOk) return;  // discarding up to the next LF
  if (fw.line_cr) return framing_error_seen(kFrameCtrl);  // CR must be the byte right before LF
  if (c == '\r') {
    fw.line_cr = 1;
    return;
  }
  if (c < 0x20 || c > 0x7E) return framing_error_seen(kFrameCtrl);
  if (fw.line_len >= kLineMax) return framing_error_seen(kFrameLong);  // the 64th content byte
  fw.line[fw.line_len++] = static_cast<char>(c);
}

}  // namespace

void link_boot() {
  fw.line_len = 0;
  fw.line_err = kFrameOk;
  fw.line_cr = 0;
  fw.tx_head = fw.tx_len = 0;
}

void link_rx() {
  for (uint8_t i = 0; i < kRxPerTick; ++i) {
    if (hw::rx_available() <= 0) break;
    const int c = hw::rx_read();
    if (c < 0) break;
    feed_byte(static_cast<uint8_t>(c));
  }
}

// Step 12 (spec 5.10, 11.3, decision 12.1-66). ESP8266 Arduino core 3.1.2: availableForWrite() is 128 minus the
// TX FIFO fill, and Serial.write() waits while the fill is 127 or more. So at most room - 1 bytes go out per
// write and room <= 1 returns at once: Serial.write never waits and loop() never stalls on serial output.
void link_tx_pump() {
  while (fw.tx_len > 0) {
    const int room = hw::tx_room();
    if (room <= 1) return;
    uint16_t chunk = static_cast<uint16_t>(kTxRing - fw.tx_head);  // contiguous bytes from the head
    if (chunk > fw.tx_len) chunk = fw.tx_len;
    if (static_cast<int>(chunk) > room - 1) chunk = static_cast<uint16_t>(room - 1);
    const size_t sent = hw::tx_write(fw.tx + fw.tx_head, chunk);
    if (sent == 0) return;
    fw.tx_head = static_cast<uint16_t>((fw.tx_head + sent) % kTxRing);
    fw.tx_len = static_cast<uint16_t>(fw.tx_len - sent);
  }
}

void emit_banner() {
  OutLine line;
  line.clear();
  line.str("UBTS_STEER_V3 hello proto=3 build=");
  line.str(kFwBuild);
  line.str(" dev=");
  line.hex(fw.chip_id & 0xFFFFFFu, 6);
  line.str(" boot=");
  line.hex(fw.boot_id, 8);
  line.str(" rst=");
  switch (fw.rst_reason) {
    case 0: line.str("power_on"); break;
    case 1: line.str("hw_wdt"); break;
    case 2: line.str("exception"); break;
    case 3: line.str("soft_wdt"); break;
    case 4: line.str("soft_restart"); break;
    case 5: line.str("deep_sleep"); break;
    case 6: line.str("ext_sys"); break;
    default: line.str("unknown"); break;
  }
  line.str(" hw=");
  line.str(kHwProfile);
  line.str(" out=drive2,steer1 cal=");
  if (fw.r_valid) {
    line.u32(fw.r_cal_id);
  } else {
    line.str("none");
  }
  line.str(" sim=");
  line.u32(kSimBuild);
  line.str(" wd=");
  line.u32(fw.w_par[kPWd]);
  line.str(" ttl=");
  line.u32(fw.w_par[kPTtlmax]);
  line.str(" st=");
  line.u32(fw.w_par[kPSthz]);
  line.str(" drop=");
  line.u32(fw.drop);
  line.finish();
  push_line(line);
}

void emit_cal_line() {
  const Cal &w = fw.w_cal;
  OutLine line;
  line.clear();
  line.str("UBTS_STEER_V3 cal id=");
  if (fw.r_valid) {
    line.u32(fw.r_cal_id);
  } else {
    line.str("none");
  }
  line.str(" crc=");
  line.hex(fw.r_valid ? fw.r_crc : 0u, 8);
  const uint16_t marks[3] = {w.raw_l, w.raw_c, w.raw_r};
  const char *const names[3] = {" left=", " center=", " right="};
  for (int i = 0; i < 3; ++i) {
    line.str(names[i]);
    if (marks[i] == kMarkUnset) {
      line.ch('-');
    } else {
      line.u32(marks[i]);
    }
  }
  line.str(" soft=");
  line.i32(w.soft_l);
  line.ch(',');
  line.i32(w.soft_r);
  line.str(w.pol_ok ? " pol=ok" : " pol=no");
  line.str(w.mdir < 0 ? " mdir=-1" : " mdir=1");
  const bool caldirty = fw.r_valid ? !cal_equal(w, fw.r_cal) : cal_any_set(w);
  uint16_t defaults[kParamCount];
  params_defaults(defaults);
  const bool cfgdirty = !params_equal(fw.w_par, fw.r_valid ? fw.r_par : defaults);
  line.str(caldirty ? " caldirty=1" : " caldirty=0");
  line.str(cfgdirty ? " cfgdirty=1" : " cfgdirty=0");
  line.finish();
  push_line(line);
}

void emit_status_line() {
  OutLine line;
  line.clear();
  const bool servo = steer_servo_mode();
  int32_t tgt = servo ? fw.tgt : fw.pos;
  int32_t cmd = servo ? fw.cmd : fw.pos;
  if (tgt > 9999) tgt = 9999;
  if (tgt < -9999) tgt = -9999;
  if (cmd > 9999) cmd = 9999;
  if (cmd < -9999) cmd = -9999;
  uint32_t wd = fw.now - fw.last_input;
  if (wd > 9999u) wd = 9999u;
  uint32_t tl = 0;
  if (fw.state == kStArmed && fw.c_valid) tl = static_cast<uint32_t>(fw.c_ttl) - (fw.now - fw.c_ms);
  line.str("S ");
  line.u32(fw.now);
  line.ch(' ');
  line.u32(fw.status_n);
  line.ch(' ');
  line.u32(fw.state);
  line.ch(' ');
  line.u32(fw.last_q);
  line.ch(' ');
  line.i32(fw.side[0].d);
  line.ch(' ');
  line.i32(fw.side[1].d);
  line.ch(' ');
  line.u32(1u + 2u * fw.pin_sen);  // bit0 drive EN (HIGH after setup), bit1 steering EN
  line.ch(' ');
  line.u32(fw.f);
  line.ch(' ');
  line.i32(fw.pos);
  line.ch(' ');
  line.i32(tgt);
  line.ch(' ');
  line.i32(cmd);
  line.ch(' ');
  line.i32(static_cast<int32_t>(fw.pin_sa) - static_cast<int32_t>(fw.pin_sb));
  line.ch(' ');
  line.hex(fault_mask(), 4);
  line.ch(' ');
  line.u32(wd);
  line.ch(' ');
  line.u32(tl);
  line.finish();
  push_line(line);
  fw.status_n = static_cast<uint16_t>(fw.status_n + 1u);  // also for a dropped line
}

void emit_fault_line(uint16_t bit, int32_t arg) {
  int index = 0;
  while (index < 15 && bit != (1u << index)) ++index;
  OutLine line;
  line.clear();
  line.str("F ");
  line.str(kFaultName[index]);
  line.ch(' ');
  line.i32(arg);
  line.ch(' ');
  line.u32(fw.now);
  line.finish();
  push_line(line);
}

void emit_watchdog_line() {
  OutLine line;
  line.clear();
  line.str("W ");
  line.u32(fw.now);
  line.finish();
  push_line(line);
}

void emit_pol_result(bool ok, int32_t d1, int32_t d2) {
  OutLine line;
  line.clear();
  line.str(ok ? "A POL ok " : "A POL fail ");
  line.i32(d1);
  line.ch(' ');
  line.i32(d2);
  line.finish();
  push_line(line);
}

}  // namespace ubts_fw
