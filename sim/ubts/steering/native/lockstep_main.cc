// Lockstep driver of the native steering process (docs/steering/LOCKSTEP_PROTOCOL_KO.md, v1 and v2).
//
// stdin : one ASCII JSON startup line, then binary request frames.
//         v1 / v2 direct_pwm, test_servo: STEP (0x0001) and RESET (0x0002).
//         v2 firmware: FW_STEP (0x0003) and RESET (0x0002); the MCU firmware runs in this process (section 11).
// stdout: READY, then exactly one response frame per request, flushed per frame. The header version is the
//         startup protocol version (1 or 2).
// stderr: on error one line "steer_native: <name>: <detail>" and the documented exit code.
// Exit codes: 0 ok (EOF at a frame boundary or before the startup line), 2 bad_startup,
// 3 bad_header, 4 sequence_mismatch, 5 bad_length, 6 bad_value, 7 truncated_frame,
// 8 io_error, 9 internal_error.
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <cmath>
#include <exception>
#include <new>
#include <string>
#include <vector>

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

#include "fw_session.h"
#include "steer_config.h"
#include "steer_model.h"
#include "steer_wire.h"

#ifndef UBTS_STEER_SRC_ID
#define UBTS_STEER_SRC_ID unknown
#endif
#ifndef UBTS_STEER_FW_SRC
#define UBTS_STEER_FW_SRC 00000000
#endif
#ifndef UBTS_STEER_BAUD
#define UBTS_STEER_BAUD 115200
#endif
#define UBTS_STEER_STR2(x) #x
#define UBTS_STEER_STR(x) UBTS_STEER_STR2(x)

namespace {

using namespace ubts_steer;

const int kExitOk = 0;
const int kExitBadStartup = 2;
const int kExitBadHeader = 3;
const int kExitSequence = 4;
const int kExitBadLength = 5;
const int kExitBadValue = 6;
const int kExitTruncated = 7;
const int kExitIo = 8;
const int kExitInternal = 9;
const double kHalfPi = 1.57079632679489661923;

const char *exit_name(int code) {
  switch (code) {
    case kExitOk: return "ok";
    case kExitBadStartup: return "bad_startup";
    case kExitBadHeader: return "bad_header";
    case kExitSequence: return "sequence_mismatch";
    case kExitBadLength: return "bad_length";
    case kExitBadValue: return "bad_value";
    case kExitTruncated: return "truncated_frame";
    case kExitIo: return "io_error";
    default: return "internal_error";
  }
}

struct Exit {
  Exit(int c, const std::string &d) : code(c), detail(d) {}
  int code;
  std::string detail;
};

bool is_finite(double x) { return x == x && x <= 1.7976931348623157e308 && x >= -1.7976931348623157e308; }

std::string fmt(const char *format, double a) {
  char buf[160];
  snprintf(buf, sizeof buf, format, a);
  return buf;
}

std::string fmt_u(const char *format, unsigned long a, unsigned long b = 0) {
  char buf[160];
  snprintf(buf, sizeof buf, format, a, b);
  return buf;
}

std::string clean_id(const std::string &id) {
  std::string clean;
  for (size_t i = 0; i < id.size() && clean.size() < kMaxBuildIdBytes; ++i) {
    const unsigned char c = static_cast<unsigned char>(id[i]);
    clean.push_back(c >= 0x20 && c < 0x7f ? static_cast<char>(c) : '_');
  }
  return clean;
}

std::string compiler_and_bits() {
  std::string id;
#if defined(__clang__)
  id += " cxx=clang-" __clang_version__;
#elif defined(__GNUC__)
  id += " cxx=gcc-" __VERSION__;
#else
  id += " cxx=unknown";
#endif
  char bits[32];
  snprintf(bits, sizeof bits, " bits=%d", static_cast<int>(sizeof(void *) * 8));
  id += bits;
  return id;
}

// READY build id: v1 "ubts-steer-native/1 src=... cxx=... bits=..", v2 adds the firmware build (section 11.8).
std::string build_id(int protocol) {
  std::string id = protocol == 2 ? "ubts-steer-native/2 src=" UBTS_STEER_STR(UBTS_STEER_SRC_ID)
                                   " fw=" UBTS_STEER_STR(UBTS_STEER_FW_SRC)
                                 : "ubts-steer-native/1 src=" UBTS_STEER_STR(UBTS_STEER_SRC_ID);
  id += compiler_and_bits();
  return clean_id(id);
}

// Reads exactly n bytes; returns the count read before EOF (n on success).
size_t read_full(unsigned char *buf, size_t n) {
  size_t have = 0;
  while (have < n) {
    const size_t got = fread(buf + have, 1, n - have, stdin);
    if (got == 0) {
      if (ferror(stdin)) throw Exit(kExitIo, "stdin read failed");
      break;
    }
    have += got;
  }
  return have;
}

void write_all(const unsigned char *buf, size_t n) {
  if (fwrite(buf, 1, n, stdout) != n || fflush(stdout) != 0) throw Exit(kExitIo, "stdout write failed");
}

void put_header(unsigned char *p, uint16_t version, uint16_t type, uint32_t seq, uint32_t length) {
  memcpy(p, "USTP", 4);
  put_u16(p + 4, version);
  put_u16(p + 6, type);
  put_u32(p + 8, seq);
  put_u32(p + 12, length);
}

// The 80-byte v1 response block (section 4.4).
void put_block(unsigned char *p, const StepOutput &o, double drive_left, double drive_right, bool drive_en) {
  put_f64(p + 0, o.sim_time_s);
  put_f64(p + 8, o.delta_left_rad);
  put_f64(p + 16, o.delta_right_rad);
  put_f64(p + 24, o.delta_eq_rad);
  put_f64(p + 32, o.motor_angle_rad);
  put_f64(p + 40, o.applied_pwm);
  put_f64(p + 48, drive_left);
  put_f64(p + 56, drive_right);
  put_u32(p + 64, o.ticks);
  put_u16(p + 68, o.pot_raw);
  put_u16(p + 70, o.fault_bits);
  put_u16(p + 72, o.model_flags);
  p[74] = o.steer_en ? 1 : 0;
  p[75] = drive_en ? 1 : 0;
  put_u32(p + 76, 0);  // serial_out_len
}

void write_response(uint16_t version, uint16_t type, uint32_t seq, const StepOutput &o) {
  unsigned char buf[kHeaderBytes + kResponseBytes];
  put_header(buf, version, type, seq, kResponseBytes);
  put_block(buf + kHeaderBytes, o, 0.0, 0.0, false);  // drive fields are firmware mode only
  write_all(buf, sizeof buf);
}

void write_fw_reset_response(uint32_t seq, const FwStepOutput &o) {
  unsigned char buf[kHeaderBytes + kResponseBytes];
  put_header(buf, kWireVersionV2, kFrameResetResponse, seq, kResponseBytes);
  put_block(buf + kHeaderBytes, o.base, o.drive_left, o.drive_right, o.drive_en);
  write_all(buf, sizeof buf);
}

void write_fw_step_response(uint32_t seq, const FwStepOutput &o) {
  if (o.changes.size() > kMaxFwChanges || o.tx.size() > kMaxFwTx) {
    throw Exit(kExitInternal, "firmware step produced more than 4096 changes or tx bytes");
  }
  const size_t length = kFwStepResponseBytes + kFwChangeBytes * o.changes.size() + kFwTxBytes * o.tx.size();
  std::vector<unsigned char> buf(kHeaderBytes + length, 0);
  unsigned char *p = &buf[0];
  put_header(p, kWireVersionV2, kFrameFwStepResponse, seq, static_cast<uint32_t>(length));
  p += kHeaderBytes;
  put_block(p, o.base, o.drive_left, o.drive_right, o.drive_en);
  put_u16(p + 80, static_cast<uint16_t>(o.changes.size()));
  put_u16(p + 82, static_cast<uint16_t>(o.tx.size()));
  p[84] = o.mcu_state;
  p[85] = o.run_flags;
  put_u16(p + 86, o.boot_count);
  put_u32(p + 88, o.mcu_millis);
  put_u32(p + 92, o.rx_lost_total);
  unsigned char *q = p + kFwStepResponseBytes;
  for (size_t i = 0; i < o.changes.size(); ++i, q += kFwChangeBytes) {
    put_u16(q, o.changes[i].offset);
    q[2] = o.changes[i].channel;
    q[3] = 0;
    put_u32(q + 4, static_cast<uint32_t>(o.changes[i].value));
  }
  for (size_t i = 0; i < o.tx.size(); ++i, q += kFwTxBytes) {
    put_u32(q, o.tx[i].queued_tick);
    put_u32(q + 4, o.tx[i].done_tick);
    q[8] = o.tx[i].byte;
    q[9] = o.tx[i].flags;
    put_u16(q + 10, 0);
  }
  write_all(&buf[0], buf.size());
}

void check_output(const SteerModel &model, const StepOutput &o) {
  if (!model.finite_state() || !is_finite(o.delta_left_rad) || !is_finite(o.delta_right_rad) ||
      !is_finite(o.delta_eq_rad) || !is_finite(o.applied_pwm) || std::fabs(o.applied_pwm) > 1.0) {
    throw Exit(kExitInternal, "model state is not finite or out of range");
  }
}

// Startup line: bytes up to '\n', at most 64 KiB including the '\n'. Returns false on EOF
// before the first byte.
bool read_startup_line(std::string *line) {
  bool any = false;
  for (;;) {
    const int c = getc(stdin);
    if (c == EOF) {
      if (ferror(stdin)) throw Exit(kExitIo, "stdin read failed");
      if (!any) return false;
      throw Exit(kExitBadStartup, "EOF inside the startup line");
    }
    any = true;
    if (c == '\n') return true;
    if (line->size() + 2 > kMaxStartupLineBytes) throw Exit(kExitBadStartup, "startup line longer than 64 KiB");
    line->push_back(static_cast<char>(c));
  }
}

void step_request(const Config &cfg, SteerModel *model, const unsigned char *payload, uint32_t length, uint32_t seq,
                  uint16_t version) {
  const uint32_t serial_length = get_u32(payload + 44);
  if (serial_length != length - kStepRequestBytes) {
    throw Exit(kExitBadLength, fmt_u("serial_in_len %lu does not match the payload", serial_length));
  }
  const unsigned mode = payload[40], enable = payload[41], stopped = payload[42], reserved = payload[43];
  if (mode > 2 || enable > 1 || stopped > 1 || reserved != 0) {
    throw Exit(kExitBadValue, "mode, steer_en, vehicle_stopped or reserved byte out of range");
  }
  if (static_cast<int>(mode) != cfg.mode) throw Exit(kExitBadValue, "request mode differs from the startup mode");
  StepInput in;
  in.dt_s = get_f64(payload + 0);
  in.steer_pwm = get_f64(payload + 8);
  in.target_rad = get_f64(payload + 16);
  in.speed_m_s = get_f64(payload + 24);
  in.supply_scale = get_f64(payload + 32);
  in.steer_en = enable != 0;
  in.stopped = stopped != 0;
  if (!is_finite(in.dt_s) || !(in.dt_s > 0.0) || in.dt_s > 1.0) throw Exit(kExitBadValue, "dt_s outside (0, 1]");
  if (!is_finite(in.steer_pwm) || std::fabs(in.steer_pwm) > 1.0) throw Exit(kExitBadValue, "steer_pwm outside [-1, 1]");
  if (!is_finite(in.target_rad) || !(std::fabs(in.target_rad) < kHalfPi)) {
    throw Exit(kExitBadValue, "target_delta_eq_rad outside (-pi/2, pi/2)");
  }
  if (!is_finite(in.speed_m_s)) throw Exit(kExitBadValue, "vehicle_speed_m_s is not finite");
  if (!is_finite(in.supply_scale) || !(in.supply_scale > 0.0) || in.supply_scale > 2.0) {
    throw Exit(kExitBadValue, "supply_scale outside (0, 2]");
  }
  if (serial_length != 0) throw Exit(kExitBadValue, "serial_in needs firmware mode");
  if (cfg.mode != kModeDirectPwm && in.steer_pwm != 0.0) throw Exit(kExitBadValue, "steer_pwm only in direct_pwm");
  if (cfg.mode != kModeTestServo && in.target_rad != 0.0) {
    throw Exit(kExitBadValue, "target_delta_eq_rad only in test_servo");
  }
  const uint32_t ticks = model->step(in);
  StepOutput out;
  model->output(ticks, &out);
  check_output(*model, out);
  write_response(version, kFrameStepResponse, seq, out);
}

uint32_t event_arg_max(uint16_t kind, uint32_t *min_arg) {
  switch (kind) {
    case kEvMcuReset: *min_arg = 1; return 6;
    case kEvPowerCycle: *min_arg = 1; return 60000;
    case kEvPortOpenReset: *min_arg = 0; return 0;
    case kEvMotorPower: *min_arg = 0; return 1;
    case kEvMcuStall: *min_arg = 1; return 60000;
    case kEvAdcSet: *min_arg = 0; return 1023;
    case kEvAdcModel: *min_arg = 0; return 0;
    default: *min_arg = 1; return 0;  // unknown kind: no argument is valid
  }
}

void fw_step_request(FirmwareSession *session, SteerModel *model, const unsigned char *payload, uint32_t length,
                     uint32_t seq) {
  const uint32_t events = get_u16(payload + 26);
  const uint32_t rx_count = get_u32(payload + 28);
  if (events > kMaxFwEvents || rx_count > kMaxFwRx) {
    throw Exit(kExitBadLength, fmt_u("event_count %lu or rx_count %lu above the limits (64, 4096)", events, rx_count));
  }
  if (length != kFwStepRequestBytes + kFwEventBytes * events + kFwRxBytes * rx_count) {
    throw Exit(kExitBadLength, fmt_u("payload length %lu does not match the counts", length));
  }
  FwStepInput in;
  in.dt_s = get_f64(payload + 0);
  in.speed_m_s = get_f64(payload + 8);
  in.supply_scale = get_f64(payload + 16);
  const unsigned stopped = payload[24], reserved = payload[25];
  if (!is_finite(in.dt_s) || !(in.dt_s > 0.0) || in.dt_s > kMaxFwDtS) throw Exit(kExitBadValue, "dt_s outside (0, 0.1]");
  if (!is_finite(in.speed_m_s)) throw Exit(kExitBadValue, "vehicle_speed_m_s is not finite");
  if (!is_finite(in.supply_scale) || !(in.supply_scale > 0.0) || in.supply_scale > 2.0) {
    throw Exit(kExitBadValue, "supply_scale outside (0, 2]");
  }
  if (stopped > 1 || reserved != 0) throw Exit(kExitBadValue, "vehicle_stopped or reserved byte out of range");
  in.stopped = stopped != 0;
  const int64_t ticks = session->ticks_for(in.dt_s);
  const unsigned char *p = payload + kFwStepRequestBytes;
  uint32_t last = 0;
  for (uint32_t i = 0; i < events; ++i, p += kFwEventBytes) {
    FwEvent e;
    e.offset = get_u16(p);
    e.kind = get_u16(p + 2);
    e.arg = get_u32(p + 4);
    if (static_cast<int64_t>(e.offset) >= ticks || e.offset < last) {
      throw Exit(kExitBadValue, fmt_u("event tick_offset %lu outside the step or decreasing (%lu ticks)", e.offset,
                                      static_cast<unsigned long>(ticks)));
    }
    uint32_t min_arg = 0;
    const uint32_t max_arg = event_arg_max(e.kind, &min_arg);
    if (e.kind < kEvMcuReset || e.kind > kEvAdcModel) throw Exit(kExitBadValue, fmt_u("unknown event kind %lu", e.kind));
    if (e.arg < min_arg || e.arg > max_arg) {
      throw Exit(kExitBadValue, fmt_u("event kind %lu argument %lu out of range", e.kind, e.arg));
    }
    last = e.offset;
    in.events.push_back(e);
  }
  last = 0;
  for (uint32_t i = 0; i < rx_count; ++i, p += kFwRxBytes) {
    FwRx r;
    r.offset = get_u16(p);
    r.byte = p[2];
    if (static_cast<int64_t>(r.offset) >= ticks || r.offset < last || p[3] != 0) {
      throw Exit(kExitBadValue, fmt_u("rx tick_offset %lu outside the step, decreasing or reserved byte set (%lu ticks)",
                                      r.offset, static_cast<unsigned long>(ticks)));
    }
    last = r.offset;
    in.rx.push_back(r);
  }
  FwStepOutput out;
  session->step(in, &out);
  check_output(*model, out.base);
  write_fw_step_response(seq, out);
}

int run() {
  std::string line;
  if (!read_startup_line(&line)) return kExitOk;
  Config cfg;
  try {
    cfg = parse_config(line);
  } catch (const ConfigError &e) {
    throw Exit(kExitBadStartup, e.what());
  }
  const bool firmware = cfg.mode == kModeFirmware;
  const uint16_t version = cfg.protocol == 2 ? kWireVersionV2 : kWireVersion;
  if (firmware && !kFirmwareLinked) {
    throw Exit(kExitBadStartup, "mode firmware: this program was built without the firmware sources "
                                "(firmware/ubts_steer_v3 was missing at build time)");
  }
  if (firmware) {
    if (cfg.mcu_baud != static_cast<uint32_t>(UBTS_STEER_BAUD)) {
      throw Exit(kExitBadStartup, fmt_u("mcu.baud: %lu differs from the compiled firmware baud %lu", cfg.mcu_baud,
                                        static_cast<unsigned long>(UBTS_STEER_BAUD)));
    }
    if (cfg.fw.expect_build_set && cfg.fw.expect_build != UBTS_STEER_STR(UBTS_STEER_FW_SRC)) {
      throw Exit(kExitBadStartup, "firmware.expect_build: " + cfg.fw.expect_build +
                                      " differs from the compiled firmware build " UBTS_STEER_STR(UBTS_STEER_FW_SRC));
    }
  }
  SteerModel model(cfg);
  FirmwareSession session(cfg, &model);
  if (firmware) {
    FwStepOutput ready_state;
    session.reset(cfg.fw.initial_delta_eq_rad, &ready_state);  // READY state = RESET(initial_delta_eq_rad)
  } else {
    model.reset(0.0);  // READY state = RESET(0.0)
  }

  const std::string id = build_id(cfg.protocol);
  {
    std::string frame(kHeaderBytes + kReadyBytes + id.size(), '\0');
    unsigned char *p = reinterpret_cast<unsigned char *>(&frame[0]);
    put_header(p, version, kFrameReady, 0, static_cast<uint32_t>(kReadyBytes + id.size()));
    put_u32(p + kHeaderBytes, static_cast<uint32_t>(cfg.tick_hz));
    put_u32(p + kHeaderBytes + 4, static_cast<uint32_t>(id.size()));
    memcpy(p + kHeaderBytes + kReadyBytes, id.data(), id.size());
    write_all(p, frame.size());
  }

  const uint32_t max_fw_request = kFwStepRequestBytes + kFwEventBytes * kMaxFwEvents + kFwRxBytes * kMaxFwRx;
  std::vector<unsigned char> payload_buf(max_fw_request > kStepRequestBytes + kMaxSerialBytes
                                             ? max_fw_request
                                             : kStepRequestBytes + kMaxSerialBytes);
  unsigned char *payload = &payload_buf[0];
  uint32_t expected_seq = 1;
  for (;;) {
    unsigned char header[kHeaderBytes];
    const size_t got = read_full(header, kHeaderBytes);
    if (got == 0) return kExitOk;  // EOF at a frame boundary
    if (got < kHeaderBytes) {
      throw Exit(kExitTruncated, fmt_u("EOF after %lu of %lu header bytes", static_cast<unsigned long>(got),
                                       static_cast<unsigned long>(kHeaderBytes)));
    }
    if (memcmp(header, "USTP", 4) != 0) throw Exit(kExitBadHeader, "bad magic (expected USTP)");
    if (get_u16(header + 4) != version) {
      throw Exit(kExitBadHeader, fmt_u("version %lu, expected %lu", get_u16(header + 4), version));
    }
    const uint16_t type = get_u16(header + 6);
    const uint32_t seq = get_u32(header + 8);
    const uint32_t length = get_u32(header + 12);
    const bool allowed = firmware ? (type == kFrameFwStepRequest || type == kFrameResetRequest)
                                  : (type == kFrameStepRequest || type == kFrameResetRequest);
    if (!allowed) {
      throw Exit(kExitBadHeader, fmt_u(firmware ? "frame type 0x%04lx is not a firmware mode request"
                                                : "frame type 0x%04lx is not a request",
                                       type));
    }
    if (seq != expected_seq) throw Exit(kExitSequence, fmt_u("seq %lu, expected %lu", seq, expected_seq));
    bool length_ok;
    if (type == kFrameStepRequest) {
      length_ok = length >= kStepRequestBytes && length <= kStepRequestBytes + kMaxSerialBytes;
    } else if (type == kFrameFwStepRequest) {
      length_ok = length >= kFwStepRequestBytes && length <= max_fw_request;
    } else {
      length_ok = length == kResetRequestBytes;
    }
    if (!length_ok) throw Exit(kExitBadLength, fmt_u("payload length %lu for frame type 0x%04lx", length, type));
    const size_t have = read_full(payload, length);
    if (have < length) {
      throw Exit(kExitTruncated, fmt_u("EOF after %lu of %lu payload bytes", static_cast<unsigned long>(have),
                                       static_cast<unsigned long>(length)));
    }

    if (type == kFrameResetRequest) {
      const double delta = get_f64(payload);
      if (!model.reset_angle_valid(delta)) {
        throw Exit(kExitBadValue, fmt("reset angle %.17g rad is outside the equivalent mechanical limits", delta));
      }
      if (firmware) {
        FwStepOutput out;
        session.reset(delta, &out);
        check_output(model, out.base);
        write_fw_reset_response(seq, out);
      } else {
        model.reset(delta);
        StepOutput out;
        model.output(0, &out);
        check_output(model, out);
        write_response(version, kFrameResetResponse, seq, out);
      }
    } else if (type == kFrameFwStepRequest) {
      fw_step_request(&session, &model, payload, length, seq);
    } else {
      step_request(cfg, &model, payload, length, seq, version);
    }
    ++expected_seq;
  }
}

}  // namespace

int main() {
#ifdef _WIN32
  _setmode(_fileno(stdin), _O_BINARY);
  _setmode(_fileno(stdout), _O_BINARY);
#else
  signal(SIGPIPE, SIG_IGN);
#endif
  static char out_buffer[1 << 16];
  setvbuf(stdout, out_buffer, _IOFBF, sizeof out_buffer);
  int code = kExitOk;
  std::string detail;
  try {
    code = run();
  } catch (const Exit &e) {
    code = e.code;
    detail = e.detail;
  } catch (const std::bad_alloc &) {
    code = kExitInternal;
    detail = "out of memory";
  } catch (const std::exception &e) {
    code = kExitInternal;
    detail = e.what();
  }
  if (code != kExitOk) {
    fprintf(stderr, "steer_native: %s: %s\n", exit_name(code), detail.c_str());
    fflush(stderr);
  }
  return code;
}
