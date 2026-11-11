// Native unit test of the steering firmware V3 modules (CRC, text grammar, EEPROM record, raw -> permille,
// parameter table, drive ramp, servo law) and of the firmware state machine on the Arduino-style shim.
// Expected values come from docs/steering/ESP_PROTOCOL_V3_DRAFT_KO.md (computed by hand or listed there).
// Built by build.ensure_unit_test_built(); exit 0 = every check passed. Prints one line per failed check.
#include <stdio.h>
#include <string.h>

#include <string>
#include <vector>

#include "arduino_shim/sim_mcu.h"
#include "fw_cal.h"
#include "fw_config.h"
#include "fw_crc.h"
#include "fw_drive.h"
#include "fw_main.h"
#include "fw_monitor.h"
#include "fw_params.h"
#include "fw_state.h"
#include "fw_text.h"

namespace {

int g_checks = 0;
int g_failures = 0;

void check(bool ok, const char *what, int line) {
  ++g_checks;
  if (!ok) {
    ++g_failures;
    printf("FAIL line %d: %s\n", line, what);
  }
}

#define CHECK(cond) check((cond), #cond, __LINE__)

std::string crc_hex(const std::string &s) {
  char buf[4];
  snprintf(buf, sizeof buf, "%02X",
           static_cast<unsigned>(ubts_fw::crc8(reinterpret_cast<const uint8_t *>(s.data()), s.size())));
  return buf;
}

std::string with_crc(const std::string &body) { return body + "*" + crc_hex(body); }

std::string hex_of(const uint8_t *p, size_t n) {
  std::string out;
  char buf[3];
  for (size_t i = 0; i < n; ++i) {
    snprintf(buf, sizeof buf, "%02X", static_cast<unsigned>(p[i]));
    out += buf;
  }
  return out;
}

// --- pure functions --------------------------------------------------------------------------------------------
void test_crc() {
  struct Vector {
    const char *text;
    unsigned crc;
  };
  const Vector vectors[] = {{"", 0x00},       {"123456789", 0xF4}, {"A", 0xC0},          {"STOP", 0x66},
                            {"PING", 0x1F},   {"INFO", 0x58},      {"DISARM", 0x88},     {"ARM 5F3A19C2", 0xCC},
                            {"ARM 5f3a19c2", 0xA7}, {"BENCH 5F3A19C2", 0x8A}, {"C 1842 250 102 102 -310", 0x8E},
                            {"C 1 300 0 0 0", 0xA7}, {"C 2 300 128 128 -250", 0xE2}, {"CAL MARK L", 0xDD},
                            {"CAL SAVE", 0x0F}, {"CFG kp 245", 0xC9},
                            {"S 123456 2210 2 1842 102 102 3 611 -310 -323 -318 -64 0000 40 260", 0xCB},
                            {"N 1842 crc", 0x24}, {"N - long", 0x3C}, {"F POT_OPEN 3 123456", 0xC7},
                            {"W 123456", 0xCE}, {"P D8 0 0", 0xC6}};
  for (size_t i = 0; i < sizeof vectors / sizeof vectors[0]; ++i) {
    const uint8_t got = ubts_fw::crc8(reinterpret_cast<const uint8_t *>(vectors[i].text), strlen(vectors[i].text));
    check(got == vectors[i].crc, vectors[i].text, __LINE__);
  }
  CHECK(ubts_fw::crc32(reinterpret_cast<const uint8_t *>("123456789"), 9) == 0xCBF43926u);
}

void test_text() {
  using namespace ubts_fw;
  uint32_t u = 0;
  int32_t s = 0;
  CHECK(parse_uint("0", 1, &u) && u == 0);
  CHECK(parse_uint("255", 3, &u) && u == 255);
  CHECK(parse_uint("999999999", 9, &u) && u == 999999999u);
  CHECK(!parse_uint("9999999999", 10, &u));
  CHECK(!parse_uint("007", 3, &u));
  CHECK(!parse_uint("+1", 2, &u));
  CHECK(!parse_uint("-1", 2, &u));
  CHECK(!parse_uint("1.5", 3, &u));
  CHECK(!parse_uint("1junk", 5, &u));
  CHECK(!parse_uint("", 0, &u));
  CHECK(parse_int("-310", 4, &s) && s == -310);
  CHECK(parse_int("0", 1, &s) && s == 0);
  CHECK(!parse_int("-0", 2, &s));
  CHECK(!parse_int("-", 1, &s));
  CHECK(!parse_int("-007", 4, &s));
  CHECK(parse_int("-999999999", 10, &s) && s == -999999999);
  CHECK(parse_hex("5f3a19C2", 8, 8, &u) && u == 0x5F3A19C2u);
  CHECK(!parse_hex("5F3A19C", 7, 8, &u));
  CHECK(!parse_hex("5F3A19CG", 8, 8, &u));
  OutLine line;
  line.clear();
  line.str("S ");
  line.u32(123456);
  line.ch(' ');
  line.i32(-310);
  line.ch(' ');
  line.hex(0x7FFF, 4);
  line.finish();
  const std::string text(line.buf, line.len);
  CHECK(text == with_crc("S 123456 -310 7FFF") + "\n");
}

void test_record() {
  using namespace ubts_fw;
  Cal c;
  c.raw_l = 842;
  c.raw_c = 512;
  c.raw_r = 181;
  c.soft_l = 935;
  c.soft_r = -935;
  c.pol_ok = 1;
  c.mdir = 1;
  uint16_t params[kParamCount];
  params_defaults(params);
  uint8_t rec[kRecordBytes];
  record_encode(rec, false, 7, c, params);
  CHECK(hex_of(rec, kRecordBytes) ==
        "55534333016A07004A030002B500A70359FC0101F50014006600FF00C8001400C003E0012C01F40100000000D007D007D007000014"
        "0005001400EB033C00C3030800020032003200280064009900F40114007800E80350002C01CC001E00881305002C01F401A4B1AE61");
  CHECK(record_crc(rec) == 0x61AEB1A4u);
  uint16_t id = 0;
  Cal back;
  uint16_t pback[kParamCount];
  uint32_t crc = 0;
  CHECK(record_decode(rec, &id, &back, pback, &crc) && id == 7 && cal_equal(back, c) && crc == 0x61AEB1A4u);
  CHECK(params_equal(pback, params));
  uint8_t tomb[kRecordBytes];
  record_encode(tomb, true, 7, c, params);
  CHECK(record_crc(tomb) == 0x4F1B5580u);
  CHECK(!record_decode(tomb, &id, &back, pback, &crc));
  CHECK(record_tombstone_id(tomb, &id) && id == 7);
  rec[50] ^= 1;  // a flipped bit: CRC mismatch
  CHECK(!record_decode(rec, &id, &back, pback, &crc));
  record_encode(rec, false, 7, c, params);
  rec[8] = 50;  // left mark 50 < cwmin 60 (CRC recomputed below)
  rec[9] = 0;
  const uint32_t fixed = record_crc(rec);
  for (int i = 0; i < 4; ++i) rec[kRecordCrcOffset + i] = static_cast<uint8_t>((fixed >> (8 * i)) & 0xFFu);
  CHECK(!record_decode(rec, &id, &back, pback, &crc));
}

void test_permille() {
  using namespace ubts_fw;
  Cal c;
  cal_clear(&c);
  CHECK(raw_to_permille(c, 611) == 0);  // no marks
  c.raw_l = 842;
  c.raw_c = 512;
  c.raw_r = 181;
  CHECK(raw_to_permille(c, 611) == 300);
  CHECK(raw_to_permille(c, 400) == -338);
  CHECK(raw_to_permille(c, 842) == 1000);
  CHECK(raw_to_permille(c, 512) == 0);
  CHECK(raw_to_permille(c, 181) == -1000);
  CHECK(raw_to_permille(c, 1023) == 1548);  // (2000 * 511 + 330) / 660
  Cal r = c;  // reversed marks: raw grows to the right
  r.raw_l = 181;
  r.raw_r = 842;
  CHECK(raw_to_permille(r, 181) == 1000 && raw_to_permille(r, 842) == -1000);
  c.raw_c = 800;  // left span 42 < 50: invalid
  CHECK(!marks_valid(c) && raw_to_permille(c, 611) == 0);
}

void test_params() {
  using namespace ubts_fw;
  uint16_t p[kParamCount];
  params_defaults(p);
  CHECK(params_all_ok(p));
  CHECK(param_find("kp", 2) == kPKp && param_find("svhz", 4) == kPSvhz && param_find("KP", 2) < 0);
  CHECK(kParams[kPRmov].def == 960 && kParams[kPTha].def == 5000 && kParams[kPMovms].def == 300);
  CHECK(param_value_ok(kPSthz, 25) && !param_value_ok(kPSthz, 30));
  CHECK(param_value_ok(kPMed, 9) && !param_value_ok(kPMed, 4));
  CHECK(param_value_ok(kPSvhz, 1000) && !param_value_ok(kPSvhz, 750));
  CHECK(!param_value_ok(kPKp, 0) && param_value_ok(kPKp, 4096) && !param_value_ok(kPKp, 4097));
  p[kPMinp] = 200;
  p[kPMaxp] = 100;
  CHECK(!params_cross_ok(p));
  params_defaults(p);
  p[kPTha] = 31000;
  CHECK(!params_cross_ok(p));
}

void test_drive_ramp() {
  using namespace ubts_fw;
  state_reset();
  params_defaults(fw.w_par);
  drive_boot(0);
  int at[700];
  for (uint32_t now = 0; now < 700; ++now) {
    if (now == 20) drive_set_targets(100, 0);
    drive_ramp(now);
    at[now] = fw.side[0].d;
  }
  CHECK(at[119] == 0 && at[120] == 2 && at[130] == 5 && at[140] == 7 && at[150] == 10);
  CHECK(at[300] == 48 && at[309] == 48 && at[310] == 51 && at[509] < 100 && at[510] == 100);
  // release: 255/s down, then the zero dwell holds a new start for 120 ms
  state_reset();
  params_defaults(fw.w_par);
  drive_boot(0);
  for (uint32_t now = 0; now <= 1200; ++now) {
    if (now == 0) drive_set_targets(255, 0);
    drive_ramp(now);
  }
  CHECK(fw.side[0].d == 255);
  drive_set_targets(0, 0);
  uint32_t zero_ms = 0;
  for (uint32_t now = 1201; now <= 2400 && zero_ms == 0; ++now) {
    drive_ramp(now);
    if (fw.side[0].d == 0) zero_ms = now;
  }
  CHECK(zero_ms >= 2190 && zero_ms <= 2210);
  drive_set_targets(-60, 0);
  int first_reverse = 0;
  uint32_t first_ms = 0;
  for (uint32_t now = zero_ms + 1; now <= zero_ms + 400 && first_ms == 0; ++now) {
    drive_ramp(now);
    if (fw.side[0].d != 0) {
      first_reverse = fw.side[0].d;
      first_ms = now;
    }
  }
  CHECK(first_ms >= zero_ms + 120 && first_reverse < 0 && first_reverse >= -3);
  // brake: 0 in the same call, target 0
  drive_set_targets(100, 100);
  for (uint32_t now = first_ms + 1; now < first_ms + 600; ++now) drive_ramp(now);
  drive_brake(first_ms + 600);
  CHECK(fw.side[0].d == 0 && fw.side[0].g == 0 && fw.side[1].d == 0 && fw.side[1].g == 0);
}

// --- the firmware on the shim -----------------------------------------------------------------------------------
struct Bench {
  uint32_t tick;
  uint16_t adc;
  std::string out;

  Bench() : tick(0), adc(512) {}

  void boot(const uint8_t *eeprom, uint64_t seed) {
    ubts_sim::g_mcu.configure(115200, 0x5A1B2C, seed, eeprom);
    ubts_sim::g_mcu.begin_tick(0, 0, adc);
    setup();
    loop();
    tick = 1;
    collect();
  }
  void send(const std::string &text) {
    for (size_t i = 0; i < text.size(); ++i) ubts_sim::g_mcu.rx_push(static_cast<uint8_t>(text[i]));
  }
  void run(uint32_t ticks) {
    for (uint32_t i = 0; i < ticks; ++i, ++tick) {
      ubts_sim::g_mcu.begin_tick(tick, tick, adc);
      loop();
    }
    collect();
  }
  void collect() {
    std::vector<ubts_sim::TxByte> bytes;
    ubts_sim::g_mcu.take_tx_done_before(0xFFFFFFFFu, &bytes);
    for (size_t i = 0; i < bytes.size(); ++i) out += static_cast<char>(bytes[i].byte);
  }
  std::vector<std::string> lines() const {
    std::vector<std::string> v;
    size_t start = 0;
    for (size_t i = 0; i < out.size(); ++i) {
      if (out[i] == '\n') {
        v.push_back(out.substr(start, i - start));
        start = i + 1;
      }
    }
    return v;
  }
  // Non-status lines since `from` (index into lines()).
  std::vector<std::string> events(size_t from) const {
    std::vector<std::string> all = lines(), v;
    for (size_t i = from; i < all.size(); ++i) {
      if (all[i].compare(0, 2, "S ") != 0) v.push_back(all[i]);
    }
    return v;
  }
};

uint8_t g_blank[256];
uint8_t g_record[256];

void make_images() {
  memset(g_blank, 0xFF, sizeof g_blank);
  memset(g_record, 0xFF, sizeof g_record);
  ubts_fw::Cal c;
  c.raw_l = 842;
  c.raw_c = 512;
  c.raw_r = 181;
  c.soft_l = 935;
  c.soft_r = -935;
  c.pol_ok = 1;
  c.mdir = 1;
  uint16_t params[ubts_fw::kParamCount];
  ubts_fw::params_defaults(params);
  ubts_fw::record_encode(g_record, false, 7, c, params);
}

void test_boot_and_parser() {
  Bench b;
  b.boot(g_blank, 1);
  std::vector<std::string> l = b.lines();
  CHECK(l.size() == 2);
  CHECK(l.size() == 2 && l[0] == with_crc("UBTS_STEER_V3 hello proto=3 build=" UBTS_FW_STR(UBTS_STEER_FW_SRC)
                                          " dev=5A1B2C boot=89025CC1 rst=power_on hw=front_steer_v0 "
                                          "out=drive2,steer1 cal=none sim=1 wd=500 ttl=300 st=20 drop=0"));
  CHECK(l.size() == 2 && l[1] == "UBTS_STEER_V3 cal id=none crc=00000000 left=- center=- right=- soft=0,0 pol=no "
                                  "mdir=1 caldirty=0 cfgdirty=0*3F");
  CHECK(ubts_fw::debug_state() == ubts_fw::kStDisarmed);
  CHECK(ubts_fw::debug_fault_bits() == (ubts_fw::kFBoot | ubts_fw::kFNocal));
  size_t mark = b.lines().size();
  b.send(with_crc("ARM 89025CC1") + "\n");
  b.send("PING\n");
  b.send("ping\n");
  b.send("D8 0\n");
  b.send("C 1 300 0 0 0\n");
  b.send(with_crc("ARM 12345678") + "\n");
  b.send("INFO junk\n");
  b.send("\n\r\n");
  b.run(5);  // at most 64 received bytes per tick
  std::vector<std::string> e = b.events(mark);
  CHECK(e.size() == 6);
  if (e.size() == 6) {
    CHECK(e[0] == with_crc("N - nocal"));
    CHECK(e[1] == with_crc("N - cmd"));
    CHECK(e[2] == with_crc("N - state"));
    CHECK(e[3] == with_crc("N 1 nocrc"));
    CHECK(e[4] == with_crc("N - boot"));
    CHECK(e[5] == with_crc("N - syntax"));
  }
  // framing: overlong, control byte, CR rule, CRC; STOP special
  mark = b.lines().size();
  b.send(std::string(80, 'X') + "\n");
  b.send(std::string("PING\0junk\n", 10));
  b.send("D8 2\r55\n");
  b.send("PING*00\n");
  b.send("STOP*ZZ\n");
  b.run(5);  // at most 64 received bytes per tick
  e = b.events(mark);
  CHECK(e.size() == 4);
  if (e.size() == 4) {
    CHECK(e[0] == with_crc("N - long"));
    CHECK(e[1] == with_crc("N - ctrl"));
    CHECK(e[2] == with_crc("N - ctrl"));
    CHECK(e[3] == with_crc("N - crc"));
  }
  // one more bad line reaches crcn = 5: CRC fault in DISARMED (latch, F CRC, no state change)
  mark = b.lines().size();
  b.send("PING*01\n");
  b.run(5);  // at most 64 received bytes per tick
  e = b.events(mark);
  CHECK(e.size() == 2 && e[0] == with_crc("N - crc") && e[1].compare(0, 8, "F CRC 5 ") == 0);
  CHECK((ubts_fw::debug_fault_bits() & ubts_fw::kFCrc) != 0);
  // status line cadence: b + 50 k
  const std::vector<std::string> all = b.lines();
  bool saw50 = false;
  for (size_t i = 0; i < all.size(); ++i) saw50 = saw50 || all[i].compare(0, 7, "S 50 0 ") == 0;
  b.run(60);
  const std::vector<std::string> later = b.lines();
  for (size_t i = 0; i < later.size(); ++i) saw50 = saw50 || later[i].compare(0, 7, "S 50 0 ") == 0;
  CHECK(saw50);
}

void test_armed_ttl_and_watchdog() {
  Bench b;
  b.boot(g_record, 1);
  b.run(9);  // ticks 1..9
  b.send(with_crc("ARM 89025CC1") + "\n");
  b.run(10);  // tick 10..19
  CHECK(ubts_fw::debug_state() == ubts_fw::kStArmed);
  CHECK(ubts_sim::g_mcu.pin_level(4) == 1);  // steering EN (BRAKE)
  b.send(with_crc("C 1 300 100 100 0") + "\n");
  b.run(300);  // ticks 20..319
  CHECK(ubts_sim::g_mcu.pin_value(15) == 51 && ubts_sim::g_mcu.pin_value(0) == 51);
  size_t mark = b.lines().size();
  b.run(1);  // tick 320: TTL expiry, brake
  CHECK(ubts_sim::g_mcu.pin_value(15) == 0 && ubts_sim::g_mcu.pin_value(0) == 0);
  std::vector<std::string> e = b.events(mark);
  CHECK(e.size() == 1 && e[0] == with_crc("F TTL 1 320"));
  CHECK(ubts_fw::debug_state() == ubts_fw::kStArmed);
  CHECK(ubts_fw::debug_fault_bits() == ubts_fw::kFTtl);
  // the last input was the C at tick 20; the watchdog acts at 20 + 500 + 1
  mark = b.lines().size();
  b.run(200);  // ticks 321..520
  CHECK(ubts_fw::debug_state() == ubts_fw::kStArmed);
  b.run(1);  // tick 521
  e = b.events(mark);
  CHECK(e.size() == 1 && e[0] == with_crc("W 521"));
  CHECK(ubts_fw::debug_state() == ubts_fw::kStDisarmed);
  CHECK((ubts_fw::debug_fault_bits() & ubts_fw::kFWatchdog) != 0);
  CHECK(ubts_sim::g_mcu.pin_level(4) == 1);  // HOLD (measured position) for hloss
  b.run(1999);                               // ticks 522..2520: HOLD until t0 + 2000 = 2521
  CHECK(ubts_sim::g_mcu.pin_level(4) == 1);
  b.run(1);  // tick 2521: OFF
  CHECK(ubts_sim::g_mcu.pin_level(4) == 0);
}

void test_bench_pins() {
  Bench b;
  b.boot(g_record, 42);
  b.run(199);  // ticks 1..199
  size_t mark = b.lines().size();
  b.send(with_crc("BENCH 2FEB6E95") + "\n");
  b.send("D8 128\n");
  b.send("D1 0\n");
  b.send("D6 10\n");
  b.send("D8 300\n");
  b.run(1);  // tick 200
  std::vector<std::string> e = b.events(mark);
  CHECK(e.size() == 4);
  if (e.size() == 4) {
    CHECK(e[0] == with_crc("P D8 0 128"));
    CHECK(e[1] == with_crc("P D1 255 255"));
    CHECK(e[2] == with_crc("N - pin"));
    CHECK(e[3] == with_crc("N - range"));
  }
  CHECK(ubts_fw::debug_state() == ubts_fw::kStBench);
  // the ramp grid is bound to the boot tick (10 ms) and the zero dwell from boot is over: tick 200 is a grid point
  CHECK(ubts_sim::g_mcu.pin_value(15) == 2);
  b.run(10);  // ticks 201..210
  CHECK(ubts_sim::g_mcu.pin_value(15) == 5);
  // repeated zero: one reply each, no GPIO write (inside the 500 ms BENCH watchdog)
  b.send("D8 0\n");
  b.run(100);
  const uint32_t writes = ubts_sim::g_mcu.gpio_writes();
  mark = b.lines().size();
  b.send("D8 0\nD8 0\nD8 0\n");
  b.run(1);
  e = b.events(mark);
  CHECK(e.size() == 3 && e[0] == with_crc("P D8 0 0") && e[1] == e[0] && e[2] == e[0]);
  CHECK(ubts_sim::g_mcu.gpio_writes() == writes);
}

const char kCal7Line[] =
    "UBTS_STEER_V3 cal id=7 crc=61AEB1A4 left=842 center=512 right=181 soft=935,-935 pol=ok mdir=1 caldirty=0 "
    "cfgdirty=0*D6";  // spec 5.9 (record of spec 9.6)

// Decision 12.1-66: with the ESP8266 core 3.1.2 UART0 TX rules (128 B FIFO, Serial.write waits while the fill is
// 127 or more) the firmware never makes Serial.write wait, and a STOP reaches the drive pins at the start of its
// tick, before any serial output of that tick.
void test_tx_fifo_model() {
  ubts_sim::g_mcu.set_tx_fifo_model(true);
  Bench b;
  b.boot(g_record, 1);  // setup(): banner and cal line (279 B) into the TX ring
  CHECK(ubts_sim::g_mcu.tx_block_events() == 0);
  b.run(9);  // ticks 1..9
  b.send(with_crc("ARM 89025CC1") + "\n");
  b.run(10);  // ticks 10..19
  b.send(with_crc("C 1 300 100 100 0") + "\n");
  b.run(250);  // ticks 20..269: the drive ramps up from tick 120
  CHECK(ubts_sim::g_mcu.pin_value(15) > 0 && ubts_sim::g_mcu.pin_value(0) > 0);
  const size_t mark = b.lines().size();
  std::string burst = "INFO\n";                 // 279 B of reply
  for (int i = 0; i < 11; ++i) burst += "x\n";  // 11 x "N - cmd*HH" = 121 B
  burst += "STOP\n";                            // 32 B in all: one tick reads them
  b.send(burst);
  const uint32_t t = b.tick;
  b.run(1);
  CHECK(ubts_sim::g_mcu.pin_value(15) == 0 && ubts_sim::g_mcu.pin_value(0) == 0);  // brake in the STOP tick
  const uint64_t tick_start = static_cast<uint64_t>(t) * ubts_sim::kUnitsPerTick;
  CHECK(ubts_sim::g_mcu.pin_write_units(15) == tick_start);  // written before the serial output of the tick
  CHECK(ubts_sim::g_mcu.pin_write_units(0) == tick_start);
  CHECK(ubts_sim::g_mcu.tx_block_events() == 0);
  b.run(200);  // the ring drains at the line rate (about 11.5 B per ms)
  CHECK(ubts_sim::g_mcu.tx_block_events() == 0 && ubts_sim::g_mcu.tx_block_units() == 0);
  const std::vector<std::string> e = b.events(mark);
  size_t rejects = 0, banners = 0, cal_lines = 0;
  for (size_t i = 0; i < e.size(); ++i) {
    rejects += e[i] == with_crc("N - cmd") ? 1u : 0u;
    banners += e[i].compare(0, 19, "UBTS_STEER_V3 hello") == 0 ? 1u : 0u;
    cal_lines += e[i] == kCal7Line ? 1u : 0u;
  }
  CHECK(rejects == 11 && banners == 1 && cal_lines == 1);
  CHECK(ubts_fw::debug_state() == ubts_fw::kStDisarmed);
  ubts_sim::g_mcu.set_tx_fifo_model(false);
}

// Decisions 12.1-57 and 12.1-73: a failed EEPROM commit (real chip only) keeps the stored record R.
void test_eeprom_commit_failure() {
  Bench b;
  b.boot(g_record, 1);
  b.run(9);
  ubts_sim::g_mcu.set_eeprom_commit_fail(true);
  size_t mark = b.lines().size();
  b.send(with_crc("CAL ERASE") + "\n");
  b.run(2);
  std::vector<std::string> e = b.events(mark);
  CHECK(e.size() == 1 && e[0] == kCal7Line);  // no F NOCAL, R and W kept
  CHECK((ubts_fw::debug_fault_bits() & ubts_fw::kFNocal) == 0);
  mark = b.lines().size();
  b.send(with_crc("CAL SAVE") + "\n");  // would be cal_id 8; the commit fails as well
  b.run(2);
  e = b.events(mark);
  CHECK(e.size() == 1 && e[0] == kCal7Line);
  CHECK(ubts_sim::g_mcu.eeprom_commits() == 0);
  ubts_sim::g_mcu.set_eeprom_commit_fail(false);
  // A reset: the flash still holds record 7, the retried ERASE now works.
  ubts_sim::g_mcu.reset_hardware(b.tick, 4);
  ubts_sim::g_mcu.begin_tick(b.tick, 0, b.adc);
  mark = b.lines().size();
  setup();
  loop();
  ++b.tick;
  b.collect();
  e = b.events(mark);
  CHECK(e.size() == 2 && e[0].find(" cal=7 ") != std::string::npos && e[1] == kCal7Line);
  b.run(5);
  mark = b.lines().size();
  b.send(with_crc("CAL ERASE") + "\n");
  b.run(2);
  e = b.events(mark);
  CHECK(e.size() == 2 && e[0].compare(0, 10, "F NOCAL 0 ") == 0 &&
        e[1].find("cal id=none ") != std::string::npos);
  CHECK(ubts_sim::g_mcu.eeprom_commits() == 1);
}

// Decision 12.1-72: the THERMAL window count does not depend on the u32 wrap of the processed-tick counter
// (2^32 is not a multiple of the 60000-bit ring). 20000 hot ticks, then 70000 cold ones: the count falls back to 0.
void test_thermal_window_across_tick_wrap() {
  using namespace ubts_fw;
  const uint32_t starts[3] = {1000000u, 0xFFFFFFFFu - 20000u + 1u, 0xFFFFFFFFu - 10000u + 1u};
  for (int c = 0; c < 3; ++c) {
    state_reset();
    params_defaults(fw.w_par);
    monitor_boot();
    fw.mode = kModeOff;
    fw.tick = starts[c];
    fw.ticks_seen = starts[c];  // as if the MCU had run that long
    uint32_t peak = 0;
    for (uint32_t k = 0; k < 90000u; ++k) {
      fw.pin_sa = k < 20000u ? 255 : 0;
      fw.pin_sb = 0;
      monitor_end_of_tick();
      ++fw.tick;  // u32, wraps in cases 1 and 2
      if (fw.thermal_cnt > peak) peak = fw.thermal_cnt;
    }
    check(peak == 20000u, "thermal peak", __LINE__);
    check(fw.thermal_cnt == 0u, "thermal count back to 0 after 30 s", __LINE__);
  }
}

}  // namespace

int main() {
  make_images();
  test_crc();
  test_text();
  test_record();
  test_permille();
  test_params();
  test_drive_ramp();
  test_boot_and_parser();
  test_armed_ttl_and_watchdog();
  test_bench_pins();
  test_tx_fifo_model();
  test_eeprom_commit_failure();
  test_thermal_window_across_tick_wrap();
  printf("%s %d checks, %d failed\n", g_failures ? "FAIL" : "PASS", g_checks, g_failures);
  return g_failures ? 1 : 0;
}
