// Logic tests of the steering control rules (dashboard/static/steer-control-rules.js), kept from the phase-4 steering
// page draft without the page (DECISIONS R22). Run: node --test dashboard/static/test_steer_control_rules.mjs
// (no browser, no network, no files written).
// The fake bridge below follows the request rules of SteerControl / RoverControl (lease, one-use ticket, seq + 1,
// deadman, left/right refused) and a simple MCU (duty applied while the input is valid, steering moves toward the
// target at a fixed rate). The real bridge contract is checked in sim/ubts/test_steering_ui_routes.py.
import test from 'node:test';
import assert from 'node:assert';
import { readFileSync } from 'node:fs';
import vm from 'node:vm';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
const SOURCE = readFileSync(path.join(here, 'steer-control-rules.js'), 'utf8');
vm.runInThisContext(SOURCE, { filename: 'steer-control-rules.js' });
const S = globalThis.UbtsSteerRules;
const MARK = 438.030759092455;          // host profile mark (mrad), placeholder profile
const DEG = Math.PI / 180;
const mrad = (deg) => deg * DEG * 1000;
const flush = () => new Promise((resolve) => setImmediate(resolve));
const ALL_REQUESTS = [];

class FakeBridge {
  constructor(clock, options = {}) {
    this.clock = clock;
    this.limits = options.limits || { left: 409, right: 409 };
    this.lease = null; this.ticket = null; this.seq = 0; this.n = 0;
    this.throttle = 0; this.steer = null; this.inputUntil = 0; this.releaseUntil = null;
    this.armed = false; this.reason = 'startup'; this.moving = false;
    this.L = 0; this.R = 0; this.pos = options.posMrad || 0; this.tgt = this.pos;
    this.rateDegS = options.rateDegS || 24;
    this.ageMs = 20; this.stale = false; this.mcuMs = 1000; this.statusSeq = 0;
    this.faults = []; this.latches = {};
    this.requests = []; this.stops = []; this.failNext = null; this.holdReplies = false; this.pending = [];
    this.statusOk = true;
  }
  grant() {
    this.n += 1;
    this.ticket = 'T' + this.n;
    return { ok: true, lease: this.lease, ticket: this.ticket, seq: this.seq, ticket_ttl_s: 1.5, status: this.snapshot(false) };
  }
  stopNow(reason) {
    this.lease = null; this.ticket = null; this.armed = false; this.throttle = 0; this.steer = null; this.reason = reason;
    this.moving = false; this.inputUntil = 0;
  }
  owned(body) {
    if (!this.lease || body.lease !== this.lease) return 'control_not_owned';
    if (body.ticket !== this.ticket) { this.stopNow('stale_input'); return 'stale_input_rearm_required'; }
    if (body.seq !== this.seq + 1) { this.stopNow('invalid_sequence'); return 'invalid_sequence'; }
    return null;
  }
  handle(op, body) {
    if (this.failNext) { const error = this.failNext; this.failNext = null; return { ok: false, error }; }
    if (op === 'stop') { this.stops.push(body.reason); this.stopNow('operator_stop'); return { ok: true, state: 'stop_requested' }; }
    if (op === 'arm') {
      if (body.vehicle_profile !== 'front_steer') return { ok: false, error: 'steer_profile_required' };
      if (this.lease) return { ok: false, error: 'control_in_use' };
      this.lease = 'LEASE' + this.n; this.seq = 0; this.armed = true; this.reason = 'operator_armed';
      this.throttle = 0; this.steer = null;
      return this.grant();
    }
    if (op === 'command' && ('left' in body || 'right' in body)) {
      this.stopNow('differential_command_rejected');
      return { ok: false, error: 'differential_command_rejected' };
    }
    const bad = this.owned(body);
    if (bad) return { ok: false, error: bad };
    this.seq = body.seq;
    const now = this.clock.t;
    if (op === 'command') {
      const t = body.throttle, s = body.steer_mrad;
      if (body.deadman !== true || !Number.isInteger(t) || t < -100 || t > 100 || !Number.isInteger(s)
          || s < -this.limits.right || s > this.limits.left) {
        this.stopNow('invalid_drive');
        return { ok: false, error: 'invalid_drive' };
      }
      this.throttle = t; this.steer = s; this.inputUntil = now + 750; this.moving = t !== 0;
    } else if (op === 'release') {
      this.throttle = 0; this.inputUntil = now + 1100; this.moving = true;
    } else if (op !== 'keepalive') {
      return { ok: false, error: 'unknown_operation' };
    }
    return this.grant();
  }
  post(pathname, body) {
    const op = pathname.replace('/api/rover/', '');
    const copy = JSON.parse(JSON.stringify(body));
    this.requests.push({ at: this.clock.t, op, body: copy });
    ALL_REQUESTS.push({ op, body: copy });
    const reply = this.handle(op, copy);
    if (this.holdReplies && op !== 'stop') {
      return new Promise((resolve) => this.pending.push(() => resolve(reply)));
    }
    return Promise.resolve(reply);
  }
  releasePending() { const list = this.pending; this.pending = []; list.forEach((fn) => fn()); }
  advance(dtMs) {
    const now = this.clock.t;
    this.mcuMs += dtMs; this.statusSeq += 1;
    const valid = this.armed && now < this.inputUntil;
    const duty = valid ? Math.sign(this.throttle) * Math.floor((Math.abs(this.throttle) * 255 + 50) / 100) : 0;
    this.L = this.R = duty;
    if (this.moving && duty === 0 && this.throttle === 0) this.moving = false;
    if (this.armed && this.steer !== null) this.tgt = this.steer;
    const rate = (duty !== 0 ? this.rateDegS : this.rateDegS / 2) * DEG * 1000 * dtMs / 1000;
    this.pos += Math.max(-rate, Math.min(rate, this.tgt - this.pos));
  }
  snapshot(withUpdated = true) {
    const permille = (m) => (m >= 0 ? 1 : -1) * Math.floor(Math.abs(1000 * m / MARK) + 0.5);
    const pos = Math.round(this.pos);
    const pp = permille(pos);
    const measured = (pp >= 0 ? 1 : -1) * Math.floor(Math.abs(pp * MARK / 1000) + 0.5);
    const snap = {
      ok: true, state: this.armed ? 'armed_manual' : (Object.keys(this.latches).length ? 'fault' : 'disarmed'),
      connected: true, firmware: 'UBTS_STEER_V3', armed: !!this.lease, moving: this.moving,
      requested: { throttle: this.throttle, steer_mrad: this.steer, left: this.throttle, right: this.throttle },
      reason: this.reason, input_ttl_s: 0.75, automatic_input_enabled: false,
      outputs: { D8: Math.max(this.L, 0), D5: Math.max(-this.L, 0), D3: Math.max(this.R, 0), D0: Math.max(-this.R, 0),
                 D1: 255, D2: this.armed ? 255 : 0, D6: 0, D7: 0 },
      last_error: null, vehicle_profile: 'front_steer', steer_limit_mrad: { ...this.limits },
      protocol: 3, boot_id: '89025CC1', device_id: '5A1B2C', reset_reason: 'power_on', hw_profile: 'front_steer_v0',
      simulated: true, profile: 'front_steer', cal_id: 7,
      steering: { measured_mrad: measured, measured_permille: pp, raw: 512 + Math.round(pp * 0.33),
                  target_permille: permille(this.tgt), cmd_permille: permille(this.tgt), pwm: 0, mcu_ms: this.mcuMs,
                  seq: this.statusSeq, applied_q: this.n, age_ms: this.ageMs, ttl_remaining_ms: 0,
                  drive_en: true, steer_en: this.armed, hold_remaining_ms: null, stop_mode: null,
                  faults: this.faults.concat(Object.keys(this.latches)), servo_state: 'settled', saturated: false,
                  calibrated: true },
      steering_host: { arm_state: this.armed ? 'confirmed' : 'none', stop_state: 'confirmed',
                       mcu_state: this.armed ? 'ARMED' : 'DISARMED', fl: '0000', stale: this.stale,
                       target_mrad: this.steer, latches: { ...this.latches }, last_c: null, next_q: 1,
                       link: { state: 'ready', transport: 'memory', path: 'memory', boot_id: '89025CC1',
                               build: '0A1B2C3D', identity_error: null, hw_mismatch: false, sessions: 1,
                               counters: {}, last_error: null },
                       counters: {}, host_profile: {}, expected_soft_permille: [935, -935],
                       calibration_soft_matches: true },
      transport: 'simulation', transport_kind: 'memory', serial_port: 'memory', serial_open: true, mode: 'simulation',
      device_identity: null, control_page: '/rover-control',
    };
    if (withUpdated) snap.updated_at = this.clock.t / 1000;
    return snap;
  }
  status() {
    if (!this.statusOk) return Promise.reject(new Error('HTTP 502'));
    return Promise.resolve(this.snapshot(true));
  }
}

function makeEnv(options = {}) {
  const clock = { t: 10000 };
  const bridge = new FakeBridge(clock, options);
  const ctl = new S.SteerController({ now: () => clock.t, transport: bridge, sleep: () => Promise.resolve(),
                                      config: options.config });
  const env = {
    clock, bridge, ctl,
    async step(ms = 50, { poll = true } = {}) {
      clock.t += ms;
      bridge.advance(ms);
      if (poll) await ctl.pollStatus();
      const p = ctl.tick();
      // replies of the fake resolve in microtasks; a held reply (holdReplies) must not block the test clock
      if (p) await Promise.race([p, flush()]);
      await flush();
    },
    async run(ms, opts) { for (let t = 0; t < ms; t += 50) await env.step(50, opts); },
    async arm() {
      await env.step(50);
      await ctl.arm();
      await flush();
      assert.ok(ctl.lease, 'armed');
    },
    commands(since = 0) { return bridge.requests.slice(since).filter((r) => r.op === 'command'); },
  };
  return env;
}

// -------------------------------------------------------------------------------------------------------------------
test('the script exports its API and does not need a DOM', () => {
  assert.equal(typeof S.keyFromEvent, 'function');
  assert.equal(S.deriveView, undefined, 'no display model in the rules');
  assert.equal(typeof S.SteerController, 'function');
  assert.equal(S.VEHICLE_PROFILE, 'front_steer');
  assert.equal(S.DEFAULTS.movingWindowMs, 300);
  assert.equal(S.DEFAULTS.speedCapMaxPct, 60);
  assert.deepStrictEqual(S.DEFAULTS.angleChoicesDeg, [5, 10, 15, 20]);
  assert.equal(S.DEFAULTS.statusPeriodMs, 50, '20 Hz status polling');
});

test('fault catalogue: the 15 MCU fault bits in spec order and the host latches', () => {
  assert.deepStrictEqual(S.MCU_FAULTS.map((r) => r[0]), ['NOCAL', 'POT_OPEN', 'POT_SHORT', 'POT_JUMP', 'POT_NOISE',
    'POLARITY', 'DRV_STUCK', 'SOFTLIMIT', 'STALL', 'TRACK', 'THERMAL', 'TTL', 'WATCHDOG', 'CRC', 'BOOT']);
  const host = S.HOST_LATCHES.map((r) => r[0]);
  for (const name of ['steering_status_stale', 'mcu_reset_detected', 'stop_unconfirmed', 'mcu_ttl_expired',
    'steering_not_following', 'steering_ineffective', 'internal_error']) assert.ok(host.includes(name), name);
  for (const row of S.MCU_FAULTS.concat(S.HOST_LATCHES)) assert.ok(/[가-힣]/.test(row[1]), 'Korean label for ' + row[0]);
});

test('permille <-> mrad follows the bridge rule (round half away from zero, per side)', () => {
  const marks = { left: MARK, right: MARK };
  assert.equal(S.permilleFromMrad(87, marks), 199);           // round(1000 * 87 / 438.03) = 198.6 -> 199
  assert.equal(S.permilleFromMrad(-87, marks), -199);
  assert.equal(S.mradFromPermille(-310, marks), -136);
  assert.equal(S.mradFromPermille(0, marks), 0);
  assert.equal(S.roundHalfAway(-2.5), -3);
  assert.equal(S.roundHalfAway(2.5), 3);
  for (let m = -409; m <= 409; m += 7) {
    const back = S.mradFromPermille(S.permilleFromMrad(m, marks), marks);
    assert.ok(Math.abs(back - m) <= 1, `round trip ${m} -> ${back}`);
  }
});

test('MCU drive duty comes from the status line pins (L = D8 - D5, R = D3 - D0)', () => {
  assert.deepStrictEqual(S.mcuDrive({ outputs: { D8: 102, D5: 0, D3: 102, D0: 0, D1: 255, D2: 255 } }),
                         { left: 102, right: 102, en: true, steerEn: true });
  assert.deepStrictEqual(S.mcuDrive({ outputs: { D8: 0, D5: 51, D3: 0, D0: 51, D1: 0, D2: 0 } }),
                         { left: -51, right: -51, en: false, steerEn: false });
  assert.equal(S.mcuDrive({ outputs: {} }), null);
  assert.equal(S.mcuDrive(null), null);
});

test('arm sends vehicle_profile front_steer and takes the measured angle as the first target', async () => {
  const env = makeEnv({ posMrad: 52 });
  await env.arm();
  const arm = env.bridge.requests.find((r) => r.op === 'arm');
  assert.deepStrictEqual(arm.body, { vehicle_profile: 'front_steer' });
  assert.equal(env.ctl.seq, 0);
  assert.ok(Math.abs(env.ctl.hostTarget - 52) <= 1);
  await env.run(300);
  for (const c of env.commands()) {
    assert.equal(c.body.throttle, 0);
    assert.ok(Math.abs(c.body.steer_mrad - 52) <= 1, 'no steering jump at arm');
  }
});

test('an arm refusal is shown and nothing is sent afterwards (no automatic re-arm)', async () => {
  const env = makeEnv();
  env.bridge.failNext = 'steering_uncalibrated';
  await env.step(50);
  await env.ctl.arm();
  assert.equal(env.ctl.lease, null);
  assert.match(env.ctl.message.text, /steering_uncalibrated/);
  await env.run(3000);
  assert.equal(env.bridge.requests.filter((r) => r.op === 'arm').length, 1, 'no automatic re-arm');
  assert.equal(env.bridge.requests.filter((r) => r.op !== 'arm').length, 0);
});

test('W drives only while pressed, at the speed cap; requests chain the ticket and seq; never left/right', async () => {
  const env = makeEnv();
  await env.arm();
  env.ctl.setSpeedCap(40);
  env.ctl.keyDown('w');
  await env.run(500);
  const driving = env.commands();
  assert.ok(driving.length >= 4, 'drive requests while W is held');
  for (const c of driving) {
    assert.deepStrictEqual(Object.keys(c.body).sort(), ['deadman', 'lease', 'seq', 'steer_mrad', 'throttle', 'ticket']);
    assert.equal(c.body.deadman, true);
    assert.ok(Number.isInteger(c.body.steer_mrad));
  }
  assert.equal(driving[driving.length - 1].body.throttle, 40);
  const seqs = env.bridge.requests.filter((r) => r.op !== 'arm').map((r) => r.body.seq);
  seqs.forEach((s, i) => assert.equal(s, i + 1, 'seq + 1 chain'));
  env.ctl.keyUp('w');
  const n = env.bridge.requests.length;
  await env.step(50);
  const after = env.bridge.requests.slice(n);
  assert.equal(after[0].op, 'command');
  assert.equal(after[0].body.throttle, 0, 'W released: throttle 0 at once');
  await env.run(1200);
  const idle = env.bridge.requests.slice(n + 1);
  assert.ok(idle.length >= 1 && idle.every((r) => r.op === 'keepalive'), 'idle: keepalive only');
  assert.equal(env.ctl.lease !== null, true);
});

test('S reverses at the cap; W+S gives 0; the cap is 0-60 % with Q/E steps of 5', async () => {
  const env = makeEnv();
  await env.arm();
  env.ctl.setSpeedCap(80);
  assert.equal(env.ctl.speedCapPct, 60);
  env.ctl.setSpeedCap(-3);
  assert.equal(env.ctl.speedCapPct, 0);
  env.ctl.setSpeedCap(25);
  env.ctl.nudgeSpeed(1);
  assert.equal(env.ctl.speedCapPct, 30);
  env.ctl.keyDown('s');
  await env.step(50);
  assert.equal(env.commands().at(-1).body.throttle, -30);
  env.ctl.keyDown('w');
  await env.step(50);
  assert.equal(env.commands().at(-1).body.throttle, 0);
});

test('A/D while moving: target rate <= 20 deg/s toward the selected angle, never past it', async () => {
  const env = makeEnv();
  await env.arm();
  env.ctl.setSpeedCap(30);
  env.ctl.keyDown('w');
  await env.run(400);                        // moving: MCU duty != 0
  assert.ok(env.ctl.moving(env.clock.t));
  env.ctl.setAngle(10);
  env.ctl.keyDown('a');
  const start = env.commands().length;
  let prev = null;
  for (let i = 0; i < 30; i += 1) {
    await env.step(50);
  }
  const steer = env.commands().slice(start).map((c) => ({ at: 0, s: c.body.steer_mrad }));
  const sent = env.bridge.requests.filter((r) => r.op === 'command').slice(start);
  for (const r of sent) {
    if (prev) {
      const dt = (r.at - prev.at) / 1000;
      assert.ok(Math.abs(r.body.steer_mrad - prev.body.steer_mrad) <= mrad(20) * dt + 1.01, 'rate cap 20 deg/s');
    }
    prev = r;
  }
  assert.ok(steer.length > 3);
  assert.equal(sent.at(-1).body.steer_mrad, Math.round(mrad(10)));
  assert.ok(sent.every((r) => r.body.steer_mrad <= Math.round(mrad(10))));
  assert.ok(sent.every((r) => r.body.steer_mrad >= 0), 'A = left = positive');
});

test('the selected angle never exceeds the equivalent soft limit of the status', async () => {
  const env = makeEnv({ limits: { left: 300, right: 280 } });
  await env.arm();
  const choices = env.ctl.angleChoices();
  assert.deepStrictEqual(choices.map((c) => c.enabled), [true, true, true, false], '20 deg > 17.2 deg is disabled');
  assert.equal(env.ctl.setAngle(20), false);
  env.ctl.setAngle(15);
  env.ctl.keyDown('w');
  await env.run(300);
  env.ctl.keyDown('d');
  await env.run(2000);
  const last = env.commands().at(-1).body.steer_mrad;
  assert.equal(last, -Math.min(Math.round(mrad(15)), 280));
  assert.ok(env.commands().every((c) => c.body.steer_mrad >= -280 && c.body.steer_mrad <= 300));
});

test('A/D release while moving returns to the centre at <= 20 deg/s, only while moving', async () => {
  const env = makeEnv();
  await env.arm();
  env.ctl.keyDown('w');
  await env.run(300);
  env.ctl.setAngle(10);
  env.ctl.keyDown('a');
  await env.run(800);
  assert.equal(env.commands().at(-1).body.steer_mrad, Math.round(mrad(10)));
  env.ctl.keyUp('a');                        // still moving (W held)
  assert.equal(env.ctl.centreReturn, true);
  const start = env.bridge.requests.filter((r) => r.op === 'command').length;
  await env.run(1000);
  const back = env.bridge.requests.filter((r) => r.op === 'command').slice(start);
  assert.equal(back.at(-1).body.steer_mrad, 0, 'back at the centre');
  for (let i = 1; i < back.length; i += 1) {
    const dt = (back[i].at - back[i - 1].at) / 1000;
    assert.ok(back[i - 1].body.steer_mrad - back[i].body.steer_mrad <= mrad(20) * dt + 1.01);
    assert.ok(back[i].body.steer_mrad <= back[i - 1].body.steer_mrad, 'monotonic return');
  }
});

test('A/D release while standing keeps the wheels where they are: no return to the centre (A22 screen)', async () => {
  const env = makeEnv({ posMrad: 0 });
  await env.arm();
  env.ctl.setAngle(5);
  env.ctl.keyDown('a');                       // standing: steering-only input, throttle 0
  await env.run(1000);
  assert.equal(env.ctl.moving(env.clock.t), false);
  const measuredAtRelease = env.ctl.measuredMrad();
  assert.ok(measuredAtRelease > 20, 'the wheels turned while A was held');
  env.ctl.keyUp('a');
  assert.equal(env.ctl.centreReturn, false);
  const start = env.commands().length;
  await env.run(3000);                        // observe 3 s
  const after = env.commands().slice(start);
  assert.ok(after.length >= 1, 'one drive sets the hold target');
  for (const c of after) {
    assert.equal(c.body.throttle, 0);
    assert.ok(Math.abs(c.body.steer_mrad - measuredAtRelease) <= 1, 'target = measured angle at release, kept');
  }
  const pos = env.bridge.pos;
  assert.ok(Math.abs(pos - measuredAtRelease) <= 2, 'no centring motion at standstill');
});

test('moving = an MCU duty != 0 within the last 0.3 s of fresh status; stale counts as standing', async () => {
  const env = makeEnv();
  await env.arm();
  env.ctl.keyDown('w');
  await env.run(300);
  env.ctl.keyUp('w');
  await env.step(50);                          // throttle 0 sent; next status shows duty 0
  await env.step(50);
  const zeroAt = env.clock.t;
  assert.equal(env.ctl.moving(zeroAt), true, 'within 0.3 s of the last non-zero duty');
  await env.run(350);
  assert.equal(env.ctl.moving(env.clock.t), false, 'more than 0.3 s after');
  // a non-zero duty in a stale snapshot is not enough to call it moving for the release rule
  const env2 = makeEnv();
  await env2.arm();
  env2.ctl.keyDown('w');
  await env2.run(300);
  env2.bridge.ageMs = 400;                     // MCU line older than 150 ms: stale
  await env2.step(50);
  env2.ctl.keyDown('a');
  await env2.step(50);
  env2.ctl.keyUp('a');
  assert.equal(env2.ctl.centreReturn, false, 'unsure = standing: no automatic centring');
});

test('a centre return stops (freezes at the measured angle) when the vehicle stops', async () => {
  const env = makeEnv({ rateDegS: 60 });
  await env.arm();
  env.ctl.keyDown('w');
  await env.run(300);
  env.ctl.setAngle(20);
  env.ctl.keyDown('a');
  await env.run(1500);
  env.ctl.keyUp('a');
  env.ctl.keyUp('w');                          // both released: the duty goes to 0 at once in the fake MCU
  await env.run(150);
  assert.equal(env.ctl.centreReturn, true, 'still moving for up to 0.3 s');
  await env.run(400);
  assert.equal(env.ctl.centreReturn, false, 'standing: the return stopped');
  const held = env.commands().at(-1).body.steer_mrad;
  assert.ok(held > 0 && held < Math.round(mrad(20)), 'stopped part-way, not at the centre');
  const n = env.commands().length;
  await env.run(2000);
  assert.ok(env.commands().slice(n).every((c) => c.body.steer_mrad === held), 'no further centring at standstill');
});

test('standstill: one A/D press moves the target at most 5 deg at <= 12 deg/s (R18-3, proposal)', async () => {
  const env = makeEnv({ rateDegS: 4 });        // slow placeholder actuator
  await env.arm();
  env.ctl.setAngle(20);
  env.ctl.keyDown('d');
  await env.run(2000);
  const sent = env.commands();
  assert.ok(sent.length > 3);
  for (let i = 1; i < sent.length; i += 1) {
    const dt = (sent[i].at - sent[i - 1].at) / 1000;
    assert.ok(Math.abs(sent[i].body.steer_mrad - sent[i - 1].body.steer_mrad) <= mrad(12) * dt + 1.01, 'standstill rate');
  }
  assert.ok(sent.every((c) => c.body.steer_mrad >= -Math.round(mrad(5)) - 1), 'one press <= 5 deg');
  assert.equal(env.ctl.stepOrigin !== null, true);
});

test('centre button: a deadman at the standstill rate, at most 5 deg per hold; release freezes', async () => {
  const env = makeEnv({ posMrad: Math.round(mrad(12)) });
  await env.arm();
  const start = env.ctl.measuredMrad();
  env.ctl.setCentre(true);
  await env.run(1500);
  const sent = env.commands();
  for (let i = 1; i < sent.length; i += 1) {
    const dt = (sent[i].at - sent[i - 1].at) / 1000;
    assert.ok(sent[i - 1].body.steer_mrad - sent[i].body.steer_mrad <= mrad(12) * dt + 1.01);
  }
  assert.ok(sent.at(-1).body.steer_mrad >= start - Math.round(mrad(5)) - 1, '<= 5 deg per hold');
  env.ctl.setCentre(false);
  const measured = env.ctl.measuredMrad();
  const n = env.commands().length;
  await env.run(2000);
  const after = env.commands().slice(n);
  assert.ok(after.every((c) => Math.abs(c.body.steer_mrad - measured) <= 1), 'released: kept where it is');
  env.ctl.setCentre(true);                      // a second hold continues toward the centre
  await env.run(1500);
  assert.ok(env.commands().at(-1).body.steer_mrad < measured - 10);
});

test('steering-only inputs need the lease: nothing is sent before Arm', async () => {
  const env = makeEnv();
  await env.step(50);
  env.ctl.keyDown('a');
  env.ctl.setCentre(true);
  env.ctl.keyDown('w');
  await env.run(1000);
  assert.equal(env.bridge.requests.length, 0);
  assert.match(env.ctl.message.text, /Arm/);
});

test('steer hold: an A/D release while moving keeps the angle', async () => {
  const env = makeEnv();
  await env.arm();
  env.ctl.setSteerHold(true);
  env.ctl.keyDown('w');
  await env.run(300);
  env.ctl.setAngle(5);
  env.ctl.keyDown('a');
  await env.run(800);
  env.ctl.keyUp('a');
  assert.equal(env.ctl.centreReturn, false);
  await env.run(1000);
  assert.equal(env.commands().at(-1).body.steer_mrad, Math.round(mrad(5)));
});

const STOP_STATES = {
  async 'not armed'(env) { await env.step(50); },
  async 'arming (arm in flight)'(env) { env.bridge.holdReplies = true; await env.step(50); env.ctl.arm(); await flush(); },
  async 'armed idle'(env) { await env.arm(); await env.run(200); },
  async 'driving with a request in flight'(env) {
    await env.arm(); env.ctl.keyDown('w'); env.ctl.keyDown('a'); await env.run(300);
    env.bridge.holdReplies = true; env.clock.t += 120; env.ctl.tick(); await flush();
  },
  async 'releasing'(env) { await env.arm(); env.ctl.keyDown('w'); await env.run(200); env.ctl.release(); await env.step(50); },
  async 'stale status'(env) { await env.arm(); env.bridge.ageMs = 500; await env.run(200); },
};
for (const [name, setup] of Object.entries(STOP_STATES)) {
  test('STOP from every state: ' + name, async () => {
    const env = makeEnv();
    await setup(env);
    const before = env.bridge.requests.length;
    await env.ctl.stop('operator_stop');
    assert.equal(env.ctl.lease, null);
    const stops = env.bridge.requests.slice(before).filter((r) => r.op === 'stop');
    assert.equal(stops.length, 1, 'one STOP request');
    assert.deepStrictEqual(stops[0].body, { reason: 'operator_stop' });
    env.bridge.holdReplies = false;
    env.bridge.releasePending();                 // late replies of requests sent before the STOP
    await flush();
    await env.run(3000);                         // observe 3 s
    const later = env.bridge.requests.slice(before).filter((r) => r.op !== 'stop');
    assert.deepStrictEqual(later.map((r) => r.op), [], 'no command, keepalive or arm after STOP');
    assert.equal(env.ctl.lease, null, 'no lease resurrected by a late reply');
  });
}

test('an arm granted after a STOP is stopped again (cancelled_arm), never kept', async () => {
  const env = makeEnv();
  env.bridge.holdReplies = true;
  await env.step(50);
  const arming = env.ctl.arm();
  await flush();
  await env.ctl.stop('operator_stop');
  env.bridge.holdReplies = false;
  env.bridge.releasePending();
  await arming;
  await flush();
  assert.equal(env.ctl.lease, null);
  assert.deepStrictEqual(env.bridge.stops, ['operator_stop', 'cancelled_arm']);
});

test('stale status: throttle blocked and target frozen; stale 1 s while armed sends STOP', async () => {
  const env = makeEnv();
  await env.arm();
  env.ctl.keyDown('w');
  env.ctl.keyDown('a');
  await env.run(300);
  env.bridge.ageMs = 400;                         // MCU line age > 150 ms
  await env.step(50);
  const n = env.commands().length;
  await env.run(400);
  const stale = env.commands().slice(n);
  assert.ok(stale.length >= 1);
  const frozen = stale[0].body.steer_mrad;
  assert.ok(stale.every((c) => c.body.throttle === 0), 'no throttle on a stale status');
  assert.ok(stale.every((c) => c.body.steer_mrad === frozen), 'steering target frozen');
  await env.run(800);
  assert.equal(env.ctl.lease, null);
  assert.ok(env.bridge.stops.includes('page_status_stale'));
});

test('no status from any channel (polls fail, a request hangs) stops after 1 s', async () => {
  const env = makeEnv();
  await env.arm();
  env.ctl.keyDown('w');
  await env.run(200);
  env.bridge.statusOk = false;                    // GET /api/rover/status fails
  env.bridge.holdReplies = true;                  // and the next request gets no reply (its reply carries a status)
  await env.run(1200);
  assert.ok(env.ctl.lease, 'stale 0.3 s after the last status, STOP 1 s later');
  await env.run(300);
  assert.equal(env.ctl.lease, null);
  assert.ok(env.bridge.stops.includes('page_status_stale'));
  assert.ok(env.ctl.pollFailures > 5);
});

test('failing polls alone do not stop while request replies keep the status fresh', async () => {
  const env = makeEnv();
  await env.arm();
  env.ctl.keyDown('w');
  await env.run(200);
  env.bridge.statusOk = false;
  await env.run(1500);                            // drive replies every 100 ms carry the bridge status
  assert.ok(env.ctl.lease);
  assert.equal(env.bridge.stops.length, 0);
});

test('ownership loss by an error reply: lease dropped, STOP sent, no re-arm', async () => {
  const env = makeEnv();
  await env.arm();
  env.ctl.keyDown('w');
  await env.run(200);
  env.bridge.failNext = 'control_not_owned';
  await env.run(100);
  assert.equal(env.ctl.lease, null);
  assert.match(env.ctl.message.text, /control_not_owned/);
  assert.ok(env.bridge.stops.includes('page_request_failed'));
  const n = env.bridge.requests.length;
  await env.run(3000);
  assert.ok(env.bridge.requests.slice(n).every((r) => r.op === 'stop'));
});

test('ownership loss seen in the status (the bridge revoked the lease): dropped without a new STOP', async () => {
  const env = makeEnv();
  await env.arm();
  await env.run(200);
  env.bridge.stopNow('mcu_disarmed');
  env.bridge.latches.mcu_disarmed = { since: 1, detail: 'MCU watchdog', count: 1 };
  await env.step(50);
  assert.equal(env.ctl.lease, null);
  assert.match(env.ctl.message.text, /mcu_disarmed/);
  assert.equal(env.bridge.stops.length, 0);
  // the screen shows the latch from the status the rules keep (the display model went with the page)
  assert.ok(env.ctl.lastStatus.steering_host.latches.mcu_disarmed);
});

test('a status requested before the arm reply never counts as a lost lease', () => {
  const clock = { t: 5000 };
  const bridge = new FakeBridge(clock);
  const ctl = new S.SteerController({ now: () => clock.t, transport: bridge, sleep: () => Promise.resolve() });
  ctl.lease = 'L'; ctl.ticket = 'T'; ctl.leaseAt = 5000;
  const old = bridge.snapshot(true);
  old.armed = false;
  ctl.onStatus(old, 5010, 4990);                 // sent before the lease was granted
  assert.equal(ctl.lease, 'L');
  ctl.onStatus(old, 5030, 5020);                 // sent after
  assert.equal(ctl.lease, null);
});

test('request_expired keeps the lease and sends again (nothing was dispatched)', async () => {
  const env = makeEnv();
  await env.arm();
  env.ctl.keyDown('w');
  await env.step(50);
  env.bridge.failNext = 'request_expired';
  await env.step(100);
  assert.ok(env.ctl.lease);
  await env.run(300);
  assert.ok(env.ctl.lease);
  assert.equal(env.bridge.stops.length, 0);
});

test('Release: release op, then STOP once the MCU duty is 0 (graceful disarm)', async () => {
  const env = makeEnv();
  await env.arm();
  env.ctl.keyDown('w');
  await env.run(300);
  env.ctl.release();
  await env.run(400);
  const ops = env.bridge.requests.map((r) => r.op);
  assert.ok(ops.includes('release'));
  assert.ok(ops.indexOf('release') < ops.lastIndexOf('stop'));
  assert.deepStrictEqual(env.bridge.stops, ['operator_release']);
  assert.equal(env.ctl.lease, null);
});

test('blur releases every held input (no stuck keys)', async () => {
  const env = makeEnv();
  await env.arm();
  env.ctl.keyDown('w');
  env.ctl.keyDown('d');
  env.ctl.setCentre(true);
  env.ctl.blur();
  assert.deepStrictEqual({ ...env.ctl.keys }, { w: false, s: false, a: false, d: false });
  assert.equal(env.ctl.centreHeld, false);
  await env.step(50);
  assert.equal(env.commands().at(-1).body.throttle, 0);
});

test('keys map by the physical key code (layout and IME independent)', () => {
  assert.equal(S.keyFromEvent({ code: 'KeyW', key: 'ㅈ' }), 'w');
  assert.equal(S.keyFromEvent({ code: 'KeyD', key: 'ㅇ' }), 'd');
  assert.equal(S.keyFromEvent({ code: 'KeyC', key: 'c' }), 'c');
  assert.equal(S.keyFromEvent({ code: 'Space', key: ' ' }), '');
  assert.equal(S.keyFromEvent({ code: '', key: 'A' }), 'a');
});

test('no request of any test sent left/right (the page never uses the differential format)', () => {
  assert.ok(ALL_REQUESTS.length > 100);
  for (const r of ALL_REQUESTS) {
    assert.ok(!('left' in r.body) && !('right' in r.body), JSON.stringify(r));
    if (r.op === 'command') assert.equal(r.body.deadman, true);
  }
});
