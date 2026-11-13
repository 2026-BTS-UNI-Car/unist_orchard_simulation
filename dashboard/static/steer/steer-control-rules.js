/* Control rules of the front-steer manual steering (design 4.4 and 4.2, 8.3 P6b), without a screen.
 *
 * Source: unist_orchard_simulation 34cff72 dashboard/static/steer-control-rules.js (the rules kept from the phase-4
 * page draft). Changed here (2026-11-13, UI-07, docs/STEERING_SERVER_KO.md 3): the release rule has a steering mode,
 * 'centre' (default, decision 9: an A/D release returns to the centre, at standstill at the standstill rate and at most
 * one standstill step per release) or 'design' (DESIGN_v1 4.4: centre only while moving, standing keeps the wheels);
 * steer hold keeps the target in both; abandon() drops the lease locally when another control sent the STOP;
 * device() tells a steering bridge from a V2 bridge or none; the messages name the arm button '조종 활성화' as the page does.
 *
 * Kept from the phase-4 steering page draft (DECISIONS R22, plan decision 13): the page itself (HTML, CSS, display
 * model, top view, the SIM-only display feed) is not kept; the dashboard is redesigned in its own repository and its
 * steering screen is built on these rules. A classic script (no module, no build step, no DOM) that exports
 * UbtsSteerRules. The caller gives a transport {status(), post(path, body)}, calls pollStatus() and tick() every
 * statusPeriodMs, and passes key and button events (keyDown / keyUp / setCentre / setSteerHold / setSpeedCap /
 * setAngle / blur / arm / release / stop). Routes (the existing dashboard routes; app.py is not changed):
 *   POST /api/rover/arm        {vehicle_profile: "front_steer"}
 *   POST /api/rover/command    {lease, ticket, seq, deadman: true, throttle, steer_mrad}   (the bridge op "drive")
 *   POST /api/rover/keepalive  {lease, ticket, seq}
 *   POST /api/rover/release    {lease, ticket, seq}
 *   POST /api/rover/stop       {reason}                                                   (no lease needed)
 *   GET  /api/rover/status     the bridge snapshot with the steering block (HOST_BRIDGE_V3_KO.md 7), polled at 20 Hz
 * The dashboard adds "op" and "owner" (the session cookie) to each POST body. The rules never send left/right.
 * controller.marks ({left, right} mrad: the host profile calibration marks) may be set by the caller for an exact
 * permille <-> mrad conversion; without it the scale per side comes from the status itself.
 *
 * Rules (design 4.4; values marked [proposal] are placeholders, not registered values):
 *   - Arm needs a click. No automatic re-arm after any stop, refusal or lost lease.
 *   - STOP (button, Esc) is always available, also without a lease and with a stale status.
 *   - W/S drive only while pressed, at the speed cap (0-60 %).
 *   - A/D steer while pressed toward the selected angle (5/10/15/20 deg, never above the equivalent soft limit of
 *     the status) with a target rate cap of 20 deg/s while moving [proposal] and 12 deg/s at standstill [proposal].
 *   - Moving = an MCU-reported drive duty (status line L or R, from the pin outputs) other than 0 within the last
 *     0.3 s, from a fresh status. On A/D release the target returns to the centre only while moving; standing still
 *     keeps the wheels where they are (the target becomes the measured angle). Unsure (stale status) counts as
 *     standing.
 *   - Standing still, one A/D press or one centre hold moves the target at most 5 deg (DECISIONS R18 decision 3,
 *     VEHICLE_SPEC_REQUEST Q17) [proposal].
 *   - "Centre" is a deadman (hold to move) at the standstill rate cap. "Steer hold" keeps the target after an A/D
 *     release while moving (calibration drives).
 *   - Steering-only inputs (throttle 0) also need the lease, the deadman flag and an armed bridge.
 *   - A stale status blocks throttle and freezes the steering target; stale for 1 s while armed sends STOP [proposal].
 *   - Keys map by the physical key code (layout and IME independent).
 * The fault and latch catalogues carry the short Korean labels a screen shows. Placeholder-based SIM results only
 * (자리값 기반 SIM 결과), never field validation. Tests: node --test dashboard/static/test_steer_control_rules.mjs.
 */
(function (root) {
  'use strict';

  var DEFAULTS = Object.freeze({
    statusPeriodMs: 50,          // 20 Hz short polling of GET /api/rover/status (design 4.4)
    statusTimeoutMs: 400,
    commandPeriodMs: 100,        // drive requests while an input is active [proposal]
    keepalivePeriodMs: 500,      // idle and armed (the ticket lives 1.5 s, RoverControl.TICKET_TTL)
    movingWindowMs: 300,         // design 4.4: duty != 0 within the last 0.3 s
    movingRateDegS: 20,          // A/D and centre return while moving (design 4.4 example) [proposal]
    standstillRateDegS: 12,      // MCU rstd 480 permille/s at 39.845 permille/deg (CFG default) [proposal, Q7]
    standstillStepDeg: 5,        // one press or hold at standstill (R18 decision 3, Q17) [proposal]
    angleChoicesDeg: [5, 10, 15, 20],
    defaultSteerMode: 'centre',  // decision 9 (todo 4 da9): release returns to the centre; 'design' = DESIGN_v1 4.4
    defaultAngleDeg: 10,
    speedCapMaxPct: 60,
    defaultSpeedCapPct: 20,
    speedStepPct: 5,
    pollStaleMs: 300,            // no status reply for this long: stale [proposal]
    statusAgeMaxMs: 150,         // MCU status line age limit (host_status_gap_ms placeholder)
    staleStopMs: 1000,           // armed and stale this long: STOP [proposal]
    inputTtlMs: 750,             // RoverControl.INPUT_TTL (display of the remaining input deadline)
    releaseTimeoutMs: 1200,      // Release: ramp-down wait before the STOP (release bound 1.1 s + one tick)
    maxRampDtMs: 200,
    stopRetries: 3,
    stopRetryMs: 150,
    sampleKeepMs: 2000,
  });

  var VEHICLE_PROFILE = 'front_steer';

  // MCU fault bits (ESP_PROTOCOL_V3_DRAFT_KO.md 5.3, bit order) with a short Korean label and a display class.
  var MCU_FAULTS = [
    ['NOCAL', '보정 없음', 'bad'], ['POT_OPEN', '가변저항 끊김', 'bad'], ['POT_SHORT', '가변저항 단락', 'bad'],
    ['POT_JUMP', '가변저항 급변', 'bad'], ['POT_NOISE', '가변저항 잡음', 'bad'], ['POLARITY', '극성 반대', 'bad'],
    ['DRV_STUCK', '드라이버 고착', 'bad'], ['SOFTLIMIT', '소프트 한계 밖', 'warn'], ['STALL', '조향 멈춤', 'bad'],
    ['TRACK', '추종 실패', 'bad'], ['THERMAL', '과열 예산', 'bad'], ['TTL', 'C 유효시간 만료', 'warn'],
    ['WATCHDOG', '링크 감시 만료', 'bad'], ['CRC', 'CRC 연속 오류', 'bad'], ['BOOT', '방금 부팅', 'info'],
  ];
  // Host latches of the bridge (steer_bridge_core.LATCH_NAMES; mcu_ttl_expired is DECISIONS R18 decision 1).
  var HOST_LATCHES = [
    ['steering_status_stale', '상태 줄 끊김'], ['mcu_reset_detected', 'MCU 리셋 감지'],
    ['calibration_mismatch', '보정 불일치'], ['hardware_profile_mismatch', '하드웨어 프로필 불일치'],
    ['sim_identity_mismatch', 'SIM 신원 불일치'], ['steering_not_following', '조향 추종 안 됨'],
    ['steering_ineffective', '조향 효과 없음'], ['status_seq_gap', '상태 번호 누락'],
    ['stop_unconfirmed', 'STOP 확인 실패'], ['mcu_disarmed', 'MCU 스스로 해제'],
    ['mcu_state_mismatch', 'MCU 상태 불일치'], ['c_rejected', 'C 명령 거절'], ['hardware_fault', '전송 장애'],
    ['arm_rejected', 'ARM 거절'], ['arm_unconfirmed', 'ARM 확인 없음'], ['internal_error', '브리지 내부 오류'],
    ['mcu_ttl_expired', 'C TTL 만료 래치'],
  ];
  // Error codes and stop reasons (HOST_BRIDGE_V3_KO.md 3) -> short Korean text.
  var MESSAGES = {
    steer_profile_required: '조향 화면이 아닌 요청(vehicle_profile 없음): arm 거절',
    differential_command_rejected: '좌우(left/right) 명령 거절: 정지',
    firmware_v3_required: 'V3 펌웨어 배너가 없음',
    steering_uncalibrated: '조향 보정 없음',
    calibration_mismatch: '보정(cal_id) 불일치',
    hardware_profile_mismatch: '하드웨어 프로필 불일치',
    polarity_check_missing: '극성 확인(CAL POL) 필요',
    steering_fault_latched: 'MCU 결함 조건이 남아 있음',
    steering_status_stale: '조향 상태 줄 끊김',
    stop_unconfirmed: 'STOP 확인이 아직 없음 또는 실패: 물리 비상정지를 쓰세요',
    sim_identity_mismatch: 'SIM 신원 불일치(sim 배너)',
    link_config_mismatch: '링크 설정(wd, ttl, st) 불일치',
    hardware_fault: '전송 장애',
    config_unsaved: '저장하지 않은 CFG 값이 MCU에 있음',
    internal_error: '브리지 내부 오류',
    operator_required: '조종자 세션 없음',
    control_in_use: '다른 화면이 조종권을 가짐',
    control_not_owned: '조종권 없음(다른 화면 또는 이미 해제)',
    stale_input_rearm_required: '입력표가 만료됨: 다시 조종 활성화',
    invalid_sequence: '요청 순번 오류: 정지',
    invalid_drive: '구동 값 오류: 정지',
    unknown_operation: '알 수 없는 요청',
    control_busy_stopped: '브리지가 바쁨: 정지',
    stop_barrier: '다른 STOP이 먼저 처리됨',
    request_expired: '요청이 큐에서 만료됨',
    motor_bridge_unavailable: '브리지에 연결할 수 없음',
    steering_busy: 'MCU가 바쁨(ARM 거절)',
    arm_unconfirmed: 'ARM 확인 없음',
    arm_rejected: 'ARM 거절',
    mcu_reset_detected: 'MCU 리셋 감지',
    mcu_ttl_expired: 'C TTL 만료 래치',
    mcu_disarmed: 'MCU가 스스로 해제',
    input_expired: '입력 기한 만료',
    operator_expired: '조종자 30 s 무응답',
    device_unavailable: '장치 연결 끊김',
    operator_stop: 'STOP',
    operator_release: 'Release(감속 뒤 해제)',
    shutdown: '브리지 종료',
    link_reopened: '링크 다시 열림',
    page_status_stale: '화면 상태 1 s 넘게 오래됨: STOP',
    page_hidden: '화면이 가려짐: STOP',
    page_request_failed: '요청 실패: STOP',
    escape: 'Esc: STOP',
    http_failed: 'HTTP 요청 실패',
    bad_reply: '응답 형식 오류',
  };

  // ---------------------------------------------------------------------------------------------------------------
  // pure helpers
  function isNum(x) { return typeof x === 'number' && isFinite(x); }
  function num(x, fallback) { return isNum(x) ? x : (fallback === undefined ? 0 : fallback); }
  function clamp(x, lo, hi) { return Math.min(hi, Math.max(lo, x)); }
  function degToMrad(deg) { return deg * Math.PI / 0.18; }
  function mradToDeg(mrad) { return mrad * 0.18 / Math.PI; }
  function roundHalfAway(x) { return (x < 0 ? -1 : 1) * Math.floor(Math.abs(x) + 0.5); }
  function messageFor(code) {
    if (!code) return '';
    if (MESSAGES[code]) return MESSAGES[code] + ' (' + code + ')';
    if (String(code).indexOf('arm_rejected_') === 0) return 'ARM 거절 (' + code + ')';
    return String(code);
  }

  // MCU drive duty of the latest status line: the bridge pin outputs D8/D5 (left) and D3/D0 (right), D1 = drive EN
  // (steer_bridge_core.pin_outputs; the steering block itself has no L/R).
  function mcuDrive(snapshot) {
    var out = snapshot && snapshot.outputs;
    if (!out || typeof out !== 'object' || !isNum(out.D8) || !isNum(out.D5) || !isNum(out.D3) || !isNum(out.D0)) {
      return null;
    }
    return { left: out.D8 - out.D5, right: out.D3 - out.D0, en: num(out.D1) > 0, steerEn: num(out.D2) > 0 };
  }

  // permille <-> mrad with the calibration marks (HostProfile.permille_from_mrad / mrad_from_permille).
  function mradFromPermille(permille, marks) {
    if (!isNum(permille) || !marks || !isNum(marks.left) || !isNum(marks.right)) return null;
    if (permille >= 0) return roundHalfAway(permille * marks.left / 1000);
    return -roundHalfAway(-permille * marks.right / 1000);
  }
  function permilleFromMrad(mrad, marks) {
    if (!isNum(mrad) || !marks || !isNum(marks.left) || !isNum(marks.right)) return null;
    if (mrad >= 0) return roundHalfAway(1000 * mrad / marks.left);
    return -roundHalfAway(1000 * -mrad / marks.right);
  }



  // ---------------------------------------------------------------------------------------------------------------
  // controller
  function SteerController(options) {
    var o = options || {};
    this.cfg = Object.assign({}, DEFAULTS, o.config || {});
    this.now = o.now || function () { return Date.now(); };
    this.transport = o.transport;
    this.sleep = o.sleep || function (ms) { return new Promise(function (resolve) { setTimeout(resolve, ms); }); };
    this.onChange = o.onChange || function () {};
    this.epoch = 0;
    this.lease = null;
    this.ticket = null;
    this.seq = 0;
    this.leaseAt = null;
    this.inFlight = null;
    this.arming = false;
    this.releasing = false;
    this.releaseSent = false;
    this.releaseStartAt = null;
    this.releaseReplyAt = null;
    this.keys = { w: false, s: false, a: false, d: false };
    this.centreHeld = false;
    this.steerHold = false;
    this.steerMode = this.cfg.defaultSteerMode === 'design' ? 'design' : 'centre';
    this.speedCapPct = this.cfg.defaultSpeedCapPct;
    this.angleDeg = this.cfg.defaultAngleDeg;
    this.hostTarget = null;        // the page's ramped steering target (mrad, float)
    this.stepOrigin = null;        // target at the start of a standstill press (mrad)
    this.centreReturn = false;     // A/D released while moving: ramping back to 0 while moving
    this.lastRampAt = null;
    this.dirty = false;
    this.lastSent = { at: null, op: null, throttle: null, steer: null };
    this.inputDeadlineAt = null;
    this.lastStatus = null;
    this.lastStatusAt = null;
    this.lastStatusSentAt = null;
    this.polling = false;
    this.pollFailures = 0;
    this.samples = [];
    this.lastSampleKey = null;
    this.staleSince = null;
    this.scale = { left: null, right: null };
    this.marks = null;             // {left, right} mrad from geometry.json (the bridge's host profile)
    this.message = { kind: 'info', text: '조종 활성화를 눌러 시작한다.', at: null };
    this.requests = [];            // recent requests (bounded), for the display and the tests
    this.stopReplies = 0;
  }

  SteerController.prototype._emit = function () { try { this.onChange(this); } catch (err) { /* display only */ } };
  SteerController.prototype._say = function (kind, text) {
    this.message = { kind: kind, text: text, at: this.now() };
  };
  SteerController.prototype._record = function (op, body) {
    this.requests.push({ at: this.now(), op: op, body: body, epoch: this.epoch });
    if (this.requests.length > 200) this.requests.splice(0, this.requests.length - 200);
  };

  SteerController.prototype.limits = function () {
    var lim = this.lastStatus && this.lastStatus.steer_limit_mrad;
    if (!lim || !isNum(lim.left) || !isNum(lim.right) || lim.left <= 0 || lim.right <= 0) return null;
    return { left: lim.left, right: lim.right };
  };
  SteerController.prototype.steering = function () {
    var s = this.lastStatus && this.lastStatus.steering;
    return s && typeof s === 'object' ? s : {};
  };
  SteerController.prototype.measuredMrad = function () {
    var m = this.steering().measured_mrad;
    return isNum(m) ? m : null;
  };
  SteerController.prototype.statusOk = function () {
    return !!(this.lastStatus && this.lastStatus.ok !== false && this.lastStatus.steering);
  };
  SteerController.prototype.fresh = function (now) {
    if (!this.statusOk() || this.lastStatusAt === null) return false;
    if (now - this.lastStatusAt > this.cfg.pollStaleMs) return false;
    var s = this.steering();
    if (!isNum(s.age_ms) || s.age_ms > this.cfg.statusAgeMaxMs) return false;
    var host = this.lastStatus.steering_host;
    return !(host && host.stale === true);
  };
  SteerController.prototype.moving = function (now) {
    var limit = now - this.cfg.movingWindowMs;
    for (var i = this.samples.length - 1; i >= 0; i -= 1) {
      var s = this.samples[i];
      if (s.nonzero && s.t >= limit) return true;
    }
    return false;
  };
  SteerController.prototype.drive = function () { return mcuDrive(this.lastStatus); };
  SteerController.prototype.angleChoices = function () {
    var lim = this.limits();
    var maxDeg = lim ? mradToDeg(Math.min(lim.left, lim.right)) : null;
    return this.cfg.angleChoicesDeg.map(function (deg) {
      return { deg: deg, enabled: maxDeg !== null && deg <= maxDeg + 1e-9 };
    });
  };
  SteerController.prototype.selectedAngleMrad = function () {
    var lim = this.limits();
    if (!lim) return 0;
    return Math.min(degToMrad(this.angleDeg), lim.left, lim.right);
  };
  SteerController.prototype.marksForConversion = function () {
    if (this.marks) return { left: this.marks.left, right: this.marks.right, exact: true };
    if (isNum(this.scale.left) && isNum(this.scale.right)) {
      return { left: this.scale.left * 1000, right: this.scale.right * 1000, exact: false };
    }
    return null;
  };

  // -- status
  SteerController.prototype.onStatus = function (snap, receivedAt, sentAt) {
    if (!snap || typeof snap !== 'object') return;
    if (snap.ok === false && !snap.steering) {          // dashboard: bridge unavailable
      this.pollFailures += 1;
      this.lastBridgeError = snap.error || 'unavailable';
      return;
    }
    if (this.lastStatus && isNum(snap.updated_at) && isNum(this.lastStatus.updated_at)
        && snap.updated_at < this.lastStatus.updated_at && this.lastStatusAt !== null && receivedAt - this.lastStatusAt < 1000) {
      return;                                             // an older snapshot than the one shown
    }
    this.lastStatus = snap;
    this.lastStatusAt = receivedAt;
    this.lastStatusSentAt = isNum(sentAt) ? sentAt : receivedAt;
    this.lastBridgeError = null;
    var s = snap.steering || {};
    var d = mcuDrive(snap);
    if (d && isNum(s.age_ms)) {
      var key = String(s.mcu_ms) + '/' + String(s.seq);
      if (key !== this.lastSampleKey) {
        this.lastSampleKey = key;
        this.samples.push({ t: this.lastStatusSentAt - s.age_ms, nonzero: d.left !== 0 || d.right !== 0, left: d.left,
                            right: d.right });
      }
    }
    var keep = receivedAt - this.cfg.sampleKeepMs;
    while (this.samples.length && this.samples[0].t < keep) this.samples.shift();
    // calibration scale per side from the bridge's own conversion (|pos| >= 100 permille: rounding < 0.5 %)
    if (isNum(s.measured_mrad) && isNum(s.measured_permille) && Math.abs(s.measured_permille) >= 100) {
      var k = s.measured_mrad / s.measured_permille;
      if (k > 0) this.scale[s.measured_permille > 0 ? 'left' : 'right'] = k;
    }
    // A lease the bridge revoked (stop, latch, expiry): seen in a status requested after the lease was granted.
    if (this.lease && snap.armed === false && this.leaseAt !== null && this.lastStatusSentAt > this.leaseAt) {
      this._lose(snap.reason ? String(snap.reason) : 'bridge_disarmed', 'status', false);
    }
  };

  SteerController.prototype.pollStatus = function () {
    var self = this;
    if (this.polling || !this.transport) return null;
    this.polling = true;
    var sentAt = this.now();
    return Promise.resolve().then(function () { return self.transport.status(); }).then(function (snap) {
      self.onStatus(snap, self.now(), sentAt);
    }, function (err) {
      self.pollFailures += 1;
      self.lastBridgeError = 'http_failed';
    }).then(function () {
      self.polling = false;
      self._emit();
    });
  };

  // -- inputs
  SteerController.prototype._armedHint = function () {
    if (!this.lease) this._say('warn', '먼저 조종 활성화를 누른다(조향만 바꾸는 입력도 조종권이 필요하다).');
  };
  SteerController.prototype.keyDown = function (key) {
    if (!(key in this.keys) || this.keys[key]) return;
    if (this.releasing) return;
    this.keys[key] = true;
    if (key === 'a' || key === 'd') this.centreReturn = false;
    this._armedHint();
    this.dirty = true;
    this._emit();
  };
  SteerController.prototype.keyUp = function (key) {
    if (!(key in this.keys) || !this.keys[key]) return;
    this.keys[key] = false;
    if ((key === 'a' || key === 'd') && !this.keys.a && !this.keys.d && !this.centreHeld) {
      this._steerReleased(this.now());
    }
    this.dirty = true;
    this._emit();
  };
  SteerController.prototype._steerReleased = function (now) {
    this.stepOrigin = null;
    if (!this.lease) return;
    var moving = this.fresh(now) && this.moving(now);
    if (!this.steerHold && (moving || this.steerMode === 'centre')) {
      this.centreReturn = true;          // centre mode: always; design 4.4: only while moving
    } else if (!moving) {
      this._freeze(now);                 // standing (design mode or steer hold): keep the wheels where they are
    }                                    // moving with steer hold: keep the target
  };
  SteerController.prototype._freeze = function (now) {
    this.centreReturn = false;
    this.stepOrigin = null;
    var m = this.measuredMrad();
    var lim = this.limits();
    if (m !== null && lim && this.fresh(now)) this.hostTarget = clamp(m, -lim.right, lim.left);
    this.dirty = true;
  };
  SteerController.prototype.setCentre = function (held) {
    held = !!held;
    if (held === this.centreHeld || this.releasing) return;
    this.centreHeld = held;
    var now = this.now();
    if (held) {
      this.centreReturn = false;
      this.stepOrigin = null;
      this._armedHint();
    } else {
      this.stepOrigin = null;
      if (this.lease && !this.keys.a && !this.keys.d && !(this.fresh(now) && this.moving(now))) this._freeze(now);
    }
    this.dirty = true;
    this._emit();
  };
  SteerController.prototype.setSteerHold = function (on) { this.steerHold = !!on; this._emit(); };
  SteerController.prototype.setSpeedCap = function (pct) {
    if (!isNum(pct)) return;
    this.speedCapPct = Math.round(clamp(pct, 0, this.cfg.speedCapMaxPct));
    this.dirty = true;
    this._emit();
  };
  SteerController.prototype.nudgeSpeed = function (sign) { this.setSpeedCap(this.speedCapPct + sign * this.cfg.speedStepPct); };
  SteerController.prototype.setAngle = function (deg) {
    var choice = this.angleChoices().filter(function (c) { return c.deg === deg; })[0];
    if (!choice) return false;
    if (!choice.enabled && this.limits()) return false;
    this.angleDeg = deg;
    this.dirty = true;
    this._emit();
    return true;
  };
  SteerController.prototype.blur = function () {
    var self = this;
    ['w', 's', 'a', 'd'].forEach(function (k) { self.keyUp(k); });
    this.setCentre(false);
  };
  SteerController.prototype._clearInputs = function () {
    this.keys = { w: false, s: false, a: false, d: false };
    this.centreHeld = false;
    this.centreReturn = false;
    this.stepOrigin = null;
    this.dirty = false;
  };

  // -- commands
  SteerController.prototype.throttleCommand = function (now) {
    if (!this.lease || this.releasing || !this.fresh(now)) return 0;
    var dir = (this.keys.w ? 1 : 0) - (this.keys.s ? 1 : 0);
    return dir * Math.round(clamp(this.speedCapPct, 0, this.cfg.speedCapMaxPct));
  };
  SteerController.prototype.steerCommand = function () {
    var lim = this.limits();
    if (this.hostTarget === null || !lim) return null;
    return Math.round(clamp(this.hostTarget, -lim.right, lim.left));
  };
  SteerController.prototype.steeringActive = function () {
    return this.keys.a || this.keys.d || this.centreHeld || this.centreReturn;
  };
  SteerController.prototype._updateSteering = function (now) {
    var dtMs = this.lastRampAt === null ? 0 : clamp(now - this.lastRampAt, 0, this.cfg.maxRampDtMs);
    this.lastRampAt = now;
    if (!this.lease || this.hostTarget === null) return;
    var lim = this.limits();
    if (!lim || !this.fresh(now)) return;             // stale: the target stays (no ramp, no centre return)
    var moving = this.moving(now);
    var goal = null, rate = moving ? this.cfg.movingRateDegS : this.cfg.standstillRateDegS;
    var steerKey = this.keys.a || this.keys.d;
    if (this.centreHeld) {
      goal = 0;
      rate = this.cfg.standstillRateDegS;               // design 4.4: centre at the standstill rate cap
    } else if (steerKey) {
      var dir = (this.keys.a ? 1 : 0) - (this.keys.d ? 1 : 0);
      goal = dir === 0 ? this.hostTarget : dir * this.selectedAngleMrad();
    } else if (this.centreReturn) {
      if (moving || this.steerMode === 'centre') {
        goal = 0;                                         // centre mode at standstill: standstill rate and step below
      } else {
        this._freeze(now);                                // design mode, stopped during the return: keep the wheels
        return;
      }
    }
    if (goal === null) { this.stepOrigin = null; return; }
    goal = clamp(goal, -lim.right, lim.left);
    var stepLimited = false;
    if (!moving) {
      if (this.stepOrigin === null) this.stepOrigin = this.hostTarget;
      var step = degToMrad(this.cfg.standstillStepDeg);
      var limited = clamp(goal, this.stepOrigin - step, this.stepOrigin + step);
      stepLimited = limited !== goal;
      goal = limited;
    } else {
      this.stepOrigin = null;
    }
    var maxStep = degToMrad(rate) * dtMs / 1000;
    var next = this.hostTarget + clamp(goal - this.hostTarget, -maxStep, maxStep);
    this.hostTarget = clamp(next, -lim.right, lim.left);
    if (this.centreReturn && Math.abs(this.hostTarget) < 0.5) {
      this.hostTarget = 0;
      this.centreReturn = false;
      this.dirty = true;
    } else if (this.centreReturn && stepLimited && Math.abs(this.hostTarget - goal) < 0.5) {
      // centre mode at standstill: one standstill step per release; the rest needs the centre button or driving
      this.centreReturn = false;
      this.stepOrigin = null;
      this.dirty = true;
    }
  };
  SteerController.prototype.setSteerMode = function (mode) {
    if (mode !== 'centre' && mode !== 'design') return false;
    this.steerMode = mode;
    this._emit();
    return true;
  };

  // One tick (the caller calls it every statusPeriodMs, 50 ms). Returns the request promise it started, or null.
  SteerController.prototype.tick = function () {
    var now = this.now();
    if (this.lease) {
      if (this.fresh(now)) this.staleSince = null;
      else if (this.staleSince === null) this.staleSince = now;
      if (this.staleSince !== null && now - this.staleSince >= this.cfg.staleStopMs) {
        return this.stop('page_status_stale');
      }
    } else {
      this.staleSince = null;
    }
    this._updateSteering(now);
    if (!this.lease || this.inFlight || this.arming) return null;
    if (this.releasing) return this._releaseStep(now);
    var throttle = this.throttleCommand(now);
    var steer = this.steerCommand();
    var changed = steer !== null && steer !== this.lastSent.steer;
    var active = throttle !== 0 || this.steeringActive() || changed || this.dirty;
    var since = this.lastSent.at === null ? Infinity : now - this.lastSent.at;
    // key events (dirty) go at once; ramps and held keys at commandPeriodMs; idle keepalive at keepalivePeriodMs
    var due = active ? (this.dirty || since >= this.cfg.commandPeriodMs) : since >= this.cfg.keepalivePeriodMs;
    if (!due) return null;
    if (active && steer !== null) {
      return this._send('command', { lease: this.lease, ticket: this.ticket, seq: this.seq + 1, deadman: true,
                                     throttle: throttle, steer_mrad: steer });
    }
    return this._send('keepalive', { lease: this.lease, ticket: this.ticket, seq: this.seq + 1 });
  };

  SteerController.prototype._send = function (op, body) {
    var self = this;
    var epoch = this.epoch;
    var lease = this.lease;
    var sentAt = this.now();
    this.inFlight = op;
    this.dirty = false;
    this.lastSent = { at: sentAt, op: op, throttle: op === 'command' ? body.throttle : (op === 'release' ? 0 : this.lastSent.throttle),
                      steer: op === 'command' ? body.steer_mrad : this.lastSent.steer };
    this._record(op, body);
    return Promise.resolve().then(function () { return self.transport.post('/api/rover/' + op, body); }).then(
      function (reply) { return reply; },
      function (err) { return { ok: false, error: 'http_failed', detail: String(err && err.message || err) }; }
    ).then(function (reply) {
      if (self.inFlight === op) self.inFlight = null;
      if (epoch !== self.epoch || self.lease !== lease) return;     // a STOP or a lost lease came first
      if (!reply || typeof reply !== 'object' || reply.ok !== true || typeof reply.ticket !== 'string') {
        var code = reply && reply.error ? String(reply.error) : 'bad_reply';
        if (code === 'request_expired') {          // not dispatched, nothing stopped: send again
          self.dirty = true;
          self._say('warn', messageFor(code));
          return;
        }
        self._lose(code, op, true);
        return;
      }
      self.ticket = reply.ticket;
      self.seq = reply.seq;
      var at = self.now();
      if (op === 'command') self.inputDeadlineAt = at + self.cfg.inputTtlMs;
      if (op === 'release') self.releaseReplyAt = at;
      if (reply.status) self.onStatus(reply.status, at, sentAt);
    }).then(function () { self._emit(); });
  };

  // Lost the lease (an error reply, or the bridge revoked it). stopToo: also send STOP (the old screen does the same
  // after a failed command); never re-arm.
  SteerController.prototype._lose = function (code, where, stopToo) {
    this.epoch += 1;
    this.lease = this.ticket = null;
    this.leaseAt = null;
    this.inFlight = null;
    this.releasing = false;
    this._clearInputs();
    var text = '조종권 끝: ' + messageFor(code);
    this._say('bad', text + ' · 다시 조종 활성화를 해야 한다.');
    this.lastLoss = { code: code, where: where, at: this.now() };
    if (stopToo) this._postStop('page_request_failed', text);
    this._emit();
  };

  SteerController.prototype.arm = function () {
    var self = this;
    if (this.lease || this.arming || this.releasing || !this.transport) return null;
    this.arming = true;
    var epoch = this.epoch;
    var body = { vehicle_profile: VEHICLE_PROFILE };
    this._record('arm', body);
    this._say('info', '조종 활성화 요청 중');
    this._emit();
    return Promise.resolve().then(function () { return self.transport.post('/api/rover/arm', body); }).then(
      function (reply) { return reply; },
      function (err) { return { ok: false, error: 'http_failed' }; }
    ).then(function (reply) {
      self.arming = false;
      if (epoch !== self.epoch) {
        // a STOP came while the arm was on its way: a lease granted after it must not stay
        if (reply && reply.ok === true) self._postStop('cancelled_arm');
        return;
      }
      if (!reply || reply.ok !== true || typeof reply.lease !== 'string' || typeof reply.ticket !== 'string') {
        self._say('bad', '조종 활성화 거절: ' + messageFor(reply && reply.error ? reply.error : 'bad_reply'));
        return;
      }
      var at = self.now();
      self.lease = reply.lease;
      self.ticket = reply.ticket;
      self.seq = reply.seq;
      self.leaseAt = at;
      self.releasing = false;
      self.lastSent = { at: at, op: 'arm', throttle: 0, steer: null };
      self.inputDeadlineAt = null;
      self._clearInputs();
      if (reply.status) self.onStatus(reply.status, at, at);
      var m = self.measuredMrad();
      var lim = self.limits();
      self.hostTarget = m !== null && lim ? clamp(m, -lim.right, lim.left) : null;
      self._say('ok', '조종 활성화: W/S 구동, A/D 조향, C 가운데(누르는 동안), Esc = STOP');
    }).then(function () { self._emit(); });
  };

  SteerController.prototype.release = function () {
    if (!this.lease || this.releasing) return;
    this._clearInputs();
    this.releasing = true;
    this.releaseSent = false;
    this.releaseStartAt = this.now();
    this.releaseReplyAt = null;
    this._say('info', 'Release: 감속 뒤 해제');
    this._emit();
  };
  SteerController.prototype._releaseStep = function (now) {
    if (!this.releaseSent) {
      this.releaseSent = true;
      return this._send('release', { lease: this.lease, ticket: this.ticket, seq: this.seq + 1 });
    }
    var d = this.drive();
    var stopped = this.releaseReplyAt !== null && this.lastStatusAt !== null && this.lastStatusAt >= this.releaseReplyAt
      && this.fresh(now) && d && d.left === 0 && d.right === 0;
    if (stopped || now - this.releaseStartAt >= this.cfg.releaseTimeoutMs) return this.stop('operator_release');
    if (this.lastSent.at !== null && now - this.lastSent.at >= this.cfg.keepalivePeriodMs) {
      return this._send('keepalive', { lease: this.lease, ticket: this.ticket, seq: this.seq + 1 });
    }
    return null;
  };

  // STOP: always available (no lease needed). Drops the local lease at once and posts /api/rover/stop.
  SteerController.prototype.stop = function (reason) {
    var why = reason || 'operator_stop';
    this.epoch += 1;
    this.lease = this.ticket = null;
    this.leaseAt = null;
    this.inFlight = null;
    this.arming = false;
    this.releasing = false;
    this._clearInputs();
    this.lastStop = { reason: why, at: this.now() };
    this._say('bad', 'STOP 요청: ' + messageFor(why));
    this._emit();
    return this._postStop(why);
  };
  // Another control (the status bar STOP, Esc of the drive page) has sent the bridge STOP: drop the lease here at once
  // without a second STOP request.
  SteerController.prototype.abandon = function (reason) {
    var why = reason || 'operator_stop';
    this.epoch += 1;
    this.lease = this.ticket = null;
    this.leaseAt = null;
    this.inFlight = null;
    this.arming = false;
    this.releasing = false;
    this._clearInputs();
    this.lastStop = { reason: why, at: this.now() };
    this._say('bad', 'STOP 요청: ' + messageFor(why) + '. 다시 움직이려면 조종 활성화.');
    this._emit();
  };

  // What the last status says about the device: 'present' (a front-steer bridge with a steering block), 'none' (a bridge
  // answers but it is not a steering bridge: the V2 differential bridge), 'unavailable' (no bridge, no answer) or
  // 'unknown' (nothing received yet).
  SteerController.prototype.device = function () {
    var now = this.now();
    if (this.lastBridgeError && (this.lastStatusAt === null || now - this.lastStatusAt > this.cfg.staleStopMs)) {
      return 'unavailable';                // the latest replies are errors (no socket, HTTP failure) for > 1 s
    }
    if (!this.lastStatus) return 'unknown';
    var s = this.lastStatus;
    return s.vehicle_profile === VEHICLE_PROFILE && s.steering && typeof s.steering === 'object' ? 'present' : 'none';
  };

  // prefix: the text of a lost lease, kept in front of the STOP result.
  SteerController.prototype._postStop = function (reason, prefix) {
    var self = this;
    if (!this.transport) return null;
    var body = { reason: reason };
    var attempt = 0;
    var head = prefix ? prefix + ' · ' : '';
    function once() {
      attempt += 1;
      self._record('stop', body);
      return Promise.resolve().then(function () { return self.transport.post('/api/rover/stop', body); }).then(
        function (reply) {
          if (reply && reply.ok === true) {
            self.stopReplies += 1;
            if (prefix) self._say('bad', head + 'STOP 접수 · 다시 조종 활성화를 해야 한다.');
            else self._say('ok', 'STOP 접수(' + messageFor(reason) + '). 다시 움직이려면 조종 활성화.');
            self._emit();
            return reply;
          }
          throw new Error(reply && reply.error ? reply.error : 'stop_failed');
        }
      ).catch(function (err) {
        if (attempt >= self.cfg.stopRetries) {
          self._say('bad', head + 'STOP 응답 없음: 물리 비상정지를 쓰세요 (' + String(err && err.message || err) + ')');
          self._emit();
          return null;
        }
        return self.sleep(self.cfg.stopRetryMs).then(once);
      });
    }
    return once();
  };

  // key mapping (physical key code)
  function keyFromEvent(event) {
    var code = event.code || '';
    var m = /^Key([WASDCQEH])$/.exec(code);
    var key = m ? m[1].toLowerCase() : String(event.key || '').toLowerCase();
    return ['w', 'a', 's', 'd', 'c', 'q', 'e', 'h'].indexOf(key) >= 0 ? key : '';
  }

  var api = {
    DEFAULTS: DEFAULTS, VEHICLE_PROFILE: VEHICLE_PROFILE, MCU_FAULTS: MCU_FAULTS, HOST_LATCHES: HOST_LATCHES,
    MESSAGES: MESSAGES, SteerController: SteerController, mcuDrive: mcuDrive, mradFromPermille: mradFromPermille,
    permilleFromMrad: permilleFromMrad, degToMrad: degToMrad, mradToDeg: mradToDeg, roundHalfAway: roundHalfAway,
    messageFor: messageFor, keyFromEvent: keyFromEvent,
  };
  root.UbtsSteerRules = api;
})(typeof globalThis !== 'undefined' ? globalThis : this);
