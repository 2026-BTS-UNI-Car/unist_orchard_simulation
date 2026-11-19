// Manual drive and steering panel of the front-steer vehicle (UI-07, docs/STEERING_SERVER_KO.md 3 and 4).
// Loaded only on a front-steer dashboard (html data-vehicle-kind="front_steer") by static/app/14-steer.js.
// The rules (arm, lease, ticket, seq, rate caps, release-to-centre, stale status, STOP retries) live in
// steer-control-rules.js; this module binds them to the page: buttons (pointer capture, two fingers), keys, the gauge,
// the readout and the status line. Requests go through the page's own postJson/getJson (the dashboard routes
// /api/rover/*), the same functions as the differential panel. The STOP buttons of the page keep their own code
// (stopRoverNow: one /api/rover/stop); here they only drop the steering lease at once.
// The drag control (2026-11-16, bindSteerDrag): a precise angle while dragging, back to 0 when released; with an
// absolute-target bridge (the ESP32-S3 'ABS-HOLD' firmware) A/D go to the full limit and the angle choices are hidden.
// SMOOTH firmware (2026-11-16 evening, status steer_control.smooth): the same full-limit A/D is one continuous run of the
// steering motor; the drag goes in 0.25 deg steps. The readout names the bridge's drive cap (drive.max_pwm).
import './steer-control-rules.js';

const R = globalThis.UbtsSteerRules;
const DEFAULT_LIMIT_MRAD = 409;          // placeholder host profile v0 (23.46 deg) until a status gives the limits
const ARMED_STATUS_MS = 50;              // armed: one status request at a time, as often as the link answers
const IDLE_STATUS_MS = 500;              // not armed: enough for the gauge and the device check
const deg = (mrad) => (typeof mrad === 'number' && isFinite(mrad) ? R.mradToDeg(mrad) : null);
const fmtDeg = (value) => (value === null ? '--' : `${value >= 0 ? '' : '−'}${Math.abs(value).toFixed(1)}°`);
// Drag value (deg, left positive as steer_mrad) as the operator reads it.
export const dragText = (deg) => (!deg ? '0°' : `${deg > 0 ? '왼쪽' : '오른쪽'} ${Math.abs(deg).toFixed(1)}°`);
// One short line under the pad (2026-11-19: "글씨도 쓸데없는 안내가 너무 많고"; 2026-11-20 shorter again); the full rules
// are in Docs.
export const HELP_TEXT = {
  ramp: 'W/S 구동 · A/D 고른 각까지 조향 · C 가운데 · Esc 정지',
  absolute: 'W/S 구동 · A/D 끝까지 조향 · C 가운데 · Esc 정지',
  smooth: 'W/S 구동 · A/D 끝까지 끊김 없이 조향 · C 가운데 · Esc 정지',
};
// The drive readout's second line: what the MCU reports and the bridge's own drive cap (its install flag).
export function driveCapText(cap) {
  if (typeof cap !== 'number' || !isFinite(cap)) return 'MCU 보고 값 · 속도는 아님';
  if (cap <= 0) return '구동 상한이 0이라 W/S가 꺼져 있습니다.';
  return `MCU 보고 값 · 브리지 상한 PWM ${cap}`;
}

// The device chip (short) and, while not armed, the explanation in the status line (controller.device()).
export const DEVICE_TEXT = {
  present: ['ok', '조향 장치 연결됨', ''],
  none: ['warn', '조향 장치 없음', '조종 브리지가 조향 상태를 보내지 않아 조종 활성화를 할 수 없습니다. 차동 구동용 브리지이거나 조향 펌웨어가 없을 때 이렇게 됩니다.'],
  unavailable: ['bad', '연결 안 됨', '조종 브리지에 연결되지 않습니다. STOP은 계속 보냅니다.'],
  unknown: ['', '확인 중', ''],
};

const AD_RATE_KEY = 'ubts-steer-ad-rate';

// '화면 초기화' (2026-11-18, a phone that never armed again while a fresh browser did): STOP first, then forget the
// server session and everything this page stored in the browser, then load the page again.
export async function resetScreen(ctl, page, env = {}) {
  const ask = env.confirm || ((text) => globalThis.confirm(text));
  if (!ask('이 브라우저에 저장된 화면 설정과 세션을 지우고 페이지를 다시 불러올까요? 조종 중이면 먼저 STOP을 보냅니다.')) return false;
  if (ctl.lease) await Promise.resolve(ctl.stop('screen_reset')).catch(() => null);
  await Promise.resolve((page.controlRequest || page.postJson)('/api/session/reset', {})).catch(() => null);
  for (const store of [env.localStorage || globalThis.localStorage, env.sessionStorage || globalThis.sessionStorage]) {
    try { store?.clear(); } catch (_) { /* nothing stored */ }
  }
  (env.reload || (() => globalThis.location.reload()))();
  return true;
}
// The A/D steering speed as the operator reads it: the top of the range means the wheel's own speed (no ramp).
export const adRateText = (degS, max) => (degS >= max ? '최대' : `${degS} °/s`);

// Why the bridge would refuse Arm now (status steering_host.not_ready of the ESP32-S3 bridge), or null.
export function boardNotReady(status) {
  const host = status && typeof status === 'object' ? status.steering_host : null;
  return host && typeof host === 'object' && typeof host.not_ready === 'string' && host.not_ready ? host.not_ready : null;
}
export function armBlockText(code) {
  const why = {
    pot_invalid: '가변저항 값이 범위 밖입니다. 배선을 확인하면 저절로 다시 준비됩니다.',
    device_unavailable: '보드에 다시 연결하고 있습니다. 전원과 USB를 확인하세요. 보드가 연결되면 저절로 준비됩니다.',
    esp32_not_ready: '보드에 다시 연결하고 있습니다. 보드의 시작 응답과 STOP 확인을 기다립니다.',
    steering_status_stale: '조향 상태가 들어오지 않습니다. 보드와 USB를 확인하세요. 다시 들어오면 저절로 준비됩니다.',
    firmware_mismatch: '펌웨어 이름이 다릅니다. 보드의 펌웨어를 확인하세요.',
    stop_unconfirmed: 'STOP 확인을 받지 못해 보드에 다시 연결하고 있습니다.',
  };
  return `지금은 조종 활성화를 할 수 없습니다. ${why[code] || R.messageFor?.(code) || code}`;
}

// Short labels of the faults in the status (MCU bits and host latches), from the rules' catalogues.
export function faultLabels(names) {
  const table = new Map(R.MCU_FAULTS.map(([code, label, level]) => [code, [label, level]]));
  for (const [code, label] of R.HOST_LATCHES) table.set(code, [label, 'bad']);
  for (const [code, label, level] of R.ESP32_FAULTS || []) table.set(code, [label, level]);
  return (Array.isArray(names) ? names : []).map((code) => {
    const known = table.get(code);
    return { code: String(code), label: known ? known[0] : '기타 결함', level: known ? known[1] : 'bad' };
  });
}

// The gauge and readout values for one moment (pure: tested without a page).
export function steerView(ctl, now) {
  const limits = ctl.limits() || { left: DEFAULT_LIMIT_MRAD, right: DEFAULT_LIMIT_MRAD };
  const steering = ctl.steering();
  const marks = ctl.marksForConversion();
  const sent = ctl.lastSent && typeof ctl.lastSent.steer === 'number' ? ctl.lastSent.steer : null;
  const target = ctl.lease ? (ctl.steerCommand() ?? sent) : sent;
  const measured = ctl.measuredMrad();
  const mcuTarget = marks ? R.mradFromPermille(steering.target_permille, marks) : null;
  const drive = ctl.drive();
  const fresh = ctl.fresh(now);
  // left steering is positive (mrad > 0) and drawn left of the centre, as the operator sees the wheels
  const span = (mrad) => (mrad === null ? null : Math.max(0, Math.min(100, 50 - 50 * mrad / (mrad >= 0 ? limits.left : limits.right))));
  const deadline = ctl.lease && ctl.inputDeadlineAt !== null ? Math.max(0, ctl.inputDeadlineAt - now) : null;
  return {
    device: ctl.device(), fresh, armed: Boolean(ctl.lease), limits,
    targetDeg: deg(target), measuredDeg: deg(measured), mcuTargetDeg: deg(mcuTarget),
    errorDeg: target !== null && measured !== null ? deg(target - measured) : null,
    targetPos: span(target), measuredPos: span(measured),
    duty: drive ? { left: drive.left, right: drive.right, en: drive.en } : null,
    deadlineMs: deadline, saturated: steering.saturated === true, faults: faultLabels(steering.faults),
    // 2026-11-19 (user): the steering potentiometer on the manual tab: raw ADC count (-1 / absent = invalid) and the
    // steering motor PWM the bridge reads from the firmware trace (16f9d5c)
    potRaw: Number.isFinite(steering.raw) && steering.raw >= 0 ? steering.raw : null,
    potInvalid: Array.isArray(steering.faults) && steering.faults.includes('pot_invalid'),
    steerPwm: Number.isFinite(steering.pwm) ? steering.pwm : null,
    steerPwmPeak: Number.isFinite(steering.pwm_peak) ? steering.pwm_peak : null,
    centreOff: target !== null && Math.abs(target) >= 0.5 && !ctl.centreReturn && !ctl.steeringActive(),
    absolute: ctl.absoluteMode(), smooth: ctl.smoothMode(), dragStepDeg: ctl.dragStepDeg(), driveCapPwm: ctl.driveCapPwm(),
    dragActive: ctl.drag.active, dragDeg: ctl.drag.active ? ctl.drag.deg : 0,
    dragPos: dragPosition(ctl, ctl.drag.active ? ctl.drag.deg : 0),
  };
}

// Thumb position (% from the left edge) of a drag value: left steering left of the centre, per-side limits.
export function dragPosition(ctl, deg) {
  const lim = ctl.dragLimitDeg();
  const side = deg >= 0 ? lim.left : lim.right;
  return Math.max(0, Math.min(100, 50 - 50 * (side > 0 ? deg / side : 0)));
}

// The drag control. One pointer at a time with pointer capture; the value comes from the pointer's x (left of the
// centre = left steering). Released by pointer up / cancel / lost capture and touch end / cancel; the page also calls
// end() on blur and when the page is hidden. Uses only addEventListener, getBoundingClientRect and pointer capture, so
// a test can drive it with a fake element.
export function bindSteerDrag(el, ctl) {
  let pointer = null;
  const valueAt = (clientX) => {
    const r = el.getBoundingClientRect();
    const half = r.width / 2;
    if (!(half > 0) || typeof clientX !== 'number') return 0;
    const x = Math.max(-1, Math.min(1, (clientX - (r.left + half)) / half));
    const lim = ctl.dragLimitDeg();
    return x <= 0 ? -x * lim.left : -x * lim.right;
  };
  const end = (reason) => {
    const id = pointer;
    pointer = null;
    if (id !== null) {
      try { el.releasePointerCapture?.(id); } catch (_) { /* already released */ }
    }
    ctl.dragEnd(reason);
  };
  el.addEventListener('pointerdown', (event) => {
    event.preventDefault?.();
    if (pointer !== null) return;
    if (!ctl.dragStart(valueAt(event.clientX))) return;
    pointer = event.pointerId;
    try { el.setPointerCapture?.(event.pointerId); } catch (_) { /* gone already: its cancel ends the drag */ }
  });
  el.addEventListener('pointermove', (event) => {
    if (event.pointerId === pointer) ctl.dragMove(valueAt(event.clientX));
  });
  for (const name of ['pointerup', 'pointercancel', 'lostpointercapture']) {
    el.addEventListener(name, (event) => { if (event.pointerId === pointer) end(name); });
  }
  for (const name of ['touchend', 'touchcancel']) {
    el.addEventListener(name, () => { if (pointer !== null) end(name); });
  }
  el.addEventListener('contextmenu', (event) => event.preventDefault?.());
  return { end: (reason) => end(reason), held: () => pointer !== null };
}

export function initSteerDrive(page) {
  const $ = (id) => document.getElementById(id);
  const root = $('steerDrive');
  if (!root || !R) return null;
  const ctl = new R.SteerController({
    transport: {
      status: () => page.getJson('/api/rover/status'),
      // Driving requests use the page's control transport (NET-05) where there is one.
      post: (path, body) => (page.controlRequest || page.postJson)(path, body),
    },
    onChange: () => schedule(),
  });
  let timer = null, lastPollAt = 0, drawQueued = false, wasArmed = false;
  const active = () => page.currentRoute() === '/rover-control' && !document.hidden;

  function loop() {
    if (!active()) {
      if (ctl.lease) ctl.stop(document.hidden ? 'page_hidden' : 'page_inactive');
      stopLoop();
      return;
    }
    const now = Date.now();
    if (now - lastPollAt >= (ctl.lease ? ARMED_STATUS_MS : IDLE_STATUS_MS) && !ctl.polling) {
      lastPollAt = now;
      ctl.pollStatus();
    }
    ctl.tick();
  }
  function startLoop() { if (!timer && active()) { timer = setInterval(loop, ctl.cfg.statusPeriodMs); loop(); } }
  function stopLoop() { clearInterval(timer); timer = null; }

  // ---- inputs -----------------------------------------------------------------------------------------------------
  const press = { w: (on) => (on ? ctl.keyDown('w') : ctl.keyUp('w')), s: (on) => (on ? ctl.keyDown('s') : ctl.keyUp('s')),
                  a: (on) => (on ? ctl.keyDown('a') : ctl.keyUp('a')), d: (on) => (on ? ctl.keyDown('d') : ctl.keyUp('d')),
                  c: (on) => ctl.setCentre(on) };
  root.querySelectorAll('[data-steer-key]').forEach((button) => {
    const key = button.dataset.steerKey;
    let pointer = null;
    const end = () => { if (pointer !== null) { pointer = null; press[key](false); } };
    button.addEventListener('pointerdown', (event) => {
      event.preventDefault();
      if (pointer !== null) return;
      pointer = event.pointerId;
      // Every finger keeps its own button: throttle and steering can be held together.
      try { button.setPointerCapture(event.pointerId); } catch (_) { /* gone already: its cancel ends the input */ }
      press[key](true);
    });
    for (const name of ['pointerup', 'pointercancel', 'lostpointercapture']) {
      button.addEventListener(name, (event) => { if (event.pointerId === pointer) end(); });
    }
    button.addEventListener('contextmenu', (event) => event.preventDefault());
  });
  $('steerArmButton')?.addEventListener('click', () => (ctl.lease ? ctl.release() : ctl.arm()));
  $('screenResetButton')?.addEventListener('click', () => resetScreen(ctl, page));
  const dragEl = $('steerDrag');
  const drag = dragEl ? bindSteerDrag(dragEl, ctl) : null;
  const slider = $('steerSpeedSlider');
  slider?.addEventListener('input', () => ctl.setSpeedCap(Number(slider.value)));
  // A/D steering speed (2026-11-18): kept in this browser.
  const adRate = $('steerAdRate');
  try { const kept = Number(localStorage.getItem(AD_RATE_KEY)); if (kept) ctl.setAdRate(kept); } catch (_) { /* no storage */ }
  adRate?.addEventListener('input', () => {
    ctl.setAdRate(Number(adRate.value));
    try { localStorage.setItem(AD_RATE_KEY, String(ctl.adRateDegS)); } catch (_) { /* this load only */ }
  });
  root.querySelectorAll('[data-steer-angle]').forEach((button) => {
    button.addEventListener('click', () => ctl.setAngle(Number(button.dataset.steerAngle)));
  });
  root.querySelectorAll('[data-steer-mode]').forEach((button) => {
    button.addEventListener('click', () => {
      const hold = button.dataset.steerMode === 'hold';
      ctl.setSteerHold(hold);
      if (!hold) ctl.setSteerMode('centre');
    });
  });
  const typing = (event) => ['INPUT', 'TEXTAREA', 'SELECT'].includes(event.target?.tagName) && event.target?.type !== 'range';
  document.addEventListener('keydown', (event) => {
    if (page.currentRoute() !== '/rover-control') return;
    if (event.key === 'Escape') { ctl.abandon('escape'); return; }   // the page's Esc handler sends the STOP
    if (typing(event) || event.ctrlKey || event.altKey || event.metaKey) return;
    const key = R.keyFromEvent(event);
    if (!key) return;
    event.preventDefault();
    if (event.repeat) return;
    if (key === 'q' || key === 'e') ctl.nudgeSpeed(key === 'q' ? -1 : 1);
    else if (key === 'h') ctl.setSteerHold(!ctl.steerHold);
    else press[key](true);
  });
  document.addEventListener('keyup', (event) => {
    const key = R.keyFromEvent(event);
    if (key && press[key]) press[key](false);
  });
  window.addEventListener('blur', () => { drag?.end('blur'); ctl.blur(); });
  document.addEventListener('visibilitychange', () => {
    if (document.hidden) { drag?.end('page_hidden'); if (ctl.lease) ctl.stop('page_hidden'); stopLoop(); } else startLoop();
  });
  for (const id of ['frameStopButton', 'roverStopButton', 'autoStopButton']) {
    $(id)?.addEventListener('click', () => ctl.abandon('operator_stop'));   // their own handler sends the STOP
  }
  // Leaving the 수동 조종 tab (2026-11-19) with the Arm held: the pad is no longer on screen, so STOP as a hidden page does.
  window.addEventListener('ubts:rover-tab', (event) => {
    if (event.detail?.before !== 'manual' || event.detail?.tab === 'manual') return;
    drag?.end('tab_changed');
    if (ctl.lease) ctl.stop('tab_changed');
  });
  window.addEventListener('pagehide', () => {
    if (!ctl.lease) return;
    const body = new Blob([JSON.stringify({ reason: 'pagehide' })], { type: 'application/json' });
    try { navigator.sendBeacon('/api/rover/stop', body); } catch (_) { /* the bridge input deadline stops it */ }
  });

  // ---- drawing ----------------------------------------------------------------------------------------------------
  function schedule() {
    if (drawQueued) return;
    drawQueued = true;
    requestAnimationFrame(() => { drawQueued = false; draw(); });
  }
  function setText(id, value) { const el = $(id); if (el && el.textContent !== value) el.textContent = value; }
  function draw() {
    const now = Date.now();
    const v = steerView(ctl, now);
    let [level, text, explain] = DEVICE_TEXT[v.device] || DEVICE_TEXT.unknown;
    const notReady = boardNotReady(ctl.lastStatus);
    if (v.device === 'present' && notReady) {
      [level, text, explain] = ['warn', notReady === 'pot_invalid' ? '가변저항 확인' : '보드 준비 중', armBlockText(notReady)];
    }
    const device = $('steerDevice');
    if (device) { device.className = `status-chip${level ? ` ${level}` : ''}`; setText('steerDevice', text); }
    const arm = $('steerArmButton');
    if (arm) {
      arm.textContent = ctl.arming ? '활성화 중…' : ctl.releasing ? '감속 뒤 해제 중…' : ctl.lease ? '조종 활성화 ON' : '조종 활성화 OFF';
      arm.setAttribute('aria-pressed', String(Boolean(ctl.lease)));
      arm.disabled = ctl.arming || ctl.releasing || (!ctl.lease && (v.device !== 'present' || Boolean(notReady)));
      arm.title = !ctl.lease && notReady ? armBlockText(notReady) : '';
    }
    const gauge = (id, pos) => { const el = $(id); if (el) { el.hidden = pos === null; if (pos !== null) el.style.left = `${pos}%`; } };
    gauge('steerGaugeTarget', v.targetPos);
    gauge('steerGaugeMeasured', v.measuredPos);
    setText('steerGaugeLeft', `왼쪽 ${fmtDeg(deg(v.limits.left))}`);
    setText('steerGaugeRight', `오른쪽 ${fmtDeg(deg(v.limits.right))}`);
    const colours = page.chartColors();
    page.renderReadout('steerReadout', [
      { label: '목표 / 측정', value: `${fmtDeg(v.targetDeg)} / ${fmtDeg(v.measuredDeg)}`, sub: v.fresh ? `오차 ${fmtDeg(v.errorDeg)}` : '상태 오래됨', color: colours.cyan },
      { label: '구동 출력', value: v.duty ? `${v.duty.left} / ${v.duty.right}` : '--', sub: driveCapText(v.driveCapPwm), color: colours.blue },
      { label: '입력 기한', value: v.deadlineMs === null ? '--' : `${Math.round(v.deadlineMs)} ms`, sub: v.saturated ? '조향 포화' : '0.75 s · Esc 즉시 정지', color: colours.good },
      { label: '가변저항', value: v.potRaw === null ? (v.potInvalid ? '값 이상' : '--') : String(v.potRaw),
        sub: v.steerPwm === null ? '조향 PWM --' : `조향 PWM ${v.steerPwm}${v.steerPwmPeak ? ` · 최근 최대 ${v.steerPwmPeak}` : ''}`,
        color: v.potInvalid ? colours.bad || colours.cyan : colours.blue },
    ]);
    setText('steerSpeedValue', `${ctl.speedCapPct}%`);
    setText('steerAdRateValue', adRateText(ctl.adRateDegS, ctl.cfg.adRateMaxDegS));
    if (adRate && document.activeElement !== adRate && Number(adRate.value) !== ctl.adRateDegS) adRate.value = String(ctl.adRateDegS);
    if (slider && document.activeElement !== slider && Number(slider.value) !== ctl.speedCapPct) slider.value = String(ctl.speedCapPct);
    if (dragEl) {
      const thumb = $('steerDragThumb');
      if (thumb) thumb.style.left = `${v.dragPos}%`;
      dragEl.classList.toggle('active', v.dragActive);
      dragEl.setAttribute('aria-disabled', String(!ctl.lease));
      // the slider value reads left to right: -20 (left) .. +20 (right)
      const now = v.dragDeg ? -v.dragDeg : 0;
      dragEl.setAttribute('aria-valuenow', now.toFixed(1));
      dragEl.setAttribute('aria-valuetext', v.dragDeg ? dragText(v.dragDeg) : '가운데 0°');
      setText('steerDragValue', v.dragActive ? dragText(v.dragDeg) : '0°');
    }
    const choices = $('steerAngleChoices');
    if (choices) choices.hidden = v.absolute;
    setText('steerHelp', v.smooth ? HELP_TEXT.smooth : v.absolute ? HELP_TEXT.absolute : HELP_TEXT.ramp);
    for (const choice of ctl.angleChoices()) {
      const button = root.querySelector(`[data-steer-angle="${choice.deg}"]`);
      if (!button) continue;
      button.classList.toggle('active', choice.deg === ctl.angleDeg);
      button.disabled = !choice.enabled && ctl.limits() !== null;
    }
    root.querySelectorAll('[data-steer-mode]').forEach((button) => {
      button.classList.toggle('active', (button.dataset.steerMode === 'hold') === ctl.steerHold);
    });
    root.querySelectorAll('[data-steer-key]').forEach((button) => {
      const key = button.dataset.steerKey;
      button.classList.toggle('active', key === 'c' ? ctl.centreHeld : Boolean(ctl.keys[key]));
    });
    const faults = $('steerFaults');
    const faultKey = v.faults.map((f) => `${f.level}:${f.label}`).join('|');
    if (faults && faults.dataset.key !== faultKey) {
      faults.dataset.key = faultKey;
      faults.replaceChildren(...v.faults.map((f) => {
        const chip = document.createElement('span');
        chip.className = `status-chip ${f.level}`;
        chip.textContent = f.label;
        chip.title = f.code;
        return chip;
      }));
      faults.hidden = v.faults.length === 0;
    }
    const status = $('steerCommandStatus');
    if (status) {
      const idleExplain = !ctl.lease && !ctl.lastStop && !ctl.lastLoss && explain;
      const kind = idleExplain ? level : ctl.message.kind === 'info' ? 'testing' : ctl.message.kind;
      const textValue = idleExplain ? explain
        : v.centreOff && ctl.lease && ctl.message.kind === 'ok' ? `${ctl.message.text} 바퀴가 가운데에 있지 않습니다. 가운데 버튼이나 C를 누르세요.` : ctl.message.text;
      status.className = `test-result ${kind}`;
      setText('steerCommandStatus', textValue);
    }
    if (Boolean(ctl.lease) !== wasArmed) {
      wasArmed = Boolean(ctl.lease);
      const chip = $('frameDriveState');
      if (chip && wasArmed) { chip.className = 'status-chip warn'; chip.textContent = '조종 활성'; }
      else if (chip && !ctl.lastStop) { chip.className = 'status-chip'; chip.textContent = '조종 비활성'; }
    }
  }

  function routeChanged() {
    if (active()) { startLoop(); schedule(); } else if (timer) loop();
  }
  routeChanged();
  return { controller: ctl, routeChanged, draw };
}
