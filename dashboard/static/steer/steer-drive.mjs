// Manual drive and steering panel of the front-steer vehicle (UI-07, docs/STEERING_SERVER_KO.md 3 and 4).
// Loaded only on a front-steer dashboard (html data-vehicle-kind="front_steer") by static/app/14-steer.js.
// The rules (arm, lease, ticket, seq, rate caps, release-to-centre, stale status, STOP retries) live in
// steer-control-rules.js; this module binds them to the page: buttons (pointer capture, two fingers), keys, the gauge,
// the readout and the status line. Requests go through the page's own postJson/getJson (the dashboard routes
// /api/rover/*), the same functions as the differential panel. The STOP buttons of the page keep their own code
// (stopRoverNow: one /api/rover/stop); here they only drop the steering lease at once.
// The drag control (2026-11-16, bindSteerDrag): a precise angle while dragging, back to 0 when released; with an
// absolute-target bridge (the ESP32-S3 'ABS-HOLD' firmware) A/D go to the full limit and the angle choices are hidden.
import './steer-control-rules.js';

const R = globalThis.UbtsSteerRules;
const DEFAULT_LIMIT_MRAD = 409;          // placeholder host profile v0 (23.46 deg) until a status gives the limits
const ARMED_STATUS_MS = 50;              // armed: one status request at a time, as often as the link answers
const IDLE_STATUS_MS = 500;              // not armed: enough for the gauge and the device check
const deg = (mrad) => (typeof mrad === 'number' && isFinite(mrad) ? R.mradToDeg(mrad) : null);
const fmtDeg = (value) => (value === null ? '--' : `${value >= 0 ? '' : '−'}${Math.abs(value).toFixed(1)}°`);
// Drag value (deg, left positive as steer_mrad) as the operator reads it.
export const dragText = (deg) => (!deg ? '0°' : `${deg > 0 ? '왼쪽' : '오른쪽'} ${Math.abs(deg).toFixed(1)}°`);
export const HELP_TEXT = {
  ramp: '조종 활성화 뒤 W/S는 누르는 동안 구동, A/D는 고른 각까지 조향, C는 누르는 동안 가운데로. A/D를 놓으면 가운데로 돌아온다(정지 중에는 한 번 5°까지, ‘각 유지’를 고르면 그대로). 막대를 끌면 그 각으로, 놓으면 0°로. STOP·Esc는 언제나.',
  absolute: '조종 활성화 뒤 W/S는 누르는 동안 구동, A/D는 누르는 동안 왼쪽·오른쪽 끝까지, 놓으면 가운데로(‘각 유지’를 고르면 그대로), C는 가운데로. 막대를 끌면 그 각으로 가고 놓으면 0°로 돌아온다. 키와 막대는 먼저 잡은 쪽만 듣는다. STOP·Esc는 언제나.',
};

// The device chip (short) and, while not armed, the explanation in the status line (controller.device()).
export const DEVICE_TEXT = {
  present: ['ok', '조향 장치 연결됨', ''],
  none: ['warn', '조향 장치 없음', '조향 장치 없음: 조종 브리지가 조향 상태를 내지 않는다(V2 차동 브리지이거나 V3 펌웨어 없음). 조종 활성화를 할 수 없다.'],
  unavailable: ['bad', '연결 안 됨', '연결 안 됨: 조종 브리지에 닿지 않는다. STOP은 계속 보낸다.'],
  unknown: ['', '확인 중', ''],
};

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
    centreOff: target !== null && Math.abs(target) >= 0.5 && !ctl.centreReturn && !ctl.steeringActive(),
    absolute: ctl.absoluteMode(), dragActive: ctl.drag.active, dragDeg: ctl.drag.active ? ctl.drag.deg : 0,
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
  const dragEl = $('steerDrag');
  const drag = dragEl ? bindSteerDrag(dragEl, ctl) : null;
  const slider = $('steerSpeedSlider');
  slider?.addEventListener('input', () => ctl.setSpeedCap(Number(slider.value)));
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
  for (const id of ['frameStopButton', 'roverStopButton']) {
    $(id)?.addEventListener('click', () => ctl.abandon('operator_stop'));   // their own handler sends the STOP
  }
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
    const [level, text, explain] = DEVICE_TEXT[v.device] || DEVICE_TEXT.unknown;
    const device = $('steerDevice');
    if (device) { device.className = `status-chip${level ? ` ${level}` : ''}`; setText('steerDevice', text); }
    const arm = $('steerArmButton');
    if (arm) {
      arm.textContent = ctl.arming ? '활성화 중…' : ctl.releasing ? '감속 뒤 해제 중…' : ctl.lease ? '조종 활성화 ON' : '조종 활성화 OFF';
      arm.setAttribute('aria-pressed', String(Boolean(ctl.lease)));
      arm.disabled = ctl.arming || ctl.releasing || (!ctl.lease && v.device !== 'present');
    }
    const gauge = (id, pos) => { const el = $(id); if (el) { el.hidden = pos === null; if (pos !== null) el.style.left = `${pos}%`; } };
    gauge('steerGaugeTarget', v.targetPos);
    gauge('steerGaugeMeasured', v.measuredPos);
    setText('steerGaugeLeft', `왼쪽 ${fmtDeg(deg(v.limits.left))}`);
    setText('steerGaugeRight', `오른쪽 ${fmtDeg(deg(v.limits.right))}`);
    const colours = page.chartColors();
    page.renderReadout('steerReadout', [
      { label: '목표 / 측정', value: `${fmtDeg(v.targetDeg)} / ${fmtDeg(v.measuredDeg)}`, sub: v.fresh ? `오차 ${fmtDeg(v.errorDeg)}` : '상태 오래됨', color: colours.cyan },
      { label: '구동 duty', value: v.duty ? `${v.duty.left} / ${v.duty.right}` : '--', sub: 'MCU 보고 · 속도 아님', color: colours.blue },
      { label: '입력 기한', value: v.deadlineMs === null ? '--' : `${Math.round(v.deadlineMs)} ms`, sub: v.saturated ? '조향 포화' : '0.75 s · Esc 즉시 정지', color: colours.good },
    ]);
    setText('steerSpeedValue', `${ctl.speedCapPct}%`);
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
    setText('steerHelp', v.absolute ? HELP_TEXT.absolute : HELP_TEXT.ramp);
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
        : v.centreOff && ctl.lease && ctl.message.kind === 'ok' ? `${ctl.message.text} · 가운데 아님: 가운데 버튼(C)` : ctl.message.text;
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
