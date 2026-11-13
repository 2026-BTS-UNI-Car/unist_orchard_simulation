// Manual drive and steering panel of the front-steer vehicle (UI-07, docs/STEERING_SERVER_KO.md 3 and 4).
// Loaded only on a front-steer dashboard (html data-vehicle-kind="front_steer") by static/app/14-steer.js.
// The rules (arm, lease, ticket, seq, rate caps, release-to-centre, stale status, STOP retries) live in
// steer-control-rules.js; this module binds them to the page: buttons (pointer capture, two fingers), keys, the gauge,
// the readout and the status line. Requests go through the page's own postJson/getJson (the dashboard routes
// /api/rover/*), the same functions as the differential panel. The STOP buttons of the page keep their own code
// (stopRoverNow: one /api/rover/stop); here they only drop the steering lease at once.
import './steer-control-rules.js';

const R = globalThis.UbtsSteerRules;
const DEFAULT_LIMIT_MRAD = 409;          // placeholder host profile v0 (23.46 deg) until a status gives the limits
const ARMED_STATUS_MS = 50;              // armed: one status request at a time, as often as the link answers
const IDLE_STATUS_MS = 500;              // not armed: enough for the gauge and the device check
const deg = (mrad) => (typeof mrad === 'number' && isFinite(mrad) ? R.mradToDeg(mrad) : null);
const fmtDeg = (value) => (value === null ? '--' : `${value >= 0 ? '' : '−'}${Math.abs(value).toFixed(1)}°`);

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
  };
}

export function initSteerDrive(page) {
  const $ = (id) => document.getElementById(id);
  const root = $('steerDrive');
  if (!root || !R) return null;
  const ctl = new R.SteerController({
    transport: {
      status: () => page.getJson('/api/rover/status'),
      post: (path, body) => page.postJson(path, body),
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
  window.addEventListener('blur', () => ctl.blur());
  document.addEventListener('visibilitychange', () => {
    if (document.hidden) { if (ctl.lease) ctl.stop('page_hidden'); stopLoop(); } else startLoop();
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
