// Steering calibration part of Rover Control (2026-11-16, ESP32-S3 bridge op 'calibrate', firmware token
// CAL_RUNTIME=1; docs/esp_rover_control.md 1-4). Since 2026-11-19 a part of the 보정·설정 tab (before: a folded section
// of the steering panel; a page with the folded <details id="steerCalDetails"> still works): current CAL marks and the measured
// potentiometer, the buttons 지금 위치를 가운데로 / 왼쪽 끝 찾기 / 오른쪽 끝 찾기 / 적용 (with the margins) / 기본값으로,
// the progress of a running scan and the results. Every action is refused by the bridge while armed or moving and
// needs the firmware update; the buttons show the same reason. Actions that move the wheels or change the stored
// marks ask first (window.confirm). STOP (this section's button, the page's STOP buttons, Esc) ends any action; a page
// that is hidden or left while a scan runs sends STOP too. Loaded by static/app/14-steer.js when the section is
// opened on a front-steer dashboard. Requests: POST /api/rover/calibrate {action, left_margin_counts,
// right_margin_counts}; the status comes from GET /api/rover/status ('calibration' block), shared with the steering
// panel when its reply is fresh. The hard guards come from the status (the CAL line's GUARD; SMOOTH firmware r2: right
// >= 1300, left <= 2600, centre 1850..2150); the fallbacks below are only for a status without them.

export const STATUS_MS = 500;
export const SHARED_MAX_AGE_MS = 1000;
export const NOTICE_MS = 8000;           // the answer of this page's own request stays this long
export const ACTIONS = ['set_center', 'trim_left', 'trim_right', 'find_left', 'find_right', 'apply', 'default'];
// Fine zero (2026-11-18): one press moves CAL CENTER by TRIM_COUNTS (about 0.2 deg); + = left (the POT rises to the left).
export const TRIM_COUNTS = 5;
const num = (value) => (typeof value === 'number' && isFinite(value) ? value : null);

export const REASON_TEXT = {
  REBOUND: '되밀림', NO_PROGRESS: '진척 없음', GUARD: '허용 한계', ABORTED_INPUT: 'STOP이나 입력으로 중단',
  INVALID_POT: '가변저항 값 이상', UNSETTLED: '값이 안정되지 않음', BUDGET: '펄스 한도 도달', TIME: '시간 한도 도달',
};
export const STORE_TEXT = { DEFAULT: '기본값', NVS: '저장됨', UNSAVED: '저장 안 됨' };
export const ACTION_TEXT = {
  set_center: '가운데 저장', trim_left: '영점 왼쪽 이동', trim_right: '영점 오른쪽 이동', trim_center: '영점 이동', find_left: '왼쪽 끝 찾기', find_right: '오른쪽 끝 찾기', apply: '찾은 끝 적용', default: '기본값으로 되돌리기',
  read: '보정 값 읽기',
};
// What a finished action did, and what it could not do, as the sentences of the status line.
export const DONE_TEXT = {
  set_center: '가운데를 저장했습니다.', trim_center: '영점을 옮겼습니다.', find_left: '왼쪽 끝을 찾았습니다.', find_right: '오른쪽 끝을 찾았습니다.',
  apply: '찾은 끝을 적용했습니다.', default: '기본값으로 되돌렸습니다.', read: '보정 값을 읽었습니다.',
};
export const FAIL_TEXT = {
  set_center: '가운데를 저장하지 못했습니다.', trim_center: '영점을 옮기지 못했습니다.', find_left: '왼쪽 끝을 찾지 못했습니다.',
  find_right: '오른쪽 끝을 찾지 못했습니다.', apply: '찾은 끝을 적용하지 못했습니다.', default: '기본값으로 되돌리지 못했습니다.',
  read: '보정 값을 읽지 못했습니다.',
};
// Refusals and job errors (scripts/esp32_abs_hold.py, scripts/esp32_steer_bridge.py) as the operator reads them.
export const CAL_MESSAGES = {
  calibration_needs_firmware_update: '펌웨어에 실시간 보정 기능이 없습니다. 차를 받침에 올리고 펌웨어를 업데이트하세요.',
  calibration_needs_disarmed: '조종 활성화 중에는 보정할 수 없습니다. 조종 활성화를 끈 뒤 하세요.',
  calibration_needs_standstill: '차가 움직이는 중에는 보정할 수 없습니다. 차가 멈춘 뒤 하세요.',
  calibration_busy: '다른 보정 동작이 진행 중입니다.',
  calibrating: '조향 보정 중입니다.',
  calibration_stopped: 'STOP으로 멈췄습니다.',
  calibration_silent: '보드 응답이 끊겨 STOP을 보냈습니다.',
  calibration_timeout: '정해진 시간을 넘겨 STOP을 보냈습니다.',
  calibration_refused: '펌웨어가 요청을 거절했습니다.',
  calibration_end_not_found: '끝을 찾지 못했습니다.',
  calibration_readback_mismatch: '다시 읽은 보정 값이 보낸 값과 다릅니다.',
  calibration_not_saved: '저장됐다는 확인을 받지 못했습니다.',
  calibration_no_answer: '펌웨어가 응답하지 않았습니다.',
  calibration_interrupted: '보드가 다시 켜져 중단됐습니다.',
  calibration_link_closed: '연결이 끊겨 중단됐습니다.',
  calibration_stop_unconfirmed: 'STOP 확인을 받지 못해 시작하지 않았습니다.',
  calibration_nothing_found: '적용할 끝 값이 없습니다. 먼저 끝 찾기를 하세요.',
  calibration_margin_invalid: '여유는 10~120 count 사이의 정수로 넣으세요.',
  calibration_right_below_guard: '오른쪽 값이 허용 한계 밖입니다. 아래 허용 한계 줄을 확인하세요.',
  calibration_left_above_guard: '왼쪽 값이 허용 한계 밖입니다. 아래 허용 한계 줄을 확인하세요.',
  calibration_center_outside: '가운데 값이 허용 범위 밖입니다. 아래 허용 한계 줄을 확인하세요.',
  calibration_pulse_invalid: '끝 찾기 펄스는 40~100 ms 사이의 정수여야 합니다.',
  calibration_pulse_needs_firmware_update: '끝 찾기 펄스 길이는 SMOOTH 펌웨어부터 정할 수 있습니다.',
  calibration_right_span_small: '오른쪽 폭이 150 count보다 작습니다.',
  calibration_left_span_small: '왼쪽 폭이 150 count보다 작습니다.',
  invalid_calibration_action: '알 수 없는 보정 동작입니다.',
  esp32_not_ready: 'ESP32가 준비 중입니다.',
  device_unavailable: '장치 연결이 끊겼습니다.',
  firmware_mismatch: '펌웨어 이름이 ABS-HOLD가 아닙니다.',
  steering_status_stale: '조향 상태가 들어오지 않습니다.',
  pot_invalid: '가변저항 값이 올바르지 않습니다. 배선을 확인하세요.',
  stop_unconfirmed: 'STOP 확인을 받지 못했습니다.',
  operator_required: '조종자 세션이 없습니다.',
  motor_bridge_unavailable: '브리지에 연결할 수 없습니다.',
  control_busy_stopped: '브리지가 다른 요청을 처리하는 중이라 정지했습니다.',
  request_expired: '요청이 제때 처리되지 않았습니다.',
  stop_barrier: '다른 STOP이 먼저 처리됐습니다.',
  http_failed: '서버에 요청을 보내지 못했습니다.',
  bad_reply: '응답 형식이 올바르지 않습니다.',
};
export const calMessage = (code) => (code ? (CAL_MESSAGES[code] || String(code)) : '');
// The same as a sentence for the status line: an unknown code is named in one.
export const calSentence = (code) => (!code ? '이유는 알 수 없습니다.' : CAL_MESSAGES[code] || `보드가 알린 이유는 ${String(code)}입니다.`);

const marksText = (m) => (m ? `오른쪽 ${m.right} · 가운데 ${m.center} · 왼쪽 ${m.left}` : '--');
// The rows next to the centre buttons; the others (found ends, what 'apply' would set, the limits) are shown with the
// folded end finding (#steerCalMore, 2026-11-20).
export const MAIN_ROWS = ['지금 보정', '가변저항', '가변저항 정착 값'];

function foundText(found) {
  if (!found) return '--';
  const age = num(found.age_s);
  const ago = age === null ? '' : age < 90 ? ` · ${Math.round(age)}초 전` : ` · ${Math.round(age / 60)}분 전`;
  const how = `멈춘 이유 ${REASON_TEXT[found.reason] || found.reason} · ${found.pulses} 펄스 · ${(found.time_ms / 1000).toFixed(1)} s`;
  return `${found.pot} · ${how}${ago}${found.usable ? '' : ' · 쓸 수 없음'}`;
}

export const FALLBACK_GUARDS = { right_min: 1300, left_max: 2600, center_min: 1850, center_max: 2150, min_span: 150 };

// The marks 'apply' would set with these margins (as scripts/esp32_abs_hold.cal_proposal), or a problem code.
export function proposal(cal, margins) {
  if (!cal || !cal.cal) return { marks: null, sides: [], problem: 'calibration_nothing_found' };
  const base = cal.cal;
  const marks = { right: base.right, center: base.center, left: base.left };
  const sides = [];
  for (const side of ['left', 'right']) {
    const found = cal.found && cal.found[side];
    if (found && found.usable) {
      const margin = margins[side];
      marks[side] = side === 'left' ? found.pot - margin : found.pot + margin;
      sides.push(side);
    }
  }
  const [lo, hi] = Array.isArray(cal.margin_range) ? cal.margin_range : [10, 120];
  if (!sides.length) return { marks, sides, problem: 'calibration_nothing_found' };
  for (const side of sides) {
    const margin = margins[side];
    if (!Number.isInteger(margin) || margin < lo || margin > hi) return { marks, sides, problem: 'calibration_margin_invalid' };
  }
  const g = cal.guards || FALLBACK_GUARDS;
  let problem = null;
  if (marks.right < g.right_min) problem = 'calibration_right_below_guard';
  else if (marks.left > g.left_max) problem = 'calibration_left_above_guard';
  else if (marks.center < g.center_min || marks.center > g.center_max) problem = 'calibration_center_outside';
  else if (marks.center - marks.right < g.min_span) problem = 'calibration_right_span_small';
  else if (marks.left - marks.center < g.min_span) problem = 'calibration_left_span_small';
  return { marks, sides, problem };
}

// Everything the section shows for one status reply (pure: tested without a page). margins: {left, right} (ints).
// notice: the answer of this page's last request {kind, text} (shown until the next status says more).
export function calView(reply, margins = { left: 30, right: 30 }, notice = null) {
  const off = (reason, level = 'bad', chip) => ({
    level, chip: chip || reason, rows: [], samples: [], running: false, blocker: reason,
    buttons: Object.fromEntries(ACTIONS.map((a) => [a, { enabled: false, title: reason }])),
    status: { kind: level, text: reason }, apply: null,
  });
  if (!reply) return off('조종 브리지에 연결되지 않았습니다.', 'bad', '연결 안 됨');
  const cal = reply.calibration;
  if (!cal || typeof cal !== 'object') return off('이 조종 브리지에서는 조향 보정을 할 수 없습니다. ESP32-S3 브리지에서만 됩니다.', 'warn', '보정 없음');
  const running = cal.state === 'running' && cal.job;
  const blockerCode = cal.blocker || null;
  const blocker = running ? null : calMessage(blockerCode);
  const store = cal.cal ? (STORE_TEXT[cal.cal.store] || cal.cal.store || '') : '';
  const pot = running && num(cal.live_pot) !== null ? cal.live_pot : cal.pot;
  const plan = proposal(cal, margins);
  const g = { ...FALLBACK_GUARDS, ...(cal.guards || {}) };
  const rows = [
    ['지금 보정', cal.cal ? `${marksText(cal.cal)}${store ? ` · ${store}` : ''}` : '--'],
    [running ? '가변저항 정착 값' : '가변저항', num(pot) === null ? '--' : String(pot)],
    ['찾은 왼쪽 끝', foundText(cal.found && cal.found.left)],
    ['찾은 오른쪽 끝', foundText(cal.found && cal.found.right)],
    ['적용하면', plan.problem ? calMessage(plan.problem) : `${marksText(plan.marks)} · ${plan.sides.map((s) => (s === 'left' ? '왼쪽' : '오른쪽')).join('·')} 바뀜`],
    ['허용 한계', `오른쪽 ≥ ${g.right_min} · 왼쪽 ≤ ${g.left_max} · 가운데 ${g.center_min}–${g.center_max} · 폭 ≥ ${g.min_span}`],
    ['한 번에 가는 최대 각', cal.smooth === true ? 'SMOOTH 펌웨어라 목표까지 끊지 않고 한 번에 갑니다.'
      : num(cal.max_jump_deg) === null ? '--' : `${cal.max_jump_deg}° · 한 번에 1초까지만 움직여 큰 각은 나눠 갑니다.`],
  ];
  const enabled = !running && !blockerCode;
  const buttons = {};
  for (const action of ACTIONS) {
    let why = running ? `${ACTION_TEXT[cal.job.action] || cal.job.action} 동작이 진행 중입니다.` : blocker;
    if (!why && action === 'apply' && plan.problem) why = calMessage(plan.problem);
    if (!why && action === 'set_center' && cal.cal && num(cal.pot) !== null
        && (cal.pot < g.center_min || cal.pot > g.center_max)) {
      why = `지금 위치가 가운데 범위 ${g.center_min}–${g.center_max} 밖입니다. 지금 값은 ${cal.pot}입니다.`;
    }
    buttons[action] = { enabled: enabled && !why, title: why || ACTION_TEXT[action] };
  }
  let level = 'ok', chip = cal.cal ? `보정 ${store || '있음'}` : '보정 가능';
  let status = { kind: '', text: '보정할 수 있습니다. 찾은 끝은 찾은 끝 적용을 눌러야 쓰입니다.' };
  const samples = Array.isArray(cal.samples) ? cal.samples : [];
  if (running) {
    const job = cal.job;
    const last = samples.length ? samples[samples.length - 1] : null;
    const side = job.action === 'find_left' ? 'left' : job.action === 'find_right' ? 'right' : null;
    level = 'warn';
    chip = '보정 중';
    let text = `${ACTION_TEXT[job.action] || job.action} 중 · ${job.elapsed_s} s`;
    if (side && last && last.side === side) {
      text += ` · 펄스 ${last.n} · 정착 ${last.settled} · 진척 ${last.progress >= 0 ? '+' : ''}${last.progress} · 되밀림 ${last.rebound}`;
    } else if (side) text += ' · 첫 표본 대기 중';
    status = { kind: 'testing', text: `멈추려면 STOP을 누르세요. ${text}` };
  } else if (blockerCode) {
    level = blockerCode === 'calibration_needs_firmware_update' ? 'warn' : 'bad';
    if (['calibration_needs_disarmed', 'calibration_needs_standstill'].includes(blockerCode)) level = 'warn';
    chip = blockerCode === 'calibration_needs_firmware_update' ? '펌웨어 업데이트 필요' : '지금은 불가';
    status = { kind: level, text: blocker };
  } else if (cal.last) {
    const last = cal.last;
    const name = ACTION_TEXT[last.action] || last.action;
    if (last.ok) {
      let text = DONE_TEXT[last.action] || `${name} 동작이 끝났습니다.`;
      if (['apply', 'default', 'set_center', 'trim_center'].includes(last.action)) text += ` 저장한 값 · ${marksText(last.cal_after)}`;
      if (last.action === 'find_left' || last.action === 'find_right') text += ' 찾은 끝은 찾은 끝 적용을 눌러야 쓰입니다.';
      status = { kind: 'ok', text };
    } else {
      const notFound = last.error === 'calibration_end_not_found';
      const reason = notFound && last.detail ? ` 멈춘 이유는 ${REASON_TEXT[last.detail] || last.detail}입니다.`
        : last.detail ? ` 세부 내용은 ${last.detail}입니다.` : '';
      const failed = FAIL_TEXT[last.action] || `${name} 동작이 실패했습니다.`;
      // "끝을 찾지 못했습니다" twice says nothing more: a failed end search with that error keeps one of them
      const same = notFound && (last.action === 'find_left' || last.action === 'find_right');
      status = { kind: 'bad', text: `${failed}${same ? '' : ` ${calSentence(last.error)}`}${reason}` };
      if (cal.cal && cal.cal.store === 'UNSAVED') status.text += ' 저장하지 않은 값이 쓰이고 있습니다. 다시 적용하거나 기본값으로 되돌리세요.';
    }
  }
  if (notice && !running) status = notice;
  return { level, chip, rows, samples, running: Boolean(running), blocker: blockerCode, buttons, status, apply: plan };
}

export function initSteerCalibration(page, options = {}) {
  const doc = options.document || document;
  const confirmFn = options.confirm || ((text) => globalThis.confirm(text));
  const now = options.now || (() => Date.now());
  const $ = (id) => doc.getElementById(id);
  const root = $('steerCalPanel');
  const details = $('steerCalDetails');
  if (!root) return null;
  let timer = null, busy = false, posting = false, reply = null, notice = null, view = null;
  // Asked only while the part is on screen: its tab is shown (and a folded part of an older page is open).
  const active = () => page.currentRoute() === '/rover-control' && !doc.hidden && (!details || details.open)
    && (page.shown ? page.shown('steerCalPanel') : true);
  const marginInputs = { left: $('steerCalMarginLeft'), right: $('steerCalMarginRight') };
  const margins = () => {
    const read = (el) => { const v = Number(el?.value); return Number.isFinite(v) ? v : NaN; };
    return { left: read(marginInputs.left), right: read(marginInputs.right) };
  };

  async function poll() {
    if (busy) return;
    const shared = page.driveStatus?.();
    if (shared && now() - shared.at <= SHARED_MAX_AGE_MS) { reply = shared.reply; draw(); return; }
    busy = true;
    try { reply = await page.getJson('/api/rover/status'); } catch (_) { reply = null; }
    finally { busy = false; }
    draw();
  }

  function setText(el, value) { if (el && el.textContent !== value) el.textContent = value; }
  function draw() {
    const fresh = notice && now() - notice.at < NOTICE_MS ? notice : null;
    view = calView(reply, margins(), fresh);
    const chip = $('steerCalState');
    if (chip) { chip.className = `status-chip ${view.level}`; setText(chip, view.chip); }
    // With the folded end finding on the page its rows go there; a page without it shows every row in one list.
    const more = $('steerCalMore');
    const lists = more ? [[$('steerCalRows'), view.rows.filter(([name]) => MAIN_ROWS.includes(name))],
                          [more, view.rows.filter(([name]) => !MAIN_ROWS.includes(name))]] : [[$('steerCalRows'), view.rows]];
    for (const [list, rows] of lists) {
      if (!list) continue;
      const key = JSON.stringify(rows);
      if (list.dataset.key === key) continue;
      list.dataset.key = key;
      list.replaceChildren(...rows.map(([name, value]) => {
        const row = doc.createElement('div');
        const dt = doc.createElement('dt'); dt.textContent = name;
        const dd = doc.createElement('dd'); dd.textContent = value;
        row.append(dt, dd);
        return row;
      }));
    }
    root.querySelectorAll('[data-steer-cal]').forEach((button) => {
      const state = view.buttons[button.dataset.steerCal];
      button.disabled = posting || !state || !state.enabled;
      button.title = state ? state.title : '';
    });
    const status = $('steerCalStatus');
    if (status) { status.className = `test-result ${view.status.kind || ''}`.trim(); setText(status, view.status.text); }
    const wrap = $('steerCalSamplesWrap');
    const body = $('steerCalSamples');
    if (wrap && body) {
      wrap.hidden = view.samples.length === 0;
      const key = JSON.stringify(view.samples);
      if (body.dataset.key !== key) {
        body.dataset.key = key;
        body.replaceChildren(...view.samples.slice().reverse().map((s) => {
          const tr = doc.createElement('tr');
          for (const value of [s.n, s.side === 'left' ? '왼쪽' : '오른쪽', s.settled, s.progress, s.rebound]) {
            const td = doc.createElement('td'); td.textContent = String(value); tr.append(td);
          }
          return tr;
        }));
      }
    }
  }

  function question(action) {
    const cal = reply?.calibration;
    const current = marksText(cal?.cal);
    if (action === 'apply') {
      const plan = view?.apply;
      return `찾은 끝으로 조향 보정 값을 바꾸고 보드에 저장할까요?\n지금 값 · ${current}\n새 값 · ${marksText(plan?.marks)}\n`
        + '조향 명령 ±20은 찾은 끝에서 여유만큼 안쪽인 새 값에 맞춰집니다.';
    }
    if (action === 'default') {
      const d = cal?.defaults || { right: 1681, center: 1900, left: 2180 };
      return `조향 보정 값을 기본값으로 되돌리고 보드에 저장된 값을 지울까요?\n기본값 · ${marksText(d)}\n지금 값 · ${current}`;
    }
    if (action === 'set_center') {
      return `지금 바퀴 위치를 가운데로 저장할까요? 가변저항 값은 ${cal?.pot ?? '--'}입니다.\n앞바퀴가 똑바로 놓였는지 먼저 확인하세요.`;
    }
    const side = action === 'find_left' ? '왼쪽' : '오른쪽';
    return `앞바퀴를 ${side} 끝까지 천천히 돌려 끝을 찾을까요? 조향 모터만 움직이고 구동 모터는 쓰지 않습니다. 최대 약 50초 동안 움직입니다.\n`
      + '바퀴 주변에 사람이나 물건이 없는지 확인하세요. 멈추려면 STOP을 누르세요.';
  }

  async function run(action) {
    if (posting || !view?.buttons[action]?.enabled) return;
    const trim = action === 'trim_left' ? TRIM_COUNTS : action === 'trim_right' ? -TRIM_COUNTS : 0;
    if (!trim && !confirmFn(question(action))) return;   // a trim moves no wheel: no question
    const body = trim ? { action: 'trim_center', delta_counts: trim } : { action };
    if (action === 'apply') { const m = margins(); body.left_margin_counts = m.left; body.right_margin_counts = m.right; }
    posting = true;
    notice = { kind: 'testing', text: `${ACTION_TEXT[action]} 요청을 보내고 있습니다.`, at: now() };
    draw();
    try {
      const answer = await page.postJson('/api/rover/calibrate', body);
      notice = answer && answer.ok ? null : { kind: 'bad', text: `${ACTION_TEXT[action]} 요청이 거절됐습니다. ${calSentence(answer?.error || 'bad_reply')}`, at: now() };
    } catch (_) {
      notice = { kind: 'bad', text: `${ACTION_TEXT[action]} 요청을 서버에 보내지 못했습니다.`, at: now() };
    } finally {
      posting = false;
    }
    await poll();
  }

  function stopNow(reason) {
    notice = null;
    return Promise.resolve((page.controlRequest || page.postJson)('/api/rover/stop', { reason })).catch(() => null).then(() => poll());
  }

  root.querySelectorAll('[data-steer-cal]').forEach((button) => {
    button.addEventListener('click', () => run(button.dataset.steerCal));
  });
  $('steerCalStop')?.addEventListener('click', () => stopNow('calibration_stop'));
  for (const el of Object.values(marginInputs)) el?.addEventListener('input', () => { notice = null; draw(); });
  details?.addEventListener('toggle', () => routeChanged());
  doc.addEventListener('visibilitychange', () => {
    // A scan moves the wheels without an armed operator: a hidden page stops it, as it stops driving.
    if (doc.hidden && view?.running) stopNow('page_hidden');
    routeChanged();
  });
  globalThis.addEventListener?.('pagehide', () => {
    if (!view?.running) return;
    try {
      navigator.sendBeacon('/api/rover/stop', new Blob([JSON.stringify({ reason: 'pagehide' })], { type: 'application/json' }));
    } catch (_) { /* the scan ends by itself within its budget */ }
  });
  function routeChanged() {
    if (active()) { if (!timer) { timer = setInterval(poll, STATUS_MS); poll(); } }
    else {
      clearInterval(timer); timer = null;
      // A scan moves the wheels: leaving the page or its tab stops it, as a hidden page does.
      if (view?.running && (page.currentRoute() !== '/rover-control' || (page.shown && !page.shown('steerCalPanel')))) stopNow('page_inactive');
    }
  }
  routeChanged();
  return { routeChanged, draw, poll, run, view: () => view };
}
