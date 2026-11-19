// Distance move part of Rover Control's steering panel (2026-11-17, user request: choose the terrain and send "1 m
// forward"; docs/esp_rover_control.md 1-6). A folded section of the 수동 조종 tab: direction, terrain, distance, the speed
// (2026-11-19, "거리 이동하는거 속도 조절 되게": a duty slider from the terrain's lowest to its highest table duty, 1 %
// steps, never above the bridge's drive cap that the plan reply names (bridge.max_duty); between table rows the server
// interpolates; default the lowest, the 11-17 page's first choice), the server's plan (drive time, expected distance
// and the grade of each part; the details folded), a button that runs only while it is held (deadman), and the result.
// Backward (2026-11-19, "특정거리 앞으로 가는거 뿐만 아니라 뒤로 가는 기능"): the same move with direction 'backward' (the
// server drives a negative throttle, at most 2 m: nothing looks behind the vehicle; the table was driven forward, so a
// backward plan is an assumption). The tape measurement that becomes the correction factor of that terrain, direction
// and duty is a part of the 보정·설정 tab (#steerDistTapePanel; a duty without one borrows the nearest measured duty of
// the same direction, the rows say which). The data are the October 2026 drive records of the vehicle BEFORE the steering
// unit (ESP8266); the steering vehicle has not been measured.
// Terrain in the tape part (2026-11-20, user: "줄자 보정에서 도로/포장 옵션만 있는데, 그 옵션을 바꿀 수 있게"): the tape part
// has its own terrain choice (#steerDistTapeTerrain). It is the same choice as the one of the 수동 조종 tab: changing
// either changes the other, the plan and the corrections shown. The reset clears the chosen terrain. A tape distance
// belongs to the terrain its move was planned with (the server files it there), so it can be saved only while that
// terrain is the chosen one; with another terrain chosen the part says which terrain the move was.
// While the button is held the server drives with this page's lease (dashboard_server/drive_distance.py): the page hands
// its lease chain over (controller.beginExternal), sends a hold heartbeat every HOLD_MS, and takes the chain back when
// the move ends (endExternal). Release, a hidden page, blur, a lost pointer or leaving the tab -> release request; the
// server also stops on a missing heartbeat (0.4 s) and the bridge on its own input TTL (0.75 s). STOP (this section's
// button, the page's STOP buttons, Esc) always wins. Loaded by static/app/14-steer.js when the section is opened (or the
// 보정·설정 tab shown) the first time.

export const HOLD_MS = 100;
export const PLAN_DEBOUNCE_MS = 250;
export const RELEASE_RETRIES = 3;
const num = (value) => (typeof value === 'number' && isFinite(value) ? value : null);

export const REASON_TEXT = {
  planned: '계획대로 끝남', released: '버튼을 놓음', heartbeat_lost: '화면 연결 끊김', operator_stop: 'STOP',
  bridge_error: '브리지 오류', page_hidden: '화면이 가려짐', blur: '다른 창으로 넘어감', tab_changed: '탭을 바꿈',
};
// The same reasons as the sentence of the result line (수동 조종 tab).
export const REASON_SENTENCE = {
  planned: '계획한 시간만큼 이동했습니다.', released: '버튼을 놓아 멈췄습니다.', heartbeat_lost: '화면 연결이 끊겨 멈췄습니다.',
  operator_stop: 'STOP으로 멈췄습니다.', bridge_error: '브리지 오류로 멈췄습니다.', page_hidden: '화면이 가려져 멈췄습니다.',
  blur: '다른 창으로 넘어가 멈췄습니다.', tab_changed: '탭을 바꿔 멈췄습니다.',
};
export const DIRECTION_TEXT = { forward: '앞으로', backward: '뒤로' };
export const MESSAGES = {
  distance_terrain_unknown: '알 수 없는 지형입니다.',
  distance_duty_invalid: '속도 출력 값은 정수여야 합니다.',
  distance_duty_out_of_range: '이 지형의 속도 표 범위를 벗어난 출력 값입니다.',
  distance_invalid: '거리에 숫자를 넣으세요.',
  distance_out_of_range: '거리는 0.2~3.0 m 사이로 넣으세요.',
  distance_out_of_range_backward: '뒤로는 0.2~2.0 m 사이로 넣으세요. 차 뒤쪽은 감지하지 못합니다.',
  distance_direction_invalid: '방향 값이 올바르지 않습니다.',
  distance_too_short_for_duty: '이 속도로 가기에는 거리가 너무 짧습니다. 속도를 낮추거나 거리를 늘리세요.',
  distance_too_long_for_duty: '이 속도로 가기에는 너무 긴 거리입니다. 속도를 높이거나 거리를 줄이세요.',
  distance_needs_steer_bridge: 'ESP32-S3 조향 브리지에서만 쓸 수 있습니다.',
  distance_drive_cap_zero: '브리지의 구동 상한이 0이라 거리 이동을 할 수 없습니다.',
  distance_drive_cap_unknown: '브리지가 구동 상한을 알려 주지 않았습니다.',
  distance_duty_above_cap: '출력이 브리지의 구동 상한보다 커서 표의 속도가 나오지 않습니다.',
  distance_needs_arm: '먼저 조종 활성화를 누르세요.',
  distance_needs_lease: '조종 권한 정보가 없습니다. 조종 활성화를 다시 누르세요.',
  distance_needs_manual: '자율주행 중에는 거리 이동을 할 수 없습니다.',
  distance_move_running: '거리 이동을 이미 하고 있습니다.',
  distance_move_unknown: '이 화면에서 시작한 거리 이동이 아닙니다.',
  distance_feedback_needs_full_move: '계획한 시간까지 간 이동만 보정에 쓸 수 있습니다.',
  distance_feedback_done: '이 이동의 거리는 이미 저장했습니다.',
  distance_actual_invalid: '잰 거리는 0 m보다 크고 10 m 이하여야 합니다.',
  distance_not_configured: '서버에 거리 이동이 설정되어 있지 않습니다.',
  motor_bridge_unavailable: '브리지에 연결할 수 없습니다.',
  control_not_owned: '다른 화면에서 조종하고 있습니다.',
  stale_input_rearm_required: '조종 권한이 만료됐습니다. 조종 활성화를 다시 누르세요.',
  http_failed: '서버에 요청을 보내지 못했습니다.',
};
export const message = (code) => (code ? (MESSAGES[code] || String(code)) : '');
// The same as a sentence for a status line: an unknown code is named in one.
export const sentence = (code) => (!code ? '이유는 알 수 없습니다.' : MESSAGES[code] || `서버가 알린 이유는 ${String(code)}입니다.`);
// "...(으)로" after a name the server sends (a terrain label): 으로 after a final consonant other than rieul.
export function withRo(word) {
  const code = String(word || '').slice(-1).charCodeAt(0);
  if (!(code >= 0xac00 && code <= 0xd7a3)) return `${word}(으)로`;
  const final = (code - 0xac00) % 28;
  return `${word}${final === 0 || final === 8 ? '로' : '으로'}`;
}
const fixed = (value, digits) => (num(value) === null ? '--' : value.toFixed(digits));
// The rows shown under the controls; the others are in the folded '계획 자세히'.
export const MAIN_ROWS = ['계획 시간', '예상 거리', '보정 배율', '구동 상한'];

// The duty range of a terrain of the table view (the server sends it; older views: the rows' lowest and highest).
export function dutyRange(terrain) {
  const duties = (terrain?.duties || []).map((d) => d.duty);
  const min = num(terrain?.duty_min) ?? (duties.length ? Math.min(...duties) : null);
  const max = num(terrain?.duty_max) ?? (duties.length ? Math.max(...duties) : null);
  return { min, max, def: num(terrain?.default_duty) ?? min };
}

// The slider range: the terrain's table range, its top cut at the bridge's drive cap (bridge.max_duty of the plan
// reply) when that is known. A cap below the lowest table duty leaves the slider at that duty (the plan refuses it).
export function sliderRange(terrain, bridge) {
  const range = dutyRange(terrain);
  const cap = num(bridge?.max_duty);
  if (range.min === null || cap === null) return range;
  return { ...range, max: Math.max(range.min, Math.min(range.max, cap)) };
}

// The longest distance of a direction (the table view's limits; 3.0 m forward, 2.0 m backward).
export function distanceLimit(table, direction) {
  const limits = table?.limits || {};
  return direction === 'backward' ? num(limits.distance_max_backward_m) ?? 2.0 : num(limits.distance_max_m) ?? 3.0;
}

// The table speed at a duty, as the server plans it (dashboard_server/drive_distance.py speed_at): the row itself or
// the straight line between the two rows around it; null outside the table. For the slider's readout before the plan.
export function speedAt(terrain, duty) {
  const rows = [...(terrain?.duties || [])].sort((a, b) => a.duty - b.duty);
  const row = rows.find((r) => r.duty === duty);
  if (row) return row.speed_m_s;
  for (let i = 1; i < rows.length; i += 1) {
    const lo = rows[i - 1], hi = rows[i];
    if (lo.duty < duty && duty < hi.duty) return lo.speed_m_s + (hi.speed_m_s - lo.speed_m_s) * (duty - lo.duty) / (hi.duty - lo.duty);
  }
  return null;
}

// The correction row: this duty's own tape samples, another duty's borrowed (same direction), or none.
export function correctionText(plan) {
  const c = plan.correction || { factor: plan.factor, samples: 0, from_duty: null };
  const way = plan.direction === 'backward' ? '뒤로 ' : '';
  if (num(c.from_duty) === null || !c.samples) return `${fixed(num(c.factor) ?? 1, 2)} · ${way}줄자 기록 없음`;
  if (c.from_duty === plan.duty) return `${fixed(c.factor, 2)} · ${way}줄자 ${c.samples}번`;
  return `${fixed(c.factor, 2)} · 출력 ${c.from_duty} %의 ${way}배율을 빌려 씀`;
}

// The tape corrections of one terrain and direction as a row text, or null.
function measuredText(corrections, terrain, direction) {
  const entry = corrections?.[terrain];
  const list = (direction === 'backward' ? entry?.backward_duties : entry?.duties) || [];
  return list.length ? list.map((d) => `${d.duty} % ×${fixed(d.factor, 2)} (${d.samples}번)`).join(' · ') : null;
}

// The rows and the button state for one moment (pure: tested without a page).
// s: {table, corrections, plan, bridge, planError, move, result, armed, external, fresh, notice, input, direction}
// input: {terrain, duty} as the controls show them now (the plan may still be for the previous duty).
export function distView(s) {
  const rows = [];
  const plan = s.plan;
  const direction = s.direction === 'backward' ? 'backward' : 'forward';
  const terrain = s.table?.terrains?.find((t) => t.key === plan?.terrain);
  if (plan && plan.speed) {
    const sp = plan.speed;
    rows.push(['계획 시간', `${fixed(plan.drive_s, 2)} s · 출력 ${plan.duty} % · PWM ${plan.pwm}`]);
    rows.push(['예상 거리', `${fixed(plan.expected_m, 2)} m · 등급 ${plan.grade}${plan.direction === 'backward' ? ' · 뒤로' : ''}`]);
    rows.push(['속도', sp.interpolated && Array.isArray(sp.between)
      ? `${fixed(sp.value, 3)} m/s · 등급 ${sp.grade} · 표의 ${sp.between[0]} %와 ${sp.between[1]} % 사이를 직선으로 이음 · ${sp.reference}`
      : `${fixed(sp.value, 3)} m/s · 등급 ${sp.grade} · ${sp.reference} · 구간 ${sp.segments}`]);
    rows.push(['출발 지연', `${fixed(plan.lag.value, 2)} s · 등급 ${plan.lag.grade} · ${plan.lag.grade === '가정' ? '출력 100 % 출발 값을 씀' : '출력 100 % 출발 9건'}`]);
    rows.push(['놓은 뒤 미끄러짐', `${fixed(plan.coast.value, 2)} m · 등급 ${plan.coast.grade}`]);
    rows.push(['이 속도로 갈 수 있는 거리', `${fixed(plan.min_distance_m, 2)}–${fixed(plan.max_distance_m, 2)} m`]);
    rows.push(['보정 배율', correctionText(plan)]);
    const measured = measuredText(s.corrections, plan.terrain, plan.direction);
    if (measured) rows.push(['줄자 기록 있는 속도', measured]);
  }
  if (terrain?.duties?.length) rows.push(['표에 있는 출력', `${terrain.duties.map((d) => d.duty).join(' · ')} % · 사이 값은 직선으로 이음`]);
  const cap = num(s.bridge?.max_pwm);
  const capDuty = num(s.bridge?.max_duty);
  rows.push(['구동 상한', cap === null ? '--' : cap === 0 ? 'PWM 0이라 바퀴가 돌지 않습니다.' : `PWM ${cap}${capDuty === null ? '' : ` · 출력 ${capDuty} %까지`}`]);
  rows.push(['자료', `${terrain ? `${terrain.label} ${terrain.day} · ` : ''}2026-10에 조향 장치를 달기 전의 ESP8266 차로 잰 기록입니다. 지금 차로는 재지 않았습니다.${direction === 'backward' ? ' 뒤로 갈 때도 앞으로 잰 표를 그대로 쓴다고 가정했습니다.' : ''}`]);

  // The slider's readout: the duty now, its table speed at once, the drive time once the plan is for this duty.
  const inTerrain = s.table?.terrains?.find((t) => t.key === s.input?.terrain);
  const duty = num(s.input?.duty);
  let speed = '--';
  if (inTerrain && duty !== null) {
    const v = speedAt(inTerrain, duty);
    const planned = plan && plan.speed && plan.terrain === inTerrain.key && plan.duty === duty;
    const time = planned ? (plan.problem ? '이 거리는 불가' : `${fixed(plan.drive_s, 2)} s`) : s.planError ? '--' : '계산 중';
    speed = `${duty} % · ${v === null ? '--' : `${v.toFixed(2)} m/s`} · ${time}`;
  }

  const running = s.move && s.move.state === 'driving';
  let why = null;
  if (!s.table) why = '속도 표를 읽고 있습니다.';
  else if (s.external || running) why = '거리 이동 중입니다.';
  else if (!s.armed) why = message('distance_needs_arm');
  else if (!s.fresh) why = '조향 상태가 오래됐습니다.';
  else if (s.planError) why = sentence(s.planError);
  else if (!plan) why = '이동 계획을 계산하고 있습니다.';
  else if ((plan.direction || 'forward') !== direction) why = '이동 계획을 계산하고 있습니다.';
  else if (plan.problem) why = sentence(plan.problem);
  else if (s.bridge?.refusal) why = sentence(s.bridge.refusal);
  const go = { enabled: !why, title: why || '누르고 있는 동안만 이동합니다. 놓으면 바로 멈춥니다.',
               text: `누르는 동안 ${DIRECTION_TEXT[direction]}` };

  let status = { kind: '', text: '버튼을 누르고 있는 동안만 이동합니다.' };
  if (running) {
    const way = DIRECTION_TEXT[s.move.direction] ? `${DIRECTION_TEXT[s.move.direction]} ` : '';
    status = { kind: 'testing', text: `버튼을 놓으면 바로 멈춥니다. ${way}이동 중 · ${fixed(s.move.elapsed_s, 1)} / ${fixed(s.move.planned_s, 2)} s` };
  } else if (s.result) {
    const r = s.result;
    const expected = num(r.expected_for_elapsed_m) ?? r.expected_m;
    const ended = REASON_SENTENCE[r.reason] || (r.reason ? `이동이 끝났습니다. 끝난 이유는 ${r.reason}입니다.` : '이동이 끝났습니다.');
    const kind = r.reason === 'planned' ? 'ok' : r.state === 'failed' || r.state === 'stopped' ? 'bad' : 'warn';
    const text = `${ended}${r.feedback_open ? ' 줄자로 잰 거리는 보정·설정 탭에 넣으세요.' : ''} 계획 ${fixed(r.planned_s, 2)} s · 실제 ${fixed(r.elapsed_s, 2)} s · 예상 ${fixed(expected, 2)} m`;
    status = { kind, text };
  }
  if (s.notice && !running) status = s.notice;
  const feedback = { enabled: Boolean(s.result && s.result.feedback_open && !running), title: s.result?.feedback_open ? '줄자로 잰 거리를 저장합니다.' : '계획한 시간까지 이동한 뒤에 저장할 수 있습니다.' };
  const chip = running ? ['warn', '이동 중'] : go.enabled ? ['ok', '준비'] : ['', '대기'];
  return { rows, go, feedback, status, chip, speed, direction,
           main: rows.filter(([name]) => MAIN_ROWS.includes(name)), more: rows.filter(([name]) => !MAIN_ROWS.includes(name)) };
}

// The 보정·설정 tape part: the last move of this page, the corrections of the chosen terrain in both directions. A tape
// distance can be saved only for a move of the chosen terrain (note says so when another one is chosen).
export function tapeView(s) {
  const terrain = s.input?.terrain;
  const labelOf = (key) => s.table?.terrains?.find((item) => item.key === key)?.label || key || '--';
  const r = s.result;
  const moved = r && typeof r.terrain === 'string' && r.terrain ? r.terrain : null;
  const last = r ? `마지막 이동 · ${moved ? `${labelOf(moved)} · ` : ''}${DIRECTION_TEXT[r.direction] || '앞으로'} ${fixed(num(r.distance_m), 1)} m · 출력 ${num(r.duty) === null ? '--' : `${r.duty} %`} · ${REASON_TEXT[r.reason] || r.reason || ''}`
    : '이 화면에서 끝낸 거리 이동이 아직 없습니다.';
  const rows = [];
  for (const direction of ['forward', 'backward']) rows.push([`${DIRECTION_TEXT[direction]} 줄자 배율`, measuredText(s.corrections, terrain, direction) || '기록 없음']);
  const open = Boolean(r && r.feedback_open && !(s.move && s.move.state === 'driving'));
  const other = open && Boolean(moved && terrain && moved !== terrain);
  const note = other ? `마지막 이동은 ${labelOf(moved)}에서 했습니다. 저장하려면 지형을 ${withRo(labelOf(moved))} 고르세요.` : null;
  const enabled = open && !other;
  return { last, rows, note, chip: enabled ? ['ok', '입력 가능'] : other ? ['warn', '지형 다름'] : ['', '대기'],
           feedback: { enabled, title: enabled ? '줄자로 잰 거리를 저장합니다.' : note || '계획한 시간까지 이동한 뒤에 저장할 수 있습니다.' } };
}

export function initDriveDistance(page, options = {}) {
  const doc = options.document || document;
  const win = options.window || globalThis;
  const now = options.now || (() => Date.now());
  const setTimer = options.setInterval || ((fn, ms) => setInterval(fn, ms));
  const clearTimer = options.clearInterval || ((id) => clearInterval(id));
  const confirmFn = options.confirm || ((text) => globalThis.confirm(text));
  const debounceMs = options.planDebounceMs ?? PLAN_DEBOUNCE_MS;
  const $ = (id) => doc.getElementById(id);
  const root = $('steerDistPanel');
  const details = $('steerDistDetails');
  if (!root || !details) return null;
  const ui = { terrain: $('steerDistTerrain'), meters: $('steerDistMeters'), duty: $('steerDistDuty'), go: $('steerDistGo'),
               actual: $('steerDistActual'), tapeTerrain: $('steerDistTapeTerrain') };
  // The terrain choice lives in two places (수동 조종, the tape part of 보정·설정): one value.
  const terrainSelects = [ui.terrain, ui.tapeTerrain].filter(Boolean);
  const terrainKey = () => (ui.terrain || ui.tapeTerrain)?.value;
  const terrainLabel = (key) => s.table?.terrains.find((t) => t.key === key)?.label || key || '';
  const directionButtons = [...(root.querySelectorAll?.('[data-dist-direction]') || [])];
  const post = (path, body) => (page.controlRequest || page.postJson)(path, body);
  const s = { table: null, corrections: {}, plan: null, bridge: null, planError: null, move: null, result: null,
              armed: false, external: false, fresh: false, notice: null, tapeNotice: null, direction: 'forward' };
  let planTimer = null, holdTimer = null, pointer = null, moveId = null, starting = false, loaded = false, view = null;
  let dutyTouched = false;
  // Wanted: the folded part is open, or the tape part is in the shown tab (보정·설정).
  const wanted = () => details.open || Boolean(page.shown && $('steerDistTapePanel') && page.shown('steerDistTapePanel'));

  const ctl = () => (page.driveController ? page.driveController() : null);
  function syncCtl() {
    const c = ctl();
    s.armed = Boolean(c && c.lease);
    s.external = Boolean(c && c.external);
    s.fresh = Boolean(c && c.fresh(now()));
  }

  function setText(el, value) { if (el && el.textContent !== value) el.textContent = value; }
  function list(el, rows) {
    if (!el) return;
    const key = JSON.stringify(rows);
    if (el.dataset.key === key) return;
    el.dataset.key = key;
    el.replaceChildren(...rows.map(([name, value]) => {
      const row = doc.createElement('div');
      const dt = doc.createElement('dt'); dt.textContent = name;
      const dd = doc.createElement('dd'); dd.textContent = value;
      row.append(dt, dd);
      return row;
    }));
  }
  function draw() {
    syncCtl();
    s.input = { terrain: terrainKey(), duty: Number(ui.duty?.value) };
    view = distView(s);
    setText($('steerDistSpeed'), view.speed);
    const chip = $('steerDistState');
    if (chip) { chip.className = `status-chip ${view.chip[0]}`.trim(); setText(chip, view.chip[1]); }
    list($('steerDistRows'), $('steerDistMore') ? view.main : view.rows);
    list($('steerDistMore'), view.more);
    if (ui.go) {
      ui.go.disabled = !view.go.enabled && pointer === null; ui.go.title = view.go.title; ui.go.classList.toggle('active', pointer !== null);
      if (pointer === null) setText(ui.go, view.go.text);
    }
    for (const button of directionButtons) {
      const on = button.dataset.distDirection === s.direction;
      button.classList.toggle('active', on);
      button.setAttribute?.('aria-pressed', String(on));
      button.disabled = pointer !== null || moveId !== null;
    }
    const limit = distanceLimit(s.table, s.direction);
    if (ui.meters && ui.meters.max !== String(limit)) ui.meters.max = String(limit);
    setText($('steerDistMetersLabel'), `거리 (0.2~${limit.toFixed(1)} m)`);
    const status = $('steerDistStatus');
    if (status) { status.className = `test-result ${view.status.kind || ''}`.trim(); setText(status, view.status.text); }
    const toTape = $('steerDistToTape');
    if (toTape) toTape.hidden = !view.feedback.enabled;
    // The tape part (보정·설정 tab).
    const tape = tapeView(s);
    setText($('steerDistLast'), tape.last);
    list($('steerDistTapeRows'), tape.rows);
    const tapeChip = $('steerDistTapeState');
    if (tapeChip) { tapeChip.className = `status-chip ${tape.chip[0]}`.trim(); setText(tapeChip, tape.chip[1]); }
    const fb = $('steerDistFeedback');
    if (fb) { fb.disabled = !tape.feedback.enabled; fb.title = tape.feedback.title; }
    const reset = $('steerDistReset');
    if (reset) setText(reset, '줄자 보정 지우기');
    const tapeStatus = $('steerDistTapeStatus');
    const tapeNote = s.tapeNotice || (tape.note ? { kind: 'warn', text: tape.note } : null);
    if (tapeStatus) {
      tapeStatus.hidden = !tapeNote;
      if (tapeNote) { tapeStatus.className = `test-result ${tapeNote.kind || ''}`.trim(); setText(tapeStatus, tapeNote.text); }
    }
  }

  function fillTerrains() {
    if (!terrainSelects.length || !s.table) return;
    for (const select of terrainSelects) {
      select.replaceChildren(...s.table.terrains.map((t) => {
        const opt = doc.createElement('option');
        opt.value = t.key; opt.textContent = `${t.label} (${t.day.slice(5).replace('-', '/')})`;
        return opt;
      }));
    }
    syncTerrain(terrainSelects[0]);
    fillDuties();
  }
  // The other terrain choice follows the one that changed.
  function syncTerrain(from) {
    for (const select of terrainSelects) if (select !== from && select.value !== from.value) select.value = from.value;
  }
  // The speed slider spans the terrain's table duties up to the bridge cap; a duty still inside the new range stays,
  // otherwise the terrain's default (its lowest table duty, the first choice of the 2026-11-17 page).
  function fillDuties() {
    const terrain = s.table?.terrains.find((t) => t.key === terrainKey()) || s.table?.terrains[0];
    if (!ui.duty || !terrain) return;
    const range = sliderRange(terrain, s.bridge);
    if (range.min === null) return;
    const keep = dutyTouched ? Number(ui.duty.value) : NaN;
    ui.duty.min = String(range.min);
    ui.duty.max = String(range.max);
    ui.duty.step = '1';
    const def = Math.min(range.def, range.max);
    ui.duty.value = String(Number.isInteger(keep) && keep >= range.min && keep <= range.max ? keep : def);
  }
  const inputs = () => ({ terrain: terrainKey(), duty: Number(ui.duty?.value), distance_m: Number(ui.meters?.value),
                          ...(s.direction === 'backward' ? { direction: 'backward' } : {}) });

  async function load() {
    try {
      const reply = await page.getJson('/api/rover/distance');
      if (reply && reply.table) {
        s.table = reply.table; s.corrections = reply.corrections || {};
        if (!loaded) { loaded = true; fillTerrains(); }
      }
    } catch (_) { s.notice = { kind: 'bad', text: `속도 표를 읽지 못했습니다. ${message('http_failed')}` }; }
    replan();
  }
  async function replanNow() {
    planTimer = null;
    if (!s.table) return draw();
    try {
      const reply = await post('/api/rover/distance/plan', inputs());
      s.plan = reply?.plan?.speed ? reply.plan : null;
      s.planError = reply?.plan && !reply.plan.speed ? reply.plan.problem : null;
      const capBefore = num(s.bridge?.max_duty);
      s.bridge = reply?.bridge || null;
      if (num(s.bridge?.max_duty) !== capBefore) {           // the bridge cap moved: the slider follows it
        const before = ui.duty?.value;
        fillDuties();
        if (ui.duty && ui.duty.value !== before) { replan(); return; }
      }
    } catch (_) { s.plan = null; s.planError = 'http_failed'; }
    draw();
  }
  function replan() { clearTimeout(planTimer); planTimer = setTimeout(replanNow, debounceMs); draw(); }

  // ---- the held button ----------------------------------------------------------------------------------------
  function finish(reply) {
    const c = ctl();
    if (reply?.move) { s.move = reply.move; s.result = reply.move.state === 'driving' ? null : reply.move; }
    if (reply?.chain) {
      c?.endExternal(reply.chain);
    } else if (reply?.lease_lost) {
      c?.endExternal(null, 'distance_move_lost');
    } else {
      return false;
    }
    clearTimer(holdTimer); holdTimer = null;
    moveId = null;
    if (s.move && s.move.state !== 'driving') s.move = null;
    replan();
    return true;
  }
  async function hold() {
    if (moveId === null) return;
    const c = ctl();
    if (c && !c.external) { stopTimers('stopped'); return; }       // STOP or another stop dropped the lease
    try {
      const reply = await post('/api/rover/distance/hold', { move_id: moveId });
      if (reply && reply.ok) finish(reply);
    } catch (_) { /* the server stops on its own after 0.4 s without a heartbeat */ }
    draw();
  }
  async function start() {
    if (starting || moveId !== null) return;
    const c = ctl();
    if (!view?.go.enabled || !c) return;
    starting = true;
    s.notice = null; s.result = null; s.tapeNotice = null;
    let chain = null;
    for (let i = 0; i < 10 && !chain && pointer !== null; i += 1) {
      chain = c.beginExternal();
      if (!chain) await new Promise((resolve) => setTimeout(resolve, 30));   // a request of the page in flight
    }
    if (!chain) { starting = false; draw(); return; }
    let reply = null;
    try {
      reply = await post('/api/rover/distance/start', { ...inputs(), ...chain });
    } catch (_) { reply = { ok: false, error: 'http_failed' }; }
    starting = false;
    if (!reply || !reply.ok) {
      c.endExternal({ lease: chain.lease, ticket: chain.ticket, seq: chain.seq });
      s.notice = { kind: 'bad', text: `거리 이동을 시작하지 못했습니다. ${sentence(reply?.error)}` };
      replan();
      return;
    }
    moveId = reply.move.id;
    s.move = reply.move;
    holdTimer = setTimer(hold, HOLD_MS);
    if (pointer === null) await release('released');                  // let go while the start was on its way
    draw();
  }
  async function release(why) {
    if (moveId === null) return;
    for (let i = 0; i < RELEASE_RETRIES; i += 1) {
      try {
        const reply = await post('/api/rover/distance/release', { move_id: moveId, reason: why });
        if (reply && reply.ok) {
          if (finish(reply)) return draw();
          if (reply.move && reply.move.state === 'driving') continue;
        }
      } catch (_) { /* try again */ }
    }
    stopTimers('release_failed');
  }
  function stopTimers(why) {
    clearTimer(holdTimer); holdTimer = null;
    const c = ctl();
    if (c && c.external) c.endExternal(null, 'distance_move_lost');
    if (why !== 'stopped' && moveId !== null) s.notice = { kind: 'bad', text: '거리 이동이 끝났는지 확인하지 못했습니다. 조종 활성화를 다시 누르세요.' };
    moveId = null; s.move = null;
    draw();
  }
  function letGo(why) {
    const id = pointer;
    pointer = null;
    if (id !== null && typeof id === 'number') { try { ui.go?.releasePointerCapture?.(id); } catch (_) { /* gone */ } }
    if (moveId !== null) release(why); else draw();
  }

  ui.go?.addEventListener('pointerdown', (event) => {
    event.preventDefault?.();
    if (pointer !== null) return;
    syncCtl(); view = distView(s);
    if (!view.go.enabled) { draw(); return; }
    pointer = event.pointerId;
    try { ui.go.setPointerCapture?.(event.pointerId); } catch (_) { /* its cancel ends it */ }
    start();
  });
  for (const name of ['pointerup', 'pointercancel', 'lostpointercapture']) {
    ui.go?.addEventListener(name, (event) => { if (event.pointerId === pointer) letGo('released'); });
  }
  ui.go?.addEventListener('contextmenu', (event) => event.preventDefault?.());
  $('steerDistStop')?.addEventListener('click', () => {
    pointer = null;
    ctl()?.abandon?.('operator_stop');
    Promise.resolve(post('/api/rover/stop', { reason: 'distance_stop' })).catch(() => null);
    stopTimers('stopped');
  });
  for (const button of directionButtons) {
    button.addEventListener('click', () => {
      const next = button.dataset.distDirection === 'backward' ? 'backward' : 'forward';
      if (next === s.direction || pointer !== null || moveId !== null) return;
      s.direction = next;
      s.plan = null; s.notice = null;
      const limit = distanceLimit(s.table, next);
      if (ui.meters && Number(ui.meters.value) > limit) ui.meters.value = String(limit);
      replan();
    });
  }
  for (const el of terrainSelects) {
    el.addEventListener('change', () => { syncTerrain(el); fillDuties(); s.notice = null; s.tapeNotice = null; replan(); });
  }
  for (const name of ['input', 'change']) ui.duty?.addEventListener(name, () => { dutyTouched = true; });
  for (const el of [ui.duty, ui.meters]) el?.addEventListener('input', () => { s.notice = null; replan(); });
  ui.duty?.addEventListener('change', () => { s.notice = null; replan(); });
  $('steerDistFeedback')?.addEventListener('click', async () => {
    const actual = Number(ui.actual?.value);
    s.input = { terrain: terrainKey(), duty: Number(ui.duty?.value) };
    if (!s.result || !tapeView(s).feedback.enabled) { draw(); return; }      // only a move of the chosen terrain
    try {
      const reply = await post('/api/rover/distance/feedback', { move_id: s.result.id, actual_m: actual });
      if (reply && reply.ok) {
        s.corrections = reply.corrections || s.corrections;
        s.result = { ...s.result, feedback_open: false };
        const c = reply.correction;
        const at = num(s.result.duty) === null ? '' : ` · 출력 ${s.result.duty} %`;
        const way = DIRECTION_TEXT[s.result.direction] || '앞으로';
        const where = terrainLabel(s.result.terrain || terrainKey());
        s.tapeNotice = { kind: 'ok', text: `잰 거리를 저장했습니다. ${where} · ${way}${at} · 잰 거리 ${actual} m · 계산 거리 ${reply.sample.model_m} m · 배율 ${c ? `${fixed(c.factor, 2)} · 줄자 ${c.samples}번` : '--'}` };
      } else s.tapeNotice = { kind: 'bad', text: `잰 거리를 저장하지 못했습니다. ${sentence(reply?.error)}` };
    } catch (_) { s.tapeNotice = { kind: 'bad', text: `잰 거리를 저장하지 못했습니다. ${message('http_failed')}` }; }
    replan();
  });
  $('steerDistReset')?.addEventListener('click', async () => {
    const terrain = terrainKey();
    const way = DIRECTION_TEXT[s.direction];
    if (!terrain || !confirmFn(`${terrainLabel(terrain)}에서 ${way} 갈 때의 줄자 보정을 모든 속도에서 지울까요? 배율이 1.00으로 돌아갑니다.`)) return;
    try {
      const reply = await post('/api/rover/distance/reset', { terrain, direction: s.direction });
      if (reply && reply.ok) { s.corrections = reply.corrections || {}; s.tapeNotice = { kind: 'ok', text: `${terrainLabel(terrain)}에서 ${way} 갈 때의 줄자 보정을 지우고 배율을 1.00으로 되돌렸습니다.` }; }
      else s.tapeNotice = { kind: 'bad', text: `줄자 보정을 지우지 못했습니다. ${sentence(reply?.error)}` };
    } catch (_) { s.tapeNotice = { kind: 'bad', text: `줄자 보정을 지우지 못했습니다. ${message('http_failed')}` }; }
    replan();
  });
  win.addEventListener?.('blur', () => { if (pointer !== null) letGo('blur'); });
  win.addEventListener?.('ubts:rover-tab', () => { if (pointer !== null) letGo('tab_changed'); });
  doc.addEventListener?.('visibilitychange', () => { if (doc.hidden && pointer !== null) letGo('page_hidden'); });
  const wake = () => { if (wanted() && !loaded) load(); else if (wanted()) draw(); };
  details.addEventListener('toggle', wake);
  const redraw = setTimer(() => { if (wanted()) draw(); }, 500);
  if (wanted()) load();
  return { draw, load, view: () => view, state: s, stop: () => clearTimer(redraw), routeChanged: wake };
}
