// What the autonomy section shows and which button it offers, from GET /api/executor/status (UI-08,
// docs/STEERING_SERVER_KO.md 6, contract v0 with revisions 1 and 2). Pure functions: tested without a page.

export const HEARTBEAT_MS = 100;          // contract revision 1: every 100 ms (at most 250 ms)
export const STATUS_MS = 500;             // placeholder (STEERING_SERVER_KO.md 6)
export const STALE_MS = 1500;             // UI-08: no status for 1.5 s -> "끊김", values dimmed
export const HEARTBEAT_STATES = new Set(['armed', 'running', 'paused']);
export const ACTIVE_RUN_STATES = new Set(['ready', 'armed', 'running', 'paused', 'latched']);

export const RUN_STATE = {
  idle: ['', '대기'], ready: ['info', '준비됨 · Arm 전'], armed: ['info', '출발 대기'], running: ['ok', '주행 중'],
  paused: ['warn', '일시정지'], ended: ['', '끝남'], latched: ['bad', '정지 래치'],
};
export const STOP_REASON = {
  ROW_END: '정상 열 끝', OBSTACLE: '열 안 장애물', END_STRUCTURE: '열 끝 구조물(벽·울타리)',
  PERCEPTION_OR_LOCALISATION: '센서·위치 추정 부족', CONTROL_AUTHORITY: '조향·구동 권한 부족',
  OPERATOR_OR_BACKSTOP: '운영자·한도 정지', OPERATOR_LINK_LOST: '화면 연결 끊김(화면 신호 없음)',
  SYSTEM_FAULT: '시스템 오류', UNCLASSIFIED: '분류 안 됨',
};
export const GO_BLOCKER = {
  session_conflict: '다른 화면이 출발 권한을 가짐', operator_link_not_fresh: '이 화면의 연결 신호가 아직 없음',
  no_run: 'run 없음: 먼저 준비', already_running: '이미 주행 중', latched: '래치 확인이 먼저',
  arm_first: 'Arm이 먼저', inputs_not_fresh: '센서 입력이 오래됨', steering_feedback: '조향 상태가 없거나 오래됨',
  vehicle_moving: '차가 아직 움직임', corridor_not_acquired: '통로를 아직 못 찾음',
  lease_lost: '조향 브리지 연결이 끊겨 다시 Arm이 필요함',                  // contract revision 3
};
// stop_detail (the executor's own reason string) -> short words; an unknown one is shown as sent.
export const STOP_DETAIL = {
  operator_stop: '운영자 STOP', operator_link_lost: '화면 연결 끊김', row_end: '열 끝', lease_lost: '브리지가 권한을 거둠',
};
export const LEASE_LOST_TEXT = '브리지가 권한을 거뒀다: Arm을 다시 누른다';
export const GNSS_STATE = {
  none: '수신 없음', no_fix: '미확정', fix: '확정', rtk: 'RTK', stale: '오래됨', unused: '쓰지 않음',
};
export const LIMIT_REASON = {
  target: '목표 속도', corridor: '통로 신뢰도', lateral_error: '횡 오차', heading_error: '방향 오차',
  row_end: '열 끝 감속', steering: '조향 권한', obstacle: '앞 장애물', approach: '열 끝 감속',
};

const num = (value) => (typeof value === 'number' && isFinite(value) ? value : null);
const fixed = (value, digits, unit) => (num(value) === null ? '--' : `${value.toFixed(digits)}${unit}`);
const label = (table, code) => (code === null || code === undefined || code === '' ? null
  : (table[code] || `기타(${String(code)})`));

// The button of the section: [text, action, enabled]. action: prepare | arm | go | ack | null.
export function primaryAction(reply, nowMs = Date.now()) {
  if (!reply) return { text: '확인 중', action: null, enabled: false };
  if (reply.configured === false) return { text: '실행기 설정 없음', action: null, enabled: false };
  if (!reply.ok || !reply.status) return { text: '실행기 연결 안 됨', action: null, enabled: false };
  if (stale(reply, nowMs)) return { text: '실행기 응답 끊김', action: null, enabled: false };
  const s = reply.status;
  const ready = s.go_ready === undefined || s.go_ready === null ? true : s.go_ready === true;
  // contract rev. 2: an executor set to hold_to_run refuses go and heartbeat (409 start_mode_hold_to_run)
  if (s.start_mode === 'hold_to_run' && (s.run_state === 'armed' || s.run_state === 'paused' || s.run_state === 'running')) {
    return { text: '누르는 동안만 방식: 이 화면에서 출발 못 함', action: null, enabled: false };
  }
  switch (s.run_state) {
    case 'idle': case 'ended': case undefined: case null:
      return { text: s.mode === 'MOTOR' ? '준비 (run 만들기 · Arm)' : '준비 (run 만들기)', action: 'prepare', enabled: true };
    case 'ready': return leaseLost(s) ? { text: 'Arm 다시 (브리지가 권한을 거둠)', action: 'arm', enabled: true }
      : { text: 'Arm (모터 권한)', action: 'arm', enabled: true };
    case 'armed': return { text: '자율 주행 시작', action: 'go', enabled: ready };
    case 'running': return { text: '주행 중', action: null, enabled: false };
    case 'paused': return { text: '다시 출발', action: 'go', enabled: ready };
    case 'latched': return { text: '래치 확인', action: 'ack', enabled: true };
    default: return { text: `상태 ${String(s.run_state)}`, action: null, enabled: false };
  }
}

// "준비" in one press (contract v0): start, then Arm when the executor says the run is ready. The start reply is not
// enough to decide: the status it carries can come from before the start, and a status poll already in flight when
// the start was sent can answer with the old state. So: read the status again after the start reply until it says
// ready (MOTOR: send arm) or armed (SHADOW: start arms by itself), at most waitMs. Only a run state that the start
// itself makes (ready, armed) counts; idle or ended means "not yet". Returns { ok, step, runState }.
export const PREPARE_WAIT_MS = 3000;
export const PREPARE_STEP_MS = 150;
export async function prepareRun({ send, readStatus, sleep, rowLen = null, waitMs = PREPARE_WAIT_MS, stepMs = PREPARE_STEP_MS,
                                   now = () => Date.now() }) {
  const started = await send('start', { row_len_m: rowLen });
  if (!started.ok) return { ok: false, step: 'start', runState: null };
  const t0 = now();
  let status = started.reply && typeof started.reply.status === 'object' ? started.reply.status : null;
  for (;;) {
    const runState = status ? status.run_state : null;
    if (runState === 'armed') return { ok: true, step: 'armed', runState };
    if (runState === 'ready') {
      const armed = await send('arm', {});
      return { ok: armed.ok, step: 'arm', runState };
    }
    if (runState === 'latched') return { ok: false, step: 'latched', runState };
    if (now() - t0 >= waitMs) return { ok: false, step: 'wait', runState };
    await sleep(stepMs);
    const fresh = await readStatus();
    status = fresh && fresh.ok && fresh.status ? fresh.status : null;
  }
}

// Contract revision 3: the bridge took the lease back while the run waited for its first go; the run stays open.
export function leaseLost(s) {
  return Boolean(s && s.run_state === 'ready' && ((s.lease_lost && typeof s.lease_lost === 'object')
    || (Array.isArray(s.go_blockers) && s.go_blockers.includes('lease_lost'))));
}

export function stale(reply, nowMs = Date.now()) {
  const at = reply && num(reply.received_at);
  return at === null || nowMs - at * 1000 > STALE_MS;
}

export function heartbeatWanted(reply, nowMs = Date.now()) {
  return Boolean(reply && reply.ok && reply.status && !stale(reply, nowMs) && HEARTBEAT_STATES.has(reply.status.run_state)
    && reply.status.start_mode !== 'hold_to_run');
}

export function runActive(reply) {
  return Boolean(reply && reply.ok && reply.status && ACTIVE_RUN_STATES.has(reply.status.run_state));
}

// The chip and the rows of the section.
export function autoView(reply, nowMs = Date.now()) {
  if (!reply) return { chip: ['', '확인 중'], notice: ['testing', '자율 실행기 상태를 확인하고 있다.'], rows: [], stale: true };
  if (reply.configured === false) {
    return { chip: ['', '설정 없음'], rows: [], stale: true,
             notice: ['warn', '자율 실행기 설정 없음: 이 대시보드에는 UBTS_EXECUTOR_URL이 없다. 수동 조종과 STOP은 그대로 쓴다.'] };
  }
  if (!reply.ok || !reply.status) {
    const why = reply.error === 'executor_token_rejected' || reply.error === 'executor_token_missing'
      ? '토큰이 맞지 않음(실행기가 다시 시작됨)' : '실행기에 닿지 않음';
    return { chip: ['bad', '연결 안 됨'], rows: [], stale: true,
             notice: ['bad', `자율 실행기 연결 안 됨: ${why}. 주행 중이었다면 차는 스스로 선다. STOP은 브리지로도 보낸다.`] };
  }
  const s = reply.status;
  const isStale = stale(reply, nowMs);
  const [level, text] = RUN_STATE[s.run_state] || ['', String(s.run_state || '알 수 없음')];
  const sr = s.single_row || {};
  const link = s.operator_link || {};
  const speed = s.speed || {};
  const gnss = s.gnss || {};
  const category = s.stop_reason || sr.stop_category || null;
  const stopText = label(STOP_REASON, category);
  const rows = [
    ['모드', s.mode === 'SHADOW' ? 'SHADOW · 차를 몰지 않음' : s.mode === 'MOTOR' ? 'MOTOR · 차를 몬다' : (s.mode || '--')],
    ['정지 이유', stopText ? (s.stop_detail ? `${stopText} · ${STOP_DETAIL[s.stop_detail] || s.stop_detail}` : stopText) : '--'],
    ['횡 · 방향 오차', `${fixed(sr.lateral_error_m, 2, ' m')} · ${fixed(sr.heading_error_deg, 1, '°')}`],
    ['속도', `${fixed(sr.speed_m_s, 2, '')} / 목표 ${fixed(num(speed.target_mps) ?? num(sr.target_speed_m_s), 2, '')} m/s`],
    ['속도 제한', num(speed.limit_mps) === null ? '--'
      : `${fixed(speed.limit_mps, 2, ' m/s')} · ${label(LIMIT_REASON, speed.limit_reason) || '--'}`],
    ['진행 거리', fixed(sr.distance_m, 1, ' m')],
    ['GNSS', gnss.state ? `${label(GNSS_STATE, gnss.state)}${num(gnss.h_acc_m) !== null ? ` · ±${gnss.h_acc_m.toFixed(1)} m` : ''}` : '정보 없음'],
    ['화면 연결', link.fresh === true ? `연결됨 · ${fixed(link.age_s, 2, ' s')}`
      : link.fresh === false ? '끊김' : '--'],
  ];
  let notice = null;
  if (isStale) notice = ['bad', '실행기 상태가 1.5초 넘게 오지 않는다: 값이 오래됐다. 주행 중이었다면 차는 스스로 선다.'];
  else if (leaseLost(s)) notice = ['warn', `${LEASE_LOST_TEXT}. ${GO_BLOCKER.lease_lost}.`];
  else if (Array.isArray(s.go_blockers) && s.go_blockers.length && (s.run_state === 'armed' || s.run_state === 'paused')) {
    notice = ['warn', `출발 조건: ${s.go_blockers.map((code) => label(GO_BLOCKER, code)).join(', ')}`];
  } else if (s.run_state === 'running') {
    const timeout = num(link.timeout_s);
    notice = ['ok', `주행 중: 이 화면을 열어 두어야 한다. 탭을 숨기거나 화면을 떠나면 ${timeout === null ? '0.5' : timeout.toFixed(1)}초 뒤 차가 선다.`];
  } else if (s.run_state === 'paused' && category === 'OPERATOR_LINK_LOST') {
    notice = ['warn', '화면 연결이 끊겨 멈췄다. 다시 가려면 다시 출발을 누른다(자동 재출발 없음).'];
  }
  return { chip: isStale ? ['bad', '끊김'] : [level, text], notice, rows, stale: isStale };
}

// A random session id of the operator link (contract: 4 to 64 characters).
export function newSession(random = globalThis.crypto) {
  const bytes = new Uint8Array(12);
  random.getRandomValues(bytes);
  return Array.from(bytes, (b) => b.toString(16).padStart(2, '0')).join('');
}
