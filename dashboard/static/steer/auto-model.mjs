// What the autonomy tab shows and which of its three buttons are on, from GET /api/executor/status (UI-08,
// docs/STEERING_SERVER_KO.md 6, contract v0 with revisions 1 and 2). Pure functions: tested without a page.
// 2026-11-20 (user: "준비 → 열을 잡아서 자율주행 가능한 상태면 자율주행 버튼 활성화, 멈추기 버튼 누르면 멈추는, 최대한 단순한
// 과정"): three buttons that stay in place (준비, 자율 주행 시작, 멈추기) and one status sentence (statusLine). 준비 does every
// step by itself (prepareSequence: acknowledge a latch, end an open run, start, arm).

export const HEARTBEAT_MS = 100;          // contract revision 1: every 100 ms (at most 250 ms)
export const STATUS_MS = 500;             // placeholder (STEERING_SERVER_KO.md 6)
export const STALE_MS = 1500;             // UI-08: no status for 1.5 s -> "끊김", values dimmed
export const HEARTBEAT_STATES = new Set(['armed', 'running', 'paused']);
export const ACTIVE_RUN_STATES = new Set(['ready', 'armed', 'running', 'paused', 'latched']);

export const RUN_STATE = {
  idle: ['', '대기'], ready: ['info', '준비 미완료'], armed: ['info', '출발 대기'], running: ['ok', '주행 중'],
  paused: ['warn', '일시정지'], ended: ['', '끝남'], latched: ['bad', '멈춤'],
};
export const STOP_REASON = {
  ROW_END: '열 끝 도착', OBSTACLE: '열 안 장애물', END_STRUCTURE: '열 끝 구조물',
  PERCEPTION_OR_LOCALISATION: '센서·위치 추정 부족', CONTROL_AUTHORITY: '조향·구동 응답 부족',
  OPERATOR_OR_BACKSTOP: 'STOP 또는 거리 한도', OPERATOR_LINK_LOST: '화면 연결 끊김',
  SYSTEM_FAULT: '시스템 오류', UNCLASSIFIED: '이유 모름',
};
export const GO_BLOCKER = {
  session_conflict: '다른 화면에서 조종하고 있습니다.', operator_link_not_fresh: '이 화면의 연결을 확인하고 있습니다.',
  no_run: '시작된 주행이 없습니다. 준비를 누르세요.', already_running: '이미 주행 중입니다.', latched: '멈춤 상태입니다. 준비를 누르세요.',
  arm_first: '준비가 끝나지 않았습니다. 준비를 누르세요.', inputs_not_fresh: '센서 입력을 기다리고 있습니다.', steering_feedback: '조향 상태를 기다리고 있습니다.',
  vehicle_moving: '차가 멈추기를 기다리고 있습니다.', corridor_not_acquired: '통로를 찾고 있습니다.',
  lease_lost: '브리지가 조종 권한을 거두었습니다. 준비를 다시 누르세요.',                  // contract revision 3
};
// stop_detail (the executor's own reason string) -> short words; an unknown one is shown as sent.
export const STOP_DETAIL = {
  operator_stop: 'STOP 누름', operator_link_lost: '화면 연결 끊김', row_end: '열 끝 도착', lease_lost: '조종 권한 거둠',
  // executor pause / end reasons (simulation repository tools/row_executor/core.py)
  stale_lidar: 'LiDAR 입력 끊김', stale_imu: 'IMU 입력 끊김', speed_cap_below_min: '속도 제한이 최저 속도보다 낮음',
  overspeed: '과속', prediction_limit: '예측 한도 도달', obstacle_ahead: '앞 장애물', stale_repeat_limit: '잠시 멈춤 반복',
  gnss_geofence: 'GPS 범위 벗어남', bridge_error: '브리지 오류', bridge_latched: '브리지 멈춤 상태', bridge_arm_failed: '조종 권한 받기 실패',
};
// Why the run stopped as the first half of a sentence ("... 멈췄습니다."): the detail, else the category (stopClause).
export const STOP_CLAUSE = {
  operator_stop: 'STOP을 눌러', operator_link_lost: '화면 연결이 끊겨', row_end: '열 끝에 도착해', lease_lost: '브리지가 조종 권한을 거두어',
  stale_lidar: 'LiDAR 입력이 끊겨', stale_imu: 'IMU 입력이 끊겨', speed_cap_below_min: '속도 제한이 최저 속도보다 낮아',
  overspeed: '속도가 한도를 넘어', prediction_limit: '위치 예측이 한도에 닿아', obstacle_ahead: '앞에 장애물이 있어',
  stale_repeat_limit: '잠시 멈춤이 여러 번 반복되어', gnss_geofence: 'GPS 위치가 열 범위를 벗어나', bridge_error: '브리지 오류로',
  bridge_latched: '브리지가 멈춤 상태여서', bridge_arm_failed: '조종 권한을 받지 못해',
  ROW_END: '열 끝에 도착해', OBSTACLE: '열 안에 장애물이 있어', END_STRUCTURE: '열 끝 구조물 앞이어서',
  PERCEPTION_OR_LOCALISATION: '센서나 위치 추정이 부족해', CONTROL_AUTHORITY: '조향이나 구동이 명령을 따르지 못해',
  OPERATOR_OR_BACKSTOP: 'STOP을 누르거나 거리 한도에 닿아', OPERATOR_LINK_LOST: '화면 연결이 끊겨', SYSTEM_FAULT: '시스템 오류로',
};
export const GNSS_STATE = {
  none: '수신 없음', no_fix: '미확정', fix: '확정', rtk: 'RTK', stale: '오래됨', unused: '쓰지 않음', off: '꺼짐',
  absent: '수신 없음', waiting: '대기', ok: '정상', accepted: '받아들임', rejected: '거부', fence_tripped: '범위 벗어남',
  fixed: 'RTK FIXED', float: 'RTK FLOAT',
  // executor gnss_status states (tools/row_executor/core.py) and the RTK constraint states (gnss_rtk.py)
  no_run: '주행 없음', rejected_no_fix: '위치 미확정이라 쓰지 않음', rejected_hacc: '정확도가 낮아 쓰지 않음',
  rejected_jump: '위치가 급변해 쓰지 않음', rejected_quality: '품질이 낮아 쓰지 않음', learning: 'RTK 맞추는 중',
  active: 'RTK 보정 중', fading: 'RTK 보정 줄이는 중', refused: 'RTK 보정 안 함',
};
// The run's GPS mode (2026-11-19, tools/row_executor/config.py gnss_mode): the choice of the autonomy start.
export const GNSS_MODES = ['off', 'log_only', 'geofence', 'weak_constraint'];
export const GNSS_MODE_TEXT = {
  // the start screen offers off / weak_constraint only (2026-11-20); the other two can still come from the executor
  off: 'GPS 사용 안 함', log_only: 'GPS 기록만', geofence: 'GPS 범위 감시', weak_constraint: 'GPS 사용함',
};
// Refusals of a start (the executor's own codes, and the dashboard's 400) as the operator reads them.
export const START_ERROR = {
  invalid_gnss_mode: 'GPS 설정 값이 올바르지 않습니다.', invalid_row_len: '열 길이는 0보다 크고 1000 m 이하여야 합니다.',
  unknown_parameter: '이 실행기는 GPS 선택을 받지 않습니다. 실행기를 업데이트하세요.',
  gnss_mode_not_supported: '이 실행기는 GPS 선택을 받지 않습니다. 실행기를 업데이트하세요.',
  gnss_mode_invalid: 'GPS 설정 값이 올바르지 않습니다.',
  gnss_mode_unsupported: '실행기 설정에서 GPS 사용이 꺼져 있습니다. 설정의 gnss_run_modes에 weak_constraint를 넣으세요.',
  gnss_rtk_not_fixed: 'RTK FIXED 상태가 아니어서 RTK 보정을 쓸 수 없습니다.', gnss_needs_rtk_fixed: 'RTK FIXED 상태가 아니어서 RTK 보정을 쓸 수 없습니다.',
  gnss_not_available: 'GPS 수신이 없습니다.', gnss_mode_needs_front_steer: '이 GPS 설정은 앞바퀴 조향 실행기에서만 쓸 수 있습니다.',
};
// Refusals of the other actions (executor request handlers, tools/row_executor/executor_main.py) in screen words.
export const EXEC_ERROR = {
  run_in_progress: '이전 주행이 아직 끝나지 않았습니다.', bridge_still_armed: '차가 아직 멈추는 중입니다.',
  acknowledge_latch_first: '멈춤 상태를 먼저 해제해야 합니다.', start_a_run_first: '시작된 주행이 없습니다.', already_armed: '이미 준비되어 있습니다.',
  motor_output_not_allowed: '이 실행기는 모터를 쓰지 않도록 되어 있습니다.', row_motor_output_not_enabled: '실행기 설정에서 모터 출력이 꺼져 있습니다.',
  config_not_feasible: '실행기 설정 값이 서로 맞지 않습니다.', no_run: '시작된 주행이 없습니다.', invalid_session_id: '화면 연결 정보가 올바르지 않습니다.',
  start_mode_hold_to_run: '이 실행기는 버튼을 누르는 동안만 가는 방식입니다.', control_in_use: '다른 화면에서 조종하고 있습니다.',
  executor_unreachable: '실행기에 연결하지 못했습니다.', executor_not_configured: '실행기가 설정되어 있지 않습니다.',
  still_latched: '멈춤 상태가 해제되지 않았습니다.', still_open: '이전 주행이 끝나지 않았습니다.',       // prepareSequence's own
};
export const LIMIT_REASON = {
  target: '목표 속도', corridor: '통로 신뢰도', lateral_error: '횡 오차', heading_error: '방향 오차',
  row_end: '열 끝 감속', steering: '조향 여유', obstacle: '앞 장애물', approach: '열 끝 감속',
};
// PW-10 (contract revision 6): why the online scale is held, and why its offer can not be taken.
export const SCALE_HOLD = {
  lidar_stale: 'LiDAR 스캔 없음', lidar_unmatched: 'LiDAR 속도 품질 낮음', not_driving: '주행 중 아님',
  no_status: '구동 상태 없음', monitor_off: '감시 꺼짐',
};
export const SCALE_WHY_NOT = {
  no_scale: '주행이 10초보다 짧아 배율이 아직 없습니다.', no_forward_motion: 'LiDAR에서 앞으로 간 움직임이 확인되지 않았습니다.',
  scale_set_in_config: '실행기 설정 파일에 배율이 따로 정해져 있습니다.', out_of_range: '제안된 배율이 허용 범위 밖입니다.', no_change: '지금 배율과 같습니다.',
  no_field_tuning: '실행기에 현장 조정 값이 없습니다.',
};

const num = (value) => (typeof value === 'number' && isFinite(value) ? value : null);
const fixed = (value, digits, unit) => (num(value) === null ? '--' : `${value.toFixed(digits)}${unit}`);
const label = (table, code) => (code === null || code === undefined || code === '' ? null
  : (table[code] || `기타(${String(code)})`));

// ---- The three buttons -----------------------------------------------------------------------------------------------
// Start conditions that the press itself clears: the page sends its heartbeat right before the go.
export const GO_PRESS_CLEARS = new Set(['operator_link_not_fresh']);

// The executor's start conditions that are still open (without those of GO_PRESS_CLEARS); null when it sends no list.
export function openBlockers(s) {
  return s && Array.isArray(s.go_blockers) ? s.go_blockers.filter((code) => !GO_PRESS_CLEARS.has(code)) : null;
}

// True when a go may be sent: a run that is armed (or paused: 다시 출발) and nothing in its way but the operator link.
// An executor without the list is judged by go_ready (absent = ready, as before). hold_to_run (contract rev. 2): never.
export function goAllowed(s) {
  if (!s || s.start_mode === 'hold_to_run' || (s.run_state !== 'armed' && s.run_state !== 'paused')) return false;
  if (Array.isArray(s.go_blockers) && s.go_blockers.length) return openBlockers(s).length === 0;
  return s.go_ready === undefined || s.go_ready === null ? true : s.go_ready === true;
}

// The status of a reply that can be acted on (configured, reachable, not older than STALE_MS), else null.
function usable(reply, nowMs) {
  return reply && reply.configured !== false && reply.ok && reply.status && !stale(reply, nowMs) ? reply.status : null;
}

// Which buttons are on. 준비: whenever the executor answers and the vehicle is not driving. 자율 주행 시작 (다시 출발 after a
// pause): goAllowed. 멈추기 is always on (it is not in this table).
export function autoButtons(reply, nowMs = Date.now()) {
  const s = usable(reply, nowMs);
  return { prepare: Boolean(s) && s.run_state !== 'running', go: goAllowed(s),
           goText: s && s.run_state === 'paused' ? '다시 출발' : '자율 주행 시작' };
}

// ---- The status sentence -----------------------------------------------------------------------------------------------
// Why the run stopped, in the operator's words: the known detail, else the category; null when there is no reason.
export function stopWords(s) {
  const category = s.stop_reason || (s.single_row || {}).stop_category || null;
  if (category === 'ROW_END') return '열 끝 도착';
  if (typeof s.stop_detail === 'string' && STOP_DETAIL[s.stop_detail]) return STOP_DETAIL[s.stop_detail];
  return label(STOP_REASON, category);
}

// Why the run stopped as the first half of a sentence: the known detail, else the category; null when there is no
// reason (the same order as stopWords). An unknown one reads "알 수 없는 이유로"; its code is in the folded rows.
export function stopClause(s) {
  const category = s.stop_reason || (s.single_row || {}).stop_category || null;
  if (category === 'ROW_END') return STOP_CLAUSE.ROW_END;
  if (typeof s.stop_detail === 'string' && STOP_DETAIL[s.stop_detail] && STOP_CLAUSE[s.stop_detail]) return STOP_CLAUSE[s.stop_detail];
  if (!category) return null;
  return STOP_CLAUSE[category] || '알 수 없는 이유로';
}

// A refusal reason as a sentence: a sentence of the tables as it is; anything else (an unknown code) is named in one.
export const reasonSentence = (text) => (!text ? '이유는 알 수 없습니다.' : /[.?]\s*$/.test(text) ? text : `알려 온 이유는 ${text}입니다.`);

// What the start still waits for (the open start conditions): at most `most` of them, one sentence each (every one
// is in the folded rows).
function waitWords(s, most = 2) {
  const open = openBlockers(s) || [];
  if (!open.length) return '출발 조건을 확인하고 있습니다.';
  return [...new Set(open.slice(0, most).map((code) => GO_BLOCKER[code] || '출발 조건이 아직 맞지 않습니다.'))].join(' ');
}

// A time in words: 35초, 1분 45초, 1시간 2분.
export function durationText(seconds) {
  const total = Math.max(0, Math.round(seconds));
  if (total < 60) return `${total}초`;
  const minutes = Math.floor(total / 60) % 60, rest = total % 60;
  if (total < 3600) return `${minutes}분${rest ? ` ${rest}초` : ''}`;
  return `${Math.floor(total / 3600)}시간${minutes ? ` ${minutes}분` : ''}`;
}

// How long and how far the run of this status drove (2026-11-20, user request: show for how many seconds it drove):
// { time_s, distance_m, part, source }. The time is the executor's own count of the driving ticks (status.metrics
// time_s: standing still does not count, a start again goes on counting), so a reload of the page loses nothing.
// part: { time_s, distance_m } of the stretch since the last 다시 출발, from the server's record of that stretch
// (segment: /api/recordings/status autonomy_segment; null for a run that was not started again). An executor without
// the metrics: the server's clock of the stretches (source 'clock'). null fields = not known.
export function runTimes(reply, segment = null) {
  const s = reply && reply.ok && reply.status ? reply.status : null;
  if (!s) return { time_s: null, distance_m: null, part: null, source: null };
  const sr = s.single_row || {};
  const run = num(s.run);
  const m = s.metrics && typeof s.metrics === 'object' && (num(s.metrics.run_id) === null || run === null || s.metrics.run_id === run) ? s.metrics : null;
  const seg = segment && typeof segment === 'object' && run !== null && segment.run === run ? segment : null;
  const start = (seg && seg.start) || {};
  const distance = num(sr.distance_m) ?? (m ? num(m.distance_m) : null) ?? num(s.dist);
  let time = m ? num(m.time_s) : null, source = time === null ? null : 'executor';
  if (time === null && seg) {
    const went = num(seg.go?.epoch_s), at = num(reply.received_at);
    if (seg.end) time = num(seg.run_drive_time_s);
    else if (num(start.run_drive_time_s) !== null) time = start.run_drive_time_s + (s.run_state === 'running' && went !== null && at !== null ? Math.max(0, at - went) : 0);
    source = time === null ? null : 'clock';
  }
  let part = null;
  if (seg && (seg.resumed === true || (num(seg.segment) ?? 0) >= 2)) {
    const t = seg.end ? num(seg.drive_time_s) : time !== null && num(start.run_drive_time_s) !== null ? Math.max(0, time - start.run_drive_time_s) : null;
    const d = seg.end ? num(seg.distance_m) : distance !== null && num(start.run_distance_m) !== null ? Math.max(0, distance - start.run_distance_m) : null;
    if (t !== null) part = { time_s: t, distance_m: d };
  }
  return { time_s: time, distance_m: distance, part, source };
}

// The run's driving in a sentence (after it stopped or paused); '' when it did not drive or the time is not known.
// A run that was started again says the whole run first ("모두") and then the stretch since the last start.
export function tripSentence(times) {
  if (!times || num(times.time_s) === null || Math.round(times.time_s) < 1) return '';
  const said = (t, d) => (num(d) === null ? `${durationText(t)} 동안 주행했습니다.` : `${durationText(t)} 동안 ${d.toFixed(1)} m를 주행했습니다.`);
  if (!times.part) return said(times.time_s, times.distance_m);
  return `모두 ${said(times.time_s, times.distance_m)} 다시 출발한 뒤로는 ${said(times.part.time_s, times.part.distance_m)}`;
}

// One sentence: the state now and what to do next. Returns [level, text]; level: '' | ok | warn | bad | testing.
// segment: the server's record of the newest recorded stretch (runTimes), or null.
export function statusLine(reply, nowMs = Date.now(), segment = null) {
  if (!reply) return ['', '자율주행 실행기를 확인하고 있습니다.'];
  if (reply.configured === false) return ['warn', '자율주행 실행기가 설정되어 있지 않아 자율주행을 쓸 수 없습니다.'];
  if (!reply.ok || !reply.status) {
    const token = reply.error === 'executor_token_rejected' || reply.error === 'executor_token_missing';
    return ['bad', token ? '자율주행 실행기에 연결하지 못했습니다. 토큰이 맞지 않습니다.' : '자율주행 실행기에 연결하지 못했습니다.'];
  }
  if (stale(reply, nowMs)) return ['bad', '자율주행 실행기의 응답이 끊겼습니다. 주행 중이었다면 차는 스스로 멈춥니다.'];
  const s = reply.status;
  if (s.start_mode === 'hold_to_run' && HEARTBEAT_STATES.has(s.run_state)) return ['warn', '이 실행기는 버튼을 누르는 동안만 가는 방식이라 이 화면에서는 출발할 수 없습니다.'];
  const shadow = s.mode === 'SHADOW' ? ' 연습 모드라서 차는 움직이지 않습니다.' : '';
  const why = s.stop_detail === 'operator_end' ? null : stopClause(s);
  const sr = s.single_row || {};
  const again = '다시 하려면 준비를 누르세요.';
  const times = runTimes(reply, segment);
  const trip = tripSentence(times), tripAfter = trip ? ` ${trip}` : '';
  switch (s.run_state) {
    case 'idle': case undefined: case null: return ['', `준비를 누르세요.${shadow}`];
    case 'ended':
      if (!why) return ['', `준비를 누르세요.${shadow}`];
      return (s.stop_reason || sr.stop_category) === 'ROW_END' ? ['ok', `${why} 멈췄습니다.${tripAfter}`] : ['warn', `${why} 멈췄습니다.${tripAfter} ${again}`];
    case 'latched': return ['bad', `${why ? `${why} 멈췄습니다.` : '멈췄습니다.'}${tripAfter} ${again}`];
    case 'ready': return ['warn', leaseLost(s) ? GO_BLOCKER.lease_lost : '준비가 끝나지 않았습니다. 준비를 다시 누르세요.'];
    case 'armed': return goAllowed(s) ? ['ok', `출발할 수 있습니다.${shadow}`] : ['testing', waitWords(s)];
    case 'running': {
      // the distance and the time are the whole run's (the tile says the stretch since a start again)
      const distance = num(sr.distance_m) ?? num(s.dist), speed = num(sr.speed_m_s) ?? num(s.v);
      return ['ok', ['주행 중', distance === null ? null : `${distance.toFixed(1)} m`,
                     speed === null ? null : `${speed.toFixed(2)} m/s`,
                     times.time_s === null ? null : durationText(times.time_s)].filter(Boolean).join(' · ')];
    }
    case 'paused': return ['warn', `${why ? `${why} 잠시 멈췄습니다.` : '잠시 멈췄습니다.'}${tripAfter} ${goAllowed(s) ? '다시 출발을 누르세요.' : waitWords(s, 1)}`];
    default: return ['', '실행기 상태를 알 수 없습니다.'];
  }
}

// ---- 준비 in one press ---------------------------------------------------------------------------------------------------
// The body of a start. gnssMode (2026-11-19): the run's GPS mode (GNSS_MODES); null = the executor's configured mode
// (the field is not sent).
function startBody(rowLen, gnssMode) {
  const body = { row_len_m: rowLen };
  if (gnssMode) body.gnss_mode = gnssMode;
  return body;
}
// The refusal of a request as a result carries it ({error: code, text: words}; nothing when the sender gave none).
const refusal = (answer) => ({ ...(answer && answer.error ? { error: answer.error } : {}), ...(answer && answer.text ? { text: answer.text } : {}) });

// After a start the executor accepted (contract v0): Arm when the executor says the run is ready. The start reply is
// not enough to decide: the status it carries can come from before the start, and a status poll already in flight when
// the start was sent can answer with the old state. So: read the status again after the start reply until it says
// ready (MOTOR: send arm) or armed (SHADOW: start arms by itself), at most waitMs. Only a run state that the start
// itself makes (ready, armed) counts; idle or ended means "not yet". Returns { ok, step, runState[, error, text] }.
export const PREPARE_WAIT_MS = 3000;
export const PREPARE_STEP_MS = 150;
export const ARM_SHOWN_MS = 2000;
export async function armAfterStart({ started, send, readStatus, sleep, aborted = () => false, waitMs = PREPARE_WAIT_MS,
                                      stepMs = PREPARE_STEP_MS, now = () => Date.now() }) {
  const t0 = now();
  let status = started.reply && typeof started.reply.status === 'object' ? started.reply.status : null;
  for (;;) {
    const runState = status ? status.run_state : null;
    if (runState === 'armed') return { ok: true, step: 'armed', runState };
    if (runState === 'ready') {
      if (aborted()) return { ok: false, step: 'aborted', runState };
      const armed = await send('arm', {});
      if (!armed.ok) return { ok: false, step: 'arm', runState, ...refusal(armed) };
      // The arm was accepted: stay in this step until the executor's status shows it (at most ARM_SHOWN_MS). Without
      // this the status line read "준비가 끝나지 않았습니다" for the half second until the next status read.
      for (const from = now(); now() - from < ARM_SHOWN_MS;) {
        await sleep(stepMs);
        const fresh = await readStatus();
        const shown = fresh && fresh.ok && fresh.status ? fresh.status.run_state : null;
        if (shown && shown !== 'ready') break;
      }
      return { ok: true, step: 'arm', runState };
    }
    if (runState === 'latched') return { ok: false, step: 'latched', runState };
    if (now() - t0 >= waitMs) return { ok: false, step: 'wait', runState };
    await sleep(stepMs);
    const fresh = await readStatus();
    status = fresh && fresh.ok && fresh.status ? fresh.status : null;
  }
}

// 준비 (2026-11-20): one press does what is needed, each step only when the executor's state asks for it:
//   latched            -> ack (the operator's acknowledgement of the stop)
//   ready/armed/paused -> end (a run that is still open)
//   then               -> start (row length, GPS mode) and Arm (armAfterStart)
// After an end the bridge stays armed until the vehicle stands (executor terminal_stop_wait_s): a start refused with
// bridge_still_armed is sent again every retryMs. A start refused because of a latch or an open run goes back to that
// step (each at most twice). The whole press takes at most totalMs. send(action, body) -> { ok, reply, error, text };
// readStatus() -> a fresh status reply or null; aborted() -> true once 멈추기 was pressed (nothing more is sent);
// onStep(name) names the step for the status line. Returns { ok, step, runState[, error, text] }; step: status |
// running | ack | end | start | arm | armed | latched | wait | aborted.
export const PREPARE_TOTAL_MS = 12000;
export const START_RETRY_MS = 300;
export const PREPARE_STEP = { ack: '멈춤 상태를 해제하고 있습니다.', end: '이전 주행을 끝내고 있습니다.', start: '새 주행을 시작하고 있습니다.',
                              arm: '조종 권한을 받고 있습니다.' };
const OPEN_RUN_STATES = new Set(['ready', 'armed', 'paused']);
export async function prepareSequence({ send, readStatus, sleep, rowLen = null, gnssMode = null, aborted = () => false,
                                        onStep = () => {}, waitMs = PREPARE_WAIT_MS, stepMs = PREPARE_STEP_MS,
                                        totalMs = PREPARE_TOTAL_MS, retryMs = START_RETRY_MS, now = () => Date.now() }) {
  const t0 = now();
  const read = async () => { const fresh = await readStatus(); return fresh && fresh.ok && fresh.status ? fresh.status : null; };
  // Reads until done(status) or waitMs: the last status read (null when none came).
  const settle = async (done) => {
    const from = now();
    let seen = null;
    for (;;) {
      await sleep(stepMs);
      seen = (await read()) || seen;
      if ((seen && done(seen)) || now() - from >= waitMs) return seen;
    }
  };
  let status = await read();
  if (!status) return { ok: false, step: 'status', runState: null };
  let acks = 0, ends = 0, last = {};
  for (;;) {
    const runState = status.run_state ?? null;
    if (aborted()) return { ok: false, step: 'aborted', runState };
    if (now() - t0 >= totalMs) return { ok: false, step: 'wait', runState, ...last };
    if (runState === 'running') return { ok: false, step: 'running', runState };
    if (runState === 'latched') {
      if (acks >= 2) return { ok: false, step: 'ack', runState, error: 'still_latched' };
      acks += 1;
      onStep('ack');
      const acked = await send('ack', {});
      if (!acked.ok) return { ok: false, step: 'ack', runState, ...refusal(acked) };
      status = (await settle((s) => s.run_state !== 'latched')) || status;
      continue;
    }
    if (OPEN_RUN_STATES.has(runState)) {
      if (ends >= 2) return { ok: false, step: 'end', runState, error: 'still_open' };
      ends += 1;
      onStep('end');
      const ended = await send('end', {});
      if (!ended.ok && ended.error !== 'no_run') return { ok: false, step: 'end', runState, ...refusal(ended) };
      status = (await settle((s) => !OPEN_RUN_STATES.has(s.run_state))) || status;
      continue;
    }
    onStep('start');
    const started = await send('start', startBody(rowLen, gnssMode));
    if (!started.ok) {
      last = refusal(started);
      if (started.error === 'bridge_still_armed') { await sleep(retryMs); status = (await read()) || status; continue; }
      if (started.error === 'acknowledge_latch_first') { status = { ...status, run_state: 'latched' }; continue; }
      if (started.error === 'run_in_progress') { status = { ...status, run_state: 'paused' }; continue; }
      return { ok: false, step: 'start', runState, ...last };
    }
    onStep('arm');
    return armAfterStart({ started, send, readStatus, sleep, aborted, waitMs, stepMs, now });
  }
}

// A failed 준비 as one sentence: where and why. words(code) -> the refusal in screen words.
export function prepareFailText(result, words = (code) => code) {
  const raw = result.text || (result.error ? words(result.error) : '');
  const why = raw ? reasonSentence(raw) : '';
  switch (result.step) {
    case 'status': return '준비하지 못했습니다. 실행기 상태를 읽지 못했습니다.';
    case 'running': return '주행 중에는 준비할 수 없습니다. 먼저 멈추기를 누르세요.';
    case 'aborted': return '멈추기를 눌러 준비를 그만두었습니다.';
    case 'latched': return '준비하지 못했습니다. 실행기가 다시 멈춤 상태가 됐습니다.';
    case 'wait': return `시간 안에 준비를 끝내지 못했습니다. ${why || '실행기 상태는 자세히에서 확인하세요.'}`;
    default: return `준비하지 못했습니다. ${why || '이유는 알 수 없습니다.'}`;
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

// The state chip (tab badge, monitoring head) and the rows of the folded 자세히: the executor's own codes are here.
export function autoView(reply, nowMs = Date.now()) {
  if (!reply) return { chip: ['', '확인 중'], rows: [], stale: true };
  if (reply.configured === false) return { chip: ['', '설정 없음'], rows: [], stale: true };
  if (!reply.ok || !reply.status) return { chip: ['bad', '연결 안 됨'], rows: [], stale: true };
  const s = reply.status;
  const isStale = stale(reply, nowMs);
  const [level, text] = RUN_STATE[s.run_state] || ['', String(s.run_state || '알 수 없음')];
  const sr = s.single_row || {};
  const link = s.operator_link || {};
  const speed = s.speed || {};
  const gnss = s.gnss || {};
  const category = s.stop_reason || sr.stop_category || null;
  const stopText = label(STOP_REASON, category);
  const blockers = Array.isArray(s.go_blockers) ? s.go_blockers : [];
  const rows = [
    ['모드', s.mode === 'SHADOW' ? '연습 모드(차는 움직이지 않음) [SHADOW]' : s.mode === 'MOTOR' ? '실제 주행 모드 [MOTOR]' : (s.mode || '--')],
    ['정지 이유', stopText ? [...new Set([stopText, s.stop_detail ? STOP_DETAIL[s.stop_detail] || s.stop_detail : stopText])].join(' · ') : '--'],
    ['실행기 코드', [s.run_state, category, s.stop_detail].filter((code) => typeof code === 'string' && code).join(' · ') || '--'],
    ['출발 조건', blockers.length ? blockers.map((code) => `${GO_BLOCKER[code] || '알 수 없는 조건입니다.'} [${code}]`).join(' ') : '--'],
    ['횡 · 방향 오차', `${fixed(sr.lateral_error_m, 2, ' m')} · ${fixed(sr.heading_error_deg, 1, '°')}`],
    ['속도', `${fixed(sr.speed_m_s, 2, '')} / 목표 ${fixed(num(speed.target_mps) ?? num(sr.target_speed_m_s), 2, '')} m/s`],
    ['속도 제한', num(speed.limit_mps) === null ? '--'
      : `${fixed(speed.limit_mps, 2, ' m/s')} · ${label(LIMIT_REASON, speed.limit_reason) || '--'}`],
    ['진행 거리', fixed(sr.distance_m, 1, ' m')],
    ['GPS', gnss.state || gnss.mode ? [gnss.mode ? GNSS_MODE_TEXT[gnss.mode] || gnss.mode : null, gnss.state ? label(GNSS_STATE, gnss.state) : null,
      num(num(gnss.hacc_m) ?? gnss.h_acc_m) !== null ? `±${(num(gnss.hacc_m) ?? gnss.h_acc_m).toFixed(1)} m` : null].filter(Boolean).join(' · ') : '정보 없음'],
    ['화면 연결', link.fresh === true ? `연결됨 · ${fixed(link.age_s, 2, ' s')}`
      : link.fresh === false ? '끊김' : '--'],
  ];
  return { chip: isStale ? ['bad', '끊김'] : [level, text], rows, stale: isStale };
}

// The executor's motor mode as a chip next to the run state (2026-11-17): SHADOW runs never move the vehicle, so the
// operator sees it before pressing start, not only in the collapsed details. null while the status is missing or stale.
export function modeChip(reply, nowMs = Date.now()) {
  const s = reply && reply.ok && reply.status ? reply.status : null;
  if (!s || !s.mode || stale(reply, nowMs)) return null;
  if (s.mode === 'SHADOW') return ['', '연습 모드'];
  if (s.mode === 'MOTOR') return ['warn', '실제 주행 모드'];
  return ['', String(s.mode)];
}

// PW-10: "LiDAR 속도 ÷ 표 속도" of the last ~20 s of driving, or why there is none / why it does not change. The line
// of the autonomy section (shown only when the executor sends the block: a front-steer executor with its monitor).
export function scaleLine(reply, nowMs = Date.now()) {
  const s = reply && reply.ok && reply.status ? reply.status : null;
  if (!s || !s.online_scale || typeof s.online_scale !== 'object' || stale(reply, nowMs)) return null;
  return scaleText(s.online_scale);
}

export function scaleText(scale) {
  if (!scale || scale.enabled === false) return '꺼짐';
  const win = num(scale.window_s);
  const minimum = num(scale.min_s);
  const target = num(scale.target_s);
  let text;
  if (num(scale.value) === null) {
    text = `모으는 중 ${win === null ? '--' : win.toFixed(0)}/${minimum === null ? '10' : minimum.toFixed(0)} s`;
  } else {
    text = `${scale.value.toFixed(3)} · 최근 ${win === null ? '--' : win.toFixed(0)}${target === null ? '' : `/${target.toFixed(0)}`} s`;
  }
  if (scale.frozen === true || (num(scale.value) === null && scale.hold && scale.hold !== 'not_driving')) {
    text += ` · 갱신 멈춤 · ${label(SCALE_HOLD, scale.hold) || '이유 모름'}`;
  }
  if (scale.mismatch === true) text += ' · 속도 표와 어긋남';
  return text;
}

// PW-10: the question after a run ended (null when there is nothing to ask). accept: the executor sets k_field.
export function scaleOffer(reply) {
  const s = reply && reply.ok && reply.status ? reply.status : null;
  const scale = s && s.online_scale;
  const offer = scale && typeof scale.offer === 'object' ? scale.offer : null;
  if (!offer || !Number.isInteger(offer.run_id) || HEARTBEAT_STATES.has(s.run_state)) return null;
  const value = num(offer.value);
  const win = num(offer.window_s);
  const span = win === null ? '' : `마지막 ${win.toFixed(0)}초 동안 잰 `;
  const measured = value === null ? `${offer.run_id}번 주행에서는 온라인 배율을 재지 못했습니다.`
    : `${offer.run_id}번 주행에서 ${span}온라인 배율은 ${value.toFixed(3)}입니다.`;
  if (offer.ok !== true) {
    return { runId: offer.run_id, canAccept: false,
             text: `${measured} 전체 배율로 저장할 수 없습니다. ${offer.why_not ? reasonSentence(SCALE_WHY_NOT[offer.why_not] || offer.why_not) : '이유는 알 수 없습니다.'}` };
  }
  const now = num(offer.speed_scale_now);
  const next = num(offer.speed_scale_new);
  return { runId: offer.run_id, canAccept: true, from: now, to: next,
           text: `${measured} 지금 전체 배율은 ${fixed(now, 3, '')}입니다. ${fixed(next, 3, '')} 값으로 바꿔 저장할까요?` };
}

// ---- Speed of the run (2026-11-20, user: "자율주행 속도 설정은 따로 못 하나?") -----------------------------------------
// The speed choice of the tab is the executor's field tuning key target_speed_m_s. Everything it needs comes with the
// status the panel already reads (status.tuning; simulation repository tools/row_executor/field_tuning.py State.status):
//   - the range: basic[].low / high of the key, offered in SPEED_STEP_M_S steps;
//   - the executor's own rule between values (field_tuning.cross_problems: min_speed_m_s < target <= max_speed_m_s): a
//     step it would refuse is not offered (with the 2026-11 values 0.20 is in the range and refused, 0.25 .. 0.50 are
//     taken); without those two values every step of the range is offered and the executor judges;
//   - the value now (also when another screen or the field tuning part changed it; a value between the steps is shown
//     as a choice of its own);
//   - whether a change is taken now: tuning.editable (executor_main._tuning_lock: not while a run is open, i.e. ready,
//     armed, running or paused, not while the bridge is still armed, not while the vehicle moves);
//   - the overspeed line max_speed_m_s + overspeed_margin_m_s for the one line of caution.
// Without the range the choice is off and shows the value now only. Returns { value, options, enabled, limit, note }.
export const SPEED_KEY = 'target_speed_m_s';
export const SPEED_STEP_M_S = 0.05;
export const SPEED_NEAR_LIMIT_M_S = 0.10;       // a value this close to the overspeed line gets the line of caution
const hundredths = (value) => Math.round(value * 100) / 100;
export const speedLabel = (value) => `${value.toFixed(2)} m/s`;
export function speedNote(value, limit) {
  if (num(value) === null || num(limit) === null || limit - value > SPEED_NEAR_LIMIT_M_S + 1e-9) return '';
  return `과속 한도(${limit.toFixed(2)} m/s)에 가까워 주행 중 잠깐 느려질 때가 있습니다.`;
}
export function speedChoice(reply, nowMs = Date.now()) {
  const s = usable(reply, nowMs);
  if (!s) return { value: null, options: [], enabled: false, limit: null, note: '' };
  const tuning = s.tuning && typeof s.tuning === 'object' ? s.tuning : {};
  const entry = Array.isArray(tuning.basic) ? tuning.basic.find((item) => item && item.name === SPEED_KEY) : null;
  const values = tuning.values && typeof tuning.values === 'object' ? tuning.values : {};
  const value = num(values.basic?.[SPEED_KEY]) ?? num(entry?.value) ?? num(s.speed?.target_mps);
  const top = num(values.locked?.max_speed_m_s), margin = num(values.locked?.overspeed_margin_m_s);
  const limit = top !== null && margin !== null ? hundredths(top + margin) : null;
  const note = speedNote(value, limit);
  const low = num(entry?.low), high = num(entry?.high);
  if (low === null || high === null || !(high >= low)) return { value, options: value === null ? [] : [value], enabled: false, limit, note };
  const floor = num(values.advanced?.min_speed_m_s);
  const options = [];
  for (let k = Math.ceil(low / SPEED_STEP_M_S - 1e-9); k * SPEED_STEP_M_S <= high + 1e-9; k += 1) {
    const step = hundredths(k * SPEED_STEP_M_S);
    if ((floor === null || step > floor + 1e-9) && (top === null || step <= top + 1e-9)) options.push(step);
  }
  if (value !== null && !options.some((step) => Math.abs(step - value) < 1e-9)) { options.push(value); options.sort((a, b) => a - b); }
  return { value, options, enabled: options.length > 0 && tuning.editable === true && !HEARTBEAT_STATES.has(s.run_state), limit, note };
}

// A random session id of the operator link (contract: 4 to 64 characters).
export function newSession(random = globalThis.crypto) {
  const bytes = new Uint8Array(12);
  random.getRandomValues(bytes);
  return Array.from(bytes, (b) => b.toString(16).padStart(2, '0')).join('');
}

/// ---- Monitoring view of the 자율주행 tab (2026-11-19) ---------------------------------------------------------------
// User: "자율주행시에 occupancy grid map을 그리면서 하고있는거니? ... 모니터링이 쉽게 되도록". One tile per value. Since
// 2026-11-20 ("글이 너무 많고") six tiles stay on the screen (state and reason, speed, lateral error, remaining distance,
// GPS, recording); the others (heading error, the speed limit, the steering command against the measured angle, the drive
// duty, the steering motor PWM of bridge status steering.pwm / pwm_peak) are in the folded 자세히. The executor status is
// the autonomy panel's own poll; the bridge status is the steering panel's (shared, no request of its own).
const DEG = 180 / Math.PI / 1000;
const mradDeg = (mrad) => (num(mrad) === null ? null : mrad * DEG);
const signedDeg = (deg) => (deg === null ? '--' : `${Math.abs(deg) < 0.05 ? '' : deg > 0 ? '왼 ' : '오 '}${Math.abs(deg).toFixed(1)}°`);

// The GPS words of a run: [value, sub, level]. requested: the mode the page sent ('' = the executor's setting).
export function gnssTile(status, requested = '') {
  const g = status && typeof status.gnss === 'object' && status.gnss ? status.gnss : null;
  if (!g) return [requested ? GNSS_MODE_TEXT[requested] || requested : '--', '실행기가 GPS 상태를 보내지 않습니다.', ''];
  const mode = typeof g.mode === 'string' ? g.mode : null;
  const value = mode ? GNSS_MODE_TEXT[mode] || mode : '설정 모름';
  const hacc = num(g.hacc_m) ?? num(g.h_acc_m);
  const counts = g.counts && typeof g.counts === 'object' ? g.counts : {};
  const accepted = num(g.accepted) ?? num(counts.accepted);
  const rejectedOf = Object.entries(counts).filter(([k]) => !['accepted', 'restarted', 'reanchored'].includes(k));
  const rejected = num(g.rejected) ?? (rejectedOf.length ? rejectedOf.reduce((a, [, v]) => a + (num(v) || 0), 0) : null);
  const state = g.rtk?.state || g.state || g.outcome || null;
  const parts = [state ? label(GNSS_STATE, state) : null, hacc === null ? null : `±${hacc.toFixed(2)} m`,
                 accepted === null ? null : `받음 ${accepted}${rejected === null ? '' : ` · 거부 ${rejected}`}`].filter(Boolean);
  let level = '';
  if (g.fence_tripped === true) { parts.unshift('범위 벗어남'); level = 'bad'; }
  if (requested && mode && requested !== mode) { parts.unshift('요청한 설정과 다름'); level = level || 'warn'; }
  return [value, parts.join(' · ') || '정보 없음', level];
}

// The tiles [label, value, sub, level]: tiles = the six on the screen, more = the others (folded 자세히). bridge: the
// bridge status reply of the steering panel (or null); recording: {known, active, tag, auto, stopFailed} (auto =
// started by this panel for the stretch; stopFailed = its stop after the stretch was refused). segment: the server's
// record of the newest recorded stretch (runTimes).
export function monitorView(reply, bridge = null, recording = null, requestedGnss = '', nowMs = Date.now(), segment = null) {
  const s = reply && reply.ok && reply.status ? reply.status : null;
  const isStale = stale(reply, nowMs);
  const tiles = [], more = [];
  if (!s) {
    tiles.push(['상태', reply && reply.configured === false ? '실행기 없음' : reply ? '연결 안 됨' : '확인 중', '', reply ? 'bad' : '']);
  } else {
    const [level, text] = RUN_STATE[s.run_state] || ['', String(s.run_state || '알 수 없음')];
    const sr = s.single_row || {};
    const speed = s.speed || {};
    tiles.push(['상태', isStale ? '끊김' : text, stopWords(s) || '', isStale ? 'bad' : level]);
    tiles.push(['속도', fixed(num(sr.speed_m_s) ?? num(s.v), 2, ' m/s'), `목표 ${fixed(num(speed.target_mps) ?? num(sr.target_speed_m_s), 2, '')}`, '']);
    tiles.push(['횡 오차', fixed(num(sr.lateral_error_m) ?? num(s.lat), 2, ' m'), '+는 왼쪽', '']);
    const distance = num(sr.distance_m) ?? num(s.dist);
    tiles.push(['남은 거리', fixed(num(sr.end_remaining_m), 1, ' m'), distance === null ? '' : `간 거리 ${distance.toFixed(1)} m`, '']);
    // the run's driving time (standing still does not count); under it the stretch since a start again
    const times = runTimes(reply, segment);
    tiles.push(['주행 시간', times.time_s === null ? '--' : durationText(times.time_s), times.part ? `다시 출발 뒤 ${durationText(times.part.time_s)}` : '', '']);
    more.push(['방향 오차', fixed(num(sr.heading_error_deg) ?? num(s.head), 1, '°'), '', '']);
    // the limit and what binds it, while the executor sends one (not driving: limit null, reason 'not_running')
    more.push(['속도 제한', fixed(num(speed.limit_mps), 2, ' m/s'), num(speed.limit_mps) === null ? '' : label(LIMIT_REASON, speed.limit_reason) || '', '']);
    if (sr.phase) more.push(['단계', String(sr.phase), '', '']);
  }
  tiles.push(['GPS', ...gnssTile(s, requestedGnss)]);
  if (recording) {
    // one recording per driven stretch (2026-11-20): only a run that drives without one is a warning
    const driving = Boolean(s) && s.run_state === 'running';
    const rec = !recording.known ? ['확인 중', '', ''] : recording.active
      ? ['기록 중', recording.stopFailed ? '자동으로 멈추지 못했습니다. 기록 탭에서 중지하세요.'
        : recording.auto ? '모든 센서 자동 기록' : recording.tag === 'autonomy' ? '자율주행 기록' : '직접 시작한 기록',
        recording.stopFailed ? 'warn' : 'ok']
      : driving ? ['기록 꺼짐', '', 'warn'] : ['대기', '출발하면 자동 시작', ''];
    tiles.push(['기록', ...rec]);
  }
  // Steering command against the measured angle: the executor's own numbers, else the bridge's.
  const steering = bridge && typeof bridge.steering === 'object' && bridge.steering ? bridge.steering : {};
  const host = bridge && typeof bridge.steering_host === 'object' && bridge.steering_host ? bridge.steering_host : {};
  const cmd = mradDeg(num(s?.S) ?? num(s?.single_row?.steer_mrad) ?? num(host.target_mrad));
  const meas = mradDeg(num(s?.steer?.meas) ?? num(steering.measured_mrad));
  more.push(['조향 명령 / 측정', `${signedDeg(cmd)} / ${signedDeg(meas)}`, cmd !== null && meas !== null ? `차이 ${Math.abs(cmd - meas).toFixed(1)}°` : '', '']);
  const drive = bridge && typeof bridge.drive === 'object' && bridge.drive ? bridge.drive : {};
  const acked = [drive.left?.acked, drive.right?.acked].map(num);
  const throttle = num(s?.T) ?? num(s?.single_row?.throttle_pct);
  const board = acked.some((v) => v !== null) ? `보드 PWM ${acked.map((v) => (v === null ? '--' : v)).join(' / ')}` : '';
  more.push(['구동 출력', throttle === null ? '--' : `${throttle} %`, `${board}${num(drive.max_pwm) === null ? '' : `${board ? ' · ' : ''}상한 ${drive.max_pwm}`}`, '']);
  const pwm = num(steering.pwm), peak = num(steering.pwm_peak);
  more.push(['조향 모터 PWM', pwm === null ? '--' : String(pwm), peak === null ? (bridge ? '값 없음' : '조향 상태 없음') : `최근 최대 ${peak}`, '']);
  return { tiles, more, stale: isStale };
}

// status.metrics: the executor's comparison of its reference centreline with the travelled path (names read
// defensively; grid_* = the same against the centreline of the local grid). Returns {tiles, source} or null.
const METRICS = [
  ['lat_rms_m', '횡 RMS', 'm', 3], ['lat_mean_m', '횡 평균', 'm', 3], ['lat_max_abs_m', '횡 최대', 'm', 3],
  ['within_0_05_pct', '±5 cm 안', '%', 1], ['within_0_10_pct', '±10 cm 안', '%', 1],
  ['head_rms_deg', '방향 RMS', '°', 2], ['head_max_abs_deg', '방향 최대', '°', 2],
];
export function metricsView(reply) {
  const m = reply && reply.ok && reply.status && reply.status.metrics && typeof reply.status.metrics === 'object' ? reply.status.metrics : null;
  if (!m) return null;
  const unit = (value, u, digits) => (num(value) === null ? '--' : `${value.toFixed(digits)}${u === '°' ? '°' : ` ${u}`}`);
  const tiles = [];
  for (const [key, name, u, digits] of METRICS) {
    const grid = m[`grid_${key}`];
    if (num(m[key]) === null && num(grid) === null) continue;
    tiles.push([name, unit(m[key], u, digits), num(grid) === null ? '' : `격자 중심선 ${unit(grid, u, digits)}`, '']);
  }
  const samples = num(m.samples), distance = num(m.distance_m);
  if (samples !== null || distance !== null) {
    tiles.push(['표본', samples === null ? '--' : String(samples), distance === null ? '' : `${distance.toFixed(1)} m 동안`, '']);
  }
  const known = new Set(['samples', 'distance_m', ...METRICS.flatMap(([k]) => [k, `grid_${k}`])]);
  for (const [key, value] of Object.entries(m).filter(([k, v]) => !known.has(k) && num(v) !== null).slice(0, 6)) {
    tiles.push([key, String(Math.round(value * 1000) / 1000), '', '']);
  }
  return tiles.length ? { tiles, source: typeof m.reference === 'string' ? m.reference : '기준 중심선과 실제 경로 비교' } : null;
}

// Recording of a run (2026-11-19, user: "자율주행시에는 반드시 모든 센서가 녹화되도록"; 2026-11-20: one recording per driven
// stretch, user request: a run that stops is recorded up to that place, and a start again begins a new recording). Before a
// go (the first one or 다시 출발): a new recording of every sensor (tag autonomy). When the run stops driving, whatever
// stops it (a pause too): the recording this panel started stops at once. A recording somebody started by hand with
// every sensor is kept and never stopped here. The dashboard server stops a run recording that no page stops
// (dashboard_server/run_recording.py).
export const RUN_RECORD_SENSORS = ['lidar', 'camera', 'imu', 'gps', 'drive', 'steering', 'executor', 'tf'];
export const RUN_OVER_STATES = new Set(['ended', 'idle', 'latched']);
// A new recording runs this long before the heartbeat and the go are sent. Measured 2026-11-20 (SIM on the bench PC):
// 2.1 s after a recording started the steering bridge process stood still for 0.35 s (the recorder's start load), and a
// run that left at that moment ended with steering_feedback_stale (4 of 9 runs). Placeholder until the Jetson is timed.
export const GO_AFTER_RECORD_MS = 2500;
export const GO_WAIT_STEP_MS = 100;       // 멈추기 is looked at this often while waiting
// After a go the run may show 'running' this much later (the executor holds the press through the input warm-up); a
// recording is not stopped for "not driving" before that.
export const GO_SETTLE_MS = 4000;
// needed (2026-11-20): the sensor groups an autonomy run must record; a running recording without all of them is
// restarted with every sensor ('restart'), one by hand that has them is kept. A running recording with the tag
// autonomy is the one of an earlier stretch that nobody stopped yet: it is ended and a new one started ('restart').
export function recordBeforeGo(recording, wanted, needed = null) {
  if (!wanted) return 'skip';
  if (!recording || !recording.known) return 'unknown';
  if (!recording.active) return 'start';
  if (recording.tag === 'autonomy') return 'restart';
  if (!Array.isArray(needed) || !needed.length) return 'keep';
  const have = Array.isArray(recording.sensors) ? recording.sensors : [];
  return needed.every((s) => have.includes(s)) ? 'keep' : 'restart';
}
// True when the recording this panel started for a stretch (autoId) stops now: the run does not drive any more.
// going: the go of this recording is still on its way (recording, wait, heartbeat, go); sawRunning: the run was seen
// driving since; sinceGoMs: time since the go was accepted (null: no go yet). A status that is not known: wait (the
// server's keeper decides).
export function recordStopWanted(reply, autoId, recording, { going = false, sawRunning = false, sinceGoMs = null } = {}) {
  if (!autoId || !recording || !recording.known || !recording.active || recording.id !== autoId) return false;
  const s = reply && reply.ok && reply.status ? reply.status : null;
  if (!s || going || s.run_state === 'running') return false;
  if (RUN_OVER_STATES.has(s.run_state)) return true;
  if ((s.operator_link || {}).go_active === true) return false;      // the press is still held: the run may yet leave
  return sawRunning || sinceGoMs === null || sinceGoMs > GO_SETTLE_MS;
}
