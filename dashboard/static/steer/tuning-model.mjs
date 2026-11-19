// The tuning screen's model (UI-12, docs/STEERING_SERVER_KO.md 7): what the screen offers, the checks before a change
// is sent and the change log rows. The bundle (names, ranges, units, groups, presets) comes from the executor's field
// tuning through GET /api/executor/tuning; the executor stays the final judge. Pure functions: tested without a page.

export const GROUP_TEXT = {
  basic: ['기본 묶음', '차가 멈춰 있을 때 화면에서 바꿀 수 있습니다'],
  advanced: ['고급 묶음', '파일에서만 바꿀 수 있습니다'],
  locked: ['잠금 값', '파일에서 바꾸고 실행기를 다시 시작해야 적용됩니다'],
};
// Reasons the dashboard or the executor gives (field_tuning, OPERATOR_API_KO.md 6) in screen words.
export const ERROR_TEXT = {
  tuning_locked_while_running: '주행이 끝나지 않아 바꿀 수 없습니다.', tuning_locked_while_armed: '실행기가 조종 권한을 쥐고 있어 바꿀 수 없습니다.',
  tuning_locked_vehicle_moving: '차가 움직이고 있어 바꿀 수 없습니다.', tuning_locked: '실행기가 지금은 바꿀 수 없다고 답했습니다.',
  no_field_tuning: '실행기에 현장 조정 값이 없습니다.', tuning_not_writable: '실행기가 기본 파일을 쓰고 있어 저장할 수 없습니다.',
  invalid_params: '실행기가 값을 받지 않았습니다.', tuning_save_failed: '파일에 저장하지 못했습니다.', out_of_range: '범위를 벗어난 값입니다.',
  unknown_parameter: '실행기가 모르는 값입니다.', locked_parameter: '잠긴 값입니다.', unknown_preset: '없는 프리셋입니다.', bad_preset_name: '프리셋 이름이 규칙에 맞지 않습니다.',
  bad_note: '메모 길이가 한도를 넘었습니다.', no_change: '바뀐 값이 없습니다.', executor_unreachable: '실행기에 연결하지 못했습니다.',
  executor_not_configured: '실행기가 설정되어 있지 않습니다.', executor_token_rejected: '실행기 토큰이 맞지 않습니다.',
  // PW-10: the answer to the online-scale offer after a run (contract revision 6)
  no_online_scale_offer: '저장할 온라인 배율이 없습니다.', online_scale_offer_changed: '그사이 다른 주행의 배율로 바뀌었습니다.',
  online_scale_basis_changed: '그사이 배율 값이 바뀌었습니다.', online_scale_form: '온라인 배율 요청 형식이 맞지 않습니다.',
  no_scale: '주행이 10초보다 짧아 배율이 아직 없습니다.', no_forward_motion: 'LiDAR에서 앞으로 간 움직임이 확인되지 않았습니다.',
  scale_set_in_config: '실행기 설정 파일에 배율이 따로 정해져 있습니다.', online_scale_not_offered: '저장할 수 없는 배율입니다.',
};
const ACTION_TEXT = { set: '값 바꿈', load_preset: '프리셋 불러옴', save: '파일에 저장', save_preset: '프리셋으로 저장',
                      online_scale: '온라인 배율을 전체 배율로', online_scale_dismiss: '온라인 배율 저장 안 함' };
// What an accepted request did, as the sentence of the notice line.
const DONE_TEXT = { set: '값을 적용했습니다.', load_preset: '프리셋 값을 적용했습니다.', save: '파일에 저장했습니다.', save_preset: '프리셋으로 저장했습니다.',
                    online_scale: '온라인 배율을 전체 배율로 저장했습니다.', online_scale_dismiss: '온라인 배율을 저장하지 않았습니다.' };
// The executor's run state in the words of the 자율주행 tab's chip (auto-model.mjs RUN_STATE).
const RUN_STATE_TEXT = { idle: '대기', ready: '준비 미완료', armed: '출발 대기', running: '주행 중', paused: '일시정지', ended: '끝남', latched: '멈춤' };

const num = (value) => (typeof value === 'number' && isFinite(value) ? value : null);
const decimals = (step) => (num(step) === null || step >= 1 ? 2 : Math.min(3, String(step).split('.')[1]?.length || 0));
// A value of the bundle: a number, a [[duty, m/s], ...] table, or a text.
export const fmt = (value, step) => {
  if (Array.isArray(value)) return value.map((row) => (Array.isArray(row) ? `${row[0]}→${row[1]}` : '?')).join(', ') || '--';
  if (typeof value === 'string') return value;
  if (num(value) === null) return '--';
  return Number.isInteger(value) && (num(step) === null || step >= 1) ? String(value) : value.toFixed(decimals(step));
};
export const withUnit = (text, unit) => (unit && text !== '--' ? `${text} ${unit}` : text);

export function groups(view) {
  const params = Array.isArray(view?.params) ? view.params : [];
  return Object.keys(GROUP_TEXT).map((group) => ({ group, title: GROUP_TEXT[group][0], note: GROUP_TEXT[group][1],
                                                   params: params.filter((p) => p.group === group) }));
}

// The screen state line: [level, text].
export function screenState(view) {
  if (!view) return ['', '확인 중'];
  if (view.configured === false) return ['', '실행기 설정 없음'];
  if (!view.reachable) return ['bad', '실행기 연결 안 됨'];
  if (view.available === false) return ['warn', '조정 값 없음'];
  if (!view.applicable) return ['warn', '지금은 변경 불가'];
  return ['ok', '변경 가능'];
}

// Why nothing can change now (empty when it can).
export function whyNot(view) {
  if (!view) return '실행기의 조정 값을 읽고 있습니다.';
  if (view.configured === false) return '이 대시보드에는 자율주행 실행기 주소 설정 UBTS_EXECUTOR_URL이 없어 값을 읽거나 바꿀 수 없습니다.';
  if (!view.reachable) return `실행기에서 조정 값을 읽을 수 없습니다. ${ERROR_TEXT[view.error] || (view.error ? `알려 온 이유는 ${view.error}입니다.` : '실행기가 응답하지 않습니다.')}`;
  if (view.applicable) return '';
  const reason = ERROR_TEXT[view.why_not] || (view.why_not ? `알려 온 이유는 ${view.why_not}입니다.` : '지금은 바꿀 수 없습니다.');
  const state = RUN_STATE_TEXT[view.run_state];
  return `${reason} 주행을 끝내고 차가 멈춘 뒤에 바꾸세요.${state ? ` 지금 실행기 상태는 ${state}입니다.` : ''}`;
}

// The line under the title: preset (a * when changed from it), revision, unsaved.
export function presetLine(view) {
  const t = view?.tuning || {};
  if (!view?.available) return '판 --';
  return `프리셋 ${t.preset ?? '--'} · 판 ${t.revision ?? '--'}${t.unsaved ? ' · 파일에 저장 전' : ''}`;
}

// Check the typed values (strings from the inputs, by name) against the executor's ranges. Returns
// { changes: {name: number}, problems: [{name, label, text}] }. An empty input means "keep the current value".
export function check(view, typed) {
  const changes = {}, problems = [];
  for (const param of view?.params || []) {
    const raw = typed[param.name];
    if (raw === undefined || String(raw).trim() === '') continue;
    const value = Number(String(raw).trim());
    if (!param.editable) {
      if (num(param.value) === null || value !== param.value) problems.push({ name: param.name, label: param.label, text: '값은 바꿀 수 없습니다.' });
      continue;
    }
    if (!Number.isFinite(value)) { problems.push({ name: param.name, label: param.label, text: '값이 숫자가 아닙니다.' }); continue; }
    if (value < param.min || value > param.max) {
      problems.push({ name: param.name, label: param.label, text: `값이 범위 ${withUnit(`${fmt(param.min, param.step)} ~ ${fmt(param.max, param.step)}`, param.unit)} 밖입니다.` });
      continue;
    }
    if (num(param.value) !== null && Math.abs(value - param.value) < 1e-9) continue;
    changes[param.name] = value;
  }
  return { changes, problems };
}

// A preset name the executor takes (field_tuning._name_ok): 1-40 of A-Z a-z 0-9 _ - . and not "default".
export const presetNameOk = (name) => /^[A-Za-z0-9_.-]{1,40}$/.test(name) && name !== 'default';

// One line per change for the confirmation: "목표 속도 0.40 → 0.35 m/s".
export function describeChanges(view, changes) {
  const by = Object.fromEntries((view?.params || []).map((p) => [p.name, p]));
  return Object.entries(changes).map(([name, value]) => {
    const p = by[name] || { label: name, unit: '', step: 0.01 };
    return withUnit(`${p.label} ${fmt(p.value, p.step)} → ${fmt(value, p.step)}`, p.unit);
  });
}

// The page's words for a POST /api/executor/params reply: [level, text].
export function replyText(view, code, reply) {
  if (reply?.ok) {
    const changed = (reply.changes || []).map((c) => {
      const p = (view?.params || []).find((x) => x.name === c.key) || { label: c.key, step: 0.01 };
      return `${p.label} ${fmt(c.old, p.step)} → ${fmt(c.new, p.step)}`;
    });
    const what = DONE_TEXT[reply.action] || '적용했습니다.';
    return ['ok', `${what} 판 ${reply.tuning?.revision ?? '--'}${changed.length ? ` · ${changed.join(', ')}` : ''}`];
  }
  const reason = ERROR_TEXT[reply?.error] || (reply?.error ? `알려 온 이유는 ${reply.error}입니다.` : `서버가 HTTP ${code} 오류로 답했습니다.`);
  const problems = Array.isArray(reply?.problems) && reply.problems.length ? ` ${reply.problems.join(' / ')}` : '';
  return ['bad', `요청이 거절됐습니다. ${reason}${problems}`];
}

const two = (n) => String(n).padStart(2, '0');
// Short local time of a log row: "11-13 05:20:25" (a phone column stays narrow).
export const clock = (d) => `${two(d.getMonth() + 1)}-${two(d.getDate())} ${two(d.getHours())}:${two(d.getMinutes())}:${two(d.getSeconds())}`;

// Change log rows (newest first): [time, what changed, result, revision, who].
export function logRows(entries, view) {
  const by = Object.fromEntries((view?.params || []).map((p) => [p.name, p]));
  return (Array.isArray(entries) ? entries : []).map((entry) => {
    const at = num(entry.at) === null ? '--' : clock(new Date(entry.at * 1000));
    const changes = Object.entries(entry.changes || {}).map(([name, pair]) => {
      const p = by[name] || { label: name, step: 0.01 };
      const [before, after] = Array.isArray(pair) ? pair : [null, null];
      return `${p.label} ${fmt(before, p.step)} → ${fmt(after, p.step)}`;
    });
    const action = entry.action && entry.action !== 'set' ? `${ACTION_TEXT[entry.action] || entry.action}${entry.target ? ` ${entry.target}` : ''}` : '';
    const what = [action, ...changes].filter(Boolean).join(', ') || '--';
    const result = entry.ok ? '적용' : `거절 · ${ERROR_TEXT[entry.error] || entry.error || '--'}`;
    const revision = `${entry.revision_before ?? '--'} → ${entry.revision_after ?? '--'}`;
    return [at, what, result, revision, String(entry.by || '--')];
  });
}
