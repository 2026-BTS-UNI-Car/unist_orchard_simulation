// The tuning screen's model (UI-12, docs/STEERING_SERVER_KO.md 7): what the screen offers, the checks before a change
// is sent and the change log rows. The bundle (names, ranges, units, groups, presets) comes from the executor's field
// tuning through GET /api/executor/tuning; the executor stays the final judge. Pure functions: tested without a page.

export const GROUP_TEXT = {
  basic: ['기본 묶음', '차가 정지했을 때 화면에서 바꾼다'],
  advanced: ['고급 묶음', '파일에서만 바꾼다'],
  locked: ['잠금 값', '파일에서만, 실행기를 다시 시작해야 바뀐다'],
};
// Reasons the dashboard or the executor gives (field_tuning, OPERATOR_API_KO.md 6) in screen words.
export const ERROR_TEXT = {
  tuning_locked_while_running: '주행(run)이 열려 있어 바꿀 수 없다', tuning_locked_while_armed: '구동 권한(Arm)을 쥐고 있어 바꿀 수 없다',
  tuning_locked_vehicle_moving: '차가 움직이고 있어 바꿀 수 없다', tuning_locked: '실행기가 지금은 바꿀 수 없다고 한다',
  no_field_tuning: '실행기에 현장 조정 묶음이 없다', tuning_not_writable: '실행기가 기본 파일을 쓰고 있어 저장할 수 없다',
  invalid_params: '실행기가 값을 받지 않았다', tuning_save_failed: '파일 저장 실패', out_of_range: '범위 밖',
  unknown_parameter: '모르는 값', locked_parameter: '잠긴 값', unknown_preset: '모르는 프리셋', bad_preset_name: '프리셋 이름이 맞지 않음',
  bad_note: '메모가 너무 김', no_change: '바뀐 값이 없다', executor_unreachable: '실행기 연결 안 됨',
  executor_not_configured: '실행기 설정 없음', executor_token_rejected: '실행기 토큰이 맞지 않음',
};
const ACTION_TEXT = { set: '값 바꿈', load_preset: '프리셋 불러옴', save: '파일에 저장', save_preset: '프리셋으로 저장' };

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
  if (view.available === false) return ['warn', '조정 묶음 없음'];
  if (!view.applicable) return ['warn', '지금은 바꿀 수 없음'];
  return ['ok', '바꿀 수 있음'];
}

// Why nothing can change now (empty when it can).
export function whyNot(view) {
  if (!view) return '실행기 조정 값을 읽고 있다.';
  if (view.configured === false) return '자율 실행기 설정 없음: 이 대시보드에는 UBTS_EXECUTOR_URL이 없어 값을 읽거나 바꿀 수 없다.';
  if (!view.reachable) return `실행기 연결 안 됨: 값을 읽을 수 없다(${ERROR_TEXT[view.error] || view.error || '응답 없음'}).`;
  if (view.applicable) return '';
  const reason = ERROR_TEXT[view.why_not] || view.why_not || '이유 모름';
  return `${reason}. 정지 상태(run을 끝내고 구동 권한을 놓은 뒤)에서 바꾼다. 지금 상태: ${view.run_state || '모름'}.`;
}

// The line under the title: preset (a * when changed from it), revision, unsaved.
export function presetLine(view) {
  const t = view?.tuning || {};
  if (!view?.available) return '판 --';
  return `프리셋 ${t.preset ?? '--'} · 판 ${t.revision ?? '--'}${t.unsaved ? ' · 파일에 저장 안 함' : ''}`;
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
      if (num(param.value) === null || value !== param.value) problems.push({ name: param.name, label: param.label, text: '바꿀 수 없는 값' });
      continue;
    }
    if (!Number.isFinite(value)) { problems.push({ name: param.name, label: param.label, text: '수가 아님' }); continue; }
    if (value < param.min || value > param.max) {
      problems.push({ name: param.name, label: param.label, text: `범위 밖(${withUnit(`${fmt(param.min, param.step)} ~ ${fmt(param.max, param.step)}`, param.unit)})` });
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
    const what = ACTION_TEXT[reply.action] || '적용';
    return ['ok', `${what}: 판 ${reply.tuning?.revision ?? '--'}${changed.length ? ` · ${changed.join(', ')}` : ''}`];
  }
  const reason = ERROR_TEXT[reply?.error] || reply?.error || `HTTP ${code}`;
  const problems = Array.isArray(reply?.problems) && reply.problems.length ? ` · ${reply.problems.join(' / ')}` : '';
  return ['bad', `거절: ${reason}${problems}`];
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
