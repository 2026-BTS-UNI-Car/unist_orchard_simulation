// The tuning screen's model (UI-12, docs/STEERING_SERVER_KO.md 7): which values the screen offers, the checks before a
// change is sent and the change log rows. Pure functions: tested without a page. The executor stays the final judge.

export const GROUP_TEXT = {
  basic: ['기본 묶음', '정지 상태에서 바꾼다'],
  advanced: ['고급 묶음', '실행기가 받는다고 알릴 때만 바꾼다'],
  safety: ['안전 한계', '화면에서 바꾸지 않는다(잠금)'],
};
export const ERROR_TEXT = {
  not_stopped: '정지 상태(대기·준비·끝)에서만 적용한다', out_of_range: '범위 밖', unknown_parameter: '모르는 값',
  locked_parameter: '잠긴 값', no_change: '바뀐 값이 없다', executor_unreachable: '실행기 연결 안 됨',
  executor_not_configured: '실행기 설정 없음', executor_token_rejected: '실행기 토큰이 맞지 않음',
};

const num = (value) => (typeof value === 'number' && isFinite(value) ? value : null);
const decimals = (step) => (num(step) === null || step >= 1 ? 0 : Math.min(3, String(step).split('.')[1]?.length || 0));
export const fmt = (value, step) => (num(value) === null ? '--' : value.toFixed(decimals(step)));

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
  if (!view.applicable) return ['warn', `적용 불가 · ${view.run_state || '상태 모름'}`];
  return ['ok', '적용 가능'];
}

// Check the typed values (strings from the inputs, by name) against the view. Returns
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
      problems.push({ name: param.name, label: param.label, text: `범위 밖(${fmt(param.min, param.step)} ~ ${fmt(param.max, param.step)} ${param.unit})` });
      continue;
    }
    if (num(param.value) !== null && Math.abs(value - param.value) < 1e-9) continue;
    changes[param.name] = value;
  }
  return { changes, problems };
}

// One line per change for the confirmation: "목표 속도 0.40 → 0.35 m/s".
export function describeChanges(view, changes) {
  const by = Object.fromEntries((view?.params || []).map((p) => [p.name, p]));
  return Object.entries(changes).map(([name, value]) => {
    const p = by[name] || { label: name, unit: '', step: 0.01 };
    return `${p.label} ${fmt(p.value, p.step)} → ${fmt(value, p.step)} ${p.unit}`.trim();
  });
}

const two = (n) => String(n).padStart(2, '0');
// Short local time of a log row: "11-13 05:20:25" (a phone column stays narrow).
export const clock = (d) => `${two(d.getMonth() + 1)}-${two(d.getDate())} ${two(d.getHours())}:${two(d.getMinutes())}:${two(d.getSeconds())}`;

// Change log rows (newest first): [time, changes, result, revision, who].
export function logRows(entries, view) {
  const by = Object.fromEntries((view?.params || []).map((p) => [p.name, p]));
  return (Array.isArray(entries) ? entries : []).map((entry) => {
    const at = num(entry.at) === null ? '--' : clock(new Date(entry.at * 1000));
    const changes = Object.entries(entry.changes || {}).map(([name, pair]) => {
      const p = by[name] || { label: name, step: 0.01 };
      const [before, after] = Array.isArray(pair) ? pair : [null, null];
      return `${p.label} ${fmt(before, p.step)} → ${fmt(after, p.step)}`;
    }).join(', ') || '--';
    const result = entry.ok ? '적용' : `거절 · ${ERROR_TEXT[entry.error] || entry.error || '--'}`;
    const revision = `${entry.revision_before ?? '--'} → ${entry.revision_after ?? '--'}`;
    return [at, changes, result, revision, String(entry.by || '--')];
  });
}
