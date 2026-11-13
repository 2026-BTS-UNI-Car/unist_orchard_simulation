// Tuning screen (UI-12, route /tuning): the executor's basic values with their units, ranges and sources, the advanced
// group, the locked safety limits and the change log (docs/STEERING_SERVER_KO.md 7). A change is checked here, then by
// the dashboard, then by the executor; it is applied only while the vehicle is stopped. Loaded by static/app/14-steer.js
// when the route opens (any vehicle kind: without an executor the screen says so).
import { ERROR_TEXT, check, describeChanges, fmt, groups, logRows, screenState } from './tuning-model.mjs';

const VIEW_MS = 2000;

export function initTuningPage(page) {
  const $ = (id) => document.getElementById(id);
  if (!$('tuneBasic')) return null;
  let view = null, timer = null, signature = '', busy = false, message = null, logEntries = [], logAll = false;
  const LOG_FIRST = 10;                 // rows shown before "전체 보기" (the dashboard keeps the last 50)
  const active = () => page.currentRoute() === '/tuning' && !document.hidden;

  async function load() {
    try { view = await page.getJson('/api/executor/tuning'); } catch (_) { view = view ? { ...view, reachable: false } : null; }
    draw();
  }
  async function loadLog() {
    try { logEntries = (await page.getJson('/api/executor/tuning/log')).entries || []; } catch (_) { /* keep the last rows */ }
    renderLog();
  }

  function field(param) {
    const label = document.createElement('label');
    label.className = 'field';
    label.append(`${param.label} (${param.unit})`);
    const input = document.createElement('input');
    Object.assign(input, { className: 'input', type: 'number', name: param.name, min: String(param.min), max: String(param.max),
                           step: String(param.step), inputMode: 'decimal' });
    const note = document.createElement('small');
    note.dataset.note = param.name;
    label.append(input, note);
    return label;
  }
  function row(name, value) {
    const div = document.createElement('div');
    const dt = document.createElement('dt'); dt.textContent = name;
    const dd = document.createElement('dd'); dd.textContent = value;
    div.append(dt, dd);
    return div;
  }
  function build() {
    for (const { group, params } of groups(view)) {
      const box = $(`tune${group[0].toUpperCase()}${group.slice(1)}`);
      if (!box) continue;
      const inputs = params.filter((p) => p.editable_when_stopped);
      const fixed = params.filter((p) => !p.editable_when_stopped);
      const list = document.createElement('dl');
      list.className = 'detail-list';
      list.dataset.fixed = group;
      box.replaceChildren(...inputs.map(field), ...(fixed.length ? [list] : []));
    }
  }
  function draw() {
    const sig = JSON.stringify((view?.params || []).map((p) => [p.name, p.group, p.editable_when_stopped, p.min, p.max]));
    if (sig !== signature) { signature = sig; build(); }
    const [level, text] = screenState(view);
    const chip = $('tuneState');
    if (chip) { chip.className = `status-chip${level ? ` ${level}` : ''}`; chip.textContent = text; }
    const revision = view?.tuning?.revision;
    if ($('tuneRevision')) $('tuneRevision').textContent = `판 ${revision ?? '--'} · ${view?.tuning?.preset ?? '묶음 이름 없음'}`;
    for (const param of view?.params || []) {
      const input = document.querySelector(`#tuneForm input[name="${param.name}"]`);
      const note = document.querySelector(`#tuneForm small[data-note="${param.name}"]`);
      const current = `${fmt(param.value, param.step)}${param.value === null ? ' (값 모름)' : ''}`;
      if (input) {
        input.disabled = busy || !param.editable;
        input.placeholder = current;
      }
      if (note) note.textContent = `지금 ${current} · 범위 ${fmt(param.min, param.step)} ~ ${fmt(param.max, param.step)} · ${param.source}`;
    }
    for (const list of document.querySelectorAll('#tuneForm dl[data-fixed]')) {
      const rows = (view?.params || []).filter((p) => p.group === list.dataset.fixed && !p.editable_when_stopped)
        .map((p) => row(p.label, `${fmt(p.value ?? p.default, p.step)} ${p.unit}${p.value === null ? ' (기본, 실행기 값 모름)' : ''} · ${p.source}`));
      const key = JSON.stringify(rows.map((r) => r.textContent));
      if (list.dataset.key !== key) { list.dataset.key = key; list.replaceChildren(...rows); }
    }
    const apply = $('tuneApply');
    if (apply) { apply.disabled = busy || !view?.applicable; apply.textContent = busy ? '보내는 중…' : '적용'; }
    const notice = $('tuneNotice');
    const [kind, words] = message || (view?.applicable ? ['', ''] : ['warn', noticeText(view)]);
    if (notice) { notice.hidden = !words; notice.className = `test-result ${kind}`; notice.textContent = words; }
  }
  function noticeText(current) {
    if (!current) return '실행기 조정 값을 읽고 있다.';
    if (current.configured === false) return '자율 실행기 설정 없음: 이 대시보드에는 UBTS_EXECUTOR_URL이 없어 값을 읽거나 바꿀 수 없다.';
    if (!current.reachable) return `실행기 연결 안 됨: 값을 읽을 수 없다(${ERROR_TEXT[current.error] || current.error || '응답 없음'}).`;
    return `차가 정지 상태(대기·준비·끝)일 때만 적용한다. 지금 상태: ${current.run_state || '모름'}.`;
  }
  function renderLog() {
    const body = $('tuneLogBody');
    if (!body) return;
    const rows = logRows(logAll ? logEntries : logEntries.slice(0, LOG_FIRST), view);
    const more = $('tuneLogMore');
    if (more) {
      more.hidden = logEntries.length <= LOG_FIRST;
      more.textContent = logAll ? `최근 ${LOG_FIRST}건만` : `전체 보기 (${logEntries.length}건)`;
    }
    body.replaceChildren(...(rows.length ? rows : [['--', '기록 없음', '', '', '']]).map((cells) => {
      const tr = document.createElement('tr');
      for (const cell of cells) { const td = document.createElement('td'); td.textContent = cell; tr.append(td); }
      return tr;
    }));
  }

  async function submit(event) {
    event.preventDefault();
    if (busy || !view) return;
    const typed = Object.fromEntries([...document.querySelectorAll('#tuneForm input[name]')].map((input) => [input.name, input.value]));
    const { changes, problems } = check(view, typed);
    if (problems.length) { message = ['bad', `적용하지 않음: ${problems.map((p) => `${p.label} ${p.text}`).join(', ')}`]; draw(); return; }
    if (!Object.keys(changes).length) { message = ['warn', ERROR_TEXT.no_change]; draw(); return; }
    if (!window.confirm(`실행기에 적용할까요?\n${describeChanges(view, changes).join('\n')}`)) return;
    busy = true; message = ['testing', '적용 중']; draw();
    try {
      await page.postJson('/api/executor/params', changes);
      message = ['ok', `적용했다: ${describeChanges(view, changes).join(', ')}`];
      document.querySelectorAll('#tuneForm input[name]').forEach((input) => { input.value = ''; });
    } catch (err) {
      const code = String(err?.message || '').split(': ').pop();
      message = ['bad', `적용 거절(HTTP ${code}): 범위·정지 상태를 확인한다. 자세한 이유는 변경 기록에 남는다.`];
    } finally {
      busy = false;
      await load();
      await loadLog();
    }
  }
  $('tuneForm')?.addEventListener('submit', submit);
  $('tuneLogMore')?.addEventListener('click', () => { logAll = !logAll; renderLog(); });
  $('tuneClear')?.addEventListener('click', () => {
    document.querySelectorAll('#tuneForm input[name]').forEach((input) => { input.value = ''; });
    message = null; draw();
  });
  document.addEventListener('visibilitychange', () => routeChanged());

  function routeChanged() {
    if (active()) {
      if (!timer) { timer = setInterval(load, VIEW_MS); load(); loadLog(); }
    } else { clearInterval(timer); timer = null; }
  }
  routeChanged();
  return { routeChanged, draw };
}
