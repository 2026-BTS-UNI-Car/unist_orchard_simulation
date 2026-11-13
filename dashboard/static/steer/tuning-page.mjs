// Tuning screen (UI-12, route /tuning): the executor's field tuning bundle (docs/STEERING_SERVER_KO.md 7). The basic
// group with the executor's names, values, ranges and units; preset load and save; the advanced and locked groups as
// values (the file changes them); the change log. A change is checked here, then by the dashboard, then by the
// executor; it is applied only while the executor says it may change. Loaded by static/app/14-steer.js when the route
// opens (any vehicle kind: without an executor or a bundle the screen says so).
import { ERROR_TEXT, check, describeChanges, fmt, groups, logRows, presetLine, presetNameOk, replyText, screenState, whyNot,
         withUnit } from './tuning-model.mjs';

const VIEW_MS = 2000;
const BOXES = { basic: 'tuneBasic', advanced: 'tuneAdvanced', locked: 'tuneLocked' };

// POST that keeps the reply body on a refusal (the executor's reasons and problems are shown).
async function post(path, payload) {
  const res = await fetch(path, { method: 'POST', cache: 'no-store', headers: { 'Content-Type': 'application/json' },
                                  body: JSON.stringify(payload) });
  let body = null;
  try { body = await res.json(); } catch (_) { body = null; }
  return [res.status, body];
}

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
    label.append(param.unit ? `${param.label} (${param.unit})` : param.label);
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
      const box = $(BOXES[group]);
      if (!box) continue;
      const inputs = params.filter((p) => p.editable_when_stopped);
      const fixed = params.filter((p) => !p.editable_when_stopped);
      const list = document.createElement('dl');
      list.className = 'detail-list';
      list.dataset.fixed = group;
      box.replaceChildren(...inputs.map(field), ...(fixed.length ? [list] : []));
      if (!params.length && group !== 'basic') box.replaceChildren(Object.assign(document.createElement('p'), { className: 'help-text', textContent: '--' }));
    }
    const select = $('tunePreset');
    if (select) {
      const names = Array.isArray(view?.tuning?.presets) ? view.tuning.presets : [];
      select.replaceChildren(...names.map((name) => Object.assign(document.createElement('option'), { value: name, textContent: name })));
    }
  }
  function draw() {
    const sig = JSON.stringify([(view?.params || []).map((p) => [p.name, p.group, p.editable_when_stopped, p.min, p.max]), view?.tuning?.presets]);
    if (sig !== signature) { signature = sig; build(); }
    const [level, text] = screenState(view);
    const chip = $('tuneState');
    if (chip) { chip.className = `status-chip${level ? ` ${level}` : ''}`; chip.textContent = text; }
    if ($('tuneRevision')) $('tuneRevision').textContent = presetLine(view);
    for (const param of view?.params || []) {
      const input = document.querySelector(`#tuneForm input[name="${param.name}"]`);
      const note = document.querySelector(`#tuneForm small[data-note="${param.name}"]`);
      const current = withUnit(fmt(param.value, param.step), param.unit);
      if (input) {
        input.disabled = busy || !param.editable;
        input.placeholder = fmt(param.value, param.step);
      }
      if (note) note.textContent = `지금 ${current} · 범위 ${withUnit(`${fmt(param.min, param.step)} ~ ${fmt(param.max, param.step)}`, param.unit)}`;
    }
    for (const list of document.querySelectorAll('#tuneForm dl[data-fixed]')) {
      const rows = (view?.params || []).filter((p) => p.group === list.dataset.fixed && !p.editable_when_stopped)
        .map((p) => row(p.label, withUnit(fmt(p.value, p.step), p.unit)));
      const key = JSON.stringify(rows.map((r) => r.textContent));
      if (list.dataset.key !== key) { list.dataset.key = key; list.replaceChildren(...rows); }
    }
    const can = !busy && Boolean(view?.applicable);
    const apply = $('tuneApply');
    if (apply) { apply.disabled = !can; apply.textContent = busy ? '보내는 중…' : '적용'; }
    const writable = view?.tuning?.writable === true;
    if ($('tuneLoadPreset')) $('tuneLoadPreset').disabled = !can || !(view?.tuning?.presets || []).length;
    if ($('tuneSave')) $('tuneSave').disabled = !can || !writable;
    if ($('tuneSavePreset')) $('tuneSavePreset').disabled = !can || !writable;
    if ($('tunePreset')) $('tunePreset').disabled = busy || !view?.available;
    if ($('tuneSaveName')) $('tuneSaveName').disabled = busy || !writable;
    const notice = $('tuneNotice');
    const words = message || (view?.applicable ? (writable || !view?.available ? null : ['', ERROR_TEXT.tuning_not_writable + ' (실차는 tuning_file을 패키지 밖에 둔다).'])
                                               : ['warn', whyNot(view)]);
    if (notice) { notice.hidden = !words?.[1]; notice.className = `test-result ${words?.[0] || ''}`; notice.textContent = words?.[1] || ''; }
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

  // Send one of the executor's forms, then read the bundle and the log again.
  async function send(payload, question) {
    if (busy || !view) return;
    if (!window.confirm(question)) return;
    busy = true; message = ['testing', '보내는 중']; draw();
    try {
      const [code, reply] = await post('/api/executor/params', payload);
      message = replyText(view, code, reply);
      if (reply?.ok && !('preset' in payload) && !('save' in payload) && !('save_preset' in payload)) {
        document.querySelectorAll('#tuneForm input[name]').forEach((input) => { input.value = ''; });
      }
    } catch (_) {
      message = ['bad', `거절: ${ERROR_TEXT.executor_unreachable}`];
    } finally {
      busy = false;
      await load();
      await loadLog();
    }
  }
  function submit(event) {
    event.preventDefault();
    if (busy || !view) return;
    const typed = Object.fromEntries([...document.querySelectorAll('#tuneForm input[name]')].map((input) => [input.name, input.value]));
    const { changes, problems } = check(view, typed);
    if (problems.length) { message = ['bad', `적용하지 않음: ${problems.map((p) => `${p.label} ${p.text}`).join(', ')}`]; draw(); return; }
    if (!Object.keys(changes).length) { message = ['warn', ERROR_TEXT.no_change]; draw(); return; }
    send(changes, `실행기에 적용할까요?\n${describeChanges(view, changes).join('\n')}`);
  }
  $('tuneForm')?.addEventListener('submit', submit);
  $('tuneLoadPreset')?.addEventListener('click', () => {
    const name = $('tunePreset')?.value;
    if (name) send({ preset: name }, `프리셋 ${name}을(를) 불러올까요? 기본 묶음과 고급 묶음 값이 그 프리셋 값으로 바뀐다.`);
  });
  $('tuneSave')?.addEventListener('click', () => send({ save: true }, '지금 값을 실행기의 조정 파일에 저장할까요?'));
  $('tuneSavePreset')?.addEventListener('click', () => {
    const name = ($('tuneSaveName')?.value || '').trim();
    if (!presetNameOk(name)) { message = ['bad', `${ERROR_TEXT.bad_preset_name}: 영문·숫자·_ - . 1~40자, default 아님`]; draw(); return; }
    send({ save_preset: name, note: '' }, `지금 값을 프리셋 ${name}(으)로 저장할까요?`);
  });
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
