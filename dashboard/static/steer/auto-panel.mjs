// Autonomy section of the drive panel (UI-08): one press to start, STOP, the operator heartbeat while the screen is
// visible, and the state from GET /api/executor/status (docs/STEERING_SERVER_KO.md 6). Loaded by static/app/14-steer.js
// on a front-steer dashboard. Requests use the page's postJson/getJson. The page never sees the executor token.
import { HEARTBEAT_MS, STATUS_MS, autoView, heartbeatWanted, newSession, prepareRun, primaryAction, runActive, scaleLine, scaleOffer } from './auto-model.mjs';
import { ERROR_TEXT } from './tuning-model.mjs';

const HTTP_TEXT = { 403: '대시보드 세션 없음: 화면을 새로 고친다', 409: '실행기가 거절했다', 502: '실행기 연결 안 됨',
                    503: '실행기 설정 없음', 400: '요청 값 오류', 413: '요청이 너무 큼' };

export function initAutoPanel(page) {
  const $ = (id) => document.getElementById(id);
  const root = $('autoPanel');
  if (!root) return null;
  const session = newSession();
  let reply = null, pollTimer = null, beatTimer = null, polling = false, beating = false, busy = false, lastError = '';

  const active = () => page.currentRoute() === '/rover-control' && !document.hidden;

  async function poll() {
    if (polling) return;
    polling = true;
    try { reply = await page.getJson('/api/executor/status'); } catch (_) { reply = reply ? { ...reply } : null; }
    finally { polling = false; }
    draw();
  }
  async function beat() {
    if (beating || !active() || !heartbeatWanted(reply)) return;
    beating = true;
    try {
      await (page.controlRequest || page.postJson)('/api/executor/heartbeat', { session });
      if (lastError.startsWith('연결 신호')) lastError = '';
    } catch (err) {
      lastError = `연결 신호 거절: ${httpText(err)}`;
    } finally { beating = false; }
  }
  function httpText(err) {
    const code = Number(String(err && err.message || '').split(': ').pop());
    return HTTP_TEXT[code] || String(err && err.message || err);
  }
  function start() {
    if (!active()) return;
    if (!pollTimer) { pollTimer = setInterval(poll, STATUS_MS); poll(); }
    if (!beatTimer) beatTimer = setInterval(beat, HEARTBEAT_MS);
  }
  function stop() {
    clearInterval(pollTimer); pollTimer = null;
    clearInterval(beatTimer); beatTimer = null;    // no heartbeat: the executor stops the vehicle by itself
  }

  async function request(action, body = {}) {
    // Start, STOP and the other actions use the page's control transport (NET-05) where there is one.
    try { return { ok: true, reply: await (page.controlRequest || page.postJson)(`/api/executor/${action}`, body) }; }
    catch (err) { lastError = `${action}: ${httpText(err)}`; return { ok: false, reply: null }; }
  }
  const send = async (action, body = {}) => (await request(action, body)).ok;
  // A status read that is not skipped by a poll in flight (the prepare step needs one sent after the start reply).
  async function readStatus() {
    try { reply = await page.getJson('/api/executor/status'); } catch (_) { return null; }
    draw();
    return reply;
  }
  async function primary() {
    const { action } = primaryAction(reply);
    if (!action || busy) return;
    busy = true; lastError = ''; draw();
    try {
      if (action === 'prepare') {
        const raw = $('autoRowLen')?.value.trim();
        const rowLen = raw ? Number(raw) : null;
        if (raw && !(rowLen > 0)) { lastError = '열 길이는 0보다 큰 수이거나 비워 둔다'; return; }
        const result = await prepareRun({ send: request, readStatus, rowLen, sleep: (ms) => new Promise((done) => setTimeout(done, ms)) });
        if (!result.ok && result.step === 'wait') lastError = `준비: 실행기가 run을 만들었다고 알리지 않았다(상태 ${result.runState || '모름'})`;
        if (!result.ok && result.step === 'latched') lastError = '준비: 실행기가 래치 상태다. 래치 확인부터';
      } else if (action === 'go') {
        await beat();                                   // the link is fresh before the go (go_blockers: operator link)
        await send('go', { session });
      } else {
        await send(action);
      }
    } finally {
      busy = false;
      await poll();
    }
  }
  // Every STOP of the drive page also stops a run (their own handlers keep sending the bridge STOP).
  function stopRun() {
    if (!reply || reply.configured === false) return;
    if (runActive(reply) || !reply.ok) send('stop').then(poll);   // a run, or a state not known: the executor latches
  }
  // PW-10: the answer to the online-scale offer after a run (POST /api/executor/params; the reply body is kept on a
  // refusal so the executor's reason shows). The status poll above carries the offer: no polling of its own.
  async function answerScale(answer) {
    const offer = scaleOffer(reply);
    if (!offer || busy) return;
    if (answer === 'accept' && !window.confirm(`${offer.text}\n적용하면 실행기의 조정 값이 바뀐다(조정 화면의 기록에 남는다). 파일 저장은 조정 화면에서.`)) return;
    busy = true; lastError = ''; draw();
    try {
      const res = await fetch('/api/executor/params', { method: 'POST', cache: 'no-store', headers: { 'Content-Type': 'application/json' },
                                                        body: JSON.stringify({ online_scale: answer, run_id: offer.runId }) });
      let body = null;
      try { body = await res.json(); } catch (_) { body = null; }
      if (!res.ok || !body?.ok) lastError = `배율 ${answer === 'accept' ? '저장' : '닫기'} 거절: ${ERROR_TEXT[body?.error] || body?.error || `HTTP ${res.status}`}`;
    } catch (_) {
      lastError = `배율 ${answer === 'accept' ? '저장' : '닫기'}: ${ERROR_TEXT.executor_unreachable}`;
    } finally {
      busy = false;
      await poll();
    }
  }
  $('autoScaleAccept')?.addEventListener('click', () => answerScale('accept'));
  $('autoScaleDismiss')?.addEventListener('click', () => answerScale('dismiss'));
  $('autoPrimary')?.addEventListener('click', primary);
  $('autoEnd')?.addEventListener('click', () => { if (!busy) send('end').then(poll); });
  for (const id of ['frameStopButton', 'roverStopButton']) $(id)?.addEventListener('click', stopRun);
  document.addEventListener('keydown', (event) => {
    if (event.key === 'Escape' && page.currentRoute() === '/rover-control') stopRun();
  });
  document.addEventListener('visibilitychange', () => (document.hidden ? stop() : start()));

  function draw() {
    const view = autoView(reply);
    const chip = $('autoState');
    if (chip) { chip.className = `status-chip${view.chip[0] ? ` ${view.chip[0]}` : ''}`; chip.textContent = view.chip[1]; }
    const button = $('autoPrimary');
    const action = primaryAction(reply);
    if (button) { button.textContent = busy ? '보내는 중…' : action.text; button.disabled = busy || !action.enabled; }
    const end = $('autoEnd');
    if (end) end.disabled = busy || !runActive(reply) || reply.status.run_state === 'running';
    const line = $('autoScale');
    if (line) {
      const text = scaleLine(reply);
      line.hidden = text === null;
      const value = $('autoScaleValue');
      if (value && text !== null && value.textContent !== text) value.textContent = text;
    }
    const box = $('autoScaleOffer');
    if (box) {
      const offer = view.stale ? null : scaleOffer(reply);
      box.hidden = !offer;
      if (offer) {
        const text = $('autoScaleText');
        if (text && text.textContent !== offer.text) text.textContent = offer.text;
        const accept = $('autoScaleAccept');
        if (accept) { accept.hidden = !offer.canAccept; accept.disabled = busy; }
        const dismiss = $('autoScaleDismiss');
        if (dismiss) { dismiss.disabled = busy; dismiss.textContent = offer.canAccept ? '저장 안 함' : '닫기'; }
      }
    }
    const notice = $('autoNotice');
    const [kind, text] = lastError ? ['bad', lastError] : (view.notice || ['', '']);
    if (notice) { notice.hidden = !text; notice.className = `test-result ${kind}`; notice.textContent = text; }
    const list = $('autoDetails');
    if (list) {
      list.classList.toggle('stale', view.stale);
      const key = JSON.stringify(view.rows);
      if (list.dataset.key !== key) {
        list.dataset.key = key;
        list.replaceChildren(...view.rows.map(([name, value]) => {
          const row = document.createElement('div');
          const dt = document.createElement('dt'); dt.textContent = name;
          const dd = document.createElement('dd'); dd.textContent = value;
          row.append(dt, dd);
          return row;
        }));
      }
    }
  }

  function routeChanged() { if (active()) start(); else stop(); draw(); }
  routeChanged();
  return { routeChanged, draw, session: () => session, state: () => reply };
}
