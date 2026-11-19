// The 자율주행 tab of Rover Control (UI-08; 2026-11-19 tab, GPS choice, run recording and monitoring view; 2026-11-20
// three buttons and one status sentence): the operator heartbeat while the screen is visible and the state from
// GET /api/executor/status (docs/STEERING_SERVER_KO.md 6). Loaded by static/app/14-steer.js on a front-steer dashboard.
// Requests use the page's postJson/getJson. The page never sees the executor token.
// - Three buttons that stay in place (user 2026-11-20: "준비 → 열을 잡아서 자율주행 가능한 상태면 자율주행 버튼 활성화, 멈추기
//   버튼 누르면 멈추는, 최대한 단순한 과정"; auto-model.mjs autoButtons):
//     준비            every step by itself (auto-model.mjs prepareSequence): acknowledge a latch, end a run that is still
//                     open, start a run (row length, GPS choice), Arm. On while the vehicle is not driving.
//     자율 주행 시작    on only when the run is armed and nothing but the operator link is in the way (the corridor is
//                     acquired). A recording of every sensor first (no recording: no go), a short wait, a heartbeat,
//                     then the go. After a pause the same button reads 다시 출발.
//     멈추기           always on: the page's STOP (bridge stop by the drive panel's handler) and the executor stop.
//   The status sentence above them (auto-model.mjs statusLine) says the state and what to do next; a failed action
//   shows there until the state moves on or a button is pressed again. Why a run stopped stays in it until 준비.
// - GPS per run (user: "gps를 자율주행에 쓸지 안쓸지를 이제 결정해서 자율주행을 굴릴 수 있으면"): the choice goes with 준비
//   (POST /api/executor/start gnss_mode). It is kept per browser. A refusal of the executor is shown in its words.
// - Recording (user 2026-11-20: "자율주행 시작 버튼 누르는 순간 기록 시작 동시에 진행되게"): no button of its own. One
//   recording per driven stretch (auto-model.mjs recordBeforeGo / recordStopWanted): a new recording of every sensor
//   (tag autonomy) starts right before every go, the first one and 다시 출발; without it the go is not sent. It stops
//   as soon as the run does not drive any more, whatever stopped it (a pause too), so the stretch is saved up to that
//   place. A recording somebody started by hand with every sensor is kept and never stopped here. A page that is
//   closed can not stop anything: the dashboard server stops a run recording nobody stops and writes the record of
//   every stretch (which run, which stretch, a start again or not, why it stopped, how long and how far;
//   dashboard_server/run_recording.py).
// - The wait (GO_AFTER_RECORD_MS): a recording that was just started runs that long before the heartbeat and the go
//   (the recorder's start load made the steering bridge stand still for a moment; a run that left then ended at once).
//   멈추기 during the wait: no go, and the recording that was just started stops.
// - Driving time: the status sentence and a tile show how long the run drove (the executor's count of its driving
//   ticks, auto-model.mjs runTimes), and after a start again also the stretch since then.
// - Speed (user 2026-11-20: "자율주행 속도 설정은 따로 못 하나?"): a choice of the executor's target_speed_m_s in 0.05 m/s
//   steps (auto-model.mjs speedChoice: range, value now, lock and overspeed line from the status this panel reads).
//   A change sends that one key the way the field tuning part does (POST /api/executor/params, its refusal table); it
//   is not saved to the file (보정·설정 does that). Refused: the choice goes back to the executor's value and the status
//   line says why. Off while a run is open (the executor takes tuning changes at rest only).
// - Monitoring view: six tiles of auto-model.mjs monitorView (executor status of this poll, the steering panel's bridge
//   status: no request of its own); the other tiles, status.metrics, the online scale and its question, the mode chip,
//   the executor's own codes and 'run 끝내기' are in the folded 자세히. The local grid picture is local-grid.mjs.
// - The heartbeat keeps going in every tab of Rover Control (STOP stays in the status bar); leaving the page or hiding
//   it stops it, and then the executor stops the vehicle by itself.
import { EXEC_ERROR, GNSS_MODES, GO_AFTER_RECORD_MS, GO_BLOCKER, GO_WAIT_STEP_MS, HEARTBEAT_MS, PREPARE_STEP, RUN_RECORD_SENSORS, SPEED_KEY,
         START_ERROR, STATUS_MS, autoButtons, autoView, heartbeatWanted, metricsView, modeChip, monitorView, newSession, prepareFailText,
         prepareSequence, reasonSentence, recordBeforeGo, recordStopWanted, runActive, scaleLine, scaleOffer, speedChoice, speedLabel,
         statusLine } from './auto-model.mjs';
import { ERROR_TEXT, refusalReason } from './tuning-model.mjs';

const HTTP_TEXT = { 403: '화면 세션이 끝났습니다. 화면을 새로 고치세요.', 409: '실행기가 요청을 거절했습니다.', 502: '실행기에 연결하지 못했습니다.',
                    503: '실행기가 설정되어 있지 않습니다.', 400: '요청 값이 올바르지 않습니다.', 413: '요청 크기가 한도를 넘었습니다.' };
const GNSS_KEY = 'ubts-auto-gnss-mode-v1';
const SHARED_BRIDGE_MS = 2000;
const BADGE_STATES = new Set(['armed', 'running', 'paused']);
const INPUT_LOCK_STATES = new Set(['armed', 'running', 'paused']);   // the run's row length and GPS mode are fixed
const OPEN_STATES = new Set(['ready', 'armed', 'running', 'paused']);
const ACTION_FAIL = { go: '출발하지 못했습니다.', stop: '멈추기 요청이 거절됐습니다.', end: '이번 주행을 끝내지 못했습니다.' };
// The recorder's refusals that this panel can meet (static/app/11-recording.js), as sentences.
const RECORD_ERROR = { sensors_need_lidar_or_camera: 'LiDAR나 카메라 가운데 하나는 기록해야 합니다.', recording_busy: '기록 요청을 이미 처리하고 있습니다.',
                       no_reply: '녹화기가 응답하지 않았습니다.', recording_finalization_in_progress: '이전 기록을 마무리하는 중입니다.',
                       previous_sidecar_fsync_still_owned: '이전 기록의 저장을 마무리하는 중입니다.',
                       previous_mapping_writer_still_owned: '이전 기록의 저장을 마무리하는 중입니다.' };

export function initAutoPanel(page) {
  const $ = (id) => document.getElementById(id);
  const root = $('autoPanel');
  if (!root) return null;
  const session = newSession();
  let reply = null, replyAt = 0, pollTimer = null, beatTimer = null, polling = false, beating = false;
  let busy = '';             // '' or the words of the step in progress (the buttons are off meanwhile, 멈추기 is not)
  let error = null;          // { text, key, beat }: a failed action, shown until the state moves on (key) or a new press
  let stops = 0;             // 멈추기 presses: a 준비 in progress gives up
  let autoRecordId = null, recordBusy = false, recordStopFailed = false;
  // The stretch of the recording: the go on its way, when it was accepted, whether the run was seen driving since; the
  // stop of the recording that is on its way; the server's record of the newest stretch (for the times).
  let going = false, goAt = null, sawRunning = false, recordStop = null, segment = null;
  const steady = () => (globalThis.performance ? performance.now() : Date.now());
  const store = (() => { try { return globalThis.localStorage || null; } catch (_) { return null; } })();
  const read = (key) => { try { return store?.getItem(key) ?? null; } catch (_) { return null; } };
  const write = (key, value) => { try { store?.setItem(key, value); } catch (_) { /* this load only */ } };
  const sleep = (ms) => new Promise((done) => setTimeout(done, ms));
  // The wait between a new recording and its go (a test page gives a shorter one).
  const goWaitMs = Number.isFinite(page.goWaitMs) ? page.goWaitMs : GO_AFTER_RECORD_MS;

  const gnssSelect = $('autoGnssMode');
  if (gnssSelect) {
    const kept = read(GNSS_KEY);
    // two choices since 2026-11-20 (off / weak_constraint); an older remembered choice falls back to off
    gnssSelect.value = kept === 'weak_constraint' ? 'weak_constraint' : 'off';
    gnssSelect.addEventListener('change', () => write(GNSS_KEY, gnssSelect.value));
  }
  const gnssChoice = () => (gnssSelect && GNSS_MODES.includes(gnssSelect.value) ? gnssSelect.value : 'off');
  const recording = () => (page.recording ? page.recording.view() : null);

  const active = () => page.currentRoute() === '/rover-control' && !document.hidden;

  // The newest answer wins: a read that was asked earlier and answers later does not put an old state back.
  let asked = 0, taken = 0;
  async function fetchStatus() {
    const turn = asked += 1;
    const fresh = await page.getJson('/api/executor/status');
    if (turn > taken) { taken = turn; reply = fresh; replyAt = globalThis.performance ? performance.now() : Date.now(); }
    return reply;
  }
  async function poll() {
    if (polling) return;
    polling = true;
    try { await fetchStatus(); } catch (_) { reply = reply ? { ...reply } : null; }
    finally { polling = false; }
    afterRun();
    draw();
  }
  // A status read that is not skipped by a poll in flight (준비 decides its next step from it); null when it failed.
  async function readStatus() {
    let fresh = null;
    try { fresh = await fetchStatus(); } catch (_) { return null; }
    afterRun();
    draw();
    return fresh;
  }
  async function beat() {
    if (beating || !active() || !heartbeatWanted(reply)) return;
    beating = true;
    try {
      await (page.controlRequest || page.postJson)('/api/executor/heartbeat', { session });
      if (error && error.beat) error = null;
    } catch (err) {
      if (!error || error.beat) error = { text: `화면 연결 신호가 거절됐습니다. ${reasonSentence(refusalOf(err).text)}`, key: null, beat: true };
    } finally { beating = false; }
  }
  // The refusal of a request: the executor's own code when the reply carried one (err.body.error) and its words.
  function refusalOf(err) {
    const code = err && err.body && typeof err.body.error === 'string' ? err.body.error : null;
    if (code) return { error: code, text: words(code) };
    const raw = String(err && err.message || err);
    const status = Number(err && err.status) || Number(raw.split(': ').pop());
    return { error: null, text: HTTP_TEXT[status] || (status ? `서버가 HTTP ${status} 오류로 답했습니다.` : raw.replace(/^\/api\/\S+: /, '')) };
  }
  function words(code) {
    return START_ERROR[code] || EXEC_ERROR[code] || GO_BLOCKER[code] || ERROR_TEXT[code]
      || globalThis.UbtsSteerRules?.MESSAGES?.[code] || code;
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

  // { ok, reply, error (the executor's code or null), text (the refusal in screen words) }. Start, STOP and the other
  // actions use the page's control transport (NET-05) where there is one.
  async function request(action, body = {}) {
    try { return { ok: true, reply: await (page.controlRequest || page.postJson)(`/api/executor/${action}`, body) }; }
    catch (err) { return { ok: false, reply: null, ...refusalOf(err) }; }
  }
  // A failed action for the status line. Its key (the state it belongs to) is set once the next status is in.
  function fail(text) { error = { text, key: null, beat: false }; }
  const stateKey = () => `${reply && reply.ok && reply.status ? reply.status.run_state : ''}|${autoButtons(reply).go}`;
  const pinError = () => { if (error && !error.beat && error.key === null) error.key = stateKey(); };
  async function send(action, body = {}) {
    const answer = await request(action, body);
    if (!answer.ok) fail(`${ACTION_FAIL[action] || '요청이 거절됐습니다.'} ${reasonSentence(answer.text)}`);
    return answer.ok;
  }
  // One action of 준비 or 자율 주행 시작 at a time; the status line names its step.
  async function act(first, run) {
    if (busy) return;
    busy = first; error = null; draw();
    try { await run(); }
    finally {
      await readStatus();
      busy = '';
      pinError();
      draw();
    }
  }

  // Why the recorder refused, as a sentence (an unknown reason of the recorder is named in one).
  const recordWhy = (answer) => reasonSentence(RECORD_ERROR[answer?.error] || answer?.error || '');
  // Right before a go: a recording of its own for the stretch, or no go. { ok, fresh } (fresh: started just now).
  async function recordForGo() {
    if (recordStop) {
      // the recording of the stretch before is still being stopped (다시 출발 right after a pause)
      busy = '이전 구간의 기록을 저장하고 있습니다.'; draw();
      await recordStop;
    }
    const before = recording();
    const decision = recordBeforeGo(before, true, RUN_RECORD_SENSORS);
    if (decision === 'keep') return { ok: true, fresh: false };
    if (decision === 'unknown' || !page.recording) {
      fail('기록 상태를 확인할 수 없어 출발하지 않았습니다. 기록 탭에서 녹화기 상태를 확인하세요.');
      return { ok: false };
    }
    if (decision === 'restart') {
      // a recording of an earlier stretch that nobody stopped, or one without every sensor (e.g. started on the 기록
      // tab without the camera): stop it, then a new one of every sensor
      const stopped = await page.recording.stop();
      if (!stopped || stopped.ok === false) {
        fail(`${before.tag === 'autonomy' ? '이전 구간의 기록을 끝내지 못해' : '진행 중인 기록을 모든 센서 기록으로 바꾸지 못해'} 출발하지 않았습니다. ${recordWhy(stopped)}`);
        return { ok: false };
      }
      busy = '기록을 시작하고 있습니다.'; draw();
    }
    // The recorder may still be finishing the previous recording: try again for up to 5 s, only for that reason.
    let answer = null;
    for (let tries = 0; tries < 10; tries += 1) {
      answer = await page.recording.start({ sensors: RUN_RECORD_SENSORS, tag: 'autonomy' });
      if (answer && answer.ok !== false) break;
      if (!/finaliz|busy|already|cleanup/.test(String(answer?.state || '') + String(answer?.error || ''))) break;
      await sleep(500);
    }
    if (!answer || answer.ok === false) {
      fail(`기록을 시작하지 못해 출발하지 않았습니다. ${recordWhy(answer)}`);
      return { ok: false };
    }
    // '?' until the recorder names it (the next status read)
    autoRecordId = answer.recording_id || answer.status?.recording_id || recording()?.id || '?';
    recordStopFailed = false;
    goAt = null; sawRunning = false;
    return { ok: true, fresh: true };
  }
  // The go did not happen (멈추기, a refusal): the recording that was started for it stops; one that was kept stays.
  async function dropRecording(record) {
    if (!record.fresh || !autoRecordId) return;
    let answer = null;
    try { answer = await page.recording.stop(); } catch (_) { answer = null; }
    if (answer && answer.ok !== false) autoRecordId = null;
    else recordStopFailed = true;
  }
  // After every status read: the recording of the stretch stops as soon as the run does not drive any more.
  function afterRun() {
    const rec = recording();
    if (rec?.segment) segment = rec.segment;
    if (autoRecordId === '?' && rec?.active && rec.id) autoRecordId = rec.id;
    // stopped elsewhere (the server's keeper, the 기록 tab)
    if (autoRecordId && !going && !recordBusy && rec?.known && !rec.active) { autoRecordId = null; recordStopFailed = false; return; }
    if (autoRecordId && reply && reply.ok && reply.status && reply.status.run_state === 'running') sawRunning = true;
    if (recordBusy || !recordStopWanted(reply, autoRecordId, rec, { going, sawRunning, sinceGoMs: goAt === null ? null : steady() - goAt })) return;
    recordBusy = true;
    recordStop = Promise.resolve().then(() => page.recording.stop()).then((answer) => {
      recordStopFailed = !(answer && answer.ok !== false);
      if (!recordStopFailed) autoRecordId = null;
    }).catch(() => { recordStopFailed = true; }).finally(() => { recordBusy = false; recordStop = null; draw(); });
  }

  // 준비: whatever the executor's state needs, then a new run and Arm (auto-model.mjs prepareSequence).
  function prepare() {
    if (!autoButtons(reply).prepare) return null;
    return act('준비하고 있습니다.', async () => {
      const raw = $('autoRowLen')?.value.trim();
      const rowLen = raw ? Number(raw) : null;
      if (raw && !(rowLen > 0)) { fail('열 길이는 0보다 큰 수로 넣거나 비워 두세요.'); return; }
      const mark = stops;
      const result = await prepareSequence({ send: request, readStatus, sleep, rowLen, gnssMode: gnssChoice() || null,
                                             aborted: () => stops !== mark,
                                             onStep: (step) => { busy = PREPARE_STEP[step] || '준비하고 있습니다.'; draw(); } });
      if (stops !== mark) {
        // 멈추기 while 준비 was on its way: a run that it left open (or a state not known) is stopped as a run is.
        const fresh = await readStatus();
        const state = fresh && fresh.ok && fresh.status ? fresh.status.run_state : null;
        if (!fresh || !fresh.ok || OPEN_STATES.has(state)) await request('stop');
        fail(prepareFailText({ step: 'aborted' }));
      } else if (!result.ok) fail(prepareFailText(result, words));
    });
  }
  // 자율 주행 시작 / 다시 출발: a recording of its own for the stretch, a wait while a new recording settles, the
  // operator link fresh, then the go. 멈추기 anywhere before the go: no go, and the new recording stops.
  function go() {
    if (!autoButtons(reply).go) return null;
    return act('기록을 시작하고 있습니다.', async () => {
      const mark = stops;
      const halted = () => stops !== mark;
      going = true;
      try {
        const record = await recordForGo();
        if (!record.ok) return;
        if (record.fresh && !halted()) {
          busy = '기록을 시작했습니다. 잠시 뒤 출발합니다.'; draw();
          for (let waited = 0; waited < goWaitMs && !halted(); waited += GO_WAIT_STEP_MS) await sleep(Math.min(GO_WAIT_STEP_MS, goWaitMs - waited));
        }
        if (!halted()) {
          busy = '출발 요청을 보내고 있습니다.'; draw();
          await beat();                                 // the link is fresh before the go (go_blockers: operator link)
        }
        if (halted()) {
          await dropRecording(record);
          fail('멈추기를 눌러 출발하지 않았습니다.');
          return;
        }
        if (await send('go', { session })) goAt = steady();
        else await dropRecording(record);
      } finally { going = false; }
    });
  }
  // Every STOP of the page also stops a run (their own handlers keep sending the bridge STOP).
  function stopRun() {
    stops += 1;
    if (!reply || reply.configured === false) return;
    if (runActive(reply) || !reply.ok) send('stop').then(readStatus).then(() => { pinError(); draw(); });   // a run, or a state not known: the executor latches
  }
  // POST /api/executor/params that keeps the reply body on a refusal (the executor's reason and problems are shown),
  // as the field tuning part sends it: [HTTP status, reply or null].
  async function postParams(payload) {
    const res = await fetch('/api/executor/params', { method: 'POST', cache: 'no-store', headers: { 'Content-Type': 'application/json' },
                                                      body: JSON.stringify(payload) });
    let body = null;
    try { body = await res.json(); } catch (_) { body = null; }
    return [res.status, body];
  }
  // The speed choice: one key, not saved to the file. The status read after it shows the executor's value (the new one,
  // or the old one after a refusal: the choice goes back by itself).
  const speedSelect = $('autoSpeed');
  function changeSpeed() {
    const now = speedChoice(reply);
    const wanted = Number(speedSelect?.value);
    if (!now.enabled || !(wanted > 0) || (now.value !== null && Math.abs(wanted - now.value) < 1e-9)) { draw(); return null; }
    return act('속도를 바꾸고 있습니다.', async () => {
      try {
        const [code, body] = await postParams({ [SPEED_KEY]: wanted });
        if (!body?.ok) fail(`속도를 바꾸지 못했습니다. ${refusalReason(code, body)}`);
      } catch (_) {
        fail(`속도를 바꾸지 못했습니다. ${ERROR_TEXT.executor_unreachable}`);
      }
    });
  }
  speedSelect?.addEventListener('change', changeSpeed);
  // PW-10: the answer to the online-scale offer after a run (POST /api/executor/params; the reply body is kept on a
  // refusal so the executor's reason shows). The status poll above carries the offer: no polling of its own.
  async function answerScale(answer) {
    const offer = scaleOffer(reply);
    if (!offer || busy) return;
    if (answer === 'accept' && !window.confirm(`${offer.text}\n저장하면 실행기의 조정 값이 바뀌고 현장 조정 값의 변경 기록에 남습니다. 파일에는 보정·설정 탭의 현장 조정 값에서 따로 저장하세요.`)) return;
    const accept = answer === 'accept';
    const failed = accept ? '배율을 저장하지 못했습니다.' : '배율 제안을 닫지 못했습니다.';
    await act(accept ? '배율을 저장하고 있습니다.' : '배율 제안을 닫고 있습니다.', async () => {
      try {
        const [code, body] = await postParams({ online_scale: answer, run_id: offer.runId });
        if (!(code >= 200 && code < 300) || !body?.ok) fail(`${failed} ${reasonSentence(ERROR_TEXT[body?.error] || body?.error || `서버가 HTTP ${code} 오류로 답했습니다.`)}`);
      } catch (_) {
        fail(`${failed} ${ERROR_TEXT.executor_unreachable}`);
      }
    });
  }
  $('autoScaleAccept')?.addEventListener('click', () => answerScale('accept'));
  $('autoScaleDismiss')?.addEventListener('click', () => answerScale('dismiss'));
  $('autoPrepare')?.addEventListener('click', prepare);
  $('autoPrimary')?.addEventListener('click', go);
  $('autoEnd')?.addEventListener('click', () => { if (!busy) send('end').then(readStatus).then(() => { pinError(); draw(); }); });
  for (const id of ['frameStopButton', 'roverStopButton', 'autoStopButton']) $(id)?.addEventListener('click', stopRun);
  document.addEventListener('keydown', (event) => {
    if (event.key === 'Escape' && page.currentRoute() === '/rover-control') stopRun();
  });
  document.addEventListener('visibilitychange', () => (document.hidden ? stop() : start()));

  function setText(el, value) { if (el && el.textContent !== value) el.textContent = value; }
  function chip(el, level, value) {
    if (!el) return;
    const name = `status-chip${level ? ` ${level}` : ''}`;
    if (el.className !== name) el.className = name;
    setText(el, value);
  }
  // Tiles [label, value, sub, level] into a <dl class="monitor-grid"> (or a detail list).
  function tiles(list, rows, isStale = false) {
    if (!list) return;
    list.classList?.toggle('stale', isStale);
    const key = JSON.stringify(rows);
    if (list.dataset.key === key) return;
    list.dataset.key = key;
    list.replaceChildren(...rows.map(([name, value, sub, level]) => {
      const row = document.createElement('div');
      if (level) row.dataset.level = level;
      const dt = document.createElement('dt'); dt.textContent = name;
      const dd = document.createElement('dd'); dd.textContent = value;
      row.append(dt, dd);
      if (sub) { const small = document.createElement('small'); small.textContent = sub; row.append(small); }
      return row;
    }));
  }
  function draw() {
    const view = autoView(reply);
    const buttons = autoButtons(reply);
    const s = reply && reply.ok && reply.status ? reply.status : null;
    // A failure belongs to the state it happened in: once the run state or the start button moved on, it goes.
    if (error && !error.beat && error.key !== null && error.key !== stateKey()) error = null;
    if (error && error.beat && !heartbeatWanted(reply)) error = null;
    const [level, text] = busy ? ['testing', busy] : error ? ['bad', error.text] : statusLine(reply, Date.now(), segment);
    const line = $('autoStatus');
    if (line) {
      const name = `test-result auto-status${level ? ` ${level}` : ''}`;
      if (line.className !== name) line.className = name;
      setText(line, text);
    }
    const prepareButton = $('autoPrepare');
    if (prepareButton) prepareButton.disabled = Boolean(busy) || !buttons.prepare;
    const goButton = $('autoPrimary');
    if (goButton) { goButton.disabled = Boolean(busy) || !buttons.go; setText(goButton, buttons.goText); }
    const locked = Boolean(busy) || Boolean(s && INPUT_LOCK_STATES.has(s.run_state));
    if (gnssSelect) gnssSelect.disabled = locked;
    const rowLen = $('autoRowLen');
    if (rowLen) rowLen.disabled = locked;
    // The speed choice follows the executor: its steps, its value now (also one changed elsewhere), its lock.
    const speed = speedChoice(reply);
    if (speedSelect) {
      const key = JSON.stringify(speed.options);
      if (speedSelect.dataset.key !== key) {
        speedSelect.dataset.key = key;
        speedSelect.replaceChildren(...(speed.options.length ? speed.options : [null]).map((step) => {
          const option = document.createElement('option');
          option.value = step === null ? '' : String(step);
          option.textContent = step === null ? '--' : speedLabel(step);
          return option;
        }));
      }
      const shown = speed.value === null ? '' : String(speed.value);
      if (!busy && speedSelect.value !== shown) speedSelect.value = shown;      // while a change is on its way: the pick
      speedSelect.disabled = Boolean(busy) || !speed.enabled;
    }
    const speedNote = $('autoSpeedNote');
    if (speedNote) { speedNote.hidden = !speed.note; setText(speedNote, speed.note); }
    const timeout = Number(s?.operator_link?.timeout_s);
    if (timeout > 0) setText($('autoHint'), `화면을 끄거나 가리면 ${Number.isInteger(timeout) ? timeout : timeout.toFixed(1)}초 뒤 차가 멈춥니다.`);
    // The tab badge: a run that holds the vehicle shows in the tab row from every tab.
    const badge = $('roverTabAutoBadge');
    if (badge) {
      const on = Boolean(s && !view.stale && BADGE_STATES.has(s.run_state));
      badge.hidden = !on;
      if (on) setText(badge, view.chip[1]);
    }
    // Monitoring view: six tiles; the rest in the folded 자세히.
    const rec = recording();
    const shared = page.driveStatus?.();
    const bridge = shared && Date.now() - shared.at <= SHARED_BRIDGE_MS ? shared.reply : null;
    if (rec?.segment) segment = rec.segment;
    const monitor = monitorView(reply, bridge, rec ? { ...rec, auto: Boolean(autoRecordId && rec.id === autoRecordId),
                                                       stopFailed: recordStopFailed } : null, gnssChoice(), Date.now(), segment);
    tiles($('autoMonitorGrid'), monitor.tiles, monitor.stale);
    tiles($('autoMoreGrid'), monitor.more, monitor.stale);
    const monitorState = $('autoMonitorState');
    if (monitorState) {
      const tag = `tag ${monitor.stale ? 'bad' : view.chip[0]}`.trim();
      if (monitorState.className !== tag) monitorState.className = tag;
      setText(monitorState, monitor.stale ? '끊김' : view.chip[1]);
    }
    // 자세히: the mode chip, run 끝내기, the centreline metrics, the online scale and its question, the executor's rows.
    const mode = $('autoMode');
    const modeView = modeChip(reply);
    if (mode) {
      mode.hidden = !modeView;
      if (modeView) chip(mode, modeView[0], modeView[1]);
    }
    const end = $('autoEnd');
    if (end) end.disabled = Boolean(busy) || !runActive(reply) || reply.status.run_state === 'running';
    const metrics = metricsView(reply);
    const block = $('autoMetricsBlock');
    if (block) block.hidden = !metrics;
    if (metrics) { tiles($('autoMetricsGrid'), metrics.tiles, monitor.stale); setText($('autoMetricsSource'), metrics.source); }
    const scale = $('autoScale');
    if (scale) {
      const value = scaleLine(reply);
      scale.hidden = value === null;
      if (value !== null) setText($('autoScaleValue'), value);
    }
    const box = $('autoScaleOffer');
    if (box) {
      const offer = view.stale ? null : scaleOffer(reply);
      box.hidden = !offer;
      if (offer) {
        setText($('autoScaleText'), offer.text);
        const accept = $('autoScaleAccept');
        if (accept) { accept.hidden = !offer.canAccept; accept.disabled = Boolean(busy); }
        const dismiss = $('autoScaleDismiss');
        if (dismiss) { dismiss.disabled = Boolean(busy); setText(dismiss, offer.canAccept ? '저장 안 함' : '닫기'); }
      }
    }
    tiles($('autoDetails'), view.rows.map(([name, value]) => [name, value, '', '']), view.stale);
  }

  function routeChanged() { if (active()) start(); else stop(); draw(); }
  routeChanged();
  return { routeChanged, draw, prepare, go, changeSpeed, session: () => session, state: () => reply, stateAt: () => replyAt,
           autoRecording: () => autoRecordId,
           busy: () => busy };
}
