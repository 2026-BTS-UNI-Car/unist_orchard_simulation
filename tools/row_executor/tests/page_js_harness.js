// Runs the operator page's inline script (operator_http.PAGE) against a minimal fake DOM, fake timers and a
// scripted fetch; prints one JSON object with the observations of each scenario. Used by test_page_js.py.
// No browser, no network. Usage: node page_js_harness.js <page_script.js>
'use strict';
const fs = require('fs');
const vm = require('vm');

const CODE = fs.readFileSync(process.argv[2], 'utf8');

function statusBody(mode, extra) {
  return Object.assign({
    mode, shadow_auto: false, profile: 'field', state: 'PAUSED', ctl: [null, null], reason: 'await_hold',
    end_reason: null, vehicle_control: mode === 'SHADOW' ? 'none' : 'not_armed',
    stop_button: mode === 'SHADOW' ? 'ends_shadow_run_only' : 'vehicle_stop',
    log: {state: 'ok', dropped: 0, last_error: null}, loop_exceptions: 0, lidar_age: 0.05, imu_age: 0.02,
    counts: {scans_dropped: 0}, scan: {}, feasibility: {status: 'ok'}, onsite_limits: {}, params: null,
  }, extra || {});
}

function makeEnv(handler) {
  let now = 0;
  let nextId = 1;
  const timers = [];
  const elements = {};
  const requests = [];
  const listeners = {doc: {}, win: {}};
  const contexts = {};
  function canvas(id) {
    if (!contexts[id]) {
      const ops = [];
      const ctx = {ops, fillStyle: '', strokeStyle: '', lineWidth: 1, font: ''};
      for (const name of ['clearRect', 'fillRect', 'beginPath', 'moveTo', 'lineTo', 'fill', 'closePath', 'arc',
                          'setLineDash', 'fillText', 'save', 'restore']) {
        ctx[name] = (...args) => { ops.push([name].concat(args)); };
      }
      ctx.stroke = () => { ops.push(['stroke', ctx.strokeStyle, ctx.lineWidth]); };
      contexts[id] = ctx;
    }
    return contexts[id];
  }
  function el(id) {
    if (!elements[id]) {
      const own = {};
      const set = new Set();
      elements[id] = {
        id, textContent: '', className: '', hidden: false, disabled: false, value: '', innerHTML: '',
        classList: {add: (c) => set.add(c), remove: (c) => set.delete(c), contains: (c) => set.has(c)},
        addEventListener: (t, fn) => { (own[t] = own[t] || []).push(fn); },
        setPointerCapture: () => {},
        getContext: () => canvas(id),       // recording 2D context (single-row panel canvas)
        fire: (t) => { for (const fn of (own[t] || [])) fn({preventDefault() {}, pointerId: 1}); },
      };
    }
    return elements[id];
  }
  class FakeAbortController {
    constructor() { this.signal = {aborted: false, fns: [], addEventListener(t, fn) { this.fns.push(fn); }}; }
    abort() { this.signal.aborted = true; for (const fn of this.signal.fns) fn(); }
  }
  const sandbox = {
    console,
    document: {getElementById: el, hidden: false,
               addEventListener: (t, fn) => { (listeners.doc[t] = listeners.doc[t] || []).push(fn); }},
    window: {addEventListener: (t, fn) => { (listeners.win[t] = listeners.win[t] || []).push(fn); }},
    location: {reloads: 0, reload() { this.reloads += 1; }},
    crypto: {getRandomValues(a) { for (let i = 0; i < a.length; i++) a[i] = (i * 37 + nextId * 11) & 255; nextId++; return a; }},
    AbortController: FakeAbortController,
    setTimeout: (fn, ms) => { const id = nextId++; timers.push({id, at: now + (ms || 0), fn, every: 0}); return id; },
    clearTimeout: (id) => { const i = timers.findIndex((t) => t.id === id); if (i >= 0) timers.splice(i, 1); },
    setInterval: (fn, ms) => { const id = nextId++; timers.push({id, at: now + ms, fn, every: ms}); return id; },
    clearInterval: (id) => { const i = timers.findIndex((t) => t.id === id); if (i >= 0) timers.splice(i, 1); },
    fetch: (path, opts) => {
      const body = opts && opts.body ? JSON.parse(opts.body) : undefined;
      const req = {path, body, t: now};
      requests.push(req);
      const answer = handler(path, body, req);
      return new Promise((resolve, reject) => {
        const signal = opts && opts.signal;
        if (signal) signal.addEventListener('abort', () => reject(new Error('aborted')));
        const deliver = (a) => resolve({status: a.status, json: () => Promise.resolve(a.body)});
        if (answer === 'hang') return;
        if (answer && typeof answer.then === 'function') { answer.then(deliver); return; }
        deliver(answer);
      });
    },
  };
  sandbox.window.document = sandbox.document;
  vm.createContext(sandbox);
  const env = {
    sandbox, el, requests, listeners, contexts,
    async flush() { for (let i = 0; i < 20; i++) await new Promise((r) => setImmediate(r)); },
    async advance(ms) {
      const end = now + ms;
      for (;;) {
        timers.sort((a, b) => a.at - b.at);
        const t = timers[0];
        if (!t || t.at > end) break;
        now = t.at;
        if (t.every) t.at += t.every; else timers.shift();
        t.fn();
        await env.flush();
      }
      now = end;
      await env.flush();
    },
    get now() { return now; },
  };
  vm.runInContext(CODE, sandbox);
  return env;
}

async function scenario(mode, stopAnswer, opts) {
  opts = opts || {};
  let statusAnswer = {status: 200, body: statusBody(mode, opts.status)};
  const env = makeEnv((path) => {
    if (path === '/api/status') return opts.statusAnswer || statusAnswer;
    if (path === '/api/stop') return stopAnswer;
    return {status: 200, body: {ok: true}};
  });
  await env.flush();
  await env.advance(600);                     // first status polls
  const before = {banner: env.el('banner').textContent, bannerClass: env.el('banner').className,
                  stop: env.el('stop').textContent, stopClass: env.el('stop').className,
                  hold: env.el('hold').textContent, alert: env.el('alert').textContent,
                  alertHidden: env.el('alert').hidden, holdDisabled: env.el('hold').disabled};
  let after = null;
  if (stopAnswer !== undefined) {
    env.el('stop').fire('click');
    await env.flush();
    await env.advance(3000);                  // STOP fetch timeout is 2.5 s
    after = {alert: env.el('alert').textContent, alertClass: env.el('alert').className,
             alertHidden: env.el('alert').hidden, reloads: env.sandbox.location.reloads,
             holdDisabled: env.el('hold').disabled};
    await env.advance(6000);                  // 403: reload after 5 s
    after.reloadsLater = env.sandbox.location.reloads;
  } else {
    await env.advance(6000);
    before.reloadsLater = env.sandbox.location.reloads;
    before.tableClass = env.el('tab').className;
    before.alertLater = env.el('alert').textContent;
  }
  return {before, after, requests: env.requests.map((r) => r.path)};
}

async function releaseOrder() {
  // one device: release then a quick new press; the new press's first beat must follow the release request
  let resolveRelease = null;
  const env = makeEnv((path) => {
    if (path === '/api/status') return {status: 200, body: statusBody('MOTOR')};
    if (path === '/api/release') return new Promise((r) => { resolveRelease = () => r({status: 200, body: {ok: true}}); });
    return {status: 200, body: {ok: true}};
  });
  await env.advance(600);
  const H = env.el('hold');
  H.fire('pointerdown');
  await env.flush();
  await env.advance(250);
  H.fire('pointerup');
  await env.flush();
  const releaseIndex = env.requests.length - 1;
  const firstPress = env.requests.filter((r) => r.path === '/api/hold')[0].body.press;
  H.fire('pointerdown');                      // new press while the release is still in flight
  await env.flush();
  const beatsWhilePending = env.requests.slice(releaseIndex + 1).filter((r) => r.path === '/api/hold').length;
  resolveRelease();
  await env.flush();
  await env.advance(150);
  const later = env.requests.slice(releaseIndex + 1).filter((r) => r.path === '/api/hold');
  // a hung release must not block the next press for longer than 0.7 s
  H.fire('pointerup');
  await env.flush();
  H.fire('pointerdown');
  await env.flush();
  const idx = env.requests.length;
  await env.advance(400);
  const beforeCap = env.requests.slice(idx).filter((r) => r.path === '/api/hold').length;
  await env.advance(500);
  const afterCap = env.requests.slice(idx).filter((r) => r.path === '/api/hold').length;
  return {releasePath: env.requests[releaseIndex].path, beatsWhilePending, beatsAfterRelease: later.length,
          newPressDiffers: later.length > 0 && later[0].body.press !== firstPress, beforeCap, afterCap};
}

(async () => {
  const out = {};
  out.shadow_stop = await scenario('SHADOW', {status: 200, body: {ok: true, stop_sent: null, shadow: true}});
  out.motor_ok = await scenario('MOTOR', {status: 200, body: {ok: true, stop_sent: true, shadow: false}});
  out.motor_502 = await scenario('MOTOR', {status: 502, body: {ok: true, stop_sent: false, error: 'timeout'}});
  out.motor_403 = await scenario('MOTOR', {status: 403, body: {ok: false, error: 'forbidden'}});
  out.motor_timeout = await scenario('MOTOR', 'hang');
  out.armed = await scenario('MOTOR', undefined, {status: {vehicle_control: 'armed'}});
  out.status_403 = await scenario('MOTOR', undefined, {statusAnswer: {status: 403, body: {ok: false}}});
  out.status_lost = await scenario('MOTOR', undefined, {statusAnswer: 'hang'});
  out.log_failing = await scenario('SHADOW', undefined,
                                   {status: {log: {state: 'failing', dropped: 12, last_error: 'OSError: [Errno 28]'}}});
  out.release_order = await releaseOrder();
  out.conflict = await (async () => {
    const env = makeEnv((path) => {
      if (path === '/api/status') return {status: 200, body: statusBody('MOTOR')};
      if (path === '/api/hold') return {status: 409, body: {ok: false, error: 'press_conflict'}};
      return {status: 200, body: {ok: true}};
    });
    await env.advance(600);
    env.el('hold').fire('pointerdown');
    await env.flush();
    await env.advance(300);
    return {alert: env.el('alert').textContent, alertClass: env.el('alert').className,
            holdOn: env.el('hold').classList.contains('on'),
            beats: env.requests.filter((r) => r.path === '/api/hold').length,
            releases: env.requests.filter((r) => r.path === '/api/release').length};
  })();
  out.calib = await (async () => {
    // calibration card (calib_runs.py): options from the status, bounded request bodies, results table
    const options = {floor_pct: 34, floor_source: 'breakaway_duty_pct', start_floor_pct: 34,
      start_floor_source: 'breakaway_duty_pct', base_pct: 55, arc_default: 'd10_d20', stop: {duty: 55, ok: true, why: null},
      arc_sets: [{id: 'd10', steps: [[55, 45], [45, 55]], ok: true, why: null},
                 {id: 'd10_d20', steps: [[55, 45], [45, 55], [55, 35], [35, 55]], ok: true, why: null},
                 {id: 'sym20', steps: [[65, 45], [45, 65]], ok: false, why: 'outer 65 above duty_cap_pct 62'}],
      rolldown: [{step_pct: 3, duties: [55, 52, 49, 46, 43, 40], ok: true, why: null}], distance_m: [2, 15], time_s: [5, 45]};
    const calib = {kind: 'arc', phase: 'ended', step_index: 3, steps_total: 4, step: null, counted_m: 9.1, distance_cap_m: 15,
      end_reason: 'calib_complete', results: [{i: 0, L: 55, R: 45, diff: -10, seconds: 3, gyro_yaw_dps: -1.5, lidar_yaw_dps: -1.48,
      speed_m_s: 0.42, gain_dps_per_pct: 0.148, valid: true}, {i: 1, L: 45, R: 55, diff: 10, seconds: 1, gyro_yaw_dps: 1.5,
      lidar_yaw_dps: null, speed_m_s: 0.4, gain_dps_per_pct: 0.15, valid: false}],
      outcome: {sides: {left: {gain: 0.15}, right: {gain: 0.148}}, full_diff_pct: 20, yaw_at_full_diff_dps: 2.96, extrapolated: false}};
    const env = makeEnv((path) => {
      if (path === '/api/status') return {status: 200, body: statusBody('MOTOR', {calib_options: options, calib: calib,
        row_motor: false, calib_allowed: true, yaw_params: {measured_yaw_at_max_diff_deg_s: 3.0, max_yaw_rate_deg_s: 1.8}})};
      return {status: 200, body: {ok: true}};
    });
    await env.advance(600);
    const planArc = env.el('calibplan').textContent;
    const filled = {set: env.el('c_set').value, pct: env.el('c_pct').value, dist: env.el('c_dist').value,
                    setOptions: env.el('c_set').innerHTML};
    env.el('c_dist').value = '8';
    env.el('calibstart').fire('click');
    await env.flush();
    env.el('c_kind').value = 'rolldown';
    env.el('calibstart').fire('click');
    await env.flush();
    await env.advance(600);
    const planRoll = env.el('calibplan').textContent;
    env.el('c_kind').value = 'stop';                       // day-2: C7 stop kind
    env.el('calibstart').fire('click');
    await env.flush();
    await env.advance(600);
    const planStop = env.el('calibplan').textContent;
    const stopCalib = {kind: 'stop', phase: 'ended', step_index: 0, steps_total: 1, step: null, counted_m: 3.2,
      distance_cap_m: 15, end_reason: 'calib_stop_test', results: [{i: 0, duty: 55, seconds: 3, speed_m_s: 0.45,
      min_speed_m_s: 0.43, stalled: false, steady: true}], outcome: {steady: true, speed_m_s: 0.45},
      stop: {done: true, ok: true, stop_distance_m: 0.31, v_at_zero_m_s: 0.45, time_to_rest_s: 1.2}};
    const envStop = makeEnv((path) => (path === '/api/status' ? {status: 200, body: statusBody('MOTOR',
      {calib_options: options, calib: stopCalib, row_motor: false, calib_allowed: true})} : {status: 200, body: {ok: true}}));
    await envStop.advance(600);
    const shadow = makeEnv((path) => (path === '/api/status' ? {status: 200, body: statusBody('SHADOW')} : {status: 200, body: {ok: true}}));
    await shadow.advance(600);
    return {cardHidden: env.el('calibcard').hidden, shadowCardHidden: shadow.el('calibcard').hidden, filled,
            plan: planRoll, planArc, planStop, tab: env.el('calibtab').innerHTML, out: env.el('calibout').textContent,
            stopOut: envStop.el('calibout').textContent, stopTab: envStop.el('calibtab').innerHTML,
            limits: env.el('limits').textContent, banner: env.el('banner').textContent,
            bodies: env.requests.filter((r) => r.path === '/api/calib_start').map((r) => r.body)};
  })();
  out.single_row = await (async () => {
    // single-row panel (controller_mode single_row): numbers, category text, optional row length, panel STOP, canvas
    const geometry = {frame: 'estimator_odometry_run_start', base: [1.0, 0.1, 0.05],
      band: {heading_rad: 0.02, centre_m: 0.0, width_m: 3.6, end_left_m: 20.0, end_right_m: null},
      path: [[0, 0], [0.2, 0.01], [0.4, 0.03], [1.0, 0.1]], footprint_m: [1.22, 0.7]};
    const sr = {phase: 'APPROACH_END', localisation: 'SENSOR_ONLY_SINGLE_ROW', lateral_error_m: 0.123,
      heading_error_deg: -2.5, confidence: {level: 'LOW', inputs_fresh: true, corridor_valid: true, corridor_age_s: 0.8,
      odom_valid: true}, left_pct: 52, right_pct: 58, differential_pct: 3.0, differential_limit_pct: 20, saturated: true,
      saturation_fraction: 0.25, authority_state: 'SLOW', bias_pct: -1.5, base_duty_pct: 55, target_speed_m_s: 0.2,
      row_end_ahead_m: 2.5, end_remaining_m: 1.89, distance_m: 41.2, speed_m_s: 0.21, stop_reason: null, stop_category: null};
    let current = statusBody('SHADOW', {controller_mode: 'single_row', single_row: sr, single_row_geometry: geometry,
      single_row_settings: {localisation: 'SENSOR_ONLY_SINGLE_ROW'}, native_navigation: {provider_present: false}});
    const env = makeEnv((path) => (path === '/api/status' ? {status: 200, body: current} : {status: 200, body: {ok: true}}));
    await env.advance(600);
    const shown = {cardHidden: env.el('srcard').hidden, phase: env.el('srphase').textContent, tab: env.el('srtab').innerHTML,
                   loc: env.el('srloc').textContent, stop: env.el('srstop').textContent};
    const all = env.contexts.srcanvas ? env.contexts.srcanvas.ops.slice() : [];
    const last = all.map((o) => o[0]).lastIndexOf('clearRect');
    const ops = last >= 0 ? all.slice(last) : [];          // the last drawn frame only
    env.el('srstart').fire('click');                        // panel Start without a length -> null
    await env.flush();
    env.el('srlen').value = '64.5';
    env.el('srstart').fire('click');                        // panel Start with a length
    await env.flush();
    env.el('rowlen').value = '';
    env.el('start').fire('click');                          // main Start without a length (allowed in single_row)
    await env.flush();
    const startMsg = env.el('msg').textContent;
    env.el('srstopbtn').fire('click');                      // panel STOP = the page STOP
    await env.flush();
    await env.advance(3000);
    current = statusBody('SHADOW', {controller_mode: 'single_row', state: 'ENDED', single_row: Object.assign({}, sr,
      {phase: 'STOPPED', stop_reason: 'row_end', stop_category: 'ROW_END', stop_label: 'ROW_END',
       first_stop: {t: 88.2, state: 'ENDED', reason: 'row_end', category: 'ROW_END'}}), single_row_geometry: null});
    await env.advance(600);
    const stopped = {phase: env.el('srphase').textContent, stop: env.el('srstop').textContent,
                     stopClass: env.el('srstop').className, tab: env.el('srtab').innerHTML,
                     texts: env.contexts.srcanvas.ops.filter((o) => o[0] === 'fillText').map((o) => o[1])};
    current = statusBody('SHADOW', {controller_mode: 'single_row', state: 'ENDED', single_row: Object.assign({}, sr,
      {phase: 'STOPPED', stop_reason: 'row_distance_backstop', stop_category: 'OPERATOR_OR_BACKSTOP'})});
    await env.advance(600);
    const backstop = {stop: env.el('srstop').textContent, stopClass: env.el('srstop').className};
    const plain = makeEnv((path) => (path === '/api/status' ? {status: 200, body: statusBody('SHADOW')}
                                                            : {status: 200, body: {ok: true}}));
    await plain.advance(600);
    plain.el('rowlen').value = '';
    plain.el('start').fire('click');
    await plain.flush();
    // the yellow front edge of the vehicle: its points must lie ABOVE the vehicle centre (the corridor axis points up)
    const front = [];
    for (let i = 0; i < ops.length; i++) {
      if (ops[i][0] === 'stroke' && ops[i][1] === '#ffeb3b') {
        for (let j = i - 1; j >= 0 && ops[j][0] !== 'beginPath'; j--) {
          if (ops[j][0] === 'moveTo' || ops[j][0] === 'lineTo') front.push([ops[j][1], ops[j][2]]);
        }
      }
    }
    const points = ops.filter((o) => o[0] === 'moveTo' || o[0] === 'lineTo');
    return {shown, ops: ops.length, strokes: ops.filter((o) => o[0] === 'stroke').map((o) => o[1]),
            finite: points.length > 0 && points.every((o) => Number.isFinite(o[1]) && Number.isFinite(o[2])),
            front, stopped, backstop, startMsg,
            starts: env.requests.filter((r) => r.path === '/api/start').map((r) => r.body),
            stops: env.requests.filter((r) => r.path === '/api/stop').length,
            plainHidden: plain.el('srcard').hidden, plainStarts: plain.requests.filter((r) => r.path === '/api/start').length,
            plainMsg: plain.el('msg').textContent};
  })();
  out.one_press = await (async () => {
    // start_mode one_press (design v2 V2-2): heartbeats while visible, the big button sends one go of the same
    // session, hiding the page stops the heartbeats; the field tuning card replaces the differential keys
    const ready = statusBody('MOTOR', {start_mode: 'one_press', vehicle_control: 'armed', run_state: 'armed',
      go_ready: true, go_blockers: [], operator_link: {fresh: true, age_s: 0.05, timeout_s: 0.5},
      speed: {target_mps: 0.4, limit_mps: null, limit_reason: 'not_running'},
      tuning: {preset: 'default', revision: 0, editable: true, digest: 'abcdef0123456789',
               presets: ['default', 'slow_030'], basic: [{name: 'k_d', value: 1, low: 0, high: 3, unit: ''}]}});
    let status = ready;
    const env = makeEnv((path) => {
      if (path === '/api/status') return {status: 200, body: status};
      return {status: 200, body: {ok: true}};
    });
    await env.flush();
    await env.advance(600);
    const count = (p) => env.requests.filter((r) => r.path === p).length;
    const beatsVisible = count('/api/heartbeat');
    const ready_ = {holdText: env.el('hold').textContent, holdDisabled: env.el('hold').disabled,
                    tuningHidden: env.el('tuning').hidden, diffHidden: env.el('diffparams').hidden,
                    form: env.el('tuningform').innerHTML, presets: env.el('tnpreset').innerHTML,
                    info: env.el('tuninginfo').textContent, banner: env.el('banner').textContent};
    env.el('hold').fire('pointerdown');
    await env.flush();
    env.el('hold').fire('pointerup');
    await env.flush();
    const gos = env.requests.filter((r) => r.path === '/api/go').map((r) => r.body);
    const beatSessions = env.requests.filter((r) => r.path === '/api/heartbeat').map((r) => r.body.session);
    env.sandbox.document.hidden = true;
    for (const fn of env.listeners.doc.visibilitychange || []) fn();
    const before = count('/api/heartbeat');
    await env.advance(1000);
    const hiddenBeats = count('/api/heartbeat') - before;
    env.sandbox.document.hidden = false;
    for (const fn of env.listeners.doc.visibilitychange || []) fn();
    await env.advance(300);
    const backBeats = count('/api/heartbeat') - before - hiddenBeats;
    status = statusBody('MOTOR', Object.assign({}, ready, {go_ready: false, go_blockers: ['inputs_not_fresh']}));
    await env.advance(600);
    return Object.assign(ready_, {beatsVisible, gos, beatSessions: Array.from(new Set(beatSessions)), hiddenBeats,
                                  backBeats, holds: count('/api/hold') + count('/api/release'),
                                  blockedText: env.el('hold').textContent, blockedDisabled: env.el('hold').disabled});
  })();
  process.stdout.write(JSON.stringify(out));
})().catch((e) => { process.stderr.write(String(e && e.stack || e)); process.exit(1); });
