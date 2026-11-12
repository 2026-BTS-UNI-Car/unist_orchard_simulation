"""Operator page for the row executor (AP-mode field loop). Standard library only.

Binding: the configured address only (default the Jetson AP address 10.77.0.1:8097; config.validate()
refuses wildcard binds). cloudflared proxies 127.0.0.1:8080 only, so this page is not reachable through
the public tunnel; at home bind a LAN address in the bench config instead.

Access control
  * a token generated at every process start (printed once, written 0600 to <log-dir>/operator_token.txt):
    GET / needs ?t=<token>; every API call needs the header X-Row-Token: <token> (constant-time compare).
  * Host header must be exactly <bind_host>:<bind_port> (DNS rebinding); every POST needs an Origin header
    equal to http://<bind_host>:<bind_port>; JSON bodies only, at most 4 KiB, strict parsing.
  * responses: no-store, no framing, a per-page CSP nonce for the single inline script.

API (all JSON; the autonomy contract of the front-steer executor is docs/steering/OPERATOR_API_KO.md)
  GET  /api/status                          the loop status (2 Hz polling by the page); also start_mode, run_state,
                                            operator_link, go_ready / go_blockers, stop_reason, speed, tuning
  POST /api/go       {"session": id}        start_mode one_press: start / resume the open run once (409 + the first
                                            blocker when a start condition fails, 409 session_conflict for another screen)
  POST /api/heartbeat {"session": id}       one_press: the screen is visible (page: every 100 ms); the run pauses
                                            (operator_link_lost) operator_timeout_s after the last one, a new go resumes
  POST /api/hold     {"press": id}          start_mode hold_to_run only: heartbeat of a press (page: every 100 ms)
  POST /api/release  {"press": id}          hold_to_run only: pointerup / blur / visibilitychange / pagehide
  POST /api/stop     {}                     MOTOR: op stop to the bridge at once, run ends, LATCHED (HTTP 502 when
                                            the bridge did not confirm). SHADOW: ends the shadow run only.
  POST /api/start    {"row_len_m": L}       new run (row length required in the field profile; optional in mode
                                            single_row: null = no length, the single-row distance / time caps bound it)
  POST /api/end      {}                     end the run normally (no latch)
  POST /api/arm      {}                     MOTOR only: take the bridge lease for the current run
  POST /api/ack      {}                     acknowledge a LATCHED bridge client (does not arm)
  POST /api/params   {base_duty_pct, reduce_duty_pct, max_differential_pct}  bounded on-site values
  POST /api/calib_start {"kind": "arc"|"rolldown"|"stop", "params": {...}}   calibration run (calib_runs.py; MOTOR
                                            with calib_motor_output_enabled only; params bounded by make_plan:
                                            arc: arc_set, step_s; rolldown: step_pct, hold_s, floor_pct; stop (C7):
                                            steady_s; all: distance_m, time_s). Then Arm, then hold, as a row run.

Hold-to-run on the page: pointerdown on the big button starts a press with a fresh random id and sends a
heartbeat at once and every 100 ms; pointerup, pointercancel, lostpointercapture, blur, visibilitychange
(hidden) and pagehide send a release and stop the heartbeats. The button takes pointer capture (touch does so
implicitly): a finger that slides off the button does NOT release; lifting it does. Space works the same on a
laptop (key repeat ignored). A new press waits until the previous release request has completed (keeps the
order of one device's requests). The server considers a press dead 0.5 s (hold_timeout_s) after its last
heartbeat and never revives it; a press from a second device while one is held releases both (HoldState).

Single-row panel (status controller_mode 'single_row', 2026-11-02): Start (row length optional) / End run / STOP (the same
op stop as the big button), phase, estimated lateral / heading error, confidence, L/R duty (a command, not a speed),
differential / limit / saturation / authority state, drift-bias estimate, base duty, target speed, row-end and stop-point
distance, distance, speed, stop reason and its category in words (ROW_END = 정상 열 끝, OBSTACLE, END_STRUCTURE,
PERCEPTION_OR_LOCALISATION, CONTROL_AUTHORITY, OPERATOR_OR_BACKSTOP = never a row end, ...), and a small top-down canvas of
the executor's corridor estimate in the estimator frame (estimated centreline + row lines + observed support ends +
vehicle + recent path; corridor axis up). Display only: the panel changes nothing on the motor path.

Page safety texts (review S9, S10): a full-width banner says what the executor does to the vehicle (SHADOW:
nothing, the STOP button only ends the shadow run; MOTOR: STOP = vehicle op stop). A failed STOP (HTTP 403
token, timeout, bridge not confirming) is shown as a large red alert naming the dashboard STOP / physical
E-stop. A 403 on the status poll (the executor restarted: new token) disables the page and reloads it after
a few seconds into the token instructions page (GET / without a valid token).
"""
import hmac
import html
import json
import secrets
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

MAX_BODY = 4096
JSON_TYPE = 'application/json; charset=utf-8'


class OperatorServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, handler, loop, token):
        super().__init__(address, handler)
        host, port = self.server_address[:2]
        self.loop, self.token = loop, token
        self.host_header = ('[%s]:%d' if ':' in host else '%s:%d') % (host, port)
        self.origin = 'http://' + self.host_header


def make_server(loop, bind_host, bind_port, token):
    """Bound (not yet serving) server; port 0 picks a free port (tests)."""
    if not isinstance(token, str) or len(token) < 16:
        raise ValueError('token too short')
    return OperatorServer((bind_host, bind_port), Handler, loop, token)


def _reject_constant(token):
    raise ValueError('non-finite number')


class Handler(BaseHTTPRequestHandler):
    server_version = 'ubts-row-executor'
    sys_version = ''
    timeout = 3.
    protocol_version = 'HTTP/1.1'

    def log_message(self, fmt, *args):      # the executor log has the requests; keep stderr quiet
        pass

    # --- helpers -------------------------------------------------------------------------------------
    def _send(self, code, body, ctype=JSON_TYPE, extra=None):
        data = body if isinstance(body, bytes) else json.dumps(body, allow_nan=False).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Frame-Options', 'DENY')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Referrer-Policy', 'no-referrer')
        for key, value in (extra or {}).items():
            self.send_header(key, value)
        self.end_headers()
        self.wfile.write(data)

    def _deny(self, code=403, error='forbidden'):
        self._send(code, dict(ok=False, error=error))

    def _host_ok(self):
        return self.headers.get('Host') == self.server.host_header

    def _token_ok(self, value):
        return isinstance(value, str) and hmac.compare_digest(value.encode('utf-8'), self.server.token.encode('utf-8'))

    def _query_token(self):
        if '?' not in self.path:
            return None
        for part in self.path.split('?', 1)[1].split('&'):
            if part.startswith('t='):
                return part[2:]
        return None

    def _route(self):
        return self.path.split('?', 1)[0]

    # --- GET -----------------------------------------------------------------------------------------
    def do_GET(self):
        if not self._host_ok():
            return self._deny(421, 'wrong_host')
        route = self._route()
        if route == '/':
            if not self._token_ok(self._query_token()):
                return self._send(403, TOKEN_PAGE.encode('utf-8'), 'text/html; charset=utf-8',
                                  {'Content-Security-Policy': "default-src 'none'; style-src 'unsafe-inline'"})
            nonce = secrets.token_urlsafe(16)
            page = PAGE.replace('__NONCE__', nonce).replace('__TOKEN__', html.escape(self.server.token, quote=True))
            csp = ("default-src 'none'; script-src 'nonce-%s'; style-src 'unsafe-inline'; connect-src 'self'; "
                   "img-src 'none'; base-uri 'none'; form-action 'none'; frame-ancestors 'none'" % nonce)
            return self._send(200, page.encode('utf-8'), 'text/html; charset=utf-8', {'Content-Security-Policy': csp})
        if route == '/api/status':
            if not self._token_ok(self.headers.get('X-Row-Token')):
                return self._deny()
            return self._send(200, self.server.loop.status())
        return self._deny(404, 'not_found')

    # --- POST ----------------------------------------------------------------------------------------
    def do_POST(self):
        if not self._host_ok():
            return self._deny(421, 'wrong_host')
        if self.headers.get('Origin') != self.server.origin:
            return self._deny(403, 'origin')
        if not self._token_ok(self.headers.get('X-Row-Token')):
            return self._deny()
        length = self.headers.get('Content-Length')
        if length is None or not length.isdigit() or int(length) > MAX_BODY:
            return self._deny(413, 'body_size')
        raw = self.rfile.read(int(length)) if int(length) else b'{}'
        try:
            body = json.loads(raw.decode('utf-8'), parse_constant=_reject_constant)
        except (ValueError, UnicodeDecodeError):
            return self._deny(400, 'invalid_json')
        if not isinstance(body, dict):
            return self._deny(400, 'object_required')
        loop = self.server.loop
        route = self._route()
        if route == '/api/hold':
            ok, why = loop.hold_beat_detail(body.get('press'))
            return self._send(200 if ok else 409, dict(ok=ok) if ok else dict(ok=False, error=why))
        if route == '/api/release':
            if not loop.hold_release(body.get('press') if isinstance(body.get('press'), str) else None):
                return self._send(409, dict(ok=False, error='start_mode_one_press'))
            return self._send(200, dict(ok=True))
        if route == '/api/heartbeat':
            if set(body) - {'session'}:
                return self._deny(400, 'unknown_parameter')
            ok, why = loop.operator_heartbeat(body.get('session'))
            return self._send(200 if ok else 409, dict(ok=ok) if ok else dict(ok=False, error=why))
        if route == '/api/go':
            if set(body) - {'session'}:
                return self._deny(400, 'unknown_parameter')
            return self._result(loop.request('go', session=body.get('session')))
        if route == '/api/stop':
            result = loop.emergency_stop('operator_page')
            return self._send(502 if result.get('stop_sent') is False else 200, result)
        if route == '/api/start':
            return self._result(loop.request('start', row_len_m=body.get('row_len_m')))
        if route == '/api/end':
            return self._result(loop.request('end'))
        if route == '/api/arm':
            return self._result(loop.request('arm'))
        if route == '/api/ack':
            return self._result(loop.request('ack'))
        if route == '/api/calib_start':
            if set(body) - {'kind', 'params'}:
                return self._deny(400, 'unknown_parameter')
            params = body.get('params', {})
            if not isinstance(params, dict):
                return self._deny(400, 'params_object_required')
            return self._result(loop.request('calib_start', calib_kind=body.get('kind'), params=params))
        if route == '/api/turn_observe':
            allowed = {'direction', 'row_width_m', 'x_c_m', 'reach_m',
                       'headland_depth_m', 'creep_max_m', 'operator_aligned'}
            if set(body) - allowed:
                return self._deny(400, 'unknown_parameter')
            return self._result(loop.request('turn_observe', params=body))
        if route == '/api/params':
            params = {k: body[k] for k in ('base_duty_pct', 'reduce_duty_pct', 'max_differential_pct') if k in body}
            if set(body) - set(params):
                return self._deny(400, 'unknown_parameter')
            return self._result(loop.request('params', params=params))
        return self._deny(404, 'not_found')

    def _result(self, result):
        return self._send(200 if result.get('ok') else 409, result)


TOKEN_PAGE = """<!doctype html>
<html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>UBTS row executor: token</title></head>
<body style="margin:0;background:#111;color:#eee;font:17px/1.45 system-ui,sans-serif;padding:16px">
<h1 style="color:#ff8a80;font-size:24px">실행기 화면 토큰이 없거나 틀림 (403)</h1>
<p style="background:#c62828;color:#fff;padding:12px;border-radius:8px;font-weight:700">
이 주소의 버튼은 동작하지 않는다. 차량 정지는 대시보드 STOP 또는 물리 E-stop으로 한다.</p>
<p>실행기는 시작할 때마다 새 토큰을 만든다(재시작 뒤에는 옛 주소가 무효). 새 주소 확인(Jetson):</p>
<ul>
<li><code>run_executor.sh logs</code> 출력의 <code>operator page: http://10.77.0.1:8097/?t=...</code> 줄</li>
<li>또는 <code>sudo cat ~/row-executor/logs/operator_token.txt</code> &rarr; <code>http://10.77.0.1:8097/?t=&lt;토큰&gt;</code></li>
</ul>
<p>새 주소로 이 화면을 다시 연다. 실행기 화면은 한 기기에서만 연다.</p>
</body></html>
"""


PAGE = r"""<!doctype html>
<html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>UBTS row executor</title>
<style>
:root{--bg:#111;--fg:#eee;--dim:#999;--ok:#2e7d32;--warn:#b26a00;--bad:#c62828;--card:#1c1c1c}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:15px/1.35 system-ui,sans-serif;
-webkit-user-select:none;user-select:none;-webkit-touch-callout:none}
header{padding:8px 12px;display:flex;gap:10px;align-items:center;flex-wrap:wrap}
.badge{padding:3px 8px;border-radius:4px;font-weight:700}
#mode.SHADOW{background:#455a64}#mode.MOTOR{background:var(--bad)}
#banner{margin:0 12px 8px;padding:12px;border-radius:8px;font-size:19px;font-weight:800;line-height:1.3;background:#37474f}
#banner.SHADOW{background:#e65100;color:#fff}#banner.MOTOR{background:#4a148c;color:#fff}#banner.armed{background:var(--bad);color:#fff}
#alert{margin:0 12px 8px;padding:14px;border-radius:8px;font-size:20px;font-weight:800;line-height:1.3}
#alert.bad{background:var(--bad);color:#fff;border:4px solid #fff}#alert.warn{background:#b26a00;color:#fff}#alert.ok{background:var(--ok);color:#fff}
main{padding:0 12px 16px;display:grid;gap:10px;grid-template-columns:1fr}
@media(min-width:820px){main{grid-template-columns:1.1fr 1fr}}
.card{background:var(--card);border-radius:8px;padding:10px}
#hold{width:100%;height:42vh;min-height:180px;font-size:30px;font-weight:800;border:0;border-radius:12px;
background:#1e3a5f;color:#fff;touch-action:none}
#hold.on{background:var(--ok)}#hold:disabled{opacity:.4}
#stop{width:100%;min-height:90px;font-size:34px;font-weight:900;border:0;border-radius:12px;background:var(--bad);color:#fff;margin-top:10px}
#stop.shadow{background:#546e7a;font-size:22px}
button.small{font-size:16px;padding:10px 14px;border-radius:8px;border:0;background:#37474f;color:#fff;margin:3px 3px 3px 0}
button.small:disabled{opacity:.35}
input{width:6.5em;font-size:16px;padding:6px;border-radius:6px;border:1px solid #555;background:#000;color:#fff}
table{width:100%;border-collapse:collapse}td{padding:2px 4px;border-bottom:1px solid #2a2a2a}td:first-child{color:var(--dim)}
table.stale{opacity:.35}
.big{font-size:22px;font-weight:700}.bad{color:#ff8a80}.warn{color:#ffcc80}.ok{color:#a5d6a7}
#msg{min-height:1.4em}
</style></head><body>
<header><span id="mode" class="badge">?</span><span id="state" class="big">-</span><span id="reason"></span>
<span id="conn" class="badge"></span></header>
<div id="banner">상태 확인 중… (차량 정지는 대시보드 STOP·물리 E-stop)</div>
<div id="alert" hidden></div>
<main>
<section>
<button id="hold">누르고 있기 (HOLD)</button>
<button id="stop">STOP</button>
<div class="card" style="margin-top:10px">
<div>열 길이 L [m] <input id="rowlen" type="number" inputmode="decimal" step="0.1" min="1" max="300">
<button class="small" id="start">Start</button><button class="small" id="end">End run</button></div>
<div id="rowlenhelp" style="color:var(--dim)">L = 지금 차 위치에서 마지막 줄기선까지 줄자로 잰 값. 열 중간에서 다시 Start할 때도 그 자리에서 다시 잰다(원래 L·화면 주행거리로 빼지 않는다).</div>
<div><button class="small" id="arm">Arm (모터)</button><button class="small" id="ack">Latch 확인</button></div>
<div id="msg"></div>
</div>
</section>
<section>
<div class="card" id="srcard" hidden>
<div><b>단일열 주행 (single_row)</b> · <span id="srloc" style="color:var(--dim)"></span></div>
<div>열 길이 L [m, 비워도 됨] <input id="srlen" type="number" inputmode="decimal" step="0.1" min="1" max="300">
<button class="small" id="srstart">단일열 Start</button><button class="small" id="srend">End run</button>
<button class="small" id="srstopbtn" style="background:var(--bad)">STOP</button></div>
<div id="srhelp" style="color:var(--dim)">L을 비우면 거리 상한(max_row_length)과 그 시간 상한만 백스톱이다. L을 넣으면 L−2 m에서도 끊으므로
실제 열 길이보다 넉넉히(+5 m) 넣는다. 열 끝 판정은 L·시간·거리를 쓰지 않는다(양측 지지 종료 + 전방 여유). 정지 뒤 자동 회전·재출발 없음.</div>
<div class="big"><span id="srphase">-</span> <span id="srstop"></span></div>
<canvas id="srcanvas" width="300" height="420" style="width:100%;max-width:300px;background:#0b0b0b;border-radius:6px"></canvas>
<div id="srlegend" style="color:var(--dim)">추정기 좌표(이번 run 시작점 기준, 지도·GNSS 아님): 녹색 = 추정 중심선, 파랑 = 추정 열 선,
빨강 = 관측된 지지 끝, 주황 = 최근 경로, 흰 사각형 = 차체(위 = 통로 축 방향)</div>
<table id="srtab"></table>
</div>
<div class="card"><table id="tab"></table></div>
<div class="card"><div>현장 설정 (run 없음·disarmed일 때만)</div>
<div>base <input id="p_base" type="number" step="1"> reduce <input id="p_reduce" type="number" step="1">
diff <input id="p_diff" type="number" step="1"> <button class="small" id="params">적용</button></div>
<div id="limits" style="color:var(--dim)"></div><div id="feas"></div></div>
<div class="card" id="calibcard" hidden><div><b>보정 run (C5 arc / C3b rolldown / C7 stop) · MOTOR · 열 밖 넓은 곳</b></div>
<div>종류 <select id="c_kind"><option value="arc">arc (C5: 차등별 yaw)</option>
<option value="rolldown">rolldown (C3b: 굴러가는 중 최저 duty)</option>
<option value="stop">stop (C7: 실행기 0/0 정지 거리, 3회)</option></select></div>
<div>arc 묶음 <select id="c_set"></select> 단계 측정 [s] <input id="c_step" type="number" step="0.5"></div>
<div>rolldown 단계 [%] <select id="c_pct"></select> 단계 유지 [s] <input id="c_hold" type="number" step="0.5"></div>
<div>거리 한도 [m] <input id="c_dist" type="number" step="0.5"> 시간 한도 [s] <input id="c_time" type="number" step="1">
<button class="small" id="calibstart">보정 Start</button></div>
<div id="calibplan" style="color:var(--dim)"></div>
<table id="calibtab"></table><div id="calibout"></div></div>
<div class="card" id="turncard" hidden>
<b>보조 회전 관찰 · INFO_ONLY · 모터 허가 없음</b>
<p>SHADOW에서만 사용한다. 정지 상태에서 시작 후 사람이 대시보드로 수동 회전한다.
관찰 종료는 차량을 정지시키지 않는다. 차량 정지는 대시보드 STOP·물리 E-stop이다.</p>
<div>방향 <select id="t_dir"><option value="left">왼쪽</option><option value="right">오른쪽</option></select>
열 간격 W [m] <input id="t_width" type="number" min="2.8" max="5.5" step="0.1"></div>
<div>수관 기준 x_c [m] <input id="t_xc" type="number" min="1.2" max="3" step="0.1">
최외곽 reach [m] <input id="t_reach" type="number" min="2" max="6.5" step="0.1"></div>
<div>실측 headland 빈 깊이 [m] <input id="t_depth" type="number" min="0" max="50" step="0.1">
<label><input id="t_aligned" type="checkbox">현재 차가 열 방향으로 정렬됐음을 확인</label></div>
<button class="small" id="turnstart">관찰 시작 (차량 제어 없음)</button>
<div id="turnstatus">회전 출력은 이 설치에서 사용할 수 없다.</div></div>
</section>
</main>
<script nonce="__NONCE__">
"use strict";
const TOKEN = "__TOKEN__";
const TOKEN_HELP = "새 주소: Jetson에서 run_executor.sh logs 의 'operator page:' 줄 또는 sudo cat ~/row-executor/logs/operator_token.txt";
const $ = (id) => document.getElementById(id);
let press = null, beatTimer = null, lastStatus = null, statusFail = 0, tokenBad = false, stopAlert = false;
let releasing = Promise.resolve();
// start_mode one_press (design v2 V2-2): one session id per page load, heartbeats every 100 ms while the page is visible;
// the big button sends one go; hiding or leaving the page stops the heartbeats (the executor then pauses the run)
let oneMode = false, oneTimer = null;
const SESSION = (() => { const a = new Uint8Array(12); crypto.getRandomValues(a);
  return Array.from(a, b => b.toString(16).padStart(2, "0")).join(""); })();
function call(path, body, timeoutMs) {
  // -> {status, body}; status 0 = no answer (network error or timeout)
  const ctl = new AbortController();
  const timer = setTimeout(() => ctl.abort(), timeoutMs || 2500);
  return fetch(path, {method: body === undefined ? "GET" : "POST", cache: "no-store", signal: ctl.signal,
    headers: {"X-Row-Token": TOKEN, "Content-Type": "application/json"},
    body: body === undefined ? undefined : JSON.stringify(body)})
    .then(r => r.json().catch(() => ({ok: false})).then(b => ({status: r.status, body: b})))
    .catch(() => ({status: 0, body: {ok: false, error: "no_answer"}}))
    .finally(() => clearTimeout(timer));
}
function api(path, body) { return call(path, body).then(r => { if (r.status === 403) badToken(); return r.body; }); }
function alertBox(text, cls) { const a = $("alert"); a.textContent = text; a.className = cls; a.hidden = !text; }
$("alert").addEventListener("click", () => { if (!tokenBad) { stopAlert = false; alertBox("", ""); } });
function badToken() {
  if (tokenBad) return;
  tokenBad = true; release(); oneStop(); $("hold").disabled = true;
  alertBox("토큰 무효(403): 실행기가 재시작됐다. 이 화면의 버튼(STOP 포함)은 동작하지 않는다. 차량 정지 = 대시보드 STOP / 물리 E-stop. " +
           TOKEN_HELP + ". 5초 뒤 안내 화면으로 다시 불러온다.", "bad");
  setTimeout(() => location.reload(), 5000);
}
function newPress() {
  const a = new Uint8Array(12); crypto.getRandomValues(a);
  return Array.from(a, b => b.toString(16).padStart(2, "0")).join("");
}
function beat() {
  const p = press;
  if (p) api("/api/hold", {press: p}).then(r => {
    if (r.ok || press !== p) return;
    release();
    if (r.error === "press_conflict" && !stopAlert)
      alertBox("다른 기기/탭의 누름과 겹쳐 둘 다 해제됐다(일시정지). 실행기 화면은 한 기기에서만 연다. 다시 누르기.", "warn");
  });
}
function oneBeat() {
  if (document.hidden || tokenBad) return;
  api("/api/heartbeat", {session: SESSION}).then(r => {
    if (!r.ok && r.error === "session_conflict" && !stopAlert)
      alertBox("다른 화면이 이 실행기의 운영자 연결을 쓰는 중이다: 이 화면에서는 자율 시작이 안 된다(STOP은 된다).", "warn");
  });
}
function oneStart() { if (oneTimer === null && !document.hidden && !tokenBad) { oneBeat(); oneTimer = setInterval(oneBeat, 100); } }
function oneStop() { if (oneTimer !== null) { clearInterval(oneTimer); oneTimer = null; } }
function go() { api("/api/go", {session: SESSION}).then(show); }
function hold(ev) {
  if (ev) ev.preventDefault();
  if (oneMode) { if (!$("hold").disabled) go(); return; }
  if (press || $("hold").disabled) return;
  const p = newPress(); press = p; $("hold").classList.add("on");
  releasing.then(() => {            // one device: the previous release reaches the server first
    if (press !== p) return;
    beat(); beatTimer = setInterval(beat, 100);
  });
}
function release() {
  if (oneMode || !press) return;
  const p = press; press = null; clearInterval(beatTimer); beatTimer = null; $("hold").classList.remove("on");
  const body = JSON.stringify({press: p});
  // the next press waits for this release, but never longer than 0.7 s (a hung request must not block presses)
  releasing = Promise.race([
    fetch("/api/release", {method: "POST", keepalive: true, headers: {"X-Row-Token": TOKEN, "Content-Type": "application/json"}, body})
      .catch(() => {}),
    new Promise(r => setTimeout(r, 700))]).then(() => {});
}
const H = $("hold");
H.addEventListener("pointerdown", (e) => { H.setPointerCapture && H.setPointerCapture(e.pointerId); hold(e); });
// pointer capture: sliding off the button does not release; lifting the finger (pointerup) does
for (const ev of ["pointerup", "pointercancel", "pointerleave", "lostpointercapture"]) H.addEventListener(ev, release);
H.addEventListener("contextmenu", (e) => e.preventDefault());
window.addEventListener("blur", release);
document.addEventListener("visibilitychange", () => {
  if (document.hidden) { release(); oneStop(); } else if (oneMode) oneStart();
});
window.addEventListener("pagehide", () => { release(); oneStop(); });
document.addEventListener("keydown", (e) => {
  if (e.code === "Space") { e.preventDefault(); if (!e.repeat) hold(); }
  if (e.code === "Escape") stop();
});
document.addEventListener("keyup", (e) => { if (e.code === "Space") { e.preventDefault(); release(); } });
function stop() {
  release();
  const motor = !lastStatus || lastStatus.mode !== "SHADOW";
  stopAlert = true;
  alertBox("STOP 보내는 중…", "warn");
  call("/api/stop", {}, 2500).then(({status, body}) => {
    if (status === 200 && body.ok && body.shadow) {
      alertBox("SHADOW run 종료. 차량은 이 버튼으로 멈추지 않는다(대시보드 STOP / 물리 E-stop).", "warn");
    } else if (status === 200 && body.ok && body.stop_sent) {
      alertBox("STOP 전송: 브리지가 확인했다. 실행기는 LATCHED. 차량이 멈췄는지 눈으로 확인.", "ok");
    } else if (status === 403) {
      badToken();
    } else {
      const why = status === 0 ? "응답 없음/시간 초과" : ("HTTP " + status + " " + (body.error || ""));
      alertBox("STOP 실패 (" + why + ")" + (motor ? ": 지금 대시보드 STOP 또는 물리 E-stop!" : "") + " (눌러서 닫기)", "bad");
    }
  });
}
$("stop").addEventListener("click", stop);
function msg(t, cls) { const m = $("msg"); m.textContent = t; m.className = cls || ""; }
function show(r) { msg((r.ok ? "OK " : "거부 ") + JSON.stringify(r), r.ok ? "ok" : "bad"); }
$("start").addEventListener("click", () => {
  const v = parseFloat($("rowlen").value);
  const optional = lastStatus && (lastStatus.profile === "bench" || lastStatus.controller_mode === "single_row");
  if (!optional && !(v > 0)) { msg("열 길이를 입력해야 Start할 수 있다", "bad"); return; }
  api("/api/start", {row_len_m: v > 0 ? v : null}).then(show);
});
// single-row panel (controller_mode single_row): the row length is optional, STOP is the same op stop as the big button
$("srstart").addEventListener("click", () => {
  const v = parseFloat($("srlen").value);
  api("/api/start", {row_len_m: v > 0 ? v : null}).then(show);
});
$("srend").addEventListener("click", () => api("/api/end", {}).then(show));
$("srstopbtn").addEventListener("click", stop);
$("end").addEventListener("click", () => api("/api/end", {}).then(show));
$("arm").addEventListener("click", () => api("/api/arm", {}).then(show));
$("ack").addEventListener("click", () => api("/api/ack", {}).then(show));
function calibRequest() {
  const kind = $("c_kind").value || "arc";
  const num = (id) => { const v = parseFloat($(id).value); return isFinite(v) ? v : undefined; };
  const p = {};
  if (kind === "arc") {
    if ($("c_set").value) p.arc_set = $("c_set").value;
    if (num("c_step") !== undefined) p.step_s = num("c_step");
  } else if (kind === "rolldown") {
    const st = parseInt($("c_pct").value, 10);
    if (isFinite(st)) p.step_pct = st;
    if (num("c_hold") !== undefined) p.hold_s = num("c_hold");
  }
  if (num("c_dist") !== undefined) p.distance_m = num("c_dist");
  if (num("c_time") !== undefined) p.time_s = num("c_time");
  return {kind: kind, params: p};
}
$("calibstart").addEventListener("click", () => api("/api/calib_start", calibRequest()).then(show));
$("turnstart").addEventListener("click", () => {
  const p = {direction: $("t_dir").value || "left", operator_aligned: !!$("t_aligned").checked};
  for (const [id, key] of [["t_width", "row_width_m"], ["t_xc", "x_c_m"],
                           ["t_reach", "reach_m"], ["t_depth", "headland_depth_m"]]) {
    const v = parseFloat($(id).value);
    if (!isFinite(v)) { msg("회전 관찰 치수는 표·실측값을 모두 입력해야 한다", "bad"); return; }
    p[key] = v;
  }
  api("/api/turn_observe", p).then(show);
});
$("params").addEventListener("click", () => {
  const body = {base_duty_pct: parseInt($("p_base").value, 10), reduce_duty_pct: parseInt($("p_reduce").value, 10),
                max_differential_pct: parseInt($("p_diff").value, 10)};
  api("/api/params", body).then(show);
});
function fmt(v, d) { return (v === null || v === undefined) ? "-" : (typeof v === "number" ? v.toFixed(d) : String(v)); }
function row(k, v, cls) { return "<tr><td>" + k + "</td><td class='" + (cls || "") + "'>" + v + "</td></tr>"; }
function esc(s) { return String(s).replace(/[&<>"']/g, c => ({"&":"&amp;","<":"&lt;",">":"&gt;","\"":"&quot;","'":"&#39;"}[c])); }
let paramsFilled = false, calibFilled = false;
function calibCard(s) {
  const o = s.calib_options;
  $("calibcard").hidden = !o || !!o.error;
  if (!o || o.error) return;
  if (!calibFilled) {
    $("c_set").innerHTML = (o.arc_sets || []).map(a => "<option value='" + esc(a.id) + "'>" + esc(a.id) + (a.ok ? "" : " (불가)") +
      "</option>").join("");
    $("c_set").value = o.arc_default || "";
    $("c_pct").innerHTML = (o.rolldown || []).map(r => "<option value='" + r.step_pct + "'>" + r.step_pct + (r.ok ? "" : " (불가)") +
      "</option>").join("");
    $("c_pct").value = "3";
    $("c_step").value = 4; $("c_hold").value = 3;
    $("c_dist").value = o.distance_m ? o.distance_m[1] : ""; $("c_time").value = o.time_s ? o.time_s[1] : "";
    calibFilled = true;
  }
  const kind = $("c_kind").value || "arc";
  const src = {breakaway_duty_pct: "측정 breakaway", rolling_breakaway_duty_pct: "측정 rolling",
               unknown_breakaway_default: "breakaway 모름: 40"};
  let plan = "출발 최소 " + o.start_floor_pct + " (" + (src[o.start_floor_source] || esc(o.start_floor_source)) + "), base " + o.base_pct + ". ";
  if (kind === "arc") {
    const a = (o.arc_sets || []).find(x => x.id === $("c_set").value);
    plan += "base/base로 출발, 굴러가면 쌍(안쪽 최소 " + o.floor_pct + ", " + (src[o.floor_source] || esc(o.floor_source)) + "). ";
    if (a) plan += "좌/우: " + a.steps.map(p => p[0] + "/" + p[1]).join(", ") + (a.ok ? "" : " — 불가: " + a.why);
  } else if (kind === "rolldown") {
    const r = (o.rolldown || []).find(x => String(x.step_pct) === String($("c_pct").value));
    if (r) plan += "duty(양쪽 같게, B 아래까지): " + (r.ok ? r.duties.join(" → ") : "불가: " + r.why);
  } else {
    const st = o.stop || {};
    plan += "직진 " + st.duty + "/" + st.duty + " → 굴러가면 1.5 s 안정 + 3 s 측정 → 실행기가 스스로 0/0, 정지 거리를 잰다(C7). 3회." +
      (st.ok === false ? " — 불가: " + st.why : "");
  }
  $("calibplan").textContent = plan;
  const c = s.calib;
  if (!c) { $("calibtab").innerHTML = ""; $("calibout").textContent = ""; return; }
  let t;
  if (c.kind === "arc") {
    t = "<tr><td>#</td><td>좌/우</td><td>s</td><td>gyro °/s</td><td>LiDAR °/s</td><td>m/s</td><td>gain</td><td></td></tr>";
    for (const r of c.results || []) t += "<tr><td>" + (r.i + 1) + "</td><td>" + r.L + "/" + r.R + "</td><td>" + fmt(r.seconds, 1) +
      "</td><td>" + fmt(r.gyro_yaw_dps, 2) + "</td><td>" + fmt(r.lidar_yaw_dps, 2) + "</td><td>" + fmt(r.speed_m_s, 2) +
      "</td><td>" + fmt(r.gain_dps_per_pct, 3) + "</td><td class='" + (r.valid ? "ok" : "bad") + "'>" + (r.valid ? "유효" : "무효") + "</td></tr>";
  } else {
    t = "<tr><td>#</td><td>duty</td><td>s</td><td>m/s</td><td>최저 m/s</td><td></td></tr>";
    for (const r of c.results || []) {
      const good = c.kind === "stop" ? r.steady : r.sustained;
      t += "<tr><td>" + (r.i + 1) + "</td><td>" + r.duty + "</td><td>" + fmt(r.seconds, 1) + "</td><td>" +
        fmt(r.speed_m_s, 2) + "</td><td>" + fmt(r.min_speed_m_s, 2) + "</td><td class='" + (r.stalled ? "bad" : good ? "ok" : "warn") +
        "'>" + (r.stalled ? "정지" : good ? "유지" : "미완") + "</td></tr>";
    }
  }
  $("calibtab").innerHTML = t;
  const out = c.outcome;
  let line = "보정 " + esc(c.kind) + " · 단계 " + (Math.min(c.step_index + 1, c.steps_total)) + "/" + c.steps_total + " · " + esc(c.phase) +
    " · 거리 " + fmt(c.counted_m, 1) + "/" + fmt(c.distance_cap_m, 0) + " m" + (c.end_reason ? " · 끝: " + esc(c.end_reason) : "");
  if (out && c.kind === "arc") {
    const sd = out.sides || {};
    line += " | gain 좌 " + fmt(sd.left && sd.left.gain, 3) + " / 우 " + fmt(sd.right && sd.right.gain, 3) +
      " · |L−R| " + fmt(out.full_diff_pct, 0) + "에서 yaw ≈ " + fmt(out.yaw_at_full_diff_dps, 2) + " °/s" + (out.extrapolated ? " (외삽)" : "");
  } else if (out && c.kind === "rolldown") {
    line += " | rolling 후보 " + fmt(out.rolling_breakaway_duty_pct, 0) + " · " + esc(out.note || "");
  } else if (out) {
    line += " | 정속 " + fmt(out.speed_m_s, 2) + " m/s";
  }
  const sp = c.stop;
  if (sp) line += " | 0/0 뒤 정지 " + (sp.done ? (sp.ok ? fmt(sp.stop_distance_m, 2) + " m (0/0 때 " + fmt(sp.v_at_zero_m_s, 2) +
    " m/s, " + fmt(sp.time_to_rest_s, 1) + " s)" : "측정 실패: " + esc(sp.why || "")) : "관찰 중…");
  $("calibout").textContent = line;
}
const HINT = {hold_released: "놓음: 새로 누르면 재개", corridor_lost: "통로 상실: 차가 멈춘 뒤 새로 눌러야 재개",
  reacquire_needs_press: "통로 재획득: 차가 멈춘 뒤 새로 눌러야 재개",
  press_ignored_not_standstill: "정지 확인 전 누름은 무시: 멈춘 뒤 다시 누르기",
  press_ignored_inputs_not_fresh: "입력이 1 s 신선해진 뒤 다시 누르기", stale_lidar: "LiDAR 끊김: 신선해진 뒤 새로 누르기",
  stale_imu: "IMU 끊김: 신선해진 뒤 새로 누르기", stall_guard: "루프 지연: 새로 누르기", await_hold: "누르고 있기로 출발"};
function banner(s) {
  const b = $("banner");
  $("hold").disabled = false;
  if (s.mode === "SHADOW") {
    b.className = "SHADOW";
    b.textContent = "SHADOW" + (s.shadow_auto ? " 자동" : "") + ": 실행기는 차량을 제어하지 않는다(제안만 계산·기록). " +
                    "차량 정지 = 대시보드 STOP / 물리 E-stop. 아래 STOP은 shadow run만 끝낸다.";
    $("stop").textContent = "shadow run 종료 (차량 정지 아님)"; $("stop").className = "shadow";
    $("hold").textContent = "누르고 있기 (SHADOW: 제안만, 차량 안 움직임)";
    if (s.run_kind === "turn_observe") {
      b.textContent = "INFO_ONLY · 보조 회전 관찰: 차량 제어·Arm·자동 인계 없음. 차량 정지는 대시보드 STOP / 물리 E-stop.";
      $("hold").textContent = "관찰 전용 · 차량은 대시보드로 수동 조종";
      $("hold").disabled = true;
    }
  } else {
    const armed = s.vehicle_control === "armed";
    b.className = armed ? "armed" : "MOTOR";
    b.textContent = armed ? (oneMode ? "MOTOR · ARMED: 한 번 누르면 열 끝까지 간다. 화면을 가리거나 닫으면 선다. STOP = 차량 정지(op stop)."
                                     : "MOTOR · ARMED: 누르고 있는 동안 실행기가 차량을 몬다. STOP = 차량 정지(op stop).")
                          : "MOTOR · Arm 전: 실행기는 아직 차량을 몰지 않는다. STOP = 차량 정지(op stop, lease 무관).";
    if (s.row_motor === false && s.calib_allowed) b.textContent += " 이 설정은 보정 run만 출력한다(열 추종 출력 없음).";
    $("stop").textContent = "STOP (차량 정지)"; $("stop").className = "";
    $("hold").textContent = "누르고 있기 (HOLD)";
  }
  if (oneMode) {
    $("hold").textContent = s.run_state === "running" ? "주행 중 (STOP으로 정지)" :
      "자율 주행 시작 (한 번 누름)" + (s.go_ready ? "" : " · 불가: " + (s.go_blockers || []).join(", "));
    $("hold").disabled = !s.go_ready;
  }
}
function render(s) {
  lastStatus = s;
  oneMode = s.start_mode === "one_press";
  if (oneMode) oneStart(); else oneStop();
  banner(s);
  $("mode").textContent = s.mode + (s.shadow_auto ? " auto" : "") + (s.profile === "bench" ? " BENCH: 바퀴 띄움" : "");
  $("mode").className = "badge " + s.mode;
  $("state").textContent = s.state + (s.ctl && s.ctl[0] ? " / " + s.ctl[0] : "");
  $("reason").textContent = (s.end_reason ? "end: " + s.end_reason + (s.end_note ? " (" + s.end_note + ")" : "") + " · " : "") +
    (s.reason || "") + (s.state === "PAUSED" && HINT[s.reason] ? " — " + HINT[s.reason] : "");
  const bs = s.backstop || {};
  const b = s.bridge_detail;
  const lg = s.log || {};
  const ages = [s.lidar_age, s.imu_age];
  const stale = (s.lidar_age === null || s.lidar_age > 0.45) || (s.imu_age === null || s.imu_age > 0.4);
  let t = "";
  t += row("좌/우 duty (제안)", fmt(s.L, 0) + " / " + fmt(s.R, 0), "big");
  t += row("전송", s.sent ? s.sent.join(" / ") : "-");
  t += row("횡오차 [m] (+ 왼쪽)", fmt(s.lat, 3));
  t += row("heading 오차 [°]", fmt(s.head, 1));
  t += row("통로 폭 [m]", fmt(s.width, 2) + " " + (s.corridor ? "" : "(통로 없음: " + esc(s.corridor_reason) + ")"), s.corridor ? "" : "warn");
  t += row("열 끝까지 [m]", fmt(s.end, 2));
  t += row("전방 / 좌 / 우 여유 [m]", fmt(s.front, 2) + " / " + fmt(s.sideL, 2) + " / " + fmt(s.sideR, 2));
  t += row("입력 나이 LiDAR / IMU [s]", fmt(ages[0], 3) + " / " + fmt(ages[1], 3), stale ? "bad" : "ok");
  t += row("주행거리 / 한도 [m]", fmt(bs.travelled_m, 1) + " / " + fmt(bs.limit_m, 1) +
    (bs.expected_rest_m !== null && bs.expected_rest_m !== undefined ? " (예상 정지 " + fmt(bs.expected_rest_m, 1) + ")" : ""));
  t += row("이동시간 / 한도 [s]", fmt(bs.motion_time_s, 1) + " / " + fmt(bs.time_limit_s, 1));
  t += row("속도 [m/s] · yaw [°/s]", fmt(s.v, 2) + " · " + fmt(s.yaw, 1));
  t += row("scan 처리 [ms] · 버림", fmt(s.scan && s.scan.proc_ms, 1) + " · " + fmt(s.counts && s.counts.scans_dropped, 0));
  if (s.calib && !s.calib.end_reason) t += row("보정 단계", esc(s.calib.kind) + " " + (s.calib.step_index + 1) + "/" + s.calib.steps_total +
    " · " + (s.calib.step ? s.calib.step.join(" / ") : "-") + " · " + esc(s.calib.phase), "big");
  if (oneMode) {
    const ol = s.operator_link || {};
    t += row("운영자 연결 · run 상태", (ol.fresh ? "살아 있음" : "끊김") + " (" + fmt(ol.age_s, 2) + " / " + fmt(ol.timeout_s, 1) +
      " s) · " + esc(s.run_state || "-") + (s.stop_reason ? " · " + esc(s.stop_reason) : ""), ol.fresh ? "ok" : "bad");
    const sp = s.speed || {};
    t += row("목표 / 상한 속도 [m/s]", fmt(sp.target_mps, 2) + " / " + fmt(sp.limit_mps, 2) + " (" + esc(sp.limit_reason || "-") + ")");
  } else {
    t += row("hold (서버)", s.hold ? "누름" : "놓음", s.hold ? "ok" : "");
  }
  t += row("bridge", b ? esc(b.state) + (b.latch_reason ? " (" + esc(b.latch_reason) + ")" : "") : (s.mode === "MOTOR" ? "disarmed (Arm 전)" : "연결 없음 (SHADOW)"), b && b.state === "LATCHED" ? "bad" : "");
  t += row("stale/pause 수", fmt(s.stale_events, 0) + " / " + fmt(s.pauses, 0));
  const logText = {ok: "정상", failing: "실패", stalled: "지연(디스크 응답 없음)", capped: "용량 한도(tick 중단)"}[lg.state] || esc(lg.state);
  t += row("기록 (JSONL)", logText + " · 버린 줄 " + fmt(lg.dropped, 0) + (lg.last_error && lg.state !== "ok" ? " · " + esc(lg.last_error) : ""),
           lg.state === "ok" && !lg.dropped ? "ok" : "bad");
  if (s.loop_exceptions) t += row("루프 예외", fmt(s.loop_exceptions, 0) + " · " + esc(s.loop_error || ""), "bad");
  $("tab").innerHTML = t;
  $("tab").className = "";
  $("arm").disabled = s.mode !== "MOTOR";
  const f = s.feasibility || {};
  $("feas").innerHTML = "<span class='" + (f.status === "ok" ? "ok" : f.status === "unknown" ? "warn" : "bad") + "'>feasibility: " +
    esc(f.status) + "</span> " + esc((f.problems || []).concat(f.unknown || []).join("; "));
  const lim = s.onsite_limits || {};
  const yp = s.yaw_params || {};
  $("limits").textContent = "범위: base " + (lim.base_duty_pct || []).join("–") + ", reduce " + (lim.reduce_duty_pct || []).join("–") +
    ", diff " + (lim.max_differential_pct || []).join("–") + " (cap·안쪽 바퀴 규칙으로 추가 제한). diff를 낮추면 yaw 권한(측정 × 새 diff / 파일 diff)과 " +
    "max_yaw가 함께 낮아진다: 지금 yaw 권한 " + fmt(yp.measured_yaw_at_max_diff_deg_s, 2) + " °/s, max_yaw " + fmt(yp.max_yaw_rate_deg_s, 1) + " °/s.";
  calibCard(s);
  $("turncard").hidden = !s.turn_observe_available;
  const o = s.turn_observe, om = o && o.measurements || {};
  $("turnstart").disabled = !s.turn_observe_available || s.state === "ACTIVE" || s.state === "PAUSED";
  $("turnstatus").textContent = o ? "INFO_ONLY · " + o.phase + " · " + o.reason +
    " · gyro " + fmt(om.psi_gyro_deg, 1) + "° / LiDAR " + fmt(om.psi_lidar_deg, 1) +
    "° · 회전 관찰 " + ((o.observe_log && o.observe_log.rotations) || []).length + "회 · 모터 허가 없음"
    : "관찰 시작 전 · 회전 출력은 이 설치에서 사용할 수 없다.";
  if (!paramsFilled && s.params) {
    $("p_base").value = s.params.base_duty_pct; $("p_reduce").value = s.params.reduce_duty_pct;
    $("p_diff").value = s.params.max_differential_pct; paramsFilled = true;
  }
  if (!stopAlert && !tokenBad) {
    if (lg.state && lg.state !== "ok" && lg.state !== "capped") alertBox("기록 " + logText + ": 버린 줄 " + fmt(lg.dropped, 0) + " (" + esc(lg.last_error || "") + "). 디스크 확인.", "bad");
    else alertBox("", "");
  }
  srRender(s);
}
// --- single-row panel ------------------------------------------------------------------------------------------
const SR_PHASE = {START: "출발 준비 (정지 확인 중)", FOLLOW: "중심선 추종", APPROACH_END: "열 끝 감속", STOPPING: "정지 중",
  WAIT: "대기 (통로 재획득)", STOPPED: "정지 (종료: 자동 회전·재출발 없음)", PAUSED: "일시정지", IDLE: "run 없음"};
const SR_CAT = {ROW_END: "정상 열 끝", END_STRUCTURE: "열 끝 구조물 (벽·울타리)", OBSTACLE: "열 안 장애물·벽",
  PERCEPTION_OR_LOCALISATION: "센서·위치 추정 신뢰도 부족", CONTROL_AUTHORITY: "조향·구동 권한 부족 (소프트웨어로 해결 안 됨)",
  OPERATOR_OR_BACKSTOP: "운용자·백스톱 정지 (열 끝 아님)", SYSTEM_FAULT: "시스템 오류", UNCLASSIFIED: "분류 안 됨"};
const SR_CONF = {OK: "정상", LOW: "낮음", NONE: "없음"};
function srCat(c) { return c ? (SR_CAT[c] || String(c)) : "-"; }
function srRender(s) {
  const card = $("srcard");
  const on = s.controller_mode === "single_row";
  card.hidden = !on;
  if (!on) return;
  const sr = s.single_row || {};
  const st = s.single_row_settings || {};
  $("srloc").textContent = "위치 추정: " + (st.localisation || "SENSOR_ONLY_SINGLE_ROW") + " (LiDAR + raw IMU, GPS·카메라·PWM 아님)" +
    (s.native_navigation && s.native_navigation.provider_present ? " · native 교차 확인: " + (s.native_navigation.native_cross_check ?
      s.native_navigation.native_cross_check.reason : "-") : "");
  $("srphase").textContent = SR_PHASE[sr.phase] || (sr.phase ? String(sr.phase) : "-");
  const stopText = sr.stop_reason ? "정지 사유: " + sr.stop_reason + " → " + srCat(sr.stop_category) : "";
  $("srstop").textContent = stopText;
  $("srstop").className = sr.stop_category === "ROW_END" ? "ok" : (sr.stop_category ? "bad" : "");
  const c = sr.confidence || {};
  let t = "";
  t += row("횡오차 [m] (+ 왼쪽) · heading 오차 [°]", fmt(sr.lateral_error_m, 3) + " · " + fmt(sr.heading_error_deg, 1));
  t += row("신뢰도", (SR_CONF[c.level] || "-") + " · 통로 " + (c.corridor_valid ? "유효" : "없음") + " · 나이 " +
    fmt(c.corridor_age_s, 2) + " s · 정합 " + (c.odom_valid ? "유효" : "무효") + " · 입력 " + (c.inputs_fresh ? "신선" : "오래됨"),
    c.level === "OK" ? "ok" : (c.level === "LOW" ? "warn" : "bad"));
  t += row("좌/우 PWM [%] (명령, 속도 아님)", fmt(sr.left_pct, 0) + " / " + fmt(sr.right_pct, 0), "big");
  t += row("차등 / 한계 [%] · 포화 · 권한", fmt(sr.differential_pct, 1) + " / " + fmt(sr.differential_limit_pct, 0) + " · " +
    (sr.saturated ? "포화" : "-") + " (" + fmt(sr.saturation_fraction === null || sr.saturation_fraction === undefined ? null :
      100 * sr.saturation_fraction, 0) + " %) · " + esc(sr.authority_state || "-"), sr.authority_state === "SLOW" || sr.saturated ? "warn" : "");
  t += row("편향 추정 [%] · base [%] · 목표 속도 [m/s]", fmt(sr.bias_pct, 1) + " · " + fmt(sr.base_duty_pct, 1) + " · " + fmt(sr.target_speed_m_s, 2));
  t += row("열 끝까지 / 정지점까지 [m]", fmt(sr.row_end_ahead_m, 2) + " / " + fmt(sr.end_remaining_m, 2));
  t += row("주행거리 [m] · 속도 [m/s]", fmt(sr.distance_m, 1) + " · " + fmt(sr.speed_m_s, 2));
  t += row("정지 분류", esc(srCat(sr.stop_category)) + (sr.stop_label && sr.stop_label !== sr.stop_category ? " (" + esc(sr.stop_label) + ")" : ""),
    sr.stop_category === "ROW_END" ? "ok" : (sr.stop_category ? "bad" : ""));
  if (sr.first_stop) t += row("첫 정지", esc(sr.first_stop.reason) + " · " + esc(srCat(sr.first_stop.category)) + " · t " + fmt(sr.first_stop.t, 1));
  $("srtab").innerHTML = t;
  srDraw(s.single_row_geometry);
}
function srDraw(g) {
  const cv = $("srcanvas");
  const ctx = cv && cv.getContext ? cv.getContext("2d") : null;
  if (!ctx) return;
  const W = cv.width || 300, H = cv.height || 420, k = 20;        // 20 px per metre; vehicle at 75 % height
  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = "#0b0b0b"; ctx.fillRect(0, 0, W, H);
  if (!g || !g.base) { ctx.fillStyle = "#999"; ctx.font = "14px sans-serif"; ctx.fillText("추정 없음 (scan 대기)", 10, 24); return; }
  const bx = g.base[0], by = g.base[1], yaw = g.base[2];
  const band = g.band;
  const axis = band ? band.heading_rad : yaw;
  const rot = Math.PI / 2 - axis, cr = Math.cos(rot), sr_ = Math.sin(rot);
  const cx = W / 2, cy = H * 0.75;
  const P = (x, y) => { const dx = x - bx, dy = y - by; return [cx + (cr * dx - sr_ * dy) * k, cy - (sr_ * dx + cr * dy) * k]; };
  const line = (pts, colour, width, dash) => {
    if (pts.length < 2) return;
    ctx.beginPath(); ctx.strokeStyle = colour; ctx.lineWidth = width; ctx.setLineDash(dash || []);
    let p = P(pts[0][0], pts[0][1]); ctx.moveTo(p[0], p[1]);
    for (let i = 1; i < pts.length; i++) { p = P(pts[i][0], pts[i][1]); ctx.lineTo(p[0], p[1]); }
    ctx.stroke(); ctx.setLineDash([]);
  };
  if (band) {
    const ux = Math.cos(band.heading_rad), uy = Math.sin(band.heading_rad), nx = -uy, ny = ux;
    const a0 = ux * bx + uy * by;
    const at = (a, off) => [nx * off + ux * a, ny * off + uy * a];
    const span = (off) => [at(a0 - 6, off), at(a0 + 16, off)];
    line(span(band.centre_m), "#43a047", 2, [6, 4]);
    line(span(band.centre_m + band.width_m / 2), "#64b5f6", 2);
    line(span(band.centre_m - band.width_m / 2), "#64b5f6", 2);
    for (const [end, side] of [[band.end_left_m, 1], [band.end_right_m, -1]]) {
      if (end === null || end === undefined) continue;
      const off = band.centre_m + side * band.width_m / 2;
      line([at(end, off - 0.5), at(end, off + 0.5)], "#e53935", 4);
    }
  }
  line(g.path || [], "#ff9800", 2);
  const fp = g.footprint_m || [1.22, 0.7], L = fp[0] / 2, Wd = fp[1] / 2, c = Math.cos(yaw), s_ = Math.sin(yaw);
  const corners = [[L, Wd], [L, -Wd], [-L, -Wd], [-L, Wd], [L, Wd]].map(([a, b]) => [bx + c * a - s_ * b, by + s_ * a + c * b]);
  line(corners, "#ffffff", 2);
  line([corners[0], corners[1]], "#ffeb3b", 3);                  // front edge
  ctx.fillStyle = "#bbb"; ctx.font = "12px sans-serif"; ctx.fillText("1 m", 12, H - 14);
  ctx.beginPath(); ctx.strokeStyle = "#bbb"; ctx.lineWidth = 2; ctx.moveTo(40, H - 18); ctx.lineTo(40 + k, H - 18); ctx.stroke();
}
function poll() {
  if (tokenBad) return;
  call("/api/status", undefined, 2000).then(({status, body}) => {
    if (status === 200) { statusFail = 0; $("conn").textContent = ""; $("conn").className = "badge"; render(body); return; }
    if (status === 403) { badToken(); return; }
    statusFail += 1; $("conn").textContent = "연결 끊김"; $("conn").className = "badge bad";
    if (statusFail >= 2) {
      $("tab").className = "stale";
      if (!stopAlert) alertBox("실행기 화면 연결 끊김: 표시 값은 오래된 것이다. 차량 정지 = 대시보드 STOP / 물리 E-stop.", "bad");
    }
  });
}
setInterval(poll, 500); poll();
</script></body></html>
"""
