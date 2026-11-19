// The steering potentiometer rows of Rover Control's drive panel (UI-11 front-steer part, UI-16): the measured front
// wheel angle and the potentiometer reading from GET /api/rover/status (the steering bridge's steering block), with
// the faults the MCU or the bridge reports. Read only: no request changes anything. Loaded by static/app/14-steer.js on
// a front-steer dashboard when Rover Control opens. Every 500 ms while the page is visible it shows the steering
// panel's latest status reply (page.driveStatus) and asks itself only when that reply is older than 1 s.
import { DEVICE_TEXT, faultLabels } from './steer-drive.mjs';

export const STATUS_MS = 500;              // placeholder, same as the drive panel when not armed
export const SHARED_MAX_AGE_MS = 1000;     // a steering panel reply younger than this is shown without a request
const R = globalThis.UbtsSteerRules;
const num = (value) => (typeof value === 'number' && isFinite(value) ? value : null);
const angle = (mrad) => (num(mrad) === null ? '--' : `${mrad >= 0 ? '' : '−'}${Math.abs(R.mradToDeg(mrad)).toFixed(1)}° (${mrad} mrad)`);

// How the steering firmware moves (status steer_control, 2026-11-17): the SMOOTH firmware runs to a target in one go,
// SMOOTH=0 moves in pulses; an absolute-target bridge without the flag is an older firmware; no block is the ramp bridge.
export function firmwareText(reply) {
  const sc = reply && typeof reply.steer_control === 'object' && reply.steer_control ? reply.steer_control : null;
  if (!sc) return '--';
  if (sc.keys !== 'full_limit') return '단계 조향';
  if (sc.smooth === true) return 'SMOOTH · 한 번에 끊김 없이';
  if (sc.smooth === false) return 'PULSE · 끊어서 이동';
  return '끝 목표 · SMOOTH 보고 없음';
}

// [chip level, chip text, rows [[name, value], ...]] of one status reply (pure: tested without a page).
export function potView(reply) {
  const steering = reply && typeof reply.steering === 'object' && reply.steering ? reply.steering : null;
  if (!reply) return ['bad', DEVICE_TEXT.unavailable[1], []];
  if (!steering) return ['warn', DEVICE_TEXT.none[1], []];
  const faults = faultLabels(steering.faults);
  const stale = reply.steering_host?.stale === true;
  const pot = faults.filter((f) => f.code.startsWith('POT_') || f.code === 'NOCAL');
  const level = stale || pot.length ? 'bad' : faults.length ? 'warn' : 'ok';
  const text = stale ? '상태 오래됨' : pot.length ? pot[0].label : faults.length ? '결함 있음' : '정상';
  const permille = num(steering.measured_permille);
  return [level, text, [
    ['측정 각도', angle(num(steering.measured_mrad))],
    ['측정 위치', permille === null ? '--' : `${permille} ‰ · 왼쪽이 +`],
    ['가변저항 원값', num(steering.raw) === null ? '--' : String(steering.raw)],
    ['보정', steering.calibrated === true ? '있음' : steering.calibrated === false ? '없음' : '--'],
    ['조향 펌웨어', firmwareText(reply)],
    ['마지막 상태', num(steering.age_ms) === null ? '--' : `${steering.age_ms} ms 전${stale ? ' · 오래됨' : ''}`],
    ['결함', faults.length ? faults.map((f) => f.label).join(', ') : '없음'],
  ]];
}

export function initSteerSensor(page) {
  const $ = (id) => document.getElementById(id);
  const list = $('steerPotDetails');
  if (!list) return null;
  let timer = null, busy = false, reply;
  // Only while its part is in the shown tab (보정·설정, 2026-11-19).
  const active = () => page.currentRoute() === '/rover-control' && !document.hidden && (page.shown ? page.shown('steerPotPanel') : true);

  async function poll() {
    if (busy) return;
    const shared = page.driveStatus?.();
    if (shared && Date.now() - shared.at <= SHARED_MAX_AGE_MS) { reply = shared.reply; draw(); return; }
    busy = true;
    try { reply = await page.getJson('/api/rover/status'); } catch (_) { reply = null; }
    finally { busy = false; }
    draw();
  }
  function draw() {
    const [level, text, rows] = potView(reply);
    const chip = $('steerPotState');
    if (chip) { chip.className = `status-chip ${level}`; chip.textContent = text; }
    const key = JSON.stringify(rows);
    if (list.dataset.key === key) return;
    list.dataset.key = key;
    list.replaceChildren(...rows.map(([name, value]) => {
      const row = document.createElement('div');
      const dt = document.createElement('dt'); dt.textContent = name;
      const dd = document.createElement('dd'); dd.textContent = value;
      row.append(dt, dd);
      return row;
    }));
  }
  function routeChanged() {
    if (active()) { if (!timer) { timer = setInterval(poll, STATUS_MS); poll(); } }
    else { clearInterval(timer); timer = null; }
  }
  document.addEventListener('visibilitychange', () => routeChanged());
  routeChanged();
  return { routeChanged, draw };
}
