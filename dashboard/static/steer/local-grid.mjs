// The executor's local grid in Rover Control's 자율주행 tab (UI-09, 2026-11-19 monitoring view): the occupied cells of the
// row executor's local grid, its last row-line fit (left and right row lines, centre line), the estimator band's
// centreline, the travelled path, the lookahead point and the vehicle, from GET /api/executor/status?grid=1
// (status.local_grid_map, schema ubts.row_executor.local_grid.v1, simulation repository docs/steering/LOCAL_GRID_KO.md;
// status.single_row_geometry). The frame is the run's odometry (not a map, not GNSS). The grid axis (the row direction)
// points right on the screen, the left of the row up (static/steer/run-view.mjs draws it). When the executor sends
// nothing to draw the picture stays hidden; without an executor the module stops asking. It asks at most once a second
// and only while its block (#autoGridBlock, in the 자율주행 tab) is on screen. Loaded by static/app/14-steer.js when that
// tab is opened the first time.

import { drawScene, sceneColours, sceneFromStatus } from './run-view.mjs';

export const POLL_MS = 1000;              // placeholder: the executor makes a new export at most every 0.5 s
export const STALE_S = 2.0;               // an export older than this is shown as old
export const SCHEMA = 'ubts.row_executor.local_grid.v1';

const num = (value) => (typeof value === 'number' && isFinite(value) ? value : null);

// base64 text -> bytes (atob in the page, Buffer in Node).
function bytesOf(text) {
  if (typeof atob === 'function') {
    const raw = atob(text);
    const out = new Uint8Array(raw.length);
    for (let k = 0; k < raw.length; k += 1) out[k] = raw.charCodeAt(k);
    return out;
  }
  return new Uint8Array(Buffer.from(text, 'base64'));
}

// The occupied cells [[i, j], ...] of an export: the bitmap (np.packbits order, big end first, rows = i along the
// axis, columns = j across) or the cell list. null when the export carries no cells (truncated bitmap, bad shape).
export function gridCells(grid) {
  const [ni, nj] = Array.isArray(grid?.shape) ? grid.shape : [];
  if (!(ni > 0 && nj > 0)) return null;
  if (grid.encoding === 'cells') {
    return Array.isArray(grid.cells) ? grid.cells.filter((c) => Array.isArray(c) && c[0] >= 0 && c[0] < ni && c[1] >= 0 && c[1] < nj) : null;
  }
  if (typeof grid.bits !== 'string' || !grid.bits) return null;
  let bytes;
  try { bytes = bytesOf(grid.bits); } catch (_) { return null; }
  if (bytes.length < Math.ceil((ni * nj) / 8)) return null;
  const out = [];
  for (let k = 0; k < ni * nj; k += 1) {
    if (bytes[k >> 3] & (0x80 >> (k & 7))) out.push([Math.floor(k / nj), k % nj]);
  }
  return out;
}

// Odometry (x, y) -> grid frame (u along the axis, v to the left of it).
export function toGrid(axis, x, y) {
  const c = Math.cos(axis), s = Math.sin(axis);
  return [x * c + y * s, -x * s + y * c];
}

// Where the window goes on a canvas of w x h CSS pixels: { scale (px/m), ox, oy } with screen x = ox + (u - u0) * scale,
// screen y = oy + (vTop - v) * scale. The whole window fits with a margin, same scale on both axes.
export function layout(grid, w, h, margin = 12) {
  const [ni, nj] = grid.shape;
  const cell = grid.cell_m;
  const [u0, v0] = grid.origin_uv;
  const length = ni * cell, width = nj * cell;
  const scale = Math.max(1e-6, Math.min((w - 2 * margin) / length, (h - 2 * margin) / width));
  const ox = (w - length * scale) / 2, oy = (h - width * scale) / 2;
  return { scale, ox, oy, u0, vTop: v0 + width, length, width };
}

// Screen point of an odometry point.
export function screenOf(grid, box, x, y) {
  const [u, v] = toGrid(num(grid.axis_rad) ?? 0, x, y);
  return [box.ox + (u - box.u0) * box.scale, box.oy + (box.vTop - v) * box.scale];
}

// The words of the panel: [tag level, tag text, status line].
export function describe(grid) {
  const age = num(grid.age_s);
  const old = age !== null && age > STALE_S;
  const tag = [old ? 'warn' : 'ok', old ? `오래됨 ${age.toFixed(1)} s` : `갱신 ${grid.updates ?? '--'}`];
  const shape = Array.isArray(grid.shape) ? `${grid.shape[0]}×${grid.shape[1]}` : '--';
  const quality = num(grid.lines?.quality);
  const parts = [`셀 ${num(grid.cell_m) ?? '--'} m · ${shape}`, `점유 ${grid.occupied ?? '--'}`,
                 grid.lines?.centre ? `선 품질 ${quality === null ? '--' : quality.toFixed(2)}` : '열 선 아직 없음'];
  if (age !== null) parts.push(`${age.toFixed(1)} s 전`);
  if (grid.truncated) parts.push(grid.encoding === 'cells' ? '셀이 많아 가까운 것만 표시' : '격자가 커서 셀을 받지 못함');
  return [tag[0], tag[1], parts.join(' · ')];
}

function cssVar(name, fallback) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim() || fallback;
}

// The path of the run when the executor sends no single_row_geometry (an older executor): the base poses of the grid
// reads, one per poll, a new list when the run or the grid export starts again.
export function trailStep(trail, status) {
  const base = Array.isArray(status?.local_grid_map?.base_pose) ? status.local_grid_map.base_pose : null;
  const run = status?.run ?? null, updates = num(status?.local_grid_map?.updates);
  const fresh = !trail || trail.run !== run || (updates !== null && trail.updates !== null && updates < trail.updates);
  const next = fresh ? { run, updates, points: [] } : { ...trail, updates, points: trail.points.slice(-299) };
  if (base && base.slice(0, 2).every((v) => num(v) !== null)) {
    const last = next.points[next.points.length - 1];
    if (!last || Math.hypot(base[0] - last[0], base[1] - last[1]) >= 0.05) next.points.push([base[0], base[1]]);
  }
  return next;
}

const RUN_OPEN = new Set(['ready', 'armed', 'running', 'paused']);

export function initLocalGrid(page) {
  const $ = (id) => document.getElementById(id);
  const panel = $('localGridPanel'), canvas = $('localGridCanvas');
  if (!panel || !canvas) return null;
  let timer = null, busy = false, off = false, grid = null, cells = null, scene = null, drawnKey = '', trail = null;
  // The picture sits in the monitoring view of the 자율주행 tab (2026-11-19); it is hidden until the executor sends
  // something to draw, so its always-visible block is watched (the live map part on an older page).
  const watched = $('autoGridBlock') || $('liveMapPanel');
  let onScreen = null;
  if (watched && typeof IntersectionObserver === 'function') {
    new IntersectionObserver((entries) => { onScreen = Boolean(entries[entries.length - 1].isIntersecting); routeChanged(); }).observe(watched);
  }
  const active = () => page.currentRoute() === '/rover-control' && !document.hidden
    && (onScreen === true || (onScreen === null && !(watched && typeof IntersectionObserver === 'function')));

  function show(on) {
    if (panel.hidden === !on) return;
    panel.hidden = !on;
    drawnKey = '';
  }
  function words(level, text, line) {
    const tag = $('localGridState');
    if (tag) { tag.className = `tag ${level}`.trim(); tag.textContent = text; }
    if ($('localGridStatus')) $('localGridStatus').textContent = line;
  }
  async function poll() {
    if (busy || off) return;
    busy = true;
    let reply = null;
    try { reply = await page.getJson('/api/executor/status?grid=1'); } catch (_) { reply = null; }
    finally { busy = false; }
    if (reply && reply.configured === false) { off = true; stop(); show(false); words('', '실행기 없음', '자율주행 실행기가 설정되어 있지 않습니다.'); return; }
    if (!reply?.ok) {
      if (scene) words('bad', '실행기 연결 안 됨', grid ? describe(grid)[2] : '마지막 그림을 유지합니다.');
      return;
    }
    const status = reply.status || {};
    // After a run (ended / idle / latched) the executor keeps integrating scans into its grid; the picture stays
    // the last one of the run until the next run opens.
    if (scene && !RUN_OPEN.has(status.run_state)) {
      words('', '멈춤', '마지막 주행의 그림입니다. 다음 주행에서 다시 그립니다.');
      return;
    }
    const next = status.local_grid_map && status.local_grid_map.schema === SCHEMA ? status.local_grid_map : null;
    if (next && (!grid || next.updates !== grid.updates || next.bits !== grid.bits || next.lines?.stamp !== grid.lines?.stamp)) cells = gridCells(next);
    grid = next;
    if (!next) cells = null;
    trail = trailStep(trail, status);
    scene = sceneFromStatus(next ? status : { ...status, local_grid_map: undefined });
    if (scene && !scene.path.length && trail.points.length >= 2) scene.path = trail.points;
    if (!scene) { show(false); words('', '대기', '주행 중에 그립니다.'); return; }
    show(true);
    if (grid) words(...describe(grid));
    else words('', '격자 없음', '실행기가 격자를 보내지 않아 경로와 차만 그립니다.');
    draw();
  }
  function draw() {
    if (!scene || panel.hidden) return;
    const rect = canvas.getBoundingClientRect();
    if (!(rect.width > 0 && rect.height > 0)) return;
    const dpr = Math.min(window.devicePixelRatio || 1, 1.5);
    const bw = Math.round(rect.width * dpr), bh = Math.round(rect.height * dpr);
    const key = `${bw}x${bh}:${grid?.updates}:${grid?.lines?.stamp}:${grid?.bits?.length}:${scene.pose}:${scene.path.length}:${scene.lookahead}`;
    if (key === drawnKey) return;
    drawnKey = key;
    if (canvas.width !== bw || canvas.height !== bh) { canvas.width = bw; canvas.height = bh; }
    const ctx = canvas.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    drawScene(ctx, scene, rect.width, rect.height, sceneColours(cssVar), cells);
  }
  function start() { if (!timer && !off) { timer = setInterval(poll, POLL_MS); poll(); } }
  function stop() { clearInterval(timer); timer = null; }
  if (typeof ResizeObserver === 'function') new ResizeObserver(() => { drawnKey = ''; draw(); }).observe(canvas);
  document.addEventListener('visibilitychange', () => routeChanged());

  function routeChanged() { if (active()) start(); else stop(); }
  routeChanged();
  return { routeChanged, draw, state: () => grid, scene: () => scene };
}
