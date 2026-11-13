// The executor's local grid on the map screen (UI-09, route /map): the occupied cells of the row executor's local
// grid, its last row-line fit (left and right row lines, centre line) and the vehicle, from GET
// /api/executor/status?grid=1 (status.local_grid_map, schema ubts.row_executor.local_grid.v1, simulation repository
// docs/steering/LOCAL_GRID_KO.md). The frame is the run's odometry (not a map, not GNSS). The grid axis (the row
// direction) points right on the screen, the left of the row up. When the executor sends no grid the panel stays
// hidden; without an executor the module stops asking. Loaded by static/app/14-steer.js when /map opens.

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
  if (grid.truncated) parts.push(grid.encoding === 'cells' ? '셀이 많아 가까운 것만' : '격자가 커서 셀을 못 받음');
  return [tag[0], tag[1], parts.join(' · ')];
}

function cssVar(name, fallback) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim() || fallback;
}

export function initLocalGrid(page) {
  const $ = (id) => document.getElementById(id);
  const panel = $('localGridPanel'), canvas = $('localGridCanvas');
  if (!panel || !canvas) return null;
  let timer = null, busy = false, off = false, grid = null, cells = null, drawnKey = '';
  const active = () => page.currentRoute() === '/map' && !document.hidden;

  function show(on) {
    if (panel.hidden === !on) return;
    panel.hidden = !on;
    drawnKey = '';
  }
  async function poll() {
    if (busy || off) return;
    busy = true;
    let reply = null;
    try { reply = await page.getJson('/api/executor/status?grid=1'); } catch (_) { reply = null; }
    finally { busy = false; }
    if (reply && reply.configured === false) { off = true; stop(); show(false); return; }   // no executor here
    const next = reply?.ok ? reply.status?.local_grid_map : undefined;
    if (reply?.ok && (!next || next.schema !== SCHEMA)) { grid = null; cells = null; show(false); return; }
    if (next) {
      if (!grid || next.updates !== grid.updates || next.bits !== grid.bits || next.lines?.stamp !== grid.lines?.stamp) cells = gridCells(next);
      grid = next;
    }
    if (!grid) return;
    show(true);
    const [level, text, line] = reply?.ok ? describe(grid) : ['bad', '실행기 연결 안 됨', describe(grid)[2]];
    const tag = $('localGridState');
    if (tag) { tag.className = `tag ${level}`; tag.textContent = text; }
    if ($('localGridStatus')) $('localGridStatus').textContent = line;
    draw();
  }
  function draw() {
    if (!grid || panel.hidden) return;
    const rect = canvas.getBoundingClientRect();
    if (!(rect.width > 0 && rect.height > 0)) return;
    const dpr = Math.min(window.devicePixelRatio || 1, 1.5);
    const bw = Math.round(rect.width * dpr), bh = Math.round(rect.height * dpr);
    const key = `${bw}x${bh}:${grid.updates}:${grid.lines?.stamp}:${grid.bits?.length}:${grid.base_pose}`;
    if (key === drawnKey) return;
    drawnKey = key;
    if (canvas.width !== bw || canvas.height !== bh) { canvas.width = bw; canvas.height = bh; }
    const ctx = canvas.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    const w = rect.width, h = rect.height;
    ctx.clearRect(0, 0, w, h);
    if (!(num(grid.cell_m) > 0 && Array.isArray(grid.origin_uv))) return;
    const box = layout(grid, w, h);
    const size = grid.cell_m * box.scale;
    ctx.strokeStyle = cssVar('--chart-grid', '#888');
    ctx.lineWidth = 1;
    ctx.strokeRect(box.ox, box.oy, box.length * box.scale, box.width * box.scale);
    ctx.fillStyle = cssVar('--color-media-text', '#ddd');
    const nj = grid.shape[1];
    for (const [i, j] of cells || []) {
      ctx.fillRect(box.ox + i * size, box.oy + (nj - 1 - j) * size, Math.max(1, size), Math.max(1, size));
    }
    const lines = grid.lines || {};
    const segment = (points, colour, dashed) => {
      if (!Array.isArray(points) || points.length !== 2) return;
      const [a, b] = points.map(([x, y]) => screenOf(grid, box, x, y));
      ctx.save();
      ctx.strokeStyle = colour;
      ctx.lineWidth = 2;
      if (dashed) ctx.setLineDash([8, 6]);
      ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
      ctx.restore();
    };
    const blue = cssVar('--chart-1', '#2a78d6');
    segment(lines.left, blue, false);
    segment(lines.right, blue, false);
    segment(lines.centre, cssVar('--chart-3', '#1baf7a'), true);
    const pose = Array.isArray(grid.base_pose) && grid.base_pose.every((v) => num(v) !== null) ? grid.base_pose : null;
    if (pose) {
      const [px, py] = screenOf(grid, box, pose[0], pose[1]);
      const heading = pose[2] - (num(grid.axis_rad) ?? 0);
      const dx = Math.cos(heading), dy = -Math.sin(heading);
      ctx.save();
      ctx.fillStyle = cssVar('--chart-4', '#eda100');
      ctx.beginPath();
      ctx.moveTo(px + dx * 14, py + dy * 14);
      ctx.lineTo(px - dx * 8 - dy * 7, py - dy * 8 + dx * 7);
      ctx.lineTo(px - dx * 8 + dy * 7, py - dy * 8 - dx * 7);
      ctx.closePath();
      ctx.fill();
      ctx.restore();
    }
  }
  function start() { if (!timer && !off) { timer = setInterval(poll, POLL_MS); poll(); } }
  function stop() { clearInterval(timer); timer = null; }
  if (typeof ResizeObserver === 'function') new ResizeObserver(() => { drawnKey = ''; draw(); }).observe(canvas);
  document.addEventListener('visibilitychange', () => routeChanged());

  function routeChanged() { if (active()) start(); else stop(); }
  routeChanged();
  return { routeChanged, draw, state: () => grid };
}
