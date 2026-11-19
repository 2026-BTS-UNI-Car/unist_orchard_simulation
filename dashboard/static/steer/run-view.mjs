// One picture of what the row executor computes (2026-11-19, user request: "자율주행을 위해 계산하는 과정같은거도 시각적으로
// 좀 보이면 좋을듯. 자율주행 녹화를 보면 해당 시각 자료도 같이 보일 수 있도록"): the occupied cells of its local grid, the
// estimated row lines and centreline, the reference centreline, the travelled path, the lookahead (target) point and the
// vehicle. Everything is in the run's odometry frame (metres from the start pose of the run; no map, no GNSS).
// Live the scene comes from GET /api/executor/status?grid=1 (local_grid_map, single_row_geometry: sceneFromStatus); in a
// recording from the /row_executor/state messages (schema ubts.row_executor.state.v1: sceneFromState). Every field is
// optional and read defensively: what is missing is not drawn. Pure functions apart from drawScene (a 2D context).

const num = (value) => (typeof value === 'number' && isFinite(value) ? value : null);

// [x, y] from [x, y] or {x, y}; null otherwise.
export function point(value) {
  if (Array.isArray(value) && value.length >= 2 && num(value[0]) !== null && num(value[1]) !== null) return [value[0], value[1]];
  if (value && typeof value === 'object' && num(value.x) !== null && num(value.y) !== null) return [value.x, value.y];
  return null;
}

// A polyline (at least two points) from a list of points, {points: [...]}, or a segment {a, b} / {start, end}.
export function polyline(value) {
  let list = null;
  if (Array.isArray(value)) list = value;
  else if (value && typeof value === 'object') {
    if (Array.isArray(value.points)) list = value.points;
    else if (Array.isArray(value.polyline)) list = value.polyline;
    else if (value.a !== undefined || value.start !== undefined) list = [value.a ?? value.start, value.b ?? value.end];
  }
  const out = (list || []).map(point).filter(Boolean);
  return out.length >= 2 ? out : null;
}

// The pose [x, y, yaw rad] from [x, y, yaw] or {x, y, yaw | yaw_rad | yaw_deg}.
export function pose(value) {
  if (Array.isArray(value) && value.length >= 3 && value.slice(0, 3).every((v) => num(v) !== null)) return value.slice(0, 3);
  if (value && typeof value === 'object' && num(value.x) !== null && num(value.y) !== null) {
    const yaw = num(value.yaw_rad) ?? num(value.yaw) ?? (num(value.yaw_deg) === null ? null : value.yaw_deg * Math.PI / 180);
    return [value.x, value.y, yaw ?? 0];
  }
  return null;
}

// The estimator band as a centreline: the corridor axis (heading_rad) through the point centre_m along its left normal,
// drawn from a little behind the vehicle to `ahead` metres in front of it.
export function bandCentre(band, base, behind = 2, ahead = 8) {
  const heading = num(band?.heading_rad), centre = num(band?.centre_m);
  if (heading === null || centre === null) return null;
  const d = [Math.cos(heading), Math.sin(heading)], n = [-Math.sin(heading), Math.cos(heading)];
  const along = base ? base[0] * d[0] + base[1] * d[1] : 0;
  const at = (s) => [centre * n[0] + s * d[0], centre * n[1] + s * d[1]];
  return [at(along - behind), at(along + ahead)];
}

// Live scene: the executor status of the ?grid=1 read (dashboard_server/executor_proxy.py page_status).
export function sceneFromStatus(status) {
  if (!status || typeof status !== 'object') return null;
  const grid = status.local_grid_map && typeof status.local_grid_map === 'object' ? status.local_grid_map : null;
  const geo = status.single_row_geometry && typeof status.single_row_geometry === 'object' ? status.single_row_geometry : null;
  const base = pose(geo?.base) || pose(grid?.base_pose);
  const lines = grid?.lines || {};
  const scene = {
    grid, pose: base, path: polyline(geo?.path) || [],
    rows: { left: polyline(lines.left), right: polyline(lines.right) },
    centre: polyline(lines.centre),
    reference: bandCentre(geo?.band, base),
    lookahead: point(geo?.lookahead) || point(geo?.target),
    axis: num(grid?.axis_rad) ?? num(geo?.band?.heading_rad) ?? (base ? base[2] : 0),
    width: num(geo?.band?.width_m),
  };
  return scene.pose || scene.centre || scene.path.length || grid ? scene : null;
}

// Recorded scene: one /row_executor/state message and the earlier ones (for the travelled path).
export function sceneFromState(msg, earlier = []) {
  if (!msg || typeof msg !== 'object') return null;
  const base = pose(msg.pose);
  const ref = msg.reference || msg.centreline || msg.reference_line || msg.ref;
  // ubts.row_executor.state.v1 (sim 9298b6e): centreline / grid_centreline / lookahead carry {odom, base}; the
  // scene is in the run's odometry frame, so the odom form is used.
  const reference = polyline(ref) || polyline(ref?.odom) || polyline(ref?.line) || polyline(msg.reference_polyline) || null;
  const path = earlier.map((m) => pose(m?.pose)).filter(Boolean).map((p) => [p[0], p[1]]);
  if (base) path.push([base[0], base[1]]);
  const rows = msg.rows || msg.row_lines || {};
  const scene = {
    grid: null, pose: base, path: path.length >= 2 ? path : [],
    rows: { left: polyline(rows.left), right: polyline(rows.right) },
    centre: polyline(msg.centre) || polyline(msg.estimated_centre) || polyline(msg.grid_centreline?.odom) || null,
    reference, lookahead: point(msg.lookahead) || point(msg.lookahead?.odom) || point(msg.target) || point(msg.lookahead_point),
    axis: null, width: num(msg.width_m),
  };
  const line = scene.reference || scene.centre;
  scene.axis = line ? Math.atan2(line[line.length - 1][1] - line[0][1], line[line.length - 1][0] - line[0][0]) : base ? base[2] : 0;
  return base || line || scene.path.length ? scene : null;
}

// The screen frame of a scene on a w x h CSS pixel canvas: the axis points right, its left up, one scale for both.
// With a grid the window of the grid fits; otherwise every point of the scene (at least 6 m x 3 m around them).
export function sceneBox(scene, w, h, margin = 12) {
  const axis = num(scene?.axis) ?? 0;
  const c = Math.cos(axis), s = Math.sin(axis);
  const uv = ([x, y]) => [x * c + y * s, -x * s + y * c];
  let u0, u1, v0, v1;
  const grid = scene?.grid;
  if (grid && Array.isArray(grid.shape) && num(grid.cell_m) > 0 && Array.isArray(grid.origin_uv)) {
    [u0, v0] = grid.origin_uv;
    u1 = u0 + grid.shape[0] * grid.cell_m; v1 = v0 + grid.shape[1] * grid.cell_m;
  } else {
    const pts = [...(scene?.path || []), ...(scene?.reference || []), ...(scene?.centre || []), ...(scene?.rows?.left || []),
                 ...(scene?.rows?.right || [])];
    if (scene?.pose) pts.push(scene.pose);
    if (scene?.lookahead) pts.push(scene.lookahead);
    const all = pts.map(uv);
    if (!all.length) all.push([0, 0]);
    u0 = Math.min(...all.map((p) => p[0])); u1 = Math.max(...all.map((p) => p[0]));
    v0 = Math.min(...all.map((p) => p[1])); v1 = Math.max(...all.map((p) => p[1]));
    const grow = (lo, hi, least) => { const pad = Math.max(0, least - (hi - lo)) / 2 + 0.5; return [lo - pad, hi + pad]; };
    [u0, u1] = grow(u0, u1, 6); [v0, v1] = grow(v0, v1, 3);
  }
  const length = u1 - u0, width = v1 - v0;
  const scale = Math.max(1e-6, Math.min((w - 2 * margin) / length, (h - 2 * margin) / width));
  const ox = (w - length * scale) / 2, oy = (h - width * scale) / 2;
  return {
    axis, scale, ox, oy, u0, v0, u1, v1, length, width,
    screen(x, y) { const [u, v] = uv([x, y]); return [ox + (u - u0) * scale, oy + (v1 - v) * scale]; },
  };
}

// Draw a scene. colours: {cells, frame, rows, centre, reference, path, target, vehicle}; cells: [[i, j], ...] of the grid.
export function drawScene(ctx, scene, w, h, colours, cells = null) {
  ctx.clearRect(0, 0, w, h);
  if (!scene) return null;
  const box = sceneBox(scene, w, h);
  const grid = scene.grid;
  if (grid && num(grid.cell_m) > 0 && Array.isArray(grid.shape)) {
    ctx.strokeStyle = colours.frame;
    ctx.lineWidth = 1;
    ctx.strokeRect(box.ox, box.oy, box.length * box.scale, box.width * box.scale);
    const size = grid.cell_m * box.scale, nj = grid.shape[1];
    ctx.fillStyle = colours.cells;
    for (const [i, j] of cells || []) ctx.fillRect(box.ox + i * size, box.oy + (nj - 1 - j) * size, Math.max(1, size), Math.max(1, size));
  }
  const line = (points, colour, width, dash) => {
    if (!points || points.length < 2) return;
    ctx.save();
    ctx.strokeStyle = colour; ctx.lineWidth = width;
    if (dash) ctx.setLineDash(dash);
    ctx.beginPath();
    points.forEach(([x, y], k) => { const [sx, sy] = box.screen(x, y); if (k) ctx.lineTo(sx, sy); else ctx.moveTo(sx, sy); });
    ctx.stroke();
    ctx.restore();
  };
  line(scene.rows?.left, colours.rows, 2);
  line(scene.rows?.right, colours.rows, 2);
  line(scene.reference, colours.reference, 2, [3, 4]);
  line(scene.centre, colours.centre, 2, [8, 6]);
  line(scene.path, colours.path, 2.5);
  if (scene.lookahead) {
    const [sx, sy] = box.screen(...scene.lookahead);
    ctx.save(); ctx.strokeStyle = colours.target; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(sx, sy, 6, 0, 2 * Math.PI); ctx.stroke();
    if (scene.pose) { const [px, py] = box.screen(scene.pose[0], scene.pose[1]); ctx.setLineDash([2, 3]); ctx.beginPath(); ctx.moveTo(px, py); ctx.lineTo(sx, sy); ctx.stroke(); }
    ctx.restore();
  }
  if (scene.pose) {
    const [px, py] = box.screen(scene.pose[0], scene.pose[1]);
    const heading = scene.pose[2] - box.axis;
    const dx = Math.cos(heading), dy = -Math.sin(heading);
    ctx.save();
    ctx.fillStyle = colours.vehicle;
    ctx.beginPath();
    ctx.moveTo(px + dx * 14, py + dy * 14);
    ctx.lineTo(px - dx * 8 - dy * 7, py - dy * 8 + dx * 7);
    ctx.lineTo(px - dx * 8 + dy * 7, py - dy * 8 - dx * 7);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }
  return box;
}

// The colours of the picture from the page's chart tokens (both themes).
export function sceneColours(read) {
  return {
    frame: read('--chart-grid', '#888'), cells: read('--color-media-text', '#ddd'), rows: read('--chart-1', '#2a78d6'),
    centre: read('--chart-3', '#1baf7a'), reference: read('--chart-5', '#6250d6'), path: read('--chart-2', '#eb6834'),
    target: read('--color-text', '#222'), vehicle: read('--chart-4', '#eda100'),
  };
}
