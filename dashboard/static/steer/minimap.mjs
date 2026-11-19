// The moving picture of the monitoring view (2026-11-20, user request: the local grid picture jumped once a second and
// the car wandered over it; it should read like the minimap of a racing game). The car stays at one place of the
// canvas (the middle, 5/17 of the height above the lower edge) and the ground moves under it: the occupied cells of the
// executor's local grid, the fitted row lines, the centre line, the travelled path and the lookahead point.
// - Up is the row axis: the fitted centre line of the grid, else the estimator band, else the heading of the car. The
//   axis is eased (AXIS_TAU_S); the car's arrow turns only by its heading error to that axis, and a lateral error shows
//   as the centre line passing left or right of the car.
// - One fixed scale: AHEAD_M in front, BEHIND_M behind, at least HALF_M to each side. Never fitted to the data.
// - The pose arrives in samples (the autonomy panel's status read twice a second, else the grid read once a second).
//   A new sample starts a glide from the pose drawn now to the sample, as long as the samples were apart (a little
//   longer, so a sample that is a little late does not make a halt): the picture runs about one sample behind and never
//   ahead of what was measured. Without a new sample the car waits at the last one.
// Everything is in the run's odometry frame (metres from the start pose of the run; no map, no GNSS). Pure functions
// apart from drawMinimap (a 2D context); static/steer/local-grid.mjs owns the reads and the frames.

const num = (value) => (typeof value === 'number' && isFinite(value) ? value : null);
const TAU = 2 * Math.PI;

export const AHEAD_M = 12;            // shown in front of the car
export const BEHIND_M = 5;            // shown behind it
export const HALF_M = 5;              // shown to each side at least (both tree rows of a row)
export const SCALE_BAR_M = 2;         // the scale bar and the ground lines
export const TRAIL_M = 15;            // the travelled path behind the car
export const AXIS_TAU_S = 0.5;        // easing of the row axis
export const FRAME_MS = 1000 / 30;    // at most 30 pictures a second
export const GLIDE_STRETCH = 1.2;     // a glide lasts this many sample intervals
export const GLIDE_MIN_MS = 100;
export const GLIDE_MAX_MS = 1800;
export const JUMP_M = 2;              // a sample this far from the last one is a new run: no glide

// An angle in [-pi, pi).
export function wrap(angle) {
  const a = (angle + Math.PI) % TAU;
  return (a < 0 ? a + TAU : a) - Math.PI;
}

// The pose [x, y, yaw] of a glide at a time: straight from `from` to `to`, the yaw the short way round, held at `to`.
export function poseAt(glide, now) {
  if (!glide) return null;
  const k = glide.dur > 0 ? Math.max(0, Math.min(1, (now - glide.t0) / glide.dur)) : 1;
  return [glide.from[0] + (glide.to[0] - glide.from[0]) * k, glide.from[1] + (glide.to[1] - glide.from[1]) * k,
          wrap(glide.from[2] + wrap(glide.to[2] - glide.from[2]) * k)];
}

// A new pose sample that arrived at the time `at` (the clock of the frames). The glide starts at that time from the
// pose the old glide had then, so the drawn track does not depend on when the frames fall. With smooth false (reduced
// motion), for the first sample and for a jump the picture goes to the sample at once.
export function follow(glide, sample, at, smooth = true) {
  const jump = glide && Math.hypot(sample[0] - glide.to[0], sample[1] - glide.to[1]) > JUMP_M;
  if (!glide || !smooth || jump) return { from: sample, to: sample, t0: at, dur: 0, at };
  const dur = Math.max(GLIDE_MIN_MS, Math.min(GLIDE_MAX_MS, (at - glide.at) * GLIDE_STRETCH));
  return { from: poseAt(glide, at), to: sample, t0: at, dur, at };
}

// The row axis [rad, odometry frame] of a scene (run-view.mjs sceneFromStatus): the grid's fitted centre line, pointing
// the way the car goes; else the estimator band (scene.reference); else the heading of the car. null without any.
export function rowAxis(scene) {
  const yaw = scene?.pose ? scene.pose[2] : null;
  for (const line of [scene?.centre, scene?.reference]) {
    if (!line) continue;
    const a = line[0], b = line[line.length - 1];
    if (Math.hypot(b[0] - a[0], b[1] - a[1]) < 0.5) continue;
    const dir = Math.atan2(b[1] - a[1], b[0] - a[0]);
    return yaw !== null && Math.abs(wrap(dir - yaw)) > Math.PI / 2 ? wrap(dir + Math.PI) : dir;
  }
  return yaw;
}

// One easing step of the drawn axis towards the scene's axis (first order, the short way round).
export function easeAxis(axis, target, dtMs, tauS = AXIS_TAU_S) {
  if (num(axis) === null || !(dtMs >= 0)) return target;
  return wrap(axis + wrap(target - axis) * (1 - Math.exp(-dtMs / (tauS * 1000))));
}

// The fixed frame of the picture on a w x h canvas (CSS pixels): pixels per metre and the place of the car.
export function viewBox(w, h) {
  const scale = Math.max(1e-6, Math.min(w / (2 * HALF_M), h / (AHEAD_M + BEHIND_M)));
  return { w, h, scale, cx: w / 2, cy: h * AHEAD_M / (AHEAD_M + BEHIND_M) };
}

// Screen point of an odometry point: the car (pose) at (cx, cy), the axis up, the left of the row to the left.
export function toScreen(box, pose, axis, x, y) {
  const c = Math.cos(axis), s = Math.sin(axis), dx = x - pose[0], dy = y - pose[1];
  return [box.cx - (dy * c - dx * s) * box.scale, box.cy - (dx * c + dy * s) * box.scale];
}

// The canvas transform [a, b, c, d, e, f] that puts a picture of the grid's cells on the screen: cell (i, j) covers the
// picture's pixels [i k, (i + 1) k) x [j k, (j + 1) k). null when the export does not say where its window is.
export function gridTransform(box, pose, axis, grid, k = 1) {
  const cell = num(grid?.cell_m), origin = Array.isArray(grid?.origin_uv) ? grid.origin_uv : [];
  if (!(cell > 0) || num(origin[0]) === null || num(origin[1]) === null) return null;
  const g = num(grid.axis_rad) ?? 0, gc = Math.cos(g), gs = Math.sin(g), step = cell / k;
  const at = (u, v) => toScreen(box, pose, axis, u * gc - v * gs, u * gs + v * gc);
  const o = at(origin[0], origin[1]), i = at(origin[0] + step, origin[1]), j = at(origin[0], origin[1] + step);
  return [i[0] - o[0], i[1] - o[1], j[0] - o[0], j[1] - o[1], o[0], o[1]];
}

// The tail behind the car: [[x, y, metres back along the tail], ...] from the car backwards over at most `length`. The
// path can be a sample newer than the car that is drawn, so its points in front of the car are left out.
export function tail(path, pose, length = TRAIL_M) {
  const out = [[pose[0], pose[1], 0]];
  const c = Math.cos(pose[2]), s = Math.sin(pose[2]);
  let k = (path || []).length - 1;
  while (k >= 0 && Math.hypot(path[k][0] - pose[0], path[k][1] - pose[1]) < JUMP_M
         && (path[k][0] - pose[0]) * c + (path[k][1] - pose[1]) * s > 0) k -= 1;
  for (; k >= 0; k -= 1) {
    const last = out[out.length - 1];
    const step = Math.hypot(path[k][0] - last[0], path[k][1] - last[1]);
    if (!(step > 0)) continue;
    if (last[2] + step >= length) {
      const f = (length - last[2]) / step;
      out.push([last[0] + (path[k][0] - last[0]) * f, last[1] + (path[k][1] - last[1]) * f, length]);
      break;
    }
    out.push([path[k][0], path[k][1], last[2] + step]);
  }
  return out;
}

// A straight line of the scene (two points) drawn over the whole picture: from behind the lower edge to beyond the
// upper one, measured from the foot of the car on the line.
export function spanLine(line, pose, behind = BEHIND_M + 2, ahead = AHEAD_M + 2) {
  if (!line || line.length < 2) return null;
  const a = line[0], b = line[line.length - 1];
  const len = Math.hypot(b[0] - a[0], b[1] - a[1]);
  if (!(len > 0)) return null;
  let d = [(b[0] - a[0]) / len, (b[1] - a[1]) / len];
  if (d[0] * Math.cos(pose[2]) + d[1] * Math.sin(pose[2]) < 0) d = [-d[0], -d[1]];
  const s = (pose[0] - a[0]) * d[0] + (pose[1] - a[1]) * d[1];
  const foot = [a[0] + d[0] * s, a[1] + d[1] * s];
  return [[foot[0] - d[0] * behind, foot[1] - d[1] * behind], [foot[0] + d[0] * ahead, foot[1] + d[1] * ahead]];
}

// The controller's lookahead point in the car's own frame [ahead m, to the left m], from what the status carries: the
// angle to the point (single_row.lookahead_error_deg) and the lookahead distance of the controller's own rule
// (simulation repository sim/ubts/closed_loop/steer_control.py: base + per_speed * max(min speed, speed), between base
// and max; the values are the field tuning's). null while the run does not drive or a value is missing.
export function lookahead(status) {
  const row = status?.single_row, basic = status?.tuning?.values?.basic;
  const error = num(row?.lookahead_error_deg), base = num(basic?.lookahead_base_m);
  const per = num(basic?.lookahead_per_speed_s), most = num(basic?.lookahead_max_m);
  if (status?.run_state !== 'running' || error === null || base === null || per === null || most === null || Math.abs(error) >= 80) return null;
  const speed = Math.max(num(status.tuning.values.advanced?.min_speed_m_s) ?? 0, num(row.speed_m_s) ?? 0);
  const ahead = Math.max(base, Math.min(most, base + per * speed));
  return [ahead, ahead * Math.tan(error * Math.PI / 180)];
}

// A point of the car's own frame [ahead, left] in the odometry frame.
export const fromCar = (pose, ahead, left) => [pose[0] + Math.cos(pose[2]) * ahead - Math.sin(pose[2]) * left,
                                               pose[1] + Math.sin(pose[2]) * ahead + Math.cos(pose[2]) * left];

// Draw one frame. frame: { pose, axis, grid, image (a picture of the cells, `imageK` pixels a cell) or cells
// [[i, j], ...], rows: { left, right }, centre (two points), tail (tail()), lookahead ([x, y] odometry) }.
// colours: { ground, cells, rows, centre, path, target, vehicle }. The context is in CSS pixels.
export function drawMinimap(ctx, box, frame, colours) {
  const { w, h, scale, cx, cy } = box;
  const at = (x, y) => toScreen(box, frame.pose, frame.axis, x, y);
  ctx.clearRect(0, 0, w, h);

  // ground lines across the row (HALF_M to each side), one every SCALE_BAR_M of the odometry: they show the motion
  // where nothing else is
  const along = frame.pose[0] * Math.cos(frame.axis) + frame.pose[1] * Math.sin(frame.axis);
  ctx.save();
  ctx.globalAlpha = 0.1; ctx.strokeStyle = colours.cells; ctx.lineWidth = 1;
  ctx.beginPath();
  for (let m = Math.ceil((along - (h - cy) / scale) / SCALE_BAR_M) * SCALE_BAR_M; m <= along + cy / scale; m += SCALE_BAR_M) {
    const y = cy - (m - along) * scale;
    ctx.moveTo(cx - HALF_M * scale, y); ctx.lineTo(cx + HALF_M * scale, y);
  }
  ctx.stroke();
  ctx.restore();

  const k = frame.image ? frame.imageK : 1;
  const transform = frame.image || frame.cells ? gridTransform(box, frame.pose, frame.axis, frame.grid, k) : null;
  if (transform) {
    ctx.save();
    ctx.transform(...transform);
    if (frame.image) ctx.drawImage(frame.image, 0, 0);
    else { ctx.fillStyle = colours.cells; for (const [i, j] of frame.cells) ctx.fillRect(i, j, 1, 1); }
    ctx.restore();
  }

  const line = (points, colour, width, dash, alpha = 1) => {
    if (!points || points.length < 2) return;
    ctx.save();
    ctx.globalAlpha = alpha; ctx.strokeStyle = colour; ctx.lineWidth = width; ctx.lineJoin = 'round';
    if (dash) ctx.setLineDash(dash);
    ctx.beginPath();
    points.forEach(([x, y], n) => { const [sx, sy] = at(x, y); if (n) ctx.lineTo(sx, sy); else ctx.moveTo(sx, sy); });
    ctx.stroke();
    ctx.restore();
  };
  line(frame.rows?.left, colours.rows, 2, null, 0.8);
  line(frame.rows?.right, colours.rows, 2, null, 0.8);
  line(frame.centre, colours.centre, 2, [8, 7]);

  // the tail fades towards its end (five steps)
  const points = frame.tail || [];
  for (let band = 0, n = 0; band < 5 && n < points.length - 1; band += 1) {
    const part = [points[n]];
    while (n < points.length - 1 && points[n][2] < (band + 1) * TRAIL_M / 5) { n += 1; part.push(points[n]); }
    line(part, colours.path, 3, null, 1 - band * 0.2);
  }

  if (frame.lookahead) {
    const [sx, sy] = at(frame.lookahead[0], frame.lookahead[1]);
    ctx.save();
    ctx.fillStyle = colours.target;
    ctx.beginPath(); ctx.arc(sx, sy, 3.5, 0, TAU); ctx.fill();
    ctx.restore();
  }

  // the car: an arrow at its fixed place (about the size of the car at this scale), turned by the heading error to
  // the row axis (to the left = anticlockwise)
  const error = wrap(frame.pose[2] - frame.axis);
  const fx = -Math.sin(error), fy = -Math.cos(error);
  const corner = (ahead, left) => [cx + fx * ahead + fy * left, cy + fy * ahead - fx * left];
  ctx.save();
  ctx.fillStyle = colours.vehicle; ctx.strokeStyle = colours.ground; ctx.lineWidth = 1.5; ctx.lineJoin = 'round';
  ctx.beginPath();
  [[16, 0], [-10, 9.5], [-4.5, 0], [-10, -9.5]].forEach(([a, l], n) => { const [sx, sy] = corner(a, l); if (n) ctx.lineTo(sx, sy); else ctx.moveTo(sx, sy); });
  ctx.closePath();
  ctx.fill(); ctx.stroke();
  ctx.restore();

  // the scale bar in the lower left corner, on a patch of the ground colour so that a dot under it does not read as a tick
  const x0 = 12, y0 = h - 12, x1 = x0 + SCALE_BAR_M * scale;
  ctx.save();
  ctx.globalAlpha = 0.7; ctx.fillStyle = colours.ground;
  ctx.beginPath(); ctx.rect(x0 - 6, y0 - 21, x1 - x0 + 12, 27); ctx.fill();
  ctx.globalAlpha = 0.85; ctx.strokeStyle = colours.cells; ctx.fillStyle = colours.cells; ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(x0, y0 - 5); ctx.lineTo(x0, y0); ctx.lineTo(x1, y0); ctx.lineTo(x1, y0 - 5);
  ctx.stroke();
  ctx.font = '500 11px "UBTS Sans", system-ui, -apple-system, "Segoe UI", "Malgun Gothic", sans-serif';
  ctx.textAlign = 'center'; ctx.textBaseline = 'bottom';
  ctx.fillText(`${SCALE_BAR_M} m`, (x0 + x1) / 2, y0 - 4);
  ctx.restore();
  return box;
}
