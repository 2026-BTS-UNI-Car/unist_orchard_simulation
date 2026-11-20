// The moving picture of the executor's local grid for a recorded autonomy run (2026-11-20, user request: the picture of
// the live autonomy tab in the recording player too). A recording does not hold the executor's grid; it holds the
// executor state (/row_executor/state, about four samples a second: pose in the run's odometry frame, centre lines,
// lookahead point) and LiDAR frames (about two a second, every fourth beam). So the occupied cells are built again here:
// each recorded LiDAR frame is placed with the executor pose of its own moment (interpolated between the state samples)
// the way the executor places a scan in its grid, and counted in the executor's 0.1 m cells. The picture itself is
// static/steer/minimap.mjs, the same as live: the car at one place, the row axis up, one fixed scale.
// - The placement is the executor's (simulation repository tools/row_executor: core.py ScanAdapter, perception_layers.py
//   on_scan, local_grid.py insert): bearing from the travel axis = beam angle + LIDAR_YAW_OFFSET_DEG (not mirrored), the
//   LiDAR LIDAR_X_IN_BASE_M ahead of the base centre on that axis, returns up to MAX_RANGE_M, cell = floor(metres / 0.1)
//   on the odometry axes (the executor's grid axis is the heading at the start of the run, which is the odometry x axis).
//   The numbers are the field configuration of 2026-11-20; a recording does not carry them.
// - A cell is drawn when at least MIN_HITS frames hit it. The executor asks for 3 of its own updates (2.5 a second, every
//   beam); with 2 of the recorded frames the number of cells was 82 to 99 % of the executor's own count on the 38 m
//   run of 2026-11-20 (with 1 frame 100 to 244 %).
// - Only frames at or before the playback time are counted, and only those of the same run within REACH_M of the car.
//   The cells are a function of the playback time and of the frames held, so a scrub backwards never keeps later cells.
// - Bounded: at most KEEP_SCANS frames are held (two Int16 a point), at most MAX_SCANS are counted for one picture, and
//   the cells are counted again only when another frame comes into play or the window moves (a 1 m step).
// Pure functions and a class without DOM, apart from recordedPainter (a canvas). static/app/12-archive.js feeds it.

import { pose as poseOf, point, sceneColours, sceneFromState } from './run-view.mjs';
import { JUMP_M, TRAIL_M, drawMinimap, fromCar, rowAxis, spanLine, tail, viewBox, wrap } from './minimap.mjs';

const num = (value) => (typeof value === 'number' && isFinite(value) ? value : null);

export const LIDAR_FRAME = 'lidar_bridge_xy_v1';   // recorded points: angle_deg is the angle of the /scan beam
export const LIDAR_YAW_OFFSET_DEG = 179.08;        // executor lidar_yaw_offset_deg
export const LIDAR_X_IN_BASE_M = 0.1185;           // executor lidar_x_in_base_m
export const SCAN_LATENCY_S = 0.08;                // executor scan_latency_s: a scan is this much older than its receipt
export const MIN_RANGE_M = 0.05;                   // executor range_min_floor_m
export const MAX_RANGE_M = 12;                     // the grid's max_range_m
export const CELL_M = 0.1;                         // local_grid_cell_m
export const WINDOW_LENGTH_M = 20;                 // local_grid_length_m
export const WINDOW_WIDTH_M = 10;                  // local_grid_width_m
export const WINDOW_BEHIND_M = 6;                  // the grid's behind_m
export const WINDOW_STEP_M = 1;                    // the window follows the car in steps (the grid's recentre_m)
export const MIN_HITS = 2;                         // frames that must hit a cell
export const REACH_M = MAX_RANGE_M + WINDOW_BEHIND_M + WINDOW_STEP_M;   // a frame taken farther away cannot reach the window
export const MAX_SCANS = 160;                      // frames counted for one picture (the newest)
export const KEEP_SCANS = 400;                     // frames held (the nearest in time to the playback time)
export const HISTORY_S = 24;                       // after a jump in time the frames of this long before it are asked for
export const POSE_GAP_S = 2.5;                     // samples farther apart are not joined (a window that is not loaded)
export const POSE_HOLD_S = 0.5;                    // a frame this close to a lone sample takes that sample's pose
export const ROW_BEHIND_M = 4;                     // the row lines: as the executor's export (fit_behind_m, fit_ahead_m)
export const ROW_AHEAD_M = 10;
const CELL_PX = 4;                                 // the picture of the cells: pixels a cell

// The index of the last entry at or before `at` in a rising list (-1 when there is none).
function lastAtOrBefore(list, at, time = (item) => item) {
  let low = 0, high = list.length;
  while (low < high) {
    const mid = (low + high) >> 1;
    if (time(list[mid]) <= at) low = mid + 1; else high = mid;
  }
  return low - 1;
}

// A recorded LiDAR frame as points of the car's own axes at the LiDAR (x ahead, y to the left; millimetres, two Int16 a
// point): { t, xy }. points: lists with `fields` (the timeline answer) or objects with angle_deg and distance_m. null
// for a frame of another convention (its angles are not known to be beam angles) or without a time.
export function compactScan(frame, fields = null, frameName = null) {
  const t = num(frame?.t_s);
  if (t === null || !Array.isArray(frame.points) || (frame.coordinate_frame || frameName) !== LIDAR_FRAME) return null;
  const ia = fields ? fields.indexOf('angle_deg') : -1, ir = fields ? fields.indexOf('distance_m') : -1;
  const xy = new Int16Array(frame.points.length * 2);
  let n = 0;
  for (const p of frame.points) {
    const list = Array.isArray(p);
    const angle = num(list ? p[ia] : p?.angle_deg), range = num(list ? p[ir] : p?.distance_m);
    if (angle === null || range === null || range < MIN_RANGE_M || range > MAX_RANGE_M) continue;
    const bearing = (angle + LIDAR_YAW_OFFSET_DEG) * Math.PI / 180;
    xy[n] = Math.round(range * Math.cos(bearing) * 1000);
    xy[n + 1] = Math.round(range * Math.sin(bearing) * 1000);
    n += 2;
  }
  return { t, xy: xy.slice(0, n) };
}

// The pose samples of the recorded executor states (sorted by t_s): times [s of the recording], x, y [m], yaw [rad],
// the run of each sample and its row axis (minimap.mjs rowAxis). States without a pose are left out.
export function poseTrack(frames) {
  const track = { t: [], x: [], y: [], yaw: [], run: [], axis: [] };
  for (const frame of frames || []) {
    const t = num(frame?.t_s), base = poseOf(frame?.state?.pose);
    if (t === null || !base || (track.t.length && t <= track.t[track.t.length - 1])) continue;
    track.t.push(t); track.x.push(base[0]); track.y.push(base[1]); track.yaw.push(base[2]);
    track.run.push(frame.state.run_id ?? null);
    track.axis.push(rowAxis(sceneFromState(frame.state)) ?? base[2]);
  }
  return track;
}

// Two neighbouring samples are one stretch: the same run (a new run starts a new odometry frame), close in time and place.
function joined(track, a, b) {
  return track.run[a] === track.run[b] && track.t[b] - track.t[a] <= POSE_GAP_S
    && Math.hypot(track.x[b] - track.x[a], track.y[b] - track.y[a]) <= JUMP_M;
}

// The pose of a time: { pose: [x, y, yaw], axis, run, index (the sample at or before) }. Between two joined samples it
// goes straight from one to the other, the angles the short way round; otherwise the sample at or before is held for
// at most `hold` seconds (the first sample also for a time just before it). null when no pose is known for that time.
export function trackPose(track, at, hold = POSE_HOLD_S) {
  const n = track?.t?.length || 0;
  if (!n || num(at) === null) return null;
  const sample = (k) => ({ pose: [track.x[k], track.y[k], track.yaw[k]], axis: track.axis[k], run: track.run[k], index: k });
  const k = lastAtOrBefore(track.t, at);
  if (k < 0) return track.t[0] - at <= hold ? sample(0) : null;
  if (k + 1 < n && joined(track, k, k + 1)) {
    const f = (at - track.t[k]) / (track.t[k + 1] - track.t[k]);
    return { pose: [track.x[k] + (track.x[k + 1] - track.x[k]) * f, track.y[k] + (track.y[k + 1] - track.y[k]) * f,
                    wrap(track.yaw[k] + wrap(track.yaw[k + 1] - track.yaw[k]) * f)],
             axis: wrap(track.axis[k] + wrap(track.axis[k + 1] - track.axis[k]) * f), run: track.run[k], index: k };
  }
  return at - track.t[k] <= hold ? sample(k) : null;
}

// Every point of a compact scan as a cell of the odometry axes (i along x, j along y; cell = floor(metres / CELL_M)),
// for the base pose [x, y, yaw] of the scan's moment: fn(i, j) once a point.
export function eachCell(scan, pose, fn) {
  const c = Math.cos(pose[2]), s = Math.sin(pose[2]), cell = CELL_M * 1000;
  const lx = (pose[0] + LIDAR_X_IN_BASE_M * c) * 1000, ly = (pose[1] + LIDAR_X_IN_BASE_M * s) * 1000;
  const xy = scan.xy;
  for (let k = 0; k < xy.length; k += 2) {
    fn(Math.floor((lx + xy[k] * c - xy[k + 1] * s) / cell), Math.floor((ly + xy[k] * s + xy[k + 1] * c) / cell));
  }
}

// The cells [[i, j], ...] a scan hits (each once).
export function scanCells(scan, pose) {
  const seen = new Set(), out = [];
  eachCell(scan, pose, (i, j) => {
    const key = `${i}:${j}`;
    if (!seen.has(key)) { seen.add(key); out.push([i, j]); }
  });
  return out;
}

// The window of cells around a base pose, as the executor's grid: WINDOW_LENGTH_M along x with WINDOW_BEHIND_M (up to
// one step more) behind the car, WINDOW_WIDTH_M across with the car in the middle (half a step). It moves in steps of
// WINDOW_STEP_M. { i0, j0 (the cell of the window's corner), ni, nj }.
export function gridWindow(pose) {
  const ni = Math.round(WINDOW_LENGTH_M / CELL_M), nj = Math.round(WINDOW_WIDTH_M / CELL_M), step = Math.round(WINDOW_STEP_M / CELL_M);
  return { i0: Math.floor((Math.floor(pose[0] / CELL_M) - Math.round(WINDOW_BEHIND_M / CELL_M)) / step) * step,
           j0: Math.round((Math.floor(pose[1] / CELL_M) - nj / 2) / step) * step, ni, nj };
}

// The occupied cells of a playback time: the scans at or before it (the newest MAX_SCANS of the car's run within
// REACH_M of the car), each placed with the pose of its own moment; a cell counts once a scan and is occupied from
// MIN_HITS scans. scans: compactScan results sorted by t. { grid (what minimap.mjs gridTransform reads), cells
// [[i, j], ...] of the window, scans (how many were counted) }; null when the car has no pose at that time.
export function rebuildCells(scans, track, at) {
  if (lastAtOrBefore(track?.t || [], at) < 0) return null;
  const car = trackPose(track, at, Infinity);
  if (!car) return null;
  const win = gridWindow(car.pose);
  const hits = new Uint8Array(win.ni * win.nj), mark = new Int32Array(win.ni * win.nj);
  let used = 0;
  for (let n = lastAtOrBefore(scans, at, (scan) => scan.t); n >= 0 && used < MAX_SCANS; n -= 1) {
    const then = trackPose(track, scans[n].t - SCAN_LATENCY_S);
    if (!then || then.run !== car.run || Math.hypot(then.pose[0] - car.pose[0], then.pose[1] - car.pose[1]) > REACH_M) continue;
    used += 1;
    eachCell(scans[n], then.pose, (gi, gj) => {
      const i = gi - win.i0, j = gj - win.j0;
      if (i < 0 || i >= win.ni || j < 0 || j >= win.nj) return;
      const k = i * win.nj + j;
      if (mark[k] === used) return;
      mark[k] = used;
      if (hits[k] < 255) hits[k] += 1;
    });
  }
  const cells = [];
  for (let k = 0; k < hits.length; k += 1) if (hits[k] >= MIN_HITS) cells.push([Math.floor(k / win.nj), k % win.nj]);
  return { grid: { cell_m: CELL_M, axis_rad: 0, origin_uv: [win.i0 * CELL_M, win.j0 * CELL_M], shape: [win.ni, win.nj] }, cells, scans: used };
}

// The path up to sample `index`: [[x, y], ...] in the order driven, back to where the stretch began or a little more
// than the tail's length.
function pathBefore(track, index) {
  const out = [[track.x[index], track.y[index]]];
  let length = 0;
  for (let k = index; k > 0 && length <= TRAIL_M + 1 && joined(track, k - 1, k); k -= 1) {
    length += Math.hypot(track.x[k] - track.x[k - 1], track.y[k] - track.y[k - 1]);
    out.push([track.x[k - 1], track.y[k - 1]]);
  }
  return out.reverse();
}

// A two point line moved sideways (to the left of its direction for a positive distance).
function shifted(line, distance) {
  const dx = line[1][0] - line[0][0], dy = line[1][1] - line[0][1], length = Math.hypot(dx, dy);
  if (!(length > 0)) return null;
  const nx = -dy / length * distance, ny = dx / length * distance;
  return line.map(([x, y]) => [x + nx, y + ny]);
}

// What the picture of a playback time shows besides the cells: { frame (the state at or before that time: the values
// and the lines), pose and axis (between two samples the car goes from one to the other; with smooth false it stays at
// the sample at or before), run, rows (the grid's centre line moved half its width to each side), centre, lookahead,
// tail }. frames: [{ t_s, state }] sorted by t_s. null while no state with a pose is at or before that time.
export function sceneAt(frames, track, at, smooth = true) {
  const n = lastAtOrBefore(frames || [], at, (frame) => frame.t_s), k = lastAtOrBefore(track?.t || [], at);
  if (n < 0 || k < 0) return null;
  const frame = frames[n], car = trackPose(track, smooth ? at : track.t[k], Infinity);
  const scene = sceneFromState(frame.state) || { rows: {} };
  const width = num(frame.state?.grid_centreline?.width_m);
  const mid = width > 0 ? spanLine(scene.centre, car.pose, ROW_BEHIND_M, ROW_AHEAD_M) : null;
  const rows = scene.rows.left || scene.rows.right ? scene.rows
    : { left: mid ? shifted(mid, width / 2) : null, right: mid ? shifted(mid, -width / 2) : null };
  const aim = point(frame.state?.lookahead?.base);
  return { frame, pose: car.pose, axis: car.axis, run: car.run, rows,
           centre: spanLine(scene.reference, car.pose) || scene.centre || null,
           lookahead: aim ? fromCar(car.pose, aim[0], aim[1]) : scene.lookahead || null,
           tail: tail(pathBefore(track, car.index), car.pose) };
}

// The frames and states of one recording, and the picture of a playback time made from them.
export class RecordedGrid {
  constructor() { this.reset(); }

  reset() {
    this.scans = []; this.times = new Set(); this.windows = new Map(); this.version = 0;
    this.frames = []; this.track = poseTrack([]); this.statesKey = null; this.built = null;
  }

  // The LiDAR frames of one answer. With startS and endS the answer was the whole window [startS, endS) of the
  // recording, which is then remembered as held. fields: the names of a point's values when points are lists; frame:
  // the convention of the answer when a frame does not name its own. Returns how many frames were new.
  ingest(frames, { fields = null, frame = null, startS = null, endS = null } = {}) {
    let added = 0;
    for (const item of frames || []) {
      const scan = compactScan(item, fields, frame);
      if (!scan || this.times.has(scan.t)) continue;
      this.times.add(scan.t); this.scans.push(scan); added += 1;
    }
    if (added) { this.scans.sort((a, b) => a.t - b.t); this.version += 1; }
    if (num(startS) !== null && num(endS) !== null) this.windows.set(startS, endS);
    return added;
  }

  // Whether the window that starts at startS is held.
  covered(startS) { return this.windows.has(startS); }

  // The starts of the windows (windowS long, counted from 0) before the one of `at` whose frames the picture of `at`
  // needs and that are not held, the newest first.
  missing(at, windowS) {
    const out = [];
    if (!(windowS > 0) || num(at) === null) return out;
    for (let start = (Math.floor(at / windowS) - 1) * windowS; start >= 0 && start + windowS > at - HISTORY_S; start -= windowS) {
      if (!this.covered(start)) out.push(start);
    }
    return out;
  }

  // The executor states of the recording (sorted by t_s). `key` names the list: the track is made again when it changes.
  states(frames, key) {
    if (key === this.statesKey) return;
    this.statesKey = key; this.frames = frames || []; this.track = poseTrack(this.frames); this.version += 1;
  }

  // At most KEEP_SCANS frames stay: the ones farthest in time from `at` go, and their windows are no longer held.
  trim(at) {
    let gone = 0;
    while (this.scans.length > KEEP_SCANS) {
      const first = this.scans[0], last = this.scans[this.scans.length - 1];
      const scan = at - first.t >= last.t - at ? this.scans.shift() : this.scans.pop();
      this.times.delete(scan.t);
      for (const [start, end] of this.windows) if (scan.t >= start && scan.t < end) this.windows.delete(start);
      gone += 1;
    }
    if (gone) this.version += 1;
    return gone;
  }

  // The picture of a playback time: sceneAt plus { grid, cells, scans, key }. The cells are counted again only when
  // the frames, the states, the newest frame in play, the window or the run change. null without a pose at that time.
  picture(at, smooth = true) {
    const scene = sceneAt(this.frames, this.track, at, smooth);
    if (!scene) return null;
    this.trim(at);
    const win = gridWindow(scene.pose);
    const key = `${this.version}:${lastAtOrBefore(this.scans, at, (scan) => scan.t)}:${win.i0}:${win.j0}:${scene.run}`;
    if (!this.built || this.built.key !== key) this.built = { key, ...rebuildCells(this.scans, this.track, at) };
    return { ...scene, grid: this.built.grid, cells: this.built.cells, scans: this.built.scans, key };
  }
}

// Draws pictures on a canvas: the cells as one sheet of dots (made again only for other cells or another colour), then
// the frame of minimap.mjs. read(name, fallback): a colour token of the page. paint(picture) returns true when it drew;
// the same picture on the same canvas size is not drawn twice. paint(null) empties the canvas. resized(width, height):
// the size of the canvas box in CSS pixels as an observer reports it; once it is known the box is not measured for
// every picture (measuring it in the middle of the player's frame makes the page lay itself out an extra time).
export function recordedPainter(canvas, read, dprLimit = 1.5) {
  let sheet = null, sheetKey = '', filled = false, drawn = '', colours = null, coloursAt = -Infinity, box = null;
  const clock = () => (globalThis.performance ? performance.now() : Date.now());
  function cellSheet(picture) {
    const key = `${picture.key}:${colours.cells}`;
    if (key === sheetKey) return filled ? sheet : null;
    sheetKey = key; filled = false;
    if (!picture.cells.length) return null;
    const [ni, nj] = picture.grid.shape;
    sheet ||= document.createElement('canvas');
    const pen = typeof sheet.getContext === 'function' ? sheet.getContext('2d') : null;
    if (!pen) return null;
    if (sheet.width !== ni * CELL_PX || sheet.height !== nj * CELL_PX) { sheet.width = ni * CELL_PX; sheet.height = nj * CELL_PX; }
    else pen.clearRect(0, 0, sheet.width, sheet.height);
    pen.fillStyle = colours.cells;
    for (const [i, j] of picture.cells) { pen.beginPath(); pen.arc((i + 0.5) * CELL_PX, (j + 0.5) * CELL_PX, CELL_PX * 0.75, 0, 2 * Math.PI); pen.fill(); }
    filled = true;
    return sheet;
  }
  function paint(picture) {
    const rect = box || canvas.getBoundingClientRect();
    if (!(rect.width > 0 && rect.height > 0)) return false;
    const dpr = Math.min(window.devicePixelRatio || 1, dprLimit);
    const bw = Math.round(rect.width * dpr), bh = Math.round(rect.height * dpr);
    const now = clock();
    if (!colours || now - coloursAt > 1000) {      // the theme can change while a recording is open
      const base = sceneColours(read);
      colours = { ...base, ground: read('--color-media-bg', '#000'), target: base.vehicle };
      coloursAt = now;
    }
    const p = picture?.pose;
    const key = picture ? `${bw}x${bh}:${picture.key}:${picture.frame.t_s}:${p[0].toFixed(3)}:${p[1].toFixed(3)}:${p[2].toFixed(4)}:${picture.axis.toFixed(4)}:${colours.cells}:${colours.ground}`
      : `${bw}x${bh}:none`;
    if (key === drawn) return false;
    drawn = key;
    if (canvas.width !== bw || canvas.height !== bh) { canvas.width = bw; canvas.height = bh; }
    const ctx = canvas.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    if (!picture) { ctx.clearRect(0, 0, rect.width, rect.height); return true; }
    const image = cellSheet(picture);
    drawMinimap(ctx, viewBox(rect.width, rect.height), {
      pose: picture.pose, axis: picture.axis, grid: picture.grid, image, imageK: CELL_PX, cells: image ? null : picture.cells,
      rows: picture.rows, centre: picture.centre, tail: picture.tail, lookahead: picture.lookahead,
    }, colours);
    return true;
  }
  return { paint, forget() { drawn = ''; }, resized(width, height) { box = { width, height }; } };
}
