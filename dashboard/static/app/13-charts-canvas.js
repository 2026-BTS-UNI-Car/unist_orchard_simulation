

function requestChartAnimation({ redrawLidar = true } = {}) {
  // Preserve explicit theme/resize/range requests even when a history RAF is
  // already queued. Telemetry history alone does not invalidate the scan.
  state.chartLidarRedrawRequested ||= redrawLidar;
  if (state.animationFrame) return;
  state.animationFrame = requestAnimationFrame(animateCharts);
}

function animateCharts(timestamp) {
  state.animationFrame = null;
  const historyActive = stepVisualHistory(timestamp);
  const previousRange = state.lidarMap.displayRange;
  const lidarActive = stepLidarMapRange();
  const redrawLidar = state.chartLidarRedrawRequested || previousRange !== state.lidarMap.displayRange;
  state.chartLidarRedrawRequested = false;
  drawCharts({ redrawLidar });
  if (historyActive || lidarActive) state.animationFrame = requestAnimationFrame(animateCharts);
}

function stepVisualHistory(timestamp) {
  if (state.chartShift <= 0) return false;
  const elapsed = Math.max(0, timestamp - state.chartShiftStartedAt);
  state.chartShift = Math.max(0, 1 - elapsed / Math.max(300, state.chartShiftDurationMs || POLL_INTERVAL_MS));
  return state.chartShift > 0;
}

function stepLidarMapRange() {
  const target = clampLidarRange(state.lidarMap.range);
  if (!Number.isFinite(state.lidarMap.displayRange)) {
    state.lidarMap.displayRange = target;
    return false;
  }
  const diff = target - state.lidarMap.displayRange;
  if (Math.abs(diff) < 0.025) {
    state.lidarMap.displayRange = target;
    return false;
  }
  state.lidarMap.displayRange += diff * 0.22;
  return true;
}

function drawCharts({ redrawLidar = true } = {}) {
  const route = currentRoute();
  // Live sensor/docs pages have no history plots. Avoid constructing every
  // route's series and reading CSS on each history-animation frame there.
  if (route !== "/" && route !== "/system" && route !== "/network" && route !== "/docker" && route !== "/fan") {
    if (redrawLidar && routeSupportsLidarLive()) renderActiveLidarMap();
    return;
  }
  const colors = chartColors();
  const powerMax = Math.max(10, ...state.history.power.filter((v) => v !== null));
  const groups = {
    "/": [
      ["chartOverviewSystem", [
        { values: state.visualHistory.cpu, color: colors.good, max: 100 },
        { values: state.visualHistory.memory, color: colors.cyan, max: 100 },
      ]],
      ["chartOverviewNetwork", [
        { values: normalizeBps(state.visualHistory.rx, "rx"), color: colors.blue, max: 100 },
        { values: normalizeBps(state.visualHistory.tx, "tx"), color: colors.rose, max: 100 },
      ]],
    ],
    "/connection": [],
    "/system": [
      ...(state.cpuMode === "combined" ? [["chartCpu", [{ values: state.visualHistory.cpu, color: colors.good, max: 100 }]]] : []),
      ["chartMemory", [{ values: state.visualHistory.memory, color: colors.cyan, max: 100 }]],
      ["chartSwap", [{ values: state.visualHistory.swap, color: colors.rose, max: 100 }]],
      ["chartSystemNetwork", [
        { values: normalizeBps(state.visualHistory.rx, "rx"), color: colors.blue, max: 100 },
        { values: normalizeBps(state.visualHistory.tx, "tx"), color: colors.rose, max: 100 },
      ]],
      ["chartPower", [{ values: state.visualHistory.power, color: colors.amber, max: powerMax }]],
    ],
    "/network": [
      ["chartNetwork", [
        { values: normalizeBps(state.visualHistory.rx, "rx"), color: colors.blue, max: 100 },
        { values: normalizeBps(state.visualHistory.tx, "tx"), color: colors.rose, max: 100 },
      ]],
    ],
    "/docker": [
      ["chartDockerCpu", [{ values: state.visualHistory.dockerCpu, color: colors.good, max: 100 }]],
      ["chartDockerMem", [{ values: state.visualHistory.dockerMemory, color: colors.cyan, max: 100 }]],
    ],
    "/fan": [
      ["chartFanTemp", [{ values: state.visualHistory.fanTemp, color: colors.amber, max: Math.max(90, ...state.history.fanTemp.filter((v) => v !== null)) }]],
    ],
    "/rover-control": [],
  };
  for (const [id, series] of groups[route] || []) drawMultiLine(id, series);
  if (route === "/system" && state.cpuMode === "separate") drawCoreCharts();
  if (redrawLidar && routeSupportsLidarLive()) renderActiveLidarMap();
}

function renderActiveLidarMap() {
  drawLidarMap(state.lidarScan);
}

function normalizeBps(values, key) {
  const max = Math.max(1, ...state.history[key].filter((v) => v !== null));
  return values.map((value) => (value === null ? null : (value / max) * 100));
}

function normalizeSeries(values, rawValues) {
  const max = Math.max(1, ...rawValues.filter((v) => v !== null));
  return values.map((value) => (value === null ? null : (value / max) * 100));
}

// Series colours of the charts and read-out marks, from the tokens of the
// current theme. The keys are the names the call sites already use; the hues
// are the five chart series: cyan = series 1 (blue), rose = 2 (orange),
// good = 3 (aqua), amber = 4 (yellow), blue = 5 (violet).
function chartColors() {
  return {
    blue: cssVar("--chart-5", "#6250d6"),
    cyan: cssVar("--chart-1", "#2a78d6"),
    rose: cssVar("--chart-2", "#eb6834"),
    amber: cssVar("--chart-4", "#eda100"),
    good: cssVar("--chart-3", "#1baf7a"),
  };
}

function cssVar(name, fallback) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim() || fallback;
}

function drawCoreCharts() {
  const target = $("cpuSeparateView");
  if (!target) return;
  const coreColor = chartColors().good;
  [...target.children].forEach((row) => {
    const canvas = row.querySelector("canvas");
    const key = row.dataset.key;
    if (!canvas || !key) return;
    drawMultiLine(canvas, [{ values: state.visualCoreHistory[key] || [], color: coreColor, max: 100 }]);
  });
}

function drawMultiLine(target, series) {
  const canvas = typeof target === "string" ? $(target) : target;
  if (!canvas) return;
  const prepared = prepareCanvas(canvas);
  if (!prepared) return;
  const { ctx, w, h } = prepared;
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = cssVar("--chart-surface", "#ffffff");
  ctx.fillRect(0, 0, w, h);
  drawGrid(ctx, w, h);
  ctx.save();
  ctx.beginPath();
  ctx.rect(0, 0, w, h);
  ctx.clip();
  for (const item of series) drawLine(ctx, item.values, item.color, item.max || 100, w, h);
  ctx.restore();
}

function prepareCanvas(canvas) {
  const rect = canvas.getBoundingClientRect();
  if (rect.width <= 0 || rect.height <= 0) return null;
  const dpr = Math.min(window.devicePixelRatio || 1, DPR_LIMIT);
  const width = Math.round(rect.width * dpr);
  const height = Math.round(rect.height * dpr);
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  const ctx = canvas.getContext("2d", { alpha: true });
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { ctx, w: rect.width, h: rect.height };
}

// A chart, LiDAR or IMU canvas is redrawn when its own box changes size (panel
// reflow, view switch, rotation), not only on a window "resize". The buffer is
// still sized where it is drawn (prepareCanvas, drawImuAttitude): box in CSS
// pixels times the capped device pixel ratio.
const canvasResize = { observer: null, frame: null, pending: new Set() };

function observeCanvasResize(canvas) {
  if (canvas && canvasResize.observer) canvasResize.observer.observe(canvas);
}

function initCanvasResize() {
  // Without ResizeObserver the window "resize" listener in initMobileNav() and
  // the redraws on new data remain, as before.
  if (typeof ResizeObserver !== "function") return;
  canvasResize.observer = new ResizeObserver(handleCanvasResize);
  document.querySelectorAll("canvas.resource-chart, canvas.lidar-map, canvas.imu-attitude-canvas").forEach(observeCanvasResize);
}

function handleCanvasResize(entries) {
  let charts = false;
  let lidar = false;
  for (const entry of entries) {
    // A hidden page or a closed panel reports an empty box: nothing to draw.
    if (!(entry.contentRect.width > 0 && entry.contentRect.height > 0)) continue;
    const canvas = entry.target;
    if (canvas.id === "roverArchiveLidarCanvas") canvasResize.pending.add("archiveLidar");
    else if (canvas.id === "roverArchiveImuCanvas") canvasResize.pending.add("archiveImu");
    else if (canvas.id === "roverImuCanvas") canvasResize.pending.add("imu");
    else if (canvas.classList.contains("lidar-map")) lidar = true;
    else charts = true;
  }
  if (charts || lidar) requestChartAnimation({ redrawLidar: lidar });
  if (canvasResize.pending.size && !canvasResize.frame) canvasResize.frame = requestAnimationFrame(flushCanvasResize);
}

function flushCanvasResize() {
  canvasResize.frame = null;
  const pending = canvasResize.pending;
  canvasResize.pending = new Set();
  if (pending.has("imu")) redrawImuLiveCanvas();
  const archive = state.archivePlayback;
  const repaintLidar = pending.has("archiveLidar") && archive.lidarPainted;
  if (pending.has("archiveLidar") && !archive.lidarPainted) {
    // Not painted yet (recordings listed, none drawn): the canvas stays blank
    // and only its buffer is matched to the box.
    const canvas = $("roverArchiveLidarCanvas");
    if (canvas) prepareCanvas(canvas);
  }
  if (repaintLidar || (pending.has("archiveImu") && state.recordings.timeline)) {
    // Same path as an archive range change: repaint the frame of the current playback time.
    if (repaintLidar) archive.lastLidarRenderKey = "";
    renderArchiveFrame();
  }
}

function redrawImuLiveCanvas() {
  const euler = imuDisplaySample().euler_deg || {};
  drawImuAttitude($("roverImuCanvas"), Number(euler.roll), Number(euler.pitch), Number(euler.yaw));
}

function drawGrid(ctx, w, h) {
  ctx.save();
  ctx.strokeStyle = cssVar("--chart-grid", "#e3e6ea");
  ctx.lineWidth = 1;
  for (let i = 1; i < 4; i += 1) {
    const y = (h / 4) * i;
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(w, y);
    ctx.stroke();
  }
  ctx.restore();
}

function drawLine(ctx, values, color, max, w, h) {
  const buffered = values.slice(-CHART_BUFFER_POINTS);
  const scrollProgress = buffered.length > MAX_POINTS ? 1 - state.chartShift : 0;
  const points = buffered.map((value, index) => {
    if (value === null || Number.isNaN(value)) return null;
    const offset = Math.max(0, MAX_POINTS - buffered.length);
    const x = buffered.length <= 1 ? w : ((offset + index - scrollProgress) / (MAX_POINTS - 1)) * w;
    const safe = Math.max(0, Math.min(max, value));
    const y = h - (safe / max) * (h - 18) - 9;
    return { x, y };
  });
  ctx.save();
  ctx.strokeStyle = color;
  ctx.lineWidth = 1.5;
  ctx.lineJoin = "round";
  ctx.lineCap = "round";
  ctx.beginPath();
  let segment = [];
  for (const point of points) {
    if (point) {
      segment.push(point);
      continue;
    }
    strokeSegment(ctx, segment);
    segment = [];
  }
  strokeSegment(ctx, segment);
  ctx.stroke();
  ctx.restore();
}

function strokeSegment(ctx, points) {
  if (!points.length) return;
  ctx.moveTo(points[0].x, points[0].y);
  if (points.length === 1) {
    ctx.lineTo(points[0].x + 0.01, points[0].y);
    return;
  }
  for (let i = 1; i < points.length; i += 1) {
    const prev = points[i - 1];
    const current = points[i];
    const midX = (prev.x + current.x) / 2;
    const midY = (prev.y + current.y) / 2;
    ctx.quadraticCurveTo(prev.x, prev.y, midX, midY);
  }
  const last = points[points.length - 1];
  ctx.lineTo(last.x, last.y);
}

function drawLidarMap(result) {
  drawLidarMapOnCanvas($("lidarScanCanvas"), result, null, {});
  drawLidarMapOnCanvas($("roverLidarCanvas"), result, null, {});
}

// Returns true when the canvas was painted (false: no canvas, or its box is empty).
function drawLidarMapOnCanvas(canvas, result, overlays = {}, options = {}) {
  if (!canvas) return false;
  const prepared = prepareCanvas(canvas);
  if (!prepared) return false;
  const { ctx, w, h } = prepared;
  const colors = chartColors();
  const points = result?.ok ? result.points || [] : [];
  const showPoints = true;
  const radius = Math.max(80, Math.min(w, h) * 0.39);
  const cx = w / 2;
  const cy = h / 2;
  const optionRange = Number(options.range);
  const range = Number.isFinite(optionRange) ? clampLidarRange(optionRange) : lidarDrawRange();
  const gridStep = lidarGridStep(range);

  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = cssVar("--chart-surface", "#ffffff");
  ctx.fillRect(0, 0, w, h);
  drawLidarGrid(ctx, cx, cy, radius, range, gridStep);

  if (!points.length) {
    drawLidarEmpty(ctx, w, h, result?.message || "스캔 대기");
    return true;
  }

  if (showPoints && points.length) {
    ctx.save();
    ctx.fillStyle = colors.cyan;
    for (const point of points) {
      const { x, y } = lidarCanvasPoint(point, { cx, cy, range, radius });
      const quality = Math.max(0, Math.min(63, Number(point.quality) || 0));
      ctx.globalAlpha = Math.max(0.75, Math.min(0.95, 0.75 + quality / 315));
      ctx.beginPath();
      ctx.arc(x, y, Math.max(1.6, Math.min(2.2, 1.6 + quality / 105)), 0, Math.PI * 2);
      ctx.fill();
    }
    ctx.restore();
  }


  ctx.save();
  ctx.fillStyle = cssVar("--color-text", "#1b2027");
  ctx.font = '600 12px "UBTS Sans", system-ui, -apple-system, "Segoe UI", "Malgun Gothic", sans-serif';
  ctx.fillText(`${showPoints ? points.length : 0} points`, 14, 22);
  ctx.fillText(`${fmtNumber(result?.metrics?.coverage_deg ?? angleCoverage(points), 0)}° coverage`, 14, 40);
  ctx.textAlign = "right";
  ctx.fillText(`Grid: ${fmtNumber(gridStep, 0)}m · Range: ${fmtNumber(range, 1)}m`, w - 14, h - 16);
  ctx.restore();
  return true;
}

function lidarCanvasPoint(point, viewport) {
  // Display-only half turn; preserve raw angles, exports and mapping inputs.
  return {
    x: viewport.cx - ((Number(point?.x_m) || 0) / viewport.range) * viewport.radius,
    y: viewport.cy + ((Number(point?.y_m) || 0) / viewport.range) * viewport.radius,
  };
}


function lidarRangeFor(result) {
  return clampLidarRange(state.lidarMap.range);
}

function lidarDrawRange() {
  return clampLidarRange(state.lidarMap.displayRange || state.lidarMap.range);
}

function lidarGridStep(range) {
  if (range <= 6) return 1;
  if (range <= 12) return 2;
  return 5;
}

function drawLidarGrid(ctx, cx, cy, radius, range, gridStep) {
  ctx.save();
  ctx.strokeStyle = cssVar("--chart-grid", "#e3e6ea");
  ctx.lineWidth = 1;
  for (let meters = gridStep; meters <= range + 0.001; meters += gridStep) {
    const r = (meters / range) * radius;
    ctx.beginPath();
    ctx.arc(cx, cy, r, 0, Math.PI * 2);
    ctx.stroke();
    ctx.fillStyle = cssVar("--color-text-secondary", "#586270");
    ctx.font = '400 12px "UBTS Sans", system-ui, -apple-system, "Segoe UI", "Malgun Gothic", sans-serif';
    ctx.fillText(`${fmtNumber(meters, 0)}m`, cx + 7, cy - r + 13);
  }
  ctx.beginPath();
  ctx.moveTo(cx - radius, cy);
  ctx.lineTo(cx + radius, cy);
  ctx.moveTo(cx, cy - radius);
  ctx.lineTo(cx, cy + radius);
  ctx.stroke();
  ctx.fillStyle = cssVar("--color-text-secondary", "#586270");
  ctx.font = '600 12px "UBTS Sans", system-ui, -apple-system, "Segoe UI", "Malgun Gothic", sans-serif';
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  const labelOffset = 20;
  ctx.fillText("180°", cx, cy - radius - labelOffset);
  ctx.fillText("90°", cx + radius + labelOffset, cy);
  ctx.fillText("0°", cx, cy + radius + labelOffset);
  ctx.fillText("270°", cx - radius - labelOffset, cy);
  ctx.fillStyle = cssVar("--color-text", "#1b2027");
  ctx.beginPath();
  ctx.arc(cx, cy, 5, 0, Math.PI * 2);
  ctx.fill();
  ctx.restore();
}

function drawLidarEmpty(ctx, w, h, message) {
  // Top-left corner, where the point count is written once a scan arrives.
  // The centre holds the origin mark, the axes and the ring labels, and the
  // top centre holds the "180°" label: the text must reach none of them.
  ctx.save();
  ctx.fillStyle = cssVar("--color-text-secondary", "#586270");
  ctx.font = '600 14px "UBTS Sans", system-ui, -apple-system, "Segoe UI", "Malgun Gothic", sans-serif';
  ctx.textAlign = "left";
  ctx.textBaseline = "middle";
  const left = 14;
  const maxWidth = Math.max(24, w / 2 - 24 - left);
  let label = String(message ?? "");
  if (ctx.measureText(label).width > maxWidth) {
    while (label.length > 1 && ctx.measureText(`${label}…`).width > maxWidth) label = label.slice(0, -1);
    label = `${label}…`;
  }
  ctx.fillText(label, left, 22);
  ctx.restore();
}

function escapeHtml(value) {
  return String(value ?? "").replace(/[&<>"']/g, (ch) => ({
    "&": "&amp;",
    "<": "&lt;",
    ">": "&gt;",
    '"': "&quot;",
    "'": "&#039;",
  })[ch]);
}
