
function initLidarTest() {
  $("lidarTestButton")?.addEventListener("click", runLidarTest);
  $("lidarLiveButton")?.addEventListener("click", toggleLidarLive);
  $("lidarDownloadButton")?.addEventListener("click", downloadCurrentLidarMap);
  document.querySelectorAll("[data-lidar-download]").forEach((button) => {
    button.addEventListener("click", () => setLidarDownloadFormat(button.dataset.lidarDownload));
  });
  document.querySelectorAll("[data-lidar-range]").forEach((button) => {
    button.addEventListener("click", () => changeLidarRange(button.dataset.lidarRange));
  });
  document.querySelectorAll("[data-lidar-range-slider]").forEach((slider) => slider.addEventListener("input", (event) => {
    setLidarRange(Number(event.target.value), { auto: false, persist: true });
    requestChartAnimation();
  }));
  document.querySelectorAll("[data-lidar-points]").forEach((button) => {
    button.addEventListener("click", () => changeLidarPointLimit(button.dataset.lidarPoints));
  });
  document.querySelectorAll("[data-lidar-point-slider]").forEach((slider) => slider.addEventListener("input", (event) => {
    setLidarPointLimit(Number(event.target.value), { persist: true, reconnect: true });
  }));

  initLidarRange();
  initLidarPointLimit();

  setLidarDownloadFormat(state.lidarExport.format);
  renderLidarLiveState();
}

function initCameraLive() {
  try {
    state.cameraLive.resolution = localStorage.getItem(CAMERA_RESOLUTION_KEY) || state.cameraLive.resolution;
    state.cameraLive.fps = Number(localStorage.getItem(CAMERA_FPS_KEY) || state.cameraLive.fps);
  } catch (_) {
    // Camera controls remain usable without localStorage.
  }
  enforceCameraSelection();
  document.querySelectorAll("[data-camera-resolution-select]").forEach((resolution) => {
    resolution.value = state.cameraLive.resolution;
    resolution.addEventListener("change", () => setCameraSettings({ resolution: resolution.value }));
  });
  document.querySelectorAll("[data-camera-fps-select]").forEach((fps) => {
    fps.value = String(state.cameraLive.fps);
    fps.addEventListener("change", () => setCameraSettings({ fps: Number(fps.value) }));
  });

  $("cameraLiveButton")?.addEventListener("click", toggleCameraLive);
  updateCameraControls();

  renderCameraLiveState();
}

function normalizeCameraModes(rawModes) {
  const source = Array.isArray(rawModes) && rawModes.length ? rawModes : DEFAULT_CAMERA_MODES;
  const rows = [];
  const seen = new Set();
  source.forEach((item) => {
    if (!item) return;
    let resolution = String(item.resolution || "");
    if (!resolution.includes("x") && item.width && item.height) resolution = `${item.width}x${item.height}`;
    const [widthText, heightText] = resolution.toLowerCase().split("x");
    const width = Number(widthText);
    const height = Number(heightText);
    if (!Number.isFinite(width) || !Number.isFinite(height) || width <= 0 || height <= 0 || seen.has(resolution)) return;
    const fps = [...new Set((Array.isArray(item.fps) ? item.fps : [])
      .map((value) => Number(value))
      .filter((value) => Number.isFinite(value) && value >= 1)
      .map((value) => Math.abs(value - Math.round(value)) < 0.05 ? Math.round(value) : Number(value.toFixed(2))))].sort((a, b) => a - b);
    rows.push({ resolution, width, height, fps: fps.length ? fps : [30], label: item.label || "" });
    seen.add(resolution);
  });
  return rows.length ? rows : DEFAULT_CAMERA_MODES;
}

function applyCameraModeCatalog(rawModes, source = "fallback") {
  if (source !== "settings" && !cameraTelemetryCanApply()) return;
  const modes = normalizeCameraModes(rawModes);
  const signature = JSON.stringify(modes);
  if (signature === state.cameraModesSignature && source === state.cameraModesSource) return;
  const before = `${state.cameraLive.resolution}@${state.cameraLive.fps}`;
  state.cameraModes = modes;
  state.cameraModesSignature = signature;
  state.cameraModesSource = source || "fallback";
  enforceCameraSelection();
  updateCameraControls();
  const after = `${state.cameraLive.resolution}@${state.cameraLive.fps}`;
  if (before !== after && (state.cameraLive.socket || state.cameraLive.enabled)) {
    scheduleCameraReconnectForSettings();
  }
}

function applyCameraStatus(camera = {}, cameraBridge = {}) {
  if (Number.isFinite(Number(cameraBridge.source_fps))) {
    state.cameraLive.sourceFps = Number(cameraBridge.source_fps);
  }
  if (cameraBridge.source_width && cameraBridge.source_height) {
    state.cameraLive.sourceWidth = cameraBridge.source_width;
    state.cameraLive.sourceHeight = cameraBridge.source_height;
  }
  if (!cameraTelemetryCanApply()) return;
  const requested = camera.settings || camera.driver_requested || {};
  const resolution = requested.resolution || (requested.width && requested.height ? `${requested.width}x${requested.height}` : "");
  const fps = Number(requested.fps);
  const resolutions = cameraResolutionOptions();
  if (!resolutions.includes(resolution) || !Number.isFinite(fps)) return;
  const allowed = cameraFpsOptions(resolution);
  const nextFps = chooseCameraFps(allowed, fps);
  const changed = state.cameraLive.resolution !== resolution || Number(state.cameraLive.fps) !== Number(nextFps);
  // First report from the server: the labels may now show the values.
  const firstReport = !state.cameraLive.settingsKnown;
  state.cameraLive.resolution = resolution;
  state.cameraLive.fps = nextFps;
  state.cameraLive.settingsKnown = true;
  if (changed || firstReport) updateCameraControls();
}

function cameraTelemetryCanApply() {
  if (state.cameraLive.settingsInFlight || Date.now() < (state.cameraLive.settingsIgnoreUntil || 0)) return false;
  // A slow telemetry response may have left before the latest settings POST.
  return state.telemetryCameraSettingsRevision === undefined ||
    state.telemetryCameraSettingsRevision === (state.cameraLive.settingsRevision || 0);
}

function cameraResolutionOptions() {
  return state.cameraModes.map((mode) => mode.resolution);
}

function cameraResolutionOptionLabels() {
  return state.cameraModes.map((mode) => ({
    value: mode.resolution,
    label: mode.label ? `${mode.resolution} · ${mode.label}` : mode.resolution,
  }));
}

function cameraFpsOptions(resolution = state.cameraLive.resolution) {
  const mode = state.cameraModes.find((item) => item.resolution === resolution);
  return mode?.fps?.length ? mode.fps : [30];
}

function chooseCameraFps(allowed, preferred) {
  const current = Number(preferred);
  if (allowed.includes(current)) return current;
  const lower = allowed.filter((value) => value <= current).pop();
  return lower || allowed[allowed.length - 1] || 30;
}

function enforceCameraSelection() {
  const resolutions = cameraResolutionOptions();
  if (!resolutions.includes(state.cameraLive.resolution)) {
    state.cameraLive.resolution = resolutions.includes("1280x720") ? "1280x720" : resolutions[0] || "1280x720";
  }
  const fps = cameraFpsOptions(state.cameraLive.resolution);
  state.cameraLive.fps = chooseCameraFps(fps, state.cameraLive.fps);
}

const CAMERA_SETTINGS_ERRORS = {
  camera_resolution_locked_while_recording: "녹화 중에는 해상도를 바꿀 수 없습니다. 녹화를 멈춘 뒤 바꾸세요(fps는 변경 가능)",
  camera_resolution_change_needs_idle_recorder: "녹화기 상태를 확인할 수 없어 해상도를 바꾸지 않았습니다",
};

async function readJsonResponse(res) {
  // Proxy/tunnel error pages (502/524 HTML) must not surface as JSON parse errors (S41).
  const type = res.headers?.get?.("content-type") || "";
  if (type && !type.toLowerCase().includes("json")) throw new Error(`HTTP ${res.status}`);
  try { return await res.json(); } catch (error) {
    if (error?.name === "AbortError") throw error;
    throw new Error(`HTTP ${res.status}`);
  }
}

async function setCameraSettings(next) {
  if (state.cameraLive.settingsInFlight) { updateCameraControls(true); return; }
  const previousCamera = { resolution: state.cameraLive.resolution, fps: state.cameraLive.fps };
  state.cameraLive.settingsRevision = (state.cameraLive.settingsRevision || 0) + 1;
  state.cameraLive.settingsIgnoreUntil = Date.now() + 2000;
  state.cameraLive.settingsInFlight = true;
  if (next.resolution) {
    const resolutions = cameraResolutionOptions();
    state.cameraLive.resolution = resolutions.includes(next.resolution) ? next.resolution : (resolutions[0] || "1280x720");
  }
  if (next.fps) state.cameraLive.fps = Number(next.fps);
  enforceCameraSelection();
  try {
    localStorage.setItem(CAMERA_RESOLUTION_KEY, state.cameraLive.resolution);
    localStorage.setItem(CAMERA_FPS_KEY, String(state.cameraLive.fps));
  } catch (_) {
    // Ignore storage failures.
  }
  updateCameraControls(true);
  state.cameraLive.settingsError = "";
  setCameraResult("testing", `카메라 캡처 설정 적용 중 · ${state.cameraLive.resolution} · ${state.cameraLive.fps} fps`);
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 8000);
  try {
    const [width, height] = state.cameraLive.resolution.split("x").map((value) => Number(value));
    const res = await fetch("/api/camera/settings", {
      method: "POST",
      cache: "no-store",
      signal: controller.signal,
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        resolution: state.cameraLive.resolution,
        width,
        height,
        fps: state.cameraLive.fps,
      }),
    });
    const result = await readJsonResponse(res);
    if (!res.ok || !result.ok) {
      throw new Error(CAMERA_SETTINGS_ERRORS[result.error] || result.error || `HTTP ${res.status}`);
    }
    if (Array.isArray(result.supported_view_modes)) {
      applyCameraModeCatalog(result.supported_view_modes, "settings");
    }
    state.cameraLive.settingsKnown = true; // accepted by the server
    setCameraResult("testing", `캡처 설정 저장됨 · ${state.cameraLive.resolution} · ${state.cameraLive.fps} fps`);
    scheduleCameraReconnectForSettings(900);
  } catch (err) {
    state.cameraLive.settingsError = err.message || String(err);
    // S27: the rejected value must not stay selected or stored.
    state.cameraLive.resolution = previousCamera.resolution;
    state.cameraLive.fps = previousCamera.fps;
    try {
      localStorage.setItem(CAMERA_RESOLUTION_KEY, state.cameraLive.resolution);
      localStorage.setItem(CAMERA_FPS_KEY, String(state.cameraLive.fps));
    } catch (_) {
      // Ignore storage failures.
    }
    setCameraResult("bad", `카메라 설정 적용 실패: ${state.cameraLive.settingsError} · 이전 설정 유지`);
  } finally {
    clearTimeout(timeout);
    state.cameraLive.settingsInFlight = false;
    state.cameraLive.settingsIgnoreUntil = Date.now() + 2000;
    updateCameraControls(true);
  }
}

function updateCameraControls(force = false) {
  // Before the server has reported or accepted the capture settings, the
  // selection is only this browser's request: show "--", not a made-up fact.
  const known = Boolean(state.cameraLive.settingsKnown);
  const resolutionLabel = known ? state.cameraLive.resolution : "--";
  const fpsLabel = known ? `${state.cameraLive.fps} fps` : "--";
  text("cameraResolutionValue", resolutionLabel);
  text("cameraFpsValue", fpsLabel);
  document.querySelectorAll("[data-camera-resolution-value]").forEach((el) => {
    el.textContent = resolutionLabel;
  });
  document.querySelectorAll("[data-camera-fps-value]").forEach((el) => {
    el.textContent = fpsLabel;
  });
  document.querySelectorAll("[data-camera-resolution-select]").forEach((resolution) => {
    syncSelectOptions(resolution, cameraResolutionOptionLabels());
    resolution.disabled = Boolean(state.cameraLive.settingsInFlight);
    if (force || document.activeElement !== resolution) resolution.value = state.cameraLive.resolution;
  });
  document.querySelectorAll("[data-camera-fps-select]").forEach((fps) => {
    syncSelectOptions(fps, cameraFpsOptions().map((value) => ({ value: String(value), label: `${value} fps` })));
    fps.disabled = Boolean(state.cameraLive.settingsInFlight);
    if (force || document.activeElement !== fps) fps.value = String(state.cameraLive.fps);
  });
}


function syncSelectOptions(select, options) {
  const signature = JSON.stringify(options);
  if (select.dataset.optionsSignature === signature) return;
  select.dataset.optionsSignature = signature;
  select.replaceChildren(...options.map((option) => {
    const node = document.createElement("option");
    node.value = option.value;
    node.textContent = option.label;
    return node;
  }));
}

function initLidarRange() {
  let stored = null;
  try {
    const rawRange = localStorage.getItem(LIDAR_RANGE_KEY);
    stored = rawRange === null ? null : Number(rawRange);
    const storedAuto = localStorage.getItem(LIDAR_RANGE_AUTO_KEY);
    state.lidarMap.auto = storedAuto === null ? true : storedAuto !== "0";
    state.lidarMap.autoPending = state.lidarMap.auto;
  } catch (_) {
    stored = null;
  }
  if (Number.isFinite(stored) && !state.lidarMap.auto) {
    state.lidarMap.range = clampLidarRange(stored);
    state.lidarMap.displayRange = state.lidarMap.range;
  }
  updateLidarRangeControls();
}

function changeLidarRange(action) {
  const current = state.lidarMap.range;
  let next = current;
  if (action === "in") next = current <= 5 ? current - 0.5 : current - 1;
  else if (action === "out") next = current < 5 ? current + 0.5 : current + 1;
  else {
    state.lidarMap.auto = true;
    state.lidarMap.autoPending = true;
    persistLidarRange();
    maybeAutoSetLidarRange(state.lidarScan, { force: true });
    updateLidarRangeControls();
    requestChartAnimation();
    return;
  }
  setLidarRange(next, { auto: false, persist: true });
  requestChartAnimation();
}

function setLidarRange(value, options = {}) {
  state.lidarMap.range = clampLidarRange(value);
  state.lidarMap.auto = Boolean(options.auto);
  state.lidarMap.autoPending = Boolean(options.autoPending);
  if (options.persist) persistLidarRange();
  updateLidarRangeControls();
}

function persistLidarRange() {
  try {
    localStorage.setItem(LIDAR_RANGE_KEY, String(state.lidarMap.range));
    localStorage.setItem(LIDAR_RANGE_AUTO_KEY, state.lidarMap.auto ? "1" : "0");
  } catch (_) {
    // Range control remains usable without localStorage.
  }
}

function clampLidarRange(value) {
  return Math.max(state.lidarMap.minRange, Math.min(state.lidarMap.maxRange, Number(value) || 25));
}

function updateLidarRangeControls() {
  const rangeLabel = `${fmtNumber(state.lidarMap.range, state.lidarMap.range % 1 ? 1 : 0)} m`;
  text("lidarRangeValue", state.lidarMap.auto ? `${rangeLabel} auto` : rangeLabel);
  document.querySelectorAll("[data-lidar-range-value]").forEach((el) => {
    el.textContent = state.lidarMap.auto ? `${rangeLabel} auto` : rangeLabel;
  });
  document.querySelectorAll("[data-lidar-range-slider]").forEach((slider) => {
    if (document.activeElement !== slider) slider.value = String(state.lidarMap.range);
  });
  document.querySelectorAll("[data-lidar-range]").forEach((button) => {
    const action = button.dataset.lidarRange;
    button.disabled = (action === "in" && state.lidarMap.range <= state.lidarMap.minRange)
      || (action === "out" && state.lidarMap.range >= state.lidarMap.maxRange);
    if (action === "reset") button.textContent = state.lidarMap.auto ? "Auto" : "Auto";
  });
}

function initLidarPointLimit() {
  let stored = null;
  try {
    const raw = localStorage.getItem(LIDAR_POINTS_KEY);
    stored = raw === null ? null : Number(raw);
  } catch (_) {
    stored = null;
  }
  if (Number.isFinite(stored)) state.lidarPoints.value = clampLidarPointLimit(stored);
  updateLidarPointControls();
}

function changeLidarPointLimit(action) {
  const current = state.lidarPoints.value;
  let next = current;
  if (action === "down") next = current - state.lidarPoints.step;
  else if (action === "up") next = current + state.lidarPoints.step;
  else next = 900;
  setLidarPointLimit(next, { persist: true, reconnect: true });
}

function setLidarPointLimit(value, options = {}) {
  state.lidarPoints.value = clampLidarPointLimit(value);
  if (options.persist) {
    try {
      localStorage.setItem(LIDAR_POINTS_KEY, String(state.lidarPoints.value));
    } catch (_) {
      // Point limit remains session-local when storage is unavailable.
    }
  }
  updateLidarPointControls();
  if (options.reconnect && (state.lidarLive.socket || state.lidarLive.enabled)) scheduleLidarReconnectForSettings();
}

function clampLidarPointLimit(value) {
  const step = state.lidarPoints.step;
  const rounded = Math.round((Number(value) || 900) / step) * step;
  return Math.max(state.lidarPoints.min, Math.min(state.lidarPoints.max, rounded));
}

function updateLidarPointControls() {
  text("lidarPointValue", `${state.lidarPoints.value} pts`);
  document.querySelectorAll("[data-lidar-point-value]").forEach((el) => {
    el.textContent = `${state.lidarPoints.value} pts`;
  });
  document.querySelectorAll("[data-lidar-point-slider]").forEach((slider) => {
    if (document.activeElement !== slider) slider.value = String(state.lidarPoints.value);
  });
  document.querySelectorAll("[data-lidar-points]").forEach((button) => {
    const action = button.dataset.lidarPoints;
    button.disabled = (action === "down" && state.lidarPoints.value <= state.lidarPoints.min)
      || (action === "up" && state.lidarPoints.value >= state.lidarPoints.max);
  });
}
