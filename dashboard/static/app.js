const SIMULATION = document.documentElement.dataset.runtimeMode === "simulation";
const ROUTES = ["/", "/connection", "/system", "/network", "/docker", "/fan", "/research", "/rover-control", "/docs"];
const HISTORY_KEYS = ["cpu", "memory", "swap", "power", "temp", "rx", "tx", "dockerCpu", "dockerMemory", "fanTemp", "fanPwm"];
const CORE_COLORS = ["#55e2b2", "#38d8ff", "#2f76ff", "#8238c9", "#f21b58", "#f4bd4f", "#7dd3fc", "#fb7185"];
const MAX_POINTS = 90;
const CHART_BUFFER_POINTS = MAX_POINTS + 1;
const POLL_INTERVAL_MS = 1000;
const DPR_LIMIT = 1.5;
const CPU_MODE_KEY = "ubts-cpu-chart-mode";
const LIDAR_RANGE_KEY = "ubts-lidar-map-range-v2";
const LIDAR_RANGE_AUTO_KEY = "ubts-lidar-map-range-auto-v2";
const LIDAR_POINTS_KEY = "ubts-lidar-points-v1";
const ARCHIVE_RANGE_KEY = "ubts-archive-lidar-range-v1";
const ARCHIVE_SEGMENT_SECONDS = 4;
const ARCHIVE_PLAYBACK_PROFILE = "dashboard-polar-v2";
const ARCHIVE_IMU_SYNC_MAX_DELTA_S = 0.2;
const LIDAR_SPEC_MAX_POINTS = 3200;
const LIDAR_COORDINATE_FRAME = "lidar_bridge_xy_v1";
const LIDAR_RECONNECT_DEBOUNCE_MS = 650;
const CAMERA_RESOLUTION_KEY = "ubts-camera-resolution-v1";
const CAMERA_FPS_KEY = "ubts-camera-fps-v1";
const CAMERA_RECONNECT_DEBOUNCE_MS = 500;
const CAMERA_BINARY_MAGIC = "UBCF";
const CAMERA_BINARY_HEADER_SIZE = 9;
const IMU_RECORD_ENABLED_KEY = "ubts-record-imu-enabled-v1";
const ROVER_SPEED_KEY = "ubts-rover-speed-v1";
const ROVER_RECORD_KIND_KEY = "ubts-rover-record-kind-v1";
const ROVER_PANEL_COLLAPSED_KEY = "ubts-rover-panel-collapsed-v1";
const ROVER_CONTROL_INTERVAL_MS = 140;
const ROVER_MAX_LINEAR_MPS = 0.45;
const ROVER_MAX_ANGULAR_RPS = 1.4;
const DEFAULT_CAMERA_MODES = [
  { resolution: "1920x1080", width: 1920, height: 1080, fps: [30], label: "1080p 16:9" },
  { resolution: "1280x720", width: 1280, height: 720, fps: [60], label: "720p 16:9" },
  { resolution: "1024x768", width: 1024, height: 768, fps: [30], label: "768p 4:3" },
  { resolution: "800x600", width: 800, height: 600, fps: [60], label: "600p 4:3" },
  { resolution: "640x480", width: 640, height: 480, fps: [30], label: "480p 4:3" },
  { resolution: "320x240", width: 320, height: 240, fps: [30], label: "240p 4:3" },
  { resolution: "1280x1024", width: 1280, height: 1024, fps: [30], label: "1024p 5:4" },
];


const state = {
  telemetry: null,
  plan: null,
  docs: [],
  activeDoc: null,
  history: Object.fromEntries(HISTORY_KEYS.map((key) => [key, []])),
  visualHistory: Object.fromEntries(HISTORY_KEYS.map((key) => [key, []])),
  coreHistory: {},
  visualCoreHistory: {},
  cpuMode: "combined",
  lidarScan: null,
  lidarLive: {
    enabled: false,
    socket: null,
    connected: false,
    scanTopicAlive: false,
    source: "waiting",
    lastMessageAt: null,
    lastScanAgeSec: null,
    pointCount: 0,
    sourcePointCount: 0,
    requestedPointCount: 900,
    error: "",
    messageCount: 0,
    userStopped: false,
    reconnectTimer: null,
    pendingRestart: false,
  },
  lidarExport: {
    latestScan: null,
    lastUpdatedAt: null,
    source: "ros2_scan",
    frameId: "laser",
    format: "json",
  },
  lidarMap: {
    range: 25,
    displayRange: 25,
    auto: true,
    autoPending: true,
    minRange: 1,
    maxRange: 30,
  },
	  lidarPoints: {
	    value: 900,
	    min: 50,
	    max: LIDAR_SPEC_MAX_POINTS,
	    step: 50,
	  },
	  cameraLive: {
    enabled: false,
    socket: null,
    connected: false,
    frameTopicAlive: false,
    lastFrameAgeSec: null,
    frameCount: 0,
    displayFps: null,
    sourceFps: null,
    fpsWindowStartedAt: null,
    fpsWindowFrames: 0,
    sourceWidth: null,
    sourceHeight: null,
    frameId: "camera_link_optical",
    error: "",
    userStopped: false,
    reconnectTimer: null,
    pendingRestart: false,
    resolution: "1280x720",
    fps: 60,
    settingsInFlight: false,
    settingsError: "",
	    lastFrameAt: null,
	    lastFrameSize: null,
    transport: "binary-v1",
    flow: "latest-ack",
    objectUrl: "",
    encodedBytes: 0,
    encodeMs: null,
    deliveryState: "starting",
    decodeStatus: "pending",
    effectiveProfile: "",
    previewQuality: null,
    ackLatencyMs: null,
    lastDecodedAt: null,
  },
  imuLive: {
    enabled: false,
    socket: null,
    connected: false,
    topicAlive: false,
    raw: {},
    magnetic: {},
    lastMessageAt: null,
    lastImuAgeSec: null,
    error: "",
    userStopped: false,
    reconnectTimer: null,
  },
  cameraModes: DEFAULT_CAMERA_MODES,
  cameraModesSignature: JSON.stringify(DEFAULT_CAMERA_MODES),
  cameraModesSource: "fallback",
	  roverControl: {
	    speedPct: 25,
	    linear: 0,
	    angular: 0,
	    targetLinear: 0,
	    targetAngular: 0,
	    currentLinear: 0,
	    currentAngular: 0,
	    activeKeys: new Set(),
	    speedKeys: new Set(),
	    activeButton: "",
	    stoppingWithFade: false,
	    commandTimer: null,
	    inFlight: false,
	    lastCommandAt: null,
	    lastError: "",
	    panelCollapsed: false,
	  },
	  recordings: {
	    status: null,
	    items: [],
	    selectedId: null,
	    selected: null,
	    timeline: null,
	    loading: false,
	    actionInFlight: false,
	    kind: "both",
	    includeImu: true,
	    includeGps: true,
	    lastStatusAt: 0,
	  },
	  archivePlayback: {
	    playing: false,
	    timeS: 0,
	    durationS: 0,
	    startedAt: 0,
	    startTimeS: 0,
	    raf: null,
	    tickTimer: null,
	    overlayMode: "both",
	    overlayVisible: true,
	    sync: true,
	    range: 25,
	    fallbackPreview: null,
	    cache: { lidarFrames: [], cameraFrames: [], imuFrames: [], gpsFrames: [], motorFrames: [], motorKeys: new Set(), gpsKeys: new Set(), lidarKeys: new Set(), cameraKeys: new Set(), imuKeys: new Set(), objectUrls: new Map(), imageElements: new Map() },
	    segmentDurationS: ARCHIVE_SEGMENT_SECONDS,
	    segmentCache: new Map(),
	    loadingSegments: new Map(),
        telemetrySegments: new Map(),
        loadingTelemetry: new Map(),
	    lastSegmentWindowKey: "",
	    seekTimer: null,
	    lastPlaybackRenderAt: 0,
	    lastLidarRenderKey: "",
	    lastCameraRenderKey: "",
	    nativeVideoUrl: "",
	    nativeVideoReady: false,
	    nativeVideoError: "",
	    resumeLiveCamera: false,
        videoClock: null,
        videoClockExplicit: false,
        videoClockState: null,
        videoClockKey: "",
        videoClockWarm: null,
        videoSeekRevision: 0,
        videoSeekPending: false,
        videoPlayRevision: 0,
        videoPlayRequested: false,
        clockBuffering: false,
        videoClockStartSkippedS: 0,
        videoClockTerminalCompleted: false,
	  },
  selectedContainerId: null,
  containerDetail: null,
  containerLogs: null,
  containerRequestInFlight: false,
  containerLogInFlight: false,
  containerLogsUpdatedAt: null,
  chartShift: 0,
  chartShiftStartedAt: 0,
  chartShiftDurationMs: POLL_INTERVAL_MS,
  animationFrame: null,
  chartLidarRedrawRequested: false,
  apiLogCounter: 0,
  tickInFlight: false,
  telemetryFailures: 0,
  networkControl: {
    actionInFlight: false,
    message: "네트워크 제어 상태를 확인하고 있습니다.",
    level: "testing",
  },
};

const $ = (id) => document.getElementById(id);

function text(id, value) {
  const el = $(id);
  if (!el) return;
  const next = String(value);
  if (el.textContent !== next) el.textContent = next;
}

function fmtPercent(value, digits = 1) {
  return value === null || value === undefined ? "--" : `${Number(value).toFixed(digits)}%`;
}

function fmtBytes(value) {
  if (value === null || value === undefined) return "--";
  const units = ["B", "KB", "MB", "GB", "TB"];
  let n = Number(value);
  let i = 0;
  while (n >= 1024 && i < units.length - 1) {
    n /= 1024;
    i += 1;
  }
  return `${n.toFixed(i === 0 ? 0 : 1)} ${units[i]}`;
}

function fmtBps(value) {
  if (value === null || value === undefined) return "--";
  return `${fmtBytes(value)}/s`;
}

function fmtTemp(value) {
  return value === null || value === undefined ? "--" : `${Number(value).toFixed(1)}°C`;
}

function fmtPower(value) {
  return value === null || value === undefined ? "--" : `${Number(value).toFixed(2)} W`;
}

function fmtNumber(value, digits = 1) {
  return value === null || value === undefined || Number.isNaN(Number(value)) ? "--" : Number(value).toFixed(digits);
}

function fmtDuration(seconds) {
  if (seconds === null || seconds === undefined) return "--";
  const s = Math.max(0, Number(seconds));
  const d = Math.floor(s / 86400);
  const h = Math.floor((s % 86400) / 3600);
  const m = Math.floor((s % 3600) / 60);
  return d > 0 ? `${d}d ${h}h` : `${h}h ${m}m`;
}

function fmtClock(seconds) {
  const total = Math.max(0, Math.floor(Number(seconds) || 0));
  const h = Math.floor(total / 3600);
  const m = Math.floor((total % 3600) / 60);
  const s = total % 60;
  const pad = (value) => String(value).padStart(2, "0");
  return h > 0 ? `${h}:${pad(m)}:${pad(s)}` : `${pad(m)}:${pad(s)}`;
}

function memoryPercent(memory) {
  return memory?.display_percent ?? memory?.percent;
}

function memoryTotal(memory) {
  return memory?.display_total ?? memory?.total;
}

function memoryLabel(memory) {
  return memory?.display_label || `${fmtBytes(memoryTotal(memory))} 기준`;
}

function cpuLabel(core) {
  if (core?.label) return core.label;
  const name = core?.name || "";
  if (name.startsWith("cpu") && /^\d+$/.test(name.slice(3))) return `CPU ${Number(name.slice(3)) + 1}`;
  return name.toUpperCase() || "CPU";
}

function coreKey(core) {
  return core?.name || cpuLabel(core);
}

function debugLog(scope, message, detail) {
  if (!window.console?.debug) return;
  if (detail === undefined) {
    console.debug(`[UBTS:${scope}] ${message}`);
  } else {
    console.debug(`[UBTS:${scope}] ${message}`, detail);
  }
}

async function getJson(path, options = {}) {
  const started = performance.now();
  try {
    const res = await fetch(path, { cache: "no-store", signal: options.signal });
    const ms = Math.round(performance.now() - started);
    state.apiLogCounter += 1;
    if (path !== "/api/telemetry" || state.apiLogCounter % 10 === 0) {
      debugLog("api", `${path} -> ${res.status} (${ms}ms)`);
    }
    if (!res.ok) throw new Error(`${path}: ${res.status}`);
    return res.json();
  } catch (err) {
    debugLog("api", `${path} failed`, err);
    throw err;
  }
}

async function postJson(path, payload = {}) {
  const res = await fetch(path, {
    method: "POST",
    cache: "no-store",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload || {}),
  });
  if (!res.ok) throw new Error(`${path}: ${res.status}`);
  return res.json();
}

function currentRoute() {
  if (location.pathname === "/docs" || location.pathname.startsWith("/docs/")) return "/docs";
  return ROUTES.includes(location.pathname) ? location.pathname : "/";
}

function routeSupportsLidarLive(route = currentRoute()) {
  return route === "/connection" || route === "/rover-control";
}

function routeSupportsCameraLive(route = currentRoute()) {
  return route === "/connection" || route === "/rover-control";
}

function routeSupportsImuLive(route = currentRoute()) {
  return route === "/connection" || route === "/rover-control";
}

function renderRoute() {
  const route = currentRoute();
  document.querySelectorAll("[data-page]").forEach((page) => {
    page.classList.toggle("active", page.dataset.page === route);
  });
  document.querySelectorAll("[data-route]").forEach((link) => {
    link.classList.toggle("active", link.dataset.route === route);
  });
  if (route === "/docs") renderDocsRoute();
  if (route === "/docker" && state.telemetry?.docker) {
    renderContainers(state.telemetry.docker);
    maybeRefreshContainerDetail(true);
  }
  if (route === "/connection") {
    renderActiveLidarMap();

    if (!state.lidarLive.socket && !state.lidarLive.userStopped) startLidarLive();
    if (!state.cameraLive.socket && !state.cameraLive.userStopped) startCameraLive();
	    if (!state.imuLive.socket && !state.imuLive.userStopped) startImuLive();
	  } else if (route === "/rover-control") {
	    renderActiveLidarMap();

	    renderRoverControl(state.telemetry || {});
	    renderRecordingStatus();
	    maybeRefreshRecordingStatus();
	    if (!state.recordings.items.length && !state.recordings.loading) loadRecordings().catch(console.error);
	    if (!state.lidarLive.socket && !state.lidarLive.userStopped) startLidarLive();
	    if (!state.cameraLive.socket && !state.cameraLive.userStopped) startCameraLive();
	    if (!state.imuLive.socket && !state.imuLive.userStopped) startImuLive();
	  } else if (state.lidarLive.socket || state.lidarLive.enabled) {
	    closeLidarLive("Live 화면 밖에서는 LiDAR WebSocket을 닫습니다.", { userStopped: false });
	  }
	  if (route !== "/rover-control" && state.archivePlayback.playing) {
	    stopArchivePlayback(false);
	  }
  if (!routeSupportsCameraLive(route) && (state.cameraLive.socket || state.cameraLive.enabled)) {
    closeCameraLive("Live 화면 밖에서는 Camera WebSocket을 닫습니다.", { userStopped: false });
  }
  if (!routeSupportsImuLive(route) && (state.imuLive.socket || state.imuLive.enabled)) {
    closeImuLive("Live 화면 밖에서는 IMU WebSocket을 닫습니다.", { userStopped: false });
  }
  window.RoverGps?.routeChanged();
  window.SimMap?.routeChanged();
  requestChartAnimation();
}

function initMobileNav() {
  const button = $("mobileMenuButton");
  const backdrop = $("mobileNavBackdrop");
  button?.addEventListener("click", () => setMobileNav(!document.body.classList.contains("nav-open")));
  backdrop?.addEventListener("click", () => setMobileNav(false));
  document.addEventListener("keydown", (event) => {
    if (event.key === "Escape") setMobileNav(false);
  });
  window.addEventListener("resize", () => {
    if (window.innerWidth > 1180) setMobileNav(false);
    if (routeSupportsLidarLive()) requestChartAnimation();
  });
}

function setMobileNav(open) {
  document.body.classList.toggle("nav-open", Boolean(open));
  const button = $("mobileMenuButton");
  if (button) {
    button.setAttribute("aria-expanded", String(Boolean(open)));
    button.setAttribute("aria-label", open ? "사이트맵 닫기" : "사이트맵 열기");
  }
}

function initTheme() {
  let stored = null;
  try {
    stored = localStorage.getItem("ubts-theme");
  } catch (_) {
    stored = null;
  }
  const theme = stored || "dark";
  applyTheme(theme);
  $("themeToggle")?.addEventListener("click", () => {
    const next = document.documentElement.dataset.theme === "light" ? "dark" : "light";
    applyTheme(next);
  });
}

function applyTheme(theme) {
  document.documentElement.dataset.theme = theme;
  try {
    localStorage.setItem("ubts-theme", theme);
  } catch (_) {
    // Ignore storage failures; theme switching should remain usable.
  }
  const isLight = theme === "light";
  const toggle = $("themeToggle");
  if (toggle) toggle.setAttribute("aria-pressed", String(isLight));
  text("themeLabel", isLight ? "Light" : "Dark");
  requestChartAnimation();
}

function initCpuMode() {
  try {
    state.cpuMode = localStorage.getItem(CPU_MODE_KEY) || "combined";
  } catch (_) {
    state.cpuMode = "combined";
  }
  document.querySelectorAll("[data-cpu-mode]").forEach((button) => {
    button.addEventListener("click", () => setCpuMode(button.dataset.cpuMode));
  });
  setCpuMode(state.cpuMode);
}

function setCpuMode(mode) {
  state.cpuMode = mode === "separate" ? "separate" : "combined";
  try {
    localStorage.setItem(CPU_MODE_KEY, state.cpuMode);
  } catch (_) {
    // Ignore storage failures; the visible toggle still works for this session.
  }
  document.querySelectorAll("[data-cpu-mode]").forEach((button) => {
    button.classList.toggle("active", button.dataset.cpuMode === state.cpuMode);
  });
  const combined = $("cpuCombinedView");
  const separate = $("cpuSeparateView");
  if (combined) combined.hidden = state.cpuMode !== "combined";
  if (separate) separate.hidden = state.cpuMode !== "separate";
  text("cpuModeText", state.cpuMode === "separate" ? "코어별 개별 그래프" : "전체 코어 통합 그래프");
  requestChartAnimation();
}

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
  if (state.cameraLive.settingsInFlight) return;
  const requested = camera.settings || camera.driver_requested || {};
  const resolution = requested.resolution || (requested.width && requested.height ? `${requested.width}x${requested.height}` : "");
  const fps = Number(requested.fps);
  const resolutions = cameraResolutionOptions();
  if (!resolutions.includes(resolution) || !Number.isFinite(fps)) return;
  const allowed = cameraFpsOptions(resolution);
  const nextFps = chooseCameraFps(allowed, fps);
  const changed = state.cameraLive.resolution !== resolution || Number(state.cameraLive.fps) !== Number(nextFps);
  state.cameraLive.resolution = resolution;
  state.cameraLive.fps = nextFps;
  if (changed) updateCameraControls();
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

async function setCameraSettings(next) {
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
  updateCameraControls();
  state.cameraLive.settingsError = "";
  state.cameraLive.settingsInFlight = true;
  setCameraResult("testing", `카메라 캡처 설정 적용 중 · ${state.cameraLive.resolution} · ${state.cameraLive.fps} fps`);
  try {
    const [width, height] = state.cameraLive.resolution.split("x").map((value) => Number(value));
    const res = await fetch("/api/camera/settings", {
      method: "POST",
      cache: "no-store",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        resolution: state.cameraLive.resolution,
        width,
        height,
        fps: state.cameraLive.fps,
      }),
    });
    const result = await res.json();
    if (!res.ok || !result.ok) {
      throw new Error(result.error || `HTTP ${res.status}`);
    }
    if (Array.isArray(result.supported_view_modes)) {
      applyCameraModeCatalog(result.supported_view_modes, "settings");
    }
    setCameraResult("testing", `캡처 설정 저장됨 · ${state.cameraLive.resolution} · ${state.cameraLive.fps} fps`);
    scheduleCameraReconnectForSettings(900);
  } catch (err) {
    state.cameraLive.settingsError = err.message || String(err);
    setCameraResult("bad", `카메라 설정 적용 실패: ${state.cameraLive.settingsError}`);
  } finally {
    state.cameraLive.settingsInFlight = false;
    updateCameraControls();
  }
}

function updateCameraControls() {
  text("cameraResolutionValue", state.cameraLive.resolution);
  text("cameraFpsValue", `${state.cameraLive.fps} fps`);
  document.querySelectorAll("[data-camera-resolution-value]").forEach((el) => {
    el.textContent = state.cameraLive.resolution;
  });
  document.querySelectorAll("[data-camera-fps-value]").forEach((el) => {
    el.textContent = `${state.cameraLive.fps} fps`;
  });
  document.querySelectorAll("[data-camera-resolution-select]").forEach((resolution) => {
    syncSelectOptions(resolution, cameraResolutionOptionLabels());
    if (document.activeElement !== resolution) resolution.value = state.cameraLive.resolution;
  });
  document.querySelectorAll("[data-camera-fps-select]").forEach((fps) => {
    syncSelectOptions(fps, cameraFpsOptions().map((value) => ({ value: String(value), label: `${value} fps` })));
    if (document.activeElement !== fps) fps.value = String(state.cameraLive.fps);
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


async function loadPlan() {
  state.plan = await getJson("/api/plan");
  renderPlan(state.plan);
}

async function loadDocs() {
  const payload = await getJson("/api/docs");
  state.docs = payload.documents || [];
  renderDocsIndex(state.docs);
  renderDocsRoute();
}

async function tick() {
  const interval = liveNetworkProfile() === "tiny" ? 10000 : ["auto", "slow"].includes(liveNetworkProfile()) ? 3000 : 1000;
  if (state.lastTelemetryPoll && Date.now() - state.lastTelemetryPoll < interval) return;
  state.lastTelemetryPoll = Date.now();
  if (state.tickInFlight) {
    debugLog("api", "/api/telemetry skipped: previous request still running");
    return;
  }
  state.tickInFlight = true;
  const controller = new AbortController();
  const timeout = window.setTimeout(() => controller.abort(), 6000);
  try {
    state.telemetry = await getJson("/api/telemetry", { signal: controller.signal });
    pushHistory(state.telemetry);
    renderTelemetry(state.telemetry);
    state.telemetryFailures = 0;
    setConnection("ok", "online");
    requestChartAnimation({ redrawLidar: false });
  } catch (err) {
    state.telemetryFailures += 1;
    if (state.telemetryFailures >= 3) setConnection("bad", "offline");
    else debugLog("api", `/api/telemetry transient failure ${state.telemetryFailures}`, err);
  } finally {
    window.clearTimeout(timeout);
    state.tickInFlight = false;
  }
}

function pushHistory(data) {
  const sys = data.system || {};
  const net = data.network || {};
  const docker = data.docker || {};
  const fan = data.fan || {};
  const cores = sys.cpu?.cores || [];
  const dockerCpu = docker.summary?.cpu_percent;
  const dockerMemory = docker.summary?.memory_limit ? (docker.summary.memory_usage / docker.summary.memory_limit) * 100 : null;
  addPoint("cpu", sys.cpu?.total_percent);
  addPoint("memory", memoryPercent(sys.memory));
  addPoint("swap", sys.swap?.percent);
  addPoint("power", sys.power?.total_w);
  addPoint("temp", sys.temperature_c);
  addPoint("rx", net.total_rx_bps);
  addPoint("tx", net.total_tx_bps);
  addPoint("dockerCpu", dockerCpu);
  addPoint("dockerMemory", dockerMemory);
  addPoint("fanTemp", fan.temperature_c);
  addPoint("fanPwm", fan.pwm);
  for (const core of cores) addCorePoint(coreKey(core), core.percent);
  pruneCoreHistory(cores.map(coreKey));
  state.chartShift = 1;
  state.chartShiftStartedAt = performance.now();
  state.chartShiftDurationMs = Math.max(500, POLL_INTERVAL_MS * 1.08);
}

function addPoint(key, value) {
  addPointTo(state.history, state.visualHistory, key, value);
}

function addCorePoint(key, value) {
  addPointTo(state.coreHistory, state.visualCoreHistory, key, value);
}

function addPointTo(historyStore, visualStore, key, value) {
  const target = historyStore[key] || [];
  const visual = visualStore[key] || [];
  historyStore[key] = target;
  visualStore[key] = visual;
  const numeric = value === null || value === undefined ? null : Number(value);
  if (target.length >= CHART_BUFFER_POINTS) {
    target.shift();
    visual.shift();
  }
  target.push(Number.isFinite(numeric) ? numeric : null);
  visual.push(Number.isFinite(numeric) ? numeric : null);
}

function pruneCoreHistory(keys) {
  const alive = new Set(keys);
  for (const key of Object.keys(state.coreHistory)) {
    if (!alive.has(key)) {
      delete state.coreHistory[key];
      delete state.visualCoreHistory[key];
    }
  }
}

function setConnection(kind, label) {
  const el = $("connectionState");
  if (!el) return;
  el.className = `status-pill ${kind}`;
  el.textContent = label;
}

function renderTelemetry(data) {
  const sys = data.system || {};
  const net = data.network || {};
  const docker = data.docker || {};
  const fan = data.fan || {};
  const services = data.services || {};
  const connection = data.connection || {};

  text("updatedAt", new Date((data.meta?.generated_at || Date.now() / 1000) * 1000).toLocaleString("ko-KR"));
  text("overviewCpu", fmtPercent(sys.cpu?.total_percent));
  text("overviewLoad", `load ${(sys.loadavg || [0])[0]?.toFixed ? sys.loadavg[0].toFixed(2) : "--"}`);
  text("overviewMemory", fmtPercent(memoryPercent(sys.memory)));
  text("overviewMemoryText", `${fmtBytes(sys.memory?.used)} / ${fmtBytes(memoryTotal(sys.memory))} · ${memoryLabel(sys.memory)}`);
  text("overviewNetwork", net.external_ip || "확인 중");
  text("overviewNetworkText", `host ${net.primary_ipv4 || "--"} · RX ${fmtBps(net.total_rx_bps)} · TX ${fmtBps(net.total_tx_bps)}`);
  text("overviewDocker", docker.available ? `${docker.summary?.running || 0}/${docker.summary?.total || 0}` : "N/A");
  text("overviewDockerText", docker.available ? "running / total" : docker.error || "socket unavailable");
  text("hostName", sys.hostname || "--");
  text("hostUptime", fmtDuration(sys.os_uptime_s));
  text("hostTemp", fmtTemp(sys.temperature_c));
  text("hostPower", fmtPower(sys.power?.total_w));
  text("hostExternalIp", net.external_ip || net.external_ip_error || "--");
  text("hostInternalIp", hostIpLabel(net));

  text("systemCpuValue", fmtPercent(sys.cpu?.total_percent));
  text("systemMemoryValue", fmtPercent(memoryPercent(sys.memory)));
  text("systemSwapValue", fmtPercent(sys.swap?.percent));
  text("systemNetworkValue", `${fmtBps(net.total_rx_bps)} / ${fmtBps(net.total_tx_bps)}`);
  text("systemPowerValue", fmtPower(sys.power?.total_w));
  setProgress("memoryBar", memoryPercent(sys.memory));
  text("memoryDetail", `${fmtBytes(sys.memory?.used)} used / ${fmtBytes(memoryTotal(sys.memory))} · ${memoryLabel(sys.memory)} · ${fmtBytes(sys.memory?.available)} available`);

  text("networkExternalIp", net.external_ip || "--");
  text("networkExternalIpText", net.external_ip ? "public endpoint" : "확인 대기");
  text("networkInternalIp", net.primary_ipv4 || "--");
  text("networkInternalIpText", [net.primary_interface, net.hostname].filter(Boolean).join(" · ") || "host route");
  text("networkThroughput", `${fmtBps(net.total_rx_bps)} RX · ${fmtBps(net.total_tx_bps)} TX`);
  renderNetworkMode(net.control || {});

  text("dockerContainerCount", docker.available ? docker.summary?.total ?? "--" : "N/A");
  text("dockerRunningText", docker.available ? `${docker.summary?.running || 0} running · ${docker.summary?.stopped || 0} stopped` : docker.error || "--");
  text("dockerImageCount", docker.available ? docker.engine?.images ?? "--" : "N/A");
  text("dockerVersion", docker.available ? `Docker ${docker.engine?.server_version || "--"}` : "--");
  text("dockerCpuTotal", fmtPercent(docker.summary?.cpu_percent));
  text("dockerCpuChartValue", fmtPercent(docker.summary?.cpu_percent));
  const dockerMemPercent = docker.summary?.memory_limit ? (docker.summary.memory_usage / docker.summary.memory_limit) * 100 : null;
  text("dockerMemTotal", fmtPercent(dockerMemPercent));
  text("dockerMemChartValue", fmtPercent(dockerMemPercent));
  text("dockerMemText", `${fmtBytes(docker.summary?.memory_usage)} / ${fmtBytes(docker.summary?.memory_limit)}`);

  text("roverState", data.rover?.state || "연결 전");
  text("experimentState", data.experiment?.state || "연결 전");
  renderFan(fan);

  renderServices(services);
  renderConnection(connection);
  renderRoverControl(data);
  renderSensors(data.sensors || {});
  renderCores(sys.cpu?.cores || []);
  renderCoreCharts(sys.cpu?.cores || []);
  renderTemperatures(sys.temperatures || []);
  renderDisks(sys.disks || []);
  renderInterfaces(net.interfaces || [], net.primary_interface);
  renderContainers(docker);
  maybeRefreshContainerDetail();
  renderChartReadouts(data);
}

function networkModeLabel(control) {
  if (control.ethernet_connected && control.ap_active) return "Ethernet + AP";
  if (control.transition === "wifi_scan") return "Wi-Fi 검색 중";
  if (control.transition === "ap_start") return "AP 전환 중";
  if (control.ap_active) return "AP";
  if (control.wifi_connected) return "Wi-Fi";
  return control.enabled === false ? "비활성화" : control.state || "연결 없음";
}

function renderNetworkMode(control) {
  const mode = networkModeLabel(control);
  text("networkModeTag", mode);
  const tag = $("networkModeTag");
  if (tag) tag.className = `tag ${control.ok ? "ok" : ""}`.trim();
  const countdownValue = control.seconds_until_auto_switch;
  const countdown = countdownValue == null ? Number.NaN : Number(countdownValue);
  const countdownText = Number.isFinite(countdown)
    ? `${Math.ceil(countdown)}초 후 자동 전환`
    : control.controls_locked_reason === "ethernet_keeps_ap_active"
      ? "Ethernet 연결로 AP 상시 유지"
      : "자동 전환 대기";
  const colors = chartColors();
  renderReadout("networkModeReadout", [
    { label: "Mode", value: mode, sub: control.connection || control.transition || "--", color: colors.cyan },
    { label: "AP", value: control.ap_ssid || "BTS_Jetson", sub: control.ap_address || "10.77.0.1", color: colors.blue },
    { label: "AP clients", value: control.ap_clients ?? 0, sub: countdownText, color: colors.good },
    { label: "Ethernet", value: control.ethernet_connected ? "connected" : "not connected", sub: control.last_error || control.reason || "--", color: control.ethernet_connected ? colors.good : colors.amber },
  ]);
  const apButton = $("networkEnableApButton");
  const wifiButton = $("networkScanWifiButton");
  if (apButton) apButton.disabled = state.networkControl.actionInFlight || !control.can_request_ap;
  if (wifiButton) wifiButton.disabled = state.networkControl.actionInFlight || !control.can_request_wifi;
  const result = $("networkModeResult");
  if (result) {
    result.className = `test-result ${state.networkControl.level}`.trim();
    result.textContent = control.last_error || state.networkControl.message;
  }
}

async function requestNetworkMode(action) {
  if (state.networkControl.actionInFlight) return;
  const isAp = action === "ap";
  const warning = isAp
    ? "Jetson의 현재 Wi-Fi 인터넷을 끊고 BTS_Jetson AP로 전환합니다. 계속할까요?"
    : "BTS_Jetson AP를 끄고 저장된 Wi-Fi를 최대 1분간 검색합니다. 계속할까요?";
  if (!window.confirm(warning)) return;
  state.networkControl.actionInFlight = true;
  state.networkControl.level = "testing";
  state.networkControl.message = isAp ? "AP 전환 요청을 전달했습니다." : "저장된 Wi-Fi 검색 요청을 전달했습니다.";
  renderNetworkMode(state.telemetry?.network?.control || {});
  try {
    const result = await postJson("/api/network/mode", {
      action,
      confirm: isAp ? "disconnect_wifi_and_enable_ap" : "leave_ap_and_scan_known_wifi",
    });
    state.networkControl.level = result.ok ? "ok" : "bad";
    state.networkControl.message = result.ok
      ? "요청이 승인되었습니다. 전환 중에는 이 페이지 연결이 잠시 끊길 수 있습니다."
      : result.error || "네트워크 전환 요청이 거부되었습니다.";
  } catch (err) {
    state.networkControl.level = "testing";
    state.networkControl.message = "연결이 종료되었습니다. AP/Wi-Fi 전환 상태를 다시 확인하세요.";
  } finally {
    state.networkControl.actionInFlight = false;
    renderNetworkMode(state.telemetry?.network?.control || {});
  }
}

function initNetworkControls() {
  $("networkEnableApButton")?.addEventListener("click", () => requestNetworkMode("ap"));
  $("networkScanWifiButton")?.addEventListener("click", () => requestNetworkMode("wifi"));
}

async function runLidarTest() {
  const button = $("lidarTestButton");
  if (button) button.disabled = true;
  setTestResult("testing", "LiDAR serial port를 열지 않고 장치 표시와 ROS2 소유 상태만 확인합니다.");
  try {
    const res = await fetch("/api/lidar/test", { method: "POST", cache: "no-store" });
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    const payload = await res.json();
    renderLidarTest(payload);
    await tick();
  } catch (err) {
    setTestResult("bad", `상태 확인 실패: ${err.message}`);
  } finally {
    if (button) button.disabled = false;
  }
}

function toggleLidarLive() {
  if (state.lidarLive.connected || state.lidarLive.socket) {
    stopLidarLive("사용자가 실시간 연결을 중지했습니다.");
    return;
  }
  startLidarLive();
}

function startLidarLive() {
  if (!routeSupportsLidarLive()) {
    debugLog("ws", "skip start outside live routes");
    return;
  }
  if (state.lidarLive.socket) return;
  clearLidarReconnectTimer();
  const url = lidarWebSocketUrl();
  state.lidarLive.enabled = true;
  state.lidarLive.userStopped = false;
  state.lidarLive.error = "";
  state.lidarLive.connected = false;
  state.lidarLive.scanTopicAlive = false;
  state.lidarLive.source = "waiting";
  state.lidarLive.requestedPointCount = state.lidarPoints.value;
  renderLidarLiveState("연결 중");
  setScanResult("testing", "LiDAR WebSocket 연결 중");
  debugLog("ws", `connecting ${url}`);
  try {
    const socket = new WebSocket(url);
    state.lidarLive.socket = socket;
    socket.addEventListener("open", () => {
      if (socket !== state.lidarLive.socket) return;
      state.lidarLive.connected = true;
      state.lidarLive.error = "";
      debugLog("ws", "open");
      renderLidarLiveState();
      if (state.lidarLive.pendingRestart) {
        state.lidarLive.pendingRestart = false;
        scheduleLidarReconnectForSettings(80);
      }
    });
    socket.addEventListener("message", (event) => {
      if (socket !== state.lidarLive.socket || !routeSupportsLidarLive()) return;
      handleLidarLiveMessage(event.data);
    });
    socket.addEventListener("error", (event) => {
      if (socket !== state.lidarLive.socket) return;
      state.lidarLive.error = "WebSocket 연결 오류";
      debugLog("ws", "error", event);
      renderLidarLiveState();
    });
    socket.addEventListener("close", (event) => {
      if (socket !== state.lidarLive.socket) return;
      const shouldReconnect = state.lidarLive.enabled && !state.lidarLive.userStopped && routeSupportsLidarLive();
      state.lidarLive.socket = null;
      state.lidarLive.connected = false;
      state.lidarLive.scanTopicAlive = false;
      debugLog("ws", `close code=${event.code} reason=${event.reason || "-"}`);
      renderLidarLiveState(state.lidarLive.error || "disconnected");
      if (state.lidarLive.pendingRestart && shouldReconnect) {
        state.lidarLive.pendingRestart = false;
        scheduleLidarReconnectForSettings(120);
      } else if (shouldReconnect) {
        state.lidarLive.reconnectTimer = window.setTimeout(() => {
          state.lidarLive.reconnectTimer = null;
          if (routeSupportsLidarLive() && !state.lidarLive.socket && !state.lidarLive.userStopped) startLidarLive();
        }, 1500);
      }
    });
  } catch (err) {
    state.lidarLive.socket = null;
    state.lidarLive.connected = false;
    state.lidarLive.scanTopicAlive = false;
    state.lidarLive.source = "waiting";
    state.lidarLive.error = err.message;
    debugLog("ws", "create failed", err);
    renderLidarLiveState();
  }
}

function stopLidarLive(message = "disconnected") {
  closeLidarLive(message, { userStopped: true });
}

function closeLidarLive(message = "disconnected", options = {}) {
  const userStopped = options.userStopped !== undefined ? Boolean(options.userStopped) : true;
  if (options.clearReconnect !== false) clearLidarReconnectTimer();
  const socket = state.lidarLive.socket;
  state.lidarLive.enabled = false;
  state.lidarLive.userStopped = userStopped;
  state.lidarLive.socket = null;
  state.lidarLive.connected = false;
  state.lidarLive.scanTopicAlive = false;
  state.lidarLive.source = "waiting";
  state.lidarLive.error = "";
  state.lidarLive.pendingRestart = false;
  if (socket) {
    closeSocketAfterConnect(socket, message);
  }
  debugLog("ws", `stopped: ${message}`);
  renderLidarLiveState(message);
}

function clearLidarReconnectTimer() {
  if (!state.lidarLive.reconnectTimer) return;
  window.clearTimeout(state.lidarLive.reconnectTimer);
  state.lidarLive.reconnectTimer = null;
}

function closeSocketAfterConnect(socket, reason = "closed") {
  const closeReason = String(reason || "closed").slice(0, 120);
  try {
    if (socket.readyState === WebSocket.CONNECTING) {
      socket.addEventListener("open", () => socket.close(1000, closeReason), { once: true });
    } else if (socket.readyState === WebSocket.OPEN) {
      socket.close(1000, closeReason);
    } else if (socket.readyState === WebSocket.CLOSING || socket.readyState === WebSocket.CLOSED) {
      return;
    }
  } catch (err) {
    debugLog("ws", "socket close skipped", err);
  }
}

function scheduleLidarReconnectForSettings(delayMs = LIDAR_RECONNECT_DEBOUNCE_MS) {
  if (!routeSupportsLidarLive()) return;
  if (!state.lidarLive.socket && !state.lidarLive.enabled) return;
  clearLidarReconnectTimer();
  state.lidarLive.pendingRestart = true;
  state.lidarLive.reconnectTimer = window.setTimeout(() => {
    state.lidarLive.reconnectTimer = null;
    restartLidarLiveForSettings();
  }, Math.max(0, delayMs));
  debugLog("ws", `settings restart scheduled max_points=${state.lidarPoints.value}`);
}

function restartLidarLiveForSettings() {
  if (!routeSupportsLidarLive()) return;
  const wasEnabled = state.lidarLive.enabled && !state.lidarLive.userStopped;
  const socket = state.lidarLive.socket;
  if (socket?.readyState === WebSocket.CONNECTING) {
    state.lidarLive.pendingRestart = true;
    debugLog("ws", "settings restart deferred until socket opens");
    return;
  }
  clearLidarReconnectTimer();
  state.lidarLive.pendingRestart = false;
  if (socket) {
    state.lidarLive.socket = null;
    state.lidarLive.connected = false;
    state.lidarLive.scanTopicAlive = false;
    closeSocketAfterConnect(socket, "point limit changed");
  }
  if (wasEnabled) {
    window.setTimeout(() => {
      if (routeSupportsLidarLive() && !state.lidarLive.socket && !state.lidarLive.userStopped) startLidarLive();
    }, 160);
  }
}

function toggleCameraLive() {
  // An explicit camera choice cancels automatic restoration after playback.
  state.archivePlayback.resumeLiveCamera = false;
  if (state.cameraLive.connected || state.cameraLive.socket) {
    stopCameraLive("사용자가 카메라 실시간 연결을 중지했습니다.");
    return;
  }
  startCameraLive();
}

function liveNetworkProfile() {
  try { return localStorage.getItem("ubts-live-network-profile") || (SIMULATION ? "detail" : "auto"); }
  catch (_) { return SIMULATION ? "detail" : "auto"; }
}
function setLiveNetworkProfile(value) {
  if (!["auto", "tiny", "slow", "balanced", "detail"].includes(value)) return;
  try { localStorage.setItem("ubts-live-network-profile", value); } catch (_) {}
  window.RoverGps?.profileChanged();
  if (state.imuLive.socket) { closeImuLive("profile changed", {userStopped: false}); setTimeout(startImuLive, 250); }
  document.querySelectorAll("[data-network-profile]").forEach(el => { el.value = value; });
  if (state.cameraLive.socket) { closeCameraLive("전송 품질 변경", {userStopped: false}); setTimeout(startCameraLive, 250); }
  if (state.lidarLive.socket) { closeLidarLive("전송 품질 변경", {userStopped: false}); setTimeout(startLidarLive, 250); }
}

function cameraWebSocketUrl() {
  const params = new URLSearchParams({
    resolution: state.cameraLive.resolution,
    fps: String(state.cameraLive.fps),
    quality: "70",
    profile: liveNetworkProfile(),
    transport: "binary-v1",
    flow: "decode-ack",
  });
  if (window.UBTS_CAMERA_WS_URL) {
    const separator = String(window.UBTS_CAMERA_WS_URL).includes("?") ? "&" : "?";
    return `${window.UBTS_CAMERA_WS_URL}${separator}${params.toString()}`;
  }
  const scheme = location.protocol === "https:" ? "wss" : "ws";
  return `${scheme}://${location.host}/ws/camera?${params.toString()}`;
}

function startCameraLive() {
  if (!routeSupportsCameraLive()) {
    debugLog("camera", "skip start outside live routes");
    return;
  }
  if (state.cameraLive.socket) return;
  clearCameraReconnectTimer();
  const url = cameraWebSocketUrl();
  state.cameraLive.enabled = true;
  state.cameraLive.userStopped = false;
  state.cameraLive.connected = false;
  state.cameraLive.frameTopicAlive = false;
  state.cameraLive.error = "";
  renderCameraLiveState("연결 중");
  setCameraResult("testing", "Camera WebSocket 연결 중");
  debugLog("camera", `connecting ${url}`);
  try {
    const socket = new WebSocket(url);
    socket.binaryType = "arraybuffer";
    state.cameraLive.socket = socket;
    socket.addEventListener("open", () => {
      if (socket !== state.cameraLive.socket) return;
      state.cameraLive.connected = true;
      state.cameraLive.error = "";
      debugLog("camera", "open");
      renderCameraLiveState();
      if (state.cameraLive.pendingRestart) {
        state.cameraLive.pendingRestart = false;
        scheduleCameraReconnectForSettings(80);
      }
    });
    socket.addEventListener("message", (event) => {
      if (socket !== state.cameraLive.socket || !routeSupportsCameraLive()) return;
      if (typeof event.data === "string") {
        handleCameraLiveMessage(event.data);
      } else {
        handleCameraLiveBinaryMessage(event.data, socket).catch((err) => {
          state.cameraLive.error = err?.message || "잘못된 Camera binary frame";
          debugLog("camera", "binary frame error", err);
          renderCameraLiveState();
        });
      }
    });
    socket.addEventListener("error", (event) => {
      if (socket !== state.cameraLive.socket) return;
      state.cameraLive.error = "Camera WebSocket 연결 오류";
      debugLog("camera", "error", event);
      renderCameraLiveState();
    });
    socket.addEventListener("close", (event) => {
      if (socket !== state.cameraLive.socket) return;
      const exhausted = event.code === 1013 && event.reason === "preview acknowledgement timeout";
      if (exhausted) {
        state.cameraLive.enabled = false;
        state.cameraLive.userStopped = true;
        state.cameraLive.deliveryState = "bandwidth_limited";
        state.cameraLive.error = "영상 응답 지연으로 일시 정지했습니다. 회선 상태나 전송 품질을 확인한 뒤 다시 시작하세요.";
      }
      const shouldReconnect = state.cameraLive.enabled && !state.cameraLive.userStopped && routeSupportsCameraLive();
      state.cameraLive.socket = null;
      state.cameraLive.connected = false;
      state.cameraLive.frameTopicAlive = false;
      debugLog("camera", `close code=${event.code} reason=${event.reason || "-"}`);
      renderCameraLiveState(state.cameraLive.error || "disconnected");
      if (state.cameraLive.pendingRestart && shouldReconnect) {
        state.cameraLive.pendingRestart = false;
        scheduleCameraReconnectForSettings(120);
      } else if (shouldReconnect) {
        state.cameraLive.reconnectTimer = window.setTimeout(() => {
          state.cameraLive.reconnectTimer = null;
          if (routeSupportsCameraLive() && !state.cameraLive.socket && !state.cameraLive.userStopped) startCameraLive();
        }, 1500);
      }
    });
  } catch (err) {
    state.cameraLive.socket = null;
    state.cameraLive.connected = false;
    state.cameraLive.frameTopicAlive = false;
    state.cameraLive.error = err.message;
    debugLog("camera", "create failed", err);
    renderCameraLiveState();
  }
}

function stopCameraLive(message = "disconnected") {
  closeCameraLive(message, { userStopped: true });
}

function closeCameraLive(message = "disconnected", options = {}) {
  const userStopped = options.userStopped !== undefined ? Boolean(options.userStopped) : true;
  if (options.clearReconnect !== false) clearCameraReconnectTimer();
  const socket = state.cameraLive.socket;
  state.cameraLive.enabled = false;
  state.cameraLive.userStopped = userStopped;
  state.cameraLive.socket = null;
  state.cameraLive.connected = false;
  state.cameraLive.frameTopicAlive = false;
  state.cameraLive.error = "";
  state.cameraLive.pendingRestart = false;
  if (socket) closeSocketAfterConnect(socket, message);
  if (state.cameraLive.objectUrl) {
    URL.revokeObjectURL(state.cameraLive.objectUrl);
    state.cameraLive.objectUrl = "";
  }
  debugLog("camera", `stopped: ${message}`);
  renderCameraLiveState(message);
}

function clearCameraReconnectTimer() {
  if (!state.cameraLive.reconnectTimer) return;
  window.clearTimeout(state.cameraLive.reconnectTimer);
  state.cameraLive.reconnectTimer = null;
}

function scheduleCameraReconnectForSettings(delayMs = CAMERA_RECONNECT_DEBOUNCE_MS) {
  if (!routeSupportsCameraLive()) return;
  if (!state.cameraLive.socket && !state.cameraLive.enabled) return;
  clearCameraReconnectTimer();
  state.cameraLive.pendingRestart = true;
  state.cameraLive.reconnectTimer = window.setTimeout(() => {
    state.cameraLive.reconnectTimer = null;
    restartCameraLiveForSettings();
  }, Math.max(0, delayMs));
  debugLog("camera", `settings restart scheduled ${state.cameraLive.resolution}@${state.cameraLive.fps}`);
}

function restartCameraLiveForSettings() {
  if (!routeSupportsCameraLive()) return;
  const wasEnabled = state.cameraLive.enabled && !state.cameraLive.userStopped;
  const socket = state.cameraLive.socket;
  if (socket?.readyState === WebSocket.CONNECTING) {
    state.cameraLive.pendingRestart = true;
    return;
  }
  clearCameraReconnectTimer();
  state.cameraLive.pendingRestart = false;
  if (socket) {
    state.cameraLive.socket = null;
    state.cameraLive.connected = false;
    closeSocketAfterConnect(socket, "camera settings changed");
  }
  if (wasEnabled) {
    window.setTimeout(() => {
      if (routeSupportsCameraLive() && !state.cameraLive.socket && !state.cameraLive.userStopped) startCameraLive();
    }, 160);
  }
}

function noteCameraFrame(payload) {
  const now = Date.now();
  if (!state.cameraLive.fpsWindowStartedAt) {
    state.cameraLive.fpsWindowStartedAt = now;
    state.cameraLive.fpsWindowFrames = 0;
  }
  state.cameraLive.fpsWindowFrames += 1;
  const elapsed = (now - state.cameraLive.fpsWindowStartedAt) / 1000;
  if (elapsed >= 0.8) {
    state.cameraLive.displayFps = state.cameraLive.fpsWindowFrames / elapsed;
    state.cameraLive.fpsWindowStartedAt = now;
    state.cameraLive.fpsWindowFrames = 0;
  }
  const sourceFps = Number(payload.source_fps);
  if (Number.isFinite(sourceFps) && sourceFps > 0) {
    state.cameraLive.sourceFps = sourceFps;
  }
}

function cameraFpsLabel(value) {
  const fps = Number(value);
  if (!Number.isFinite(fps) || fps <= 0) return "--";
  return `${fmtNumber(fps, fps >= 10 ? 0 : 1)} fps`;
}

function handleCameraLiveMessage(raw) {
  let payload = null;
  try {
    payload = JSON.parse(raw);
  } catch (err) {
    state.cameraLive.error = "잘못된 Camera WebSocket 메시지";
    debugLog("camera", "invalid message", raw);
    renderCameraLiveState();
    return;
  }
  handleCameraLivePayload(payload);
}

async function handleCameraLiveBinaryMessage(raw, socket) {
  const buffer = raw instanceof ArrayBuffer ? raw : await raw.arrayBuffer();
  if (buffer.byteLength <= CAMERA_BINARY_HEADER_SIZE) throw new Error("camera binary frame is truncated");
  const bytes = new Uint8Array(buffer);
  const magic = String.fromCharCode(...bytes.slice(0, 4));
  const version = bytes[4];
  if (magic !== CAMERA_BINARY_MAGIC || version !== 1) throw new Error("camera binary protocol mismatch");
  const metadataLength = new DataView(buffer).getUint32(5, false);
  const jpegOffset = CAMERA_BINARY_HEADER_SIZE + metadataLength;
  if (metadataLength <= 0 || jpegOffset >= buffer.byteLength) throw new Error("camera binary metadata is invalid");
  const metadata = JSON.parse(new TextDecoder().decode(bytes.slice(CAMERA_BINARY_HEADER_SIZE, jpegOffset)));
  const blob = new Blob([bytes.slice(jpegOffset)], { type: metadata.mime || "image/jpeg" });
  handleCameraLivePayload(metadata, { blob, socket });
}

function handleCameraLivePayload(payload, binary = null) {
  if (payload.type === "status" || payload.type === "frame") {
    state.cameraLive.deliveryState = payload.delivery_state || state.cameraLive.deliveryState;
    state.cameraLive.effectiveProfile = payload.effective_profile || state.cameraLive.effectiveProfile;
    state.cameraLive.previewQuality = payload.preview_quality ?? state.cameraLive.previewQuality;
    state.cameraLive.ackLatencyMs = payload.ack_latency_ms ?? state.cameraLive.ackLatencyMs;
  }
  if (payload.type === "status") {
    state.cameraLive.frameTopicAlive = Boolean(payload.frame_topic_alive);
    state.cameraLive.lastFrameAgeSec = payload.last_frame_age_sec;
    state.cameraLive.frameCount = payload.frame_count || state.cameraLive.frameCount || 0;
    if (Number.isFinite(Number(payload.source_fps))) state.cameraLive.sourceFps = Number(payload.source_fps);
    state.cameraLive.sourceWidth = payload.source_width;
    state.cameraLive.sourceHeight = payload.source_height;
    state.cameraLive.frameId = payload.frame_id || state.cameraLive.frameId;
    state.cameraLive.transport = payload.transport || state.cameraLive.transport;
    state.cameraLive.flow = payload.flow || state.cameraLive.flow;
    state.cameraLive.error = payload.error || (payload.frame_topic_alive === false ? "ROS2 /camera/image_raw 대기" : "");
    if (payload.error) debugLog("camera", "status", payload);
    if (!payload.frame_topic_alive) setCameraResult("testing", payload.error || "ROS2 카메라 프레임 수신 대기");
    renderCameraLiveState();
    renderLiveSensorLinks();
    return;
  }
  if (payload.type === "frame") {
    state.cameraLive.frameTopicAlive = true;
    state.cameraLive.lastFrameAgeSec = 0;
    state.cameraLive.frameCount = payload.frame_count || state.cameraLive.frameCount + 1;
    noteCameraFrame(payload);
    state.cameraLive.sourceWidth = payload.source_width;
    state.cameraLive.sourceHeight = payload.source_height;
    state.cameraLive.frameId = payload.frame_id || state.cameraLive.frameId;
    state.cameraLive.lastFrameAt = Date.now();
    state.cameraLive.lastFrameSize = {
      width: Number(payload.width || payload.source_width || 0),
      height: Number(payload.height || payload.source_height || 0),
    };
    state.cameraLive.transport = payload.transport || (binary ? "binary-v1" : "json-base64");
    state.cameraLive.flow = payload.flow || state.cameraLive.flow;
    state.cameraLive.encodedBytes = Number(payload.encoded_bytes || binary?.blob?.size || 0);
    state.cameraLive.encodeMs = Number.isFinite(Number(payload.encode_ms)) ? Number(payload.encode_ms) : null;
    state.cameraLive.error = "";
    if (binary?.blob) {
      displayBinaryCameraFrame(binary.blob, payload, binary.socket);
    } else {
      const src = payload.data ? `data:${payload.mime || "image/jpeg"};base64,${payload.data}` : "";
      const image = $("cameraLiveImage");
      const roverImage = $("roverCameraImage");
      if (image && src) image.src = src;
      if (roverImage && src) roverImage.src = src;
    }
    // Keep video delivery independent of readout DOM work (up to 30+ frames/s).
    const now = performance.now();
    if (now - (state.cameraLive.lastReadoutAt || 0) < 250) return;
    state.cameraLive.lastReadoutAt = now;
    const source = payload.source_width && payload.source_height ? `${payload.source_width}x${payload.source_height}` : "--";
    const output = payload.width && payload.height ? `${payload.width}x${payload.height}` : state.cameraLive.resolution;
    const rate = state.cameraLive.displayFps;
    const modeText = source === output ? source : `source ${source} · output ${output}`;
      setCameraResult("ok", `실시간 수신 중 · ${modeText} · ${cameraFpsLabel(rate)}`);
    renderCameraLiveState();

    renderLiveSensorLinks();
    return;
  }
  if (payload.type === "socket") {
    state.cameraLive.error = payload.ok ? "" : payload.message || "socket status error";
    state.cameraLive.transport = payload.transport || state.cameraLive.transport;
    state.cameraLive.flow = payload.flow || state.cameraLive.flow;
    debugLog("camera", "socket", payload);
    renderCameraLiveState();
  }
}

function displayBinaryCameraFrame(blob, payload, socket) {
  // Decode only the visible route; the hidden second image used to double decode work.
  const targets = [$(currentRoute() === "/rover-control" ? "roverCameraImage" : "cameraLiveImage")].filter(Boolean);
  const url = URL.createObjectURL(blob);
  const previousUrl = state.cameraLive.objectUrl;
  state.cameraLive.objectUrl = url;
  if (!targets.length) {
    URL.revokeObjectURL(url);
    state.cameraLive.objectUrl = previousUrl;
    acknowledgeCameraFrame(socket, payload.frame_count, "not_visible");
    return;
  }
  let settled = false;
  let timer = null;
  const finish = (status) => {
    if (settled) return;
    settled = true;
    if (timer !== null) window.clearTimeout(timer);
    if (socket !== state.cameraLive.socket || state.cameraLive.objectUrl !== url) {
      URL.revokeObjectURL(url);
      return;
    }
    for (const image of targets) { image.onload = null; image.onerror = null; }
    const statusChanged = state.cameraLive.decodeStatus !== status;
    state.cameraLive.decodeStatus = status;
    if (status === "decoded") {
      state.cameraLive.lastDecodedAt = performance.now();
      if (previousUrl) URL.revokeObjectURL(previousUrl);
    } else {
      // Keep the last usable image; an error/timeout is not a rendered frame.
      for (const image of targets) image.src = previousUrl || "";
      state.cameraLive.objectUrl = previousUrl;
      URL.revokeObjectURL(url);
    }
    acknowledgeCameraFrame(socket, payload.frame_count, status);
    if (statusChanged) renderCameraLiveState();
  };
  for (const image of targets) {
    image.onload = () => {
      if (image.currentSrc && image.currentSrc !== url) return;
      finish("decoded");
    };
    image.onerror = () => finish("error");
    image.src = url;
  }
  timer = window.setTimeout(() => finish("timeout"), 500);
}

function acknowledgeCameraFrame(socket, frameCount, decodeStatus = "unknown") {
  if (socket !== state.cameraLive.socket || socket?.readyState !== WebSocket.OPEN) return;
  try {
    socket.send(JSON.stringify({ type: "frame_ack", frame_count: Number(frameCount) || 0, decode_status: decodeStatus }));
  } catch (err) {
    debugLog("camera", "frame ack failed", err);
  }
}

function renderCameraLiveState(overrideLabel) {
  const button = $("cameraLiveButton");
  if (button) {
    button.textContent = state.cameraLive.socket && !state.cameraLive.connected
      ? "연결 중"
      : state.cameraLive.connected ? "실시간 중지" : "실시간 연결";
    button.classList.toggle("active", state.cameraLive.connected);
  }
  const source = state.cameraLive.sourceWidth && state.cameraLive.sourceHeight
    ? `${state.cameraLive.sourceWidth}x${state.cameraLive.sourceHeight}`
    : "--";
  const socketText = overrideLabel || (state.cameraLive.connected ? "connected" : state.cameraLive.error || "disconnected");
  text("cameraFrameCaption", state.cameraLive.frameTopicAlive ? `${state.cameraLive.frameId} · source ${source}` : socketText);
  const displayedAge = state.cameraLive.lastDecodedAt === null ? null : Math.max(0, (performance.now()-state.cameraLive.lastDecodedAt)/1000);
  const frameAge = displayedAge === null ? "첫 영상 대기" : `화면 갱신 ${fmtNumber(displayedAge, 1)}초 전`;
  const captureTarget = `${state.cameraLive.resolution} · ${state.cameraLive.fps} fps target`;
  const observedFps = state.cameraLive.displayFps;
  const cameraDelivery = state.cameraLive.deliveryState === "bandwidth_limited" ? "회선 제한 · 화질 유지, FPS 조절"
    : state.cameraLive.decodeStatus === "timeout" ? "영상 해독 지연"
    : state.cameraLive.decodeStatus === "error" ? "영상 해독 실패 · 이전 화면 유지"
    : `${state.cameraLive.effectiveProfile || "auto"} · Q${state.cameraLive.previewQuality ?? "--"}`;
  const ackLabel = Number.isFinite(state.cameraLive.ackLatencyMs) ? ` · ACK ${Math.round(state.cameraLive.ackLatencyMs)}ms` : "";
  const observedDetail = `ROS ${cameraFpsLabel(state.cameraLive.sourceFps)} · WS ${cameraFpsLabel(state.cameraLive.displayFps)} · ${cameraDelivery}${ackLabel}`;
  const colors = chartColors();
  renderReadout("cameraLiveStats", [
    { label: "Socket", value: state.cameraLive.connected ? "ON" : "OFF", sub: socketText, color: state.cameraLive.connected ? colors.good : colors.rose },
    { label: "Capture", value: state.cameraLive.resolution, sub: `${state.cameraLive.fps} fps target`, color: colors.amber },
    { label: "Source", value: source, sub: state.cameraLive.frameId || "camera_link_optical", color: colors.cyan },
    { label: "수신 FPS", value: cameraFpsLabel(observedFps), sub: `${observedDetail} · ${frameAge}`, color: colors.blue },
  ]);
  text("roverCameraCaption", state.cameraLive.frameTopicAlive ? `${state.cameraLive.frameId} · ${frameAge} · ${cameraDelivery}${ackLabel}` : socketText);
  const roverButton = $("roverCameraToggle");
  if (roverButton) {
    roverButton.textContent = state.cameraLive.socket && !state.cameraLive.connected
      ? "연결 중"
      : state.cameraLive.connected ? "스트리밍 중지" : "스트리밍 시작";
    roverButton.classList.toggle("active", state.cameraLive.connected);
  }
  renderReadout("roverCameraStats", [
    { label: "Socket", value: state.cameraLive.connected ? "ON" : "OFF", sub: socketText, color: state.cameraLive.connected ? colors.good : colors.rose },
    { label: "Capture", value: state.cameraLive.resolution, sub: `${state.cameraLive.fps} fps target`, color: colors.amber },
    { label: "Source", value: source, sub: state.cameraLive.frameId || "camera_link_optical", color: colors.cyan },
    { label: "수신 FPS", value: cameraFpsLabel(observedFps), sub: `${observedDetail} · ${frameAge}`, color: colors.blue },
  ]);
}

function toggleImuLive() {
  if (state.imuLive.connected || state.imuLive.socket) {
    closeImuLive("사용자가 IMU 실시간 연결을 중지했습니다.", { userStopped: true });
    return;
  }
  startImuLive();
}

function imuWebSocketUrl() {
  const params = new URLSearchParams({profile: liveNetworkProfile(), flow: "latest"});
  if (window.UBTS_IMU_WS_URL) return `${window.UBTS_IMU_WS_URL}${String(window.UBTS_IMU_WS_URL).includes("?") ? "&" : "?"}${params}`;
  const scheme = location.protocol === "https:" ? "wss" : "ws";
  return `${scheme}://${location.host}/ws/imu?${params}`;
}

function startImuLive() {
  if (!routeSupportsImuLive() || state.imuLive.socket) return;
  if (state.imuLive.reconnectTimer) {
    window.clearTimeout(state.imuLive.reconnectTimer);
    state.imuLive.reconnectTimer = null;
  }
  state.imuLive.enabled = true;
  state.imuLive.userStopped = false;
  state.imuLive.error = "";
  renderImuLiveState("연결 중");
  try {
    const socket = new WebSocket(imuWebSocketUrl());
    state.imuLive.socket = socket;
    socket.addEventListener("open", () => {
      if (socket !== state.imuLive.socket) return;
      state.imuLive.connected = true;
      state.imuLive.error = "";
      renderImuLiveState();
    });
    socket.addEventListener("message", (event) => {
      if (socket !== state.imuLive.socket || !routeSupportsImuLive()) return;
      handleImuLiveMessage(event.data);
    });
    socket.addEventListener("error", () => {
      if (socket !== state.imuLive.socket) return;
      state.imuLive.error = "IMU WebSocket 연결 오류";
      renderImuLiveState();
    });
    socket.addEventListener("close", () => {
      if (socket !== state.imuLive.socket) return;
      const reconnect = state.imuLive.enabled && !state.imuLive.userStopped && routeSupportsImuLive();
      state.imuLive.socket = null;
      state.imuLive.connected = false;
      state.imuLive.topicAlive = false;
      renderImuLiveState(state.imuLive.error || "disconnected");
      if (reconnect) {
        state.imuLive.reconnectTimer = window.setTimeout(() => {
          state.imuLive.reconnectTimer = null;
          startImuLive();
        }, 1500);
      }
    });
  } catch (err) {
    state.imuLive.socket = null;
    state.imuLive.connected = false;
    state.imuLive.error = err.message;
    renderImuLiveState();
  }
}

function closeImuLive(message = "disconnected", options = {}) {
  const socket = state.imuLive.socket;
  if (state.imuLive.reconnectTimer) window.clearTimeout(state.imuLive.reconnectTimer);
  state.imuLive.reconnectTimer = null;
  state.imuLive.enabled = false;
  state.imuLive.userStopped = options.userStopped !== undefined ? Boolean(options.userStopped) : true;
  state.imuLive.socket = null;
  state.imuLive.connected = false;
  state.imuLive.topicAlive = false;
  if (socket) closeSocketAfterConnect(socket, message);
  renderImuLiveState(message);
}

function handleImuLiveMessage(raw) {
  let payload;
  try {
    payload = JSON.parse(raw);
  } catch (_) {
    state.imuLive.error = "잘못된 IMU WebSocket 메시지";
    renderImuLiveState();
    return;
  }
  if (payload.type === "socket") return;
  if (payload.type !== "imu") return;
  if (payload.ack_required!==false && payload.sequence && state.imuLive.socket?.readyState === WebSocket.OPEN) state.imuLive.socket.send(JSON.stringify({type: "imu_ack", sequence: payload.sequence}));
  state.imuLive.topicAlive = Boolean(payload.imu_topic_alive);
  state.imuLive.raw = payload.raw || {};
  state.imuLive.magnetic = payload.magnetic || {};
  state.imuLive.lastImuAgeSec = payload.last_imu_age_s;
  state.imuLive.lastMessageAt = Date.now();
  state.imuLive.error = payload.error || (payload.imu_topic_alive ? "" : "ROS 2 /imu/data_raw 대기 중");
  renderImuLiveState();
}

function imuDisplaySample() {
  // Never select a historical/old-process corrected stream for live display.
  return state.imuLive.raw || {};
}

function vectorNorm(vector) {
  const x = Number(vector?.x);
  const y = Number(vector?.y);
  const z = Number(vector?.z);
  return [x, y, z].every(Number.isFinite) ? Math.hypot(x, y, z) : null;
}

function renderImuLiveState(overrideLabel) {
  const sample = imuDisplaySample();
  const euler = sample.euler_deg || {};
  const roll = Number(euler.roll);
  const pitch = Number(euler.pitch);
  const yaw = Number(euler.yaw);
  const sourceLabel = SIMULATION ? "Gazebo FLU /imu/data_raw" : "sensor-native raw /imu/data_raw";
  const statusLabel = overrideLabel || (state.imuLive.topicAlive ? sourceLabel : state.imuLive.error || "IMU 대기");
  const button = $("roverImuToggle");
  if (button) {
    button.textContent = state.imuLive.connected ? "IMU 중지" : "IMU 시작";
    button.classList.toggle("active", state.imuLive.connected && state.imuLive.topicAlive);
  }
  text("roverImuCaption", statusLabel);
  text("roverImuHeading", Number.isFinite(yaw) ? `${fmtNumber(yaw, 1)}°` : "--°");
  text("roverImuProcessingState", "raw · unmodified");
  const acceleration = sample.availability?.linear_acceleration?.available === false ? {} : sample.linear_acceleration || {};
  const gyro = sample.availability?.angular_velocity?.available === false ? {} : sample.angular_velocity || {};
  const magnetic = state.imuLive.magnetic?.magnetic_field_uT || {};
  const colors = chartColors();
  renderReadout("roverImuStats", [
    { label: "Roll / Pitch", value: Number.isFinite(roll) && Number.isFinite(pitch) ? `${fmtNumber(roll, 1)}° / ${fmtNumber(pitch, 1)}°` : "--", sub: sourceLabel, color: colors.cyan },
    { label: "Acceleration", value: vectorNorm(acceleration) === null ? "--" : `${fmtNumber(vectorNorm(acceleration), 3)} m/s²`, sub: `x ${fmtNumber(acceleration.x, 2)} · y ${fmtNumber(acceleration.y, 2)} · z ${fmtNumber(acceleration.z, 2)}`, color: colors.good },
    { label: "Angular velocity", value: vectorNorm(gyro) === null ? "--" : `${fmtNumber(vectorNorm(gyro), 4)} rad/s`, sub: `x ${fmtNumber(gyro.x, 3)} · y ${fmtNumber(gyro.y, 3)} · z ${fmtNumber(gyro.z, 3)}`, color: colors.amber },
    { label: "Magnetic field", value: vectorNorm(magnetic) === null ? "--" : `${fmtNumber(vectorNorm(magnetic), 2)} µT`, sub: "sensor-native", color: colors.blue },
  ]);
  drawImuAttitude($("roverImuCanvas"), roll, pitch, yaw);
}

function drawImuAttitude(canvas, roll, pitch, yaw) {
  if (!canvas) return;
  const rect = canvas.getBoundingClientRect();
  const width = Math.max(240, rect.width || 520);
  const height = Math.max(180, rect.height || 220);
  const dpr = Math.min(window.devicePixelRatio || 1, DPR_LIMIT);
  const pixelWidth = Math.round(width * dpr);
  const pixelHeight = Math.round(height * dpr);
  if (canvas.width !== pixelWidth || canvas.height !== pixelHeight) {
    canvas.width = pixelWidth;
    canvas.height = pixelHeight;
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, width, height);
  if (![roll, pitch, yaw].every(Number.isFinite)) {
    ctx.fillStyle = "#94a3b8";
    ctx.font = "14px sans-serif";
    ctx.textAlign = "center";
    ctx.fillText("ROS 2 IMU 자세 데이터 대기", width / 2, height / 2);
    return;
  }
  ctx.save();
  ctx.translate(width / 2, height / 2);
  ctx.rotate(-roll * Math.PI / 180);
  const pitchOffset = Math.max(-height, Math.min(height, pitch / 45 * height * 0.45));
  ctx.translate(0, pitchOffset);
  ctx.fillStyle = "#1667a8";
  ctx.fillRect(-width, -height * 2, width * 2, height * 2);
  ctx.fillStyle = "#72502e";
  ctx.fillRect(-width, 0, width * 2, height * 2);
  ctx.strokeStyle = "#f8fafc";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(-width, 0);
  ctx.lineTo(width, 0);
  ctx.stroke();
  ctx.restore();
  ctx.strokeStyle = "#f8fafc";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(width / 2 - 42, height / 2);
  ctx.lineTo(width / 2 - 10, height / 2);
  ctx.lineTo(width / 2, height / 2 + 8);
  ctx.lineTo(width / 2 + 10, height / 2);
  ctx.lineTo(width / 2 + 42, height / 2);
  ctx.stroke();
  ctx.fillStyle = "#e2e8f0";
  ctx.font = "12px sans-serif";
  ctx.textAlign = "center";
  ctx.fillText(`R ${fmtNumber(roll, 1)}°  P ${fmtNumber(pitch, 1)}°  Y ${fmtNumber(yaw, 1)}°`, width / 2, 18);
}


function roundRectPath(ctx, x, y, w, h, r) {
  const radius = Math.min(r, w / 2, h / 2);
  ctx.beginPath();
  ctx.moveTo(x + radius, y);
  ctx.arcTo(x + w, y, x + w, y + h, radius);
  ctx.arcTo(x + w, y + h, x, y + h, radius);
  ctx.arcTo(x, y + h, x, y, radius);
  ctx.arcTo(x, y, x + w, y, radius);
  ctx.closePath();
}

function wrapCanvasText(ctx, textValue, x, y, maxWidth, lineHeight, maxLines) {
  const words = String(textValue || "").split(/\s+/).filter(Boolean);
  let line = "";
  let lines = 0;
  for (const word of words) {
    const next = line ? `${line} ${word}` : word;
    if (ctx.measureText(next).width > maxWidth && line) {
      ctx.fillText(lines + 1 >= maxLines ? `${line.slice(0, Math.max(0, line.length - 3))}...` : line, x, y + lines * lineHeight);
      lines += 1;
      line = word;
      if (lines >= maxLines) return;
    } else {
      line = next;
    }
  }
  if (line && lines < maxLines) ctx.fillText(line, x, y + lines * lineHeight);
}


function setCameraResult(kind, message) {
  const target = $("cameraLiveResult");
  if (!target) return;
  target.className = `test-result ${kind}`;
  target.textContent = message;
}

function lidarWebSocketUrl() {
  const params = new URLSearchParams({ max_points: String(state.lidarPoints.value), format: "polar-v1", flow: "ack", profile: liveNetworkProfile() });
  if (window.UBTS_LIDAR_WS_URL) {
    const separator = String(window.UBTS_LIDAR_WS_URL).includes("?") ? "&" : "?";
    return `${window.UBTS_LIDAR_WS_URL}${separator}${params.toString()}`;
  }
  const scheme = location.protocol === "https:" ? "wss" : "ws";
  return `${scheme}://${location.host}/ws/lidar?${params.toString()}`;
}

function handleLidarLiveMessage(raw) {
  let payload = null;
  try {
    payload = JSON.parse(raw);
  } catch (err) {
    state.lidarLive.error = "잘못된 WebSocket 메시지";
    debugLog("ws", "invalid message", raw);
    renderLidarLiveState();
    return;
  }
  state.lidarLive.lastMessageAt = Date.now();
  state.lidarLive.messageCount += 1;
  if (payload.type === "status") {
    state.lidarLive.lastScanAgeSec = payload.last_scan_age_sec;
    state.lidarLive.pointCount = payload.point_count || 0;
	    state.lidarLive.sourcePointCount = payload.source_point_count || payload.point_count || 0;
	    state.lidarLive.requestedPointCount = payload.requested_max_points || state.lidarPoints.value;
	    state.lidarLive.scanTopicAlive = Boolean(payload.scan_topic_alive);
    state.lidarLive.source = payload.source || "waiting";
    state.lidarLive.error = payload.error || (payload.scan_topic_alive === false ? "ROS2 /scan 대기" : "");
	    if (state.lidarLive.messageCount % 10 === 1 || payload.error) debugLog("ws", "status", payload);
    if (!state.lidarScan || !payload.scan_topic_alive) {
      setScanResult("testing", payload.error || "ROS2 /scan 수신 대기");
    }
    renderLidarLiveState();
    renderLiveSensorLinks();
    return;
  }
  if (payload.type === "scan") {
    if (payload.point_encoding === "polar-v1") payload.points = payload.points.map(([angle_deg, distance_m, quality]) => ({angle_deg, distance_m, quality, x_m: Math.sin(angle_deg * Math.PI / 180) * distance_m, y_m: Math.cos(angle_deg * Math.PI / 180) * distance_m}));
    const result = lidarScanFromLive(payload);
    maybeAutoSetLidarRange(result);
    state.lidarLive.lastScanAgeSec = 0;
    state.lidarLive.pointCount = result.metrics.rendered_points || result.points.length;
    state.lidarLive.sourcePointCount = result.metrics.source_points || state.lidarLive.pointCount;
    state.lidarLive.requestedPointCount = result.metrics.requested_points || state.lidarPoints.value;
    state.lidarLive.scanTopicAlive = true;
    state.lidarLive.source = payload.source || "ros2";
    state.lidarLive.error = "";
    state.lidarScan = result;
	    if (state.lidarLive.messageCount % 10 === 1) {
	      debugLog("ws", "scan", {
	        source: payload.source,
	        points: result.points.length,
	      });
	    }
    renderLidarScan(result, "live_websocket");
    if (state.lidarLive.socket?.readyState === WebSocket.OPEN && Number.isInteger(payload.sequence)) {
      state.lidarLive.socket.send(JSON.stringify({type: "scan_ack", sequence: payload.sequence}));
    }
    renderLidarLiveState();
    renderLiveSensorLinks();
    return;
  }
  if (payload.type === "socket") {
    state.lidarLive.error = payload.ok ? "" : payload.message || "socket status error";
    debugLog("ws", "socket", payload);
    renderLidarLiveState();
    renderLiveSensorLinks();
  }
}

function lidarScanFromLive(payload) {
  const points = (payload.points || []).map(normalizeLidarExportPoint).filter(Boolean);
  const distances = points.map((point) => point.distance_m).filter((value) => Number.isFinite(value));
  const metrics = {
    points: payload.point_count || points.length,
    source_points: payload.source_point_count || payload.point_count || points.length,
    requested_points: payload.requested_max_points || state.lidarPoints.value,
    rendered_points: points.length,
    min_distance_m: distances.length ? Math.min(...distances) : null,
    max_distance_m: distances.length ? Math.max(...distances) : null,
    coverage_deg: Number.isFinite(payload.coverage_deg) ? payload.coverage_deg : angleCoverage(points),
    range_limit_m: 30.0,
  };
  return {
    ok: true,
    state: "live_scan",
    protocol: "ROS2 /scan WebSocket",
    frame_id: payload.frame_id || "laser",
    stamp: payload.stamp,
    tested_at: Math.floor(Date.now() / 1000),
	    points,
	    metrics,
	    source: "live_websocket",
	  };
	}

function maybeAutoSetLidarRange(result, options = {}) {
  if (!result?.ok || !result.points?.length) return;
  if (!state.lidarMap.auto && !options.force) return;
  if (!state.lidarMap.autoPending && !options.force) return;
  const suggested = suggestedLidarRange(result.points);
  if (!Number.isFinite(suggested)) return;
  state.lidarMap.range = clampLidarRange(suggested);
  state.lidarMap.auto = true;
  state.lidarMap.autoPending = false;
  persistLidarRange();
  updateLidarRangeControls();
  debugLog("lidar", "auto range selected", { range: state.lidarMap.range, points: result.points.length });
}

function suggestedLidarRange(points) {
  const extents = points
    .map((point) => Math.max(Math.abs(Number(point.x_m) || 0), Math.abs(Number(point.y_m) || 0)))
    .filter((value) => Number.isFinite(value) && value > 0.05)
    .sort((a, b) => a - b);
  if (!extents.length) return null;
  const p85 = quantile(extents, 0.85);
  const p95 = quantile(extents, 0.95);
  const denseFit = Math.max(p85 * 1.28, p95 * 1.12, 1);
  return Math.ceil(clampLidarRange(denseFit));
}

function quantile(sortedValues, q) {
  if (!sortedValues.length) return null;
  const index = Math.min(sortedValues.length - 1, Math.max(0, Math.floor((sortedValues.length - 1) * q)));
  return sortedValues[index];
}

function renderLidarLiveState(overrideLabel) {
  const button = $("lidarLiveButton");
  if (button) {
    button.textContent = state.lidarLive.socket && !state.lidarLive.connected
      ? "연결 중"
      : state.lidarLive.connected ? "실시간 중지" : "실시간 연결";
    button.classList.toggle("active", state.lidarLive.connected);
  }
  const roverButton = $("roverLidarToggle");
  if (roverButton) {
    roverButton.textContent = state.lidarLive.socket && !state.lidarLive.connected
      ? "연결 중"
      : state.lidarLive.connected ? "LiDAR 중지" : "LiDAR 시작";
    roverButton.classList.toggle("active", state.lidarLive.connected);
  }
  text("lidarLiveDevice", state.telemetry?.connection?.lidar?.device || "--");
  text("lidarLiveTopic", state.lidarLive.scanTopicAlive ? `${lidarSourceLabel(state.lidarLive.source)} live` : lidarSourceLabel(state.lidarLive.source));
  const socketText = overrideLabel || (state.lidarLive.connected ? "connected" : state.lidarLive.error || "disconnected");
  text("lidarLiveSocket", socketText);
  text("lidarLiveAge", lidarAgeLabel());
  const pointLabel = state.lidarLive.pointCount
    ? `${state.lidarLive.pointCount}/${state.lidarLive.sourcePointCount || state.lidarLive.pointCount}`
    : "--";
  text("lidarLivePoints", pointLabel);
  const colors = chartColors();
  renderReadout("roverLidarStats", [
    { label: "Socket", value: state.lidarLive.connected ? "ON" : "OFF", sub: socketText, color: state.lidarLive.connected ? colors.good : colors.rose },
    { label: "Source", value: state.lidarLive.scanTopicAlive ? "live" : lidarSourceLabel(state.lidarLive.source), sub: state.telemetry?.connection?.lidar?.device || "/scan", color: colors.cyan },
    { label: "Points", value: pointLabel, sub: lidarAgeLabel(), color: colors.blue },
  ]);
}

function renderLiveSensorLinks() {
  if (state.telemetry?.connection?.sensors) {
    renderStatusRows("connectionSensors", sensorRows(state.telemetry.connection.sensors));
  }
  if (state.telemetry?.sensors) {
    renderSensors(state.telemetry.sensors);
  }
}

function lidarSourceLabel(source) {
  return { ros2: "ROS2 /scan", serial: "Serial direct", bridge: "Bridge", waiting: "대기" }[source] || source || "대기";
}

function lidarAgeLabel() {
  if (state.lidarLive.scanTopicAlive) return "live";
  const age = state.lidarLive.lastScanAgeSec;
  return age === null || age === undefined ? "--" : `${fmtNumber(age, 1)}s`;
}

function setLidarDownloadFormat(format) {
  state.lidarExport.format = ["json", "csv", "png"].includes(format) ? format : "json";
  document.querySelectorAll("[data-lidar-download]").forEach((button) => {
    button.classList.toggle("active", button.dataset.lidarDownload === state.lidarExport.format);
  });
  updateLidarDownloadState();
}

function updateLidarExport(result, source) {
  if (!result?.ok || !(result.points || []).length) {
    updateLidarDownloadState();
    return;
  }
  const points = (result.points || []).map(normalizeLidarExportPoint).filter(Boolean);
  const latest = { ...result, points };
  state.lidarExport.latestScan = latest;
  state.lidarExport.lastUpdatedAt = new Date();
  state.lidarExport.source = source || result.source || "ros2_scan";
  state.lidarExport.frameId = result.frame_id || "laser";
  updateLidarDownloadState();
}

function updateLidarDownloadState() {
  const button = $("lidarDownloadButton");
  if (!button) return;
  const hasData = Boolean(state.lidarExport.latestScan?.points?.length);
  button.disabled = !hasData;
  button.textContent = hasData ? `현재 지도 다운로드 (${state.lidarExport.format.toUpperCase()})` : "현재 지도 다운로드";
}

function downloadCurrentLidarMap() {
  if (!state.lidarExport.latestScan) return;
  const stamp = formatTimestampForFilename();
  const format = state.lidarExport.format;
  if (format === "png") {
    downloadLidarPng(`ubts_lidar_2d_view_${stamp}.png`);
    return;
  }
  const points = state.lidarExport.latestScan.points;
  if (format === "csv") {
    const rows = points.map(normalizeLidarExportPoint).filter(Boolean);
    downloadCsv(`ubts_lidar_2d_scan_${stamp}.csv`, rows);
    return;
  }
  downloadJson(`ubts_lidar_2d_scan_${stamp}.json`, lidarExportPayload(points));
}

function lidarExportPayload(points) {
  const scan = state.lidarExport.latestScan || {};
  const metrics = scan.metrics || {};
  const range = lidarRangeFor(scan);
  return {
    type: "ubts_lidar_2d_scan",
    version: 1,
    created_at: new Date().toISOString(),
    source: state.lidarExport.source,
    frame_id: state.lidarExport.frameId || "laser",
    coordinate_system: {
      x_m: "right",
      y_m: "front",
      angle_zero_deg: "front/up on dashboard after 180deg display correction",
      angle_direction: "clockwise on dashboard if using x=sin(theta), y=cos(theta)",
    },
    scan: {
      point_count: points.length,
      metrics,
      points: points.map(normalizeLidarExportPoint).filter(Boolean),
    },
    view: {
      range_m: range,
      grid_step_m: lidarGridStep(range),
    },
  };
}

function formatTimestampForFilename(date = new Date()) {
  const pad = (value) => String(value).padStart(2, "0");
  return `${date.getFullYear()}${pad(date.getMonth() + 1)}${pad(date.getDate())}_${pad(date.getHours())}${pad(date.getMinutes())}${pad(date.getSeconds())}`;
}

function downloadBlob(filename, content, mimeType) {
  const blob = content instanceof Blob ? content : new Blob([content], { type: mimeType });
  const url = URL.createObjectURL(blob);
  const link = document.createElement("a");
  link.href = url;
  link.download = filename;
  document.body.append(link);
  link.click();
  link.remove();
  URL.revokeObjectURL(url);
}

function downloadJson(filename, data) {
  downloadBlob(filename, JSON.stringify(data, null, 2), "application/json;charset=utf-8");
}

function downloadCsv(filename, rows) {
  const header = "angle_deg,distance_m,quality,x_m,y_m";
  const body = rows.map((point) => [
    point.angle_deg,
    point.distance_m,
    point.quality,
    point.x_m,
    point.y_m,
  ].join(","));
  downloadBlob(filename, [header, ...body].join("\n"), "text/csv;charset=utf-8");
}

function downloadLidarPng(filename) {
  const source = $("lidarScanCanvas");
  if (!source) return;
  const canvas = document.createElement("canvas");
  canvas.width = source.width;
  canvas.height = source.height;
  const ctx = canvas.getContext("2d");
  const isLight = document.documentElement.dataset.theme === "light";
  ctx.fillStyle = isLight ? "#ffffff" : "#050816";
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  ctx.drawImage(source, 0, 0);
  canvas.toBlob((blob) => {
    if (blob) downloadBlob(filename, blob, "image/png");
  }, "image/png");
}

function normalizeLidarExportPoint(point) {
  if (!point) return null;
  const angleDeg = Number(point.angle_deg);
  const distanceM = Number(point.distance_m);
  const xM = Number(point.x_m);
  const yM = Number(point.y_m);
  if (!Number.isFinite(distanceM) || !Number.isFinite(xM) || !Number.isFinite(yM)) return null;
  // ROS FLU: angle zero is forward; positive angle is left. Canvas right is +x.
  const displayAngle = Number.isFinite(angleDeg) ? (360 - angleDeg) % 360 : null;
  const displayX = -xM;
  const displayY = yM;
  return {
    angle_deg: displayAngle === null ? null : Number(displayAngle.toFixed(3)),
    distance_m: Number(distanceM.toFixed(3)),
    quality: Number.isFinite(Number(point.quality)) ? Number(point.quality) : 0,
    x_m: Number(displayX.toFixed(3)),
    y_m: Number(displayY.toFixed(3)),
  };
}


function normalizeLidarResult(result, analysis = result?.analysis) {
  if (!result) return null;
  return {
    ...result,
    points: (result.points || []).map(normalizeLidarExportPoint).filter(Boolean),
  };
}

function normalizeArchiveLidarResult(result, analysis = result?.analysis, frame = result) {
  if (!result) return null;
  const coordinateFrame = frame?.coordinate_frame || result.coordinate_frame || state.recordings.timeline?.coordinate_frame || "";
  const legacyRecorderFrame = coordinateFrame !== LIDAR_COORDINATE_FRAME;
  const points = (result.points || []).map((point) => legacyRecorderFrame ? archiveLegacyPointToBridgeFrame(point) : point);
  return {
    ...result,
    coordinate_frame: coordinateFrame || "legacy_archive_recorder_xy_v0",
    points: points.map(normalizeLidarExportPoint).filter(Boolean),
  };
}

function archiveLegacyPointToBridgeFrame(point) {
  if (!point) return point;
  const xM = Number(point.x_m);
  const yM = Number(point.y_m);
  if (!Number.isFinite(xM) || !Number.isFinite(yM)) return point;
  return {
    ...point,
    x_m: yM,
    y_m: xM,
  };
}


function angleCoverage(points) {
  const bins = new Set();
  for (const point of points) {
    if (Number.isFinite(point.angle_deg)) bins.add(Math.floor(point.angle_deg) % 360);
  }
  return bins.size;
}

function renderConnection(connection) {
  const lidar = connection.lidar || {};
  const camera = connection.camera || {};
  const bridge = connection.lidar_bridge || {};
  const cameraBridge = connection.camera_bridge || {};
  const ws = connection.lidar_websocket || {};
  const cameraWs = connection.camera_websocket || {};
  const last = lidar.last_test || {};
  const lastScan = lidar.last_scan || {};
  const scanLive = Boolean(state.lidarLive.scanTopicAlive || bridge.scan_topic_alive);
  const cameraLive = Boolean(state.cameraLive.frameTopicAlive || cameraBridge.frame_topic_alive);
  const wsText = `WS ${ws.active || 0}`;
  const cameraWsText = `WS ${cameraWs.active || 0}`;
  applyCameraModeCatalog(camera.supported_view_modes, camera.supported_view_modes_source || "fallback");
  applyCameraStatus(camera, cameraBridge);
  text("connectionLidarState", scanLive ? "수신 중" : (lidar.present ? lidarStateLabel(lidar.state) : "미인식"));
  text("connectionLidarDevice", lidar.device || "--");
  text("connectionLidarProtocol", "ROS2 /scan");
  text("connectionLidarSubtext", "subscribe-only");
  text("connectionLidarHealth", scanLive ? "수신 중" : (lidar.present ? "ROS2 대기" : "--"));
  text("connectionLidarHealthText", scanLive ? `ROS2 /scan live · ${wsText}` : `ROS2 /scan 대기 · ${wsText}`);
  text("connectionCameraState", cameraLive ? "수신 중" : cameraStateLabel(camera));
  text("connectionCameraDevice", cameraLive ? `${cameraResolutionLabel(cameraBridge)} · ${cameraWsText}` : cameraDeviceLabel(camera));
  text("lidarDeviceDetail", lidar.present ? shortPath(lidar.device) : "장치 미인식");
  text("lidarByIdDetail", lidar.by_id ? shortPath(lidar.by_id, 42) : "stable ID 대기");
  text("lidarDriverDetail", driverLabel(lidar.driver_hint));
  text("lidarLastTestDetail", lidarLastTestLabel(last));
  text("lidarPolicyDetail", policyLabel(lidar.conflict_policy));
  renderStatusRows("connectionServices", serviceRows(connection.services || {}));
  renderStatusRows("connectionSensors", sensorRows(connection.sensors || {}));
  renderLidarLiveState();
  renderCameraLiveState();
  if (last.ok || last.message) renderLidarTest(last);
  if (lastScan.ok || lastScan.message) {
    if (!state.lidarScan || (lastScan.tested_at || 0) >= (state.lidarScan.tested_at || 0)) {
      state.lidarScan = lastScan;
      renderLidarScan(lastScan, "ros2_scan");
    }
  }
}

function cameraResolutionLabel(cameraBridge) {
  const width = cameraBridge?.source_width || state.cameraLive.sourceWidth;
  const height = cameraBridge?.source_height || state.cameraLive.sourceHeight;
  return width && height ? `${width}x${height}` : "ROS2 /camera";
}

function cameraStateLabel(camera) {
  if (camera.present) return lidarStateLabel(camera.state);
  if (camera.usb_present || camera.state === "usb_detected_video_node_missing") return "USB 감지";
  return "미인식";
}

function cameraDeviceLabel(camera) {
  if (camera.device) return camera.device;
  if (camera.usb_present || camera.state === "usb_detected_video_node_missing") {
    const devices = Array.isArray(camera.usb_devices) ? camera.usb_devices : [];
    const first = devices[0] || {};
    const product = [first.manufacturer, first.product].filter(Boolean).join(" ");
    return product ? `${product} · 비디오 노드 대기` : "USB camera · 비디오 노드 대기";
  }
  return "--";
}

function renderLidarTest(result) {
  if (!result) return;
  const state = result.ok ? "ok" : "bad";
  setTestResult(state, lidarTestMessage(result));
}

function setTestResult(kind, message) {
  const target = $("lidarTestResult");
  if (!target) return;
  target.className = `test-result ${kind}`;
  target.textContent = message;
}

function renderLidarScan(result, source = result?.source || "ros2_scan") {
  if (!result) return;
	  const metrics = result.metrics || {};
	  const points = result.points || [];
	  const analysis = result.analysis || {};
	  const ok = Boolean(result.ok && points.length);
  const renderedPoints = metrics.rendered_points || points.length || 0;
  const requestedPoints = metrics.requested_points || state.lidarPoints.value;
  const sourcePoints = metrics.source_points || metrics.points || points.length || 0;
  const message = ok
    ? `실시간 수신 중 · 최대 ${requestedPoints}개 표시 · ${fmtNumber(metrics.coverage_deg, 0)}도 커버리지`
    : cleanLidarError(result, "실시간 LiDAR 데이터를 기다리는 중입니다.");
  setScanResult(ok ? "ok" : "bad", message);
  const colors = chartColors();
	  renderReadout("lidarScanStats", [
	    { label: "Rendered", value: String(renderedPoints), sub: `limit ${requestedPoints} · source ${sourcePoints}`, color: colors.cyan },
	    { label: "Coverage", value: `${fmtNumber(metrics.coverage_deg, 0)}°`, sub: "1° bins", color: colors.good },
	    { label: "Range", value: `${fmtNumber(metrics.min_distance_m, 2)}-${fmtNumber(metrics.max_distance_m, 2)} m`, sub: "valid points", color: colors.blue },
	  ]);
  updateLidarExport(result, source);
  renderActiveLidarMap();
}

function setScanResult(kind, message) {
  const target = $("lidarScanResult");
  if (!target) return;
  target.className = `test-result ${kind}`;
  target.textContent = message;
}

function lidarStateLabel(state) {
  return {
    detected: "인식됨",
    in_use: "점유 중",
    scan_live: "수신 중",
    waiting_for_scan: "스캔 대기",
    waiting_for_ros2: "ROS2 대기",
    waiting_for_frame: "프레임 대기",
    waiting_for_open: "장치 열기 대기",
    video_node_missing: "비디오 노드 대기",
    usb_detected_video_node_missing: "비디오 노드 대기",
    not_found: "미인식",
  }[state] || state || "--";
}

function holderLabel(holders) {
  return holders.map((item) => `${item.pid} ${item.cmd || ""}`.trim()).join(", ");
}

function shortPath(path, maxLength = 34) {
  if (!path) return "--";
  const value = String(path);
  const byIdPrefix = "/dev/serial/by-id/";
  const label = value.startsWith(byIdPrefix) ? `by-id/${value.slice(byIdPrefix.length)}` : value;
  if (label.length <= maxLength) return label;
  const keep = Math.max(8, Math.floor((maxLength - 3) / 2));
  return `${label.slice(0, keep)}...${label.slice(-keep)}`;
}

function driverLabel(driver) {
  const value = String(driver || "").toLowerCase();
  if (value.includes("cp210")) return "CP210x USB-UART 연결";
  if (value.includes("cdc") || value.includes("ttyacm")) return "USB CDC ACM 연결";
  if (value.includes("ttyusb")) return "USB-UART 직렬 연결";
  return "직렬 드라이버 확인 전";
}

function lidarLastTestLabel(result) {
  if (!result || (!result.ok && !result.message)) return "아직 확인 전";
  if (!result.ok) return cleanLidarError(result, "응답 없음");
  return result.state === "ros2_owned" ? "ROS2 driver 소유 중" : "장치 확인됨 · subscribe-only";
}

function policyLabel(policy) {
  return policy || "Dashboard는 LiDAR serial port를 열지 않고 ROS2 /scan만 구독합니다.";
}

function lidarTestMessage(result) {
  if (!result) return "정보 없음";
  if (!result.ok) return cleanLidarError(result, "LiDAR 상태를 확인하지 못했습니다.");
  return result.message || "LiDAR 장치 상태를 확인했습니다. Dashboard는 serial port를 열지 않습니다.";
}

function cleanLidarError(result, fallback) {
  const stateName = result?.state || result?.status?.state;
  const messages = {
    not_found: "LiDAR 장치를 찾지 못했습니다. USB 연결과 전원을 확인하세요.",
    in_use: "다른 프로세스가 LiDAR 포트를 사용 중이라 테스트를 건너뛰었습니다.",
    busy: "이미 다른 LiDAR 테스트가 실행 중입니다.",
    no_protocol_response: "직렬 장치는 보이지만 RPLIDAR 진단 응답이 없습니다.",
    no_scan_data: "스캔 데이터가 들어오지 않았습니다. 모터 전원과 baudrate를 확인하세요.",
    no_scan_points: "스캔 명령 후 유효한 거리 점을 받지 못했습니다.",
    disabled: "Dashboard serial scan은 비활성화되어 있습니다. ROS2 /scan을 사용하세요.",
  };
  return messages[stateName] || result?.message || fallback || "확인 실패";
}

function hostIpLabel(network) {
  if (network.primary_ipv4) {
    return [network.primary_interface, network.primary_ipv4].filter(Boolean).join(" ");
  }
  for (const iface of network.interfaces || []) {
    if (iface.name !== "lo" && iface.ipv4?.length) return `${iface.name} ${iface.ipv4[0]}`;
  }
  return "--";
}

function setProgress(id, value) {
  setProgressFill($(`${id}`)?.querySelector("i"), value);
}

function setProgressFill(fill, value) {
  if (!fill) return;
  const pct = Math.max(0, Math.min(100, Number(value || 0)));
  fill.style.transform = `scaleX(${pct / 100})`;
  fill.setAttribute("aria-valuenow", pct.toFixed(1));
}

function renderServices(services) {
  renderStatusRows("overviewServices", serviceRows(services));
}

function serviceRows(services) {
  return Object.entries(services).map(([key, item]) => ({
    title: serviceLabel(key),
    sub: [item.host, item.port ? `:${item.port}` : "", item.state].filter(Boolean).join(" "),
    ok: Boolean(item.ok),
    level: item.ok ? "ok" : "warn",
    value: item.ok ? "OK" : "대기",
  }));
}

function renderSensors(sensors) {
  renderStatusRows("sensorList", sensorRows(sensors));
}

function sensorRows(sensors) {
  return Object.entries(sensors).map(([key, item]) => ({
    title: sensorLabel(key),
    sub: [
      item.topic,
      item.frame_id ? `frame ${item.frame_id}` : "",
      item.link ? `link ${item.link}` : "",
      item.owner ? `owner ${item.owner}` : "",
      item.dashboard_access ? item.dashboard_access : "",
      lidarStateLabel(item.state),
    ].filter(Boolean).join(" · "),
    ok: Boolean(item.ok || (key === "lidar" && lidarBridgeLive()) || (key === "camera" && cameraBridgeLive())),
    level: sensorLevel(key, item),
    value: sensorValue(key, item),
  }));
}

function sensorLevel(key, item) {
  if (key === "lidar" && lidarBridgeLive()) return "ok";
  if (key === "camera" && cameraBridgeLive()) return "ok";
  if (item.ok) return "ok";
  if (item.severity === "error" || item.state === "video_node_missing") return "bad";
  if (item.severity === "waiting" || item.state === "waiting_for_ros2") return "warn";
  if ((key === "lidar" || key === "camera") && item.dashboard_access === "subscribe_only") return "warn";
  return "bad";
}

function sensorValue(key, item) {
  if (key === "lidar" && lidarBridgeLive()) return `${lidarSourceLabel(item.source || state.lidarLive.source)} live`;
  if (key === "camera" && cameraBridgeLive()) return `${cameraResolutionLabel(item)} live`;
  if (item.rate_hz) return `${Number(item.rate_hz).toFixed(1)} Hz`;
  if (key === "lidar" && item.dashboard_access === "subscribe_only") return "ROS2 대기";
  if (key === "camera" && item.state === "video_node_missing") return "비디오 노드 대기";
  if (key === "camera" && item.dashboard_access === "subscribe_only") return "ROS2 대기";
  return "연결 전";
}

function lidarBridgeLive() {
  return Boolean(state.lidarLive.scanTopicAlive || state.telemetry?.connection?.lidar_bridge?.scan_topic_alive);
}

function cameraBridgeLive() {
  return Boolean(state.cameraLive.frameTopicAlive || state.telemetry?.connection?.camera_bridge?.frame_topic_alive);
}

function renderStatusRows(targetId, rows) {
  const target = $(targetId);
  if (!target) return;
  const renderKey = JSON.stringify(rows.map((row) => [row.id, row.title, row.sub, row.value, row.ok, row.level]));
  if (target.dataset.renderKey === renderKey) return;
  target.dataset.renderKey = renderKey;
  target.innerHTML = "";
  for (const row of rows) {
    const level = row.ok ? "ok" : row.level || "bad";
    const div = document.createElement("div");
    div.className = "status-item";
    if (row.id) div.id = row.id;
    div.innerHTML = `
      <i class="status-dot ${escapeHtml(level)}"></i>
      <span><strong>${escapeHtml(row.title)}</strong><small>${escapeHtml(row.sub || "")}</small></span>
      <b>${escapeHtml(row.value || "")}</b>
    `;
    target.append(div);
  }
}

function syncItems(targetId, rows, keyFn, createFn, updateFn) {
  const target = $(targetId);
  if (!target) return;
  const existing = new Map([...target.children].map((child) => [child.dataset.key, child]));
  const seen = new Set();
  for (const row of rows) {
    const key = String(keyFn(row));
    let el = existing.get(key);
    if (!el) {
      el = createFn();
      el.dataset.key = key;
    }
    updateFn(el, row);
    target.append(el);
    seen.add(key);
  }
  for (const [key, el] of existing) {
    if (!seen.has(key)) el.remove();
  }
}

function renderCores(cores) {
  syncItems("coreList", cores, coreKey, createCoreRow, updateCoreRow);
}

function createCoreRow() {
  const div = document.createElement("div");
  div.className = "core";
  div.innerHTML = `
    <span data-role="name"></span>
    <div class="progress"><i></i></div>
    <strong data-role="value"></strong>
  `;
  return div;
}

function updateCoreRow(el, core) {
  el.querySelector('[data-role="name"]').textContent = cpuLabel(core);
  el.querySelector('[data-role="value"]').textContent = fmtPercent(core.percent);
  setProgressFill(el.querySelector(".progress i"), core.percent);
}

function renderCoreCharts(cores) {
  syncItems("cpuSeparateView", cores, coreKey, createCoreChart, updateCoreChart);
}

function createCoreChart() {
  const article = document.createElement("article");
  article.className = "core-chart";
  article.innerHTML = `
    <canvas class="resource-chart" width="520" height="150"></canvas>
    <div class="core-chart-head">
      <span data-role="name"></span>
      <b data-role="value"></b>
      <small>Y 0-100%<br />X 최근 90초</small>
    </div>
  `;
  return article;
}

function updateCoreChart(el, core) {
  el.querySelector('[data-role="name"]').textContent = cpuLabel(core);
  el.querySelector('[data-role="value"]').textContent = fmtPercent(core.percent);
}

function renderTemperatures(rows) {
  const target = $("temperatureList");
  if (!target) return;
  target.innerHTML = "";
  for (const row of rows) {
    const chip = document.createElement("span");
    chip.className = "chip";
    chip.textContent = `${row.name} ${fmtTemp(row.temperature_c)}`;
    target.append(chip);
  }
}

function renderDisks(disks) {
  const primary = disks[0];
  text("diskSummary", primary ? `${primary.mount} · ${fmtBytes(primary.used)} / ${fmtBytes(primary.total)} · ${fmtPercent(primary.percent)}` : "--");
  syncItems("diskList", disks, (disk) => disk.mount, createDiskRow, updateDiskRow);
}

function createDiskRow() {
  const div = document.createElement("div");
  div.className = "disk-row";
  div.innerHTML = `
    <div><strong data-role="mount"></strong><small data-role="usage"></small></div>
    <div class="progress"><i></i></div>
    <b data-role="percent"></b>
  `;
  return div;
}

function updateDiskRow(el, disk) {
  el.querySelector('[data-role="mount"]').textContent = disk.mount;
  const source = [disk.source, disk.fstype].filter(Boolean).join(" · ");
  el.querySelector('[data-role="usage"]').textContent = `${source ? `${source} · ` : ""}${fmtBytes(disk.used)} used · ${fmtBytes(disk.free)} free · ${fmtBytes(disk.total)} total`;
  el.querySelector('[data-role="percent"]').textContent = fmtPercent(disk.percent);
  setProgressFill(el.querySelector(".progress i"), disk.percent);
}

function renderInterfaces(interfaces, primaryInterface) {
  const target = $("interfaceList");
  if (!target) return;
  const rows = usefulInterfaces(interfaces, primaryInterface);
  if (!rows.length) {
    target.innerHTML = `<div class="empty-state">활성 네트워크 인터페이스를 찾지 못했습니다.</div>`;
    return;
  }
  if (target.querySelector(".empty-state")) target.innerHTML = "";
  syncItems("interfaceList", rows, (iface) => iface.name, createInterfaceCard, (el, iface) => updateInterfaceCard(el, iface, primaryInterface));
}

function usefulInterfaces(interfaces, primaryInterface) {
  return interfaces
    .filter((iface) => iface.name !== "lo")
    .filter((iface) => iface.name === primaryInterface || (iface.state === "up" && ((iface.ipv4 || []).length || iface.speed_mbps || iface.rx_bps || iface.tx_bps)))
    .sort((a, b) => {
      if (a.name === primaryInterface) return -1;
      if (b.name === primaryInterface) return 1;
      if (a.state === "up" && b.state !== "up") return -1;
      if (b.state === "up" && a.state !== "up") return 1;
      return a.name.localeCompare(b.name);
    })
    .slice(0, 6);
}

function createInterfaceCard() {
  const div = document.createElement("article");
  div.className = "interface-card";
  div.innerHTML = `
    <div class="card-title">
      <strong data-role="name"></strong>
      <span data-role="state" class="tag"></span>
    </div>
    <div class="interface-metrics">
      <div><span>IPv4</span><b data-role="ip"></b></div>
      <div><span>Link</span><b data-role="link"></b></div>
      <div><span>RX / TX</span><b data-role="traffic"></b></div>
    </div>
  `;
  return div;
}

function updateInterfaceCard(el, iface, primaryInterface) {
  el.querySelector('[data-role="name"]').textContent = iface.name === primaryInterface ? `${iface.name} · primary` : iface.name;
  const stateEl = el.querySelector('[data-role="state"]');
  stateEl.className = `tag ${iface.state === "up" ? "ok" : ""}`;
  stateEl.textContent = iface.state || "unknown";
  el.querySelector('[data-role="ip"]').textContent = (iface.ipv4 || []).join(", ") || "--";
  el.querySelector('[data-role="link"]').textContent = iface.speed_mbps ? `${iface.speed_mbps} Mbps` : "--";
  el.querySelector('[data-role="traffic"]').textContent = `${fmtBps(iface.rx_bps)} / ${fmtBps(iface.tx_bps)}`;
}

function renderContainers(docker) {
  const target = $("containerList");
  if (!target) return;
  if (!docker.available) {
    target.innerHTML = `<div class="empty-state">${escapeHtml(docker.error || "Docker 정보를 읽을 수 없습니다.")}</div>`;
    state.selectedContainerId = null;
    renderContainerDetailError(docker.error || "Docker 정보를 읽을 수 없습니다.");
    return;
  }
  const containers = docker.containers || [];
  if (!containers.length) {
    target.innerHTML = `<div class="empty-state">표시할 컨테이너가 없습니다.</div>`;
    state.selectedContainerId = null;
    renderContainerDetailError("컨테이너가 없습니다.");
    return;
  }
  if (target.querySelector(".empty-state")) target.innerHTML = "";
  const ids = containers.map((container) => container.id || container.name).filter(Boolean);
  if (state.selectedContainerId && !ids.includes(state.selectedContainerId)) {
    state.selectedContainerId = null;
    state.containerDetail = null;
    state.containerLogs = null;
  }
  if (!state.selectedContainerId && currentRoute() === "/docker") state.selectedContainerId = ids[0] || null;
  syncItems("containerList", containers, (container) => container.id || container.name, createContainerRow, updateContainerRow);
}

function createContainerRow() {
  const row = document.createElement("div");
  row.className = "container-row";
  row.setAttribute("role", "button");
  row.setAttribute("tabindex", "0");
  row.addEventListener("click", () => selectContainer(row.dataset.key));
  row.addEventListener("keydown", (event) => {
    if (event.key === "Enter" || event.key === " ") {
      event.preventDefault();
      selectContainer(row.dataset.key);
    }
  });
  row.innerHTML = `
    <div>
      <strong data-role="name"></strong>
      <small data-role="image"></small>
    </div>
    <span data-role="state" class="tag"></span>
    <b data-role="cpu"></b>
    <b data-role="memory"></b>
    <small data-role="status"></small>
  `;
  return row;
}

function updateContainerRow(row, container) {
  const key = container.id || container.name;
  row.classList.toggle("active", Boolean(key && key === state.selectedContainerId));
  row.setAttribute("aria-pressed", key && key === state.selectedContainerId ? "true" : "false");
  row.querySelector('[data-role="name"]').textContent = container.name || container.id;
  row.querySelector('[data-role="image"]').textContent = container.image || "";
  const stateEl = row.querySelector('[data-role="state"]');
  stateEl.className = `tag ${container.state === "running" ? "ok" : ""}`;
  stateEl.textContent = container.state || "--";
  row.querySelector('[data-role="cpu"]').textContent = fmtPercent(container.cpu_percent);
  row.querySelector('[data-role="memory"]').textContent = fmtBytes(container.memory_usage);
  row.querySelector('[data-role="status"]').textContent = container.status || "";
}

function selectContainer(containerId) {
  if (!containerId || containerId === state.selectedContainerId) return;
  state.selectedContainerId = containerId;
  state.containerDetail = null;
  state.containerLogs = null;
  state.containerLogsUpdatedAt = null;
  document.querySelectorAll(".container-row").forEach((row) => {
    row.classList.toggle("active", row.dataset.key === containerId);
    row.setAttribute("aria-pressed", row.dataset.key === containerId ? "true" : "false");
  });
  renderContainerDetailLoading(containerId);
  maybeRefreshContainerDetail(true);
}

function maybeRefreshContainerDetail(force = false) {
  if (currentRoute() !== "/docker" || !state.selectedContainerId) return;
  const now = Date.now();
  if (!force && state.containerLogsUpdatedAt && now - state.containerLogsUpdatedAt < 2500) return;
  if (state.containerRequestInFlight || state.containerLogInFlight) return;
  loadSelectedContainer(force).catch((err) => {
    debugLog("docker", "container detail failed", err);
    renderContainerDetailError(err.message);
  });
}

async function loadSelectedContainer(force = false) {
  const id = state.selectedContainerId;
  if (!id) return;
  state.containerRequestInFlight = true;
  state.containerLogInFlight = true;
  if (force || !state.containerDetail) renderContainerDetailLoading(id);
  try {
    const encoded = encodeURIComponent(id);
    const [detail, logs] = await Promise.all([
      getJson(`/api/docker/container/${encoded}`),
      getJson(`/api/docker/container/${encoded}/logs?tail=180`),
    ]);
    if (id !== state.selectedContainerId) return;
    state.containerDetail = detail;
    state.containerLogs = logs;
    state.containerLogsUpdatedAt = Date.now();
    renderContainerDetail(detail, logs);
  } finally {
    state.containerRequestInFlight = false;
    state.containerLogInFlight = false;
  }
}

function renderContainerDetailLoading(containerId) {
  text("containerDetailTitle", containerId ? `컨테이너 상세 · ${containerId}` : "컨테이너 상세");
  const detail = $("containerDetail");
  const logs = $("containerLogs");
  renderContainerActions(null);
  if (detail) detail.innerHTML = `<div class="empty-state">컨테이너 정보를 불러오는 중입니다.</div>`;
  if (logs) logs.textContent = "로그를 불러오는 중입니다.";
}

function renderContainerDetailError(message) {
  text("containerDetailTitle", "컨테이너 상세");
  const detail = $("containerDetail");
  const logs = $("containerLogs");
  renderContainerActions(null);
  if (detail) detail.innerHTML = `<div class="empty-state">${escapeHtml(message || "컨테이너를 선택하세요.")}</div>`;
  if (logs) logs.textContent = "로그 대기 중";
}

function renderContainerDetail(detailPayload, logsPayload) {
  const detail = $("containerDetail");
  const logs = $("containerLogs");
  if (!detailPayload?.ok) {
    renderContainerDetailError(detailPayload?.error || "컨테이너 정보를 읽을 수 없습니다.");
    return;
  }
  text("containerDetailTitle", `컨테이너 상세 · ${detailPayload.name || detailPayload.id}`);
  const stats = detailPayload.stats || {};
  const stateInfo = detailPayload.state || {};
  const restartPolicy = detailPayload.restart_policy || {};
  renderContainerActions(detailPayload);
  if (detail) {
    detail.innerHTML = `
      <div class="container-detail-grid">
        ${containerDetailCell("State", containerStateLabel(stateInfo), stateInfo.StartedAt ? `since ${formatShortDate(stateInfo.StartedAt)}` : "")}
        ${containerDetailCell("Image", detailPayload.image || "--", detailPayload.full_id || detailPayload.id || "")}
        ${containerDetailCell("CPU", fmtPercent(stats.cpu_percent), "one-shot stat")}
        ${containerDetailCell("Memory", `${fmtBytes(stats.memory_usage)} / ${fmtBytes(stats.memory_limit)}`, fmtPercent(stats.memory_percent))}
        ${containerDetailCell("Network", `${fmtBytes(stats.rx_bytes)} RX / ${fmtBytes(stats.tx_bytes)} TX`, detailPayload.network_mode || "")}
        ${containerDetailCell("Restart", restartPolicyLabel(restartPolicy), "policy")}
      </div>
    `;
  }
  if (logs) {
    if (!logsPayload?.ok) {
      logs.textContent = `로그 읽기 실패: ${logsPayload?.error || "unknown"}`;
    } else if ((logsPayload.lines || []).length) {
      logs.textContent = logsPayload.lines.join("\n");
    } else {
      logs.textContent = "로그가 없습니다.";
    }
  }
}

function renderContainerActions(detailPayload) {
  const target = $("containerActions");
  if (!target) return;
  if (!detailPayload?.ok) {
    target.innerHTML = "";
    return;
  }
  if (detailPayload.self) {
    target.innerHTML = `<span class="container-self-lock">Dashboard self</span>`;
    return;
  }
  const running = Boolean(detailPayload.state?.Running);
  const id = escapeHtml(detailPayload.id || detailPayload.full_id || "");
  target.innerHTML = `
    <button type="button" data-container-action="start" data-container-id="${id}" ${running ? "disabled" : ""}>시작</button>
    <button type="button" data-container-action="stop" data-container-id="${id}" ${running ? "" : "disabled"}>중지</button>
    <button type="button" data-container-action="restart" data-container-id="${id}" ${running ? "" : "disabled"}>재시작</button>
  `;
}

function containerDetailCell(label, value, sub = "") {
  return `
    <article>
      <span>${escapeHtml(label)}</span>
      <strong>${escapeHtml(value)}</strong>
      <small>${escapeHtml(sub || "")}</small>
    </article>
  `;
}

function containerStateLabel(stateInfo) {
  if (!stateInfo) return "--";
  if (stateInfo.Running) return "running";
  return stateInfo.Status || (stateInfo.ExitCode !== undefined ? `exited ${stateInfo.ExitCode}` : "--");
}

function restartPolicyLabel(policy) {
  const name = policy?.Name || "--";
  const retry = policy?.MaximumRetryCount;
  return retry ? `${name} (${retry})` : name;
}

function formatShortDate(value) {
  if (!value) return "--";
  const date = new Date(value);
  return Number.isNaN(date.getTime()) ? String(value) : date.toLocaleString("ko-KR");
}

function initFanControls() {
  $("fanAutoToggle")?.addEventListener("change", async (event) => {
    await postFanJson("/api/fan/mode", { auto: Boolean(event.target.checked) });
  });
  $("fanPwmSlider")?.addEventListener("input", (event) => {
    text("fanSliderValue", event.target.value);
  });
  $("fanPwmSlider")?.addEventListener("change", async (event) => {
    await postFanJson("/api/fan/pwm", { pwm: Number(event.target.value) });
  });
  document.querySelectorAll("[data-fan-preset]").forEach((button) => {
    button.addEventListener("click", async () => {
      await postFanJson("/api/fan/preset", { preset: button.dataset.fanPreset });
    });
  });
}

function initDockerControls() {
  $("containerRefreshButton")?.addEventListener("click", () => maybeRefreshContainerDetail(true));
  $("containerActions")?.addEventListener("click", async (event) => {
    const button = event.target.closest("[data-container-action]");
    if (!button || button.disabled) return;
    await postContainerAction(button.dataset.containerId, button.dataset.containerAction);
  });
}

async function postContainerAction(containerId, action) {
  if (!containerId || !action) return;
  const target = $("containerActions");
  if (target) target.querySelectorAll("button").forEach((button) => { button.disabled = true; });
  debugLog("docker", `POST ${action}`, { containerId });
  try {
    const res = await fetch(`/api/docker/container/${encodeURIComponent(containerId)}/${encodeURIComponent(action)}`, {
      method: "POST",
      cache: "no-store",
    });
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    const payload = await res.json();
    debugLog("docker", `${action} result`, payload);
    state.containerDetail = null;
    state.containerLogs = null;
    state.containerLogsUpdatedAt = null;
    await tick();
    await loadSelectedContainer(true);
  } catch (err) {
    debugLog("docker", `${action} failed`, err);
    renderContainerDetailError(`컨테이너 ${action} 실패: ${err.message}`);
  }
}

async function postFanJson(path, payload) {
  setFanStatus("testing", "요청을 적용하는 중입니다.");
  debugLog("fan", `POST ${path}`, payload || {});
  try {
    const res = await fetch(path, {
      method: "POST",
      cache: "no-store",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload || {}),
    });
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    const result = await res.json();
    debugLog("fan", `${path} result`, result);
    setFanStatus(result.ok ? "ok" : "bad", fanResultMessage(result));
    await tick();
  } catch (err) {
    debugLog("fan", `${path} failed`, err);
    setFanStatus("bad", `Fan API 실패: ${err.message}`);
  }
}

function fanResultMessage(result) {
  if (result.ok && result.action === "auto") return "자동 제어로 전환했습니다.";
  if (result.ok && result.action === "manual") return "수동 제어로 전환했습니다.";
  if (result.ok) return result.message || result.state || "적용 완료";
  const writeError = result.write?.error || result.enable?.error || result.systemctl?.error;
  const stateText = result.message || result.state || result.systemctl?.state || "적용 실패";
  return writeError ? `${stateText}: ${writeError}` : stateText;
}

function setFanStatus(kind, message) {
  const target = $("fanControlStatus");
  if (!target) return;
  target.className = `test-result ${kind}`;
  target.textContent = message;
}

function renderFan(fan) {
  text("fanTempValue", fmtTemp(fan.temperature_c));
  text("fanTempText", fanTemperatureLabel(fan));
  text("fanPwmValue", fan.pwm === null || fan.pwm === undefined ? "--" : `${fan.pwm}`);
  text("fanModeValue", fan.mode === "auto" ? "자동" : fan.mode === "manual" ? "수동" : "--");
  text("fanModeText", fan.mode === "auto" ? "nvfancontrol 사용" : "PWM 직접 제어");
  text("fanTempChartValue", fmtTemp(fan.temperature_c));

  const auto = $("fanAutoToggle");
  if (auto) {
    auto.disabled = !fan.available;
    auto.checked = fan.mode === "auto";
  }
  const manualDisabled = !fan.available || fan.mode === "auto";
  const slider = $("fanPwmSlider");
  if (slider) {
    slider.disabled = manualDisabled;
    if (fan.pwm !== null && fan.pwm !== undefined) slider.value = fan.pwm;
  }
  text("fanSliderValue", fan.pwm === null || fan.pwm === undefined ? "0" : String(fan.pwm));
  document.querySelectorAll("[data-fan-preset]").forEach((button) => {
    button.disabled = manualDisabled;
  });

  if ($("fanControlStatus") && !$("fanControlStatus").classList.contains("testing")) {
    setFanStatus(fan.available ? "ok" : "bad", fan.available ? fanHoldLabel(fan) : "팬 제어 장치를 찾지 못했습니다.");
  }
}

function fanTemperatureLabel(fan) {
  const level = fan.safety?.level;
  if (level === "critical") return "위험 온도";
  if (level === "hot") return "높은 온도";
  if (level === "warm") return "주의 온도";
  if (level === "unknown") return "온도 확인 불가";
  return "정상 범위";
}

function fanHoldLabel(fan) {
  if (fan.mode === "auto") return "자동 제어 중";
  if (fan.desired?.mode === "manual" && fan.desired?.pwm !== undefined) {
    return `수동 PWM ${fan.desired.pwm} 유지 중`;
  }
  return `수동 PWM ${fan.pwm ?? "--"}`;
}

function renderChartReadouts(data) {
  const sys = data.system || {};
  const net = data.network || {};
  const docker = data.docker || {};
  const fan = data.fan || {};
  const colors = chartColors();
  const dockerMemPercent = docker.summary?.memory_limit ? (docker.summary.memory_usage / docker.summary.memory_limit) * 100 : null;
  renderReadout("overviewSystemReadout", [
    { label: "CPU", value: fmtPercent(sys.cpu?.total_percent), sub: "전체 사용률", color: colors.good },
    { label: "Memory", value: fmtPercent(memoryPercent(sys.memory)), sub: `${fmtBytes(sys.memory?.used)} / ${fmtBytes(memoryTotal(sys.memory))}`, color: colors.cyan },
  ]);
  renderReadout("overviewNetworkReadout", [
    { label: "RX", value: fmtBps(net.total_rx_bps), sub: "수신", color: colors.blue },
    { label: "TX", value: fmtBps(net.total_tx_bps), sub: "송신", color: colors.rose },
  ]);
  renderReadout("cpuReadout", [
    { label: "Total", value: fmtPercent(sys.cpu?.total_percent), sub: `${(sys.cpu?.cores || []).length} cores · 0-100%`, color: colors.good },
    ...((sys.cpu?.cores || []).slice(0, 6).map((core, index) => ({
      label: cpuLabel(core),
      value: fmtPercent(core.percent),
      sub: "개별 사용률",
      color: CORE_COLORS[index % CORE_COLORS.length],
    }))),
  ]);
  renderReadout("memoryReadout", [
    { label: "Usage", value: fmtPercent(memoryPercent(sys.memory)), sub: memoryLabel(sys.memory), color: colors.cyan },
    { label: "Used", value: fmtBytes(sys.memory?.used), sub: `of ${fmtBytes(memoryTotal(sys.memory))}`, color: colors.blue },
    { label: "Available", value: fmtBytes(sys.memory?.available), sub: "실제 MemAvailable", color: colors.good },
  ]);
  renderReadout("swapReadout", [
    { label: "Usage", value: fmtPercent(sys.swap?.percent), sub: `${fmtBytes(sys.swap?.used)} / ${fmtBytes(sys.swap?.total)}`, color: colors.rose },
    { label: "Free", value: fmtBytes(sys.swap?.free), sub: "swap available", color: colors.good },
  ]);
  renderReadout("systemNetworkReadout", [
    { label: "RX", value: fmtBps(net.total_rx_bps), sub: "수신", color: colors.blue },
    { label: "TX", value: fmtBps(net.total_tx_bps), sub: "송신", color: colors.rose },
  ]);
  renderReadout("powerReadout", [
    { label: "Power", value: fmtPower(sys.power?.total_w), sub: sys.power?.source || "ina3221", color: colors.amber },
    { label: "Rails", value: String((sys.power?.rails || []).length || 0), sub: sys.power?.available ? "available" : "unavailable", color: colors.cyan },
  ]);
  renderReadout("networkReadout", [
    { label: "RX", value: fmtBps(net.total_rx_bps), sub: "수신 속도", color: colors.blue },
    { label: "TX", value: fmtBps(net.total_tx_bps), sub: "송신 속도", color: colors.rose },
    { label: "Host", value: net.primary_ipv4 || "--", sub: net.primary_interface || "interface", color: colors.cyan },
  ]);
  renderReadout("dockerCpuReadout", [
    { label: "CPU", value: fmtPercent(docker.summary?.cpu_percent), sub: "컨테이너 합산", color: colors.good },
  ]);
  renderReadout("dockerMemReadout", [
    { label: "Memory", value: fmtPercent(dockerMemPercent), sub: `${fmtBytes(docker.summary?.memory_usage)} / ${fmtBytes(docker.summary?.memory_limit)}`, color: colors.cyan },
  ]);
  renderReadout("fanTempReadout", [
    { label: "Temp", value: fmtTemp(fan.temperature_c), sub: fanTemperatureLabel(fan), color: colors.amber },
    { label: "PWM", value: fan.pwm === null || fan.pwm === undefined ? "--" : `${fan.pwm}`, sub: "수동 출력", color: colors.rose },
  ]);
}

function renderReadout(targetId, rows) {
  const target = $(targetId);
  if (!target) return;
  const renderKey = JSON.stringify(rows.map((row) => [row.label, row.value, row.sub, row.color]));
  if (target.dataset.renderKey === renderKey) return;
  target.dataset.renderKey = renderKey;
  target.innerHTML = "";
  for (const row of rows) {
    const div = document.createElement("div");
    div.className = "readout-row";
    div.innerHTML = `
      <i class="swatch"></i>
      <span>${escapeHtml(row.label)}</span>
      <b>${escapeHtml(row.value)}</b>
      <small>${escapeHtml(row.sub || "")}</small>
    `;
    div.querySelector(".swatch").style.background = row.color;
    div.querySelector(".swatch").style.color = row.color;
    target.append(div);
  }
}

function renderPlan(plan) {
  const target = $("milestoneList");
  if (!target) return;
  target.innerHTML = "";
  for (const item of plan.milestones || []) {
    const div = document.createElement("div");
    div.className = "milestone";
    div.innerHTML = `<strong>${escapeHtml(item.month)}</strong><p>${escapeHtml(item.label)}</p><small class="${escapeHtml(item.status)}">${statusLabel(item.status)}</small>`;
    target.append(div);
  }
}

function renderDocsIndex(docs) {
  const target = $("docsIndex");
  if (!target) return;
  if (!docs.length) {
    target.innerHTML = `<div class="empty-state">표시할 문서가 없습니다.</div>`;
    return;
  }
  target.innerHTML = "";
  const groups = new Map();
  for (const doc of docs) {
    const key = doc.category || "Docs";
    if (!groups.has(key)) groups.set(key, []);
    groups.get(key).push(doc);
  }
  for (const [category, rows] of groups) {
    const section = document.createElement("section");
    section.className = "doc-group";
    const title = document.createElement("h3");
    title.textContent = category;
    section.append(title);
    for (const doc of rows) {
      const link = document.createElement("a");
      link.className = "doc-link";
      link.href = doc.url || `/docs/${doc.slug}`;
      link.dataset.docLink = "true";
      link.dataset.docSlug = doc.slug;
      link.innerHTML = `
        <strong>${escapeHtml(doc.title)}</strong>
        <small>${escapeHtml(doc.summary || doc.source || "")}</small>
      `;
      section.append(link);
    }
    target.append(section);
  }
  updateDocsIndexActive();
}

function renderDocsRoute() {
  if (currentRoute() !== "/docs" || !state.docs.length) return;
  const slug = docSlugFromRoute() || state.docs[0].slug;
  if (location.pathname === "/docs" && slug) history.replaceState({}, "", `/docs/${slug}`);
  updateDocsIndexActive(slug);
  if (state.activeDoc?.slug === slug && state.activeDoc?.markdown !== undefined) {
    renderDocPayload(state.activeDoc);
    return;
  }
  renderDocLoading(slug);
  getJson(`/api/docs/${encodeURIComponent(slug)}`)
    .then((payload) => {
      if (!payload.ok) throw new Error(payload.error || "document_not_found");
      state.activeDoc = payload;
      renderDocPayload(payload);
      updateDocsIndexActive(payload.slug);
    })
    .catch((err) => {
      renderDocError(err.message);
    });
}

function docSlugFromRoute() {
  if (!location.pathname.startsWith("/docs/")) return "";
  return decodeURIComponent(location.pathname.replace(/^\/docs\/?/, "").split("/")[0] || "");
}

function renderDocLoading(slug) {
  const doc = state.docs.find((item) => item.slug === slug);
  text("docCategory", doc?.category || "Docs");
  text("docTitle", doc?.title || "문서 로딩 중");
  text("docSummary", doc?.summary || "Markdown 문서를 불러오고 있습니다.");
  const body = $("docMarkdown");
  if (body) body.innerHTML = `<div class="empty-state">문서를 불러오는 중입니다.</div>`;
}

function renderDocPayload(doc) {
  text("docCategory", doc.category || "Docs");
  text("docTitle", doc.title || doc.slug);
  text("docSummary", doc.summary || doc.source || "");
  const permalink = $("docPermalink");
  if (permalink) {
    permalink.href = doc.url || `/docs/${doc.slug}`;
    permalink.textContent = doc.source || "link";
  }
  const body = $("docMarkdown");
  if (body) body.innerHTML = markdownToHtml(doc.markdown || "");
}

function renderDocError(message) {
  text("docCategory", "Docs");
  text("docTitle", "문서를 열 수 없습니다");
  text("docSummary", message || "알 수 없는 오류");
  const body = $("docMarkdown");
  if (body) body.innerHTML = `<div class="empty-state">${escapeHtml(message || "문서 로딩 실패")}</div>`;
}

function updateDocsIndexActive(slug = docSlugFromRoute()) {
  document.querySelectorAll("[data-doc-link]").forEach((link) => {
    link.classList.toggle("active", link.dataset.docSlug === slug);
  });
}

function markdownToHtml(markdown) {
  const lines = String(markdown || "").replace(/\r\n/g, "\n").split("\n");
  const html = [];
  let i = 0;
  while (i < lines.length) {
    const line = lines[i];
    if (!line.trim()) {
      i += 1;
      continue;
    }
    if (line.startsWith("```")) {
      const language = line.slice(3).trim();
      const code = [];
      i += 1;
      while (i < lines.length && !lines[i].startsWith("```")) {
        code.push(lines[i]);
        i += 1;
      }
      if (i < lines.length) i += 1;
      html.push(`<pre><code class="language-${escapeHtml(language)}">${escapeHtml(code.join("\n"))}</code></pre>`);
      continue;
    }
    if (line.trim() === "$$") {
      const formula = [];
      i += 1;
      while (i < lines.length && lines[i].trim() !== "$$") {
        formula.push(lines[i]);
        i += 1;
      }
      if (i < lines.length) i += 1;
      html.push(`<div class="math-block">${escapeHtml(formula.join("\n"))}</div>`);
      continue;
    }
    const heading = line.match(/^(#{1,4})\s+(.+)$/);
    if (heading) {
      const level = heading[1].length;
      const label = heading[2].trim();
      const id = slugify(label);
      html.push(`<h${level} id="${id}">${formatInline(label)}</h${level}>`);
      i += 1;
      continue;
    }
    if (/^\s*\|.+\|\s*$/.test(line) && i + 1 < lines.length && /^\s*\|?\s*:?-{3,}:?\s*(\|\s*:?-{3,}:?\s*)+\|?\s*$/.test(lines[i + 1])) {
      const header = splitTableRow(line);
      const rows = [];
      i += 2;
      while (i < lines.length && /^\s*\|.+\|\s*$/.test(lines[i])) {
        rows.push(splitTableRow(lines[i]));
        i += 1;
      }
      html.push(renderMarkdownTable(header, rows));
      continue;
    }
    if (/^\s*[-*]\s+/.test(line)) {
      const items = [];
      while (i < lines.length && /^\s*[-*]\s+/.test(lines[i])) {
        items.push(lines[i].replace(/^\s*[-*]\s+/, ""));
        i += 1;
      }
      html.push(`<ul>${items.map((item) => `<li>${formatInline(item)}</li>`).join("")}</ul>`);
      continue;
    }
    if (/^\s*\d+\.\s+/.test(line)) {
      const items = [];
      while (i < lines.length && /^\s*\d+\.\s+/.test(lines[i])) {
        items.push(lines[i].replace(/^\s*\d+\.\s+/, ""));
        i += 1;
      }
      html.push(`<ol>${items.map((item) => `<li>${formatInline(item)}</li>`).join("")}</ol>`);
      continue;
    }
    if (/^\s*>\s?/.test(line)) {
      const quote = [];
      while (i < lines.length && /^\s*>\s?/.test(lines[i])) {
        quote.push(lines[i].replace(/^\s*>\s?/, ""));
        i += 1;
      }
      html.push(`<blockquote>${formatInline(quote.join(" "))}</blockquote>`);
      continue;
    }
    if (/^\s*-{3,}\s*$/.test(line)) {
      html.push("<hr />");
      i += 1;
      continue;
    }
    const paragraph = [];
    while (i < lines.length && lines[i].trim() && !isMarkdownBlockStart(lines[i])) {
      paragraph.push(lines[i].trim());
      i += 1;
    }
    html.push(`<p>${formatInline(paragraph.join(" "))}</p>`);
  }
  return html.join("\n");
}

function isMarkdownBlockStart(line) {
  return line.startsWith("```")
    || line.trim() === "$$"
    || /^(#{1,4})\s+/.test(line)
    || /^\s*[-*]\s+/.test(line)
    || /^\s*\d+\.\s+/.test(line)
    || /^\s*>\s?/.test(line)
    || /^\s*-{3,}\s*$/.test(line);
}

function splitTableRow(line) {
  return line.trim().replace(/^\|/, "").replace(/\|$/, "").split("|").map((cell) => cell.trim());
}

function renderMarkdownTable(header, rows) {
  const head = header.map((cell) => `<th>${formatInline(cell)}</th>`).join("");
  const body = rows.map((row) => `<tr>${row.map((cell) => `<td>${formatInline(cell)}</td>`).join("")}</tr>`).join("");
  return `<div class="table-wrap"><table><thead><tr>${head}</tr></thead><tbody>${body}</tbody></table></div>`;
}

function formatInline(value) {
  let textValue = escapeHtml(value);
  textValue = textValue.replace(/`([^`]+)`/g, "<code>$1</code>");
  textValue = textValue.replace(/!\[([^\]]*)\]\(([^)]+)\)/g, (_, label, href) => {
    const safe = safeDocAssetHref(href);
    return safe ? `<img src="${escapeHtml(safe)}" alt="${label}" loading="lazy" decoding="async" />` : "";
  });
  textValue = textValue.replace(/\[([^\]]+)\]\(([^)]+)\)/g, (_, label, href) => {
    const safe = safeDocHref(href);
    const external = /^https?:\/\//.test(safe) ? ' target="_blank" rel="noreferrer"' : "";
    return `<a href="${escapeHtml(safe)}"${external}>${label}</a>`;
  });
  textValue = textValue.replace(/\*\*([^*]+)\*\*/g, "<strong>$1</strong>");
  textValue = textValue.replace(/\*([^*]+)\*/g, "<em>$1</em>");
  return textValue;
}

function safeDocAssetHref(href) {
  const raw = String(href || "").trim();
  if (/^https:\/\//.test(raw) || raw.startsWith("/docs-assets/")) return raw;
  const normalized = raw.replace(/^\.\//, "").replace(/^docs\//, "");
  if (!normalized.startsWith("assets/") || normalized.includes("..")) return "";
  return `/docs-assets/${normalized.slice("assets/".length).split("/").map(encodeURIComponent).join("/")}`;
}

function safeDocHref(href) {
  const raw = String(href || "").trim();
  if (/^https?:\/\//.test(raw) || raw.startsWith("#") || raw.startsWith("/")) return raw;
  const normalized = raw.replace(/^\.\//, "").replace(/^docs\//, "");
  const doc = state.docs.find((item) => item.source === raw || item.source === `docs/${normalized}` || item.source?.endsWith(`/${normalized}`));
  return doc?.url || "#";
}

function slugify(value) {
  return escapeHtml(String(value || "").trim().toLowerCase().replace(/[^a-z0-9가-힣]+/g, "-").replace(/^-|-$/g, ""));
}

function statusLabel(status) {
  return { done: "완료", in_progress: "진행 중", planned: "예정" }[status] || status;
}

function sensorLabel(key) {
  return { lidar: "LiDAR", camera: "Camera", imu: "IMU", encoder: "Encoder", mavlink: "MAVLink" }[key] || key;
}

function serviceLabel(key) {
  return {
    dashboard: "Dashboard",
    cloudflared: "Cloudflare",
    ros2_bridge: "ROS2 Bridge",
    lidar_bridge: "LiDAR Bridge",
    camera_bridge: "Camera Bridge",
    rover_bridge: "Rover Bridge",
    imu_bridge: "IMU Bridge",
    data_recorder: "Data Recorder",
  }[key] || key;
}

function renderRoverControl(data = state.telemetry || {}) {
  text("roverModeLabel", SIMULATION ? "SIMULATED ROVER" : "LIVE ROVER");
  text("roverControllerLabel", SIMULATION ? "Gazebo" : "ESP8266");
  const rover = data.rover || data.connection?.rover || {};
  const services = data.services || data.connection?.services || {};
  const roverBridge = services.rover_bridge || {};
  // SIM autonomy shares the core lease; its raw state does not identify mode.
  const controllerState = SIMULATION && roverBridge.state === "armed_manual" ? "armed" : roverBridge.state || "대기";
  const autonomy = SIMULATION ? window.SimMap?.autonomyStatus?.() || {} : {};
  const simulatedMode = autonomy.stale ? "상태 수신 대기" : autonomy.active ? "자율주행 중" : autonomy.selected ? "자율주행 대기" : "수동 조종";
  // Recording controls use their ordered status endpoint. General telemetry
  // can arrive late and must not replace a newer START/STOP acknowledgement.
  const imuSensor = data.sensors?.imu || data.connection?.sensors?.imu || {};
  if (!state.imuLive.topicAlive && imuSensor.raw) {
    state.imuLive.raw = imuSensor.raw || {};
    state.imuLive.magnetic = imuSensor.magnetic || {};
    state.imuLive.topicAlive = Boolean(imuSensor.ok);
    state.imuLive.lastImuAgeSec = imuSensor.last_imu_age_s;
  }
  text("roverControlState", roverBridge.serial_open ? "open" : controllerState);
  text("roverControlBase", roverBridge.serial_port || "--");
  text("roverProxyState", SIMULATION ? simulatedMode : rover.mode || roverBridge.mode || "esp_manual");
  text("roverModeCaption", SIMULATION ? "시뮬레이션 · 수동 / 자율주행" : "단일 사용자 수동 조종");
  text("roverRosState", SIMULATION
    ? roverBridge.drive_ready ? "ready" : roverBridge.drive_fault || "시뮬레이션 준비 대기"
    : roverBridge.ok ? "ready" : roverBridge.state || "대기");
  renderStatusRows("roverControlDetails", [
    {
      title: SIMULATION ? "Simulation Controller" : "ESP8266 Bridge",
      sub: SIMULATION ? `가상 로버 · ${simulatedMode}` : "수동 조종 · 단일 USB 소유자",
      ok: Boolean(roverBridge.ok),
      level: roverBridge.ok ? "ok" : "warn",
      value: controllerState,
    },
    {
      title: SIMULATION ? "가상 구동 연결" : "ESP USB",
      sub: SIMULATION ? "Gazebo 좌우 휠" : roverBridge.serial_port || "/dev/ubts-motor",
      ok: Boolean(roverBridge.ok),
      level: roverBridge.ok ? "ok" : "warn",
      value: roverBridge.ok ? "연결됨" : "대기",
    },
    {
      title: SIMULATION ? "자율주행" : "ROS 관측",
      sub: SIMULATION ? "2D LiDAR·점유 지도 기반 경로 추종 · STOP으로 해제" : "자동 주행 입력은 사용하지 않음",
      ok: SIMULATION ? !autonomy.stale : roverBridge.automatic_input_enabled === false,
      level: SIMULATION && autonomy.stale ? "warn" : "ok",
      value: SIMULATION ? simulatedMode : "비활성화",
    },
    {
      title: "조종 상태",
      sub: SIMULATION ? "입력 만료와 가상 휠 정지 감시" : "입력 만료와 MCU 정지 감시",
      ok: true,
      level: "ok",
      value: roverBridge.armed ? "조종 활성화" : "조종 해제",
    },
    {
      title: "ROS2 IMU",
      sub: "/imu/data_raw · sensor-native (raw relay /imu/data)",
      ok: Boolean(imuSensor.ok || services.imu_bridge?.ok),
      level: imuSensor.ok || services.imu_bridge?.ok ? "ok" : "warn",
      value: imuSensor.state || services.imu_bridge?.state || "대기",
    },
    {
      id: "roverRecorderDetail",
      title: "Data Recorder",
      sub: "현재 상태를 확인할 수 없습니다",
      ok: false,
      level: "warn",
      value: "상태 미확인",
    },
  ]);
  renderRoverMotionReadout();
  renderImuLiveState();
  renderRecordingStatus();
}

function initRoverControl() {
  document.querySelectorAll("[data-network-profile]").forEach(el => { el.value = liveNetworkProfile(); el.addEventListener("change", () => setLiveNetworkProfile(el.value)); });
  $("roverArmButton")?.addEventListener("click", armEspRover);
  window.addEventListener("blur", () => { if (state.roverControl.lease || state.roverControl.inFlight) stopRoverNow("focus_lost"); });
  document.addEventListener("visibilitychange", () => { if (document.hidden && (state.roverControl.lease || state.roverControl.inFlight)) stopRoverNow("page_hidden"); });
  document.addEventListener("focusin", (event) => { if (hasActiveRoverInput() && roverKeyTargetBlocked(event)) stopRoverNow("input_focus"); });
  setInterval(() => { if (state.roverControl.lease) { ensureRoverCommandTimer(); } }, 200);

  try {
    const rawSpeed = localStorage.getItem(ROVER_SPEED_KEY);
    const stored = rawSpeed === null ? null : Number(rawSpeed);
    if (Number.isFinite(stored)) state.roverControl.speedPct = Math.max(0, Math.min(100, stored));
    const recordKind = localStorage.getItem(ROVER_RECORD_KIND_KEY);
    const recordImu = localStorage.getItem(IMU_RECORD_ENABLED_KEY);
    const panelCollapsed = localStorage.getItem(ROVER_PANEL_COLLAPSED_KEY);
    const archiveRange = Number(localStorage.getItem(ARCHIVE_RANGE_KEY));
    state.roverControl.panelCollapsed = panelCollapsed === "1";
    if (Number.isFinite(archiveRange)) state.archivePlayback.range = clampLidarRange(archiveRange);
    if (["lidar", "camera", "both"].includes(recordKind)) state.recordings.kind = recordKind;
    state.recordings.includeImu = recordImu === null ? true : recordImu === "1";
  } catch (_) {
    // Speed defaults remain usable without localStorage.
  }
  $("roverSpeedSlider")?.addEventListener("input", (event) => setRoverSpeed(Number(event.target.value), true));
  $("roverStopButton")?.addEventListener("click", () => stopRoverNow("operator_stop"));
  $("roverDrivePanelToggle")?.addEventListener("click", () => setRoverDrivePanelCollapsed(!state.roverControl.panelCollapsed, true));
  $("roverCameraToggle")?.addEventListener("click", toggleCameraLive);
  $("roverLidarToggle")?.addEventListener("click", toggleLidarLive);
  $("roverImuToggle")?.addEventListener("click", toggleImuLive);
  $("roverRecordImuToggle")?.addEventListener("change", (event) => setRecordingImuEnabled(event.target.checked, true));
  $("roverRecordGpsToggle")?.addEventListener("change", (event) => { state.recordings.includeGps = event.target.checked; renderRecordingStatus(); });
  $("roverRecordStartButton")?.addEventListener("click", () => startRecording(state.recordings.kind));
  document.querySelectorAll("[data-record-kind]").forEach((button) => {
    button.addEventListener("click", () => setRecordingKind(button.dataset.recordKind, true));
  });
  $("roverRecordStopButton")?.addEventListener("click", stopRecording);
  $("roverArchiveRefreshButton")?.addEventListener("click", () => loadRecordings().catch(console.error));
  $("roverArchivePlayButton")?.addEventListener("click", toggleArchivePlayback);
  $("roverArchiveScrubber")?.addEventListener("input", (event) => setArchiveTime(Number(event.target.value), { pause: true }));
  $("roverOrchardMapScrubber")?.addEventListener("input", (event) => setArchiveTime(Number(event.target.value), { pause: true }));
  $("roverArchiveSyncToggle")?.addEventListener("change", (event) => {
    state.archivePlayback.sync = Boolean(event.target.checked);
    state.archivePlayback.lastCameraRenderKey = "";
    renderArchiveFrame();
  });

  $("roverArchiveRangeSlider")?.addEventListener("input", (event) => setArchiveRange(Number(event.target.value), true));


  document.querySelectorAll("[data-rover-drive]").forEach((button) => {
    button.addEventListener("pointerdown", (event) => {
      event.preventDefault();
      const action = button.dataset.roverDrive || "stop";
      if (action === "stop") stopRoverNow("pad_stop");
      else startRoverButtonCommand(action);
    });
    button.addEventListener("pointerup", () => releaseRoverInput("pad_release"));
    button.addEventListener("pointercancel", () => releaseRoverInput("pad_cancel"));
    button.addEventListener("pointerleave", () => {
      if (state.roverControl.activeButton) releaseRoverInput("pad_leave");
    });
  });
  document.addEventListener("keydown", handleRoverKeyDown);
  document.addEventListener("keyup", handleRoverKeyUp);
  setRoverSpeed(state.roverControl.speedPct, false);
  setRoverDrivePanelCollapsed(state.roverControl.panelCollapsed, false);
  setRecordingKind(state.recordings.kind, false);
  setRecordingImuEnabled(state.recordings.includeImu, false);

  setArchiveRange(state.archivePlayback.range, false);

  renderRoverButtons();
  renderRecordingStatus();
  renderRecordings();
}

function setRoverSpeed(value, persist) {
  state.roverControl.speedPct = Math.max(0, Math.min(100, Number(value) || 0));
  text("roverSpeedValue", `${Math.round(state.roverControl.speedPct)}%`);
  const slider = $("roverSpeedSlider");
  if (slider) slider.value = String(Math.round(state.roverControl.speedPct));
  if (persist) {
    try {
      localStorage.setItem(ROVER_SPEED_KEY, String(Math.round(state.roverControl.speedPct)));
    } catch (_) {
      // Keep speed session-local when storage is unavailable.
    }
  }
  if (hasActiveRoverInput()) updateRoverMotion();
}

function setRoverDrivePanelCollapsed(collapsed, persist) {
  state.roverControl.panelCollapsed = Boolean(collapsed);
  document.querySelector(".rover-layout")?.classList.toggle("drive-collapsed", state.roverControl.panelCollapsed);
  const button = $("roverDrivePanelToggle");
  if (button) {
    button.textContent = state.roverControl.panelCollapsed ? "조종 패널 보이기" : "조종 패널 숨기기";
    button.classList.toggle("active", state.roverControl.panelCollapsed);
  }
  if (persist) {
    try {
      localStorage.setItem(ROVER_PANEL_COLLAPSED_KEY, state.roverControl.panelCollapsed ? "1" : "0");
    } catch (_) {
      // Keep panel visibility session-local without storage.
    }
  }
}

function setRecordingKind(kind, persist) {
  state.recordings.kind = ["lidar", "camera", "both"].includes(kind) ? kind : "both";
  if (persist) {
    try {
      localStorage.setItem(ROVER_RECORD_KIND_KEY, state.recordings.kind);
    } catch (_) {
      // Kind selection remains session-local without storage.
    }
  }
  document.querySelectorAll("[data-record-kind]").forEach((button) => {
    button.classList.toggle("active", button.dataset.recordKind === state.recordings.kind);
  });
  renderRecordingStatus();
}

function setRecordingImuEnabled(enabled, persist) {
  state.recordings.includeImu = Boolean(enabled);
  const toggle = $("roverRecordImuToggle");
  if (toggle) toggle.checked = state.recordings.includeImu;
  if (persist) {
    try {
      localStorage.setItem(IMU_RECORD_ENABLED_KEY, state.recordings.includeImu ? "1" : "0");
    } catch (_) {
      // IMU recording remains enabled for this session.
    }
  }
  renderRecordingStatus();
}


function setArchiveRange(value, persist) {
  state.archivePlayback.range = clampLidarRange(value);
  const slider = $("roverArchiveRangeSlider");
  if (slider && document.activeElement !== slider) slider.value = String(state.archivePlayback.range);
  text("roverArchiveRangeValue", `${fmtNumber(state.archivePlayback.range, 1)}m`);
  if (persist) {
    try {
      localStorage.setItem(ARCHIVE_RANGE_KEY, String(state.archivePlayback.range));
    } catch (_) {
      // Ignore storage failures.
    }
  }
  state.archivePlayback.lastLidarRenderKey = "";
  renderArchiveFrame();
}


function roverKeyFromEvent(event) {
  const physical = /^Key[WASDQE]$/.test(event.code || "") ? event.code.slice(3) : event.key;
  const key = String(physical || "").toLowerCase();
  return ["w", "a", "s", "d", "q", "e"].includes(key) ? key : "";
}

function roverKeyTargetBlocked(event) {
  if (event.target?.id === "roverSpeedSlider" && event.target?.type === "range") return false;
  return ["INPUT", "TEXTAREA", "SELECT"].includes(event.target?.tagName) || event.target?.isContentEditable;
}

async function armEspRover() {
  resumeLiveCameraAfterArchive();
  const c = state.roverControl;
  if (c.lease) { await stopRoverNow("operator_disarm"); return; }
  if (c.inFlight || c.disarming) return;
  c.inFlight = true;
  c.arming = true;
  renderRoverButtons();
  const epoch = c.epoch || 0;
  try {
    const result = await postJson("/api/rover/arm");
    if (!result.ok) throw new Error(result.error);
    if (epoch !== (c.epoch || 0)) { await postJson("/api/rover/stop", {reason: "cancelled_arm"}); return; }
    c.lease = result.lease; c.ticket = result.ticket; c.seq = result.seq; c.virtualManual = SIMULATION && result.status?.virtual_manual === true;
    c.pendingRelease = false;
    setRoverCommandStatus("ok", "조종 활성화 · WASD 이동 · Q/E 속도 −5/+5% · Esc 즉시 정지");
    ensureRoverCommandTimer();
    renderRoverMotionReadout();
  } catch (err) { setRoverCommandStatus("bad", err.message); }
  finally { c.inFlight = false; c.arming = false; renderRoverButtons(); }
}

function handleRoverKeyDown(event) {
  if (currentRoute() !== "/rover-control") return;
  if (event.key === "Escape") { stopRoverNow("escape"); return; }
  if (roverKeyTargetBlocked(event) || event.ctrlKey || event.altKey || event.metaKey) return;
  const c = state.roverControl;
  if (!c.lease) return;
  const key = roverKeyFromEvent(event);
  if (!key) return;
  event.preventDefault();
  if (event.repeat) return;
  if (key === "q" || key === "e") {
    if (c.speedKeys.has(key)) return;
    c.speedKeys.add(key);
    setRoverSpeed(c.speedPct + (key === "q" ? -5 : 5), true);
    return;
  }
  if (c.activeKeys.has(key)) return;
  c.activeKeys.add(key);
  c.activeButton = "";
  updateRoverMotion();
}
function handleRoverKeyUp(event) {
  const key = roverKeyFromEvent(event);
  if (key === "q" || key === "e") { state.roverControl.speedKeys.delete(key); return; }
  if (!key || !state.roverControl.activeKeys.has(key)) return;
  state.roverControl.activeKeys.delete(key);
  if (hasActiveRoverInput()) updateRoverMotion();
  else releaseRoverInput("key_release");
}
function startRoverButtonCommand(action) {
  if (!state.roverControl.lease) { setRoverCommandStatus("bad", "먼저 조종을 활성화하세요."); return; }
  state.roverControl.activeButton = action;
  state.roverControl.activeKeys.clear();
  updateRoverMotion();
}
function hasActiveRoverInput() {
  return Boolean(state.roverControl.activeButton || state.roverControl.activeKeys.size);
}
function roverMotionVector() {
  const c = state.roverControl;
  const keys = c.activeKeys;
  let forward = (keys.has("w") ? 1 : 0) - (keys.has("s") ? 1 : 0);
  let turn = (keys.has("d") ? 1 : 0) - (keys.has("a") ? 1 : 0);
  if (c.activeButton) {
    forward = c.activeButton === "forward" ? 1 : c.activeButton === "backward" ? -1 : 0;
    turn = c.activeButton === "right" ? 1 : c.activeButton === "left" ? -1 : 0;
  }
  const speed = c.speedPct;
  if (forward && turn) {
    // Keep the inside wheel at the selected speed in both travel directions.
    const outside = Math.min(100, speed * 1.5);
    return {left: Math.round(forward * (turn > 0 ? outside : speed)),
      right: Math.round(forward * (turn < 0 ? outside : speed))};
  }
  return {left: Math.round((forward + turn) * speed), right: Math.round((forward - turn) * speed)};
}
function ensureRoverCommandTimer() {
  if (!state.roverControl.commandTimer) state.roverControl.commandTimer = setInterval(stepRoverMotion, state.roverControl.virtualManual === true ? 40 : 140);
}
function clearRoverCommandTimer() {
  clearInterval(state.roverControl.commandTimer);
  state.roverControl.commandTimer = null;
}
function updateRoverMotion() {
  state.roverControl.pendingRelease = false;
  state.roverControl.inputDirty = true;
  ensureRoverCommandTimer();
  renderRoverButtons();
  stepRoverMotion();
}
async function stepRoverMotion() {
  const c = state.roverControl;
  if (!c.lease || c.inFlight) return;
  if (document.hidden || currentRoute() !== "/rover-control") { stopRoverNow("page_inactive"); return; }
  const motion = roverMotionVector();
  const op = c.pendingRelease ? "release" : hasActiveRoverInput() ? "command" : "keepalive";
  c.pendingRelease = false;
  c.inputDirty = false;
  c.inFlight = true;
  const epoch = c.epoch || 0;
  const sentLease = c.lease;
  try {
    const result = await postJson(`/api/rover/${op}`, {lease: sentLease, ticket: c.ticket, seq: c.seq + 1,
      left: motion.left, right: motion.right, deadman: op === "command"});
    if (epoch !== (c.epoch || 0)) return;
    if (SIMULATION && result.ok === false && result.error === "autonomy_active") {
      // The Adapter rejected this manual request before dispatch. Drop only
      // this window's obsolete credentials; another client's mission may run.
      if (c.lease !== sentLease) return;
      c.epoch = epoch + 1;
      c.lease = c.ticket = null;
      c.activeButton = ""; c.activeKeys.clear(); c.speedKeys.clear();
      c.inputDirty = false; c.pendingRelease = false; c.left = c.right = 0;
      c.linear = c.angular = c.currentLinear = c.currentAngular = 0;
      c.stoppingWithFade = false;
      clearRoverCommandTimer(); renderRoverButtons(); renderRoverMotionReadout();
      setRoverCommandStatus("bad", "이 창의 조종 권한이 종료되었습니다.");
      return;
    }
    if (!result.ok) throw new Error(result.error || "control rejected");
    c.ticket = result.ticket; c.seq = result.seq;
    c.left = motion.left; c.right = motion.right;
    renderRoverMotionReadout();
  } catch (err) {
    if (epoch === (c.epoch || 0)) { stopRoverNow("command_failed"); setRoverCommandStatus("bad", `정지 · ${err.message} · 다시 활성화하세요.`); }
  } finally {
    c.inFlight = false;
    // Flush the latest input as soon as the previous request returns, without a queue.
    if (epoch === (c.epoch || 0) && c.inputDirty) stepRoverMotion();
  }
}
function releaseRoverInput(reason = "release") {
  const c = state.roverControl;
  c.activeButton = ""; c.activeKeys.clear(); c.pendingRelease = true;
  c.inputDirty = true;
  renderRoverButtons(); stepRoverMotion();
}
async function stopRoverNow(reason = "operator_stop") {
  const c = state.roverControl;
  c.epoch = (c.epoch || 0) + 1;
  const epoch = c.epoch;
  c.disarming = true;
  c.lease = c.ticket = null;
  c.activeButton = ""; c.activeKeys.clear();
  c.speedKeys.clear(); c.inputDirty = false;
  c.pendingRelease = false; c.left = c.right = 0;
  c.linear = c.angular = c.currentLinear = c.currentAngular = 0;
  c.stoppingWithFade = false;
  clearRoverCommandTimer(); renderRoverButtons(); renderRoverMotionReadout();
  setRoverCommandStatus("testing", "정지 요청 중 · 조종 해제");
  try {
    const result = await postJson("/api/rover/stop", {reason});
    setRoverCommandStatus(result.ok ? "ok" : "bad", result.ok ? "정지 요청 접수 · 다시 조종하려면 활성화하세요." : result.error);
  } catch (_) { setRoverCommandStatus("bad", "응답 없음 · 입력 만료 정지 대기 · 연결 복구 후 상태를 확인하세요."); }
  finally { if (epoch === c.epoch) c.disarming = false; renderRoverButtons(); }
}
function roverCommandMessage(payload) { return payload.ok ? (SIMULATION ? "가상 로버 요청 접수" : "ESP 요청 접수") : payload.error || "제어 실패"; }

function setRoverCommandStatus(kind, message) {
  const target = $("roverCommandStatus");
  if (!target) return;
  target.className = `test-result ${kind}`;
  target.textContent = message;
}

function renderRoverButtons() {
  const armButton = $("roverArmButton");
  if (armButton) {
    const c = state.roverControl;
    armButton.textContent = c.arming ? "활성화 중…" : c.disarming ? "해제 중…" : c.lease ? "조종 활성화 ON" : "조종 활성화 OFF";
    armButton.setAttribute("aria-pressed", String(Boolean(c.lease)));
    armButton.disabled = Boolean(c.arming || c.disarming || (SIMULATION && window.SimMap?.autonomySelected?.()));
    armButton.classList.toggle("active", Boolean(c.lease));
  }
  document.querySelectorAll("[data-rover-drive]").forEach((button) => {
    const action = button.dataset.roverDrive || "";
    const activeByKey = (action === "forward" && state.roverControl.activeKeys.has("w"))
      || (action === "backward" && state.roverControl.activeKeys.has("s"))
      || (action === "left" && state.roverControl.activeKeys.has("a"))
      || (action === "right" && state.roverControl.activeKeys.has("d"));
    button.classList.toggle("active", Boolean(action && (action === state.roverControl.activeButton || activeByKey)));
  });
}

function renderRoverMotionReadout() {
  const c = state.roverControl;
  const colors = chartColors();
  renderReadout("roverMotionReadout", [
    {label: SIMULATION ? "왼쪽 속도 요청" : "왼쪽 요청 PWM", value: `${c.left || 0}%`, sub: SIMULATION ? "시뮬레이션 설정 속도의 비율" : "실제 속도 측정값 아님", color: colors.cyan},
    {label: SIMULATION ? "오른쪽 속도 요청" : "오른쪽 요청 PWM", value: `${c.right || 0}%`, sub: SIMULATION ? "가상 휠 가감속 적용" : "MCU 가감속 적용", color: colors.blue},
    {label: "조종", value: c.lease ? "활성" : "해제", sub: "입력 만료 0.75초 · Esc 즉시 정지", color: colors.good},
  ]);
}

async function refreshRecordingStatus() {
  if (state.recordings.statusInFlight || state.recordings.actionInFlight) return;
  const ticket = state.recordings.statusRequestId = (state.recordings.statusRequestId || 0) + 1;
  state.recordings.statusInFlight = ticket;
  const requestedAt = performance.now();
  state.recordings.lastStatusPollSteadyAt = requestedAt;
  const controller = state.recordings.statusController = new AbortController();
  const timeout = setTimeout(() => {
    if (ticket !== state.recordings.statusRequestId) return;
    invalidateRecordingStatusRead();
    state.recordings.status = {ok: false, state: "unavailable", error: "status_timeout"};
    state.recordings.lastStatusAt = Date.now();
    renderRecordingStatus();
  }, 4000);
  try {
    const status = await getJson("/api/recordings/status", {signal: controller.signal});
    if (ticket !== state.recordings.statusRequestId) return;
    state.recordings.status = status;
    state.recordings.lastStatusAt = Date.now();
    state.recordings.lastStatusSteadyAt = requestedAt;
    renderRecordingStatus();
  } catch (err) {
    if (ticket !== state.recordings.statusRequestId) return;
    state.recordings.status = { ok: false, state: "unavailable", error: err.message };
    state.recordings.lastStatusAt = Date.now();
    renderRecordingStatus();
  } finally {
    clearTimeout(timeout);
    if (state.recordings.statusInFlight === ticket) {
      state.recordings.statusInFlight = null;
      state.recordings.statusController = null;
    }
  }
}

function invalidateRecordingStatusRead() {
  state.recordings.statusRequestId = (state.recordings.statusRequestId || 0) + 1;
  state.recordings.statusController?.abort();
  state.recordings.statusController = null;
  state.recordings.statusInFlight = null;
}

function maybeRefreshRecordingStatus() {
  if (currentRoute() !== "/rover-control") return;
  renderRecordingStatus();
  const now = performance.now();
  const last = state.recordings.lastStatusPollSteadyAt;
  if (!Number.isFinite(now)) return;
  if (Number.isFinite(last) && now >= last && now - last < 2200) return;
  refreshRecordingStatus().catch((err) => debugLog("recording", "status refresh failed", err));
}

async function startRecording(kind) {
  if (state.recordings.actionInFlight) return;
  state.recordings.actionInFlight = true;
  invalidateRecordingStatusRead();
  const requestedAt = performance.now();
  renderRecordingStatus("기록 시작 중");
  try {
	    const payload = await postJson("/api/recordings/start", {
	      kind: kind || state.recordings.kind,
	      include_imu: state.recordings.includeImu,
	      include_gps: state.recordings.includeGps,
	    });
	    state.recordings.status = payload.status || payload;
	    state.recordings.lastStatusAt = Date.now();
    state.recordings.lastStatusSteadyAt = requestedAt;
    renderRecordingStatus();
    await loadRecordings();
  } catch (err) {
    state.recordings.status = { ok: false, state: "start_failed", error: err.message };
    renderRecordingStatus();
  } finally {
    state.recordings.actionInFlight = false;
    renderRecordingStatus();
  }
}

async function stopRecording() {
  if (state.recordings.actionInFlight) return;
  state.recordings.actionInFlight = true;
  invalidateRecordingStatusRead();
  const requestedAt = performance.now();
  renderRecordingStatus("기록 중지 중");
  try {
    const payload = await postJson("/api/recordings/stop", {});
    state.recordings.status = payload.status || payload;
    state.recordings.lastStatusAt = Date.now();
    state.recordings.lastStatusSteadyAt = requestedAt;
    renderRecordingStatus();
    await loadRecordings();
  } catch (err) {
    state.recordings.status = { ok: false, state: "stop_failed", error: err.message };
    renderRecordingStatus();
  } finally {
    state.recordings.actionInFlight = false;
    renderRecordingStatus();
  }
}

function recordingKindLabel(kind) {
  return { lidar: "LiDAR", camera: "Camera", both: "LiDAR+Camera" }[kind] || kind || "대기";
}

function recordingImuFrameCount(payload) {
  const value = payload?.timeline?.imu_frames
    ?? payload?.timeline_summary?.imu_frames
    ?? payload?.metadata?.timeline?.imu_frames
    ?? 0;
  const count = Number(value);
  return Number.isFinite(count) ? Math.max(0, count) : 0;
}

function recordingSensorLabel(payload) {
  const kind = payload?.kind || payload?.metadata?.kind;
  const label = recordingKindLabel(kind);
  const sensors = recordingImuFrameCount(payload) > 0 ? `${label}+IMU` : label;
  const gpsCount = payload?.timeline?.gps_frames ?? payload?.timeline_summary?.gps_frames ?? payload?.metadata?.timeline?.gps_frames ?? 0;
  return Number(gpsCount) > 0 ? `${sensors}+GPS` : sensors;
}

function renderRecordingStatus(override) {
  const status = state.recordings.status || {};
  const ageMs = performance.now() - state.recordings.lastStatusSteadyAt;
  const known = status.ok === true && status.status_fresh !== false
    && typeof status.active === "boolean"
    && Number.isFinite(ageMs) && ageMs >= 0 && ageMs <= 5000;
  const active = known && status.active;
  // This secondary row shares the exact primary freshness gate. It must age
  // locally even when general telemetry and recorder requests stop arriving.
  const detail = $("roverRecorderDetail");
  if (detail) {
    detail.querySelector(".status-dot").className = `status-dot ${known ? "ok" : "warn"}`;
    detail.querySelector("small").textContent = known
      ? status.recording_id || status.status_url || "ros2-data-recorder"
      : "현재 상태를 확인할 수 없습니다";
    detail.querySelector("b").textContent = status.state === "disabled" ? "disabled"
      : !known ? "상태 미확인"
      : active ? `${recordingKindLabel(status.kind)} 기록 중` : status.state || "ready";
  }
  const kind = recordingKindLabel(active ? status.kind : state.recordings.kind);
  const includeImu = active ? status.include_imu !== false : state.recordings.includeImu;
  const includeGps = active ? status.include_gps !== false : state.recordings.includeGps;
  const sensorLabel = `${kind}${includeImu ? "+IMU" : ""}${includeGps ? "+GPS" : ""}`;
  const duration = recordingDurationSeconds(status);
  const message = override || (!known && status.state !== "disabled"
    ? "기록 상태 확인 중 · 연결을 확인하세요"
    : active
    ? `${sensorLabel} 기록 중 · ${status.recording_id || ""}`
    : status.error || status.state || `${sensorLabel} 기록 대기`);
  const target = $("roverRecordingStatus");
  if (target) {
    target.className = `test-result ${known ? "ok" : status.error ? "bad" : "testing"}`;
    target.textContent = message;
  }
  document.querySelectorAll("[data-record-kind]").forEach((button) => {
    button.disabled = state.recordings.actionInFlight || active || !known;
    button.classList.toggle("active", (active ? status.kind : state.recordings.kind) === button.dataset.recordKind);
  });
  const start = $("roverRecordStartButton");
  if (start) {
    start.disabled = state.recordings.actionInFlight || active || !known || status.state === "disabled";
    start.textContent = status.state === "disabled" ? "녹화는 후속 단계에서 활성화" : active ? "기록 중" : `${recordingKindLabel(state.recordings.kind)} 기록 시작`;
  }
  const stop = $("roverRecordStopButton");
  if (stop) stop.disabled = state.recordings.actionInFlight || (known && !active) || status.state === "disabled";
  text("roverRecordingTimer", !known ? "--:--" : active ? fmtClock(duration) : "00:00");
  text("roverRecorderState", status.state === "disabled" ? "disabled" : !known ? "상태 미확인" : active ? "recording" : "ready");
  text("roverRecorderKind", active ? sensorLabel : status.archive_count !== undefined ? `${status.archive_count} archives` : "대기");
  const imuToggle = $("roverRecordImuToggle");
  const gpsToggle = $("roverRecordGpsToggle");
  if (gpsToggle) { gpsToggle.checked = includeGps; gpsToggle.disabled = state.recordings.actionInFlight || active || !known; }
  if (imuToggle) {
    imuToggle.checked = includeImu;
    imuToggle.disabled = state.recordings.actionInFlight || active || !known;
  }
}

function recordingDurationSeconds(status) {
  if (Number.isFinite(Number(status.duration_s))) return Number(status.duration_s);
  if (!status.started_at) return 0;
  const started = new Date(status.started_at).getTime();
  if (!Number.isFinite(started)) return 0;
  return Math.max(0, (Date.now() - started) / 1000);
}

async function loadRecordings() {
  state.recordings.loading = true;
  renderRecordings();
  await refreshRecordingStatus();
  try {
    const payload = await getJson("/api/recordings");
    state.recordings.items = payload.recordings || [];
    // Fetch archive frames and maps only after an explicit selection.
    renderRecordings();
    if (state.recordings.selectedId) await selectRecording(state.recordings.selectedId);
  } finally {
    state.recordings.loading = false;
    renderRecordings();
  }
}

async function selectRecording(id) {
  state.recordings.selectedId = id;
  window.SimMap?.selectArchive(id);
  stopArchivePlayback(true);
  state.recordings.timeline = null;
  clearArchiveFrameCache();
  renderArchiveMotor(null, [], 0);

  renderRecordings();
  try {
    const detail = await getJson(`/api/recordings/${encodeURIComponent(id)}`);
    if (id !== state.recordings.selectedId) return;
    state.recordings.selected = detail;
    state.recordings.timeline = timelineShellFromRecording(detail?.recording || detail);
    renderRecordingDetail(detail);
    if (state.recordings.timeline) {
      await scheduleArchiveSegmentsForTime(0, { force: true, reschedule: true }).catch((err) => debugLog("archive", "first segment load failed", err));
      renderArchiveFrame();

    }
  } catch (err) {
    if (id !== state.recordings.selectedId) return;
    renderStatusRows("roverArchiveDetails", [
      { title: "Archive", sub: id, ok: false, level: "bad", value: err.message },
    ]);
  }
}


function timelineShellFromRecording(payload) {
  const summary = payload?.timeline_summary || {};
  const metadata = payload?.metadata || {};
  const durationS = Number(summary.duration_s || metadata.duration_s || payload?.duration_s || 0);
  const lidarFrames = Number(summary.lidar_frames || 0);
  const cameraFrames = Number(summary.camera_frames || 0);
  const imuFrames = Number(summary.imu_frames || 0);
  const gpsFrames = Number(summary.gps_frames || 0);
  const motorFrames = Number(summary.motor_frames || 0);
  if (!summary.ok && !lidarFrames && !cameraFrames && !imuFrames && !gpsFrames && !motorFrames) return null;
  return {
    ok: true,
    id: payload?.id || metadata.id || state.recordings.selectedId,
    kind: payload?.kind || metadata.kind,
    algorithm: summary.algorithm,
    coordinate_frame: summary.coordinate_frame || payload?.preview_scan?.coordinate_frame || "",
    duration_s: durationS,
    lidar: { frames: [] },
    camera: { frames: [] },
    imu: { frames: [], calibration: metadata.timeline?.imu_calibration || {} },
    gps: {frames: [], enabled: metadata.include_gps === true || gpsFrames > 0},
    motor: {frames: [], enabled: motorFrames > 0, frame_count: motorFrames},
    sync: {
      time_base: "seconds_since_recording_start",
      lidar_frames: lidarFrames,
      camera_frames: cameraFrames,
      imu_frames: imuFrames,
      gps_frames: gpsFrames,
      motor_frames: motorFrames,
    },
    streaming: true,
  };
}

function clearArchiveFrameCache() {
  const cache = state.archivePlayback.cache;
  if (state.archivePlayback.seekTimer) {
    clearTimeout(state.archivePlayback.seekTimer);
    state.archivePlayback.seekTimer = null;
  }
  if (cache?.objectUrls) {
    for (const url of cache.objectUrls.values()) {
      if (url && url.startsWith("blob:")) URL.revokeObjectURL(url);
    }
  }
  state.archivePlayback.cache = {
    lidarFrames: [],
    cameraFrames: [],
    imuFrames: [],
    gpsFrames: [],
    motorFrames: [],
    motorKeys: new Set(),
    lidarKeys: new Set(),
    cameraKeys: new Set(),
    imuKeys: new Set(),
    gpsKeys: new Set(),
    objectUrls: new Map(),
    imageElements: new Map(),
  };
  for(const task of state.archivePlayback.loadingTelemetry?.values()||[])task.controller.abort();
  state.archivePlayback.telemetrySegments = new Map();
  state.archivePlayback.loadingTelemetry = new Map();
  state.archivePlayback.segmentCache = new Map();
  for(const task of state.archivePlayback.loadingSegments?.values()||[])task.controller?.abort();
  state.archivePlayback.loadingSegments = new Map();
  state.archivePlayback.lastSegmentWindowKey = "";
  state.archivePlayback.lastPlaybackRenderAt = 0;
  state.archivePlayback.lastLidarRenderKey = "";
  state.archivePlayback.lastCameraRenderKey = "";
}

function prepareArchiveTimelineCache(timeline) {
  clearArchiveFrameCache();
  mergeArchiveTimelineSegment(timeline);
}

function expandArchiveLidarPoints(points, fields) {
  if (!Array.isArray(points) || !points.length || !Array.isArray(points[0])) return points || [];
  const names = Array.isArray(fields) && fields.length
    ? fields
    : ["x_m", "y_m", "distance_m", "quality", "angle_deg"];
  return points.map((values) => {
    const point = Object.fromEntries(names.map((name, index) => [name, values[index]]));
    if (!names.includes("x_m") && Number.isFinite(point.angle_deg) && Number.isFinite(point.distance_m)) {
      const angle = point.angle_deg * Math.PI / 180;
      point.x_m = Math.sin(angle) * point.distance_m;
      point.y_m = Math.cos(angle) * point.distance_m;
    }
    return point;
  });
}

function mergeArchiveTimelineSegment(timeline) {
  if (!timeline) return;
  const cache = state.archivePlayback.cache;
  const lidarPointFields = timeline.playback?.lidar_point_fields || [];
  if (state.recordings.timeline && timeline.imu) {
    state.recordings.timeline.imu = {
      ...(state.recordings.timeline.imu || {}),
      ...Object.fromEntries(Object.entries(timeline.imu).filter(([key]) => key !== "frames")),
      frames: [],
    };
  }
  const lidarFrames = (timeline.lidar?.frames || []).map((frame, index) => {
    const points = expandArchiveLidarPoints(frame.points, lidarPointFields);
    return {
      ...frame,
      index,
      t_s: Number(frame.t_s) || 0,
      points,
      result: normalizeArchiveLidarResult({
        ok: true,
        state: "archive_timeline",
        frame_id: frame.frame_id || "laser",
        stamp: frame.stamp,
        points,
        coordinate_frame: frame.coordinate_frame || timeline.coordinate_frame,
        metrics: frame.metrics || {},
        source: "archive_timeline",
      }, frame.overlay, frame),
    };
  });
  for (const frame of lidarFrames) {
    const key = archiveFrameKey(frame, "lidar");
    if (cache.lidarKeys.has(key)) continue;
    cache.lidarKeys.add(key);
    frame.cacheKey = key;
    cache.lidarFrames.push(frame);
  }
  cache.lidarFrames.sort((a, b) => a.t_s - b.t_s);
  cache.lidarFrames.forEach((frame, cacheIndex) => {
    frame.cacheIndex = cacheIndex;
  });
  const cameraFrames = (timeline.camera?.frames || []).map((frame, index) => ({
    ...frame,
    index,
    t_s: Number(frame.t_s) || 0,
  }));
  for (const frame of cameraFrames) {
    const key = archiveFrameKey(frame, "camera");
    if (cache.cameraKeys.has(key)) {
      const existing = cache.cameraFrames.find((item) => item.cacheKey === key);
      if (existing) Object.assign(existing, frame, { cacheKey: key });
      continue;
    }
    cache.cameraKeys.add(key);
    frame.cacheKey = key;
    cache.cameraFrames.push(frame);
  }
  cache.cameraFrames.sort((a, b) => a.t_s - b.t_s);
  cache.cameraFrames.forEach((frame, cacheIndex) => {
    frame.cacheIndex = cacheIndex;
  });
  const imuFrames = (timeline.imu?.frames || []).map((frame, index) => ({
    ...frame,
    index,
    t_s: Number(frame.t_s) || 0,
  }));
  for (const frame of imuFrames) {
    const key = archiveFrameKey(frame, "imu");
    if (cache.imuKeys.has(key)) continue;
    cache.imuKeys.add(key);
    frame.cacheKey = key;
    cache.imuFrames.push(frame);
  }
  cache.imuFrames.sort((a, b) => a.t_s - b.t_s);
  cache.imuFrames.forEach((frame, cacheIndex) => { frame.cacheIndex = cacheIndex; });
  cache.gpsFrames ||= []; cache.gpsKeys ||= new Set();
  for (const frame of timeline.gps?.frames || []) {
    const key = `${Number(frame.t_s).toFixed(6)}:${frame.sample?.sequence ?? frame.sample?.state}`;
    if (!cache.gpsKeys.has(key)) { cache.gpsKeys.add(key); cache.gpsFrames.push({...frame, t_s:Number(frame.t_s)}); }
  }
  cache.gpsFrames.sort((a, b) => a.t_s-b.t_s);
  mergeArchiveMotorSegment(timeline.motor);
  trimArchiveTimelineCache();
  state.archivePlayback.lastLidarRenderKey = "";
  state.archivePlayback.lastCameraRenderKey = "";
}

function trimArchiveTimelineCache() {
  const cache = state.archivePlayback.cache;
  const duration = Math.max(1, Number(state.archivePlayback.segmentDurationS) || ARCHIVE_SEGMENT_SECONDS);
  const center = Math.floor(Math.max(0, state.archivePlayback.timeS || 0) / duration) * duration;
  const first = Math.max(0, center - 2 * duration), last = center + 4 * duration;
  for (const kind of ["lidar", "camera", "imu"]) {
    const name = `${kind}Frames`, keys = `${kind}Keys`;
    const lower=kind==='imu'?Math.max(0,center-24):first,upper=kind==='imu'?center+48:last;
    cache[name] = (cache[name] || []).filter(frame => frame.t_s >= lower && frame.t_s < upper);
    cache[keys] = new Set(cache[name].map(frame => frame.cacheKey));
    cache[name].forEach((frame, index) => { frame.cacheIndex = index; });
  }
  for (const [key, value] of state.archivePlayback.segmentCache) {
    if (value.start_s + value.duration_s <= first || value.start_s >= last) state.archivePlayback.segmentCache.delete(key);
  }
  for (const [key, url] of cache.objectUrls) {
    if (!cache.cameraKeys.has(key)) {
      if (url?.startsWith("blob:")) URL.revokeObjectURL(url);
      cache.objectUrls.delete(key);
    }
  }
  for (const key of cache.imageElements.keys()) {
    if (!cache.cameraKeys.has(key)) cache.imageElements.delete(key);
  }
}

function archiveFrameKey(frame, kind) {
  const stamp = frame?.stamp ? `${frame.stamp.sec || 0}.${frame.stamp.nanosec || 0}` : "";
  const size = kind === "camera" ? String(frame?.frame_url || frame?.data || "").length : (frame?.points || []).length;
  return `${kind}:${Number(frame?.t_s || 0).toFixed(3)}:${stamp}:${size}`;
}

async function ensureArchiveSegmentForTime(timeS, options = {}) {
  const id = state.recordings.selectedId;
  const timeline = state.recordings.timeline;
  if (!id || !timeline) return;
  ensureArchiveTelemetryForTime(timeS).catch(err=>debugLog('archive','telemetry load failed',err));
  const segmentDuration = Math.max(1, Number(state.archivePlayback.segmentDurationS) || ARCHIVE_SEGMENT_SECONDS);
  const startS = Math.max(0, Math.floor((Number(timeS) || 0) / segmentDuration) * segmentDuration);
  await loadArchiveTimelineSegment(id, startS, segmentDuration, options);
  const nextStart = startS + segmentDuration;
  const cameraBootstrapNeeded = !archiveNativeVideo() && (state.archivePlayback.cache.cameraFrames?.length || 0) === 0
    && Number(timeline.sync?.camera_frames || 0) > 0;
  if (cameraBootstrapNeeded && nextStart < state.archivePlayback.durationS + 0.25) {
    await loadArchiveTimelineSegment(id, nextStart, segmentDuration);
  }
  const nearSegmentEnd = Number(timeS) + 2 >= nextStart;
  if ((state.archivePlayback.playing || nearSegmentEnd) && nextStart < state.archivePlayback.durationS + 0.25) {
    loadArchiveTimelineSegment(id, nextStart, segmentDuration).catch((err) => debugLog("archive", "segment prefetch failed", err));
  }
  // One LiDAR look-ahead leaves bandwidth for the priority IMU/map/video paths.
}

async function ensureArchiveTelemetryForTime(timeS) {
  const id=state.recordings.selectedId;if(!id||!state.recordings.timeline)return;
  const start=Math.max(0,Math.floor(Number(timeS)/24)*24);
  const current=loadArchiveTelemetrySegment(id,start);
  if(Number(timeS)>=start+16 && start+24<state.archivePlayback.durationS)
    current.then(()=>loadArchiveTelemetrySegment(id,start+24)).catch(err=>debugLog('archive','telemetry prefetch failed',err));
  return current;
}

async function loadArchiveTelemetrySegment(id,start) {
  const key=`${id}:${start}`;
  const playback=state.archivePlayback;
  if(playback.telemetrySegments.has(key))return;
  if(playback.loadingTelemetry.has(key))return playback.loadingTelemetry.get(key).promise;
  for(const [other,task] of playback.loadingTelemetry){if(other!==key&&Math.abs(task.start-start)>24){task.controller.abort();playback.loadingTelemetry.delete(other);}}
  const controller=new AbortController();
  const promise=getJson(`/api/recordings/${encodeURIComponent(id)}/timeline?start_s=${start}&duration_s=24&profile=dashboard-telemetry-v1`,{signal:controller.signal})
    .then(payload=>{
      if(controller.signal.aborted||id!==state.recordings.selectedId)return;
      if(!payload.ok||!payload.timeline)throw new Error(payload.error||'telemetry unavailable');
      mergeArchiveTimelineSegment(payload.timeline);playback.telemetrySegments.set(key,start);
      for(const [old,t] of playback.telemetrySegments)if(Math.abs(t-start)>24)playback.telemetrySegments.delete(old);
      renderArchiveFrame();
    }).finally(()=>{if(playback.loadingTelemetry.get(key)?.controller===controller)playback.loadingTelemetry.delete(key);});
  playback.loadingTelemetry.set(key,{controller,promise,start});return promise;
}

function archiveSegmentWindowKey(timeS) {
  const id = state.recordings.selectedId;
  if (!id || !state.recordings.timeline) return "";
  const duration = Math.max(1, Number(state.archivePlayback.segmentDurationS) || ARCHIVE_SEGMENT_SECONDS);
  const start = Math.max(0, Math.floor((Number(timeS) || 0) / duration) * duration);
  return `${id}:${start.toFixed(3)}:${duration.toFixed(3)}`;
}

function scheduleArchiveSegmentsForTime(timeS, options = {}) {
  const key = archiveSegmentWindowKey(timeS);
  if (!key) return Promise.resolve();
  if (!options.reschedule && state.archivePlayback.lastSegmentWindowKey === key) return Promise.resolve();
  state.archivePlayback.lastSegmentWindowKey = key;
  return ensureArchiveSegmentForTime(timeS, { force: Boolean(options.force) })
    .catch((error) => {
      if (state.archivePlayback.lastSegmentWindowKey === key) state.archivePlayback.lastSegmentWindowKey = "";
      throw error;
    });
}

function scheduleArchiveSeek(timeS) {
  const duration=state.archivePlayback.segmentDurationS||ARCHIVE_SEGMENT_SECONDS;
  const start=Math.floor(Number(timeS)/duration)*duration;
  for(const task of state.archivePlayback.loadingSegments.values())if(task.start<start-duration||task.start>start+duration)task.controller.abort();
  ensureArchiveTelemetryForTime(timeS).catch(err=>debugLog('archive','seek telemetry load failed',err));
  if (state.archivePlayback.seekTimer) clearTimeout(state.archivePlayback.seekTimer);
  state.archivePlayback.seekTimer = setTimeout(() => {
    state.archivePlayback.seekTimer = null;
    scheduleArchiveSegmentsForTime(timeS, { reschedule: true })
      .then(() => renderArchiveFrame())
      .catch((err) => debugLog("archive", "seek segment load failed", err));
  }, 90);
}

async function loadArchiveTimelineSegment(id, startS, durationS, options = {}) {
  const start = Math.max(0, Number(startS) || 0);
  const duration = Math.max(1, Number(durationS) || ARCHIVE_SEGMENT_SECONDS);
  const key = `${id}:${start.toFixed(3)}:${duration.toFixed(3)}`;
  if (!options.force && state.archivePlayback.segmentCache.has(key)) return state.archivePlayback.segmentCache.get(key);
  if (state.archivePlayback.loadingSegments.has(key)) return state.archivePlayback.loadingSegments.get(key).promise;
  const params = new URLSearchParams({
    start_s: start.toFixed(3),
    duration_s: duration.toFixed(3),
    profile: ARCHIVE_PLAYBACK_PROFILE,
  });
  const controller=new AbortController();
  const promise = getJson(`/api/recordings/${encodeURIComponent(id)}/timeline?${params.toString()}`,{signal:controller.signal})
    .then((payload) => {
      if (controller.signal.aborted || id !== state.recordings.selectedId) return payload;
      if (payload?.ok && payload.timeline) {
        if (!state.recordings.timeline) state.recordings.timeline = payload.timeline;
        if (payload.timeline.coordinate_frame && !state.recordings.timeline.coordinate_frame) {
          state.recordings.timeline.coordinate_frame = payload.timeline.coordinate_frame;
        }
        mergeArchiveTimelineSegment(payload.timeline);
        state.archivePlayback.segmentCache.set(key, {
          start_s: start,
          duration_s: duration,
          lidar_frames: payload.timeline.lidar?.frames?.length || 0,
          camera_frames: payload.timeline.camera?.frames?.length || 0,
        });
        trimArchiveTimelineCache();
        if (currentRoute() === "/rover-control") renderArchiveFrame();
      }
      return payload;
    })
    .finally(() => {
      if(state.archivePlayback.loadingSegments.get(key)?.controller===controller)state.archivePlayback.loadingSegments.delete(key);
    });
  state.archivePlayback.loadingSegments.set(key, {promise,controller,start});
  return promise;
}

function archiveCameraFrameSrc(frame) {
  if (frame?.frame_url) {
    preloadArchiveCameraFrame(frame);
    return String(frame.frame_url);
  }
  if (!frame?.data) return "";
  const cache = state.archivePlayback.cache;
  const key = frame.cacheKey || `${frame.index}:${String(frame.data).length}`;
  if (cache.objectUrls.has(key)) return cache.objectUrls.get(key);
  const mime = frame.mime || "image/jpeg";
  try {
    const binary = atob(frame.data);
    const bytes = new Uint8Array(binary.length);
    for (let i = 0; i < binary.length; i += 1) bytes[i] = binary.charCodeAt(i);
    const url = URL.createObjectURL(new Blob([bytes], { type: mime }));
    cache.objectUrls.set(key, url);
    return url;
  } catch (_) {
    const dataUrl = `data:${mime};base64,${frame.data}`;
    cache.objectUrls.set(key, dataUrl);
    return dataUrl;
  }
}

function preloadArchiveCameraFrame(frame, priority = "low") {
  if (!frame) return null;
  const cache = state.archivePlayback.cache;
  const key = frame.cacheKey || `${frame.index}:${frame.frame_url || String(frame.data || "").length}`;
  const existing = cache.imageElements.get(key);
  if (existing) {
    if (priority === "high") existing.fetchPriority = "high";
    return existing;
  }
  const src = frame.frame_url ? String(frame.frame_url) : archiveCameraFrameSrc(frame);
  if (!src) return null;
  const image = new Image();
  image.decoding = "async";
  image.fetchPriority = priority;
  image.src = src;
  cache.imageElements.set(key, image);
  return image;
}

function warmArchiveCameraCache(frame) {
  if (!frame) return;
  const frames = state.archivePlayback.cache.cameraFrames;
  const center = Number(frame.cacheIndex);
  if (!Number.isFinite(center)) return;
  const start = Math.max(0, center - 3);
  const end = Math.min(frames.length - 1, center + 8);
  for (let index = start; index <= end; index += 1) {
    preloadArchiveCameraFrame(frames[index], index <= center + 2 ? "high" : "low");
  }
}

function renderArchiveDownloadButton(payload, id) {
  const button = $("roverArchiveDownloadButton");
  const meta = $("roverArchiveDownloadMeta");
  if (!button) return;
  if (!id) {
    button.hidden = true;
    button.removeAttribute("href");
    button.removeAttribute("download");
    if (meta) meta.textContent = "tar.gz";
    return;
  }
  const href = payload?.download_url || `/api/recordings/${encodeURIComponent(id)}/download`;
  button.hidden = false;
  button.href = href;
  button.download = `${id}.tar.gz`;
  if (meta) meta.textContent = `${recordingSensorLabel(payload)} · tar.gz`;
}

function recordingClockLabel(payload) {
  if (payload?.clock_quality === "gnss_estimated") return "GPS 시각 보정";
  if (payload?.clock_quality === "gnss_receiver_estimated") return "GPS UTC 기준";
  return "장치 시각 미검증";
}

function recordingClockDescription(payload) {
  if (payload?.clock_quality === "gnss_estimated") return "기존 기록의 표시 시각을 보정했습니다. 센서 시각과 재생 시간은 원본을 유지합니다.";
  if (payload?.clock_quality === "gnss_receiver_estimated") return "녹화 시작 시 GPS UTC를 확인했습니다. 수신 지연은 보정하지 않았습니다.";
  return "검증된 GPS 시간 정보가 없어 장치 시각으로 표시합니다.";
}

function renderRecordings() {
  const target = $("roverArchiveList");
  if (!target) return;
  target.innerHTML = "";
  if (state.recordings.loading && !state.recordings.items.length) {
    target.innerHTML = "<span>loading</span>";
    return;
  }
  if (!state.recordings.items.length) {
    target.innerHTML = "<span>archive empty</span>";
    renderRecordingDetail(null);
    return;
  }
  for (const item of state.recordings.items) {
    const button = document.createElement("button");
    button.type = "button";
    button.classList.toggle("active", item.id === state.recordings.selectedId);
    button.innerHTML = `
      <span>${escapeHtml(recordingSensorLabel(item))}</span>
      <strong>${escapeHtml(item.id)}</strong>
      <small>${escapeHtml(formatShortDate(item.display_started_at || item.started_at || item.created_at))} · ${escapeHtml(recordingClockLabel(item))} · ${escapeHtml(item.duration_s ? `${fmtNumber(item.duration_s, 1)}s` : item.state || "ready")} · L${escapeHtml(item.timeline?.lidar_frames ?? 0)} C${escapeHtml(item.timeline?.camera_frames ?? 0)} I${escapeHtml(item.timeline?.imu_frames ?? 0)} G${escapeHtml(item.timeline?.gps_frames ?? 0)}</small>
    `;
    button.addEventListener("click", () => selectRecording(item.id));
    target.append(button);
  }
}

function renderRecordingDetail(detail) {
  const payload = detail?.recording || detail;
  const metadata = payload?.metadata || {};
  const archiveMedia = document.querySelector(".archive-media-grid");
  archiveMedia?.classList.toggle("camera-only", (metadata.kind || payload?.kind) === "camera");
  const analysis = payload?.analysis || {};
  const preview = payload?.preview_scan;
  const hasLidar = Boolean(preview?.points?.length);
  const normalizedPreview = hasLidar ? normalizeArchiveLidarResult(preview, analysis) : null;

  const id = payload ? payload.id || metadata.id || state.recordings.selectedId || "" : "";
  if (!id) state.recordings.timeline = null;
  setupArchivePlayback(payload, normalizedPreview);
  const rows = id ? [
    { title: "ID", sub: recordingSensorLabel(payload), ok: true, level: "ok", value: id },
    { title: "Duration", sub: formatShortDate(payload.display_started_at || metadata.started_at), ok: true, level: "ok", value: metadata.duration_s ? `${fmtNumber(metadata.duration_s, 1)}s` : metadata.state || "--" },
    { title: "기록 시각", sub: recordingClockDescription(payload), ok: ["gnss_estimated", "gnss_receiver_estimated"].includes(payload.clock_quality), level: ["gnss_estimated", "gnss_receiver_estimated"].includes(payload.clock_quality) ? "ok" : "warn", value: recordingClockLabel(payload) },
    { title: "Timeline", sub: `${payload?.timeline_summary?.lidar_frames || state.recordings.timeline?.lidar?.frames?.length || 0} lidar · ${payload?.timeline_summary?.camera_frames || state.recordings.timeline?.camera?.frames?.length || 0} camera · ${payload?.timeline_summary?.imu_frames || state.recordings.timeline?.imu?.frames?.length || 0} imu`, ok: Boolean(state.recordings.timeline), level: state.recordings.timeline ? "ok" : "warn", value: state.recordings.timeline ? "playable" : "preview only" },
  ] : [
    { title: "Archive", sub: "기록을 선택하세요", ok: false, level: "warn", value: "--" },
  ];
  renderStatusRows("roverArchiveDetails", rows);
  renderArchiveDownloadButton(payload, id);
}

function configureArchiveVideoClock(payload, url) {
  const p = state.archivePlayback, adaptive = payload?.adaptive_video;
  const explicit = adaptive?.playback_clock != null || adaptive?.frame_timing === "ros_camera_header_ns_VFR";
  const key = JSON.stringify([url, adaptive?.clock_index_url, adaptive?.playback_clock, explicit]);
  if (key === p.videoClockKey) return;
  p.videoClock?.destroy(); p.videoClock = null;
  p.videoClockKey = key; p.videoClockExplicit = explicit;
  p.videoSeekRevision++; p.videoPlayRevision++; p.videoSeekPending = false;
  p.videoPlayRequested = false; p.videoClockWarm = null; p.clockBuffering = false;
  p.videoClockStartSkippedS = 0; p.videoClockTerminalCompleted = false;
  p.videoClockState = explicit ? {status:"loading", reason:"clock_index_loading"} : null;
  if (!explicit) return;
  try {
    const clock = new window.ArchiveVideoClock.Clock(adaptive, {onState:status => {
      if (state.archivePlayback.videoClock !== clock) return;
      p.videoClockState = status;
      renderArchiveFrame();
      if (p.playing && !p.raf && !p.tickTimer) queueArchivePlaybackTick();
    }});
    p.videoClock = clock;
    clock.ready().then(() => { if (p.videoClock === clock) renderArchiveFrame(); });
  } catch (error) {
    p.videoClockState = {status:"invalid", reason:error?.message || "clock_descriptor_invalid"};
  }
}

function archiveVideoTime(timeS) {
  const p = state.archivePlayback;
  if (p.videoClockExplicit) return p.videoClock?.mapCached(Number(timeS), "host_to_video") ?? null;
  return Math.max(0, Number(timeS) - (p.nativeVideoOffsetS || 0));
}

function archiveHostVideoTime(video, {warm = true} = {}) {
  const p = state.archivePlayback;
  if (!video || !p.nativeVideoReady || video.readyState < 1 || video.seeking ||
      p.videoSeekPending || Number.isFinite(p.sourceSwitchTimeS)) return null;
  const media = Number(video.currentTime);
  if (!Number.isFinite(media) || media < 0) return null;
  if (!p.videoClockExplicit) return media + (p.nativeVideoOffsetS || 0);
  const value = p.videoClock?.mapCached(media, "video_to_host") ?? null;
  if (value === null && warm) warmArchiveVideoClock(media);
  return value;
}

function warmArchiveVideoClock(media) {
  const p = state.archivePlayback, clock = p.videoClock;
  if (!clock || p.videoClockWarm || p.videoSeekPending) return;
  const seekRevision = p.videoSeekRevision;
  const pending = clock.ensure(media, "video_to_host"); p.videoClockWarm = pending;
  pending.then(value => {
    if (p.videoClock !== clock || p.videoSeekRevision !== seekRevision || p.videoClockWarm !== pending) return;
    p.videoClockWarm = null;
    if (value === null) return; // Gaps/outside must not create a promise/render loop.
    if (p.playing && completeArchiveNativeClockTail(archiveNativeVideo())) return;
    if (p.playing && p.clockBuffering) resumeArchiveClockVideo();
    renderArchiveFrame();
  }).finally(() => { if (p.videoClockWarm === pending) p.videoClockWarm = null; });
}

function completeArchiveNativeClockTail(video) {
  const p = state.archivePlayback, clock = p.videoClock, index = clock?.index;
  if (!p.playing || !p.videoClockExplicit || !index || clock.dead || clock.fatal ||
      !video || !p.nativeVideoReady || video.readyState < 1 || video.seeking || p.videoSeekPending) return false;
  const media = Number(video.currentTime), lastVideo = index.coverage_video_ms[1] / 1000;
  if (!Number.isFinite(media) || media <= lastVideo + 1e-9) return false;
  const lastHost = clock.mapCached(lastVideo, "video_to_host");
  if (!Number.isFinite(lastHost)) {
    // Verify the actual final chunk before stopping at its observed anchor.
    p.clockBuffering = true; if (!video.paused) video.pause();
    warmArchiveVideoClock(lastVideo); return true;
  }
  p.timeS = lastHost; p.videoClockTerminalCompleted = true;
  p.playing = false; // Preserve this observed endpoint in stop's render sample.
  stopArchivePlayback(false); return true;
}

function resumeArchiveClockVideo() {
  const p = state.archivePlayback, video = archiveNativeVideo();
  if (!p.playing || p.videoSeekPending || !video || !p.clockBuffering) return;
  if (archiveHostVideoTime(video, {warm:false}) === null) return;
  const playRevision = p.videoPlayRevision, seekRevision = p.videoSeekRevision;
  p.clockBuffering = false;
  Promise.resolve(video.play()).then(() => {
    if (!p.playing || p.videoPlayRevision !== playRevision || p.videoSeekRevision !== seekRevision) {
      if (!p.playing && !p.videoPlayRequested) video.pause();
    }
    else if (!p.raf && !p.tickTimer) queueArchivePlaybackTick();
  }).catch(error => {
    if (p.playing && p.videoPlayRevision === playRevision && p.videoSeekRevision === seekRevision) {
      p.clockBuffering = true; p.nativeVideoError = error?.message || "native video resume failed";
    }
  });
}

async function seekArchiveNativeVideo(hostTime) {
  const p = state.archivePlayback, video = archiveNativeVideo(), clock = p.videoClock;
  if (!video) return !p.videoClockExplicit;
  const revision = ++p.videoSeekRevision;
  p.videoSeekPending = true; p.videoClockWarm = null;
  clock?.cancelPending();
  if (!video.paused) video.pause();
  const target = p.videoClockExplicit ? await clock?.ensure(hostTime, "host_to_video") : archiveVideoTime(hostTime);
  if (revision !== p.videoSeekRevision || clock !== p.videoClock) return false;
  p.videoSeekPending = false;
  if (!Number.isFinite(target)) {
    p.clockBuffering = true; renderArchiveFrame({sampleMedia:false}); return false;
  }
  const tolerance = p.videoClockExplicit ? 1e-9 : .04;
  try { if (Math.abs(Number(video.currentTime) - target) > tolerance) video.currentTime = target; }
  catch (_) { p.clockBuffering = true; return false; }
  p.clockBuffering = false;
  if (p.playing) { p.clockBuffering = true; resumeArchiveClockVideo(); }
  return true;
}

function archiveRenderClock({sampleMedia = true} = {}) {
  const p = state.archivePlayback;
  if (sampleMedia && p.playing && !p.nativeVideoError) {
    const next = archiveHostVideoTime(archiveNativeVideo());
    if (Number.isFinite(next)) p.timeS = Math.max(0, Math.min(p.durationS || next, next));
  }
  return Math.max(0, Number(p.timeS) || 0);
}

function archiveVideoClockCaption() {
  const p = state.archivePlayback;
  if (!p.videoClockExplicit) return "";
  const skipped = p.videoClockStartSkippedS > 0 ? "첫 영상 표본부터 재생 · 앞부분 영상 미동기 구간 건너뜀" : "";
  if (p.videoClockTerminalCompleted) return ["마지막 동기 표본에서 재생 종료", skipped].filter(Boolean).join(" · ");
  const status = p.videoClock?.state() || p.videoClockState || {};
  if (status.status === "synchronized") return skipped;
  if (status.status === "loading") return "영상·센서 동기화 자료 불러오는 중";
  if (status.reason === "clock_gap") return "영상·센서 시각 미동기 · 연결할 수 없는 구간";
  if (status.reason === "clock_outside_coverage") return "영상·센서 시각 미동기 · 동기화 기록 구간 밖";
  return "영상·센서 시각 미동기 · 시각 자료 확인 필요";
}

function configureArchiveNativeVideo(payload) {
  const video = $("roverArchiveCameraVideo");
  const frame = video?.closest(".archive-camera-frame");
  if (!video) return;
  const url = String(payload?.video_url || "");
  const changed = url !== state.archivePlayback.nativeVideoUrl;
  if (changed) {
    state.archivePlayback.videoPlayer?.destroy();
    state.archivePlayback.videoPlayer = null;
    state.archivePlayback.videoQuality = {};
    video.pause();
    video.removeAttribute("src");
    video.load();
    state.archivePlayback.nativeVideoUrl = url;
    state.archivePlayback.nativeVideoReady = false;
    state.archivePlayback.nativeVideoError = "";
    const image = $("roverArchiveCameraImage");
    if (url && image) image.removeAttribute("src");
  }
  configureArchiveVideoClock(payload, url);
  state.archivePlayback.nativeVideoOffsetS = Math.max(0, Number(payload?.adaptive_video?.timeline_offset_s || payload?.source_video?.timeline_offset_s) || 0);
  frame?.classList.toggle("native-video", Boolean(url));
  if (!url) return;
  video.onloadedmetadata = () => {
    if (url !== state.archivePlayback.nativeVideoUrl) return;
    state.archivePlayback.nativeVideoReady = true;
    state.archivePlayback.nativeVideoError = "";
    const duration = Number(video.duration) + state.archivePlayback.nativeVideoOffsetS;
    if (!state.archivePlayback.videoClockExplicit && Number.isFinite(duration) && duration > 0) {
      state.archivePlayback.durationS = Math.max(state.archivePlayback.durationS, duration);
      syncArchiveScrubberBounds();
    }
    if (!state.archivePlayback.playing && !state.archivePlayback.videoPlayRequested) seekArchiveNativeVideo(state.archivePlayback.timeS);
    // A fallback player's metadata listener may still restore its saved cursor.
    renderArchiveFrame({sampleMedia:false});
  };
  video.onseeked = () => {
    if (url !== state.archivePlayback.nativeVideoUrl) return;
    if (state.archivePlayback.playing && state.archivePlayback.clockBuffering) resumeArchiveClockVideo();
    renderArchiveFrame({sampleMedia:false});
  };
  video.onerror = () => {
    if (url !== state.archivePlayback.nativeVideoUrl) return;
    state.archivePlayback.nativeVideoReady = false;
    state.archivePlayback.nativeVideoError = "native video decode failed";
    renderArchiveFrame();
  };
  video.onended = () => {
    if (!state.archivePlayback.playing) return;
    if (completeArchiveNativeClockTail(video)) return;
    if (state.archivePlayback.videoClockExplicit) archiveRenderClock();
    else state.archivePlayback.timeS = state.archivePlayback.durationS;
    stopArchivePlayback(false);
  };
  if (changed) {
    state.archivePlayback.videoPlayer = new window.ArchiveVideoPlayer(video,
      $("roverArchiveVideoQuality"), payload?.adaptive_video, url, quality => {
        state.archivePlayback.videoQuality = quality; renderArchiveFrame();
      });
  }
}

function archiveNativeVideo() {
  const video = $("roverArchiveCameraVideo");
  return state.archivePlayback.nativeVideoUrl && video ? video : null;
}

function syncArchiveScrubberBounds() {
  for (const id of ["roverArchiveScrubber", "roverOrchardMapScrubber"]) {
    const scrubber = $(id);
    if (!scrubber) continue;
    scrubber.max = String(Math.max(0, state.archivePlayback.durationS));
    scrubber.value = String(Math.min(Number(scrubber.value) || 0, state.archivePlayback.durationS));
    scrubber.disabled = !state.recordings.timeline;
  }
}

function setupArchivePlayback(payload, fallbackPreview) {
  const timeline = state.recordings.timeline;
  configureArchiveNativeVideo(payload);
  prepareArchiveTimelineCache(timeline);
  const metadata = payload?.metadata || {};
  const duration = Number(timeline?.duration_s || metadata.duration_s || 0);
  const lidarFrames = state.archivePlayback.cache.lidarFrames || [];
  const cameraFrames = state.archivePlayback.cache.cameraFrames || [];
  const imuFrames = state.archivePlayback.cache.imuFrames || [];
  const gpsFrames = state.archivePlayback.cache.gpsFrames || [];
  const lastFrameTime = Math.max(0, ...gpsFrames.map((frame) => Number(frame.t_s) || 0), ...lidarFrames.map((frame) => Number(frame.t_s) || 0), ...cameraFrames.map((frame) => Number(frame.t_s) || 0), ...imuFrames.map((frame) => Number(frame.t_s) || 0));
  state.archivePlayback.durationS = Math.max(duration, lastFrameTime);
  state.archivePlayback.timeS = Math.min(state.archivePlayback.timeS || 0, state.archivePlayback.durationS);
  state.archivePlayback.fallbackPreview = fallbackPreview;
  syncArchiveScrubberBounds();
  const play = $("roverArchivePlayButton");
  if (play) play.disabled = !timeline;
  renderArchiveFrame();
}

function suspendLiveCameraForArchive() {
  const data = state.telemetry || {};
  const bridge = data.connection?.rover_bridge || (data.services || data.connection?.services || {}).rover_bridge || {};
  const age = Date.now() / 1000 - Number(bridge.updated_at);
  // Only a fresh, explicitly disarmed controller permits saving bandwidth.
  // No sensor capture/recording setting is changed; this tab's preview alone
  // is suspended. Active/unknown driving must retain its live camera view.
  if (state.roverControl.lease || bridge.alive === false || bridge.armed !== false || !Number.isFinite(age) || age < -0.25 || age > 3) return;
  if (!state.cameraLive.socket || state.cameraLive.userStopped) return;
  state.archivePlayback.resumeLiveCamera = true;
  closeCameraLive("기록 재생을 위해 실시간 영상 수신을 잠시 멈춥니다.", { userStopped: true });
}

function resumeLiveCameraAfterArchive() {
  if (!state.archivePlayback.resumeLiveCamera) return;
  state.archivePlayback.resumeLiveCamera = false;
  if (routeSupportsCameraLive() && !state.cameraLive.socket) startCameraLive();
}

async function toggleArchivePlayback() {
  if (state.archivePlayback.playing || state.archivePlayback.videoPlayRequested) {
    stopArchivePlayback(false);
    return;
  }
  if (!state.recordings.timeline) return;
  suspendLiveCameraForArchive();
  let startTime = state.archivePlayback.timeS >= state.archivePlayback.durationS ? 0 : state.archivePlayback.timeS;
  const video = archiveNativeVideo();
  const revision = ++state.archivePlayback.videoPlayRevision;
  state.archivePlayback.videoPlayRequested = true;
  state.archivePlayback.videoClockStartSkippedS = 0;
  state.archivePlayback.videoClockTerminalCompleted = false;
  if (video) {
    try {
      if (state.archivePlayback.videoClockExplicit) {
        const clock = state.archivePlayback.videoClock, index = await clock?.ready();
        if (revision !== state.archivePlayback.videoPlayRevision || clock !== state.archivePlayback.videoClock) return;
        const firstHost = index?.coverage_host_ns?.[0] / 1e9;
        if (Number.isFinite(firstHost) && startTime < firstHost) {
          const firstVideo = await clock.ensure(firstHost, "host_to_video");
          if (revision !== state.archivePlayback.videoPlayRevision || clock !== state.archivePlayback.videoClock) return;
          if (Number.isFinite(firstVideo)) {
            state.archivePlayback.videoClockStartSkippedS = firstHost - startTime;
            startTime = firstHost;
          }
        }
      }
      if (!await seekArchiveNativeVideo(startTime)) {
        if (revision === state.archivePlayback.videoPlayRevision) state.archivePlayback.videoPlayRequested = false;
        renderArchiveFrame({sampleMedia:false}); return;
      }
      if (revision !== state.archivePlayback.videoPlayRevision) return;
      state.archivePlayback.timeS = startTime;
      renderArchiveFrame({sampleMedia:false});
      await video.play();
      if (revision !== state.archivePlayback.videoPlayRevision) {
        if (!state.archivePlayback.playing && !state.archivePlayback.videoPlayRequested) video.pause();
        return;
      }
    } catch (err) {
      if (revision !== state.archivePlayback.videoPlayRevision) return;
      state.archivePlayback.nativeVideoError = err?.message || "native video play failed";
      state.archivePlayback.videoPlayRequested = false;
      resumeLiveCameraAfterArchive();
      renderArchiveFrame();
      return;
    }
  }
  if (state.archivePlayback.videoClockExplicit && !video) {
    state.archivePlayback.videoPlayRequested = false; renderArchiveFrame({sampleMedia:false}); return;
  }
  state.archivePlayback.videoPlayRequested = false;
  state.archivePlayback.playing = true;
  state.archivePlayback.startedAt = performance.now();
  state.archivePlayback.startTimeS = startTime;
  scheduleArchiveSegmentsForTime(startTime, { reschedule: true })
    .catch((err) => debugLog("archive", "initial segment load failed", err));
  archivePlaybackTick();
}

function queueArchivePlaybackTick() {
  let completed=false;
  const run=()=>{
    if(completed)return;completed=true;
    clearTimeout(state.archivePlayback.tickTimer);
    if(state.archivePlayback.raf)cancelAnimationFrame(state.archivePlayback.raf);
    state.archivePlayback.tickTimer=null;state.archivePlayback.raf=null;
    archivePlaybackTick(performance.now());
  };
  state.archivePlayback.raf=requestAnimationFrame(run);
  state.archivePlayback.tickTimer=setTimeout(run,80);
}

function stopArchivePlayback(reset = false) {
  if (!reset) archiveRenderClock();
  state.archivePlayback.playing = false;
  state.archivePlayback.videoPlayRequested = false;
  state.archivePlayback.videoPlayRevision++;
  state.archivePlayback.videoSeekRevision++;
  state.archivePlayback.videoSeekPending = false;
  state.archivePlayback.videoClockWarm = null;
  state.archivePlayback.clockBuffering = false;
  state.archivePlayback.videoClock?.cancelPending();
  resumeLiveCameraAfterArchive();
  const video = archiveNativeVideo();
  if (video && !video.paused) video.pause();
  if (state.archivePlayback.raf) {
    cancelAnimationFrame(state.archivePlayback.raf);
    state.archivePlayback.raf = null;
  }
  clearTimeout(state.archivePlayback.tickTimer);
  state.archivePlayback.tickTimer=null;
  if (reset) {
    state.archivePlayback.timeS = 0;
    if (video) {
      try { video.currentTime = 0; } catch (_) { /* metadata is not ready yet */ }
    }
  }
  renderArchiveFrame({sampleMedia:false});
}

function archivePlaybackTick(now = performance.now()) {
  if (!state.archivePlayback.playing) return;
  const video = archiveNativeVideo();
  if (completeArchiveNativeClockTail(video)) return;
  const elapsed = (now - state.archivePlayback.startedAt) / 1000;
  const next = video ? archiveHostVideoTime(video) :
    state.archivePlayback.videoClockExplicit ? null : state.archivePlayback.startTimeS + elapsed;
  if (!Number.isFinite(next)) {
    state.archivePlayback.clockBuffering = true;
    if (video && !video.paused) video.pause();
    const status = state.archivePlayback.videoClock?.state()?.status;
    if (!state.archivePlayback.videoClockExplicit || ["loading", "unavailable"].includes(status)) queueArchivePlaybackTick();
    return;
  }
  if (next >= state.archivePlayback.durationS) {
    state.archivePlayback.timeS = state.archivePlayback.durationS;
    stopArchivePlayback(false);
    return;
  }
  state.archivePlayback.timeS = next;
  scheduleArchiveSegmentsForTime(next).catch((err) => debugLog("archive", "playback segment load failed", err));
  if (now - (state.archivePlayback.lastPlaybackRenderAt || 0) >= 50) {
    state.archivePlayback.lastPlaybackRenderAt = now;
    renderArchiveFrame();
  }
  queueArchivePlaybackTick();
}

function setArchiveTime(value, options = {}) {
  if (options.pause) stopArchivePlayback(false);
  state.archivePlayback.videoClockStartSkippedS = 0;
  state.archivePlayback.videoClockTerminalCompleted = false;
  state.archivePlayback.timeS = Math.max(0, Math.min(state.archivePlayback.durationS, Number(value) || 0));
  seekArchiveNativeVideo(state.archivePlayback.timeS).catch(err => debugLog("archive", "video clock seek failed", err));
  scheduleArchiveSeek(state.archivePlayback.timeS);
  renderArchiveFrame({sampleMedia:false});
}

function renderArchiveFrame(options = {}) {
  const timeline = state.recordings.timeline;
  const timeS = archiveRenderClock(options);
  const cache = state.archivePlayback.cache || {};
  window.RoverGps?.archive(cache.gpsFrames || [], timeS, state.recordings.selectedId, timeline?.gps?.enabled === true);
  renderArchiveMotor(timeline?.motor, cache.motorFrames || [], timeS);
  window.SimMap?.archiveTime(timeS);
  const lidarFrame = timeline ? nearestTimelineFrame(cache.lidarFrames || [], timeS) : null;
  const cameraFrame = timeline && state.archivePlayback.sync ? nearestTimelineFrame(cache.cameraFrames || [], timeS) : null;
  const nearestImuFrame = timeline ? nearestTimelineFrame(cache.imuFrames || [], timeS) : null;
  const imuSyncDeltaS = nearestImuFrame ? Math.abs(Number(nearestImuFrame.t_s || 0) - timeS) : null;
  const lastImuFrame = cache.imuFrames?.at(-1);
  const imuTail = SIMULATION && nearestImuFrame && nearestImuFrame === lastImuFrame
    && timeS > Number(lastImuFrame.t_s) && imuSyncDeltaS > ARCHIVE_IMU_SYNC_MAX_DELTA_S
    && (timeline?.streaming !== true || archiveTailLoaded(Number(lastImuFrame.t_s), state.archivePlayback.durationS));
  const imuFrame = Number.isFinite(imuSyncDeltaS) && imuSyncDeltaS <= ARCHIVE_IMU_SYNC_MAX_DELTA_S
    ? nearestImuFrame
    : imuTail ? lastImuFrame : null;
  const lidarResult = lidarFrame?.result || state.archivePlayback.fallbackPreview;
  const lidarRenderKey = `${lidarFrame?.cacheIndex}:${state.archivePlayback.range}`;
  if (lidarRenderKey !== state.archivePlayback.lastLidarRenderKey) {
    drawLidarMapOnCanvas($("roverArchiveLidarCanvas"), lidarResult, lidarResult?.analysis, {
      range: state.archivePlayback.range,
    });
    state.archivePlayback.lastLidarRenderKey = lidarRenderKey;
  }

  const image = $("roverArchiveCameraImage");
  const nativeVideo = archiveNativeVideo();
  if (!nativeVideo && image && (cameraFrame?.frame_url || cameraFrame?.data)) {
    warmArchiveCameraCache(cameraFrame);
    const src = archiveCameraFrameSrc(cameraFrame);
    const cameraRenderKey = `${cameraFrame.cacheIndex}:${src}`;
    if (src && state.archivePlayback.lastCameraRenderKey !== cameraRenderKey) {
      image.src = src;
      state.archivePlayback.lastCameraRenderKey = cameraRenderKey;
    }
  } else if (!nativeVideo && image && !cameraFrame) {
    image.removeAttribute("src");
    state.archivePlayback.lastCameraRenderKey = "";
  }
  const sourceVideo = state.recordings.selected?.recording?.source_video || state.recordings.selected?.source_video || {};
  const cameraCaption = state.archivePlayback.nativeVideoError
    ? `video error · ${state.archivePlayback.nativeVideoError}`
    : nativeVideo
      ? `${nativeVideo.videoHeight || state.archivePlayback.videoQuality?.height || sourceVideo.height || ""}p · ${state.archivePlayback.videoQuality?.auto ? "자동 · " : ""}${fmtNumber(sourceVideo.fps || 0, 1)} fps · ${fmtClock(timeS)}`
    : cameraFrame
      ? `${nativeVideo ? `native ${fmtNumber(sourceVideo.fps || 0, 0)} fps · ` : ""}${cameraFrame.frame_id || "camera"} · ${fmtClock(timeS)} · ${cameraFrame.source_width || cameraFrame.width}x${cameraFrame.source_height || cameraFrame.height}`
      : state.archivePlayback.sync ? "camera preview 없음" : "동시 재생 꺼짐";
  text("roverArchiveCameraCaption", [cameraCaption, archiveVideoClockCaption()].filter(Boolean).join(" · "));
  const imuSample = imuFrame?.corrected || imuFrame?.raw || {};
  const imuEuler = imuSample.euler_deg || {};
  const imuCalibrationState = imuFrame?.calibration_state || timeline?.imu?.calibration?.state || "unavailable";
  const imuAcceleration = imuSample.linear_acceleration || {};
  const imuGyro = imuSample.angular_velocity || {};
  const imuMagnetic = imuFrame?.magnetic?.magnetic_field_uT || {};
  const imuFrameCount = Math.max(
    Number(timeline?.sync?.imu_frames || 0),
    Number(state.recordings.selected?.recording?.timeline_summary?.imu_frames || 0),
    Number(state.recordings.selected?.recording?.metadata?.timeline?.imu_frames || 0),
    cache.imuFrames?.length || 0,
  );
  const hasArchivedImu = imuFrameCount > 0;
  const imuPanel = $("roverArchiveImuPanel");
  if (imuPanel) {
    imuPanel.hidden = !hasArchivedImu;
    imuPanel.classList.toggle("archive-imu-stale", Boolean(imuTail));
  }
  const colors = chartColors();
  renderReadout("roverArchiveImuStats", [
    {
      label: "Roll / Pitch / Yaw",
      value: imuFrame ? `${fmtNumber(imuEuler.roll, 1)}° / ${fmtNumber(imuEuler.pitch, 1)}° / ${fmtNumber(imuEuler.yaw, 1)}°` : "--",
      sub: imuTail ? "마지막 표본 · 동기 범위 초과" : imuFrame ? `${imuFrame.corrected ? "start-corrected" : "raw"} · calibration ${imuCalibrationState}` : "현재 재생 시각 안에 IMU frame 없음",
      color: imuFrame?.corrected ? colors.good : colors.amber,
    },
    { label: "Acceleration", value: imuFrame && vectorNorm(imuAcceleration) !== null ? `${fmtNumber(vectorNorm(imuAcceleration), 3)} m/s²` : "--", sub: `x ${fmtNumber(imuAcceleration.x, 3)} · y ${fmtNumber(imuAcceleration.y, 3)} · z ${fmtNumber(imuAcceleration.z, 3)}`, color: colors.good },
    { label: "Angular velocity", value: imuFrame && vectorNorm(imuGyro) !== null ? `${fmtNumber(vectorNorm(imuGyro), 4)} rad/s` : "--", sub: `x ${fmtNumber(imuGyro.x, 4)} · y ${fmtNumber(imuGyro.y, 4)} · z ${fmtNumber(imuGyro.z, 4)}`, color: colors.amber },
    { label: "Magnetic field", value: imuFrame && vectorNorm(imuMagnetic) !== null ? `${fmtNumber(vectorNorm(imuMagnetic), 2)} µT` : "--", sub: imuFrame?.calibration_reference_id ? `reference ${imuFrame.calibration_reference_id}` : "magnetometer unavailable", color: colors.blue },
  ]);
  text("roverArchiveImuTime", imuFrame ? `IMU ${fmtNumber(imuFrame.t_s, 3)}s / Δ ${fmtNumber(imuSyncDeltaS, 3)}s${imuTail ? " · 마지막 표본" : ""}` : "IMU -- / Δ --");
  text("roverArchiveImuCaption", imuFrame ? `${imuTail ? "동기 범위 초과 · " : ""}${imuFrame.corrected ? "/imu/data · base_link corrected" : "/imu/data_raw · sensor-native"} · playback ${fmtNumber(timeS, 3)}s` : "동기 IMU sample 대기");
  drawImuAttitude($("roverArchiveImuCanvas"), Number(imuEuler.roll), Number(imuEuler.pitch), Number(imuEuler.yaw));
  text("roverArchiveTime", `${fmtClock(timeS)} / ${fmtClock(state.archivePlayback.durationS)}`);
  text("roverOrchardMapTime", `${fmtClock(timeS)} / ${fmtClock(state.archivePlayback.durationS)}`);
  const scrubber = $("roverArchiveScrubber");
  if (scrubber && document.activeElement !== scrubber) scrubber.value = String(timeS);
  const mapScrubber = $("roverOrchardMapScrubber");
  if (mapScrubber && document.activeElement !== mapScrubber) mapScrubber.value = String(timeS);
  const play = $("roverArchivePlayButton");
  if (play) play.textContent = state.archivePlayback.playing || state.archivePlayback.videoPlayRequested ? "일시정지" : "재생";

}


function archiveTailLoaded(lastSampleS, durationS) {
  // A partial segment cache cannot prove that its last sample is the archive's
  // last sample. Retain stale data only after every remaining segment loaded.
  let coveredUntil = lastSampleS;
  const segments = [...state.archivePlayback.segmentCache.values()].sort((a, b) => a.start_s - b.start_s);
  for (const segment of segments) {
    if (segment.start_s > coveredUntil + 0.001) break;
    coveredUntil = Math.max(coveredUntil, segment.start_s + segment.duration_s);
  }
  return coveredUntil >= durationS;
}


function mergeArchiveMotorSegment(motor) {
  if (!motor) return;
  if (state.recordings.timeline) {
    state.recordings.timeline.motor = {
      ...(state.recordings.timeline.motor || {}),
      ...Object.fromEntries(Object.entries(motor).filter(([key]) => key !== "frames")),
      frames: [],
    };
  }
  const cache = state.archivePlayback.cache;
  cache.motorFrames ||= [];
  cache.motorKeys ||= new Set();
  for (const frame of Array.isArray(motor.frames) ? motor.frames : []) {
    // Malformed timestamps must not become a fabricated sample at zero.
    if (!Number.isFinite(frame?.t_s) || frame.t_s < 0) continue;
    const key = String(frame.t_s);
    if (cache.motorKeys.has(key)) continue;
    cache.motorKeys.add(key);
    cache.motorFrames.push(frame);
  }
  cache.motorFrames.sort((a, b) => a.t_s - b.t_s);
}

function archiveMotorFrameAtOrBefore(frames, timeS) {
  if (!Array.isArray(frames) || !frames.length || !Number.isFinite(timeS)) return null;
  let low = 0;
  let high = frames.length;
  // Right insertion point: unlike sensor previews, never use a future command.
  while (low < high) {
    const mid = Math.floor((low + high) / 2);
    if (frames[mid].t_s <= timeS) low = mid + 1;
    else high = mid;
  }
  return low > 0 ? frames[low - 1] : null;
}

function archiveMotorPwmLabel(value) {
  if (!Number.isInteger(value) || value < 0 || value > 255) return "미기록";
  return `${value} / 255 · ${(value / 255 * 100).toFixed(1)}%`;
}

function renderArchiveMotor(motor, frames, timeS) {
  const panel = $("roverArchiveMotorPanel");
  if (!panel) return;
  panel.hidden = !state.recordings.selectedId;
  const frame = archiveMotorFrameAtOrBefore(frames, timeS);
  const simulated = frame?.status?.simulated === true || (!frame && motor?.source_kind === "simulation");
  text("roverArchiveMotorOutputLabel", simulated ? "시뮬레이션 출력 PWM" : "MCU 보고 PWM");
  const hasArchive = motor?.enabled === true || Number(motor?.frame_count) > 0 || frames.length > 0;
  text("roverArchiveMotorTime", frame ? `스냅샷 ${frame.t_s.toFixed(3)}s` : "스냅샷 없음");
  let status;
  if (!hasArchive) {
    status = "이 녹화에는 모터 핀 신호 기록이 없습니다.";
  } else if (!frame) {
    status = "현재 재생 시각 이전의 모터 스냅샷이 없습니다. 이후 신호는 표시하지 않습니다.";
  } else {
    const limit = Number.isFinite(motor?.stale_after_s) && motor.stale_after_s > 0 ? motor.stale_after_s : 1.5;
    const gap = Math.max(0, timeS - frame.t_s);
    const stale = gap > limit || frame.source_stale !== false;
    const connected = frame.status?.connected;
    const connection = simulated
      ? connected === true ? "가상 구동 준비됨" : connected === false ? "가상 구동 준비 안 됨" : "가상 구동 상태 미기록"
      : connected === true ? "MCU 연결됨" : connected === false ? "MCU 연결 끊김" : "MCU 연결 여부 미기록";
    status = `${stale ? "오래되거나 시각을 확인할 수 없는 스냅샷" : "기록된 상태 스냅샷"} · ${connection} · 재생 시각과 차이 ${gap.toFixed(2)}s`;
    if (Number.isFinite(frame.source_age_s) && frame.source_age_s >= 0) {
      status += ` · 수집 당시 상태 파일 경과 ${frame.source_age_s.toFixed(2)}s`;
    }
  }
  if (motor?.error || motor?.sidecar_read_error || Number(motor?.sidecar_malformed_rows) > 0 || Number(motor?.dropped_time_frames) > 0) {
    // Controlled label only; archive errors may contain untrusted free text.
    status = `일부 모터 기록 누락 또는 읽기 오류 · ${status}`;
  }
  text("roverArchiveMotorStatus", status);
  const hz = Number.isFinite(motor?.sample_hz) && motor.sample_hz > 0 ? `${motor.sample_hz} Hz ` : "";
  text("roverArchiveMotorCaption", hasArchive
    ? simulated
      ? `${hz}시뮬레이션 상태 스냅샷입니다. 요청값과 가감속된 가상 속도 명령을 최대 설정 속도 기준 0–255로 환산했습니다. 실제 GPIO 신호·MCU 보고값·전압·측정 바퀴 속도가 아닙니다.`
      : `${hz}상태 스냅샷입니다. 요청 PWM은 주행 요청에서 계산한 값이며, MCU 보고 PWM은 마지막 수신 캐시입니다. 핀별 보고 시각·모든 직렬 명령·전압·실제 바퀴 속도는 기록하지 않습니다.`
    : "이 기능 적용 후 시작한 녹화에서 핀별 요청값과 MCU 보고값을 확인할 수 있습니다.");
  const body = $("roverArchiveMotorRows");
  if (!body) return;
  const roles = {left_forward: "왼쪽 정방향", left_reverse: "왼쪽 역방향", right_forward: "오른쪽 정방향", right_reverse: "오른쪽 역방향", driver_enable: "드라이버 활성화"};
  const pins = Array.isArray(motor?.pins) ? motor.pins : [];
  const html = pins.length ? pins.map((pin) => {
    const values = frame?.pins?.[pin.pin];
    const gpio = Number.isInteger(pin.gpio) && pin.gpio >= 0 ? `GPIO ${pin.gpio}` : "GPIO 미기록";
    const role = roles[pin.role] || pin.role || "역할 미기록";
    return `<tr><th scope="row">${escapeHtml(pin.pin || "핀 미기록")}</th><td>${escapeHtml(gpio)}</td><td>${escapeHtml(role)}</td><td>${escapeHtml(archiveMotorPwmLabel(values?.requested_pwm_derived))}</td><td>${escapeHtml(archiveMotorPwmLabel(simulated ? values?.simulated_output_pwm : values?.reported_pwm))}</td></tr>`;
  }).join("") : '<tr><td colspan="5">핀 매핑 기록 없음</td></tr>';
  if (body.innerHTML !== html) body.innerHTML = html;
}


function nearestTimelineFrame(frames, timeS) {
  if (!Array.isArray(frames) || !frames.length) return null;
  let low = 0;
  let high = frames.length - 1;
  while (low <= high) {
    const mid = Math.floor((low + high) / 2);
    const stamp = Number(frames[mid].t_s) || 0;
    if (stamp < timeS) low = mid + 1;
    else high = mid - 1;
  }
  if (low <= 0) return frames[0];
  if (low >= frames.length) return frames[frames.length - 1];
  const before = frames[low - 1];
  const after = frames[low];
  return Math.abs((Number(after.t_s) || 0) - timeS) < Math.abs(timeS - (Number(before.t_s) || 0)) ? after : before;
}


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
    "/research": [],
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

function chartColors() {
  return {
    blue: cssVar("--chart-blue", "#2f76ff"),
    cyan: cssVar("--chart-cyan", "#38d8ff"),
    rose: cssVar("--chart-rose", "#f21b58"),
    amber: cssVar("--chart-amber", "#f4bd4f"),
    good: cssVar("--chart-good", "#55e2b2"),
  };
}

function cssVar(name, fallback) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim() || fallback;
}

function drawCoreCharts() {
  const target = $("cpuSeparateView");
  if (!target) return;
  [...target.children].forEach((row, index) => {
    const canvas = row.querySelector("canvas");
    const key = row.dataset.key;
    if (!canvas || !key) return;
    drawMultiLine(canvas, [{ values: state.visualCoreHistory[key] || [], color: CORE_COLORS[index % CORE_COLORS.length], max: 100 }]);
  });
}

function drawMultiLine(target, series) {
  const canvas = typeof target === "string" ? $(target) : target;
  if (!canvas) return;
  const prepared = prepareCanvas(canvas);
  if (!prepared) return;
  const { ctx, w, h } = prepared;
  ctx.clearRect(0, 0, w, h);
  const gradient = ctx.createLinearGradient(0, 0, 0, h);
  gradient.addColorStop(0, cssVar("--chart-bg-top", "rgba(255,255,255,0.05)"));
  gradient.addColorStop(1, cssVar("--chart-bg-bottom", "rgba(255,255,255,0.01)"));
  ctx.fillStyle = gradient;
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

function drawGrid(ctx, w, h) {
  ctx.save();
  ctx.strokeStyle = cssVar("--chart-grid", "rgba(255,255,255,0.1)");
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
  ctx.lineWidth = 2.8;
  ctx.lineJoin = "round";
  ctx.lineCap = "round";
  ctx.shadowColor = color;
  ctx.shadowBlur = 8;
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

function drawLidarMapOnCanvas(canvas, result, overlays = {}, options = {}) {
  if (!canvas) return;
  const prepared = prepareCanvas(canvas);
  if (!prepared) return;
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
  const gradient = ctx.createRadialGradient(cx, cy, radius * 0.12, cx, cy, radius);
  gradient.addColorStop(0, cssVar("--chart-bg-top", "rgba(255,255,255,0.05)"));
  gradient.addColorStop(1, cssVar("--chart-bg-bottom", "rgba(255,255,255,0.01)"));
  ctx.fillStyle = gradient;
  ctx.fillRect(0, 0, w, h);
  drawLidarGrid(ctx, cx, cy, radius, range, gridStep);

  if (!points.length) {
    drawLidarEmpty(ctx, w, h, result?.message || "스캔 대기");
    return;
  }

  if (showPoints && points.length) {
    ctx.save();
    ctx.fillStyle = SIMULATION ? "#f04452" : colors.cyan;
    if (SIMULATION) {
      ctx.strokeStyle = "rgba(239, 68, 82, 0.08)";
      ctx.beginPath();
      for (let i = 0; i < points.length; i += 8) {
        const p = points[i];
        ctx.moveTo(cx, cy);
        const pixel = lidarCanvasPoint(p, { cx, cy, range, radius });
        ctx.lineTo(pixel.x, pixel.y);
      }
      ctx.stroke();
    }
    // Batch identical point styles into paths. Thousands of fill calls per
    // scan consumed CPU on every dashboard; coordinates and raw data stay intact.
    const groups = new Map();
    for (const point of points) {
      const quality = Math.max(0, Math.min(63, Number(point.quality) || 0));
      if (!groups.has(quality)) groups.set(quality, []);
      groups.get(quality).push(point);
    }
    const viewport = { cx, cy, range, radius };
    for (const [quality, group] of groups) {
      const size = Math.max(1.6, Math.min(2.2, 1.6 + quality / 105));
      ctx.globalAlpha = Math.max(0.75, Math.min(0.95, 0.75 + quality / 315));
      ctx.beginPath();
      for (const point of group) {
        const { x, y } = lidarCanvasPoint(point, viewport);
        ctx.moveTo(x + size, y);
        ctx.arc(x, y, size, 0, Math.PI * 2);
      }
      ctx.fill();
    }
    ctx.restore();
  }


  ctx.save();
  ctx.fillStyle = cssVar("--text", "#f7f9ff");
  ctx.font = "800 12px Inter, Arial, sans-serif";
  ctx.fillText(`${showPoints ? points.length : 0} points`, 14, 22);
  ctx.fillText(`${fmtNumber(result?.metrics?.coverage_deg ?? angleCoverage(points), 0)}° coverage`, 14, 40);
  ctx.textAlign = "right";
  ctx.fillText(`Grid: ${fmtNumber(gridStep, 0)}m · Range: ${fmtNumber(range, 1)}m`, w - 14, h - 16);
  ctx.restore();
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
  ctx.strokeStyle = cssVar("--chart-grid", "rgba(255,255,255,0.1)");
  ctx.lineWidth = 1;
  for (let meters = gridStep; meters <= range + 0.001; meters += gridStep) {
    const r = (meters / range) * radius;
    ctx.beginPath();
    ctx.arc(cx, cy, r, 0, Math.PI * 2);
    ctx.stroke();
    ctx.fillStyle = cssVar("--dim", "#7480a3");
    ctx.font = "800 10px Inter, Arial, sans-serif";
    ctx.fillText(`${fmtNumber(meters, 0)}m`, cx + 7, cy - r + 13);
  }
  ctx.beginPath();
  ctx.moveTo(cx - radius, cy);
  ctx.lineTo(cx + radius, cy);
  ctx.moveTo(cx, cy - radius);
  ctx.lineTo(cx, cy + radius);
  ctx.stroke();
  ctx.fillStyle = cssVar("--muted", "#a9b4d2");
  ctx.font = "900 11px Inter, Arial, sans-serif";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  const labelOffset = 20;
  ctx.fillText("180°", cx, cy - radius - labelOffset);
  ctx.fillText("90°", cx + radius + labelOffset, cy);
  ctx.fillText("0°", cx, cy + radius + labelOffset);
  ctx.fillText("270°", cx - radius - labelOffset, cy);
  ctx.fillStyle = cssVar("--text", "#f7f9ff");
  ctx.beginPath();
  ctx.arc(cx, cy, 5, 0, Math.PI * 2);
  ctx.fill();
  ctx.restore();
}

function drawLidarEmpty(ctx, w, h, message) {
  ctx.save();
  ctx.fillStyle = cssVar("--muted", "#a9b4d2");
  ctx.font = "900 14px Inter, Arial, sans-serif";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillText(message, w / 2, h / 2);
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

window.addEventListener("popstate", renderRoute);

window.addEventListener("pagehide", () => {
  if (state.roverControl.lease || hasActiveRoverInput() || state.roverControl.stoppingWithFade || Math.abs(state.roverControl.currentLinear) > 0.001 || Math.abs(state.roverControl.currentAngular) > 0.001) {
    try {
      const body = JSON.stringify({ source: "pagehide" });
      if (navigator.sendBeacon) {
        navigator.sendBeacon("/api/rover/stop", new Blob([body], { type: "application/json" }));
      } else {
        fetch("/api/rover/stop", { method: "POST", keepalive: true, headers: { "Content-Type": "application/json" }, body });
      }
    } catch (_) {
      // The bridge command TTL still stops stale UI commands.
    }
  }
  if (state.lidarLive.socket || state.lidarLive.enabled) {
    closeLidarLive("페이지 종료로 LiDAR WebSocket을 닫습니다.", { userStopped: false });
  }
  if (state.cameraLive.socket || state.cameraLive.enabled) {
    closeCameraLive("페이지 종료로 Camera WebSocket을 닫습니다.", { userStopped: false });
  }
  clearArchiveFrameCache();
});
document.addEventListener("click", (event) => {
  const docLink = event.target.closest("a[data-doc-link]");
  if (docLink) {
    event.preventDefault();
    history.pushState({}, "", docLink.getAttribute("href"));
    setMobileNav(false);
    renderRoute();
    return;
  }
  const link = event.target.closest("a[data-route]");
  if (!link) return;
  event.preventDefault();
  history.pushState({}, "", link.getAttribute("href"));
  setMobileNav(false);
  renderRoute();
});

initMobileNav();
initTheme();
initCpuMode();
initLidarTest();
initCameraLive();
initFanControls();
initDockerControls();
initNetworkControls();
initRoverControl();
renderRoute();
loadPlan().catch(console.error);
loadDocs().catch(console.error);
tick();
setInterval(tick, POLL_INTERVAL_MS);
setInterval(() => {
  // Recorder polling and UI expiry are independent of general telemetry.
  maybeRefreshRecordingStatus();
  if (routeSupportsCameraLive() && (state.cameraLive.enabled || state.cameraLive.lastDecodedAt !== null)) renderCameraLiveState();
}, 1000);
