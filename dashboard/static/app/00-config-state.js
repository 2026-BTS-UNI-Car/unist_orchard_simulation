const SIMULATION = document.documentElement.dataset.runtimeMode === "simulation";
// The eight pages of the menu, as before the redesign (UI-16, docs/SCREENS_KO.md).
const ROUTES = ["/", "/connection", "/system", "/network", "/docker", "/fan", "/rover-control", "/docs"];
// Addresses of the six-screen layout (2026-11-13) and of the removed research page: the page that holds
// those panels now. The page writes the menu address in the address bar; ROUTE_FOCUS names the part of
// Rover Control that such an address showed, and the page scrolls there once.
const ROUTE_ALIASES = { "/drive": "/rover-control", "/map": "/rover-control", "/records": "/rover-control",
  "/tuning": "/rover-control", "/sensors": "/connection", "/system/resources": "/system", "/system/network": "/network",
  "/system/docker": "/docker", "/system/fan": "/fan", "/research": "/" };
const ROUTE_FOCUS = { "/map": "liveMapPanel", "/records": "roverRecordPanel", "/tuning": "tuningPanel" };
const HISTORY_KEYS = ["cpu", "memory", "swap", "power", "temp", "rx", "tx", "dockerCpu", "dockerMemory", "fanTemp", "fanPwm"];
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
    // What this browser asks the camera bridge for. It is shown as the camera's
    // setting only after the server reported or accepted it (settingsKnown).
    resolution: "1280x720",
    fps: 60,
    settingsKnown: false,
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
	    lidarPainted: false,
	    lastCameraRenderKey: "",
	    nativeVideoUrl: "",
	    nativeVideoReady: false,
	    nativeVideoError: "",
	    resumeLiveCamera: false,
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
