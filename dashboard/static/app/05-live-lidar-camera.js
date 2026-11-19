
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
    setTestResult("bad", `상태를 확인하지 못했습니다. ${err.message}`);
  } finally {
    if (button) button.disabled = false;
  }
}

// Sensor WebSocket reconnect policy (S12): exponential backoff with jitter
// from 1.5 s up to 30 s, reset when a message arrives. A camera viewer limit
// waits 30 s; an autonomy-priority close (4001) reconnects once after 0.5 s so
// the proxy can apply its new preview cap.
const SENSOR_RECONNECT_BASE_MS = 1500;
const SENSOR_RECONNECT_MAX_MS = 30000;
function sensorReconnectDelay(attempt, closeEvent = {}, random = Math.random) {
  const reason = String(closeEvent?.reason || "");
  if (closeEvent?.code === 4001 && reason === "autonomy_priority") return 500;
  if (closeEvent?.code === 1013 && reason.includes("viewer limit")) return SENSOR_RECONNECT_MAX_MS;
  const step = Math.max(0, Math.min(10, Number(attempt) || 0));
  const base = Math.min(SENSOR_RECONNECT_MAX_MS, SENSOR_RECONNECT_BASE_MS * 2 ** step);
  return Math.round(Math.min(SENSOR_RECONNECT_MAX_MS, base * (0.8 + 0.4 * random())));
}
// NET-11: when the link comes back the channels do not all reconnect at the same
// moment: LiDAR first, IMU 0.25 s later, the camera (the big one) 0.6 s later.
const SENSOR_RECONNECT_STAGGER_MS = { lidar: 0, imu: 250, camera: 600 };
function nextSensorReconnectDelay(live, closeEvent = {}) {
  const channel = live === state.cameraLive ? "camera" : live === state.imuLive ? "imu" : "lidar";
  const fast = closeEvent?.code === 4001 && closeEvent?.reason === "autonomy_priority";
  const delay = sensorReconnectDelay(live.reconnectAttempt || 0, closeEvent) + (fast ? 0 : SENSOR_RECONNECT_STAGGER_MS[channel]);
  if (!(closeEvent?.code === 4001 && closeEvent?.reason === "autonomy_priority")) {
    live.reconnectAttempt = (live.reconnectAttempt || 0) + 1;
  }
  return delay;
}

// Hidden page (S04/S11): after 1 s hidden, close the camera, LiDAR and IMU
// sockets (not a user stop) and reopen exactly those when the page is visible
// again. A camera paused by acknowledgement timeouts is restarted as well.
// The rover stop on visibilitychange (initRoverControl) is separate and unchanged.
const HIDDEN_STREAM_CLOSE_MS = 1000;
const hiddenStreams = { timer: null, closed: null };
function suspendSensorStreamsForHiddenPage() {
  hiddenStreams.timer = null;
  if (!document.hidden) return;
  const closed = {
    lidar: Boolean(state.lidarLive.socket || state.lidarLive.enabled) && !state.lidarLive.userStopped,
    camera: Boolean(state.cameraLive.socket || state.cameraLive.enabled) && !state.cameraLive.userStopped,
    imu: Boolean(state.imuLive.socket || state.imuLive.enabled) && !state.imuLive.userStopped,
  };
  const previous = hiddenStreams.closed || {};
  hiddenStreams.closed = {
    lidar: Boolean(previous.lidar || closed.lidar),
    camera: Boolean(previous.camera || closed.camera),
    imu: Boolean(previous.imu || closed.imu),
  };
  if (closed.lidar) closeLidarLive("탭이 숨겨져 LiDAR 수신을 쉽니다.", { userStopped: false });
  if (closed.camera) closeCameraLive("탭이 숨겨져 카메라 수신을 쉽니다.", { userStopped: false });
  if (closed.imu) closeImuLive("탭이 숨겨져 IMU 수신을 쉽니다.", { userStopped: false });
}
function resumeSensorStreamsForVisiblePage() {
  if (hiddenStreams.timer) { window.clearTimeout(hiddenStreams.timer); hiddenStreams.timer = null; }
  const closed = hiddenStreams.closed || {};
  hiddenStreams.closed = null;
  // A stream closed (or not yet opened) for an off-screen picture is asked again too: it opens if its picture is on screen.
  if ((closed.lidar || sensorPanels.lidar.closed) && routeSupportsLidarLive() && !state.lidarLive.socket && !state.lidarLive.userStopped) {
    state.lidarLive.reconnectAttempt = 0; startLidarLive();
  }
  const cameraPausedByAck = state.cameraLive.ackExhausted === true;
  if ((closed.camera || cameraPausedByAck || cameraPanel.closed) && routeSupportsCameraLive() && !state.cameraLive.socket &&
      (cameraPausedByAck || !state.cameraLive.userStopped)) {
    state.cameraLive.reconnectAttempt = 0;
    // An off-screen camera panel stays closed until it is scrolled back (NET-08).
    if (cameraPanel.visible === false) cameraPanel.closed = true;
    else startCameraLive();
  }
  if ((closed.imu || sensorPanels.imu.closed) && routeSupportsImuLive() && !state.imuLive.socket && !state.imuLive.userStopped) {
    state.imuLive.reconnectAttempt = 0; startImuLive();
  }
}
function handleSensorStreamVisibility() {
  if (document.hidden) {
    if (!hiddenStreams.timer) hiddenStreams.timer = window.setTimeout(suspendSensorStreamsForHiddenPage, HIDDEN_STREAM_CLOSE_MS);
    return;
  }
  resumeSensorStreamsForVisiblePage();
  tick();
}

// Hidden camera panel (NET-08): when the picture box of the camera has been off
// screen (scrolled away, collapsed or display:none) for 1 s, the camera socket is
// closed (not a user stop) and opened again as soon as the box is back on screen.
// Nothing else changes: a user stop stays a user stop and the recording is untouched.
const CAMERA_PANEL_HIDDEN_MS = 1000;
const cameraPanel = { observer: null, element: null, visible: null, timer: null, closed: false };
function cameraPanelElement() {
  const image = $(currentRoute() === "/rover-control" ? "roverCameraImage" : "cameraLiveImage");
  return image?.parentElement || image || null;
}
function watchCameraPanel() {
  if (typeof IntersectionObserver !== "function") return;
  const element = cameraPanelElement();
  if (element === cameraPanel.element) return;
  if (!cameraPanel.observer) cameraPanel.observer = new IntersectionObserver(onCameraPanelIntersection);
  if (cameraPanel.element) cameraPanel.observer.unobserve(cameraPanel.element);
  cameraPanel.element = element;
  cameraPanel.visible = null;
  if (element) cameraPanel.observer.observe(element);
}
function onCameraPanelIntersection(entries) {
  const entry = [...entries].reverse().find((item) => item.target === cameraPanel.element);
  if (!entry) return;
  cameraPanel.visible = Boolean(entry.isIntersecting);
  if (cameraPanel.visible) {
    if (cameraPanel.timer) { window.clearTimeout(cameraPanel.timer); cameraPanel.timer = null; }
    // In a hidden tab it stays closed; resumeSensorStreamsForVisiblePage opens it when the tab is back (UI-16).
    if (cameraPanel.closed && !document.hidden) {
      cameraPanel.closed = false;
      if (routeSupportsCameraLive() && !state.cameraLive.socket && !state.cameraLive.userStopped) {
        state.cameraLive.reconnectAttempt = 0;
        startCameraLive();
      }
    }
  } else if (!cameraPanel.timer) {
    cameraPanel.timer = window.setTimeout(closeCameraForHiddenPanel, CAMERA_PANEL_HIDDEN_MS);
  }
}
function closeCameraForHiddenPanel() {
  cameraPanel.timer = null;
  if (cameraPanel.visible !== false) return;
  if ((state.cameraLive.socket || state.cameraLive.enabled) && !state.cameraLive.userStopped) {
    cameraPanel.closed = true;
    closeCameraLive("카메라 화면이 보이지 않아 수신을 쉽니다.", { userStopped: false });
  }
}

// Hidden LiDAR and IMU pictures (UI-16): Rover Control holds the LiDAR, camera and IMU panels together
// again, so each of these sockets follows its own picture like the camera above: off screen for 1 s
// closes it (not a user stop), back on screen opens it again; a start waits for the observer's first
// report. Without IntersectionObserver the sockets open as before.
const sensorPanels = {
  lidar: { observer: null, element: null, visible: null, timer: null, closed: false },
  imu: { observer: null, element: null, visible: null, timer: null, closed: false },
};
function sensorPanelElement(name) {
  const onRover = currentRoute() === "/rover-control";
  const canvas = $(name === "imu" ? (onRover ? "roverImuCanvas" : "") : (onRover ? "roverLidarCanvas" : "lidarScanCanvas"));
  return canvas?.parentElement || canvas || null;
}
function sensorPanelLive(name) {
  return name === "imu" ? state.imuLive : state.lidarLive;
}
function watchSensorPanel(name) {
  if (typeof IntersectionObserver !== "function") return;
  const panel = sensorPanels[name];
  const element = sensorPanelElement(name);
  if (element === panel.element) return;
  if (!panel.observer) panel.observer = new IntersectionObserver((entries) => onSensorPanelIntersection(name, entries));
  if (panel.element) panel.observer.unobserve(panel.element);
  panel.element = element;
  panel.visible = null;
  if (element) panel.observer.observe(element);
}
// True when a start has to wait for the picture to be on screen.
function sensorPanelDefersStart(name) {
  watchSensorPanel(name);
  const panel = sensorPanels[name];
  if (!panel.observer || !panel.element || panel.visible === true) return false;
  sensorPanelLive(name).userStopped = false;
  panel.closed = true;
  return true;
}
function onSensorPanelIntersection(name, entries) {
  const panel = sensorPanels[name];
  const entry = [...entries].reverse().find((item) => item.target === panel.element);
  if (!entry) return;
  panel.visible = Boolean(entry.isIntersecting);
  if (panel.visible) {
    if (panel.timer) { window.clearTimeout(panel.timer); panel.timer = null; }
    // In a hidden tab the stream stays closed; resumeSensorStreamsForVisiblePage opens it when the tab is back.
    if (panel.closed && !document.hidden) {
      panel.closed = false;
      const live = sensorPanelLive(name);
      const supported = name === "imu" ? routeSupportsImuLive() : routeSupportsLidarLive();
      if (supported && !live.socket && !live.userStopped) {
        live.reconnectAttempt = 0;
        if (name === "imu") startImuLive(); else startLidarLive();
      }
    }
  } else if (!panel.timer) {
    panel.timer = window.setTimeout(() => closeSensorForHiddenPanel(name), CAMERA_PANEL_HIDDEN_MS);
  }
}
function closeSensorForHiddenPanel(name) {
  const panel = sensorPanels[name];
  panel.timer = null;
  if (panel.visible !== false) return;
  const live = sensorPanelLive(name);
  if ((live.socket || live.enabled) && !live.userStopped) {
    panel.closed = true;
    if (name === "imu") closeImuLive("IMU 화면이 보이지 않아 수신을 쉽니다.", { userStopped: false });
    else closeLidarLive("LiDAR 화면이 보이지 않아 수신을 쉽니다.", { userStopped: false });
  }
}

function toggleLidarLive() {
  autoLidar.borrowed = false;            // the button is the user's own choice from here on
  if (state.lidarLive.connected || state.lidarLive.socket) {
    stopLidarLive("사용자가 실시간 연결을 중지했습니다.");
    return;
  }
  startLidarLive();
}

// LiDAR in the 자율주행 tab (2026-11-20, user: "자율주행 시에 라이다 데이터라도 실시간으로 받아오게 하면 좋을 듯", in the
// place of the live occupancy grid map of that tab). The LiDAR picture of Rover Control is a part of 수동 조종 and of
// 자율주행 (one element, one socket, the same drawing). In 자율주행 it needs no button:
//   - a stream the user did not stop follows its picture as on 수동 조종 (the rule above: open while the picture is on
//     screen, closed after 1 s off screen or in a hidden page, opened again by itself);
//   - a stream the user stopped (the LiDAR button of 수동 조종 or of Connection) is started when 자율주행 is shown, and
//     stopped again as the user left it when that tab or the page is left. Only this borrowed start is undone here.
// The 자율주행 tab shows one sentence under the picture instead of the button (autoLidarSentence, drawn by
// renderLidarLiveState and once a second while the tab is shown). A picture that is no longer fed is named as such: no
// message for AUTO_LIDAR_STALE_MS (the bridge sends 3 to 10 scans a second), a status without scans, or a lost
// connection leave the last scan on the canvas, and the sentence says that it is the last one.
// The autonomy buttons, the heartbeat and the run recording do not depend on any of this.
const AUTO_LIDAR_STALE_MS = 3000;
const autoLidar = { borrowed: false };
function autoLidarShown() {
  return currentRoute() === "/rover-control" && typeof roverTab === "function" && roverTab() === "auto";
}
function syncAutoLidar() {
  const shown = autoLidarShown();
  if (shown && state.lidarLive.userStopped) {
    autoLidar.borrowed = true;
    state.lidarLive.userStopped = false;
    state.lidarLive.reconnectAttempt = 0;
    startLidarLive();
  } else if (!shown && autoLidar.borrowed) {
    autoLidar.borrowed = false;
    stopLidarLive("자율주행 탭을 떠나 LiDAR 수신을 멈춥니다.");
  }
  if (currentRoute() === "/rover-control") renderLidarLiveState();   // the button or the sentence of the tab now shown
}
function autoLidarSentence(now = Date.now()) {
  const live = state.lidarLive;
  const drawn = Boolean(state.lidarScan?.points?.length);       // a scan is on the canvas
  const fresh = live.lastMessageAt !== null && live.lastMessageAt !== undefined && now - live.lastMessageAt <= AUTO_LIDAR_STALE_MS;
  if (live.connected && live.scanTopicAlive && fresh) return "LiDAR를 실시간으로 받고 있습니다.";
  if (live.connected) return drawn ? "LiDAR 스캔이 오지 않습니다. 그림은 마지막으로 받은 스캔입니다." : "LiDAR에 연결했습니다. 스캔을 기다리고 있습니다.";
  if (live.socket) return "LiDAR에 연결하고 있습니다.";
  if (live.enabled) {
    return drawn ? "LiDAR 연결이 끊겼습니다. 그림은 마지막으로 받은 스캔입니다."
      : "LiDAR에 연결하지 못했습니다. 자율주행 조작은 그대로 할 수 있습니다.";
  }
  return "이 그림이 화면에 보이면 LiDAR를 받기 시작합니다.";
}
// The sentence of the tab (and the button of the other tab); nothing else of the LiDAR part is drawn here.
function renderAutoLidarNote() {
  const inAutoTab = autoLidarShown();
  const button = $("roverLidarToggle");
  if (button) button.hidden = inAutoTab;
  const note = $("roverLidarNote");
  if (!note) return;
  note.hidden = !inAutoTab;
  const text = inAutoTab ? autoLidarSentence() : "";
  if (note.textContent !== text) note.textContent = text;
}
window.addEventListener("ubts:rover-tab", syncAutoLidar);
window.addEventListener("ubts:route", syncAutoLidar);
// A stream that went quiet sends nothing that would redraw the sentence: look once a second while the tab is shown.
setInterval(() => { if (!document.hidden && autoLidarShown()) renderAutoLidarNote(); }, 1000);

function startLidarLive() {
  if (!routeSupportsLidarLive()) {
    debugLog("ws", "skip start outside live routes");
    return;
  }
  if (state.lidarLive.socket) return;
  clearLidarReconnectTimer();
  if (sensorPanelDefersStart("lidar")) return;   // off screen: opens when the picture is visible (UI-16)
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
    socket.binaryType = "arraybuffer";  // polar-v2 scans are binary (NET-07)
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
      state.lidarLive.reconnectAttempt = 0;
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
        }, nextSensorReconnectDelay(state.lidarLive, event));
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
// The plan every channel follows. A chosen transfer quality is used as it is; in
// automatic mode the link grade decides (04-link-grade.js, NET-10).
function effectiveLinkProfile() {
  const chosen = liveNetworkProfile();
  if (chosen !== "auto") return chosen;
  const graded = window.LinkGrade?.profile?.();
  if (["detail", "balanced", "slow", "tiny"].includes(graded)) return graded;
  const measured = state.cameraLive.effectiveProfile;
  return ["detail", "balanced", "slow", "tiny"].includes(measured) ? measured : "balanced";
}
window.RoverLink = { profile: effectiveLinkProfile, cameraEffectiveProfile: () => state.cameraLive.effectiveProfile || "",
  autonomyPriority: () => state.autonomyPriority || "disabled" };
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
  const chosen = liveNetworkProfile();
  // Automatic mode follows the plan of the link grade; a chosen quality keeps the page's settings.
  const plan = chosen === "auto" ? cameraStreamPlan() : null;
  const params = new URLSearchParams({
    resolution: plan ? plan.resolution : state.cameraLive.resolution,
    fps: String(plan ? plan.fps : state.cameraLive.fps),
    quality: String(plan ? plan.quality : 70),
    profile: plan ? plan.profile : chosen,
    transport: "binary-v1",
    flow: "decode-ack",
  });
  if (cameraPreviewMode() === "original") {
    params.set("resolution", "1920x1080");
    params.set("quality_mode", "original");
  }
  // V1 remains the default. Only an explicit supported-client opt-in asks
  // the server to negotiate more than one outstanding preview frame; in
  // automatic mode the plan says how many pictures may be on the way.
  const explicitWindow = new URLSearchParams(location.search).get("camera_ack_window") ??
    (plan ? String(plan.window) : window.UBTS_CAMERA_ACK_WINDOW);
  if (["2", "3"].includes(String(explicitWindow))) {
    params.set("flow", "decode-ack-v2");
    params.set("ack_window", String(explicitWindow));
  }
  if (window.UBTS_CAMERA_WS_URL) {
    const separator = String(window.UBTS_CAMERA_WS_URL).includes("?") ? "&" : "?";
    return `${window.UBTS_CAMERA_WS_URL}${separator}${params.toString()}`;
  }
  const scheme = location.protocol === "https:" ? "wss" : "ws";
  return `${scheme}://${location.host}/ws/camera?${params.toString()}`;
}

// Which preview the camera asks for (NET-08). The 1920x1080 original JPEG (about
// 112 kB a picture) is a preview choice only; the recording is always the original.
// It is used when it is asked for: ?camera_preview=original, or the transfer quality
// "detail" on a dashboard that allows it (window.UBTS_CAMERA_ORIGINAL_PREVIEW).
// Automatic mode starts with the bridge's re-encoded ladder (640x360 up to 1280x720).
function cameraPreviewMode(profile = liveNetworkProfile()) {
  const explicit = new URLSearchParams(location.search).get("camera_preview");
  if (explicit === "original" || explicit === "legacy") return explicit;
  if (window.UBTS_CAMERA_ORIGINAL_PREVIEW !== true) return "legacy";
  if (profile === "auto") return cameraStreamPlan().original ? "original" : "legacy";
  return profile === "detail" ? "original" : "legacy";
}

// Camera preview per link grade in automatic mode (NET-08 (1)(3), NET-10). The
// size, rate and quality bound what the camera may take of the link so that
// control answers do not wait behind it (pictures measured on the bench frames:
// 640x360 Q40 about 19 kB, 320x180 Q35 about 6 kB, 160x90 Q30 about 2 kB):
//   good      original 1080 preview where allowed, one picture on the way
//   normal    640x360 Q40, 6 per second, one on the way (two let a control answer
//             wait behind two pictures: command p95 312 ms on the bench, 5a15199)
//   slow      320x180 Q35, 5 per second, two on the way
//   very slow the bridge's "tiny" profile (160x90, 2 per second, 3 kB/s)
const CAMERA_STREAM_PLAN = {
  detail: { original: true, profile: "detail", resolution: "1280x720", fps: 30, quality: 70, window: 1 },
  balanced: { original: false, profile: "balanced", resolution: "640x360", fps: 6, quality: 40, window: 1 },
  slow: { original: false, profile: "balanced", resolution: "320x180", fps: 5, quality: 35, window: 2 },
  tiny: { original: false, profile: "tiny", resolution: "320x180", fps: 2, quality: 30, window: 1 },
};
function cameraStreamPlan(profile = effectiveLinkProfile()) {
  return CAMERA_STREAM_PLAN[profile] || CAMERA_STREAM_PLAN.balanced;
}

function startCameraLive() {
  if (!routeSupportsCameraLive()) {
    debugLog("camera", "skip start outside live routes");
    return;
  }
  if (state.cameraLive.socket) return;
  clearCameraReconnectTimer();
  watchCameraPanel();
  if (cameraPanel.visible === false || (cameraPanel.observer && cameraPanel.element && cameraPanel.visible === null)) {
    // Off screen, or not reported yet (the observer's first report follows within a frame): open when the
    // picture box is visible (NET-08). Rover Control holds many panels; the camera may be below the fold.
    state.cameraLive.userStopped = false;
    cameraPanel.closed = true;
    return;
  }
  const url = cameraWebSocketUrl();
  state.cameraLive.enabled = true;
  state.cameraLive.userStopped = false;
  state.cameraLive.ackExhausted = false;
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
      state.cameraLive.reconnectAttempt = 0;
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
        state.cameraLive.ackExhausted = true;
        state.cameraLive.deliveryState = "bandwidth_limited";
        state.cameraLive.error = "영상 응답 지연으로 일시 정지했습니다. 회선 상태나 전송 품질을 확인한 뒤 다시 시작하세요.";
      } else if (event.code === 1013 && String(event.reason || "").includes("viewer limit")) {
        state.cameraLive.error = "카메라를 보는 화면 수가 한도에 닿았습니다. 30초 뒤 다시 연결합니다.";
      }
      const shouldReconnect = state.cameraLive.enabled && !state.cameraLive.userStopped && routeSupportsCameraLive();
      resetCameraWindowDecode(socket);
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
        }, nextSensorReconnectDelay(state.cameraLive, event));
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
  resetCameraWindowDecode(socket);
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
    resetCameraWindowDecode(socket);
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
    if (payload.type === "frame") state.cameraLive.originalJpeg = payload.jpeg_passthrough === true;
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
      if (payload.ack_protocol === 2) displayWindowedCameraFrame(binary.blob, payload, binary.socket);
      else displayBinaryCameraFrame(binary.blob, payload, binary.socket);
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

function resetCameraWindowDecode(socket = null) {
  const queue = state.cameraLive.v2Decode;
  if (!queue || (socket && queue.socket !== socket)) return;
  queue.closed = true;
  queue.pending = null;
  queue.cancelActive?.();
  queue.cancelActive = null;
  state.cameraLive.v2Decode = null;
}

function displayWindowedCameraFrame(blob, payload, socket, timing = null) {
  if (socket !== state.cameraLive.socket) return;
  const valid = payload.ack_protocol === 2 && /^[0-9a-f]{32}$/.test(payload.stream_session ?? "") &&
    Number.isSafeInteger(payload.source_epoch) && payload.source_epoch > 0 &&
    Number.isSafeInteger(payload.delivery_sequence) && payload.delivery_sequence > 0 &&
    Number.isSafeInteger(payload.frame_count) && payload.frame_count > 0 && blob.size <= 5 * 1024 * 1024;
  if (!valid) throw new Error("camera V2 frame identity or byte bound");
  let queue = state.cameraLive.v2Decode;
  if (queue && queue.socket === socket && (queue.session !== payload.stream_session || queue.epoch !== payload.source_epoch)) {
    resetCameraWindowDecode(socket);
    socket.close(1002, "camera source changed");
    return;
  }
  if (!queue || queue.socket !== socket) {
    resetCameraWindowDecode();
    queue = {socket, session: payload.stream_session, epoch: payload.source_epoch, lastSequence: 0, active: false, pending: null, cancelActive: null, closed: false};
    state.cameraLive.v2Decode = queue;
  }
  if (payload.delivery_sequence <= queue.lastSequence) return;
  queue.lastSequence = payload.delivery_sequence;
  const newest = {blob, payload, socket, timing};
  if (queue.active) {
    if (queue.pending) acknowledgeCameraFrame(socket, queue.pending.payload.frame_count, "superseded", queue.pending.payload);
    queue.pending = newest;
    return;
  }
  function start(item) {
    if (queue.closed || queue !== state.cameraLive.v2Decode || socket !== state.cameraLive.socket) return;
    queue.active = true;
    queue.cancelActive = displayBinaryCameraFrame(item.blob, item.payload, socket, item.timing, () => {
      if (queue.closed || queue !== state.cameraLive.v2Decode || socket !== state.cameraLive.socket) return;
      queue.active = false; queue.cancelActive = null;
      const next = queue.pending; queue.pending = null;
      if (next) start(next);
    });
  }
  start(newest);
}

function displayBinaryCameraFrame(blob, payload, socket, timing = null, onSettled = null) {
  // Decode only the visible route; the hidden second image used to double decode work.
  const targets = [$(currentRoute() === "/rover-control" ? "roverCameraImage" : "cameraLiveImage")].filter(Boolean);
  const url = URL.createObjectURL(blob);
  const previousUrl = state.cameraLive.objectUrl;
  state.cameraLive.objectUrl = url;
  if (!targets.length) {
    URL.revokeObjectURL(url);
    state.cameraLive.objectUrl = previousUrl;
    acknowledgeCameraFrame(socket, payload.frame_count, "not_visible", payload);
    onSettled?.();
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
      onSettled?.();
      return;
    }
    for (const image of targets) { image.onload = null; image.onerror = null; }
    const statusChanged = state.cameraLive.decodeStatus !== status;
    state.cameraLive.decodeStatus = status;
    if (status === "decoded") {
      // Original sensor pixels declare orientation; preview pixels are already
      // upright. Replace the transform once after decode, never accumulate it.
      for (const image of targets) image.style.transform = cameraDisplayRotation(payload);
      state.cameraLive.lastDecodedAt = performance.now();
      if (previousUrl) URL.revokeObjectURL(previousUrl);
    } else {
      // Keep the last usable image; an error/timeout is not a rendered frame.
      for (const image of targets) image.src = previousUrl || "";
      state.cameraLive.objectUrl = previousUrl;
      URL.revokeObjectURL(url);
    }
    acknowledgeCameraFrame(socket, payload.frame_count, status, payload);
    if (statusChanged) renderCameraLiveState();
    onSettled?.();
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
  return () => {
    if (settled) return;
    settled = true;
    if (timer !== null) window.clearTimeout(timer);
    if (state.cameraLive.objectUrl === url) {
      for (const image of targets) { image.onload = null; image.onerror = null; image.src = previousUrl || ""; }
      state.cameraLive.objectUrl = previousUrl;
    }
    URL.revokeObjectURL(url);
  };
}

function cameraDisplayRotation(payload) {
  return payload.jpeg_passthrough === true && payload.source_transport === "ros2-original-jpeg" &&
    payload.display_rotation_deg === 180 ? "rotate(180deg)" : "";
}

function acknowledgeCameraFrame(socket, frameCount, decodeStatus = "unknown", delivery = null) {
  if (socket !== state.cameraLive.socket || socket?.readyState !== WebSocket.OPEN) return;
  try {
    const ack = { type: "frame_ack", frame_count: Number(frameCount) || 0, decode_status: decodeStatus };
    if (delivery?.ack_protocol === 2) Object.assign(ack, {ack_protocol: 2, stream_session: delivery.stream_session,
      source_epoch: delivery.source_epoch, delivery_sequence: delivery.delivery_sequence});
    socket.send(JSON.stringify(ack));
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
    : state.cameraLive.originalJpeg ? "원본 JPEG · 재압축 없음"
    : `${state.cameraLive.effectiveProfile || "auto"} · Q${state.cameraLive.previewQuality ?? "--"}`;
  const ackLabel = Number.isFinite(state.cameraLive.ackLatencyMs) ? ` · ACK ${Math.round(state.cameraLive.ackLatencyMs)}ms` : "";
  const observedDetail = `ROS ${cameraFpsLabel(state.cameraLive.sourceFps)} · WS ${cameraFpsLabel(state.cameraLive.displayFps)} · ${cameraDelivery}${ackLabel}`;
  const colors = chartColors();
  renderReadout("cameraLiveStats", [
    { label: "WebSocket", value: state.cameraLive.connected ? "ON" : "OFF", sub: socketText, color: state.cameraLive.connected ? colors.good : colors.rose },
    { label: "캡처 설정", value: state.cameraLive.resolution, sub: `목표 ${state.cameraLive.fps} fps`, color: colors.amber },
    { label: "출처", value: source, sub: state.cameraLive.frameId || "camera_link_optical", color: colors.cyan },
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
    { label: "WebSocket", value: state.cameraLive.connected ? "ON" : "OFF", sub: socketText, color: state.cameraLive.connected ? colors.good : colors.rose },
    { label: "캡처 설정", value: state.cameraLive.resolution, sub: `목표 ${state.cameraLive.fps} fps`, color: colors.amber },
    { label: "출처", value: source, sub: state.cameraLive.frameId || "camera_link_optical", color: colors.cyan },
    { label: "수신 FPS", value: cameraFpsLabel(observedFps), sub: `${observedDetail} · ${frameAge}`, color: colors.blue },
  ]);
}
