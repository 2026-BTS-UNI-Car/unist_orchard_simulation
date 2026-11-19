
function renderRoverControl(data) {
  const rover = data.rover || data.connection?.rover || {};
  const services = data.services || data.connection?.services || {};
  const roverBridge = services.rover_bridge || {};
  // The ESP32-S3 'ABS-HOLD' bridge (ROVER_ESP_KIND=esp32_abs_hold) replaced the ESP8266 on the vehicle (2026-11-16).
  const espName = roverBridge.esp_kind === "esp32_abs_hold" ? "ESP32-S3" : "ESP8266";
  text("roverModeLabel", SIMULATION ? "시뮬레이션" : "실차");
  text("roverControllerLabel", SIMULATION ? "Gazebo" : espName);
  if (typeof adoptBridgeVehicle === "function") adoptBridgeVehicle(roverBridge);   // 14-steer.js
  // Recording controls use their ordered status endpoint. General telemetry
  // can arrive late and must not replace a newer START/STOP acknowledgement.
  const imuSensor = data.sensors?.imu || data.connection?.sensors?.imu || {};
  if (!state.imuLive.topicAlive && imuSensor.raw) {
    state.imuLive.raw = imuSensor.raw || {};
    state.imuLive.magnetic = imuSensor.magnetic || {};
    state.imuLive.topicAlive = Boolean(imuSensor.ok);
    state.imuLive.lastImuAgeSec = imuSensor.last_imu_age_s;
  }
  text("roverControlState", roverBridge.serial_open ? "연결됨" : roverBridge.state || "대기");
  text("roverControlBase", roverBridge.serial_port || "--");
  text("roverProxyState", rover.mode || roverBridge.mode || "esp_manual");
  text("roverRosState", roverBridge.ok ? "준비됨" : roverBridge.state || "대기");
  renderStatusRows("roverControlDetails", [
    {
      title: SIMULATION ? "Simulation Controller" : `${espName} Bridge`,
      sub: SIMULATION ? "가상 로버 수동 조종" : "수동 조종 · 단일 USB 소유자",
      ok: Boolean(roverBridge.ok),
      level: roverBridge.ok ? "ok" : "warn",
      value: roverBridge.state || "대기",
    },
    {
      title: SIMULATION ? "가상 구동 연결" : "ESP USB",
      sub: SIMULATION ? "Gazebo 좌우 휠" : roverBridge.serial_port || "/dev/ubts-motor",
      ok: Boolean(roverBridge.ok),
      level: roverBridge.ok ? "ok" : "warn",
      value: roverBridge.ok ? "연결됨" : "대기",
    },
    {
      title: "ROS 관측",
      sub: "자동 주행 입력은 쓰지 않습니다",
      ok: roverBridge.automatic_input_enabled === false,
      level: "ok",
      value: "비활성화",
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
  document.addEventListener("visibilitychange", handleSensorStreamVisibility);
  document.addEventListener("focusin", (event) => { if (hasActiveRoverInput() && roverKeyTargetBlocked(event)) stopRoverNow("input_focus"); });
  setInterval(() => { if (state.roverControl.lease) { ensureRoverCommandTimer(); } }, 200);

  try {
    const rawSpeed = localStorage.getItem(ROVER_SPEED_KEY);
    const stored = rawSpeed === null ? null : Number(rawSpeed);
    if (Number.isFinite(stored)) state.roverControl.speedPct = Math.max(0, Math.min(100, stored));
    const archiveRange = Number(localStorage.getItem(ARCHIVE_RANGE_KEY));
    if (Number.isFinite(archiveRange)) state.archivePlayback.range = clampLidarRange(archiveRange);
    state.recordings.sensors = storedRecordingSensors(localStorage);
  } catch (_) {
    // Speed defaults remain usable without localStorage.
  }
  $("roverSpeedSlider")?.addEventListener("input", (event) => setRoverSpeed(Number(event.target.value), true));
  $("roverStopButton")?.addEventListener("click", () => stopRoverNow("operator_stop"));
  initFrameStop();
  $("autoStopButton")?.addEventListener("click", () => stopRoverNow("operator_stop"));   // the 자율주행 tab's STOP: the same path
  $("roverCameraToggle")?.addEventListener("click", toggleCameraLive);
  $("roverLidarToggle")?.addEventListener("click", toggleLidarLive);
  $("roverImuToggle")?.addEventListener("click", toggleImuLive);
  $("roverRecordStartButton")?.addEventListener("click", () => startRecording());
  document.querySelectorAll("[data-record-sensor]").forEach((box) => {
    box.addEventListener("change", () => setRecordingSensor(box.dataset.recordSensor, box.checked, true));
  });
  $("roverRecordStopButton")?.addEventListener("click", stopRecording);
  $("roverArchiveRefreshButton")?.addEventListener("click", () => loadRecordings().catch(console.error));
  $("roverArchivePlayButton")?.addEventListener("click", toggleArchivePlayback);
  $("roverArchiveScrubber")?.addEventListener("input", (event) => setArchiveTime(Number(event.target.value), { pause: true }));
  $("roverArchiveSyncToggle")?.addEventListener("change", (event) => {
    state.archivePlayback.sync = Boolean(event.target.checked);
    state.archivePlayback.lastCameraRenderKey = "";
    renderArchiveFrame();
  });

  $("roverArchiveRangeSlider")?.addEventListener("input", (event) => setArchiveRange(Number(event.target.value), true));


  initRoverPad();
  document.addEventListener("keydown", handleRoverKeyDown);
  document.addEventListener("keyup", handleRoverKeyUp);
  setRoverSpeed(state.roverControl.speedPct, false);
  setRecordingSensors(state.recordings.sensors, false);
  initRoverTabs();

  setArchiveRange(state.archivePlayback.range, false);

  renderRoverButtons();
  renderRecordingStatus();
  renderRecordings();
}

// Direction pad. Press, release, cancel and leave act exactly as before, and a
// mouse is handled exactly as before. A finger or a pen keeps the button until
// it lifts: without that the page took a drifting thumb over as a scroll,
// cancelled the pointer and dropped the input while the finger was still down.
// However the pointer is lost, the input ends through releaseRoverInput().
function initRoverPad() {
  let padPointerType = "mouse";
  document.querySelectorAll("[data-rover-drive]").forEach((button) => {
    button.addEventListener("pointerdown", (event) => {
      event.preventDefault();
      padPointerType = event.pointerType || "mouse";
      if (padPointerType !== "mouse") {
        try { button.setPointerCapture(event.pointerId); } catch (_) { /* pointer already gone: its cancel ends the input */ }
      }
      const action = button.dataset.roverDrive || "stop";
      if (action === "stop") stopRoverNow("pad_stop");
      else startRoverButtonCommand(action);
    });
    button.addEventListener("pointerup", () => releaseRoverInput("pad_release"));
    button.addEventListener("pointercancel", () => releaseRoverInput("pad_cancel"));
    button.addEventListener("pointerleave", () => {
      if (state.roverControl.activeButton) releaseRoverInput("pad_leave");
    });
    button.addEventListener("lostpointercapture", () => {
      if (state.roverControl.activeButton) releaseRoverInput("pad_capture_lost");
    });
    // A long press on a touch screen must not open a menu over a held button.
    button.addEventListener("contextmenu", (event) => {
      if (padPointerType !== "mouse") event.preventDefault();
    });
  });
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

// ---- In-page tabs of Rover Control (2026-11-19) ---------------------------
// User: "보정관련된 영역을 좀 분리해서 보여주면 좋겠음 아예 탭을 따로 만들던가", "자율주행 관련 영역을 따로 만들던지 탭을 새로
// 만들던지". The menu does not change: the tabs only show and hide the parts of this one page (each part names its tabs
// in data-rover-tab, the style sheet hides the others). The chosen tab is kept per browser. Leaving 수동 조종 while this
// page holds the manual Arm stops it (the pad is no longer on screen; the steering panel does the same in steer-drive.mjs).
// Everything that streams watches its own box, so a hidden part stops asking (camera, LiDAR, GPS, maps).
const ROVER_TABS = ["manual", "auto", "setup", "record"];

function roverTab() {
  const tab = $("roverLayout")?.dataset.tab;
  return ROVER_TABS.includes(tab) ? tab : "manual";
}

// The first tab that shows an element (null when it is not inside a tabbed part).
function roverTabOf(id) {
  const holder = $(id)?.closest?.("[data-rover-tab]");
  return holder ? holder.dataset.roverTab.split(" ")[0] : null;
}

// True when the element is in a part of the active tab (or not inside a tabbed part).
function roverPartShown(id) {
  const holder = $(id)?.closest?.("[data-rover-tab]");
  if ($(id) === null) return false;
  return !holder || holder.dataset.roverTab.split(" ").includes(roverTab());
}

function setRoverTab(name, { persist = true } = {}) {
  const layout = $("roverLayout");
  if (!layout) return;
  const tab = ROVER_TABS.includes(name) ? name : "manual";
  const before = layout.dataset.tab;
  layout.dataset.tab = tab;
  document.querySelectorAll("[data-rover-tab-button]").forEach((button) => {
    const on = button.dataset.roverTabButton === tab;
    button.setAttribute("aria-selected", String(on));
    button.tabIndex = on ? 0 : -1;
  });
  if (persist) {
    try { localStorage.setItem(ROVER_TAB_KEY, tab); } catch (_) { /* this load only */ }
  }
  if (before && before !== tab) roverTabChanged(before, tab);
}

function roverTabChanged(before, tab) {
  if (before === "manual" && (state.roverControl.lease || state.roverControl.inFlight)) stopRoverNow("tab_changed");
  if (state.archivePlayback.playing && !roverPartShown("roverArchiveList")) stopArchivePlayback(false);
  renderRecordings();
  requestChartAnimation();
  window.LiveMap?.routeChanged();
  window.RoverGps?.routeChanged();
  window.dispatchEvent(new CustomEvent("ubts:rover-tab", { detail: { tab, before } }));
}

function initRoverTabs() {
  let stored = null;
  try { stored = localStorage.getItem(ROVER_TAB_KEY); } catch (_) { stored = null; }
  setRoverTab(stored, { persist: false });
  const buttons = [...document.querySelectorAll("[data-rover-tab-button]")];
  buttons.forEach((button, index) => {
    button.addEventListener("click", () => setRoverTab(button.dataset.roverTabButton));
    button.addEventListener("keydown", (event) => {
      if (event.key !== "ArrowRight" && event.key !== "ArrowLeft") return;
      event.preventDefault();
      const next = buttons[(index + (event.key === "ArrowRight" ? 1 : buttons.length - 1)) % buttons.length];
      setRoverTab(next.dataset.roverTabButton);
      next.focus();
    });
  });
  // A button inside a part that leads to another tab (e.g. the distance move's tape entry).
  document.querySelectorAll("[data-rover-tab-goto]").forEach((button) => {
    button.addEventListener("click", () => setRoverTab(button.dataset.roverTabGoto));
  });
}

// ---- Recording sensors (2026-11-19) -------------------------------------
// User: "녹화시에 ... 우리가 센서가 좀 많잖아? 그거 전부 하나하나 선택 가능하도록". One check box per recorder sensor group;
// at least LiDAR or the camera (the recorder's own rule, sensors_need_lidar_or_camera). The choice is kept per browser.
const RECORDING_SENSORS = ["lidar", "camera", "imu", "gps", "drive", "steering", "executor", "tf"];

function storedRecordingSensors(storage) {
  let raw = null;
  try { raw = JSON.parse(storage?.getItem(ROVER_RECORD_SENSORS_KEY) || "null"); } catch (_) { raw = null; }
  if (Array.isArray(raw)) {
    const chosen = RECORDING_SENSORS.filter((name) => raw.includes(name));
    if (chosen.includes("lidar") || chosen.includes("camera")) return chosen;
  }
  // The choice of the old page (kind and the IMU switch), when there is one.
  let kind = null, imu = null;
  try { kind = storage?.getItem(ROVER_RECORD_KIND_KEY); imu = storage?.getItem(IMU_RECORD_ENABLED_KEY); } catch (_) { kind = imu = null; }
  if (!["lidar", "camera", "both"].includes(kind) && imu === null) return [...state.recordings.sensors];
  const out = new Set(["gps", "drive", "steering", "executor", "tf"]);
  if (kind === "camera" || kind === "both") out.add("camera");
  if (kind !== "camera") out.add("lidar");
  if (imu !== "0") out.add("imu");
  return RECORDING_SENSORS.filter((name) => out.has(name));
}

function setRecordingSensors(sensors, persist) {
  state.recordings.sensors = RECORDING_SENSORS.filter((name) => (sensors || []).includes(name));
  // The old request fields follow the choice (a recorder without "sensors" still gets what it knows).
  const lidar = state.recordings.sensors.includes("lidar"), camera = state.recordings.sensors.includes("camera");
  state.recordings.kind = lidar && camera ? "both" : camera ? "camera" : "lidar";
  state.recordings.includeImu = state.recordings.sensors.includes("imu");
  state.recordings.includeGps = state.recordings.sensors.includes("gps");
  if (persist) {
    try { localStorage.setItem(ROVER_RECORD_SENSORS_KEY, JSON.stringify(state.recordings.sensors)); } catch (_) { /* this load only */ }
  }
  renderRecordingStatus();
}

function setRecordingSensor(name, on, persist) {
  const next = new Set(state.recordings.sensors);
  if (on) next.add(name); else next.delete(name);
  setRecordingSensors([...next], persist);
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
    const result = await controlRequest("/api/rover/arm");
    if (!result.ok && result.error === "steer_profile_required") {
      // The bridge is the front-steer one: this pad cannot drive it. Take the steering panel instead (14-steer.js).
      if (typeof adoptBridgeVehicle === "function") adoptBridgeVehicle("front_steer");
      throw new Error("조향 차량용 브리지라서 조향 패널로 전환했습니다. 조향 패널에서 조종 활성화를 다시 누르세요.");
    }
    if (!result.ok) throw new Error(result.message || result.error);
    if (epoch !== (c.epoch || 0)) { await controlRequest("/api/rover/stop", {reason: "cancelled_arm"}); return; }
    c.lease = result.lease; c.ticket = result.ticket; c.seq = result.seq;
    c.pendingRelease = false;
    setRoverCommandStatus("ok", "조종이 활성화됐습니다. WASD 이동 · Q/E 속도 −5/+5% · Esc 즉시 정지");
    ensureRoverCommandTimer();
    renderRoverMotionReadout();
  } catch (err) {
    // the reason as a sentence: a sentence as it is (without the address in front), a code named in one
    const why = String(err.message || "").replace(/^\/api\/\S+: /, "");
    setRoverCommandStatus("bad", `조종 활성화를 하지 못했습니다. ${/[.?]\s*$/.test(why) ? why : `알려 온 이유는 ${why}입니다.`}`);
  }
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
  if (!state.roverControl.lease) { setRoverCommandStatus("bad", "먼저 조종 활성화를 누르세요."); return; }
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
  if (!state.roverControl.commandTimer) state.roverControl.commandTimer = setInterval(stepRoverMotion, 140);
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
  try {
    const result = await controlRequest(`/api/rover/${op}`, {lease: c.lease, ticket: c.ticket, seq: c.seq + 1,
      left: motion.left, right: motion.right, deadman: op === "command"});
    if (epoch !== (c.epoch || 0)) return;
    if (!result.ok) throw new Error(result.error || "control rejected");
    c.ticket = result.ticket; c.seq = result.seq;
    c.left = motion.left; c.right = motion.right;
    renderRoverMotionReadout();
  } catch (err) {
    if (epoch === (c.epoch || 0)) { stopRoverNow("command_failed"); const why = String(err.message || "").replace(/^\/api\/\S+: /, "");
      setRoverCommandStatus("bad", `명령이 실패해 정지를 요청했습니다. ${/[.?]\s*$/.test(why) ? why : `알려 온 이유는 ${why}입니다.`} 다시 조종하려면 조종 활성화를 누르세요.`); }
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
  setRoverCommandStatus("testing", "정지를 요청하고 있습니다. 조종은 해제했습니다.");
  try {
    const result = await controlRequest("/api/rover/stop", {reason});
    setRoverCommandStatus(result.ok ? "ok" : "bad", result.ok ? "정지 요청이 접수됐습니다. 다시 조종하려면 조종 활성화를 누르세요."
      : `정지 요청이 거절됐습니다. 알려 온 이유는 ${result.error}입니다.`);
  } catch (_) { setRoverCommandStatus("bad", "정지 요청에 응답이 없습니다. 입력이 끊기면 차는 스스로 멈춥니다. 연결이 복구되면 상태를 확인하세요."); }
  finally { if (epoch === c.epoch) c.disarming = false; renderRoverButtons(); }
}
function roverCommandMessage(payload) { return payload.ok ? (SIMULATION ? "가상 로버 요청 접수" : "ESP 요청 접수") : payload.error || "제어 실패"; }

function setRoverCommandStatus(kind, message) {
  // stopRoverNow() reports its result while it is still disarming. A STOP that
  // got no answer or was refused ("not confirmed") stays visible in the status
  // bar on every route.
  state.roverControl.stopFailed = kind === "bad" && Boolean(state.roverControl.disarming);
  renderFrameDriveState();
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
    armButton.disabled = Boolean(c.arming || c.disarming);
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
  renderFrameDriveState();
}

// ---- Status bar (application frame) -----------------------------------
// The STOP in the status bar is the drive panel STOP on every route: the same
// function with the same reason, so the same request.
function initFrameStop() {
  $("frameStopButton")?.addEventListener("click", () => stopRoverNow("operator_stop"));
}

// Drive control state. It stays visible when the drive panel is collapsed and
// on the other routes. Who holds control is not decided here.
function renderFrameDriveState() {
  const el = $("frameDriveState");
  if (!el) return;
  const c = state.roverControl;
  const [level, label] = c.arming ? ["", "활성화 중"]
    : c.disarming ? ["", "정지 요청 중"]
    : c.lease ? ["warn", "조종 활성"]
    : c.stopFailed ? ["bad", "정지 미확인"]
    : ["", "조종 비활성"];
  const className = level ? `status-chip ${level}` : "status-chip";
  if (el.className !== className) el.className = className;
  if (el.textContent !== label) el.textContent = label;
}

function renderFrameClock() {
  const now = new Date();
  const two = (value) => String(value).padStart(2, "0");
  text("frameClock", `${two(now.getHours())}:${two(now.getMinutes())}:${two(now.getSeconds())}`);
}

function renderRoverMotionReadout() {
  const c = state.roverControl;
  const colors = chartColors();
  renderReadout("roverMotionReadout", [
    {label: SIMULATION ? "왼쪽 속도 요청" : "왼쪽 요청 PWM", value: `${c.left || 0}%`, sub: SIMULATION ? "최대 가상 휠 속도 0.50 m/s의 비율" : "실제 속도는 아님", color: colors.cyan},
    {label: SIMULATION ? "오른쪽 속도 요청" : "오른쪽 요청 PWM", value: `${c.right || 0}%`, sub: SIMULATION ? "가상 휠 가감속 적용" : "MCU 가감속 적용", color: colors.blue},
    {label: "조종", value: c.lease ? "활성" : "해제", sub: "입력 만료 0.75초 · Esc 즉시 정지", color: colors.good},
  ]);
}
