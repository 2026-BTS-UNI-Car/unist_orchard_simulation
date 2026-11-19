
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

// The sensors of a recording request (2026-11-19: one check box per sensor). A state without the list (an older
// caller) asks for what its kind and switches said.
const RECORDING_SENSOR_LABEL = { lidar: "LiDAR", camera: "카메라", imu: "IMU", gps: "GPS", drive: "구동", steering: "조향",
  executor: "실행기", tf: "TF" };
const RECORDING_ALL_SENSORS = Object.keys(RECORDING_SENSOR_LABEL);
const RECORDING_ERROR_TEXT = {
  sensors_need_lidar_or_camera: "LiDAR나 카메라 가운데 하나는 골라야 합니다.",
  recording_busy: "기록 요청을 이미 처리하고 있습니다.",
  recording_finalization_in_progress: "이전 기록을 마무리하는 중입니다.",
  previous_sidecar_fsync_still_owned: "이전 기록의 저장을 마무리하는 중입니다.",
  previous_mapping_writer_still_owned: "이전 기록의 저장을 마무리하는 중입니다.",
  status_timeout: "녹화기 상태 응답이 제때 오지 않았습니다.",
  no_reply: "녹화기가 응답하지 않았습니다.",
};
// Recorder states that are not an error, as the status line says them (idle reads as "기록을 시작할 수 있습니다").
const RECORDING_STATE_TEXT = { finalizing: "이전 기록을 마무리하고 있습니다.", already_recording: "이미 기록 중입니다." };

// A recorder error for the status line: the table's sentence, a sentence as it is (without the address in front), a
// server error number, or the recorder's own code named in a sentence.
function recordingErrorSentence(error) {
  const text = String(error || "").replace(/^\/api\/\S+: /, "");
  if (!text) return "";
  if (RECORDING_ERROR_TEXT[text]) return RECORDING_ERROR_TEXT[text];
  if (/^\d{3}$/.test(text)) return `녹화기 요청에 서버가 HTTP ${text} 오류로 답했습니다.`;
  return /[.?]\s*$/.test(text) ? text : `녹화기가 알린 이유는 ${text}입니다.`;
}

function recordingSensorsOf(recordings) {
  if (Array.isArray(recordings?.sensors)) return recordings.sensors.filter((name) => RECORDING_ALL_SENSORS.includes(name));
  const kind = recordings?.kind || "both";
  return RECORDING_ALL_SENSORS.filter((name) => (name === "lidar" && kind !== "camera") || (name === "camera" && kind !== "lidar")
    || (name === "imu" && recordings?.includeImu !== false) || (name === "gps" && recordings?.includeGps !== false));
}

// The request body: the new fields (sensors, tag) and the old ones (kind, include_imu, include_gps) for a recorder that
// does not know the list yet. A recorder that knows "sensors" uses it.
function recordingStartBody(sensors, tag = "manual") {
  const lidar = sensors.includes("lidar"), camera = sensors.includes("camera");
  return { sensors: [...sensors], tag: tag === "autonomy" ? "autonomy" : "manual",
    kind: lidar && camera ? "both" : camera ? "camera" : "lidar",
    include_imu: sensors.includes("imu"), include_gps: sensors.includes("gps") };
}

function recordingSensorsLabel(sensors) {
  const list = (sensors || []).filter((name) => RECORDING_SENSOR_LABEL[name]);
  if (list.length === RECORDING_ALL_SENSORS.length) return "모든 센서";
  return list.map((name) => RECORDING_SENSOR_LABEL[name]).join("+") || "센서 없음";
}

// The recorder state as the panels read it: known only within the 5 s freshness gate. segment: the server's record of
// the newest stretch of an autonomy run (dashboard_server/run_recording.py; the autonomy tab reads its times).
function recordingStatusView() {
  const status = state.recordings.status || {};
  const ageMs = performance.now() - state.recordings.lastStatusSteadyAt;
  const known = status.ok === true && status.status_fresh !== false
    && typeof status.active === "boolean"
    && Number.isFinite(ageMs) && ageMs >= 0 && ageMs <= 5000;
  return { known, active: known && status.active, status, busy: Boolean(state.recordings.actionInFlight),
           id: known && status.active ? status.recording_id || null : null, tag: status.tag || null,
           sensors: Array.isArray(status.sensors) ? status.sensors : null,
           segment: status.autonomy_segment && typeof status.autonomy_segment === "object" ? status.autonomy_segment : null };
}

// options: {sensors, tag} (the autonomy panel: every sensor, tag "autonomy"); without them the check boxes.
async function startRecording(options = {}) {
  if (state.recordings.actionInFlight) return { ok: false, error: "recording_busy" };
  const sensors = Array.isArray(options?.sensors) ? options.sensors : recordingSensorsOf(state.recordings);
  if (!sensors.includes("lidar") && !sensors.includes("camera")) {
    state.recordings.lastError = RECORDING_ERROR_TEXT.sensors_need_lidar_or_camera;
    renderRecordingStatus();
    return { ok: false, error: "sensors_need_lidar_or_camera" };
  }
  state.recordings.actionInFlight = true;
  state.recordings.lastError = "";
  invalidateRecordingStatusRead();
  const requestedAt = performance.now();
  renderRecordingStatus("기록을 시작하고 있습니다.");
  let payload = null;
  try {
	    payload = await postJson("/api/recordings/start", recordingStartBody(sensors, options?.tag));
	    state.recordings.status = payload.status || payload;
	    state.recordings.lastStatusAt = Date.now();
    state.recordings.lastStatusSteadyAt = requestedAt;
    if (payload && payload.ok === false) {
      state.recordings.lastError = `기록을 시작하지 못했습니다. ${recordingErrorSentence(payload.error) || RECORDING_STATE_TEXT[payload.state] || recordingErrorSentence(payload.state) || "이유는 알 수 없습니다."}`;
    }
    renderRecordingStatus();
    await loadRecordings();
  } catch (err) {
    state.recordings.status = { ok: false, state: "start_failed", error: err.message };
    payload = { ok: false, error: err.message };
    renderRecordingStatus();
  } finally {
    state.recordings.actionInFlight = false;
    renderRecordingStatus();
  }
  return payload || { ok: false, error: "no_reply" };
}

async function stopRecording() {
  if (state.recordings.actionInFlight) return { ok: false, error: "recording_busy" };
  let result = null;
  state.recordings.actionInFlight = true;
  state.recordings.lastError = "";
  invalidateRecordingStatusRead();
  const requestedAt = performance.now();
  renderRecordingStatus("기록을 멈추고 있습니다.");
  try {
    const payload = await postJson("/api/recordings/stop", {});
    result = payload;
    state.recordings.status = payload.status || payload;
    state.recordings.lastStatusAt = Date.now();
    state.recordings.lastStatusSteadyAt = requestedAt;
    renderRecordingStatus();
    await loadRecordings();
  } catch (err) {
    state.recordings.status = { ok: false, state: "stop_failed", error: err.message };
    result = { ok: false, error: err.message };
    renderRecordingStatus();
  } finally {
    state.recordings.actionInFlight = false;
    renderRecordingStatus();
  }
  return result || { ok: false, error: "no_reply" };
}

function recordingKindLabel(kind) {
  return { lidar: "LiDAR", camera: "카메라", both: "LiDAR+카메라" }[kind] || kind || "대기";
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
  const listed = payload?.sensors || payload?.metadata?.sensors;
  if (Array.isArray(listed) && listed.length) return recordingSensorsLabel(listed);
  const kind = payload?.kind || payload?.metadata?.kind;
  const label = recordingKindLabel(kind);
  const sensors = recordingImuFrameCount(payload) > 0 ? `${label}+IMU` : label;
  const gpsCount = payload?.timeline?.gps_frames ?? payload?.timeline_summary?.gps_frames ?? payload?.metadata?.timeline?.gps_frames ?? 0;
  return Number(gpsCount) > 0 ? `${sensors}+GPS` : sensors;
}

function renderRecordingStatus(override) {
  const { status, known, active } = recordingStatusView();
  // This secondary row shares the exact primary freshness gate. It must age
  // locally even when general telemetry and recorder requests stop arriving.
  const detail = $("roverRecorderDetail");
  if (detail) {
    detail.querySelector(".status-dot").className = `status-dot ${known ? "ok" : "warn"}`;
    detail.querySelector("small").textContent = known
      ? status.recording_id || status.status_url || "ros2-data-recorder"
      : "현재 상태를 확인할 수 없습니다";
    detail.querySelector("b").textContent = status.state === "disabled" ? "꺼짐"
      : !known ? "상태 미확인"
      : active ? `${recordingKindLabel(status.kind)} 기록 중` : status.state === "idle" ? "대기" : status.state || "대기";
  }
  const chosen = recordingSensorsOf(state.recordings);
  const shown = active ? (Array.isArray(status.sensors) ? status.sensors : recordingSensorsOf({
    kind: status.kind, includeImu: status.include_imu !== false, includeGps: status.include_gps !== false })) : chosen;
  const sensorLabel = recordingSensorsLabel(shown);
  const noImage = !chosen.includes("lidar") && !chosen.includes("camera");
  const duration = recordingDurationSeconds(status);
  const message = override || (!known && status.state !== "disabled"
    ? "기록 상태를 확인하고 있습니다. 계속 이 글이 보이면 연결을 확인하세요."
    : active
    ? `${sensorLabel} 기록 중${status.tag === "autonomy" ? " · 자율주행" : ""} · ${status.recording_id || ""}`
    : state.recordings.lastError || recordingErrorSentence(status.error) || (noImage ? RECORDING_ERROR_TEXT.sensors_need_lidar_or_camera : "")
      || RECORDING_STATE_TEXT[status.state] || (status.state && status.state !== "idle" ? status.state : "") || `${sensorLabel} 기록을 시작할 수 있습니다.`);
  const target = $("roverRecordingStatus");
  if (target) {
    target.className = `test-result ${known && !state.recordings.lastError ? "ok" : status.error || state.recordings.lastError ? "bad" : "testing"}`;
    target.textContent = message;
  }
  document.querySelectorAll("[data-record-sensor]").forEach((box) => {
    box.checked = shown.includes(box.dataset.recordSensor);
    box.disabled = state.recordings.actionInFlight || active || !known;
  });
  const start = $("roverRecordStartButton");
  if (start) {
    start.disabled = state.recordings.actionInFlight || active || !known || status.state === "disabled" || noImage;
    start.title = noImage ? RECORDING_ERROR_TEXT.sensors_need_lidar_or_camera : "";
    start.textContent = status.state === "disabled" ? "기록 기능 꺼짐" : active ? "기록 중"
      : `${chosen.length > 3 && chosen.length < RECORDING_ALL_SENSORS.length ? `센서 ${chosen.length}개` : recordingSensorsLabel(chosen)} 기록 시작`;
  }
  const stop = $("roverRecordStopButton");
  if (stop) stop.disabled = state.recordings.actionInFlight || (known && !active) || status.state === "disabled";
  text("roverRecordingTimer", !known ? "--:--" : active ? fmtClock(duration) : "00:00");
  text("roverRecorderState", !known ? "상태 미확인" : active ? "기록 중" : "대기");
  text("roverRecorderKind", active ? sensorLabel : status.archive_count !== undefined ? `기록 ${status.archive_count}개` : "대기");
  renderFrameRecordState(known, active);
}

// Recording indicator of the status bar. On the drive page it shows what the
// recorder panel shows (ordered status reads with the 5 s freshness gate).
// The other routes do not poll the recorder: there the newest telemetry row
// is used, and nothing is claimed once telemetry has stopped.
function renderFrameRecordState(known, active) {
  const el = $("frameRecordState");
  if (!el) return;
  if (currentRoute() !== "/rover-control") {
    const service = state.telemetry?.services?.data_recorder;
    known = Boolean(service?.ok) && !(state.telemetryFailures >= 3);
    active = known && service.active === true;
  }
  const className = active ? "status-chip recording" : "status-chip";
  const label = !known ? "기록 미확인" : active ? "기록 중" : "기록 대기";
  if (el.className !== className) el.className = className;
  if (el.textContent !== label) el.textContent = label;
}

function recordingDurationSeconds(status) {
  if (Number.isFinite(Number(status.duration_s))) return Number(status.duration_s);
  if (!status.started_at) return 0;
  const started = new Date(status.started_at).getTime();
  if (!Number.isFinite(started)) return 0;
  return Math.max(0, (Date.now() - started) / 1000);
}
