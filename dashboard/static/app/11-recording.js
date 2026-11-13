
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
  if (currentRoute() !== "/records") return;
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
  text("roverRecorderState", !known ? "상태 미확인" : active ? "recording" : "ready");
  text("roverRecorderKind", active ? sensorLabel : status.archive_count !== undefined ? `${status.archive_count} archives` : "대기");
  const imuToggle = $("roverRecordImuToggle");
  const gpsToggle = $("roverRecordGpsToggle");
  if (gpsToggle) { gpsToggle.checked = includeGps; gpsToggle.disabled = state.recordings.actionInFlight || active || !known; }
  if (imuToggle) {
    imuToggle.checked = includeImu;
    imuToggle.disabled = state.recordings.actionInFlight || active || !known;
  }
  renderFrameRecordState(known, active);
}

// Recording indicator of the status bar. On the records screen it shows what the
// recorder panel shows (ordered status reads with the 5 s freshness gate).
// The other routes do not poll the recorder: there the newest telemetry row
// is used, and nothing is claimed once telemetry has stopped.
function renderFrameRecordState(known, active) {
  const el = $("frameRecordState");
  if (!el) return;
  if (currentRoute() !== "/records") {
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
