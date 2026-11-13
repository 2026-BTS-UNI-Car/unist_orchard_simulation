
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
      state.imuLive.reconnectAttempt = 0;
      handleImuLiveMessage(event.data);
    });
    socket.addEventListener("error", () => {
      if (socket !== state.imuLive.socket) return;
      state.imuLive.error = "IMU WebSocket 연결 오류";
      renderImuLiveState();
    });
    socket.addEventListener("close", (event) => {
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
        }, nextSensorReconnectDelay(state.imuLive, event));
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
  text("roverImuProcessingState", "원본 그대로");
  const acceleration = sample.availability?.linear_acceleration?.available === false ? {} : sample.linear_acceleration || {};
  const gyro = sample.availability?.angular_velocity?.available === false ? {} : sample.angular_velocity || {};
  const magnetic = state.imuLive.magnetic?.magnetic_field_uT || {};
  const colors = chartColors();
  renderReadout("roverImuStats", [
    { label: "Roll / Pitch", value: Number.isFinite(roll) && Number.isFinite(pitch) ? `${fmtNumber(roll, 1)}° / ${fmtNumber(pitch, 1)}°` : "--", sub: sourceLabel, color: colors.cyan },
    { label: "가속도", value: vectorNorm(acceleration) === null ? "--" : `${fmtNumber(vectorNorm(acceleration), 3)} m/s²`, sub: `x ${fmtNumber(acceleration.x, 2)} · y ${fmtNumber(acceleration.y, 2)} · z ${fmtNumber(acceleration.z, 2)}`, color: colors.good },
    { label: "각속도", value: vectorNorm(gyro) === null ? "--" : `${fmtNumber(vectorNorm(gyro), 4)} rad/s`, sub: `x ${fmtNumber(gyro.x, 3)} · y ${fmtNumber(gyro.y, 3)} · z ${fmtNumber(gyro.z, 3)}`, color: colors.amber },
    { label: "자기장", value: vectorNorm(magnetic) === null ? "--" : `${fmtNumber(vectorNorm(magnetic), 2)} µT`, sub: "sensor-native", color: colors.blue },
  ]);
  drawImuAttitude($("roverImuCanvas"), roll, pitch, yaw);
}

function drawImuAttitude(canvas, roll, pitch, yaw) {
  if (!canvas || document.hidden) return;
  const rect = canvas.getBoundingClientRect();
  if (!(rect.width >= 1 && rect.height >= 1)) return;
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
