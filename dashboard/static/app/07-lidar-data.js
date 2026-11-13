
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
    try {
      renderLidarScan(result, "live_websocket");
    } catch (err) {
      // A render error must not stop the stream: the bridge waits for this ACK.
      state.lidarLive.renderErrors = (state.lidarLive.renderErrors || 0) + 1;
      debugLog("ws", "scan render failed", err);
    } finally {
      if (state.lidarLive.socket?.readyState === WebSocket.OPEN && Number.isInteger(payload.sequence)) {
        state.lidarLive.socket.send(JSON.stringify({type: "scan_ack", sequence: payload.sequence}));
      }
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
  const socketText = overrideLabel || (state.lidarLive.connected ? "connected" : state.lidarLive.error || "disconnected");
  const pointLabel = state.lidarLive.pointCount
    ? `${state.lidarLive.pointCount}/${state.lidarLive.sourcePointCount || state.lidarLive.pointCount}`
    : "--";
  const colors = chartColors();
  renderReadout("roverLidarStats", [
    { label: "WebSocket", value: state.lidarLive.connected ? "ON" : "OFF", sub: socketText, color: state.lidarLive.connected ? colors.good : colors.rose },
    { label: "출처", value: state.lidarLive.scanTopicAlive ? "live" : lidarSourceLabel(state.lidarLive.source), sub: state.telemetry?.connection?.lidar?.device || "/scan", color: colors.cyan },
    { label: "점 수", value: pointLabel, sub: lidarAgeLabel(), color: colors.blue },
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
  ctx.fillStyle = cssVar("--chart-surface", "#ffffff");
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
