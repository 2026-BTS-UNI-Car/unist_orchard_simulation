
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
	    { label: "그린 점", value: String(renderedPoints), sub: `상한 ${requestedPoints} · 원본 ${sourcePoints}`, color: colors.cyan },
	    { label: "각도 범위", value: `${fmtNumber(metrics.coverage_deg, 0)}°`, sub: "1° 단위", color: colors.good },
	    { label: "거리 범위", value: `${fmtNumber(metrics.min_distance_m, 2)}-${fmtNumber(metrics.max_distance_m, 2)} m`, sub: "유효한 점", color: colors.blue },
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
      <small>Y 0-100%<br /><span data-role="window">${chartWindowLabel()}</span></small>
    </div>
  `;
  observeCanvasResize(article.querySelector("canvas"));
  return article;
}

function updateCoreChart(el, core) {
  el.querySelector('[data-role="name"]').textContent = cpuLabel(core);
  el.querySelector('[data-role="value"]').textContent = fmtPercent(core.percent);
  const windowLabel = el.querySelector('[data-role="window"]');
  if (windowLabel) windowLabel.textContent = chartWindowLabel();
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
      <div><span>링크</span><b data-role="link"></b></div>
      <div><span>RX · TX</span><b data-role="traffic"></b></div>
    </div>
  `;
  return div;
}

function updateInterfaceCard(el, iface, primaryInterface) {
  el.querySelector('[data-role="name"]').textContent = iface.name === primaryInterface ? `${iface.name} · 기본` : iface.name;
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
        ${containerDetailCell("상태", containerStateLabel(stateInfo), stateInfo.StartedAt ? `${formatShortDate(stateInfo.StartedAt)}부터` : "")}
        ${containerDetailCell("이미지", detailPayload.image || "--", detailPayload.full_id || detailPayload.id || "")}
        ${containerDetailCell("CPU", fmtPercent(stats.cpu_percent), "한 번 잰 값")}
        ${containerDetailCell("메모리", `${fmtBytes(stats.memory_usage)} / ${fmtBytes(stats.memory_limit)}`, fmtPercent(stats.memory_percent))}
        ${containerDetailCell("네트워크", `RX ${fmtBytes(stats.rx_bytes)} · TX ${fmtBytes(stats.tx_bytes)}`, detailPayload.network_mode || "")}
        ${containerDetailCell("재시작", restartPolicyLabel(restartPolicy), "재시작 정책")}
      </div>
    `;
  }
  if (logs) {
    if (!logsPayload?.ok) {
      logs.textContent = `로그 읽기 실패: ${logsPayload?.error || "unknown"}`;
    } else if ((logsPayload.lines || []).length) {
      // S33: this runs every 2.5 s. Rewrite only when the text changed, so an
      // unchanged log keeps the reader's text selection.
      const next = logsPayload.lines.join("\n");
      if (logs.textContent !== next) logs.textContent = next;
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
    target.innerHTML = `<span class="container-self-lock">이 대시보드</span>`;
    return;
  }
  const running = Boolean(detailPayload.state?.Running);
  const id = escapeHtml(detailPayload.id || detailPayload.full_id || "");
  const name = escapeHtml(String(detailPayload.name || "").replace(/^\//, ""));
  target.innerHTML = `
    <button type="button" data-container-action="start" data-container-id="${id}" data-container-name="${name}" ${running ? "disabled" : ""}>시작</button>
    <button type="button" data-container-action="stop" data-container-id="${id}" data-container-name="${name}" ${running ? "" : "disabled"}>중지</button>
    <button type="button" data-container-action="restart" data-container-id="${id}" data-container-name="${name}" ${running ? "" : "disabled"}>재시작</button>
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
    // UI-13: stopping or restarting a container can stop a sensor, the drive bridge or the recorder; ask first.
    const verb = { stop: "중지", restart: "재시작" }[button.dataset.containerAction];
    if (verb && !window.confirm(`컨테이너 ${button.dataset.containerName || button.dataset.containerId}을(를) ${verb}할까요?
센서·조종·기록 서비스라면 그동안 화면 연결과 조종이 끊긴다.`)) return;
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
    { label: "메모리", value: fmtPercent(memoryPercent(sys.memory)), sub: `${fmtBytes(sys.memory?.used)} / ${fmtBytes(memoryTotal(sys.memory))}`, color: colors.cyan },
  ]);
  renderReadout("overviewNetworkReadout", [
    { label: "RX", value: fmtBps(net.total_rx_bps), sub: "수신", color: colors.blue },
    { label: "TX", value: fmtBps(net.total_tx_bps), sub: "송신", color: colors.rose },
  ]);
  renderReadout("cpuReadout", [
    { label: "전체", value: fmtPercent(sys.cpu?.total_percent), sub: `코어 ${(sys.cpu?.cores || []).length}개 · 0-100%`, color: colors.good },
    ...((sys.cpu?.cores || []).slice(0, 6).map((core, index) => ({
      label: cpuLabel(core),
      value: fmtPercent(core.percent),
      sub: "개별 사용률",
      color: colors.good,
    }))),
  ]);
  renderReadout("memoryReadout", [
    { label: "사용률", value: fmtPercent(memoryPercent(sys.memory)), sub: memoryLabel(sys.memory), color: colors.cyan },
    { label: "사용", value: fmtBytes(sys.memory?.used), sub: `전체 ${fmtBytes(memoryTotal(sys.memory))}`, color: colors.blue },
    { label: "사용 가능", value: fmtBytes(sys.memory?.available), sub: "MemAvailable", color: colors.good },
  ]);
  renderReadout("swapReadout", [
    { label: "사용률", value: fmtPercent(sys.swap?.percent), sub: `${fmtBytes(sys.swap?.used)} / ${fmtBytes(sys.swap?.total)}`, color: colors.rose },
    { label: "여유", value: fmtBytes(sys.swap?.free), sub: "쓸 수 있는 swap", color: colors.good },
  ]);
  renderReadout("systemNetworkReadout", [
    { label: "RX", value: fmtBps(net.total_rx_bps), sub: "수신", color: colors.blue },
    { label: "TX", value: fmtBps(net.total_tx_bps), sub: "송신", color: colors.rose },
  ]);
  renderReadout("powerReadout", [
    { label: "전력", value: fmtPower(sys.power?.total_w), sub: sys.power?.source || "ina3221", color: colors.amber },
    { label: "전원 레일", value: String((sys.power?.rails || []).length || 0), sub: sys.power?.available ? "측정 가능" : "측정 불가", color: colors.cyan },
  ]);
  renderReadout("networkReadout", [
    { label: "RX", value: fmtBps(net.total_rx_bps), sub: "수신 속도", color: colors.blue },
    { label: "TX", value: fmtBps(net.total_tx_bps), sub: "송신 속도", color: colors.rose },
    { label: "호스트", value: net.primary_ipv4 || "--", sub: net.primary_interface || "--", color: colors.cyan },
  ]);
  renderReadout("dockerCpuReadout", [
    { label: "CPU", value: fmtPercent(docker.summary?.cpu_percent), sub: "컨테이너 합산", color: colors.good },
  ]);
  renderReadout("dockerMemReadout", [
    { label: "메모리", value: fmtPercent(dockerMemPercent), sub: `${fmtBytes(docker.summary?.memory_usage)} / ${fmtBytes(docker.summary?.memory_limit)}`, color: colors.cyan },
  ]);
  renderReadout("fanTempReadout", [
    { label: "온도", value: fmtTemp(fan.temperature_c), sub: fanTemperatureLabel(fan), color: colors.amber },
    { label: "PWM", value: fan.pwm === null || fan.pwm === undefined ? "--" : `${fan.pwm}`, sub: "수동 출력", color: colors.rose },
  ]);
}

function renderReadout(targetId, rows) {
  const target = $(targetId);
  if (!target) return;
  const renderKey = JSON.stringify(rows.map((row) => [row.label, row.value, row.sub, row.color]));
  const cache = renderReadout._ubtsRows ||= new WeakMap();
  let slots = cache.get(target);
  const intact = slots && target.childNodes.length === slots.length && slots.every((slot, index) => {
    const parts = [slot.swatch, slot.label, slot.value, slot.sub];
    const texts = [[slot.label, slot.labelText], [slot.value, slot.valueText], [slot.sub, slot.subText]];
    return target.childNodes[index] === slot.root && slot.root.parentNode === target &&
      slot.root.className === "readout-row" && slot.swatch.className === "swatch" &&
      slot.root.childNodes.length === 4 && parts.every((part, i) => slot.root.childNodes[i] === part) &&
      texts.every(([element, text]) => element.childNodes.length === 1 && element.firstChild === text && text.nodeType === 3);
  });
  if (!intact) {
    target.innerHTML = "";
    slots = [];
    cache.set(target, slots);
  }
  while (slots.length > rows.length) slots.pop().root.remove();
  for (let index = 0; index < rows.length; index++) {
    const row = rows[index];
    let slot = slots[index];
    if (!slot) {
      const root = document.createElement("div"), swatch = document.createElement("i");
      const label = document.createElement("span"), value = document.createElement("b"), sub = document.createElement("small");
      const labelText = document.createTextNode(""), valueText = document.createTextNode(""), subText = document.createTextNode("");
      root.className = "readout-row"; swatch.className = "swatch";
      label.append(labelText); value.append(valueText); sub.append(subText);
      root.append(swatch, label, value, sub); target.append(root);
      slot = {root, swatch, label, value, sub, labelText, valueText, subText, inputColor: {}, appliedStyle: null};
      slots.push(slot);
    }
    // Text node data has the same literal display as escapeHtml + innerHTML,
    // without parsing markup or allocating new rows/text on every sensor tick.
    const values = [String(row.label ?? ""), String(row.value ?? ""), String(row.sub || "")];
    [slot.labelText, slot.valueText, slot.subText].forEach((node, i) => { if (node.data !== values[i]) node.data = values[i]; });
    if (slot.inputColor !== row.color || slot.swatch.style.cssText !== slot.appliedStyle) {
      slot.swatch.style.cssText = "";
      slot.swatch.style.background = row.color; slot.swatch.style.color = row.color;
      slot.inputColor = row.color; slot.appliedStyle = slot.swatch.style.cssText;
    }
  }
  target.dataset.renderKey = renderKey;
}
