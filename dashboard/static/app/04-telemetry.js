

async function loadDocs() {
  const payload = await getJson("/api/docs");
  state.docs = payload.documents || [];
  renderDocsIndex(state.docs);
  renderDocsRoute();
}

async function tick() {
  if (document.hidden) return;
  const interval = telemetryIntervalMs();
  if (state.lastTelemetryPoll && Date.now() - state.lastTelemetryPoll < interval) return;
  state.lastTelemetryPoll = Date.now();
  if (state.tickInFlight) {
    debugLog("api", "/api/telemetry skipped: previous request still running");
    return;
  }
  state.tickInFlight = true;
  const cameraSettingsRevision = state.cameraLive?.settingsRevision || 0;
  const controller = new AbortController();
  const timeout = window.setTimeout(() => controller.abort(), 6000);
  try {
    const path = telemetryPath();
    state.telemetryPath = path;
    state.telemetry = await getJson(path, { signal: controller.signal });
    state.telemetryCameraSettingsRevision = cameraSettingsRevision;
    pushHistory(state.telemetry);
    renderTelemetry(state.telemetry);
    state.telemetryFailures = 0;
    setConnection("ok", "연결됨");
    requestChartAnimation({ redrawLidar: false });
  } catch (err) {
    state.telemetryFailures += 1;
    if (state.telemetryFailures >= 3) setConnection("bad", "연결 끊김");
    else debugLog("api", `/api/telemetry transient failure ${state.telemetryFailures}`, err);
  } finally {
    window.clearTimeout(timeout);
    state.tickInFlight = false;
  }
}

// Rover Control asks only for its own status (NET-06): sensors, services and
// the rover, about a third of the full answer. Every other page gets the full one.
function telemetryPath(route = currentRoute()) {
  return route === "/rover-control" ? "/api/telemetry?bundle=drive" : "/api/telemetry";
}

function telemetryIntervalMs(profile = liveNetworkProfile()) {
  // Automatic mode follows the link grade (NET-10): 1 s on a good link, 10 s on a very slow one.
  if (profile === "auto") {
    const graded = effectiveLinkProfile();
    return { detail: 1000, balanced: 2000, slow: 3000, tiny: 10000 }[graded] || 3000;
  }
  return profile === "tiny" ? 10000 : profile === "slow" ? 3000 : 1000;
}
function chartWindowLabel() {
  return `X 최근 ${Math.round(MAX_POINTS * telemetryIntervalMs() / 1000)}초`;
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
  state.chartShiftDurationMs = Math.max(500, telemetryIntervalMs() * 1.08);
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
  // The badge shows the link grade (04-link-grade.js) and turns "연결 끊김" when status stops.
  if (typeof renderLinkBadge === "function") renderLinkBadge();
  else { el.className = `status-chip ${kind}`; el.textContent = label; }
  // Telemetry stopped: the status bar must not keep an old recorder state.
  if (kind !== "ok") renderRecordingStatus();
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
  text("overviewDockerText", docker.available ? "실행 중 / 전체" : docker.error || "Docker 소켓 없음");
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
  text("networkExternalIpText", net.external_ip ? "공개 주소" : "확인 대기");
  text("networkInternalIp", net.primary_ipv4 || "--");
  text("networkInternalIpText", [net.primary_interface, net.hostname].filter(Boolean).join(" · ") || "--");
  text("networkThroughput", `RX ${fmtBps(net.total_rx_bps)} · TX ${fmtBps(net.total_tx_bps)}`);
  renderNetworkMode(net.control || {});

  text("dockerContainerCount", docker.available ? docker.summary?.total ?? "--" : "N/A");
  text("dockerRunningText", docker.available ? `실행 ${docker.summary?.running || 0} · 정지 ${docker.summary?.stopped || 0}` : docker.error || "--");
  text("dockerImageCount", docker.available ? docker.engine?.images ?? "--" : "N/A");
  text("dockerVersion", docker.available ? `Docker ${docker.engine?.server_version || "--"}` : "--");
  text("dockerCpuTotal", fmtPercent(docker.summary?.cpu_percent));
  text("dockerCpuChartValue", fmtPercent(docker.summary?.cpu_percent));
  const dockerMemPercent = docker.summary?.memory_limit ? (docker.summary.memory_usage / docker.summary.memory_limit) * 100 : null;
  text("dockerMemTotal", fmtPercent(dockerMemPercent));
  text("dockerMemChartValue", fmtPercent(dockerMemPercent));
  text("dockerMemText", `${fmtBytes(docker.summary?.memory_usage)} / ${fmtBytes(docker.summary?.memory_limit)}`);

  renderFan(fan);

  renderServices(services);
  renderConnection(connection);
  renderRoverControl(data);
  renderCores(sys.cpu?.cores || []);
  renderCoreCharts(sys.cpu?.cores || []);
  renderDisks(sys.disks || []);
  renderInterfaces(net.interfaces || [], net.primary_interface);
  renderContainers(docker);
  maybeRefreshContainerDetail();
  renderChartReadouts(data);
}

function networkModeLabel(control) {
  if (control.ethernet_connected && control.ap_active) return "Ethernet + AP";
  if (control.transition === "wifi_scan") return "Wi-Fi 검색 중";
  if (control.transition === "wifi_trial") return "Wi-Fi 시험 연결";
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
    { label: "방식", value: mode, sub: control.connection || control.transition || "--", color: colors.cyan },
    { label: "AP", value: control.ap_ssid || "BTS_Jetson", sub: control.ap_address || "10.77.0.1", color: colors.blue },
    { label: "AP 접속 기기", value: control.ap_clients ?? 0, sub: countdownText, color: colors.good },
    { label: "Ethernet", value: control.ethernet_connected ? "connected" : "not connected", sub: control.last_error || control.reason || "--", color: control.ethernet_connected ? colors.good : colors.amber },
    ...(control.link ? [wifiLinkReadout(control.link, colors)] : []),
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
  wifiScreen?.render(control);
}

// Wi-Fi management of the Network page (static/network/wifi-manager.mjs) is fetched the first time the
// Network page opens, so no page's first load grows (NET-03); then it follows every status (above).
let wifiScreen = null;
let wifiScreenLoad = null;

function loadWifiScreen(route = currentRoute()) {
  if (wifiScreen) wifiScreen.routeChanged();
  if (route !== "/network" || wifiScreenLoad) return;
  wifiScreenLoad = import("/static/network/wifi-manager.mjs")
    .then((module) => {
      wifiScreen = module.initWifiManager({ getJson, currentRoute, debugLog, telemetryFailures: () => state.telemetryFailures || 0 });
      wifiScreen.render(state.telemetry?.network?.control || {});
    })
    .catch((err) => { wifiScreenLoad = null; console.warn("wifi manager", err); });
}

// The Wi-Fi link the Jetson uses now (iw dev link on the host): signal and transmit rate.
function wifiLinkReadout(link, colors) {
  const band = link.band ? link.band.replace("GHz", " GHz") : "";
  const rate = Number.isFinite(Number(link.tx_mbps)) && link.tx_mbps !== null ? `송신 ${Math.round(link.tx_mbps)} Mbit/s` : "";
  return {
    label: "Wi-Fi 신호",
    value: Number.isFinite(Number(link.signal_dbm)) && link.signal_dbm !== null ? `${link.signal_dbm} dBm` : "--",
    sub: [band, rate].filter(Boolean).join(" · ") || "--",
    color: colors.cyan,
  };
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
  window.addEventListener("ubts:route", (event) => loadWifiScreen(event.detail?.route));
}
