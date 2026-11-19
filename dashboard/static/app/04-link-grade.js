// ---- Link grade (NET-10) ------------------------------------------------------
// One place that judges the link to the dashboard and, in automatic transfer
// quality, chooses the plan of every channel (camera, LiDAR, IMU, GPS, status
// polls, request limiter) through effectiveLinkProfile().
//
// Measure: a tiny GET /api/ping (204, no body) twice a second, and while driving
// also the round trip of every control request. The server time (Server-Timing)
// is taken off, so a sample is the network round trip including queueing.
// Judge: the smallest sample of the last 4 s (queueing only adds time, so the
// smallest is close to the bare round trip):
//   good < 25 ms, normal < 90 ms, slow < 220 ms, otherwise very slow.
// A worse grade is taken when the smallest of 4 s is 25 % past the boundary; a
// better one only after it has held for 10 s with 15 % margin, so a link that
// swings does not swing the plan. Two pings in a row without an answer (3 s
// each) is "offline". Seen on the bench (097519b), and why the window is time:
// - with "the last 4 samples", the control answers while driving (about 5 a
//   second) all waited behind one camera picture now and then, the grade fell,
//   the smaller plan made the link fast again, and the plan changed 13 times in
//   2 minutes on the normal link;
// - a 30 % upgrade margin left a 150 ms link that once looked slower stuck in
//   the very-slow plan (160 ms is never under 0.7 x 220 ms);
// - without driving there are only the pings: with one a second and 15 %, all
//   four pings of 4 s sometimes waited behind a camera picture (640x360, 76 ms
//   on the normal link) and the plan changed 4 times in a minute (c84bfa8,
//   hidden/normal). Two pings a second and 25 % keep a normal link normal.
// The thresholds are assumptions; the field link log (below) is meant to replace them.
//
// Field log (optional, NET-10 (5)): with ?link_log=1 or localStorage
// "ubts-link-log" = "1" the page sends its samples every 10 s to the dashboard,
// which keeps them in runtime/link_log/ (one JSONL file a day).
const LINK_GRADE = {
  pingIntervalMs: 500,
  pingTimeoutMs: 3000,
  windowMs: 4000,
  warmupSamples: 4,
  degradeMargin: 1.25,
  thresholdsMs: { good: 25, normal: 90, slow: 220 },
  upgradeHoldMs: 10000,
  upgradeMargin: 0.85,
  offlineAfterFailures: 2,
  order: ["good", "normal", "slow", "very_slow"],
  profile: { good: "detail", normal: "balanced", slow: "slow", very_slow: "tiny", offline: null, unknown: "slow" },
  label: { good: "연결 좋음", normal: "연결 보통", slow: "연결 느림", very_slow: "연결 아주 느림", offline: "연결 끊김", unknown: "연결 확인 중" },
  level: { good: "ok", normal: "ok", slow: "warn", very_slow: "warn", offline: "bad", unknown: "" },
};
const linkGrade = {
  grade: "unknown",
  profile: LINK_GRADE.profile.unknown,
  samples: [],
  failures: 0,
  betterSince: 0,
  betterGrade: null,
  changes: [],
  timer: null,
  inFlight: false,
  log: [],
  logTimer: null,
};

function linkGradeOf(rttMs, margin = 1) {
  const t = LINK_GRADE.thresholdsMs;
  if (rttMs < t.good * margin) return "good";
  if (rttMs < t.normal * margin) return "normal";
  if (rttMs < t.slow * margin) return "slow";
  return "very_slow";
}

// The decision, separate from timers and the network so it can be tested.
// `now` is a millisecond clock. Returns the new grade (may be unchanged).
function nextLinkGrade(model, now) {
  if (model.failures >= LINK_GRADE.offlineAfterFailures) return "offline";
  // After an outage only answers to requests sent after the link came back count:
  // a ping sent during a 10 s stall answers late (say 1 s) and is not the link's
  // round trip. Until such an answer is there the grade before the outage stays.
  const since = model.recoveredAt || 0;
  const fresh = model.samples.filter((sample) => sample[2] >= since);
  if (model.grade === "offline") {
    if (!fresh.length) return model.beforeOffline || "offline";
  }
  const inWindow = fresh.filter((sample) => now - sample[0] <= LINK_GRADE.windowMs).map((sample) => sample[1]);
  const recent = inWindow.length ? inWindow : fresh.slice(-1).map((sample) => sample[1]);
  if (!recent.length) return model.grade === "offline" ? "offline" : model.grade;
  // The first answer often comes while the page is still starting (seen: 300-500 ms
  // on localhost); wait for a second one, and take the first seconds without the
  // upgrade hold so the page does not sit in the very-slow plan for 10 s.
  if (model.grade === "unknown" && model.samples.length < 2) return "unknown";
  const warmingUp = (model.count || model.samples.length) < LINK_GRADE.warmupSamples;
  const base = Math.min(...recent);
  const current = model.grade;
  const rank = (grade) => LINK_GRADE.order.indexOf(grade);
  if (warmingUp || current === "unknown") {
    model.betterGrade = null;
    return linkGradeOf(base);
  }
  if (current === "offline") {
    // Back from an outage: the grade before it, then the usual rules from there.
    model.betterGrade = null;
    const before = model.beforeOffline && model.beforeOffline !== "unknown" ? model.beforeOffline : linkGradeOf(base);
    return rank(linkGradeOf(base, LINK_GRADE.degradeMargin)) > rank(before) ? linkGradeOf(base, LINK_GRADE.degradeMargin) : before;
  }
  const worse = linkGradeOf(base, LINK_GRADE.degradeMargin);
  if (rank(worse) > rank(current)) {
    model.betterGrade = null;
    return worse;
  }
  const better = linkGradeOf(base, LINK_GRADE.upgradeMargin);
  if (rank(better) < rank(current)) {
    if (model.betterGrade !== better) { model.betterGrade = better; model.betterSince = now; }
    if (now - model.betterSince >= LINK_GRADE.upgradeHoldMs) { model.betterGrade = null; return better; }
  } else {
    model.betterGrade = null;
  }
  return current;
}

function noteLinkSample(rttMs, ok = true, kind = "ping") {
  const now = performance.now();
  if (ok && Number.isFinite(rttMs) && rttMs >= 0) {
    // [answered at, round trip, sent at]
    linkGrade.samples.push([now, rttMs, now - rttMs]);
    linkGrade.count = (linkGrade.count || 0) + 1;
    if (linkGrade.samples.length > 64) linkGrade.samples.splice(0, linkGrade.samples.length - 64);
    if (kind === "ping") {
      if (linkGrade.failures >= LINK_GRADE.offlineAfterFailures) linkGrade.recoveredAt = now;
      linkGrade.failures = 0;
    }
  } else if (kind === "ping") {
    linkGrade.failures += 1;
  }
  if (linkGrade.logOn) {
    linkGrade.log.push({ t: Date.now(), kind, ok: Boolean(ok), rtt_ms: Number.isFinite(rttMs) ? Math.round(rttMs) : null, grade: linkGrade.grade });
    if (linkGrade.log.length > 2000) linkGrade.log.splice(0, linkGrade.log.length - 2000);
  }
  const next = nextLinkGrade(linkGrade, now);
  if (next !== linkGrade.grade) setLinkGrade(next, now);
  else renderLinkBadge();
}

function serverTimeMs(response) {
  const header = response?.headers?.get?.("Server-Timing") || "";
  const match = /dur=([0-9.]+)/.exec(header);
  return match ? Number(match[1]) : 0;
}

async function pingLink() {
  if (linkGrade.inFlight || document.hidden) return;
  linkGrade.inFlight = true;
  const controller = new AbortController();
  const timer = setTimeout(() => controller.abort(), LINK_GRADE.pingTimeoutMs);
  const started = performance.now();
  try {
    const response = await pageFetch("/api/ping", { cache: "no-store", priority: "high", signal: controller.signal });
    // Finish the request (no body) so the browser does not keep it open.
    await response.arrayBuffer().catch(() => null);
    const elapsed = performance.now() - started;
    noteLinkSample(Math.max(0, elapsed - serverTimeMs(response)), response.ok || response.status === 204, "ping");
  } catch (_) {
    noteLinkSample(null, false, "ping");
  } finally {
    clearTimeout(timer);
    linkGrade.inFlight = false;
  }
}

function setLinkGrade(grade, now = performance.now()) {
  const previousProfile = linkGrade.profile;
  const previousGrade = linkGrade.grade;
  if (grade === "offline") linkGrade.beforeOffline = previousGrade;
  linkGrade.grade = grade;
  // The camera stops itself when no picture is answered for 8 s ("preview
  // acknowledgement timeout"); after an outage it is opened again (NET-11).
  if (previousGrade === "offline" && grade !== "offline" && state.cameraLive?.ackExhausted &&
      !document.hidden && typeof routeSupportsCameraLive === "function" && routeSupportsCameraLive()) {
    state.cameraLive.reconnectAttempt = 0;
    setTimeout(startCameraLive, 600);
  }
  linkGrade.changes.push(now);
  if (linkGrade.changes.length > 32) linkGrade.changes.shift();
  // Offline keeps the last plan: nothing is reopened until the link answers again.
  if (LINK_GRADE.profile[grade]) linkGrade.profile = LINK_GRADE.profile[grade];
  renderLinkBadge();
  if (linkGrade.profile !== previousProfile && liveNetworkProfile() === "auto") applyLinkProfile();
}

// Reopen the streams with the plan of the new grade (automatic mode only).
function applyLinkProfile() {
  window.RoverGps?.profileChanged?.();
  if (state.imuLive.socket) { closeImuLive("전송 등급 변경", { userStopped: false }); setTimeout(startImuLive, 250); }
  if (state.cameraLive.socket) { closeCameraLive("전송 등급 변경", { userStopped: false }); setTimeout(startCameraLive, 250); }
  if (state.lidarLive.socket) { closeLidarLive("전송 등급 변경", { userStopped: false }); setTimeout(startLidarLive, 250); }
  state.lastTelemetryPoll = Math.min(state.lastTelemetryPoll || 0, Date.now() - 1000);
}

function linkGradeReason() {
  const now = performance.now();
  const recent = linkGrade.samples.filter((sample) => now - sample[0] <= LINK_GRADE.windowMs).map((sample) => sample[1]);
  const base = recent.length ? Math.round(Math.min(...recent)) : null;
  const rtt = base === null ? "" : ` 왕복 시간은 ${base} ms입니다.`;
  if (linkGrade.grade === "offline") return "연결이 끊겼습니다. 조종 중이었다면 차는 입력이 끊겨 스스로 멈춥니다.";
  if (liveNetworkProfile() !== "auto") return `전송 품질을 직접 골랐습니다.${rtt}`;
  if (linkGrade.grade === "slow" || linkGrade.grade === "very_slow") return `연결이 느려 화면 자료를 줄였습니다.${rtt}`;
  if (linkGrade.grade === "unknown") return "연결을 재는 중입니다.";
  return `연결 상태에 맞춰 화면 자료를 보냅니다.${rtt}`;
}

// The status bar's connection badge shows the grade (one badge, no new element).
function renderLinkBadge() {
  const el = $("connectionState");
  if (!el) return;
  const telemetryDown = (state.telemetryFailures || 0) >= 3;
  const grade = telemetryDown ? "offline" : linkGrade.grade;
  const level = LINK_GRADE.level[grade] ?? "";
  const label = LINK_GRADE.label[grade] || "연결됨";
  const className = level ? `status-chip ${level}` : "status-chip";
  if (el.className !== className) el.className = className;
  if (el.textContent !== label) el.textContent = label;
  const reason = telemetryDown ? "상태 응답이 없습니다. 조종 중이었다면 차는 입력이 끊겨 스스로 멈춥니다." : linkGradeReason();
  if (el.title !== reason) el.title = reason;
}

function linkLogEnabled() {
  try {
    if (new URLSearchParams(location.search).get("link_log") === "1") return true;
    return localStorage.getItem("ubts-link-log") === "1";
  } catch (_) {
    return false;
  }
}

async function flushLinkLog() {
  if (!linkGrade.log.length) return;
  const samples = linkGrade.log.splice(0, 500);
  const recording = state.recordings?.status?.active ? state.recordings.status.recording_id || null : null;
  try {
    await pageFetch("/api/link-log", {
      method: "POST", cache: "no-store", priority: "low",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ samples, recording_id: recording, profile: liveNetworkProfile() }),
    });
  } catch (_) {
    // A lost batch is not retried; the log is a field aid, not a record.
  }
}

function initLinkGrade() {
  if (linkGrade.timer) return;
  linkGrade.logOn = linkLogEnabled();
  pingLink();
  linkGrade.timer = setInterval(pingLink, LINK_GRADE.pingIntervalMs);
  if (linkGrade.logOn) linkGrade.logTimer = setInterval(flushLinkLog, 10000);
  document.addEventListener("visibilitychange", () => { if (!document.hidden) pingLink(); });
  renderLinkBadge();
}

window.LinkGrade = {
  grade: () => linkGrade.grade,
  profile: () => linkGrade.profile,
  constrained: () => linkGrade.profile === "slow" || linkGrade.profile === "tiny",
  samples: () => linkGrade.samples.map((sample) => sample[1]),
  beforeOffline: () => linkGrade.beforeOffline || null,
  note: noteLinkSample,
  changesPerMinute: () => linkGrade.changes.filter((t) => performance.now() - t < 60000).length,
};
