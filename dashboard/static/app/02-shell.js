
function currentRoute() {
  if (location.pathname === "/docs" || location.pathname.startsWith("/docs/")) return "/docs";
  return ROUTES.includes(location.pathname) ? location.pathname : "/";
}

function routeSupportsLidarLive(route = currentRoute()) {
  return route === "/connection" || route === "/rover-control";
}

function routeSupportsCameraLive(route = currentRoute()) {
  return route === "/connection" || route === "/rover-control";
}

function routeSupportsImuLive(route = currentRoute()) {
  return route === "/connection" || route === "/rover-control";
}

function renderRoute() {
  const route = currentRoute();
  document.querySelectorAll("[data-page]").forEach((page) => {
    page.classList.toggle("active", page.dataset.page === route);
  });
  document.querySelectorAll("[data-route]").forEach((link) => {
    link.classList.toggle("active", link.dataset.route === route);
  });
  if (route === "/docs") renderDocsRoute();
  if (route === "/docker" && state.telemetry?.docker) {
    renderContainers(state.telemetry.docker);
    maybeRefreshContainerDetail(true);
  }
  if (route === "/connection") {
    renderActiveLidarMap();

    if (!state.lidarLive.socket && !state.lidarLive.userStopped) startLidarLive();
    if (!state.cameraLive.socket && !state.cameraLive.userStopped) startCameraLive();
	    if (!state.imuLive.socket && !state.imuLive.userStopped) startImuLive();
	  } else if (route === "/rover-control") {
	    renderActiveLidarMap();

	    renderRoverControl(state.telemetry || {});
	    renderRecordingStatus();
	    maybeRefreshRecordingStatus();
	    if (!state.recordings.items.length && !state.recordings.loading) loadRecordings().catch(console.error);
	    if (!state.lidarLive.socket && !state.lidarLive.userStopped) startLidarLive();
	    if (!state.cameraLive.socket && !state.cameraLive.userStopped) startCameraLive();
	    if (!state.imuLive.socket && !state.imuLive.userStopped) startImuLive();
	  } else if (state.lidarLive.socket || state.lidarLive.enabled) {
	    closeLidarLive("Live 화면 밖에서는 LiDAR WebSocket을 닫습니다.", { userStopped: false });
	  }
	  if (route !== "/rover-control" && state.archivePlayback.playing) {
	    stopArchivePlayback(false);
	  }
  if (!routeSupportsCameraLive(route) && (state.cameraLive.socket || state.cameraLive.enabled)) {
    closeCameraLive("Live 화면 밖에서는 Camera WebSocket을 닫습니다.", { userStopped: false });
  }
  if (!routeSupportsImuLive(route) && (state.imuLive.socket || state.imuLive.enabled)) {
    closeImuLive("Live 화면 밖에서는 IMU WebSocket을 닫습니다.", { userStopped: false });
  }
  // The recording indicator of the status bar has a different source off the drive page.
  if (route !== "/rover-control") renderRecordingStatus();
  window.RoverGps?.routeChanged();
  requestChartAnimation();
  window.dispatchEvent(new CustomEvent("ubts:route", { detail: { route } }));
}

function initMobileNav() {
  const button = $("mobileMenuButton");
  const backdrop = $("mobileNavBackdrop");
  button?.addEventListener("click", () => setMobileNav(!document.body.classList.contains("nav-open")));
  backdrop?.addEventListener("click", () => setMobileNav(false));
  document.addEventListener("keydown", (event) => {
    if (event.key === "Escape") setMobileNav(false);
  });
  window.addEventListener("resize", () => {
    if (window.innerWidth > 1023) setMobileNav(false);
    if (routeSupportsLidarLive()) requestChartAnimation();
  });
}

function setMobileNav(open) {
  document.body.classList.toggle("nav-open", Boolean(open));
  const button = $("mobileMenuButton");
  if (button) {
    button.setAttribute("aria-expanded", String(Boolean(open)));
    button.setAttribute("aria-label", open ? "메뉴 닫기" : "메뉴 열기");
  }
}

function initTheme() {
  let stored = null;
  try {
    stored = localStorage.getItem("ubts-theme");
  } catch (_) {
    stored = null;
  }
  const theme = stored || "light";
  applyTheme(theme);
  $("themeToggle")?.addEventListener("click", () => {
    const next = document.documentElement.dataset.theme === "light" ? "dark" : "light";
    applyTheme(next);
  });
}

function applyTheme(theme) {
  document.documentElement.dataset.theme = theme;
  try {
    localStorage.setItem("ubts-theme", theme);
  } catch (_) {
    // Ignore storage failures; theme switching should remain usable.
  }
  const isLight = theme === "light";
  // The switch reads "다크 테마": pressed means dark.
  const toggle = $("themeToggle");
  if (toggle) toggle.setAttribute("aria-pressed", String(!isLight));
  requestChartAnimation();
}

function initCpuMode() {
  try {
    state.cpuMode = localStorage.getItem(CPU_MODE_KEY) || "combined";
  } catch (_) {
    state.cpuMode = "combined";
  }
  document.querySelectorAll("[data-cpu-mode]").forEach((button) => {
    button.addEventListener("click", () => setCpuMode(button.dataset.cpuMode));
  });
  setCpuMode(state.cpuMode);
}

function setCpuMode(mode) {
  state.cpuMode = mode === "separate" ? "separate" : "combined";
  try {
    localStorage.setItem(CPU_MODE_KEY, state.cpuMode);
  } catch (_) {
    // Ignore storage failures; the visible toggle still works for this session.
  }
  document.querySelectorAll("[data-cpu-mode]").forEach((button) => {
    button.classList.toggle("active", button.dataset.cpuMode === state.cpuMode);
  });
  const combined = $("cpuCombinedView");
  const separate = $("cpuSeparateView");
  if (combined) combined.hidden = state.cpuMode !== "combined";
  if (separate) separate.hidden = state.cpuMode !== "separate";
  text("cpuModeText", state.cpuMode === "separate" ? "코어별 개별 그래프" : "전체 코어 통합 그래프");
  requestChartAnimation();
}

// Slider fill: the style sheet draws the filled part of a slider track from
// the custom property --range-fill. Keep it equal to the value, whether the
// value comes from the pointer, the keyboard or a script.
function paintRangeFill(input) {
  const min = Number(input.min) || 0;
  const max = input.max === "" ? 100 : Number(input.max);
  const share = max > min ? (Number(input.value) - min) / (max - min) : 0;
  input.style.setProperty("--range-fill", `${Math.max(0, Math.min(1, share)) * 100}%`);
}

function initRangeFill() {
  const native = Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, "value");
  const limits = new MutationObserver((records) => records.forEach((record) => paintRangeFill(record.target)));
  document.querySelectorAll('input[type="range"]').forEach((input) => {
    // A script that assigns input.value fires no event: wrap the property on
    // this element. The native getter and setter still do the work.
    Object.defineProperty(input, "value", {
      configurable: true,
      enumerable: true,
      get() { return native.get.call(this); },
      set(next) { native.set.call(this, next); paintRangeFill(this); },
    });
    input.addEventListener("input", () => paintRangeFill(input));
    limits.observe(input, { attributes: true, attributeFilter: ["min", "max"] });
    paintRangeFill(input);
  });
}
