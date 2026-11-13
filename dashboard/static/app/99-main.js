
window.addEventListener("popstate", renderRoute);

window.addEventListener("pagehide", () => {
  if (state.roverControl.lease || hasActiveRoverInput() || state.roverControl.stoppingWithFade || Math.abs(state.roverControl.currentLinear) > 0.001 || Math.abs(state.roverControl.currentAngular) > 0.001) {
    try {
      const body = JSON.stringify({ source: "pagehide" });
      if (navigator.sendBeacon) {
        navigator.sendBeacon("/api/rover/stop", new Blob([body], { type: "application/json" }));
      } else {
        fetch("/api/rover/stop", { method: "POST", keepalive: true, headers: { "Content-Type": "application/json" }, body });
      }
    } catch (_) {
      // The bridge command TTL still stops stale UI commands.
    }
  }
  if (state.lidarLive.socket || state.lidarLive.enabled) {
    closeLidarLive("페이지 종료로 LiDAR WebSocket을 닫습니다.", { userStopped: false });
  }
  if (state.cameraLive.socket || state.cameraLive.enabled) {
    closeCameraLive("페이지 종료로 Camera WebSocket을 닫습니다.", { userStopped: false });
  }
  clearArchiveFrameCache();
});
document.addEventListener("click", (event) => {
  const docLink = event.target.closest("a[data-doc-link]");
  if (docLink) {
    event.preventDefault();
    history.pushState({}, "", docLink.getAttribute("href"));
    setMobileNav(false);
    renderRoute();
    return;
  }
  const link = event.target.closest("a[data-route]");
  if (!link) return;
  event.preventDefault();
  history.pushState({}, "", link.getAttribute("href"));
  setMobileNav(false);
  renderRoute();
});

initMobileNav();
initTheme();
initCpuMode();
initLidarTest();
initCameraLive();
initFanControls();
initDockerControls();
initNetworkControls();
initRoverControl();
initCanvasResize();
initRangeFill();
renderRoute();
loadPlan().catch(console.error);
loadDocs().catch(console.error);
tick();
setInterval(tick, POLL_INTERVAL_MS);
renderFrameClock();
setInterval(renderFrameClock, 1000);
setInterval(() => {
  // Recorder polling and UI expiry are independent of general telemetry.
  maybeRefreshRecordingStatus();
  if (routeSupportsCameraLive() && (state.cameraLive.enabled || state.cameraLive.lastDecodedAt !== null)) renderCameraLiveState();
}, 1000);

// Reuse the same widgets, playback clock and network streams in the overview.
const overviewToggle = document.getElementById('roverArchiveOverviewToggle');
if (overviewToggle) {
  try { overviewToggle.checked = localStorage.getItem('ubts-archive-overview-v1') === '1'; } catch (_) { /* storage blocked */ }
  const applyOverview = () => {
    document.querySelector('.archive-preview')?.classList.toggle('sensor-overview', overviewToggle.checked);
    try { localStorage.setItem('ubts-archive-overview-v1', overviewToggle.checked ? '1' : '0'); } catch (_) { /* storage blocked */ }
    // No synthetic window "resize": the canvases watch their own boxes.
    window.LiveMap?.routeChanged();
    window.RoverGps?.routeChanged();
  };
  overviewToggle.addEventListener('change', applyOverview); applyOverview();
}
