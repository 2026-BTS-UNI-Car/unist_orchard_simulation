// Simulation dashboard only (DASHBOARD_EXTRA_STATIC, added to the page by ubts_sim_dashboard.extension.page_filter).
// Writes the Gazebo scene and the real-time factor into the dashboard's simulation banner. Reads GET /api/sim/status
// every 5 s while the tab is visible; text only, no control. Placeholder-based SIM (자리값 기반 SIM 결과).
(function () {
  "use strict";
  var PERIOD_MS = 5000;
  var banner = document.querySelector(".simulation-banner");
  if (!banner) return;
  var base = banner.textContent;
  var timer = null;

  function describe(s) {
    if (!s || s.ok !== true) return " · SIM 상태 없음";
    if (!s.fresh) return " · SIM 상태 오래됨";
    var parts = [];
    if (typeof s.scene === "string") parts.push("장면 " + s.scene);
    if (typeof s.rtf_5s === "number" && isFinite(s.rtf_5s)) parts.push("실시간 대비 " + s.rtf_5s.toFixed(2) + "×");
    var alive = s.sensors_alive || {};
    var down = Object.keys(alive).filter(function (k) { return alive[k] !== true; });
    if (down.length) parts.push("멈춘 센서 " + down.join(", "));
    return parts.length ? " · " + parts.join(" · ") : "";
  }

  function poll() {
    if (document.hidden) return;
    fetch("/api/sim/status", { credentials: "same-origin", cache: "no-store" })
      .then(function (r) { return r.ok ? r.json() : null; })
      .then(function (s) { banner.textContent = base + describe(s); })
      .catch(function () { banner.textContent = base + " · SIM 상태 없음"; });
  }

  function start() { if (timer === null) { poll(); timer = setInterval(poll, PERIOD_MS); } }
  function stop() { if (timer !== null) { clearInterval(timer); timer = null; } }
  document.addEventListener("visibilitychange", function () { if (document.hidden) stop(); else start(); });
  start();
})();
