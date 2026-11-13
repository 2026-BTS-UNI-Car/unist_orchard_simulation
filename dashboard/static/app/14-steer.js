// Front-steer screens (UI-07, UI-08, UI-12, docs/STEERING_SERVER_KO.md). The server marks the page with
// data-vehicle-kind="front_steer" (DASHBOARD_VEHICLE, D-12); the style sheet then hides the differential pad and shows
// the steering panel and the autonomy section. Their modules are fetched only on a front-steer dashboard and only when
// the drive route opens, so the first load of the differential dashboard does not grow. renderRoute() announces every
// route with "ubts:route".
const STEER_VEHICLE = document.documentElement.dataset.vehicleKind === "front_steer";
const steerScreens = { drive: null, auto: null, tuning: null };

function steerPageApi() {
  return { getJson, postJson, currentRoute, renderReadout, chartColors };
}

function loadSteerScreen(name, url, init) {
  if (!steerScreens[name]) {
    steerScreens[name] = import(url)
      .then((module) => module[init](steerPageApi()))
      .catch((err) => { steerScreens[name] = null; console.warn(`front-steer ${name}`, err); });
  }
  steerScreens[name]?.then((screen) => screen?.routeChanged());
}

function loadSteerScreens(route = currentRoute()) {
  if (STEER_VEHICLE && (route === "/rover-control" || steerScreens.drive)) {
    loadSteerScreen("drive", "/static/steer/steer-drive.mjs", "initSteerDrive");
    loadSteerScreen("auto", "/static/steer/auto-panel.mjs", "initAutoPanel");
  }
  // The tuning screen (UI-12) is a route of every dashboard; without an executor it says so.
  if (route === "/tuning" || steerScreens.tuning) loadSteerScreen("tuning", "/static/steer/tuning-page.mjs", "initTuningPage");
}

window.addEventListener("ubts:route", (event) => loadSteerScreens(event.detail?.route));
