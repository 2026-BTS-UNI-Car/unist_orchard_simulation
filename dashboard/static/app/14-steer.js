// Front-steer and executor parts of Rover Control (UI-07, UI-08, UI-09 local grid, UI-12, UI-16, docs/SCREENS_KO.md,
// docs/STEERING_SERVER_KO.md). The server marks the page with data-vehicle-kind="front_steer" (DASHBOARD_VEHICLE, D-12);
// the style sheet then hides the differential pad and shows the steering panel, the potentiometer and the autonomy
// section. Their modules are fetched only on a front-steer dashboard and only when Rover Control opens, so the first
// load of the differential dashboard does not grow. The field tuning values and the executor's local grid are parts of
// Rover Control on every dashboard; their modules are fetched when they are wanted the first time: the tuning values
// when their collapsed part is opened, the local grid when the live map part comes on screen (UI-16: the first open of
// Rover Control stays as light as the drive page was). renderRoute() announces every route with "ubts:route".
// A dashboard started without DASHBOARD_VEHICLE (differential) takes the steering panel when the control bridge says
// vehicle_profile "front_steer" (the ESP32-S3 bridge, 2026-11-16): the differential pad cannot drive such a bridge (it
// refuses left/right), so showing it would only mislead. The menu and the page groups do not change.
const steerVehicle = () => document.documentElement.dataset.vehicleKind === "front_steer";

function adoptBridgeVehicle(bridge) {
  const profile = bridge && typeof bridge === "object" ? bridge.vehicle_profile : bridge;
  if (profile !== "front_steer" || steerVehicle()) return false;
  document.documentElement.dataset.vehicleKind = "front_steer";
  document.documentElement.dataset.vehicleKindFrom = "bridge";
  loadSteerScreens();
  return true;
}

const steerScreens = { drive: null, auto: null, tuning: null, grid: null, pot: null, driveScreen: null };

function steerPageApi() {
  // controlRequest: the one transport for driving requests (NET-05): high priority, never queued, with a time limit.
  // driveStatus: the steering panel's latest /api/rover/status reply, so the potentiometer rows on the same page do
  // not ask for it again.
  return { getJson, postJson, controlRequest, currentRoute, renderReadout, chartColors, driveStatus: steerDriveStatus };
}

function steerDriveStatus() {
  const ctl = steerScreens.driveScreen?.controller;
  return ctl && ctl.lastStatusAt !== null ? { reply: ctl.lastStatus, at: ctl.lastStatusAt } : null;
}

function loadSteerScreen(name, url, init) {
  if (!steerScreens[name]) {
    steerScreens[name] = import(url)
      .then((module) => { const screen = module[init](steerPageApi()); if (name === "drive") steerScreens.driveScreen = screen; return screen; })
      .catch((err) => { steerScreens[name] = null; console.warn(`front-steer ${name}`, err); });
  }
  steerScreens[name]?.then((screen) => screen?.routeChanged());
}

function loadSteerScreens(route = currentRoute()) {
  if (steerVehicle() && (route === "/rover-control" || steerScreens.drive)) {
    loadSteerScreen("drive", "/static/steer/steer-drive.mjs", "initSteerDrive");
    loadSteerScreen("auto", "/static/steer/auto-panel.mjs", "initAutoPanel");
    loadSteerScreen("pot", "/static/steer/steer-sensor.mjs", "initSteerSensor");
  }
  // The field tuning values (UI-12) are a part of Rover Control on every dashboard; without an executor they say so.
  if (steerScreens.tuning || (route === "/rover-control" && steerPartWanted.tuning())) loadSteerScreen("tuning", "/static/steer/tuning-page.mjs", "initTuningPage");
  // The executor's local grid on the live occupancy grid map (UI-09): shown only when the executor sends one.
  if (steerScreens.grid || (route === "/rover-control" && steerPartWanted.grid())) loadSteerScreen("grid", "/static/steer/local-grid.mjs", "initLocalGrid");
}

// When the two parts are wanted the first time. Without IntersectionObserver the local grid is wanted at once (as before).
const steerPartWanted = {
  tuning: () => Boolean($("tuningDetails")?.open),
  mapSeen: typeof IntersectionObserver !== "function" || !$("liveMapPanel"),
  grid: () => steerPartWanted.mapSeen,
};
$("tuningDetails")?.addEventListener("toggle", () => loadSteerScreens());
if (!steerPartWanted.mapSeen) {
  const mapWatch = new IntersectionObserver((entries) => {
    if (!entries.some((entry) => entry.isIntersecting)) return;
    steerPartWanted.mapSeen = true;
    mapWatch.disconnect();
    loadSteerScreens();
  });
  mapWatch.observe($("liveMapPanel"));
}

window.addEventListener("ubts:route", (event) => loadSteerScreens(event.detail?.route));
