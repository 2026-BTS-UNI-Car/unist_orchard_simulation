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
// 2026-11-18 (battery swap in the field): while the board or the bridge restarted, the status had no vehicle_profile,
// the page kept the differential pad, and its Arm (no vehicle_profile) got 'steer_profile_required'. A front-steer
// bridge is now remembered in this browser (STEER_KIND_KEY) and taken at once on the next load, and every status or
// refusal that names the profile is used; the page never goes back to the differential pad by itself.
// 2026-11-19 (in-page tabs of Rover Control, static/app/10-rover-control.js): the steering panel and the autonomy
// section load with the page as before; the parts of 보정·설정 (steering calibration, the tape corrections of the distance
// move) load when that tab is shown the first time, the executor's local grid when 자율주행 is shown the first time, the
// distance move when its folded part is opened (or 보정·설정 is shown), the tuning values when their folded part is opened.
const steerVehicle = () => document.documentElement.dataset.vehicleKind === "front_steer";
const STEER_KIND_KEY = "ubts-vehicle-kind-seen";

function adoptBridgeVehicle(bridge) {
  const profile = bridge && typeof bridge === "object" ? bridge.vehicle_profile : bridge;
  if (profile !== "front_steer") return false;
  try { localStorage.setItem(STEER_KIND_KEY, "front_steer"); } catch (_) { /* private mode: this load only */ }
  if (steerVehicle()) return false;
  document.documentElement.dataset.vehicleKind = "front_steer";
  document.documentElement.dataset.vehicleKindFrom = "bridge";
  loadSteerScreens();
  return true;
}

function rememberedSteerVehicle() {
  try { return localStorage.getItem(STEER_KIND_KEY) === "front_steer"; } catch (_) { return false; }
}

const steerScreens = { drive: null, auto: null, tuning: null, grid: null, pot: null, cal: null, dist: null, driveScreen: null,
                       autoScreen: null };

function steerPageApi() {
  // controlRequest: the one transport for driving requests (NET-05): high priority, never queued, with a time limit.
  // driveStatus: the steering panel's latest /api/rover/status reply, so the potentiometer rows on the same page do
  // not ask for it again.
  // driveController: the steering panel's controller (the distance move hands its lease chain to the server).
  // roverTab / shown: the active in-page tab and whether an element is in a part of it (10-rover-control.js).
  // recording: the recorder state and the start / stop of 11-recording.js (the autonomy tab's run recording).
  // executorStatus: the autonomy panel's latest /api/executor/status reply and when it came (performance.now()), so
  // the moving picture of the monitoring view takes the pose from it and asks for nothing more.
  const tabs = typeof roverTab === "function";
  return { getJson, postJson, controlRequest, currentRoute, renderReadout, chartColors, driveStatus: steerDriveStatus,
           executorStatus: steerExecutorStatus,
           driveController: () => steerScreens.driveScreen?.controller || null,
           roverTab: () => (tabs ? roverTab() : null), shown: (id) => (tabs ? roverPartShown(id) : true),
           recording: typeof recordingStatusView === "function"
             ? { view: recordingStatusView, start: (options) => startRecording(options), stop: () => stopRecording() } : null };
}

function steerDriveStatus() {
  const ctl = steerScreens.driveScreen?.controller;
  return ctl && ctl.lastStatusAt !== null ? { reply: ctl.lastStatus, at: ctl.lastStatusAt } : null;
}

function steerExecutorStatus() {
  const screen = steerScreens.autoScreen;
  return screen && screen.stateAt() ? { reply: screen.state(), at: screen.stateAt() } : null;
}

function loadSteerScreen(name, url, init) {
  if (!steerScreens[name]) {
    steerScreens[name] = import(url)
      .then((module) => {
        const screen = module[init](steerPageApi());
        if (name === "drive") steerScreens.driveScreen = screen;
        if (name === "auto") steerScreens.autoScreen = screen;
        return screen;
      })
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
  // The steering calibration (2026-11-16, ESP32-S3 runtime calibration): fetched when 보정·설정 is shown the first time.
  if (steerVehicle() && (steerScreens.cal || (route === "/rover-control" && steerPartWanted.cal()))) {
    loadSteerScreen("cal", "/static/steer/steer-calibration.mjs", "initSteerCalibration");
  }
  // The distance move section (2026-11-17): fetched when it is opened (or its tape part in 보정·설정 shown) the first time.
  if (steerVehicle() && (steerScreens.dist || (route === "/rover-control" && steerPartWanted.dist()))) {
    loadSteerScreen("dist", "/static/steer/drive-distance.mjs", "initDriveDistance");
  }
  // The field tuning values (UI-12) are a part of Rover Control on every dashboard; without an executor they say so.
  if (steerScreens.tuning || (route === "/rover-control" && steerPartWanted.tuning())) loadSteerScreen("tuning", "/static/steer/tuning-page.mjs", "initTuningPage");
  // The executor's local grid in the 자율주행 tab's monitoring view (UI-09): drawn only when the executor sends one.
  if (steerScreens.grid || (route === "/rover-control" && steerPartWanted.grid())) loadSteerScreen("grid", "/static/steer/local-grid.mjs", "initLocalGrid");
}

// When the parts are wanted the first time: a folded part when it is opened, a tab part when its tab is shown.
const steerTabShown = (tab) => typeof roverTab === "function" && roverTab() === tab;
const steerPartWanted = {
  tuning: () => Boolean($("tuningDetails")?.open),
  cal: () => steerTabShown("setup"),
  dist: () => Boolean($("steerDistDetails")?.open) || steerTabShown("setup"),
  grid: () => steerTabShown("auto"),
};
$("tuningDetails")?.addEventListener("toggle", () => loadSteerScreens());
$("steerDistDetails")?.addEventListener("toggle", () => loadSteerScreens());

window.addEventListener("ubts:route", (event) => loadSteerScreens(event.detail?.route));
window.addEventListener("ubts:rover-tab", () => loadSteerScreens());
if (!steerVehicle() && rememberedSteerVehicle()) {
  document.documentElement.dataset.vehicleKind = "front_steer";
  document.documentElement.dataset.vehicleKindFrom = "remembered";
  loadSteerScreens();
}
