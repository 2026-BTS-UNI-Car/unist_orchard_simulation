// Live map polling policy (pure functions; unit-tested with node).
//
// - The server sends only the path points after the client's last known
//   point (path_from = its index) when the client names that point by
//   path_token/path_count/path_stamp. Anything unexpected falls back to a
//   full path on the next poll; a partial path is never displayed.
// - Live polls use a short timeout. A slow link adapts the poll interval to
//   the measured round trip instead of re-polling every 100 ms.

export const LIVE_MAP_TIMEOUT_MS = 3000;
export const IDLE_POLL_MS = 1000;
export const AUTONOMY_POLL_MS = 1000;
const PROFILE_FLOOR_MS = {detail: 100, balanced: 100, slow: 500, tiny: 2000};

export function profileFloor(profile) {
  return PROFILE_FLOOR_MS[profile] ?? PROFILE_FLOOR_MS.balanced;
}

// Delay until the next poll, measured from the end of the current one.
// Start-to-start interval = max(profile floor, 1.5 x last round trip).
export function livePollDelay({profile = 'balanced', drawing = false, rttMs = 0, elapsedMs = rttMs,
  autonomyActive = false} = {}) {
  let floor = profileFloor(profile);
  if (!drawing) floor = Math.max(floor, IDLE_POLL_MS);
  if (autonomyActive) floor = Math.max(floor, AUTONOMY_POLL_MS);
  const rtt = Number.isFinite(rttMs) && rttMs > 0 ? rttMs : 0;
  const elapsed = Number.isFinite(elapsedMs) && elapsedMs > 0 ? elapsedMs : 0;
  return Math.max(0, Math.round(Math.max(floor, 1.5 * rtt) - elapsed));
}

// Query parameters naming the newest path point the client already holds.
export function livePathQuery(snapshot, needFull = false) {
  const path = Array.isArray(snapshot?.path) ? snapshot.path : [];
  const last = path.at(-1);
  if (needFull || !snapshot?.path_token || !path.length || !Number.isFinite(last?.stamp)) return {};
  return {path_token: String(snapshot.path_token), path_count: String(path.length), path_stamp: String(last.stamp)};
}

// Combine a response with the previous snapshot's path.
// Returns {path, ok}; ok=false means "keep the previous path and ask for the
// full path on the next poll".
export function mergeLivePath(previous, snapshot) {
  const before = Array.isArray(previous?.path) ? previous.path : [];
  const incoming = Array.isArray(snapshot?.path) ? snapshot.path : [];
  if (Number.isInteger(snapshot?.path_from) && snapshot.path_from >= 0) {
    if (snapshot.tiles_reset || snapshot.path_token !== previous?.path_token || before.length < snapshot.path_from) {
      return {path: before, ok: false};
    }
    const path = before.slice(0, snapshot.path_from).concat(incoming);
    if (Number.isInteger(snapshot.path_count) && path.length !== snapshot.path_count) return {path: before, ok: false};
    return {path, ok: true};
  }
  // Earlier contract: path_append concatenates without overlap.
  if (snapshot?.path_append && !snapshot.tiles_reset) return {path: [...before, ...incoming], ok: true};
  return {path: snapshot?.path ?? incoming, ok: true};
}

// Accept a live poll response only when no map action (new map, stop,
// start) started after the poll departed, and the response is not an older
// version of the same map epoch than the one displayed.
export function acceptLiveResponse({requestActionSeq, currentActionSeq, actionBusy = false, value, shown}) {
  if (actionBusy || requestActionSeq !== currentActionSeq) return false;
  if (!value || typeof value !== 'object') return false;
  if (shown?.map_epoch && value.map_epoch === shown.map_epoch &&
      Number.isFinite(Number(value.version)) && Number.isFinite(Number(shown.version)) &&
      Number(value.version) < Number(shown.version)) return false;
  return true;
}
