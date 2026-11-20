
async function loadRecordings() {
  state.recordings.loading = true;
  renderRecordings();
  try {
    await refreshRecordingStatus();
    const selectedId = state.recordings.selectedId;
    const before = state.recordings.items.find(item => item.id === selectedId);
    const payload = await getJson("/api/recordings");
    state.recordings.items = payload.recordings || [];
    // Fetch archive frames and maps only after an explicit selection.
    renderRecordings();
    const after = state.recordings.items.find(item => item.id === selectedId);
    // Refreshing a list must not reset the playback clock, tiles or sensor
    // cache, even if post-processing changed the selected record's metadata.
    if (selectedId && selectedId === state.recordings.selectedId &&
        (!state.recordings.selected || recordingListSignature(before) !== recordingListSignature(after))) {
      await selectRecording(selectedId, { preservePlayback: true });
    }
  } finally {
    state.recordings.loading = false;
    renderRecordings();
  }
}

function recordingListSignature(item) {
  if (!item) return "";
  const stable = (value) => value && typeof value === "object" && !Array.isArray(value)
    ? Object.keys(value).sort().reduce((out, key) => { out[key] = stable(value[key]); return out; }, {})
    : Array.isArray(value) ? value.map(stable) : value;
  return JSON.stringify(stable(item));
}

async function selectRecording(id, { preservePlayback = false } = {}) {
  const preserve = preservePlayback && id === state.recordings.selectedId && Boolean(state.recordings.timeline);
  const revision = (state.recordings.detailRevision || 0) + 1;
  state.recordings.detailRevision = revision;
  state.recordings.selectedId = id;
  if (!preserve) {
    stopArchivePlayback(true);
    state.recordings.timeline = null;
    clearArchiveFrameCache();
    renderArchiveMotor(null, [], 0);
  }

  renderRecordings();
  try {
    const detail = await getJson(`/api/recordings/${encodeURIComponent(id)}`);
    if (id !== state.recordings.selectedId || revision !== state.recordings.detailRevision) return;
    state.recordings.selected = detail;
    state.recordings.timeline = timelineShellFromRecording(detail?.recording || detail) ||
      (preserve ? state.recordings.timeline : null);
    renderRecordingDetail(detail, { preservePlayback: preserve });
    if (state.recordings.timeline && !preserve) {
      await scheduleArchiveSegmentsForTime(0, { force: true, reschedule: true }).catch((err) => debugLog("archive", "first segment load failed", err));
      renderArchiveFrame();

    }
  } catch (err) {
    if (id !== state.recordings.selectedId || revision !== state.recordings.detailRevision) return;
    renderStatusRows("roverArchiveDetails", [
      { title: "Archive", sub: id, ok: false, level: "bad", value: err.message },
    ]);
  }
}


function timelineShellFromRecording(payload) {
  const summary = payload?.timeline_summary || {};
  const metadata = payload?.metadata || {};
  const durationS = Number(summary.duration_s || metadata.duration_s || payload?.duration_s || 0);
  const lidarFrames = Number(summary.lidar_frames || 0);
  const cameraFrames = Number(summary.camera_frames || 0);
  const imuFrames = Number(summary.imu_frames || 0);
  const gpsFrames = Number(summary.gps_frames || 0);
  const motorFrames = Number(summary.motor_frames || 0);
  if (!summary.ok && !lidarFrames && !cameraFrames && !imuFrames && !gpsFrames && !motorFrames) return null;
  return {
    ok: true,
    id: payload?.id || metadata.id || state.recordings.selectedId,
    kind: payload?.kind || metadata.kind,
    algorithm: summary.algorithm,
    coordinate_frame: summary.coordinate_frame || payload?.preview_scan?.coordinate_frame || "",
    duration_s: durationS,
    lidar: { frames: [] },
    camera: { frames: [] },
    imu: { frames: [], calibration: metadata.timeline?.imu_calibration || {} },
    gps: {frames: [], enabled: metadata.include_gps === true || gpsFrames > 0},
    motor: {frames: [], enabled: motorFrames > 0, frame_count: motorFrames},
    sync: {
      time_base: "seconds_since_recording_start",
      lidar_frames: lidarFrames,
      camera_frames: cameraFrames,
      imu_frames: imuFrames,
      gps_frames: gpsFrames,
      motor_frames: motorFrames,
    },
    streaming: true,
  };
}

function clearArchiveFrameCache() {
  const cache = state.archivePlayback.cache;
  if (state.archivePlayback.seekTimer) {
    clearTimeout(state.archivePlayback.seekTimer);
    state.archivePlayback.seekTimer = null;
  }
  if (cache?.objectUrls) {
    for (const url of cache.objectUrls.values()) {
      if (url && url.startsWith("blob:")) URL.revokeObjectURL(url);
    }
  }
  state.archivePlayback.cache = {
    lidarFrames: [],
    cameraFrames: [],
    imuFrames: [],
    gpsFrames: [],
    motorFrames: [],
    motorKeys: new Set(),
    executorFrames: [],
    executorKeys: new Set(),
    lidarKeys: new Set(),
    cameraKeys: new Set(),
    imuKeys: new Set(),
    gpsKeys: new Set(),
    objectUrls: new Map(),
    imageElements: new Map(),
  };
  state.archivePlayback.lastExecutorKey = "";
  if (typeof archiveGridReset === "function") archiveGridReset();
  for(const task of state.archivePlayback.loadingTelemetry?.values()||[])task.controller.abort();
  state.archivePlayback.telemetrySegments = new Map();
  state.archivePlayback.loadingTelemetry = new Map();
  state.archivePlayback.segmentCache = new Map();
  for(const task of state.archivePlayback.loadingSegments?.values()||[])task.controller?.abort();
  state.archivePlayback.loadingSegments = new Map();
  state.archivePlayback.lastSegmentWindowKey = "";
  state.archivePlayback.lastPlaybackRenderAt = 0;
  state.archivePlayback.lastLidarRenderKey = "";
  state.archivePlayback.lastCameraRenderKey = "";
}

function prepareArchiveTimelineCache(timeline) {
  clearArchiveFrameCache();
  mergeArchiveTimelineSegment(timeline);
}

function expandArchiveLidarPoints(points, fields) {
  if (!Array.isArray(points) || !points.length || !Array.isArray(points[0])) return points || [];
  const names = Array.isArray(fields) && fields.length
    ? fields
    : ["x_m", "y_m", "distance_m", "quality", "angle_deg"];
  return points.map((values) => {
    const point = Object.fromEntries(names.map((name, index) => [name, values[index]]));
    if (!names.includes("x_m") && Number.isFinite(point.angle_deg) && Number.isFinite(point.distance_m)) {
      const angle = point.angle_deg * Math.PI / 180;
      point.x_m = Math.sin(angle) * point.distance_m;
      point.y_m = Math.cos(angle) * point.distance_m;
    }
    return point;
  });
}

function mergeArchiveTimelineSegment(timeline) {
  if (!timeline) return;
  const cache = state.archivePlayback.cache;
  const lidarPointFields = timeline.playback?.lidar_point_fields || [];
  if (state.recordings.timeline && timeline.imu) {
    state.recordings.timeline.imu = {
      ...(state.recordings.timeline.imu || {}),
      ...Object.fromEntries(Object.entries(timeline.imu).filter(([key]) => key !== "frames")),
      frames: [],
    };
  }
  const lidarFrames = (timeline.lidar?.frames || []).map((frame, index) => {
    const points = expandArchiveLidarPoints(frame.points, lidarPointFields);
    return {
      ...frame,
      index,
      t_s: Number(frame.t_s) || 0,
      points,
      result: normalizeArchiveLidarResult({
        ok: true,
        state: "archive_timeline",
        frame_id: frame.frame_id || "laser",
        stamp: frame.stamp,
        points,
        coordinate_frame: frame.coordinate_frame || timeline.coordinate_frame,
        metrics: frame.metrics || {},
        source: "archive_timeline",
      }, frame.overlay, frame),
    };
  });
  for (const frame of lidarFrames) {
    const key = archiveFrameKey(frame, "lidar");
    if (cache.lidarKeys.has(key)) continue;
    cache.lidarKeys.add(key);
    frame.cacheKey = key;
    cache.lidarFrames.push(frame);
  }
  cache.lidarFrames.sort((a, b) => a.t_s - b.t_s);
  cache.lidarFrames.forEach((frame, cacheIndex) => {
    frame.cacheIndex = cacheIndex;
  });
  const cameraFrames = (timeline.camera?.frames || []).map((frame, index) => ({
    ...frame,
    index,
    t_s: Number(frame.t_s) || 0,
  }));
  for (const frame of cameraFrames) {
    const key = archiveFrameKey(frame, "camera");
    if (cache.cameraKeys.has(key)) {
      const existing = cache.cameraFrames.find((item) => item.cacheKey === key);
      if (existing) Object.assign(existing, frame, { cacheKey: key });
      continue;
    }
    cache.cameraKeys.add(key);
    frame.cacheKey = key;
    cache.cameraFrames.push(frame);
  }
  cache.cameraFrames.sort((a, b) => a.t_s - b.t_s);
  cache.cameraFrames.forEach((frame, cacheIndex) => {
    frame.cacheIndex = cacheIndex;
  });
  const imuFrames = (timeline.imu?.frames || []).map((frame, index) => ({
    ...frame,
    index,
    t_s: Number(frame.t_s) || 0,
  }));
  for (const frame of imuFrames) {
    const key = archiveFrameKey(frame, "imu");
    if (cache.imuKeys.has(key)) continue;
    cache.imuKeys.add(key);
    frame.cacheKey = key;
    cache.imuFrames.push(frame);
  }
  cache.imuFrames.sort((a, b) => a.t_s - b.t_s);
  cache.imuFrames.forEach((frame, cacheIndex) => { frame.cacheIndex = cacheIndex; });
  cache.gpsFrames ||= []; cache.gpsKeys ||= new Set();
  for (const frame of timeline.gps?.frames || []) {
    const key = `${Number(frame.t_s).toFixed(6)}:${frame.sample?.sequence ?? frame.sample?.state}`;
    if (!cache.gpsKeys.has(key)) { cache.gpsKeys.add(key); cache.gpsFrames.push({...frame, t_s:Number(frame.t_s)}); }
  }
  cache.gpsFrames.sort((a, b) => a.t_s-b.t_s);
  mergeArchiveMotorSegment(timeline.motor);
  if (typeof mergeArchiveExecutorSegment === "function") mergeArchiveExecutorSegment(timeline.executor);
  trimArchiveTimelineCache();
  state.archivePlayback.lastLidarRenderKey = "";
  state.archivePlayback.lastCameraRenderKey = "";
}

function trimArchiveTimelineCache() {
  const cache = state.archivePlayback.cache;
  const duration = Math.max(1, Number(state.archivePlayback.segmentDurationS) || ARCHIVE_SEGMENT_SECONDS);
  const center = Math.floor(Math.max(0, state.archivePlayback.timeS || 0) / duration) * duration;
  const first = Math.max(0, center - 2 * duration), last = center + 4 * duration;
  for (const kind of ["lidar", "camera", "imu"]) {
    const name = `${kind}Frames`, keys = `${kind}Keys`;
    const lower=kind==='imu'?Math.max(0,center-24):first,upper=kind==='imu'?center+48:last;
    cache[name] = (cache[name] || []).filter(frame => frame.t_s >= lower && frame.t_s < upper);
    cache[keys] = new Set(cache[name].map(frame => frame.cacheKey));
    cache[name].forEach((frame, index) => { frame.cacheIndex = index; });
  }
  for (const [key, value] of state.archivePlayback.segmentCache) {
    if (value.start_s + value.duration_s <= first || value.start_s >= last) state.archivePlayback.segmentCache.delete(key);
  }
  for (const [key, url] of cache.objectUrls) {
    if (!cache.cameraKeys.has(key)) {
      if (url?.startsWith("blob:")) URL.revokeObjectURL(url);
      cache.objectUrls.delete(key);
    }
  }
  for (const key of cache.imageElements.keys()) {
    if (!cache.cameraKeys.has(key)) cache.imageElements.delete(key);
  }
}

function archiveFrameKey(frame, kind) {
  const stamp = frame?.stamp ? `${frame.stamp.sec || 0}.${frame.stamp.nanosec || 0}` : "";
  const size = kind === "camera" ? String(frame?.frame_url || frame?.data || "").length : (frame?.points || []).length;
  return `${kind}:${Number(frame?.t_s || 0).toFixed(3)}:${stamp}:${size}`;
}

async function ensureArchiveSegmentForTime(timeS, options = {}) {
  const id = state.recordings.selectedId;
  const timeline = state.recordings.timeline;
  if (!id || !timeline) return;
  ensureArchiveTelemetryForTime(timeS).catch(err=>debugLog('archive','telemetry load failed',err));
  const segmentDuration = Math.max(1, Number(state.archivePlayback.segmentDurationS) || ARCHIVE_SEGMENT_SECONDS);
  const startS = Math.max(0, Math.floor((Number(timeS) || 0) / segmentDuration) * segmentDuration);
  await loadArchiveTimelineSegment(id, startS, segmentDuration, options);
  const nextStart = startS + segmentDuration;
  const cameraBootstrapNeeded = !archiveNativeVideo() && (state.archivePlayback.cache.cameraFrames?.length || 0) === 0
    && Number(timeline.sync?.camera_frames || 0) > 0;
  if (cameraBootstrapNeeded && nextStart < state.archivePlayback.durationS + 0.25) {
    await loadArchiveTimelineSegment(id, nextStart, segmentDuration);
  }
  const nearSegmentEnd = Number(timeS) + 2 >= nextStart;
  if ((state.archivePlayback.playing || nearSegmentEnd) && nextStart < state.archivePlayback.durationS + 0.25) {
    loadArchiveTimelineSegment(id, nextStart, segmentDuration).catch((err) => debugLog("archive", "segment prefetch failed", err));
  }
  // One LiDAR look-ahead leaves bandwidth for the priority IMU/map/video paths.
}

async function ensureArchiveTelemetryForTime(timeS) {
  const id=state.recordings.selectedId;if(!id||!state.recordings.timeline)return;
  const start=Math.max(0,Math.floor(Number(timeS)/24)*24);
  const current=loadArchiveTelemetrySegment(id,start);
  if(Number(timeS)>=start+16 && start+24<state.archivePlayback.durationS)
    current.then(()=>loadArchiveTelemetrySegment(id,start+24)).catch(err=>debugLog('archive','telemetry prefetch failed',err));
  return current;
}

async function loadArchiveTelemetrySegment(id,start) {
  const key=`${id}:${start}`;
  const playback=state.archivePlayback;
  if(playback.telemetrySegments.has(key))return;
  if(playback.loadingTelemetry.has(key))return playback.loadingTelemetry.get(key).promise;
  for(const [other,task] of playback.loadingTelemetry){if(other!==key&&Math.abs(task.start-start)>24){task.controller.abort();playback.loadingTelemetry.delete(other);}}
  const controller=new AbortController();
  const promise=getJson(`/api/recordings/${encodeURIComponent(id)}/timeline?start_s=${start}&duration_s=24&profile=dashboard-telemetry-v1`,{signal:controller.signal})
    .then(payload=>{
      if(controller.signal.aborted||id!==state.recordings.selectedId)return;
      if(!payload.ok||!payload.timeline)throw new Error(payload.error||'telemetry unavailable');
      mergeArchiveTimelineSegment(payload.timeline);playback.telemetrySegments.set(key,start);
      for(const [old,t] of playback.telemetrySegments)if(Math.abs(t-start)>24)playback.telemetrySegments.delete(old);
      renderArchiveFrame();
    }).finally(()=>{if(playback.loadingTelemetry.get(key)?.controller===controller)playback.loadingTelemetry.delete(key);});
  playback.loadingTelemetry.set(key,{controller,promise,start});return promise;
}

function archiveSegmentWindowKey(timeS) {
  const id = state.recordings.selectedId;
  if (!id || !state.recordings.timeline) return "";
  const duration = Math.max(1, Number(state.archivePlayback.segmentDurationS) || ARCHIVE_SEGMENT_SECONDS);
  const start = Math.max(0, Math.floor((Number(timeS) || 0) / duration) * duration);
  return `${id}:${start.toFixed(3)}:${duration.toFixed(3)}`;
}

function scheduleArchiveSegmentsForTime(timeS, options = {}) {
  const key = archiveSegmentWindowKey(timeS);
  if (!key) return Promise.resolve();
  if (!options.reschedule && state.archivePlayback.lastSegmentWindowKey === key) return Promise.resolve();
  state.archivePlayback.lastSegmentWindowKey = key;
  return ensureArchiveSegmentForTime(timeS, { force: Boolean(options.force) })
    .catch((error) => {
      if (state.archivePlayback.lastSegmentWindowKey === key) state.archivePlayback.lastSegmentWindowKey = "";
      throw error;
    });
}

function scheduleArchiveSeek(timeS) {
  const duration=state.archivePlayback.segmentDurationS||ARCHIVE_SEGMENT_SECONDS;
  const start=Math.floor(Number(timeS)/duration)*duration;
  for(const task of state.archivePlayback.loadingSegments.values())if(task.start<start-duration||task.start>start+duration)task.controller.abort();
  ensureArchiveTelemetryForTime(timeS).catch(err=>debugLog('archive','seek telemetry load failed',err));
  if (state.archivePlayback.seekTimer) clearTimeout(state.archivePlayback.seekTimer);
  state.archivePlayback.seekTimer = setTimeout(() => {
    state.archivePlayback.seekTimer = null;
    scheduleArchiveSegmentsForTime(timeS, { reschedule: true })
      .then(() => renderArchiveFrame())
      .catch((err) => debugLog("archive", "seek segment load failed", err));
  }, 90);
}

async function loadArchiveTimelineSegment(id, startS, durationS, options = {}) {
  const start = Math.max(0, Number(startS) || 0);
  const duration = Math.max(1, Number(durationS) || ARCHIVE_SEGMENT_SECONDS);
  const key = `${id}:${start.toFixed(3)}:${duration.toFixed(3)}`;
  if (!options.force && state.archivePlayback.segmentCache.has(key)) return state.archivePlayback.segmentCache.get(key);
  if (state.archivePlayback.loadingSegments.has(key)) return state.archivePlayback.loadingSegments.get(key).promise;
  const params = new URLSearchParams({
    start_s: start.toFixed(3),
    duration_s: duration.toFixed(3),
    profile: ARCHIVE_PLAYBACK_PROFILE,
  });
  const controller=new AbortController();
  const promise = getJson(`/api/recordings/${encodeURIComponent(id)}/timeline?${params.toString()}`,{signal:controller.signal})
    .then((payload) => {
      if (controller.signal.aborted || id !== state.recordings.selectedId) return payload;
      if (payload?.ok && payload.timeline) {
        if (!state.recordings.timeline) state.recordings.timeline = payload.timeline;
        if (payload.timeline.coordinate_frame && !state.recordings.timeline.coordinate_frame) {
          state.recordings.timeline.coordinate_frame = payload.timeline.coordinate_frame;
        }
        mergeArchiveTimelineSegment(payload.timeline);
        if (typeof archiveGridIngest === "function") archiveGridIngest(payload.timeline, start, duration);
        state.archivePlayback.segmentCache.set(key, {
          start_s: start,
          duration_s: duration,
          lidar_frames: payload.timeline.lidar?.frames?.length || 0,
          camera_frames: payload.timeline.camera?.frames?.length || 0,
        });
        trimArchiveTimelineCache();
        if (currentRoute() === "/rover-control") renderArchiveFrame();
      }
      return payload;
    })
    .finally(() => {
      if(state.archivePlayback.loadingSegments.get(key)?.controller===controller)state.archivePlayback.loadingSegments.delete(key);
    });
  state.archivePlayback.loadingSegments.set(key, {promise,controller,start});
  return promise;
}

function archiveCameraFrameSrc(frame) {
  if (frame?.frame_url) {
    preloadArchiveCameraFrame(frame);
    return String(frame.frame_url);
  }
  if (!frame?.data) return "";
  const cache = state.archivePlayback.cache;
  const key = frame.cacheKey || `${frame.index}:${String(frame.data).length}`;
  if (cache.objectUrls.has(key)) return cache.objectUrls.get(key);
  const mime = frame.mime || "image/jpeg";
  try {
    const binary = atob(frame.data);
    const bytes = new Uint8Array(binary.length);
    for (let i = 0; i < binary.length; i += 1) bytes[i] = binary.charCodeAt(i);
    const url = URL.createObjectURL(new Blob([bytes], { type: mime }));
    cache.objectUrls.set(key, url);
    return url;
  } catch (_) {
    const dataUrl = `data:${mime};base64,${frame.data}`;
    cache.objectUrls.set(key, dataUrl);
    return dataUrl;
  }
}

function preloadArchiveCameraFrame(frame, priority = "low") {
  if (!frame) return null;
  const cache = state.archivePlayback.cache;
  const key = frame.cacheKey || `${frame.index}:${frame.frame_url || String(frame.data || "").length}`;
  const existing = cache.imageElements.get(key);
  if (existing) {
    if (priority === "high") existing.fetchPriority = "high";
    return existing;
  }
  const src = frame.frame_url ? String(frame.frame_url) : archiveCameraFrameSrc(frame);
  if (!src) return null;
  const image = new Image();
  image.decoding = "async";
  image.fetchPriority = priority;
  image.src = src;
  cache.imageElements.set(key, image);
  return image;
}

function warmArchiveCameraCache(frame) {
  if (!frame) return;
  const frames = state.archivePlayback.cache.cameraFrames;
  const center = Number(frame.cacheIndex);
  if (!Number.isFinite(center)) return;
  const start = Math.max(0, center - 3);
  const end = Math.min(frames.length - 1, center + 8);
  for (let index = start; index <= end; index += 1) {
    preloadArchiveCameraFrame(frames[index], index <= center + 2 ? "high" : "low");
  }
}

function renderArchiveDownloadButton(payload, id) {
  const button = $("roverArchiveDownloadButton");
  const meta = $("roverArchiveDownloadMeta");
  if (!button) return;
  if (!id) {
    button.hidden = true;
    button.removeAttribute("href");
    button.removeAttribute("download");
    if (meta) meta.textContent = "tar.gz";
    return;
  }
  const href = payload?.download_url || `/api/recordings/${encodeURIComponent(id)}/download`;
  button.hidden = false;
  button.href = href;
  button.download = `${id}.tar.gz`;
  if (meta) meta.textContent = `${recordingSensorLabel(payload)} · tar.gz`;
}

function recordingClockLabel(payload) {
  if (payload?.clock_quality === "gnss_estimated") return "GPS 시각 보정";
  if (payload?.clock_quality === "gnss_receiver_estimated") return "GPS UTC 기준";
  return "장치 시각 미검증";
}

function recordingClockDescription(payload) {
  if (payload?.clock_quality === "gnss_estimated") return "기존 기록의 표시 시각을 보정했습니다. 센서 시각과 재생 시간은 원본을 유지합니다.";
  if (payload?.clock_quality === "gnss_receiver_estimated") return "녹화 시작 시 GPS UTC를 확인했습니다. 수신 지연은 보정하지 않았습니다.";
  return "검증된 GPS 시간 정보가 없어 장치 시각으로 표시합니다.";
}

function renderRecordings() {
  const target = $("roverArchiveList");
  if (!target) return;
  target.innerHTML = "";
  if (state.recordings.loading && !state.recordings.items.length) {
    target.innerHTML = "<span>불러오는 중</span>";
    return;
  }
  if (!state.recordings.items.length) {
    target.innerHTML = "<span>기록 없음</span>";
    renderRecordingDetail(null);
    return;
  }
  // The recordings of the tab (2026-11-19): 기록 shows the manual ones, 자율주행 the autonomy ones (tag; a recording
  // without a tag is a manual one). The player keeps a selection made in the other tab.
  const scope = archiveScopeTag();
  text("roverArchiveScope", scope === "autonomy" ? "자율주행 기록" : "수동 주행 기록");
  const items = state.recordings.items.filter((item) => recordingTag(item) === scope);
  if (!items.length) {
    target.innerHTML = `<span>${scope === "autonomy" ? "자율주행 기록 없음" : "수동 주행 기록 없음"}</span>`;
    return;
  }
  for (const item of items) {
    const button = document.createElement("button");
    button.type = "button";
    button.classList.toggle("active", item.id === state.recordings.selectedId);
    // An autonomy recording is one driven stretch: which one of its run, a start again or not, and how it went.
    const stretch = recordingStretchText(item);
    button.innerHTML = `
      <span>${escapeHtml(stretch ? stretch[0] : recordingSensorLabel(item))}</span>
      <strong>${escapeHtml(item.id)}</strong>
      ${stretch && stretch[1] ? `<small>${escapeHtml(stretch[1])}</small>` : ""}
      <small>${escapeHtml(formatShortDate(item.display_started_at || item.started_at || item.created_at))} · ${escapeHtml(recordingClockLabel(item))} · ${escapeHtml(item.duration_s ? `${fmtNumber(item.duration_s, 1)}s` : item.state || "ready")} · L${escapeHtml(item.timeline?.lidar_frames ?? 0)} C${escapeHtml(item.timeline?.camera_frames ?? 0)} I${escapeHtml(item.timeline?.imu_frames ?? 0)} G${escapeHtml(item.timeline?.gps_frames ?? 0)}</small>
    `;
    button.addEventListener("click", () => selectRecording(item.id));
    target.append(button);
  }
}

function recordingTag(item) {
  const tag = item?.tag ?? item?.metadata?.tag;
  return tag === "autonomy" ? "autonomy" : "manual";
}

// A time in words (35초, 1분 45초, 1시간 2분), as static/steer/auto-model.mjs durationText.
function fmtDurationWords(seconds) {
  const total = Math.max(0, Math.round(Number(seconds) || 0));
  if (total < 60) return `${total}초`;
  const minutes = Math.floor(total / 60) % 60, rest = total % 60;
  if (total < 3600) return `${minutes}분${rest ? ` ${rest}초` : ""}`;
  return `${Math.floor(total / 3600)}시간${minutes ? ` ${minutes}분` : ""}`;
}

// The stretch record of an autonomy recording (2026-11-20, dashboard_server/run_recording.py: item.segment) as two
// short lines for the list: ["자율주행 · 2번 구간 · 다시 출발", "1분 45초 · 43.2 m · 열 끝 도착"]. null without a record.
function recordingStretchText(item) {
  const record = item?.segment;
  if (!record || typeof record !== "object") return null;
  const head = ["자율주행", Number.isInteger(record.segment) ? `${record.segment}번 구간` : null, record.resumed === true ? "다시 출발" : null];
  const time = Number(record.drive_time_s), distance = Number(record.distance_m);
  const line = record.ended !== true ? ["기록 중"]
    : [record.drive_time_s != null && Number.isFinite(time) ? fmtDurationWords(time) : null,
       record.distance_m != null && Number.isFinite(distance) ? `${distance.toFixed(1)} m` : null,
       typeof record.end_text === "string" && record.end_text ? record.end_text : null];
  return [head.filter(Boolean).join(" · "), line.filter(Boolean).join(" · ")];
}

function archiveScopeTag() {
  return typeof roverTab === "function" && roverTab() === "auto" ? "autonomy" : "manual";
}

// ---- Recorded executor state (2026-11-19) and its moving picture (2026-11-20) -------------------------------------
// An autonomy recording carries /row_executor/state (schema ubts.row_executor.state.v1). When the server's telemetry
// window of a recording has "executor": {"frames": [{"t_s", "state"}]} (the dashboard-telemetry-v1 profile, like the
// motor and GPS sidecars) the player shows the values of the state at or before the playback time (never a later one)
// and the moving picture of the 자율주행 tab for that time: the car at one place, the row axis up, the centre line, the
// row lines, the travelled path and the lookahead point (static/steer/minimap.mjs). A recording does not hold the
// executor's grid, so the cells are built again from the recorded LiDAR frames at or before the playback time, each
// placed with the pose of its own moment (static/steer/recorded-grid.mjs, loaded with the first autonomy recording).
// The player's own cache keeps only the LiDAR frames around the playback time, so every loaded window is also handed
// to the picture's store (archiveGridIngest). After a jump in time the windows of the seconds before it are asked for
// (archiveGridBackfill): the newest first, one at a time, only while the picture is on screen.
const ARCHIVE_EXECUTOR_KEEP = 3000;
const ARCHIVE_GRID_TELEMETRY_S = 24;      // the length of a telemetry window (ensureArchiveTelemetryForTime)
const ARCHIVE_GRID_SETTLE_MS = 350;       // a jump in time rests this long before older frames are asked for
const ARCHIVE_GRID_RETRY_MS = 5000;       // after a load that failed
const ARCHIVE_GRID_POINT_FIELDS = ["x_m", "y_m", "distance_m", "quality", "angle_deg"];   // as expandArchiveLidarPoints
const archiveGrid = { load: null, view: null, pending: [], onScreen: null, timer: null, busy: false, retryAt: 0, calm: null };

function mergeArchiveExecutorSegment(section) {
  const frames = Array.isArray(section?.frames) ? section.frames : [];
  if (!frames.length) return;
  const cache = state.archivePlayback.cache;
  cache.executorFrames ||= [];
  cache.executorKeys ||= new Set();
  for (const frame of frames) {
    const t = Number(frame?.t_s);
    const body = frame?.state ?? frame?.message ?? frame?.msg ?? frame?.data;
    if (!Number.isFinite(t) || t < 0 || !body || typeof body !== "object") continue;
    const key = t.toFixed(4);
    if (cache.executorKeys.has(key)) continue;
    cache.executorKeys.add(key);
    cache.executorFrames.push({ t_s: t, state: body });
    cache.executorVersion = (cache.executorVersion || 0) + 1;
  }
  cache.executorFrames.sort((a, b) => a.t_s - b.t_s);
  if (cache.executorFrames.length > ARCHIVE_EXECUTOR_KEEP) {
    cache.executorFrames = cache.executorFrames.slice(-ARCHIVE_EXECUTOR_KEEP);
    cache.executorKeys = new Set(cache.executorFrames.map((frame) => frame.t_s.toFixed(4)));
  }
}

// The recorded GPS mode and state in the words of the autonomy tab (static/steer/auto-model.mjs GNSS_MODE_TEXT,
// GNSS_STATE); an unknown code is shown as recorded.
const ARCHIVE_GNSS_MODE = { off: "사용 안 함", log_only: "기록만", geofence: "범위 감시", weak_constraint: "사용함" };
const ARCHIVE_GNSS_STATE = {
  ok: "정상", accepted: "받아들임", rejected: "거부", none: "수신 없음", absent: "수신 없음", no_fix: "미확정", fix: "확정",
  rtk: "RTK", stale: "오래됨", unused: "쓰지 않음", off: "꺼짐", waiting: "대기", fence_tripped: "범위 벗어남",
  no_run: "주행 없음", rejected_no_fix: "위치 미확정", rejected_hacc: "정확도 낮음", rejected_jump: "위치 급변",
  rejected_quality: "품질 낮음",
};

function archiveExecutorTiles(msg) {
  const n = (value) => (typeof value === "number" && Number.isFinite(value) ? value : null);
  const f = (value, digits, unit) => (n(value) === null ? "--" : `${value.toFixed(digits)}${unit}`);
  const deg = (mrad) => (n(mrad) === null ? "--" : `${(mrad * 180 / Math.PI / 1000).toFixed(1)}°`);
  const metrics = msg.metrics && typeof msg.metrics === "object" ? msg.metrics : {};
  const gnss = msg.gnss && typeof msg.gnss === "object" ? msg.gnss : {};
  return [
    ["횡 오차", f(msg.lat_m, 2, " m")], ["방향 오차", f(msg.head_deg, 1, "°")],
    ["조향 명령 / 측정", `${deg(msg.steer_cmd_mrad)} / ${deg(msg.steer_meas_mrad)}`], ["구동", f(msg.throttle_pct, 0, " %")],
    ["속도", f(msg.speed_mps, 2, " m/s")], ["거리", f(msg.dist_m, 1, " m")],
    ["통로", msg.corridor_ok === true ? `찾음${n(msg.width_m) === null ? "" : ` · 폭 ${msg.width_m.toFixed(2)} m`}` : msg.corridor_ok === false ? "못 찾음" : "--"],
    ["횡 RMS", f(metrics.lat_rms_m, 3, " m")],
    ["GPS", [ARCHIVE_GNSS_MODE[gnss.mode] ?? gnss.mode, gnss.mode === "off" ? null : (ARCHIVE_GNSS_STATE[gnss.state] ?? gnss.state)]
      .filter((v) => typeof v === "string").join(" · ") || "--"],
  ];
}

// The recorded executor part is shown for a recording with executor states, and for an autonomy recording before they arrive.
function archiveGridWanted(list) {
  const id = state.recordings.selectedId;
  if (!id) return false;
  if (Array.isArray(list) && list.length) return true;
  const item = state.recordings.items?.find((row) => row.id === id) || state.recordings.selected?.recording || state.recordings.selected;
  return recordingTag(item) === "autonomy";
}

// Another recording (or the same one opened again): nothing of the old one stays in the picture.
function archiveGridReset() {
  archiveGrid.pending = [];
  clearTimeout(archiveGrid.timer);
  archiveGrid.timer = null;
  archiveGrid.retryAt = 0;
  archiveGrid.view?.grid.reset();
  archiveGrid.view?.painter.forget();
}

// The LiDAR frames of a loaded window [startS, startS + durationS) for the picture's store.
function archiveGridIngest(timeline, startS, durationS) {
  if (!archiveGridWanted(state.archivePlayback.cache?.executorFrames)) return;
  const fields = timeline?.playback?.lidar_point_fields;
  const part = { frames: timeline?.lidar?.frames || [], fields: Array.isArray(fields) && fields.length ? fields : ARCHIVE_GRID_POINT_FIELDS,
    frame: timeline?.coordinate_frame || null, startS, endS: startS + durationS };
  if (archiveGrid.view) archiveGrid.view.grid.ingest(part.frames, part);
  else {
    archiveGrid.pending.push(part);                 // the module is on its way
    if (archiveGrid.pending.length > 12) archiveGrid.pending.shift();
  }
}

function archiveGridRedraw() {
  renderArchiveExecutor(state.archivePlayback.cache?.executorFrames || [], Math.max(0, Number(state.archivePlayback.timeS) || 0));
}

function archiveGridStart() {
  const canvas = $("roverArchiveAutoCanvas");
  if (archiveGrid.load || !canvas || performance.now() < archiveGrid.retryAt) return;
  archiveGrid.load = import("/static/steer/recorded-grid.mjs").then((mod) => {
    const read = (name, fallback) => getComputedStyle(document.documentElement).getPropertyValue(name).trim() || fallback;
    const grid = new mod.RecordedGrid();
    archiveGrid.view = { grid, painter: mod.recordedPainter(canvas, read, DPR_LIMIT) };
    // what arrived while the module was on its way, and the frames the player holds now
    for (const part of archiveGrid.pending.splice(0)) grid.ingest(part.frames, part);
    grid.ingest(state.archivePlayback.cache?.lidarFrames || [], { frame: state.recordings.timeline?.coordinate_frame || null });
    if (typeof ResizeObserver === "function") {
      new ResizeObserver((entries) => {
        const box = entries[entries.length - 1].contentRect;
        archiveGrid.view.painter.resized(box.width, box.height);
        archiveGridRedraw();
      }).observe(canvas);
    }
    const panel = $("roverArchiveAutoPanel");
    if (panel && typeof IntersectionObserver === "function") {
      new IntersectionObserver((entries) => {
        const seen = Boolean(entries[entries.length - 1].isIntersecting);
        if (seen === archiveGrid.onScreen) return;
        archiveGrid.onScreen = seen;
        if (seen) archiveGridRedraw();
      }).observe(panel);
    }
    archiveGridRedraw();
  }).catch((err) => {
    archiveGrid.load = null;
    archiveGrid.retryAt = performance.now() + ARCHIVE_GRID_RETRY_MS;
    debugLog("archive", "recorded grid", err);
  });
}

// After a jump in time the picture lacks the LiDAR frames of the seconds before it (and their poses). They are asked
// for one window at a time, the newest first, once the jump has rested. A new jump ends the asking (scheduleArchiveSeek
// aborts loads far from it) and the next picture starts it again for the new time.
function archiveGridBackfill(timeS) {
  const view = archiveGrid.view, playback = state.archivePlayback;
  if (!view || archiveGrid.busy || archiveGrid.timer || !state.recordings.timeline || performance.now() < archiveGrid.retryAt) return;
  const windowS = Math.max(1, Number(playback.segmentDurationS) || ARCHIVE_SEGMENT_SECONDS);
  if (!view.grid.missing(timeS, windowS).length) return;
  const id = state.recordings.selectedId;
  archiveGrid.timer = setTimeout(async () => {
    archiveGrid.timer = null;
    archiveGrid.busy = true;
    let poses = false;
    try {
      for (let asked = 0; asked < 12 && id === state.recordings.selectedId && archiveGrid.view === view && archiveGrid.onScreen !== false; asked += 1) {
        const at = Math.max(0, Number(playback.timeS) || 0);
        const start = view.grid.missing(at, windowS)[0];
        if (start === undefined) break;
        // the poses of those seconds are in the telemetry window before the one of the playback time
        const earlier = (Math.floor(at / ARCHIVE_GRID_TELEMETRY_S) - 1) * ARCHIVE_GRID_TELEMETRY_S;
        if (!poses && earlier >= 0 && start < earlier + ARCHIVE_GRID_TELEMETRY_S) {
          poses = true;
          await Promise.allSettled([...playback.loadingTelemetry.values()].map((task) => task.promise));
          if (id !== state.recordings.selectedId) break;
          await loadArchiveTelemetrySegment(id, earlier).catch((err) => debugLog("archive", "recorded grid poses", err));
        }
        const payload = await loadArchiveTimelineSegment(id, start, windowS);
        if (!payload || payload.ok === false) { archiveGrid.retryAt = performance.now() + ARCHIVE_GRID_RETRY_MS; break; }
        // a window the player already holds answers without frames: its frames came in when it was loaded
        if (id === state.recordings.selectedId && !view.grid.covered(start)) view.grid.ingest([], { startS: start, endS: start + windowS });
      }
    } catch (err) {
      if (err?.name !== "AbortError") archiveGrid.retryAt = performance.now() + ARCHIVE_GRID_RETRY_MS;
      debugLog("archive", "recorded grid history", err);
    } finally {
      archiveGrid.busy = false;
    }
  }, ARCHIVE_GRID_SETTLE_MS);
}

function renderArchiveExecutor(frames, timeS) {
  const panel = $("roverArchiveAutoPanel");
  if (!panel) return;
  const id = state.recordings.selectedId;
  const list = Array.isArray(frames) ? frames : [];
  const hidden = !archiveGridWanted(list);
  if (panel.hidden !== hidden) panel.hidden = hidden;
  if (hidden) return;
  const frame = archiveMotorFrameAtOrBefore(list, timeS);
  // the cells are drawn for a state with a pose in a recording with LiDAR frames
  const lidar = Number(state.recordings.timeline?.sync?.lidar_frames) > 0 && Boolean(frame?.state?.pose);
  const key = `${id}:${frame?.t_s}:${lidar}`;
  if (key !== state.archivePlayback.lastExecutorKey) {
    // the values of the moment: the state at or before the playback time
    state.archivePlayback.lastExecutorKey = key;
    text("roverArchiveAutoTime", frame ? `${fmtNumber(frame.t_s, 1)} s` : "--");
    text("roverArchiveAutoStatus", frame ? `재생 시각 이전에 기록된 마지막 실행기 상태입니다.${lidar ? " 나무 줄은 기록된 LiDAR로 다시 그렸습니다." : ""}`
      : list.length ? "이 시각 이전에 기록된 실행기 상태가 없습니다." : "이 기록에 실행기 상태가 없거나 서버가 아직 보내지 않았습니다.");
    const grid = $("roverArchiveAutoGrid");
    if (grid) {
      const rows = frame ? archiveExecutorTiles(frame.state) : [];
      const html = rows.map(([name, value]) => `<div><dt>${escapeHtml(name)}</dt><dd>${escapeHtml(value)}</dd></div>`).join("");
      if (grid.dataset.key !== html) { grid.dataset.key = html; grid.innerHTML = html; }
    }
  }
  // the moving picture of the playback time
  archiveGridStart();
  const view = archiveGrid.view;
  if (!view || archiveGrid.onScreen === false) return;
  view.grid.states(list, `${id}:${state.archivePlayback.cache?.executorVersion || 0}:${list.length}`);
  archiveGrid.calm ||= typeof matchMedia === "function" ? matchMedia("(prefers-reduced-motion: reduce)") : { matches: false };
  view.painter.paint(view.grid.picture(timeS, !archiveGrid.calm.matches));
  archiveGridBackfill(timeS);
}

function renderRecordingDetail(detail, options = {}) {
  const payload = detail?.recording || detail;
  const metadata = payload?.metadata || {};
  const archiveMedia = document.querySelector(".archive-media-grid");
  archiveMedia?.classList.toggle("camera-only", (metadata.kind || payload?.kind) === "camera");
  const analysis = payload?.analysis || {};
  const preview = payload?.preview_scan;
  const hasLidar = Boolean(preview?.points?.length);
  const normalizedPreview = hasLidar ? normalizeArchiveLidarResult(preview, analysis) : null;

  const id = payload ? payload.id || metadata.id || state.recordings.selectedId || "" : "";
  window.LiveMap?.selectArchive(id);
  if (!id) state.recordings.timeline = null;
  setupArchivePlayback(payload, normalizedPreview, options);
  const rows = id ? [
    { title: "ID", sub: recordingSensorLabel(payload), ok: true, level: "ok", value: id },
    { title: "길이", sub: `${formatShortDate(payload.display_started_at || metadata.started_at)}${payload.clock_quality === "gnss_estimated" ? ` · GPS 시각 보정 (원본 ${formatShortDate(metadata.started_at)})` : ""}`, ok: true, level: "ok", value: metadata.duration_s ? `${fmtNumber(metadata.duration_s, 1)}s` : metadata.state || "--" },
    { title: "기록 시각", sub: recordingClockDescription(payload), ok: ["gnss_estimated", "gnss_receiver_estimated"].includes(payload.clock_quality), level: ["gnss_estimated", "gnss_receiver_estimated"].includes(payload.clock_quality) ? "ok" : "warn", value: recordingClockLabel(payload) },
    { title: "Timeline", sub: `${payload?.timeline_summary?.lidar_frames || state.recordings.timeline?.lidar?.frames?.length || 0} lidar · ${payload?.timeline_summary?.camera_frames || state.recordings.timeline?.camera?.frames?.length || 0} camera · ${payload?.timeline_summary?.imu_frames || state.recordings.timeline?.imu?.frames?.length || 0} imu`, ok: Boolean(state.recordings.timeline), level: state.recordings.timeline ? "ok" : "warn", value: state.recordings.timeline ? "playable" : "preview only" },
  ] : [
    { title: "기록", sub: "기록을 선택하세요", ok: false, level: "warn", value: "--" },
  ];
  renderStatusRows("roverArchiveDetails", rows);
  renderArchiveDownloadButton(payload, id);
}

function configureArchiveNativeVideo(payload, { preservePlayback = false } = {}) {
  // A metadata refresh must not switch an explicitly selected camera source.
  if (preservePlayback && state.archivePlayback.selectedSourceVideo) return;
  const selector = $("roverArchiveCameraSource");
  const alternatives = payload?.adaptive_video?.alternative_sources || [];
  const archiveId = payload?.id || state.recordings.selectedId;
  if (selector && !payload?.selected_camera_source) {
    selector.innerHTML = ""; selector.hidden = !alternatives.length;
    const primary = payload?.adaptive_video;
    if (primary) {
      for (const source of [{...primary,id:'primary',label:`${primary.source_height}p 고속 원본`},...alternatives]) {
        const option = document.createElement('option');option.value=source.id;
        option.textContent=`${source.label} · ${fmtNumber(source.fps,1)} fps`;
        selector.append(option);
      }
      selector.value='primary';
      selector.onchange=()=>{
        if(archiveId!==state.recordings.selectedId)return;
        const source=selector.value==='primary'?primary:alternatives.find(s=>s.id===selector.value);
        if(!source)return;
        const replacement={...payload,selected_camera_source:true,adaptive_video:source,
          source_video:{width:source.source_width,height:source.source_height,fps:source.fps,timeline_offset_s:source.timeline_offset_s},
          video_url:source.fallback_url};
        state.archivePlayback.selectedSourceVideo=replacement.source_video;
        state.archivePlayback.sourceSwitchTimeS=state.archivePlayback.timeS;
        configureArchiveNativeVideo(replacement);
        const video=$("roverArchiveCameraVideo");
        if(state.archivePlayback.playing && video) {
          Promise.resolve(video.play()).then(()=>{
            if(!state.archivePlayback.playing && !state.archivePlayback.videoPlayRequested)video.pause();
          }).catch(()=>{});
        }
      };
    }
    state.archivePlayback.selectedSourceVideo=null;
  }
  const video = $("roverArchiveCameraVideo");
  const frame = video?.closest(".archive-camera-frame");
  if (!video) return;
  const url = String(payload?.video_url || "");
  const changed = url !== state.archivePlayback.nativeVideoUrl;
  // A later same-URL detail refresh must retain the pending source switch's
  // resume intent until metadata arrives. An explicit seek already owns its cursor.
  const restorePlayback = preservePlayback && Boolean(url) &&
    (changed || Number.isFinite(state.archivePlayback.sourceSwitchTimeS));
  if (restorePlayback && changed) state.archivePlayback.sourceSwitchTimeS = state.archivePlayback.timeS;
  if (changed) {
    state.archivePlayback.videoPlayRevision=(state.archivePlayback.videoPlayRevision||0)+1;
    state.archivePlayback.videoPlayRequested=false;
    state.archivePlayback.videoPlayer?.destroy();
    state.archivePlayback.videoPlayer = null;
    state.archivePlayback.videoQuality = {};
    video.pause();
    video.removeAttribute("src");
    video.load();
    state.archivePlayback.nativeVideoUrl = url;
    state.archivePlayback.nativeVideoReady = false;
    state.archivePlayback.nativeVideoError = "";
    const image = $("roverArchiveCameraImage");
    if (url && image) image.removeAttribute("src");
  }
  state.archivePlayback.nativeVideoOffsetS = Math.max(0, Number(payload?.adaptive_video?.timeline_offset_s || payload?.source_video?.timeline_offset_s) || 0);
  frame?.classList.toggle("native-video", Boolean(url));
  if (!url) return;
  video.onloadedmetadata = () => {
    if (url !== state.archivePlayback.nativeVideoUrl) return;
    if (Number.isFinite(state.archivePlayback.sourceSwitchTimeS)) {
      const cursor=state.archivePlayback.sourceSwitchTimeS;
      state.archivePlayback.sourceSwitchTimeS=null;
      state.archivePlayback.timeS=cursor;
      video.currentTime=archiveVideoTime(cursor);
    }
    state.archivePlayback.nativeVideoReady = true;
    state.archivePlayback.nativeVideoError = "";
    const duration = Number(video.duration) + state.archivePlayback.nativeVideoOffsetS;
    if (Number.isFinite(duration) && duration > 0) {
      state.archivePlayback.durationS = Math.max(state.archivePlayback.durationS, duration);
      syncArchiveScrubberBounds();
    }
    if (!state.archivePlayback.playing && Math.abs(video.currentTime - archiveVideoTime(state.archivePlayback.timeS)) > 0.08) {
      video.currentTime = archiveVideoTime(state.archivePlayback.timeS);
    }
    if (restorePlayback && state.archivePlayback.playing) {
      const revision = state.archivePlayback.videoPlayRevision || 0;
      Promise.resolve(video.play()).then(() => {
        if (revision !== (state.archivePlayback.videoPlayRevision || 0) || !state.archivePlayback.playing) {
          if (!state.archivePlayback.playing && !state.archivePlayback.videoPlayRequested) video.pause();
        }
      }).catch(error => {
        if (url !== state.archivePlayback.nativeVideoUrl || revision !== (state.archivePlayback.videoPlayRevision || 0) ||
            !state.archivePlayback.playing) return;
        state.archivePlayback.nativeVideoError = error?.message || "native video play failed";
        stopArchivePlayback(false);
      });
    }
    // Another metadata listener restores the fallback's saved media cursor.
    renderArchiveFrame({sampleMedia:false});
  };
  video.onerror = () => {
    if (url !== state.archivePlayback.nativeVideoUrl) return;
    state.archivePlayback.nativeVideoReady = false;
    state.archivePlayback.nativeVideoError = "native video decode failed";
    renderArchiveFrame();
  };
  video.onended = () => {
    if (!state.archivePlayback.playing) return;
    state.archivePlayback.timeS = state.archivePlayback.durationS;
    stopArchivePlayback(false);
  };
  if (changed) {
    state.archivePlayback.videoPlayer = new window.ArchiveVideoPlayer(video,
      $("roverArchiveVideoQuality"), payload?.adaptive_video, url, quality => {
        state.archivePlayback.videoQuality = quality; renderArchiveFrame();
      });
  }
}

function archiveVideoTime(timeS) {
  return Math.max(0, Number(timeS) - (state.archivePlayback.nativeVideoOffsetS || 0));
}

function archiveRenderClock({sampleMedia=true}={}) {
  const p=state.archivePlayback,video=archiveNativeVideo();
  if(sampleMedia && p.playing && p.nativeVideoReady && !p.nativeVideoError
      && !Number.isFinite(p.sourceSwitchTimeS) && video && video.readyState>=1 && !video.seeking) {
    const media=Number(video.currentTime),offset=Number(p.nativeVideoOffsetS||0),next=media+offset;
    if(Number.isFinite(media) && media>=0 && Number.isFinite(offset) && Number.isFinite(next)) {
      const duration=Number(p.durationS);
      p.timeS=Math.max(0,Number.isFinite(duration)&&duration>0?Math.min(duration,next):next);
    }
  }
  return Math.max(0,Number(p.timeS)||0);
}

function archiveNativeVideo() {
  const video = $("roverArchiveCameraVideo");
  return state.archivePlayback.nativeVideoUrl && video ? video : null;
}

function syncArchiveScrubberBounds() {
  for (const id of ["roverArchiveScrubber"]) {
    const scrubber = $(id);
    if (!scrubber) continue;
    scrubber.max = String(Math.max(0, state.archivePlayback.durationS));
    scrubber.value = String(Math.min(Number(scrubber.value) || 0, state.archivePlayback.durationS));
    scrubber.disabled = !state.recordings.timeline;
  }
}

function setupArchivePlayback(payload, fallbackPreview, { preservePlayback = false } = {}) {
  const timeline = state.recordings.timeline;
  configureArchiveNativeVideo(payload, { preservePlayback });
  if (!preservePlayback) prepareArchiveTimelineCache(timeline);
  const metadata = payload?.metadata || {};
  const duration = Number(timeline?.duration_s || metadata.duration_s || 0);
  const lidarFrames = state.archivePlayback.cache.lidarFrames || [];
  const cameraFrames = state.archivePlayback.cache.cameraFrames || [];
  const imuFrames = state.archivePlayback.cache.imuFrames || [];
  const gpsFrames = state.archivePlayback.cache.gpsFrames || [];
  const lastFrameTime = Math.max(0, ...gpsFrames.map((frame) => Number(frame.t_s) || 0), ...lidarFrames.map((frame) => Number(frame.t_s) || 0), ...cameraFrames.map((frame) => Number(frame.t_s) || 0), ...imuFrames.map((frame) => Number(frame.t_s) || 0));
  state.archivePlayback.durationS = Math.max(duration, lastFrameTime);
  state.archivePlayback.timeS = Math.min(state.archivePlayback.timeS || 0, state.archivePlayback.durationS);
  state.archivePlayback.fallbackPreview = fallbackPreview;
  syncArchiveScrubberBounds();
  const play = $("roverArchivePlayButton");
  if (play) play.disabled = !timeline;
  renderArchiveFrame();
}

function suspendLiveCameraForArchive() {
  const data = state.telemetry || {};
  const bridge = data.connection?.rover_bridge || (data.services || data.connection?.services || {}).rover_bridge || {};
  const age = Date.now() / 1000 - Number(bridge.updated_at);
  // Only a fresh, explicitly disarmed controller permits saving bandwidth.
  // No sensor capture/recording setting is changed; this tab's preview alone
  // is suspended. Active/unknown driving must retain its live camera view.
  if (state.roverControl.lease || bridge.alive === false || bridge.armed !== false || !Number.isFinite(age) || age < -0.25 || age > 3) return;
  if (!state.cameraLive.socket || state.cameraLive.userStopped) return;
  state.archivePlayback.resumeLiveCamera = true;
  closeCameraLive("기록 재생을 위해 실시간 영상 수신을 잠시 멈춥니다.", { userStopped: true });
}

function resumeLiveCameraAfterArchive() {
  if (!state.archivePlayback.resumeLiveCamera) return;
  state.archivePlayback.resumeLiveCamera = false;
  if (routeSupportsCameraLive() && !state.cameraLive.socket) startCameraLive();
}

async function toggleArchivePlayback() {
  if (state.archivePlayback.playing || state.archivePlayback.videoPlayRequested) {
    stopArchivePlayback(false);
    return;
  }
  if (!state.recordings.timeline) return;
  suspendLiveCameraForArchive();
  const startTime = state.archivePlayback.timeS >= state.archivePlayback.durationS ? 0 : state.archivePlayback.timeS;
  const video = archiveNativeVideo();
  const revision=(state.archivePlayback.videoPlayRevision||0)+1;
  state.archivePlayback.videoPlayRevision=revision;state.archivePlayback.videoPlayRequested=true;
  if (video) {
    try {
      video.currentTime = archiveVideoTime(startTime);
      await video.play();
      if(revision!==state.archivePlayback.videoPlayRevision) {
        if(!state.archivePlayback.playing && !state.archivePlayback.videoPlayRequested)video.pause();
        return;
      }
    } catch (err) {
      if(revision!==state.archivePlayback.videoPlayRevision)return;
      state.archivePlayback.videoPlayRequested=false;
      state.archivePlayback.nativeVideoError = err?.message || "native video play failed";
      resumeLiveCameraAfterArchive();
      renderArchiveFrame();
      return;
    }
  }
  state.archivePlayback.videoPlayRequested=false;
  state.archivePlayback.playing = true;
  state.archivePlayback.startedAt = performance.now();
  state.archivePlayback.startTimeS = startTime;
  scheduleArchiveSegmentsForTime(startTime, { reschedule: true })
    .catch((err) => debugLog("archive", "initial segment load failed", err));
  archivePlaybackTick();
}

function queueArchivePlaybackTick() {
  // Visible/occluded windows can throttle rAF to one second. Keep the sensor
  // playback clock at media rate; the first callback cancels its partner.
  let completed=false;
  const run=()=>{
    if(completed)return;completed=true;
    clearTimeout(state.archivePlayback.tickTimer);
    if(state.archivePlayback.raf)cancelAnimationFrame(state.archivePlayback.raf);
    state.archivePlayback.tickTimer=null;state.archivePlayback.raf=null;
    archivePlaybackTick(performance.now());
  };
  state.archivePlayback.raf=requestAnimationFrame(run);
  state.archivePlayback.tickTimer=setTimeout(run,80);
}

function stopArchivePlayback(reset = false) {
  state.archivePlayback.playing = false;
  state.archivePlayback.videoPlayRequested=false;
  state.archivePlayback.videoPlayRevision=(state.archivePlayback.videoPlayRevision||0)+1;
  state.archivePlayback.mapLagS = 0;
  resumeLiveCameraAfterArchive();
  const video = archiveNativeVideo();
  if (video && !video.paused) video.pause();
  if (state.archivePlayback.raf) {
    cancelAnimationFrame(state.archivePlayback.raf);
    state.archivePlayback.raf = null;
  }
  clearTimeout(state.archivePlayback.tickTimer);
  state.archivePlayback.tickTimer=null;
  if (reset) {
    state.archivePlayback.timeS = 0;
    if (video) {
      try { video.currentTime = 0; } catch (_) { /* metadata is not ready yet */ }
    }
  }
  renderArchiveFrame();
}

function syncArchiveMapClock(next){
  // 2026-11-21: the video and the sensors do not wait for the recorded map any more. The map of a recording is
  // built on the Jetson while the recording is played (about 1.6 times real time, measured), one request per step,
  // so over a link with a 0.2 s round trip it is always some tenths of a second behind the video. Holding the video
  // until the map had caught up paused it several times a second (measured through the tunnel: 26 s of playback in
  // 45 s) and for minutes after a jump forward. The map now follows the playback; how far it is behind only puts
  // a note in the time label, from ARCHIVE_MAP_LAG_NOTE_S on.
  const lag=Number(window.LiveMap?.archiveLag?.(state.recordings.selectedId,next));
  state.archivePlayback.mapLagS=Number.isFinite(lag)&&lag>0?lag:0;
}
const ARCHIVE_MAP_LAG_NOTE_S=2;

function archivePlaybackTick(now = performance.now()) {
  if (!state.archivePlayback.playing) return;
  if (Number.isFinite(state.archivePlayback.sourceSwitchTimeS)) {
    queueArchivePlaybackTick();return;
  }
  const video = archiveNativeVideo();
  const elapsed = (now - state.archivePlayback.startedAt) / 1000;
  const next = video ? Number(video.currentTime) + (state.archivePlayback.nativeVideoOffsetS || 0) :
    state.archivePlayback.startTimeS + elapsed;
  if (next >= state.archivePlayback.durationS) {
    state.archivePlayback.timeS = state.archivePlayback.durationS;
    stopArchivePlayback(false);
    return;
  }
  state.archivePlayback.timeS = next;
  syncArchiveMapClock(next);
  scheduleArchiveSegmentsForTime(next).catch((err) => debugLog("archive", "playback segment load failed", err));
  if (now - (state.archivePlayback.lastPlaybackRenderAt || 0) >= 50) {
    state.archivePlayback.lastPlaybackRenderAt = now;
    renderArchiveFrame();
  }
  queueArchivePlaybackTick();
}

function setArchiveTime(value, options = {}) {
  if (options.pause) stopArchivePlayback(false);
  state.archivePlayback.timeS = Math.max(0, Math.min(state.archivePlayback.durationS, Number(value) || 0));
  // An explicit seek while the replacement video's metadata is loading wins
  // over the cursor saved when the source switch began.
  if (Number.isFinite(state.archivePlayback.sourceSwitchTimeS)) {
    state.archivePlayback.sourceSwitchTimeS = state.archivePlayback.timeS;
  }
  window.LiveMap?.archiveTime(state.recordings.selectedId,state.archivePlayback.timeS,state.archivePlayback.playing,true);
  const video = archiveNativeVideo();
  if (video && Math.abs(Number(video.currentTime || 0) - archiveVideoTime(state.archivePlayback.timeS)) > 0.04) {
    try { video.currentTime = archiveVideoTime(state.archivePlayback.timeS); } catch (_) { /* metadata is not ready yet */ }
  }
  scheduleArchiveSeek(state.archivePlayback.timeS);
  renderArchiveFrame({sampleMedia:false});
}

function renderArchiveFrame(options={}) {
  const timeline = state.recordings.timeline;
  const timeS = archiveRenderClock(options);
  window.LiveMap?.archiveTime(state.recordings.selectedId, timeS, state.archivePlayback.playing);
  const cache = state.archivePlayback.cache || {};
  window.RoverGps?.archive(cache.gpsFrames || [], timeS, state.recordings.selectedId, timeline?.gps?.enabled === true);
  renderArchiveMotor(timeline?.motor, cache.motorFrames || [], timeS);
  if (typeof renderArchiveExecutor === "function") renderArchiveExecutor(cache.executorFrames || [], timeS);
  const lidarFrame = timeline ? nearestTimelineFrame(cache.lidarFrames || [], timeS) : null;
  const cameraFrame = timeline && state.archivePlayback.sync ? nearestTimelineFrame(cache.cameraFrames || [], timeS) : null;
  const nearestImuFrame = timeline ? nearestTimelineFrame(cache.imuFrames || [], timeS) : null;
  const imuSyncDeltaS = nearestImuFrame ? Math.abs(Number(nearestImuFrame.t_s || 0) - timeS) : null;
  const imuFrame = Number.isFinite(imuSyncDeltaS) && imuSyncDeltaS <= ARCHIVE_IMU_SYNC_MAX_DELTA_S
    ? nearestImuFrame
    : null;
  const lidarResult = lidarFrame?.result || state.archivePlayback.fallbackPreview;
  const lidarRenderKey = `${lidarFrame?.cacheIndex}:${state.archivePlayback.range}`;
  if (lidarRenderKey !== state.archivePlayback.lastLidarRenderKey) {
    const painted = drawLidarMapOnCanvas($("roverArchiveLidarCanvas"), lidarResult, lidarResult?.analysis, {
      range: state.archivePlayback.range,
    });
    // Remembered for the resize path: a canvas that was never painted stays blank.
    if (painted) state.archivePlayback.lidarPainted = true;
    state.archivePlayback.lastLidarRenderKey = lidarRenderKey;
  }

  const image = $("roverArchiveCameraImage");
  const nativeVideo = archiveNativeVideo();
  if (!nativeVideo && image && (cameraFrame?.frame_url || cameraFrame?.data)) {
    warmArchiveCameraCache(cameraFrame);
    const src = archiveCameraFrameSrc(cameraFrame);
    const cameraRenderKey = `${cameraFrame.cacheIndex}:${src}`;
    if (src && state.archivePlayback.lastCameraRenderKey !== cameraRenderKey) {
      image.src = src;
      state.archivePlayback.lastCameraRenderKey = cameraRenderKey;
    }
  } else if (!nativeVideo && image && !cameraFrame) {
    image.removeAttribute("src");
    state.archivePlayback.lastCameraRenderKey = "";
  }
  const sourceVideo = state.archivePlayback.selectedSourceVideo || state.recordings.selected?.recording?.source_video || state.recordings.selected?.source_video || {};
  const cameraCaption = state.archivePlayback.nativeVideoError
    ? `video error · ${state.archivePlayback.nativeVideoError}`
    : nativeVideo
      ? `${state.archivePlayback.videoQuality?.loading ? "재생기 받는 중 · " : state.archivePlayback.videoQuality?.retrying ? "네트워크 재연결 중 · " : ""}${nativeVideo.videoHeight || state.archivePlayback.videoQuality?.height || sourceVideo.height || ""}p · ${state.archivePlayback.videoQuality?.auto ? "자동 · " : ""}${fmtNumber(sourceVideo.fps || 0, 1)} fps · ${fmtClock(timeS)}`
    : cameraFrame
      ? `${nativeVideo ? `native ${fmtNumber(sourceVideo.fps || 0, 0)} fps · ` : ""}${cameraFrame.frame_id || "camera"} · ${fmtClock(timeS)} · ${cameraFrame.source_width || cameraFrame.width}x${cameraFrame.source_height || cameraFrame.height}`
      : state.archivePlayback.sync ? "카메라 화면 없음" : "동시 재생 꺼짐";
  text("roverArchiveCameraCaption", cameraCaption);
  const imuSample = imuFrame?.corrected || imuFrame?.raw || {};
  const imuEuler = imuSample.euler_deg || {};
  const imuCalibrationState = imuFrame?.calibration_state || timeline?.imu?.calibration?.state || "unavailable";
  const imuAcceleration = imuSample.linear_acceleration || {};
  const imuGyro = imuSample.angular_velocity || {};
  const imuMagnetic = imuFrame?.magnetic?.magnetic_field_uT || {};
  const imuFrameCount = Math.max(
    Number(timeline?.sync?.imu_frames || 0),
    Number(state.recordings.selected?.recording?.timeline_summary?.imu_frames || 0),
    Number(state.recordings.selected?.recording?.metadata?.timeline?.imu_frames || 0),
    cache.imuFrames?.length || 0,
  );
  const hasArchivedImu = imuFrameCount > 0;
  const waitingForImu=hasArchivedImu && !imuFrame && state.archivePlayback.loadingTelemetry.size>0;
  const imuPanel = $("roverArchiveImuPanel");
  if (imuPanel) imuPanel.hidden = !hasArchivedImu;
  const colors = chartColors();
  renderReadout("roverArchiveImuStats", [
    {
      label: "Roll / Pitch / Yaw",
      value: imuFrame ? `${fmtNumber(imuEuler.roll, 1)}° / ${fmtNumber(imuEuler.pitch, 1)}° / ${fmtNumber(imuEuler.yaw, 1)}°` : "--",
      sub: imuFrame ? `${imuFrame.corrected ? "start-corrected" : "raw"} · calibration ${imuCalibrationState}` : waitingForImu?"IMU 기록을 받는 중":"이 재생 시각에는 IMU 기록 없음",
      color: imuFrame?.corrected ? colors.good : colors.amber,
    },
    { label: "가속도", value: imuFrame && vectorNorm(imuAcceleration) !== null ? `${fmtNumber(vectorNorm(imuAcceleration), 3)} m/s²` : "--", sub: `x ${fmtNumber(imuAcceleration.x, 3)} · y ${fmtNumber(imuAcceleration.y, 3)} · z ${fmtNumber(imuAcceleration.z, 3)}`, color: colors.good },
    { label: "각속도", value: imuFrame && vectorNorm(imuGyro) !== null ? `${fmtNumber(vectorNorm(imuGyro), 4)} rad/s` : "--", sub: `x ${fmtNumber(imuGyro.x, 4)} · y ${fmtNumber(imuGyro.y, 4)} · z ${fmtNumber(imuGyro.z, 4)}`, color: colors.amber },
    { label: "자기장", value: imuFrame && vectorNorm(imuMagnetic) !== null ? `${fmtNumber(vectorNorm(imuMagnetic), 2)} µT` : "--", sub: imuFrame?.calibration_reference_id ? `reference ${imuFrame.calibration_reference_id}` : "magnetometer unavailable", color: colors.blue },
  ]);
  text("roverArchiveImuTime", imuFrame ? `IMU ${fmtNumber(imuFrame.t_s, 3)}s / Δ ${fmtNumber(imuSyncDeltaS, 3)}s` : "IMU -- / Δ --");
  text("roverArchiveImuCaption", imuFrame ? `${imuFrame.corrected ? "/imu/data · base_link corrected" : "/imu/data_raw · sensor-native"} · playback ${fmtNumber(timeS, 3)}s` : waitingForImu?"IMU 기록을 받아 재생 시각에 맞추고 있습니다.":"재생 시각에 맞는 IMU 기록을 기다립니다.");
  drawImuAttitude($("roverArchiveImuCanvas"), Number(imuEuler.roll), Number(imuEuler.pitch), Number(imuEuler.yaw));
  const mapLagS = state.archivePlayback.playing ? Number(state.archivePlayback.mapLagS) || 0 : 0;
  const mapWait = mapLagS >= ARCHIVE_MAP_LAG_NOTE_S ? (window.LiveMap?.waitReason?.() || `지도는 ${Math.round(mapLagS)}초 뒤에 따라옵니다`) : '';
  text("roverArchiveTime", `${fmtClock(timeS)} / ${fmtClock(state.archivePlayback.durationS)}${mapWait ? ` · ${mapWait}` : ''}`);
  const scrubber = $("roverArchiveScrubber");
  if (scrubber && document.activeElement !== scrubber) scrubber.value = String(timeS);
  const play = $("roverArchivePlayButton");
  if (play) play.textContent = state.archivePlayback.playing || state.archivePlayback.videoPlayRequested ? "일시정지" : "재생";

}


function mergeArchiveMotorSegment(motor) {
  if (!motor) return;
  if (state.recordings.timeline) {
    state.recordings.timeline.motor = {
      ...(state.recordings.timeline.motor || {}),
      ...Object.fromEntries(Object.entries(motor).filter(([key]) => key !== "frames")),
      frames: [],
    };
  }
  const cache = state.archivePlayback.cache;
  cache.motorFrames ||= [];
  cache.motorKeys ||= new Set();
  for (const frame of Array.isArray(motor.frames) ? motor.frames : []) {
    // Malformed timestamps must not become a fabricated sample at zero.
    if (!Number.isFinite(frame?.t_s) || frame.t_s < 0) continue;
    const key = String(frame.t_s);
    if (cache.motorKeys.has(key)) continue;
    cache.motorKeys.add(key);
    cache.motorFrames.push(frame);
  }
  cache.motorFrames.sort((a, b) => a.t_s - b.t_s);
}

function archiveMotorFrameAtOrBefore(frames, timeS) {
  if (!Array.isArray(frames) || !frames.length || !Number.isFinite(timeS)) return null;
  let low = 0;
  let high = frames.length;
  // Right insertion point: unlike sensor previews, never use a future command.
  while (low < high) {
    const mid = Math.floor((low + high) / 2);
    if (frames[mid].t_s <= timeS) low = mid + 1;
    else high = mid;
  }
  return low > 0 ? frames[low - 1] : null;
}

function archiveMotorPwmLabel(value) {
  if (!Number.isInteger(value) || value < 0 || value > 255) return "미기록";
  return `${value} / 255 · ${(value / 255 * 100).toFixed(1)}%`;
}

function renderArchiveMotor(motor, frames, timeS) {
  const panel = $("roverArchiveMotorPanel");
  if (!panel) return;
  panel.hidden = !state.recordings.selectedId;
  const frame = archiveMotorFrameAtOrBefore(frames, timeS);
  const simulated = frame?.status?.simulated === true || (!frame && motor?.source_kind === "simulation");
  text("roverArchiveMotorOutputLabel", simulated ? "시뮬레이션 출력 PWM" : "MCU 보고 PWM");
  const hasArchive = motor?.enabled === true || Number(motor?.frame_count) > 0 || frames.length > 0;
  text("roverArchiveMotorTime", frame ? `스냅샷 ${frame.t_s.toFixed(3)}s` : "스냅샷 없음");
  let status;
  if (!hasArchive) {
    status = "이 녹화에는 모터 핀 신호 기록이 없습니다.";
  } else if (!frame) {
    status = "현재 재생 시각 이전의 모터 스냅샷이 없습니다. 이후 신호는 표시하지 않습니다.";
  } else {
    const limit = Number.isFinite(motor?.stale_after_s) && motor.stale_after_s > 0 ? motor.stale_after_s : 1.5;
    const gap = Math.max(0, timeS - frame.t_s);
    const stale = gap > limit || frame.source_stale !== false;
    const connected = frame.status?.connected;
    const connection = simulated
      ? connected === true ? "가상 구동 준비됨" : connected === false ? "가상 구동 준비 안 됨" : "가상 구동 상태 미기록"
      : connected === true ? "MCU 연결됨" : connected === false ? "MCU 연결 끊김" : "MCU 연결 여부 미기록";
    status = `${stale ? "오래되거나 시각을 확인할 수 없는 스냅샷" : "기록된 상태 스냅샷"} · ${connection} · 재생 시각과 차이 ${gap.toFixed(2)}s`;
    if (Number.isFinite(frame.source_age_s) && frame.source_age_s >= 0) {
      status += ` · 수집 당시 상태 파일 경과 ${frame.source_age_s.toFixed(2)}s`;
    }
  }
  if (motor?.error || motor?.sidecar_read_error || Number(motor?.sidecar_malformed_rows) > 0 || Number(motor?.dropped_time_frames) > 0) {
    // Controlled label only; archive errors may contain untrusted free text.
    status = `일부 모터 기록 누락 또는 읽기 오류 · ${status}`;
  }
  text("roverArchiveMotorStatus", status);
  const hz = Number.isFinite(motor?.sample_hz) && motor.sample_hz > 0 ? `${motor.sample_hz} Hz ` : "";
  text("roverArchiveMotorCaption", hasArchive
    ? simulated
      ? `${hz}시뮬레이션 상태 스냅샷입니다. 요청값과 가감속된 가상 속도 명령을 최대 설정 속도 기준 0–255로 환산했습니다. 실제 GPIO 신호·MCU 보고값·전압·측정 바퀴 속도가 아닙니다.`
      : `${hz}상태 스냅샷입니다. 요청 PWM은 주행 요청에서 계산한 값이며, MCU 보고 PWM은 마지막 수신 캐시입니다. 핀별 보고 시각·모든 직렬 명령·전압·실제 바퀴 속도는 기록하지 않습니다.`
    : "이 기능 적용 후 시작한 녹화에서 핀별 요청값과 MCU 보고값을 확인할 수 있습니다.");
  const body = $("roverArchiveMotorRows");
  if (!body) return;
  const roles = {left_forward: "왼쪽 정방향", left_reverse: "왼쪽 역방향", right_forward: "오른쪽 정방향", right_reverse: "오른쪽 역방향", driver_enable: "드라이버 활성화"};
  const pins = Array.isArray(motor?.pins) ? motor.pins : [];
  const html = pins.length ? pins.map((pin) => {
    const values = frame?.pins?.[pin.pin];
    const gpio = Number.isInteger(pin.gpio) && pin.gpio >= 0 ? `GPIO ${pin.gpio}` : "GPIO 미기록";
    const role = roles[pin.role] || pin.role || "역할 미기록";
    return `<tr><th scope="row">${escapeHtml(pin.pin || "핀 미기록")}</th><td>${escapeHtml(gpio)}</td><td>${escapeHtml(role)}</td><td>${escapeHtml(archiveMotorPwmLabel(values?.requested_pwm_derived))}</td><td>${escapeHtml(archiveMotorPwmLabel(simulated ? values?.simulated_output_pwm : values?.reported_pwm))}</td></tr>`;
  }).join("") : '<tr><td colspan="5">핀 매핑 기록 없음</td></tr>';
  if (body.innerHTML !== html) body.innerHTML = html;
}


function nearestTimelineFrame(frames, timeS) {
  if (!Array.isArray(frames) || !frames.length) return null;
  let low = 0;
  let high = frames.length - 1;
  while (low <= high) {
    const mid = Math.floor((low + high) / 2);
    const stamp = Number(frames[mid].t_s) || 0;
    if (stamp < timeS) low = mid + 1;
    else high = mid - 1;
  }
  if (low <= 0) return frames[0];
  if (low >= frames.length) return frames[frames.length - 1];
  const before = frames[low - 1];
  const after = frames[low];
  return Math.abs((Number(after.t_s) || 0) - timeS) < Math.abs(timeS - (Number(before.t_s) || 0)) ? after : before;
}
