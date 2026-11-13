import {FollowModel, validFix, coordinates, frameAt, pathFeatures, accuracyCircle, usedSatellitesText, receptionText} from './gps-map-model.mjs';

// The map library (about 300 kB compressed, with its style sheet and the pmtiles reader) and the Korean label font
// (about 630 kB) are needed only while a map is on screen, so they are loaded then and not with the page (NET-03).
// The server gives these addresses their content-hash versions (NET-04); the library would otherwise find its worker
// file from its own address without one.
const MAP_LIBRARY_URL = "/static/vendor/maplibre-gl/maplibre-gl.mjs";
const MAP_WORKER_URL = "/static/vendor/maplibre-gl/maplibre-gl-worker.mjs";
const MAP_STYLE_URL = "/static/vendor/maplibre-gl/maplibre-gl.css";
const PMTILES_URL = "/static/vendor/pmtiles/pmtiles.js";
let mapLibraryPromise = null;
function addToHead(element, address) {
  return new Promise((resolve, reject) => {
    element.onload = () => resolve(element);
    element.onerror = () => { element.remove(); reject(new Error(`지도 파일을 받지 못했습니다 · ${address}`)); };
    document.head.append(element);
  });
}
function mapStyleSheet() {
  if (document.querySelector('link[data-map-style]')) return null;
  const link = document.createElement('link');
  link.rel = 'stylesheet'; link.href = MAP_STYLE_URL; link.dataset.mapStyle = '';
  return addToHead(link, MAP_STYLE_URL);
}
function pmtilesReader() {
  if (window.pmtiles) return null;
  const script = document.createElement('script');
  script.src = PMTILES_URL; script.async = true;
  return addToHead(script, PMTILES_URL);
}
function mapLibrary() {
  if (!mapLibraryPromise) {
    mapLibraryPromise = Promise.all([import(MAP_LIBRARY_URL), pmtilesReader(), mapStyleSheet()]).then(([library]) => {
      library.setWorkerUrl?.(MAP_WORKER_URL);
      library.addProtocol('pmtiles', new window.pmtiles.Protocol().tile);
      return library;
    }).catch(error => { mapLibraryPromise = null; throw error; });
  }
  return mapLibraryPromise;
}

const byId = id => document.getElementById(id);
const text = (id, value) => { const element = byId(id); if (element) element.textContent = value; };
const number = (value, digits=2) => typeof value === 'number' && Number.isFinite(value) ? value.toFixed(digits) : '—';
const empty = () => ({type:'FeatureCollection', features:[]});
// pathFeatures owns validity/gap/order behavior. Compare its actual LineString
// output, never raw replay time, input identity or an assumed sorted prefix.
// Keep only the last submitted GeoJSON; no hash/string/history snapshot or queue.
function samePathFeatures(left, right) {
  if (!left || left.features.length !== right.features.length) return false;
  for (let index = 0; index < left.features.length; index++) {
    const a = left.features[index].geometry.coordinates;
    const b = right.features[index].geometry.coordinates;
    if (a.length !== b.length) return false;
    for (let point = 0; point < a.length; point++)
      if (a[point][0] !== b[point][0] || a[point][1] !== b[point][1]) return false;
  }
  return true;
}
const qualityName = {autonomous:'일반 GNSS', dgps:'DGNSS', rtk_float:'RTK FLOAT', rtk_fixed:'RTK FIX', dead_reckoning:'추정 위치', no_fix:'위치 미확정'};
let bundlePromise;
function bundle() {
  if (!bundlePromise) bundlePromise = fetch('/api/maps/status').then(r => {
    if (!r.ok) throw new Error(`지도 상태 HTTP ${r.status}`);
    return r.json();
  }).then(info => {
    if (!info?.ok) throw new Error(info?.error || '오프라인 지도 파일 확인 필요');
    return info;
  }).catch(error => {
    // Share in-flight/successful metadata, but never cache a transient failure.
    bundlePromise = null;
    throw error;
  });
  return bundlePromise;
}

async function localStyle() {
  const theme = document.documentElement.dataset.theme === 'light' ? 'light' : 'dark';
  const style = await fetch(`/static/gps-style-${theme}.json`).then(r => r.json());
  style.sources.basemap.url = `pmtiles://${location.origin}/maps/korea.pmtiles`;
  style.glyphs = `${location.origin}/maps/fonts/{fontstack}/{range}.pbf`;
  style.sprite = `${location.origin}/maps/sprites/v4/${theme}`;
  return style;
}

class GpsMapView {
  constructor(prefix, archive=false) {
    this.prefix = prefix; this.archive = archive;
    this.model = new FollowModel(); this.map = null; this.marker = null;
    this.sample = {}; this.frames = []; this.until = Infinity; this.lastTrackData = null;
    this.initializing = null; this.lastCentered = '';
    this.retryInitAt = 0;
    this.markerMounted = false; this.lastPositionKey = null; this.lastAppearanceKey = null;
    this.lastAccuracyKey = null;
    this.paintTask = null; this.paintGeneration = 0;
    // Stop following before MapLibre's drag threshold. A new GPS frame can
    // otherwise jumpTo() and cancel the gesture before dragstart ever fires.
    const container = byId(`${prefix}Map`);
    const releaseFollow = () => { this.model.userPan(); this.renderFollow(); };
    for (const event of ['pointerdown', 'wheel', 'touchstart', 'keydown'])
      container?.addEventListener(event, releaseFollow, {capture:true, passive:true});
    byId(`${prefix}Follow`)?.addEventListener('click', () => {
      const center = this.model.recenter();
      if (center && this.map) this.map.jumpTo({center});
      this.renderFollow();
    });
    this.renderFollow();
  }
  renderFollow() {
    const button = byId(`${this.prefix}Follow`);
    if (!button) return;
    button.setAttribute('aria-pressed', String(this.model.follow));
    button.classList.toggle('active', this.model.follow);
    button.textContent = this.model.follow ? '로버 중심 추적 중' : '로버 중심으로';
  }
  // The live map starts when its box comes within 200 px of the screen (NET-03): a page opened on a phone, or with
  // the map below the visible part, does not download the map library and label font until the map is wanted.
  initWhenVisible() {
    const container = byId(`${this.prefix}Map`);
    if (this.map || this.initializing || !container || typeof IntersectionObserver !== 'function') return this.init();
    if (this.visibility) return;
    this.visibility = new IntersectionObserver(entries => {
      if (!entries.some(entry => entry.isIntersecting)) return;
      this.visibility.disconnect(); this.visibility = null;
      this.init();
    }, {rootMargin: '200px'});
    this.visibility.observe(container);
  }
  async init() {
    if (this.map) return; // ResizeObserver handles layout changes, not each GPS frame.
    if (this.initializing) return this.initializing;
    if (performance.now() < this.retryInitAt) return;
    this.initializing = this.create().catch(error => {
      text(`${this.prefix}MapStatus`, `지도를 불러오지 못했습니다 · ${error.message}`);
      this.retryInitAt = performance.now() + 2000;
      this.initializing = null;
    });
    return this.initializing;
  }
  async create() {
    const info = await bundle();
    if (!info.ok) throw new Error(info.error || '오프라인 지도 파일 확인 필요');
    const maplibregl = await mapLibrary();
    const font = new FontFace('UBTS Map Korean', 'url(/maps/fonts/korean.woff2)', {weight:'400'});
    document.fonts.add(await font.load());
    const style = await localStyle();
    this.map = new maplibregl.Map({container:`${this.prefix}Map`, style,
      center:[129.2, 35.55], zoom:11, minZoom:5, maxZoom:20,
      maxBounds:[[123.8,32.5],[132.3,39.1]],
      localIdeographFontFamily:'UBTS Map Korean',
      dragRotate:false, pitchWithRotate:false, touchPitch:false,
      attributionControl:true, fadeDuration:0});
    this.map.addControl(new maplibregl.NavigationControl({showCompass:false}), 'top-right');
    this.map.addControl(new maplibregl.ScaleControl({unit:'metric'}), 'bottom-left');
    this.map.on('dragstart', () => { this.model.userPan(); this.renderFollow(); });
    this.map.on('boxzoomstart', () => { this.model.userPan(); this.renderFollow(); });
    this.map.on('style.load', () => { this.addOverlays(); this.requestPaint(); });
    this.map.on('error', event => {
      text(`${this.prefix}MapStatus`, `지도 일부를 표시하지 못했습니다 · ${event.error?.message || '파일 확인 필요'}`);
    });
    const element = document.createElement('div'); element.className = 'gps-rover-marker';
    element.innerHTML = '<span class="gps-marker-arrow"></span><span class="gps-marker-dot"></span>';
    element.setAttribute('role', 'img'); element.setAttribute('aria-label', '로버 GPS 위치');
    this.marker = new maplibregl.Marker({element, anchor:'center'});
    this.markerElement = element;
    this.resizeObserver = new ResizeObserver(() => this.map?.resize());
    this.resizeObserver.observe(byId(`${this.prefix}Map`));
    text(`${this.prefix}MapStatus`, `오프라인 지도 · 대한민국 전역 · ${info.data_date || '날짜 확인 중'} · ${(info.bytes/1048576).toFixed(0)} MiB`);
  }
  addOverlays() {
    for (const id of ['gps-accuracy', 'gps-track']) this.map.addSource(id, {type:'geojson', data:empty()});
    this.map.addLayer({id:'gps-accuracy-fill', type:'fill', source:'gps-accuracy', paint:{'fill-color':'#20b5cf','fill-opacity':.15}});
    this.map.addLayer({id:'gps-accuracy-line', type:'line', source:'gps-accuracy', paint:{'line-color':'#20b5cf','line-width':1}});
    this.map.addLayer({id:'gps-track-line', type:'line', source:'gps-track', paint:{'line-color':'#00b4cf','line-width':3,'line-opacity':.8}});
    this.lastTrackData = null; this.lastAccuracyKey = null;
  }
  update(sample, frames, until=Infinity, receptionElapsedS=0) {
    this.sample = sample || {}; this.frames = frames || []; this.until = until;
    this.model.update(this.sample);
    this.requestPaint();
    const sampleValid = validFix(this.sample);
    text(`${this.prefix}Position`, sampleValid ? `${number(this.sample.latitude, 7)}, ${number(this.sample.longitude, 7)}` : this.model.position ? `마지막 위치 ${number(this.model.position[1], 7)}, ${number(this.model.position[0], 7)}` : 'GPS 위치 수신 대기');
    text(`${this.prefix}Quality`, sampleValid ? `${qualityName[this.sample.fix_type] || 'GNSS'} · ${usedSatellitesText(this.sample)}` : this.sample.state === 'stale' ? '수신 끊김 · 마지막 위치' : this.sample.state === 'no_fix' ? `위치 미확정 · ${usedSatellitesText(this.sample)}` : 'GPS 데이터 없음');
    text(`${this.prefix}Signals`, receptionText(this.sample, receptionElapsedS));
    text(`${this.prefix}Accuracy`, sampleValid && this.sample.h_acc_m != null ? `수평 ±${number(this.sample.h_acc_m, 3)} m` : '정확도 미보고');
    text(`${this.prefix}Speed`, sampleValid && this.sample.speed_mps != null ? `${number(this.sample.speed_mps*3.6, 1)} km/h` : '— km/h');
    text(`${this.prefix}Altitude`, sampleValid ? `해발 ${number(this.sample.altitude_msl_m, 1)} m · HDOP ${number(this.sample.hdop, 1)}` : '현재 위치가 확정되면 표시됩니다.');
    const badge = byId(`${this.prefix}FixState`);
    if (badge) { badge.dataset.state = sampleValid ? 'fix' : 'waiting'; badge.textContent = sampleValid ? qualityName[this.sample.fix_type] || 'GNSS' : '수신 대기'; }
  }
  requestPaint() {
    if (!this.map || !this.marker || this.paintTask) return;
    // One pending paint per view. Every callback reads the newest desired GPS
    // state; raw fixes/readouts and replay time are never queued or rewritten.
    const task = {generation:this.paintGeneration, raf:null, timer:null};
    this.paintTask = task;
    const run = () => {
      if (this.paintTask !== task) return;
      this.paintTask = null;
      if (task.raf !== null) cancelAnimationFrame(task.raf);
      if (task.timer !== null) clearTimeout(task.timer);
      if (task.generation === this.paintGeneration) this.paint();
    };
    task.raf = requestAnimationFrame(run);
    task.timer = setTimeout(run, 100);
  }
  cancelPaint() {
    this.paintGeneration++;
    const task = this.paintTask; this.paintTask = null;
    if (task?.raf !== null && task?.raf !== undefined) cancelAnimationFrame(task.raf);
    if (task?.timer !== null && task?.timer !== undefined) clearTimeout(task.timer);
  }
  paint() {
    if (!this.map || !this.marker) return;
    const point = this.model.position;
    if (point) {
      const positionKey = point.join(',');
      if (!this.markerMounted || positionKey !== this.lastPositionKey) this.marker.setLngLat(point);
      if (!this.markerMounted) { this.marker.addTo(this.map); this.markerMounted = true; }
      this.lastPositionKey = positionKey;
      const heading = this.model.fresh && this.sample.speed_mps > .3 && Number.isFinite(this.sample.course_deg);
      const appearanceKey = `${this.model.fresh}:${heading}:${heading ? this.sample.course_deg : 0}`;
      if (appearanceKey !== this.lastAppearanceKey) {
        this.markerElement.classList.toggle('stale', !this.model.fresh);
        this.markerElement.classList.toggle('has-course', heading);
        this.markerElement.style.setProperty('--gps-course', `${heading ? this.sample.course_deg : 0}deg`);
        this.lastAppearanceKey = appearanceKey;
      }
      const centerKey = positionKey;
      // Repeated status frames must not cancel a user's drag/zoom animation.
      if (this.model.follow && this.lastCentered !== centerKey) this.map.jumpTo({center:point});
      this.lastCentered = centerKey;
    } else {
      if (this.markerMounted) this.marker.remove();
      this.markerMounted = false; this.lastPositionKey = null; this.lastAppearanceKey = null;
    }
    const accuracy = this.map.getSource('gps-accuracy');
    const meters = this.model.fresh ? this.sample.h_acc_m : null;
    const accuracyKey = point && Number.isFinite(meters) && meters > 0 && meters <= 100000
      ? `${point.join(',')}:${meters}` : 'empty';
    if (accuracy && accuracyKey !== this.lastAccuracyKey) {
      accuracy.setData(accuracyCircle(point, meters)); this.lastAccuracyKey = accuracyKey;
    }
    const track = this.map.getSource('gps-track');
    if (track) {
      const data = pathFeatures(this.frames, this.until);
      if (!samePathFeatures(this.lastTrackData, data)) {
        track.setData(data); this.lastTrackData = data;
      }
    }
  }
  reset() {
    this.cancelPaint();
    this.model = new FollowModel(); this.frames = [];
    if (this.markerMounted) this.marker?.remove();
    this.markerMounted = false; this.lastPositionKey = null; this.lastAppearanceKey = null;
    this.lastTrackData = null; this.lastAccuracyKey = null; this.lastCentered = '';
    this.renderFollow(); this.requestPaint();
  }
  async theme() { if (this.map) this.map.setStyle(await localStyle()); }
}

const live = new GpsMapView('roverGps');
const archive = new GpsMapView('roverArchiveGps', true);
let socket = null, reconnect = null, latest = null, receivedAt = 0, frames = [], frameKey = '', archiveId = null;
// S12: exponential reconnect backoff with jitter (1.5 s to 30 s), reset by a message.
let reconnectAttempt = 0;
function gpsReconnectDelay(attempt, random = Math.random) {
  const base = Math.min(30000, 1500 * 2 ** Math.max(0, Math.min(10, Number(attempt) || 0)));
  return Math.round(Math.min(30000, base * (0.8 + 0.4 * random())));
}
const active = () => location.pathname.replace(/\/$/, '') === '/map' && !document.hidden;   // the GPS map is on the map screen
function closeSocket() {
  clearTimeout(reconnect); reconnect = null;
  const current = socket; socket = null;
  if (current) { current.onclose = null; current.close(); }
}
function paintLive() {
  if (!latest || !active()) return;
  const age = (latest.age_s ?? 0) + (Date.now()-receivedAt)/1000;
  const sample = age > 3 ? {...latest, valid:false, state:'stale'} : latest;
  live.update(sample, frames, Infinity, Math.max(0, (Date.now()-receivedAt)/1000));
  const driver = latest.driver || {};
  const uart = driver.uart_counters;
  const errors = uart ? (uart.frame || 0)+(uart.parity || 0)+(uart.overrun || 0)+(uart.buf_overrun || 0) : 0;
  text('roverGpsConnection', driver.error || `${driver.serial_open ? 'UART 열림' : 'UART 대기'} · ${driver.baud || '—'} baud · 수신 ${driver.bytes_received ?? 0} bytes · 유효 패킷 ${driver.valid_packets ?? 0}${errors ? ` · UART 오류 ${errors}` : ''}`);
  const detection = driver.auto_baud && driver.baud_attempts?.length ? ` · ${driver.baud_attempts.length}개 통신 속도 확인` : '';
  text('roverGpsAge', latest.received_at ? `${number(age, 1)}초 전 GPS 표본` : `GPS 응답 수신 전${detection}`);
}
function connect() {
  if (!active() || socket) return;
  let profile = 'auto';
  try { profile = localStorage.getItem('ubts-live-network-profile') || 'auto'; } catch (_) {}
  if (!['detail', 'balanced', 'slow', 'tiny'].includes(profile)) {
    // Automatic: the link grade's plan (NET-10). GPS rates: balanced 5/s, slow 2/s, tiny 1/s.
    const graded = window.RoverLink?.profile?.() || '';
    profile = {detail: 'balanced', balanced: 'slow', slow: 'tiny', tiny: 'tiny'}[graded] || 'slow';
  }
  const ws = new WebSocket(`${location.protocol === 'https:' ? 'wss:' : 'ws:'}//${location.host}/ws/gps?profile=${encodeURIComponent(profile)}`);
  socket = ws;
  ws.onmessage = event => {
    if (socket !== ws) return;
    try {
      const payload = JSON.parse(event.data);
      if (payload.type !== 'gps') return;
      latest = payload; receivedAt = Date.now(); reconnectAttempt = 0;
      const key = `${payload.received_at}:${payload.sequence}`;
      if (key !== frameKey && payload.received_at) {
        frames.push({t_s:payload.received_at, sample:payload});
        if (frames.length > 6000) frames.splice(0, frames.length-6000);
        frameKey = key;
      }
      paintLive();
      if (payload.stream_seq !== undefined && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify({ack:payload.stream_seq}));
    } catch (_) { text('roverGpsConnection', 'GPS 메시지 확인 실패'); }
  };
  ws.onclose = () => { if (socket === ws) { socket = null; if (active()) reconnect = setTimeout(connect, gpsReconnectDelay(reconnectAttempt++)); } };
  ws.onerror = () => text('roverGpsConnection', 'GPS 브리지 연결 대기');
}
function routeChanged() {
  if (active()) { live.initWhenVisible(); connect(); }
  else closeSocket();
}
window.RoverGps = {
  routeChanged,
  profileChanged() { closeSocket(); connect(); },
  archive(frames, seconds, id, enabled=false) {
    const panel = byId('roverArchiveGpsPanel');
    if (!panel) return;
    panel.hidden = !enabled && !frames?.length;
    if (panel.hidden) return;
    if (archiveId !== id) { archiveId = id; archive.reset(); }
    archive.init();
    const frame = frameAt(frames || [], seconds);
    if (!frame || !validFix(frame.sample)) archive.model.position = null;
    archive.update(frame?.sample || {valid:false, state:'no_data'}, frames || [], frame?.t_s ?? Math.floor(seconds));
    text('roverArchiveGpsTime', frame ? `GPS ${number(frame.t_s, 2)}s · 재생 ${number(seconds, 2)}s` : `재생 ${number(seconds, 2)}s · 동기 GPS 표본 없음`);
  },
};
new MutationObserver(() => { live.theme().catch(console.error); archive.theme().catch(console.error); }).observe(document.documentElement, {attributes:true, attributeFilter:['data-theme']});
document.addEventListener('visibilitychange', routeChanged);
window.addEventListener('pagehide', closeSocket);
setInterval(paintLive, 500);
routeChanged();
