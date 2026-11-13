/* Local, offline-capable adaptive VOD player. Sensor time follows video time. */
// D1 (S13): a fatal HLS network error is retried with backoff (1, 2, 4, 8,
// then every 15 s) instead of switching for the rest of the session to the
// heavy 1080p native.mp4. On slow links (profile slow/tiny) the mp4 is never
// chosen automatically. Media errors: recover, then swap audio codec, then
// (fast links only) mp4.
// D2: quality is capped to the player size; slow links start at the lowest
// level, cap automatic quality at 360p and buffer further ahead.
const HLS_RETRY_DELAYS_MS = [1000, 2000, 4000, 8000, 15000];
const HLS_FAST_LINK_RETRIES = HLS_RETRY_DELAYS_MS.length;
function archiveVideoLinkIsSlow(profile) {
  return profile === "slow" || profile === "tiny";
}
function archiveVideoHlsConfig(slow) {
  const base = { capLevelToPlayerSize: true, maxMaxBufferLength: slow ? 120 : 48 };
  if (!slow) return { ...base, maxBufferLength: 24, backBufferLength: 12, startLevel: -1, abrEwmaDefaultEstimate: 5000000 };
  const retry = { maxNumRetry: 6, retryDelayMs: 1000, maxRetryDelayMs: 15000 };
  return { ...base, maxBufferLength: 60, backBufferLength: 30, startLevel: 0, abrEwmaDefaultEstimate: 600000,
    fragLoadPolicy: { default: { maxTimeToFirstByteMs: 20000, maxLoadTimeMs: 120000, timeoutRetry: retry, errorRetry: retry } },
    playlistLoadPolicy: { default: { maxTimeToFirstByteMs: 15000, maxLoadTimeMs: 30000, timeoutRetry: retry, errorRetry: retry } } };
}
function archiveVideoLevelCap(levels, maxHeight = 360) {
  let cap = -1;
  (levels || []).forEach((level, index) => { if (Number(level?.height) <= maxHeight) cap = index; });
  return cap >= 0 ? cap : 0;
}
// The HLS library (about 190 kB compressed) is needed only for recorded video, so the page does not load it;
// the first player that needs it adds the script once (NET-03). The server adds the version to this address.
const HLS_LIBRARY_URL = "/static/vendor/hls/hls.min.js";
let hlsLibraryLoading = null;
function loadHlsLibrary(doc = document) {
  if (window.Hls) return Promise.resolve(window.Hls);
  if (!hlsLibraryLoading) {
    hlsLibraryLoading = new Promise((resolve, reject) => {
      const script = doc.createElement("script");
      script.src = HLS_LIBRARY_URL; script.async = true;
      script.onload = () => (window.Hls ? resolve(window.Hls) : reject(new Error("hls_library_missing")));
      script.onerror = () => reject(new Error("hls_library_unavailable"));
      doc.head.append(script);
    }).catch(error => { hlsLibraryLoading = null; throw error; });
  }
  return hlsLibraryLoading;
}
class ArchiveVideoPlayer {
  constructor(video, select, adaptive, fallback, notify = () => {}, options = {}) {
    this.video = video; this.select = select; this.adaptive = adaptive;
    this.fallback = fallback; this.notify = notify; this.hls = null;
    this.dead = false; this.recovered = false; this.mode = "mp4";
    this.mediaErrors = 0; this.networkRetries = 0; this.retryTimer = null;
    this.timers = options.timers || { set: (f, ms) => setTimeout(f, ms), clear: id => clearTimeout(id) };
    const profile = options.profile ?? window.RoverLink?.profile?.();
    this.slowLink = archiveVideoLinkIsSlow(profile);
    this.onQuality = () => this.choose(select.value);
    select?.addEventListener("change", this.onQuality);
    if (select) { select.hidden = !adaptive; select.replaceChildren(); }
    const loader = options.loadHls ?? (typeof document !== "undefined" ? loadHlsLibrary : null);
    if (adaptive && !window.Hls && loader) {
      // Load the library first; if it cannot be loaded, start as if it were missing (native HLS or the mp4).
      this.mode = "loading";
      this.notify({ mode: this.mode, loading: true });
      const begin = () => { if (!this.dead) this.begin(video, adaptive, fallback); };
      loader().then(begin, begin);
    } else this.begin(video, adaptive, fallback);
  }
  begin(video, adaptive, fallback) {
    this.mode = "mp4";
    if (adaptive && window.Hls?.isSupported()) {
      this.mode = "adaptive";
      this.hls = new window.Hls(archiveVideoHlsConfig(this.slowLink));
      this.hls.on(window.Hls.Events.MANIFEST_PARSED, (_, data) => {
        if (this.dead) return;
        this.options(data.levels);
        if (this.slowLink) this.hls.autoLevelCapping = archiveVideoLevelCap(data.levels, 360);
        this.notify({ mode: this.mode, auto: true });
      });
      this.hls.on(window.Hls.Events.LEVEL_SWITCHED, (_, data) => {
        if (this.dead) return;
        this.notify({ mode: this.mode, height: this.hls.levels[data.level]?.height,
          auto: this.hls.autoLevelEnabled });
      });
      this.hls.on(window.Hls.Events.FRAG_LOADED, () => {
        if (this.dead || !this.networkRetries) return;
        this.networkRetries = 0;
        this.notify({ mode: this.mode, retrying: false });
      });
      this.hls.on(window.Hls.Events.ERROR, (_, data) => {
        if (this.dead || !data.fatal) return;
        if (data.type === window.Hls.ErrorTypes.NETWORK_ERROR) this.retryNetwork(data);
        else if (data.type === window.Hls.ErrorTypes.MEDIA_ERROR) this.recoverMedia();
        else if (!this.slowLink) this.useFallback();
        else this.notify({ mode: this.mode, error: data.details || "hls_error" });
      });
      this.hls.loadSource(adaptive.manifest_url); this.hls.attachMedia(video);
    } else if (adaptive && video.canPlayType("application/vnd.apple.mpegurl")) {
      this.mode = "native-hls";
      this.options([{ name: "자동", height: null, url: adaptive.manifest_url },
        ...adaptive.variants.map(v => ({ height: v.height,
          url: adaptive.manifest_url.split("master.m3u8")[0] + v.playlist }))], true);
      video.src = adaptive.manifest_url; video.load();
    } else { video.src = fallback; video.load(); }
  }
  retryNetwork(data) {
    if (!this.slowLink && this.networkRetries >= HLS_FAST_LINK_RETRIES) { this.useFallback(); return; }
    const delay = HLS_RETRY_DELAYS_MS[Math.min(this.networkRetries, HLS_RETRY_DELAYS_MS.length - 1)];
    this.networkRetries += 1;
    this.notify({ mode: this.mode, retrying: true, attempt: this.networkRetries, delay_ms: delay });
    if (this.retryTimer !== null) this.timers.clear(this.retryTimer);
    this.retryTimer = this.timers.set(() => {
      this.retryTimer = null;
      if (this.dead || !this.hls) return;
      // A failed playlist needs the manifest again; segments resume loading.
      if (String(data?.details || "").toLowerCase().includes("manifest")) this.hls.loadSource(this.adaptive.manifest_url);
      else this.hls.startLoad();
    }, delay);
  }
  recoverMedia() {
    this.mediaErrors += 1;
    if (this.mediaErrors === 1) { this.recovered = true; this.hls.recoverMediaError(); return; }
    if (this.mediaErrors === 2) { this.hls.swapAudioCodec?.(); this.hls.recoverMediaError(); return; }
    if (!this.slowLink) { this.useFallback(); return; }
    this.hls.recoverMediaError();
    this.notify({ mode: this.mode, error: "media_error" });
  }
  options(levels, native = false) {
    this.nativeLevels = native ? levels : null;
    if (!this.select) return;
    this.select.replaceChildren();
    const add = (value, label) => { const option = document.createElement("option");
      option.value = String(value); option.textContent = label; this.select.append(option); };
    add(-1, "화질 자동");
    levels.forEach((v,i) => { if (v.height) add(i, `${v.height}p`); });
    this.select.value = "-1";
  }
  choose(value) {
    const index = Number(value);
    if (!Number.isInteger(index) || index < -1) return;
    if (this.hls && (index === -1 || index < this.hls.levels.length)) {
      // A manual choice is honoured; the 360p automatic cap applies to "자동" only.
      this.hls.nextLevel = index;
      this.notify({ mode: this.mode, auto: index === -1,
        height: index >= 0 ? this.hls.levels[index].height : null });
    } else if (this.nativeLevels) {
      const level = index === -1 ? this.nativeLevels[0] : this.nativeLevels[index];
      if (!level) return;
      const time = this.video.currentTime; const playing = !this.video.paused;
      this.video.src = level.url; this.video.load();
      this.video.addEventListener("loadedmetadata", () => {
        if (this.dead) return;
        this.video.currentTime = time; if (playing) this.video.play().catch(() => {});
      }, { once: true });
      this.notify({ mode: this.mode, height: level.height, auto: index === -1 });
    }
  }
  useFallback() {
    if (this.dead || this.mode === "mp4") return;
    if (this.retryTimer !== null) { this.timers.clear(this.retryTimer); this.retryTimer = null; }
    this.hls?.destroy(); this.hls = null; this.mode = "mp4";
    if (this.select) this.select.hidden = true;
    const time = this.video.currentTime; const playing = !this.video.paused;
    this.video.src = this.fallback; this.video.load();
    this.video.addEventListener("loadedmetadata", () => {
      if (this.dead) return;
      this.video.currentTime = time; if (playing) this.video.play().catch(() => {});
    }, { once: true });
    this.notify({ mode: "mp4", fallback: true });
  }
  destroy() {
    this.dead = true;
    if (this.retryTimer !== null) { this.timers.clear(this.retryTimer); this.retryTimer = null; }
    this.hls?.destroy(); this.hls = null;
    this.select?.removeEventListener("change", this.onQuality);
  }
}
window.ArchiveVideoPlayer = ArchiveVideoPlayer;
