/* Local, offline-capable adaptive VOD player. Sensor time follows video time. */
class ArchiveVideoPlayer {
  constructor(video, select, adaptive, fallback, notify = () => {}) {
    this.video = video; this.select = select; this.adaptive = adaptive;
    this.fallback = fallback; this.notify = notify; this.hls = null;
    this.dead = false; this.recovered = false; this.mode = "mp4";
    this.onQuality = () => this.choose(select.value);
    select?.addEventListener("change", this.onQuality);
    if (select) { select.hidden = !adaptive; select.replaceChildren(); }
    if (adaptive && window.Hls?.isSupported()) {
      this.mode = "adaptive";
      this.hls = new window.Hls({ maxBufferLength: 24, maxMaxBufferLength: 48,
        backBufferLength: 12, startLevel: 0, abrEwmaDefaultEstimate: 1500000, capLevelToPlayerSize: false });
      this.hls.on(window.Hls.Events.MANIFEST_PARSED, (_, data) => {
        if (this.dead) return;
        this.options(data.levels);
        this.notify({ mode: this.mode, auto: true });
      });
      this.hls.on(window.Hls.Events.LEVEL_SWITCHED, (_, data) => {
        if (this.dead) return;
        this.notify({ mode: this.mode, height: this.hls.levels[data.level]?.height,
          auto: this.hls.autoLevelEnabled });
      });
      this.hls.on(window.Hls.Events.ERROR, (_, data) => {
        if (this.dead || !data.fatal) return;
        if (data.type === window.Hls.ErrorTypes.MEDIA_ERROR && !this.recovered) {
          this.recovered = true; this.hls.recoverMediaError();
        } else this.useFallback();
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
    this.dead = true; this.hls?.destroy(); this.hls = null;
    this.select?.removeEventListener("change", this.onQuality);
  }
}
window.ArchiveVideoPlayer = ArchiveVideoPlayer;
