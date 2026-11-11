/* Explicit future SIM host<->native-video clock. Never guesses a ROS offset. */
(() => {
  'use strict';
  const SCHEMA = 'ubts.sim_video_clock.v1';
  const MAX_INDEX_BYTES = 1024 * 1024, MAX_CHUNK_BYTES = 64 * 1024;
  const MAX_POINTS = 1000000, MAX_CHUNKS = 4096, MAX_CACHE = 3, MAX_PENDING = 2;
  const HOST = 'host_wall_elapsed_unchanged', VIDEO = 'ros_camera_header_ns';
  const outside = 'unsynchronized_no_extrapolation';
  const terminal = 'native_hold_has_no_invented_host_anchor';
  const fail = reason => { throw new Error(reason); };
  const integer = value => Number.isSafeInteger(value) && value >= 0;
  const range = value => Array.isArray(value) && value.length === 2 &&
    value.every(integer) && value[0] <= value[1];
  const equal = (a, b) => a?.[0] === b?.[0] && a?.[1] === b?.[1];
  function validateIndex(value, descriptor) {
    if (!value || value.schema !== SCHEMA || value.host_domain !== HOST ||
        value.video_domain !== VIDEO || value.source_topic !== '/camera/image_raw' ||
        value.outside_policy !== outside || value.terminal_policy !== terminal ||
        value.overlap_policy !== 'one_identical_boundary_anchor' ||
        value.float_query_roundoff_tolerance?.host_ns !== .5 ||
        value.float_query_roundoff_tolerance?.video_ms !== .000001 ||
        !range(value.coverage_host_ns) || !range(value.coverage_video_ms) ||
        !integer(value.point_count) || value.point_count < 1 || value.point_count > MAX_POINTS ||
        !integer(value.chunk_points_bound) || value.chunk_points_bound < 2 || value.chunk_points_bound > 512 ||
        value.chunk_bytes_bound !== MAX_CHUNK_BYTES || !Array.isArray(value.chunks) ||
        !value.chunks.length || value.chunks.length > MAX_CHUNKS) fail('clock_index_invalid');
    if (!equal(value.coverage_host_ns, descriptor.coverage_host_ns) ||
        !equal(value.coverage_video_ms, descriptor.coverage_video_ms)) fail('clock_coverage_mismatch');
    let count = 0, previous = null;
    value.chunks.forEach((entry, index) => {
      if (!entry || entry.asset !== `clock-${String(index).padStart(5, '0')}.json` ||
          !integer(entry.point_count) || entry.point_count < 1 ||
          entry.point_count > value.chunk_points_bound ||
          (entry.point_count === 1 && value.point_count !== 1) ||
          !range(entry.host_range_ns) || !range(entry.video_range_ms) ||
          ((entry.host_range_ns[0] === entry.host_range_ns[1] ||
            entry.video_range_ms[0] === entry.video_range_ms[1]) && entry.point_count !== 1))
        fail('clock_chunk_index_invalid');
      if (previous && (previous.host_range_ns[1] !== entry.host_range_ns[0] ||
          previous.video_range_ms[1] !== entry.video_range_ms[0])) fail('clock_boundary_invalid');
      count += entry.point_count - (index ? 1 : 0); previous = entry;
    });
    if (count !== value.point_count ||
        !equal(value.coverage_host_ns, [value.chunks[0].host_range_ns[0], previous.host_range_ns[1]]) ||
        !equal(value.coverage_video_ms, [value.chunks[0].video_range_ms[0], previous.video_range_ms[1]]))
      fail('clock_point_count_invalid');
    return value;
  }
  function validateChunk(value, entry) {
    if (!value || value.schema !== SCHEMA || !Array.isArray(value.points_ns_ms) ||
        value.points_ns_ms.length !== entry.point_count || value.points_ns_ms.length > 512 ||
        !Array.isArray(value.valid_links) || value.valid_links.length !== value.points_ns_ms.length - 1 ||
        !value.valid_links.every(link => typeof link === 'boolean')) fail('clock_chunk_invalid');
    let previous = null;
    for (const point of value.points_ns_ms) {
      if (!Array.isArray(point) || point.length !== 2 || !point.every(integer) ||
          (previous && (point[0] <= previous[0] || point[1] <= previous[1])))
        fail('clock_points_invalid');
      previous = point;
    }
    if (!equal(entry.host_range_ns, [value.points_ns_ms[0][0], previous[0]]) ||
        !equal(entry.video_range_ms, [value.points_ns_ms[0][1], previous[1]]))
      fail('clock_chunk_coverage_invalid');
    return value;
  }
  function interpolate(chunk, seconds, direction) {
    if (!Number.isFinite(seconds) || !['host_to_video', 'video_to_host'].includes(direction)) return null;
    const host = direction === 'host_to_video', source = host ? 0 : 1, target = host ? 1 : 0;
    const sourceScale = host ? 1e9 : 1e3, targetScale = host ? 1e3 : 1e9;
    const tolerance = host ? .5 : .000001, query = seconds * sourceScale;
    const points = chunk.points_ns_ms;
    if (query < points[0][source] - tolerance || query > points.at(-1)[source] + tolerance) return null;
    let low = 0, high = points.length;
    while (low < high) {
      const mid = (low + high) >>> 1;
      if (points[mid][source] < query) low = mid + 1; else high = mid;
    }
    for (const index of [low, low - 1]) {
      if (index >= 0 && index < points.length && Math.abs(query - points[index][source]) <= tolerance)
        return points[index][target] / targetScale;
    }
    if (!low || low === points.length || !chunk.valid_links[low - 1]) return null;
    const a = points[low - 1], b = points[low];
    return (a[target] + (query - a[source]) / (b[source] - a[source]) * (b[target] - a[target])) / targetScale;
  }
  async function boundedJson(response, limit, signal) {
    if (!response.ok) fail(`clock_http_${response.status}`);
    if (!/^application\/json(?:\s*;|$)/i.test(response.headers.get('content-type') || ''))
      fail('clock_content_type_invalid');
    const stated = response.headers.get('content-length');
    if (stated && (!/^\d+$/.test(stated) || Number(stated) > limit)) fail('clock_response_too_large');
    if (!response.body?.getReader) fail('clock_response_not_streamable');
    const reader = response.body.getReader(), chunks = []; let bytes = 0;
    const abort = () => { reader.cancel().catch(() => {}); };
    signal.addEventListener('abort', abort, {once:true});
    try {
      while (true) {
        if (signal.aborted) { await reader.cancel(); fail('clock_request_aborted'); }
        const {value, done} = await reader.read();
        if (signal.aborted) fail('clock_request_aborted');
        if (done) break;
        bytes += value.byteLength;
        if (bytes > limit) { await reader.cancel(); fail('clock_response_too_large'); }
        chunks.push(value);
      }
    } finally { signal.removeEventListener('abort', abort); reader.releaseLock(); }
    const data = new Uint8Array(bytes); let offset = 0;
    for (const value of chunks) { data.set(value, offset); offset += value.byteLength; }
    try { return JSON.parse(new TextDecoder('utf-8', {fatal:true}).decode(data)); }
    catch (_) { fail('clock_json_invalid'); }
  }
  class Clock {
    constructor(adaptive, options = {}) {
      const descriptor = adaptive?.playback_clock;
      if (!descriptor || descriptor.schema !== SCHEMA || descriptor.host_domain !== HOST ||
          descriptor.video_domain !== VIDEO || descriptor.synchronization !== 'piecewise_exact_native_header_anchors' ||
          !range(descriptor.coverage_host_ns) || !range(descriptor.coverage_video_ms))
        fail('clock_descriptor_invalid');
      const origin = new URL(options.baseURL || location.href);
      const base = new URL(adaptive.clock_asset_base_url, origin), index = new URL(adaptive.clock_index_url, origin);
      if (!adaptive.clock_asset_base_url || !adaptive.clock_index_url ||
          !/^https?:$/.test(base.protocol) || base.origin !== origin.origin || index.origin !== origin.origin ||
          base.search || base.hash || !base.pathname.endsWith('/adaptive-video/') ||
          index.pathname !== base.pathname + 'clock-index.json' || index.hash ||
          !/^[a-f0-9]{16}$/.test(adaptive.clock_asset_version || '') ||
          index.searchParams.get('v') !== adaptive.clock_asset_version) fail('clock_asset_url_invalid');
      this.descriptor = descriptor; this.indexURL = index.href; this.assetBase = base;
      this.version = adaptive.clock_asset_version; this.fetch = options.fetch || window.fetch.bind(window);
      this.onState = options.onState || (() => {}); this.schedule = options.schedule || queueMicrotask;
      this.index = null; this.indexRequest = null; this.pending = new Map(); this.cache = new Map();
      this.generation = 0; this.dead = false; this.fatal = false; this.retryAfter = 0;
      this.status = 'loading'; this.reason = 'clock_index_loading'; this.notification = false;
    }
    state() { return {status:this.status, reason:this.reason, generation:this.generation,
      cache_chunks:this.cache.size, pending_requests:this.pending.size + Number(!!this.indexRequest),
      index_loaded:!!this.index, synchronized:this.status === 'synchronized'}; }
    setState(status, reason) {
      if (status === this.status && reason === this.reason) return;
      this.status = status; this.reason = reason;
      if (!this.notification && !this.dead) {
        this.notification = true;
        this.schedule(() => { this.notification = false; if (!this.dead) this.onState(this.state()); });
      }
    }
    cancelPending() {
      this.generation++;
      this.indexRequest?.controller.abort(); this.indexRequest = null;
      for (const request of this.pending.values()) request.controller.abort();
      this.pending.clear();
    }
    destroy() { this.dead = true; this.cancelPending(); this.cache.clear(); this.index = null; }
    async request(url, controller, limit) {
      const timeout = setTimeout(() => controller.abort(), 10000);
      try {
        const response = await this.fetch(url, {method:'GET', credentials:'same-origin',
          cache:'force-cache', signal:controller.signal});
        if (controller.signal.aborted) {
          await response.body?.cancel?.(); fail('clock_request_aborted');
        }
        return await boundedJson(response, limit, controller.signal);
      }
      finally { clearTimeout(timeout); }
    }
    error(error, generation) {
      if (this.dead || generation !== this.generation) return;
      const reason = String(error?.message || error);
      this.fatal = /clock_(?:descriptor|asset_url|index_invalid|coverage|chunk_|boundary|point_count|points_|response_too_large|content_type|json_invalid)/.test(reason);
      if (this.fatal) this.cancelPending();
      this.retryAfter = Date.now() + 1000;
      this.setState(this.fatal ? 'invalid' : 'unavailable', reason);
    }
    async ready() {
      if (this.dead || this.fatal) return null;
      if (this.index) return this.index;
      if (this.indexRequest) return this.indexRequest.promise;
      if (Date.now() < this.retryAfter) return null;
      const request = {controller:new AbortController(), generation:this.generation};
      this.setState('loading', 'clock_index_loading'); this.indexRequest = request;
      request.promise = this.request(this.indexURL, request.controller, MAX_INDEX_BYTES).then(value => {
        if (this.dead || this.indexRequest !== request || request.generation !== this.generation) return null;
        this.index = validateIndex(value, this.descriptor); return this.index;
      }).catch(error => { this.error(error, request.generation); return null; })
        .finally(() => { if (this.indexRequest === request) this.indexRequest = null; });
      return request.promise;
    }
    entry(seconds, direction) {
      if (!this.index || !Number.isFinite(seconds) || !['host_to_video','video_to_host'].includes(direction)) return null;
      const host = direction === 'host_to_video', field = host ? 'host_range_ns' : 'video_range_ms';
      const value = seconds * (host ? 1e9 : 1e3), tolerance = host ? .5 : .000001;
      const chunks = this.index.chunks;
      if (value < chunks[0][field][0] - tolerance || value > chunks.at(-1)[field][1] + tolerance) return null;
      let low = 0, high = chunks.length;
      while (low < high) {
        const mid = (low + high) >>> 1;
        if (chunks[mid][field][1] < value - tolerance) low = mid + 1; else high = mid;
      }
      return chunks[low] || null;
    }
    getCached(entry) {
      const value = this.cache.get(entry.asset);
      if (value) { this.cache.delete(entry.asset); this.cache.set(entry.asset, value); }
      return value;
    }
    async loadChunk(entry) {
      if (this.dead || this.fatal) return null;
      const cached = this.getCached(entry); if (cached) return cached;
      if (this.pending.has(entry.asset)) return this.pending.get(entry.asset).promise;
      if (Date.now() < this.retryAfter) return null;
      while (this.pending.size >= MAX_PENDING) {
        const [key, old] = this.pending.entries().next().value;
        old.controller.abort(); this.pending.delete(key);
      }
      const url = new URL(entry.asset, this.assetBase); url.searchParams.set('v', this.version);
      const request = {controller:new AbortController(), generation:this.generation};
      this.pending.set(entry.asset, request); this.setState('loading', 'clock_chunk_loading');
      request.promise = this.request(url.href, request.controller, MAX_CHUNK_BYTES).then(value => {
        if (this.dead || this.pending.get(entry.asset) !== request || request.generation !== this.generation) return null;
        const chunk = validateChunk(value, entry); this.cache.set(entry.asset, chunk);
        while (this.cache.size > MAX_CACHE) this.cache.delete(this.cache.keys().next().value);
        return chunk;
      }).catch(error => {
        if (this.pending.get(entry.asset) === request) this.error(error, request.generation);
        return null;
      }).finally(() => { if (this.pending.get(entry.asset) === request) this.pending.delete(entry.asset); });
      return request.promise;
    }
    mapCached(seconds, direction) {
      if (this.dead || this.fatal) return null;
      const entry = this.entry(seconds, direction);
      if (!entry) { if (this.index) this.setState('unsynchronized', 'clock_outside_coverage'); return null; }
      const chunk = this.getCached(entry); if (!chunk) return null;
      const value = interpolate(chunk, seconds, direction);
      this.setState(value === null ? 'unsynchronized' : 'synchronized', value === null ? 'clock_gap' : null);
      return value;
    }
    async ensure(seconds, direction) {
      if (!Number.isFinite(seconds) || !['host_to_video','video_to_host'].includes(direction)) {
        this.setState('unsynchronized', 'clock_query_invalid'); return null;
      }
      const generation = this.generation;
      if (!await this.ready() || this.dead || generation !== this.generation) return null;
      const entry = this.entry(seconds, direction);
      if (!entry) { this.setState('unsynchronized', 'clock_outside_coverage'); return null; }
      const chunk = await this.loadChunk(entry);
      if (!chunk || this.dead || generation !== this.generation) return null;
      const value = interpolate(chunk, seconds, direction);
      this.setState(value === null ? 'unsynchronized' : 'synchronized', value === null ? 'clock_gap' : null);
      return value;
    }
  }
  window.ArchiveVideoClock = {Clock, validateIndex, validateChunk, interpolate, SCHEMA};
})();
