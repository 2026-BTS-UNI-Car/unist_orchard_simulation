
function text(id, value) {
  const el = $(id);
  if (!el) return;
  const full = String(value);
  // data-shorten="N": a long value without spaces (device path, host name) is
  // cut in the middle to N characters; the title keeps the whole value.
  const limit = Number(el.dataset?.shorten) || 0;
  const next = limit ? shortenMiddle(full, limit) : full;
  if (el.textContent !== next) el.textContent = next;
  if (limit) {
    if (next === full) el.removeAttribute("title");
    else if (el.getAttribute("title") !== full) el.setAttribute("title", full);
  }
}

function shortenMiddle(value, limit) {
  if (value.length <= limit) return value;
  const kept = Math.max(2, limit - 1);
  return `${value.slice(0, Math.ceil(kept / 2))}\u2026${value.slice(-Math.floor(kept / 2))}`;
}

function fmtPercent(value, digits = 1) {
  return value === null || value === undefined ? "--" : `${Number(value).toFixed(digits)}%`;
}

function fmtBytes(value) {
  if (value === null || value === undefined) return "--";
  const units = ["B", "KB", "MB", "GB", "TB"];
  let n = Number(value);
  let i = 0;
  while (n >= 1024 && i < units.length - 1) {
    n /= 1024;
    i += 1;
  }
  return `${n.toFixed(i === 0 ? 0 : 1)} ${units[i]}`;
}

function fmtBps(value) {
  if (value === null || value === undefined) return "--";
  return `${fmtBytes(value)}/s`;
}

function fmtTemp(value) {
  return value === null || value === undefined ? "--" : `${Number(value).toFixed(1)}°C`;
}

function fmtPower(value) {
  return value === null || value === undefined ? "--" : `${Number(value).toFixed(2)} W`;
}

function fmtNumber(value, digits = 1) {
  return value === null || value === undefined || Number.isNaN(Number(value)) ? "--" : Number(value).toFixed(digits);
}

function fmtDuration(seconds) {
  if (seconds === null || seconds === undefined) return "--";
  const s = Math.max(0, Number(seconds));
  const d = Math.floor(s / 86400);
  const h = Math.floor((s % 86400) / 3600);
  const m = Math.floor((s % 3600) / 60);
  return d > 0 ? `${d}d ${h}h` : `${h}h ${m}m`;
}

function fmtClock(seconds) {
  const total = Math.max(0, Math.floor(Number(seconds) || 0));
  const h = Math.floor(total / 3600);
  const m = Math.floor((total % 3600) / 60);
  const s = total % 60;
  const pad = (value) => String(value).padStart(2, "0");
  return h > 0 ? `${h}:${pad(m)}:${pad(s)}` : `${pad(m)}:${pad(s)}`;
}

function memoryPercent(memory) {
  return memory?.display_percent ?? memory?.percent;
}

function memoryTotal(memory) {
  return memory?.display_total ?? memory?.total;
}

function memoryLabel(memory) {
  return memory?.display_label || `${fmtBytes(memoryTotal(memory))} 기준`;
}

function cpuLabel(core) {
  if (core?.label) return core.label;
  const name = core?.name || "";
  if (name.startsWith("cpu") && /^\d+$/.test(name.slice(3))) return `CPU ${Number(name.slice(3)) + 1}`;
  return name.toUpperCase() || "CPU";
}

function coreKey(core) {
  return core?.name || cpuLabel(core);
}

function debugLog(scope, message, detail) {
  if (!window.console?.debug) return;
  if (detail === undefined) {
    console.debug(`[UBTS:${scope}] ${message}`);
  } else {
    console.debug(`[UBTS:${scope}] ${message}`, detail);
  }
}

async function getJson(path, options = {}) {
  const started = performance.now();
  try {
    const res = await fetch(path, { cache: "no-store", signal: options.signal });
    const ms = Math.round(performance.now() - started);
    state.apiLogCounter += 1;
    if (path !== "/api/telemetry" || state.apiLogCounter % 10 === 0) {
      debugLog("api", `${path} -> ${res.status} (${ms}ms)`);
    }
    if (!res.ok) throw new Error(`${path}: ${res.status}`);
    return res.json();
  } catch (err) {
    debugLog("api", `${path} failed`, err);
    throw err;
  }
}

// A refused request: the same message as before ("<path>: <status>"), and the reply body when it is JSON
// (err.status, err.body: the autonomy panel shows the executor's own refusal, 2026-11-19).
async function httpError(path, res) {
  const err = new Error(`${path}: ${res.status}`);
  err.status = res.status;
  try { err.body = typeof res.json === "function" ? await res.json() : null; } catch (_) { err.body = null; }
  return err;
}

async function postJson(path, payload = {}) {
  const res = await fetch(path, {
    method: "POST",
    cache: "no-store",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload || {}),
  });
  if (!res.ok) throw await httpError(path, res);
  return res.json();
}

// ---- Request transport (NET-05) --------------------------------------------
// Control and STOP go through controlRequest(): high priority, never queued
// behind other requests, with a time limit. This is the one transport function
// for driving requests; other control screens use it too.
//
// Every other GET to /api/ passes a small limiter so that status polls cannot
// take all of the browser's connections to the dashboard (HTTP/1.1 allows six
// per host): at most 4 at a time, 2 while the control lease is held or the link
// is slow. Bulk requests (recording list, archive data, container details, media
// jobs, documents) go one at a time and only when no status request waits while
// the lease is held. User actions (POST) are not queued. The server keeps
// connections open (keep-alive), so a request does not pay a new connection.
//
// Status polls are also conditional (NET-06): the page keeps the last small JSON
// answer of each address with its ETag and names it in If-None-Match; an answer
// that has not changed comes back as "304" without a body and the caller gets
// the kept copy, exactly as if the server had sent it again.
const CONTROL_TIMEOUT_MS = { stop: 8000, other: 5000 };
const REQUEST_LIMIT = { normal: 4, constrained: 2 };
// 2026-11-19 field (phone): a queued request that never answered (a dropped connection) kept its slot for good,
// the status polls then failed behind it and the bar stayed '연결 끊김' until a reload. Every queued request now
// gives its slot back after this long (the caller sees an abort, as for its own timeout).
const REQUEST_TIMEOUT_MS = { status: 12000, bulk: 90000 };
const BULK_REQUEST = /^\/api\/(?:recordings(?:$|\/(?!status$))|live\/recordings\/|live\/mapping-jobs|docker\/container\/|docs|plan$|site-map$|maps\/status$)/;
const requestQueue = { active: 0, bulkActive: 0, waiting: [] };
const CONDITIONAL_CACHE = { entries: new Map(), maxEntries: 24, maxBytes: 65536 };
const pageFetch = window.fetch.bind(window);

async function controlRequest(path, payload = {}, options = {}) {
  const op = String(path).split("/").pop();
  const timeoutMs = Number(options.timeoutMs) > 0 ? Number(options.timeoutMs)
    : op === "stop" ? CONTROL_TIMEOUT_MS.stop : CONTROL_TIMEOUT_MS.other;
  const controller = new AbortController();
  const timer = setTimeout(() => controller.abort(), timeoutMs);
  const started = performance.now();
  try {
    const res = await pageFetch(path, {
      method: "POST",
      cache: "no-store",
      priority: "high",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload || {}),
      signal: controller.signal,
    });
    // The control round trip is also a link sample (NET-10), without the server's own time.
    window.LinkGrade?.note?.(performance.now() - started - serverTimeMs(res), res.ok, "control");
    if (!res.ok) throw await httpError(path, res);
    return await res.json();
  } catch (err) {
    if (err?.name === "AbortError") throw new Error(`${path}: ${Math.round(timeoutMs / 1000)}초 동안 응답이 없었습니다.`);
    throw err;
  } finally {
    clearTimeout(timer);
  }
}

function requestPathOf(input) {
  try {
    const url = new URL(typeof input === "string" || input instanceof URL ? String(input) : input.url, location.href);
    return url.origin === location.origin ? url.pathname : "";
  } catch (_) {
    return "";
  }
}

function requestClass(input, init) {
  const method = String(init?.method || (typeof input === "object" && input?.method) || "GET").toUpperCase();
  const path = requestPathOf(input);
  if (!path.startsWith("/api/") || path.startsWith("/api/rover/") || (method !== "GET" && method !== "HEAD")) return "direct";
  return BULK_REQUEST.test(path) ? "bulk" : "status";
}

function requestLimitConstrained() {
  if (state.roverControl?.lease) return true;
  return typeof window.LinkGrade?.constrained === "function" && window.LinkGrade.constrained();
}

function startQueuedRequests() {
  const limit = requestLimitConstrained() ? REQUEST_LIMIT.constrained : REQUEST_LIMIT.normal;
  const driving = Boolean(state.roverControl?.lease);
  while (requestQueue.active < limit && requestQueue.waiting.length) {
    let index = requestQueue.waiting.findIndex((item) => item.kind === "status");
    if (index < 0) {
      if (requestQueue.bulkActive > 0 && driving) return;
      index = 0;
    }
    const [item] = requestQueue.waiting.splice(index, 1);
    item.start();
  }
}

function limitedFetch(input, init) {
  const kind = requestClass(input, init);
  if (kind === "direct") return pageFetch(input, init);
  return new Promise((resolve, reject) => {
    const signal = init?.signal;
    const item = { kind, start: null };
    const abort = () => {
      const index = requestQueue.waiting.indexOf(item);
      if (index >= 0) requestQueue.waiting.splice(index, 1);
      reject(signal.reason ?? new DOMException("The operation was aborted.", "AbortError"));
    };
    item.start = () => {
      signal?.removeEventListener?.("abort", abort);
      requestQueue.active += 1;
      if (kind === "bulk") requestQueue.bulkActive += 1;
      // The slot's own deadline; the caller's abort still ends the request at once.
      const deadline = new AbortController();
      const timer = setTimeout(() => deadline.abort(), REQUEST_TIMEOUT_MS[kind] || REQUEST_TIMEOUT_MS.status);
      const relay = () => deadline.abort();
      if (signal) {
        if (signal.aborted) deadline.abort();
        else signal.addEventListener?.("abort", relay, { once: true });
      }
      const runInit = { ...(init || {}), signal: deadline.signal };
      let request;
      try {
        request = kind === "status" ? conditionalFetch(input, runInit)
          : pageFetch(input, { priority: "low", ...runInit });
      } catch (err) {
        request = Promise.reject(err);
      }
      request.then(resolve, reject).finally(() => {
        clearTimeout(timer);
        signal?.removeEventListener?.("abort", relay);
        requestQueue.active -= 1;
        if (kind === "bulk") requestQueue.bulkActive -= 1;
        startQueuedRequests();
      });
    };
    if (signal?.aborted) { abort(); return; }
    signal?.addEventListener?.("abort", abort, { once: true });
    requestQueue.waiting.push(item);
    startQueuedRequests();
  });
}
function conditionalKey(input) {
  if (typeof input !== "string" && !(input instanceof URL)) return "";
  try {
    const url = new URL(String(input), location.href);
    return url.origin === location.origin ? url.pathname + url.search : "";
  } catch (_) {
    return "";
  }
}

async function conditionalFetch(input, init) {
  const key = conditionalKey(input);
  const kept = key ? CONDITIONAL_CACHE.entries.get(key) : null;
  const options = { priority: "low", ...(init || {}) };
  if (kept) {
    const headers = new Headers(init?.headers || undefined);
    headers.set("If-None-Match", kept.etag);
    options.headers = headers;
  }
  const response = await pageFetch(input, options);
  if (response.status === 304 && kept) {
    // Most recently used goes last; the oldest is dropped first.
    CONDITIONAL_CACHE.entries.delete(key);
    CONDITIONAL_CACHE.entries.set(key, kept);
    return new Response(kept.body, { status: 200, statusText: "OK", headers: kept.headers });
  }
  const etag = response.headers?.get?.("ETag");
  const type = response.headers?.get?.("Content-Type") || "";
  if (key && response.status === 200 && etag && type.includes("json")) {
    response.clone().text().then((body) => {
      if (body.length > CONDITIONAL_CACHE.maxBytes) return;
      CONDITIONAL_CACHE.entries.delete(key);
      CONDITIONAL_CACHE.entries.set(key, { etag, body, headers: [["Content-Type", type], ["ETag", etag]] });
      while (CONDITIONAL_CACHE.entries.size > CONDITIONAL_CACHE.maxEntries) {
        CONDITIONAL_CACHE.entries.delete(CONDITIONAL_CACHE.entries.keys().next().value);
      }
    }).catch(() => {});
  } else if (key && response.status !== 304) {
    CONDITIONAL_CACHE.entries.delete(key);
  }
  return response;
}
window.fetch = limitedFetch;
