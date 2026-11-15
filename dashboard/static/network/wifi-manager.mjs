// Wi-Fi management of the Network page (2026-11-16; server: dashboard_server/wifi_manager.py, host helper:
// deploy/offline_ap/ubts_network_mode.py). Lists the networks of a scan and the saved profiles, joins a network
// with a password, and changes autoconnect and priority. Every change of connection is a trial on the Jetson:
// unless "이 연결 유지" is pressed within the trial time, the Jetson goes back to the previous connection by itself.
//
// Slow-link rules (docs/slow_link.md): this module is fetched the first time the Network page opens
// (04-telemetry.js); the lists are fetched only while that page is on screen (once, then every minute); a scan
// runs only on the button; the trial state comes with the normal status poll.
// A password stays in its input field until it is sent and is cleared afterwards. It is never kept in a variable
// of this module, never written to the log and never part of an answer.

export const WIFI_REFRESH_MS = 60000;
export const PRIORITY_MIN = -999;
export const PRIORITY_MAX = 999;

const ERROR_TEXT = {
  ssid_required: "SSID를 고르세요.",
  ssid_invalid: "SSID가 올바르지 않습니다.",
  ssid_length: "SSID는 32바이트 이하입니다.",
  ssid_control_character: "SSID에 쓸 수 없는 글자가 있습니다.",
  ssid_not_in_scan: "최근 검색 결과에 없는 Wi-Fi입니다. 주변 검색을 다시 하세요.",
  scan_too_old: "검색한 지 10분이 지났습니다. 주변 검색을 다시 하세요.",
  ssid_already_saved: "이미 저장된 Wi-Fi입니다. 저장된 Wi-Fi 목록에서 연결하세요.",
  security_not_supported: "이 Wi-Fi의 보안 방식(기업용 802.1X, WEP 등)은 대시보드에서 연결할 수 없습니다.",
  password_required: "비밀번호를 넣으세요.",
  password_invalid: "비밀번호는 영문·숫자·기호 8~63자(WPA3는 128자까지)입니다.",
  password_length: "비밀번호 길이가 맞지 않습니다(WPA2 8~63자, 16진수 64자).",
  password_characters: "비밀번호에는 영문·숫자·기호만 쓸 수 있습니다.",
  password_not_editable: "이 저장 Wi-Fi는 대시보드에서 비밀번호를 바꿀 수 없습니다.",
  open_network_takes_no_password: "비밀번호가 없는 Wi-Fi입니다. 비밀번호 칸을 비우세요.",
  already_connected: "지금 연결된 Wi-Fi입니다.",
  unknown_saved_profile: "저장된 Wi-Fi 목록이 바뀌었습니다. 목록을 다시 읽으세요.",
  priority_invalid: "우선순위는 -999~999 사이 정수입니다.",
  priority_integer: "우선순위는 정수입니다.",
  priority_range: "우선순위는 -999~999 사이입니다.",
  profile_without_security: "이 저장 Wi-Fi에는 비밀번호 설정이 없습니다.",
  profile_file_not_private: "이 저장 Wi-Fi 파일의 권한이 예상과 달라 바꾸지 않았습니다.",
  profile_file_not_regular: "이 저장 Wi-Fi 파일을 바꿀 수 없습니다.",
  profile_file_not_utf8: "이 저장 Wi-Fi 파일을 읽을 수 없습니다.",
  txn_invalid: "시험 연결 번호가 올바르지 않습니다.",
};
const STATE_TEXT = {
  disabled: "네트워크 제어가 꺼져 있습니다.",
  rate_limited: "조금 뒤에 다시 하세요.",
  scan_in_progress: "검색 중입니다.",
  busy: "다른 네트워크 변경이 진행 중입니다.",
  action_not_available: "지금 상태에서는 할 수 없습니다.",
  ethernet_ap_locked: "Ethernet이 연결되어 AP가 유지되는 동안은 바꿀 수 없습니다.",
  scan_not_available: "지금은 주변 검색을 할 수 없습니다(AP 방식이거나 전환 중).",
  no_such_trial: "그 시험 연결은 이미 끝났습니다.",
  not_ready: "새 연결이 아직 준비되지 않았습니다.",
  controller_unavailable: "Jetson 네트워크 제어 서비스가 응답하지 않습니다.",
  helper_unavailable: "Jetson 네트워크 도우미를 찾지 못했습니다.",
  helper_error: "Jetson 네트워크 도우미 실행에 실패했습니다.",
  helper_failed: "Jetson 네트워크 도우미가 실패를 알렸습니다.",
  confirmation_required: "확인이 필요합니다.",
  forbidden: "이 화면의 세션이 끝났습니다. 화면을 새로 여세요.",
};
const REASON_TEXT = {
  operator_confirmed: "확인을 눌러",
  operator_revert: "직접 되돌려",
  not_confirmed_in_time: "제한 시간 안에 확인이 없어",
  no_address_or_route_in_time: "주소(DHCP)나 게이트웨이를 받지 못해",
  activation_failed: "연결하지 못해(비밀번호와 신호를 확인하세요)",
  activation_rejected: "NetworkManager가 연결을 거부해",
  link_lost: "새 연결이 끊겨",
  ethernet_connected: "Ethernet이 연결되어",
  controller_restarted: "제어 서비스가 다시 시작되어",
  request_expired: "요청이 제때 처리되지 않아",
  state_changed: "그사이 연결 상태가 바뀌어",
  invalid_command: "요청이 올바르지 않아",
};

const HTML_ESCAPES = { "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#039;" };
export function escapeHtml(value) {
  return String(value ?? "").replace(/[&<>"']/g, (ch) => HTML_ESCAPES[ch]);
}

export function errorText(result) {
  const code = String(result?.error || "");
  if (ERROR_TEXT[code]) return ERROR_TEXT[code];
  const state = String(result?.state || "");
  if (state === "rate_limited" && result.retry_after_s) return `${Math.ceil(result.retry_after_s)}초 뒤에 다시 하세요.`;
  return STATE_TEXT[state] || code || "요청이 거부되었습니다.";
}

export function signalText(network) {
  const parts = [];
  if (Number.isFinite(network?.signal_dbm)) parts.push(`${network.signal_dbm} dBm`);
  if (Number.isFinite(network?.signal_pct)) parts.push(`${network.signal_pct}%`);
  return parts.join(" · ") || "--";
}

export function bandText(band, channel) {
  return [band ? String(band).replace("GHz", " GHz") : "", channel ? `ch ${channel}` : ""].filter(Boolean).join(" · ") || "--";
}

export function securityText(value) {
  const text = String(value || "");
  if (!text || text === "open" || text === "none") return "비밀번호 없음";
  if (text === "wpa-psk") return "WPA/WPA2";
  if (text === "sae") return "WPA3";
  return text;
}

export function ageText(seconds) {
  const value = Math.max(0, Math.round(Number(seconds) || 0));
  if (value < 60) return `${value}초 전`;
  if (value < 3600) return `${Math.round(value / 60)}분 전`;
  if (value < 86400) return `${Math.round(value / 3600)}시간 전`;
  return `${Math.round(value / 86400)}일 전`;
}

export function clockText(seconds) {
  const value = Math.max(0, Math.ceil(Number(seconds) || 0));
  return `${Math.floor(value / 60)}:${String(value % 60).padStart(2, "0")}`;
}

export function validPriority(value) {
  const number = Number(value);
  return String(value).trim() !== "" && Number.isInteger(number) && number >= PRIORITY_MIN && number <= PRIORITY_MAX ? number : null;
}

function metric(label, value) {
  return `<div><span>${escapeHtml(label)}</span><b>${escapeHtml(value)}</b></div>`;
}

// One card per network of the scan (strongest access point of an SSID and band).
export function scanCardsHtml(networks, saved, canChange) {
  return networks.map((network, index) => {
    const savedRow = saved.find((row) => row.ssid === network.ssid);
    const tag = network.in_use ? '<span class="tag ok">연결됨</span>'
      : !network.supported ? '<span class="tag warn">연결 불가</span>'
        : savedRow ? '<span class="tag info">저장됨</span>' : "";
    const button = network.in_use || !network.supported ? ""
      : `<div class="panel-actions network-mode-actions"><button class="action-button secondary" data-wifi-action="pick-scan" data-wifi-index="${index}" type="button"${canChange ? "" : " disabled"}>연결</button></div>`;
    return `<article class="interface-card wifi-stack"><div class="card-title"><strong>${escapeHtml(network.ssid)}</strong>${tag}</div>
<div class="interface-metrics">${metric("신호", signalText(network))}${metric("대역", bandText(network.band, network.channel))}${metric("보안", `${securityText(network.security)}${network.ap_count > 1 ? ` · AP ${network.ap_count}개` : ""}`)}</div>${button}</article>`;
  }).join("");
}

// One card per saved profile: autoconnect, priority, save, connect.
export function savedCardsHtml(rows, canChange, canSave) {
  return rows.map((row, index) => {
    const tag = row.active ? '<span class="tag ok">연결됨</span>'
      : row.trial ? '<span class="tag warn">시험 중</span>'
        : row.storage === "runtime" ? '<span class="tag">재부팅 때 지워짐</span>' : "";
    const used = row.last_used ? new Date(row.last_used * 1000).toLocaleString("ko-KR", { month: "numeric", day: "numeric", hour: "2-digit", minute: "2-digit" }) : "--";
    const security = [securityText(row.key_mgmt), row.band ? String(row.band).replace("GHz", " GHz") : ""].filter(Boolean).join(" · ");
    return `<article class="interface-card wifi-stack" data-wifi-saved-row="${index}"><div class="card-title"><strong>${escapeHtml(row.name)}</strong>${tag}</div>
<div class="interface-metrics">${row.name !== row.ssid ? metric("SSID", row.ssid) : ""}${metric("보안", security)}${metric("마지막 연결", used)}</div>
<div class="panel-actions network-mode-actions"><label class="switch-control"><input data-wifi-field="autoconnect" type="checkbox"${row.autoconnect ? " checked" : ""}/><span>자동 연결</span></label>
<label class="field-inline">우선순위<input class="input wifi-priority" data-wifi-field="priority" inputmode="numeric" max="${PRIORITY_MAX}" min="${PRIORITY_MIN}" step="1" type="number" value="${Number(row.priority) || 0}"/></label></div>
<div class="panel-actions network-mode-actions"><button class="action-button secondary" data-wifi-action="save-profile" data-wifi-index="${index}" type="button"${canSave ? "" : " disabled"}>저장</button>${row.active ? "" : `<button class="action-button secondary" data-wifi-action="pick-saved" data-wifi-index="${index}" type="button"${canChange ? "" : " disabled"}>연결</button>`}</div></article>`;
  }).join("");
}

// Title and detail of the trial notice.
export function trialText(trial, secondsLeft, offline) {
  const ssid = trial.ssid || "새 Wi-Fi";
  const previous = trial.previous || "이전 연결";
  let title;
  let detail;
  if (trial.phase === "reverting") {
    title = `${ssid} 시험 연결을 되돌리는 중`;
    detail = `${previous}(으)로 돌아가고 있습니다.`;
  } else if (trial.phase === "waiting_confirm") {
    title = `${ssid}에 연결됨 · 남은 시간 ${clockText(secondsLeft)}`;
    const checks = [
      trial.ipv4 ? `주소 ${trial.ipv4}` : "",
      trial.gateway_reply === true ? "게이트웨이 응답 있음" : trial.gateway_reply === false ? "게이트웨이 응답 없음" : "",
      trial.internet_reply === true ? "인터넷 응답 있음" : trial.internet_reply === false ? "인터넷 응답 없음" : "",
    ].filter(Boolean).join(" · ");
    detail = `${checks ? `${checks}. ` : ""}'이 연결 유지'를 누르지 않으면 ${previous}(으)로 돌아갑니다.`;
  } else {
    title = `${ssid}에 연결하는 중 · 남은 시간 ${clockText(secondsLeft)}`;
    detail = `연결되면 이 화면이 다시 이어진 뒤 확인할 수 있습니다. 확인이 없으면 ${previous}(으)로 돌아갑니다.`;
  }
  if (offline) detail += " 지금 이 화면은 Jetson과 끊겨 있습니다.";
  return { title, detail };
}

// The message after a trial ended (last_trial of the status).
export function lastTrialMessage(last) {
  const reason = REASON_TEXT[last.reason] || last.reason || "";
  const warnings = (last.warnings || []).filter(Boolean);
  if (last.result === "kept") {
    const extra = warnings.includes("profile_not_on_disk") ? " 프로필이 디스크에 저장되지 않아 재부팅하면 사라질 수 있습니다."
      : warnings.length ? ` 설정 일부를 적용하지 못했습니다(${warnings.join(", ")}).` : "";
    return { level: warnings.length ? "warn" : "ok", text: `${last.ssid} 연결을 유지합니다.${extra}` };
  }
  if (last.result === "reverted") return { level: "warn", text: `${last.ssid} 시험 연결을 ${reason} 이전 연결로 되돌렸습니다.` };
  return { level: "bad", text: `${last.ssid} 시험 연결을 ${reason} 시작하지 않았습니다.` };
}

// POST with the page's session cookie; the answer is read even when the status is not 200.
async function post(path, payload) {
  const res = await fetch(path, {
    method: "POST",
    cache: "no-store",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload || {}),
  });
  let body = null;
  try { body = await res.json(); } catch (_) { body = null; }
  return body && typeof body === "object" ? body : { ok: false, state: res.status === 403 ? "forbidden" : "http_error", error: `HTTP ${res.status}` };
}

export function initWifiManager(api) {
  const $ = (id) => document.getElementById(id);
  const view = {
    overview: null, loadedAt: 0, loading: false, scanning: false, busy: false, form: null, control: {},
    trialDeadline: 0, trialTxn: null, trialTimer: null, lastTrialTxn: undefined,
  };

  const canChange = () => !!view.control.can_request_connect && !view.control.trial && !view.busy;
  const canSave = () => !view.busy && !view.control.trial && !view.control.transition && view.control.enabled !== false;
  const savedRows = () => view.overview?.saved || [];

  function showResult(level, message) {
    const box = $("wifiResult");
    if (!box) return;
    box.hidden = !message;
    box.className = `test-result ${level || ""}`.trim();
    box.textContent = message || "";
  }

  function syncButtons() {
    const scanButton = $("wifiScanButton");
    if (scanButton) scanButton.disabled = view.scanning || view.busy || !view.control.can_scan;
    document.querySelectorAll('[data-wifi-action="pick-scan"], [data-wifi-action="pick-saved"]').forEach((button) => { button.disabled = !canChange(); });
    document.querySelectorAll('[data-wifi-action="save-profile"]').forEach((button) => { button.disabled = !canSave(); });
    const submit = $("wifiConnectSubmit");
    if (submit) submit.disabled = !canChange();
  }

  function renderScan() {
    const list = $("wifiScanList");
    if (!list) return;
    const scan = view.overview?.scan;
    const networks = Array.isArray(scan?.networks) ? scan.networks : [];
    const age = $("wifiScanAge");
    if (age) {
      age.textContent = view.scanning ? "검색 중…"
        : scan ? `${ageText(scan.age_s)} · ${networks.length}개${scan.hidden_count ? ` · 숨긴 Wi-Fi ${scan.hidden_count}개` : ""}` : "검색 전";
    }
    list.innerHTML = networks.length ? scanCardsHtml(networks, savedRows(), canChange())
      : `<p class="control-note">${scan ? "찾은 Wi-Fi가 없습니다." : view.control.can_scan === false && view.control.ap_active ? "AP 방식에서는 주변 검색을 할 수 없습니다(무선랜이 하나). 저장된 Wi-Fi로는 연결할 수 있습니다." : "주변 검색을 누르면 Jetson이 찾은 Wi-Fi가 여기에 나옵니다."}</p>`;
  }

  function renderSaved() {
    const list = $("wifiSavedList");
    if (!list) return;
    const overview = view.overview;
    const rows = savedRows();
    const count = $("wifiSavedCount");
    if (count) count.textContent = overview ? (overview.enabled === false ? "제어 꺼짐" : `${rows.length}개`) : "확인 중";
    let note = "";
    if (!overview) note = "저장된 Wi-Fi를 읽는 중입니다.";
    else if (overview.enabled === false) note = "이 대시보드에서는 네트워크 제어가 꺼져 있습니다.";
    else if (!overview.ok && !rows.length) note = errorText(overview);
    else if (!rows.length) note = "저장된 Wi-Fi가 없습니다.";
    list.innerHTML = note ? `<p class="control-note">${escapeHtml(note)}</p>` : savedCardsHtml(rows, canChange(), canSave());
  }

  async function load(force = false) {
    if (view.loading) return;
    view.loading = true;
    try {
      const payload = await api.getJson(force ? "/api/network/wifi?refresh=1" : "/api/network/wifi");
      // Keep this page's scan when the server has none (for example after the server restarted).
      if (!payload.scan && view.overview?.scan) payload.scan = view.overview.scan;
      view.overview = payload;
    } catch (err) {
      api.debugLog?.("api", "/api/network/wifi failed", err);
    } finally {
      view.loadedAt = Date.now();
      view.loading = false;
    }
    renderScan();
    renderSaved();
  }

  async function scan() {
    if (view.scanning) return;
    view.scanning = true;
    renderScan();
    syncButtons();
    try {
      const result = await post("/api/network/wifi/scan", {});
      const fresh = Array.isArray(result.networks) ? result : result.scan && Array.isArray(result.scan.networks) ? result.scan : null;
      if (fresh) view.overview = { ...(view.overview || { saved: [] }), scan: { networks: fresh.networks, hidden_count: fresh.hidden_count || 0, age_s: fresh.age_s || 0 } };
      showResult(result.ok ? "" : "warn", result.ok ? "" : errorText(result));
    } catch (err) {
      showResult("bad", "검색 요청이 Jetson에 닿지 않았습니다.");
    } finally {
      view.scanning = false;
      renderScan();
      syncButtons();
    }
  }

  function openForm(target) {
    const form = $("wifiConnectForm");
    if (!form) return;
    view.form = target;
    const saved = target.profile || null;
    const canEditPassword = saved ? !!saved.password_editable : target.keyMgmt !== "none";
    $("wifiPasswordField").hidden = !canEditPassword;
    $("wifiPasswordShowField").hidden = !canEditPassword;
    const password = $("wifiPassword");
    password.value = "";
    password.type = "password";
    $("wifiPasswordShow").checked = false;
    $("wifiPasswordLabel").textContent = saved ? "새 비밀번호 (비우면 저장된 비밀번호)" : "비밀번호";
    $("wifiAutoconnect").checked = saved ? !!saved.autoconnect : true;
    $("wifiPriority").value = String(saved ? Number(saved.priority) || 0 : 0);
    $("wifiFormTitle").textContent = `${target.ssid}에 연결`;
    $("wifiFormTag").textContent = saved ? "저장된 Wi-Fi" : "새 Wi-Fi";
    $("wifiFormSub").textContent = target.detail || "";
    const seconds = Number(view.control.trial_seconds) || 150;
    $("wifiFormNote").textContent = `Jetson이 이 Wi-Fi로 바꿔 붙습니다. 이 화면은 잠시 끊길 수 있고, ${seconds}초 안에 '이 연결 유지'를 누르지 않으면 이전 연결로 돌아갑니다. 지금 AP로 접속했다면 휴대폰도 같은 Wi-Fi나 인터넷으로 옮겨 대시보드에 들어와야 확인할 수 있습니다.`;
    form.hidden = false;
    form.scrollIntoView({ block: "nearest" });
    syncButtons();
    if (canEditPassword && !saved) password.focus();
  }

  function closeForm() {
    const form = $("wifiConnectForm");
    if (form) form.hidden = true;
    const password = $("wifiPassword");
    if (password) { password.value = ""; password.type = "password"; }
    view.form = null;
  }

  async function submitForm(event) {
    event.preventDefault();
    const target = view.form;
    if (!target || !canChange()) return;
    const priority = validPriority($("wifiPriority").value);
    if (priority === null) {
      showResult("bad", ERROR_TEXT.priority_invalid);
      return;
    }
    const request = { confirm: "connect_with_auto_revert", autoconnect: $("wifiAutoconnect").checked, priority };
    if (target.profile) request.uuid = target.profile.uuid;
    else request.ssid = target.ssid;
    const passwordInput = $("wifiPassword");
    if (!$("wifiPasswordField").hidden && passwordInput.value) request.password = passwordInput.value;
    if (!target.profile && target.keyMgmt !== "none" && !request.password) {
      showResult("bad", ERROR_TEXT.password_required);
      return;
    }
    const previous = view.control.connection || (view.control.ap_active ? "AP" : "지금 연결");
    const seconds = Number(view.control.trial_seconds) || 150;
    if (!window.confirm(`Jetson을 ${target.ssid}에 연결합니다. 이 화면이 잠시 끊길 수 있습니다. ${seconds}초 안에 '이 연결 유지'를 누르지 않으면 ${previous}(으)로 돌아갑니다. 계속할까요?`)) {
      delete request.password;
      return;
    }
    view.busy = true;
    syncButtons();
    showResult("testing", `${target.ssid} 시험 연결을 요청했습니다.`);
    try {
      const result = await post("/api/network/wifi/connect", request);
      delete request.password;
      if (result.ok) {
        closeForm();
        showResult("testing", `${result.ssid || target.ssid}(으)로 바꾸는 중입니다. 이 화면이 끊겼다가 다시 이어지면 위의 '이 연결 유지'를 누르세요.`);
      } else {
        showResult("bad", errorText(result));
      }
    } catch (err) {
      showResult("testing", "응답 전에 연결이 끊겼습니다. 화면이 다시 이어지면 시험 연결 상태를 확인하세요.");
    } finally {
      delete request.password;
      view.busy = false;
      syncButtons();
      load(true);
    }
  }

  async function saveProfile(index, button) {
    const row = savedRows()[index];
    const card = button.closest("[data-wifi-saved-row]");
    if (!row || !card) return;
    const autoconnect = card.querySelector('[data-wifi-field="autoconnect"]').checked;
    const priority = validPriority(card.querySelector('[data-wifi-field="priority"]').value);
    if (priority === null) {
      showResult("bad", ERROR_TEXT.priority_invalid);
      return;
    }
    view.busy = true;
    syncButtons();
    try {
      const result = await post("/api/network/wifi/profile", { uuid: row.uuid, autoconnect, priority });
      showResult(result.ok ? "ok" : "bad", result.ok ? `${row.name}: 자동 연결 ${autoconnect ? "켬" : "끔"}, 우선순위 ${priority}로 저장했습니다.` : errorText(result));
    } catch (err) {
      showResult("bad", "저장 요청이 Jetson에 닿지 않았습니다.");
    } finally {
      view.busy = false;
      await load(true);
    }
  }

  async function trialAction(action) {
    const txn = view.trialTxn;
    if (!txn) return;
    if (action === "revert" && !window.confirm("시험 연결을 끝내고 이전 연결로 되돌립니다. 이 화면이 잠시 끊길 수 있습니다. 계속할까요?")) return;
    $("wifiTrialConfirm").disabled = true;
    $("wifiTrialRevert").disabled = true;
    try {
      const result = await post(`/api/network/wifi/${action}`, { txn });
      showResult(result.ok ? "testing" : "bad", result.ok
        ? (action === "confirm" ? "확인을 보냈습니다. 새 연결을 저장하는 중입니다." : "되돌리기를 보냈습니다. 이 화면이 잠시 끊길 수 있습니다.")
        : errorText(result));
    } catch (err) {
      showResult("testing", "응답 전에 연결이 끊겼습니다. 화면이 다시 이어지면 상태를 확인하세요.");
    } finally {
      $("wifiTrialRevert").disabled = false;
    }
  }

  function renderTrial() {
    const banner = $("wifiTrialBanner");
    const actions = $("wifiTrialActions");
    if (!banner || !actions) return;
    const trial = view.control.trial;
    if (!trial) {
      banner.hidden = true;
      actions.hidden = true;
      if (view.trialTimer) { window.clearInterval(view.trialTimer); view.trialTimer = null; }
      return;
    }
    const offline = (api.telemetryFailures?.() || 0) > 0;
    const { title, detail } = trialText(trial, (view.trialDeadline - Date.now()) / 1000, offline);
    $("wifiTrialTitle").textContent = title;
    $("wifiTrialDetail").textContent = detail;
    $("wifiTrialConfirm").disabled = !trial.can_confirm || offline;
    banner.hidden = false;
    actions.hidden = false;
    if (!view.trialTimer) view.trialTimer = window.setInterval(renderTrial, 1000);
  }

  function renderLastTrial() {
    const last = view.control.last_trial;
    if (view.lastTrialTxn === undefined) {
      // The first status after opening the page: remember it, do not announce an old result.
      view.lastTrialTxn = last?.txn || null;
      return;
    }
    if (!last || last.txn === view.lastTrialTxn) return;
    view.lastTrialTxn = last.txn;
    const message = lastTrialMessage(last);
    showResult(message.level, message.text);
    load(true);
  }

  // Called with every status (renderNetworkMode, 04-telemetry.js).
  function render(control) {
    view.control = control || {};
    const trial = view.control.trial;
    if (trial && Number.isFinite(Number(trial.seconds_left))) {
      const deadline = Date.now() + Number(trial.seconds_left) * 1000;
      if (trial.txn !== view.trialTxn || Math.abs(deadline - view.trialDeadline) > 2000) view.trialDeadline = deadline;
      view.trialTxn = trial.txn;
    } else if (!trial) {
      view.trialTxn = null;
    }
    renderTrial();
    renderLastTrial();
    syncButtons();
    if (api.currentRoute() === "/network" && !document.hidden && !view.loading
        && (!view.loadedAt || Date.now() - view.loadedAt > WIFI_REFRESH_MS)) {
      load();
    }
  }

  $("wifiScanButton")?.addEventListener("click", scan);
  $("wifiConnectForm")?.addEventListener("submit", submitForm);
  $("wifiConnectCancel")?.addEventListener("click", closeForm);
  $("wifiTrialConfirm")?.addEventListener("click", () => trialAction("confirm"));
  $("wifiTrialRevert")?.addEventListener("click", () => trialAction("revert"));
  $("wifiPasswordShow")?.addEventListener("change", (event) => { $("wifiPassword").type = event.target.checked ? "text" : "password"; });
  $("wifiScanList")?.addEventListener("click", (event) => {
    const button = event.target.closest('[data-wifi-action="pick-scan"]');
    const network = button ? view.overview?.scan?.networks?.[Number(button.dataset.wifiIndex)] : null;
    if (!network) return;
    const profile = savedRows().find((row) => row.ssid === network.ssid) || null;
    const detail = [bandText(network.band, network.channel), securityText(network.security), signalText(network)].join(" · ");
    openForm({ ssid: network.ssid, keyMgmt: profile ? profile.key_mgmt : network.key_mgmt, profile, detail });
  });
  $("wifiSavedList")?.addEventListener("click", (event) => {
    const button = event.target.closest("[data-wifi-action]");
    if (!button) return;
    const index = Number(button.dataset.wifiIndex);
    if (button.dataset.wifiAction === "save-profile") {
      saveProfile(index, button);
      return;
    }
    const row = savedRows()[index];
    if (row) openForm({ ssid: row.ssid || row.name, keyMgmt: row.key_mgmt, profile: row, detail: securityText(row.key_mgmt) });
  });

  return {
    render,
    routeChanged() {
      if (api.currentRoute() === "/network" && !view.loadedAt) load();
    },
  };
}
