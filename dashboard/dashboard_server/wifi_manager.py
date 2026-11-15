"""Wi-Fi management of the Network page (2026-11-16).

The dashboard never talks to NetworkManager itself. Every request goes to the
host helper (deploy/offline_ap/ubts_network_mode.py, installed as
/usr/local/sbin/ubts-network-mode) through TelemetryProvider._network_mode_command,
with the request as JSON on the helper's stdin. The helper is the authority for
validation and state; this module checks shapes early, keeps small caches and
limits how often a scan or a connect can start:

- a radio scan at most once per SCAN_MIN_INTERVAL_S for all viewers (the helper
  has its own 10 s limit); one scan at a time;
- the saved list is cached for SAVED_TTL_S;
- a connect at most once per CONNECT_MIN_INTERVAL_S.

A password is passed to the helper and dropped. It is never stored here, never
logged and never part of an answer.
"""

from __future__ import annotations

import re
import threading
import time
from typing import Any, Callable

SCAN_MIN_INTERVAL_S = 15.0
SAVED_TTL_S = 10.0
CONNECT_MIN_INTERVAL_S = 5.0
PROFILE_MIN_INTERVAL_S = 1.0
PRIORITY_MIN, PRIORITY_MAX = -999, 999
CONNECT_CONFIRM = "connect_with_auto_revert"
UUID_RE = re.compile(r"[0-9a-fA-F]{8}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{12}")
TXN_RE = re.compile(r"[0-9a-f]{16}")

# HTTP status of a helper answer by its state (the body is always JSON with ok and state).
STATUS_BY_STATE = {
    "invalid_request": 400,
    "bad_request": 400,
    "confirmation_required": 400,
    "rate_limited": 429,
    "busy": 409,
    "scan_in_progress": 409,
    "action_not_available": 409,
    "ethernet_ap_locked": 409,
    "scan_not_available": 409,
    "no_such_trial": 409,
    "not_ready": 409,
    "disabled": 503,
    "helper_unavailable": 503,
    "helper_error": 503,
    "helper_failed": 502,
    "controller_unavailable": 503,
}

Command = Callable[..., dict]


def http_status(result: dict) -> int:
    if result.get("ok"):
        return 200
    return STATUS_BY_STATE.get(str(result.get("state") or ""), 409)


def _bad(code: str) -> dict:
    return {"ok": False, "state": "bad_request", "error": code}


def _ssid_ok(value: Any) -> bool:
    return (isinstance(value, str) and 1 <= len(value.encode("utf-8")) <= 32
            and not any(ord(character) < 0x20 or ord(character) == 0x7F for character in value))


def _password_ok(value: Any) -> bool:
    # The helper checks the length rule of the network's security; here only the outer bounds.
    return isinstance(value, str) and 8 <= len(value) <= 128 and all(0x20 <= ord(character) <= 0x7E for character in value)


def _priority_ok(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and PRIORITY_MIN <= value <= PRIORITY_MAX


class WifiManager:
    def __init__(self, command: Command, enabled: Callable[[], bool], clock: Callable[[], float] = time.monotonic):
        self._command = command
        self._enabled = enabled
        self._clock = clock
        self._lock = threading.Lock()
        self._scan_lock = threading.Lock()
        self._scan: dict | None = None
        self._scan_started_at: float | None = None
        self._saved: dict | None = None
        self._saved_at: float | None = None
        self._last_connect_at: float | None = None
        self._last_profile_at: float | None = None

    # ---- reads -----------------------------------------------------------
    def overview(self, refresh_saved: bool = False) -> dict:
        """GET /api/network/wifi: saved profiles and the last scan (a scan is never started here)."""
        if not self._enabled():
            return {"ok": False, "enabled": False, "state": "disabled", "saved": [], "scan": None}
        saved = self.saved(force=refresh_saved)
        with self._lock:
            scan = dict(self._scan) if self._scan else None
            started = self._scan_started_at
        if scan is not None and started is not None:
            scan["age_s"] = round(max(0.0, self._clock() - started), 1)
        return {
            "ok": bool(saved.get("ok")),
            "enabled": True,
            "state": saved.get("state") or ("ok" if saved.get("ok") else "helper_failed"),
            "error": saved.get("error"),
            "saved": saved.get("profiles") or [],
            "scan": scan,
            "limits": {"scan_min_interval_s": SCAN_MIN_INTERVAL_S, "priority_min": PRIORITY_MIN, "priority_max": PRIORITY_MAX},
        }

    def saved(self, force: bool = False) -> dict:
        now = self._clock()
        with self._lock:
            if not force and self._saved is not None and self._saved_at is not None and now - self._saved_at < SAVED_TTL_S:
                return self._saved
        result = self._command("wifi-saved", payload={})
        if result.get("ok"):
            with self._lock:
                self._saved = result
                self._saved_at = self._clock()
        return result

    def _forget_saved(self) -> None:
        with self._lock:
            self._saved = None
            self._saved_at = None

    # ---- scan ------------------------------------------------------------
    def scan(self) -> dict:
        if not self._enabled():
            return {"ok": False, "state": "disabled"}
        now = self._clock()
        with self._lock:
            if self._scan_started_at is not None and now - self._scan_started_at < SCAN_MIN_INTERVAL_S:
                wait = SCAN_MIN_INTERVAL_S - (now - self._scan_started_at)
                scan = dict(self._scan) if self._scan else None
                if scan is not None:
                    scan["age_s"] = round(now - self._scan_started_at, 1)
                return {"ok": False, "state": "rate_limited", "retry_after_s": round(wait, 1), "scan": scan}
        if not self._scan_lock.acquire(blocking=False):
            return {"ok": False, "state": "scan_in_progress"}
        try:
            with self._lock:
                self._scan_started_at = self._clock()
            result = self._command("wifi-scan", payload={})
            networks = result.get("networks")
            if result.get("ok") and isinstance(networks, list):
                with self._lock:
                    self._scan = {
                        "networks": networks,
                        "hidden_count": result.get("hidden_count", 0),
                        "rescanned": result.get("rescanned"),
                        "scanned_at": result.get("scanned_at"),
                    }
                    self._scan_started_at = self._clock() - float(result.get("age_s") or 0.0)
            else:
                with self._lock:
                    # A refused scan (AP mode, controller busy) does not use up the interval.
                    self._scan_started_at = None
                    cached = result.get("scan")
                    if isinstance(cached, dict) and isinstance(cached.get("networks"), list) and self._scan is None:
                        self._scan = {key: cached.get(key) for key in ("networks", "hidden_count", "rescanned", "scanned_at")}
            return result
        finally:
            self._scan_lock.release()

    # ---- changes ---------------------------------------------------------
    def connect(self, payload: Any) -> dict:
        if not self._enabled():
            return {"ok": False, "state": "disabled"}
        if not isinstance(payload, dict):
            return _bad("object_required")
        if payload.get("confirm") != CONNECT_CONFIRM:
            return {"ok": False, "state": "confirmation_required"}
        request: dict[str, Any] = {}
        if payload.get("uuid") is not None:
            if not isinstance(payload.get("uuid"), str) or not UUID_RE.fullmatch(payload["uuid"]):
                return _bad("uuid_invalid")
            request["uuid"] = payload["uuid"].lower()
        elif _ssid_ok(payload.get("ssid")):
            request["ssid"] = payload["ssid"]
        else:
            return _bad("ssid_invalid")
        password = payload.get("password")
        if password not in (None, ""):
            if not _password_ok(password):
                return _bad("password_invalid")
            request["password"] = password
        autoconnect = payload.get("autoconnect", True)
        if not isinstance(autoconnect, bool):
            return _bad("autoconnect_boolean")
        priority = payload.get("priority", 0)
        if not _priority_ok(priority):
            return _bad("priority_invalid")
        request.update(autoconnect=autoconnect, priority=priority)
        now = self._clock()
        with self._lock:
            if self._last_connect_at is not None and now - self._last_connect_at < CONNECT_MIN_INTERVAL_S:
                return {"ok": False, "state": "rate_limited",
                        "retry_after_s": round(CONNECT_MIN_INTERVAL_S - (now - self._last_connect_at), 1)}
            self._last_connect_at = now
        try:
            result = self._command("wifi-connect", payload=request)
        finally:
            request.pop("password", None)
        self._forget_saved()
        return result

    def confirm(self, payload: Any) -> dict:
        return self._trial("wifi-confirm", payload)

    def revert(self, payload: Any) -> dict:
        return self._trial("wifi-revert", payload)

    def _trial(self, command: str, payload: Any) -> dict:
        if not self._enabled():
            return {"ok": False, "state": "disabled"}
        txn = payload.get("txn") if isinstance(payload, dict) else None
        if not isinstance(txn, str) or not TXN_RE.fullmatch(txn):
            return _bad("txn_invalid")
        result = self._command(command, payload={"txn": txn})
        self._forget_saved()
        return result

    def update_profile(self, payload: Any) -> dict:
        if not self._enabled():
            return {"ok": False, "state": "disabled"}
        if not isinstance(payload, dict):
            return _bad("object_required")
        uuid = payload.get("uuid")
        if not isinstance(uuid, str) or not UUID_RE.fullmatch(uuid):
            return _bad("uuid_invalid")
        if not isinstance(payload.get("autoconnect"), bool):
            return _bad("autoconnect_boolean")
        if not _priority_ok(payload.get("priority")):
            return _bad("priority_invalid")
        now = self._clock()
        with self._lock:
            if self._last_profile_at is not None and now - self._last_profile_at < PROFILE_MIN_INTERVAL_S:
                return {"ok": False, "state": "rate_limited", "retry_after_s": PROFILE_MIN_INTERVAL_S}
            self._last_profile_at = now
        result = self._command("wifi-profile", payload={"uuid": uuid.lower(), "autoconnect": payload["autoconnect"],
                                                         "priority": payload["priority"]})
        self._forget_saved()
        return result
