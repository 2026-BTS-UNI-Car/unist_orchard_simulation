"""Shared /api/telemetry snapshot (S03/S23/S29).

Every dashboard tab polls /api/telemetry. Building the payload reads /proc,
hwmon, bridge status files and the Docker API, so concurrent clients used to
repeat the whole collection. This wrapper builds one snapshot at most once per
TTL (single-flight: concurrent callers wait for the same build) and shares it.

The rover connection state is NOT shared: it is read again for every request,
because the armed indicator and control-related UI decisions use it.
Docker data and the external IP lookup are refreshed in the background by the
provider (see TelemetryProvider.docker_cached and _cached_external_ip).
"""
from __future__ import annotations

import threading
import time
from typing import Any, Callable


class SharedTelemetry:
    def __init__(self, provider: Any, ttl_s: float = 1.0, clock: Callable[[], float] = time.monotonic,
                 wait_s: float = 10.0):
        self.provider = provider
        self.ttl_s = float(ttl_s)
        self.clock = clock
        self.wait_s = float(wait_s)
        self._cond = threading.Condition()
        self._value: dict[str, Any] | None = None
        self._built_at = 0.0
        self._building = False
        self._error: BaseException | None = None
        self.builds = 0

    def _shared(self) -> dict[str, Any]:
        with self._cond:
            now = self.clock()
            if self._value is not None and now - self._built_at < self.ttl_s:
                return self._value
            if self._building:
                deadline = time.monotonic() + self.wait_s
                generation = self.builds
                while self._building and time.monotonic() < deadline:
                    self._cond.wait(timeout=max(0.01, deadline - time.monotonic()))
                if self._value is not None and self.builds != generation:
                    return self._value
                if self._building:
                    # The builder is stuck: serve the last snapshot rather than block.
                    if self._value is not None:
                        return self._value
                    raise TimeoutError("telemetry_snapshot_busy")
                if self._error is not None:
                    raise self._error
                if self._value is not None:
                    return self._value
            self._building = True
            self._error = None
        try:
            value = self.provider.telemetry(include_rover=False)
        except BaseException as error:
            with self._cond:
                self._building = False
                self._error = error
                self._cond.notify_all()
            raise
        with self._cond:
            self._value = value
            self._built_at = self.clock()
            self._building = False
            self.builds += 1
            self._cond.notify_all()
        return value

    def get(self) -> dict[str, Any]:
        shared = self._shared()
        age = max(0.0, self.clock() - self._built_at)
        rover = self.provider.rover_status()
        connection = {**shared.get("connection", {}), "rover": rover}
        payload = {**shared, "connection": connection,
                   "rover": shared["rover"] if shared.get("rover_source") == "external" else rover}
        payload["snapshot_age_s"] = round(age, 3)
        return payload
