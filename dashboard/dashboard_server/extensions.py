"""Extension points of the dashboard (D-12, docs/STEERING_SERVER_KO.md section 2).

Settings come from the environment once, when the server is created:

  DASHBOARD_VEHICLE       differential (default) | front_steer   which drive panel the page shows
  DASHBOARD_MODE          live (default) | simulation
  DASHBOARD_PROVIDER      "package.module:Class"   status provider instead of TelemetryProvider   (simulation only)
  DASHBOARD_EXTRA_STATIC  folder served below /ext-static/                                         (simulation only)
  DASHBOARD_EXTENSION     "package.module" with ROUTES (and optionally page_filter)                (simulation only)

In live mode the three simulation-only variables are ignored (and listed in ``ignored``), so a stray variable on
the vehicle cannot change the dashboard. With nothing set the server behaves exactly as before: no provider swap,
no extra folder, no extra route and the page bytes unchanged (``page_filter`` returns None).

An extension module may define

  ROUTES = [("GET", "/api/sim/map", handler), ("POST", "/api/sim/map/*", handler), ...]
      handler(request_handler, parsed_url) writes the whole response. A pattern ending in "*" is a prefix.
      Extension routes are tried only when the dashboard itself has no route for the path, so they can add
      paths but never replace one (STOP, control, status keep their own code).
  page_filter(text) -> text
      changes the page (index.html) before it is versioned and served, e.g. to add a script of the extra folder.
"""
from __future__ import annotations

import importlib
import os
import re
from dataclasses import dataclass, field
from pathlib import Path

VEHICLE_KINDS = ("differential", "front_steer")
MODES = ("live", "simulation")
SIM_ONLY = ("DASHBOARD_PROVIDER", "DASHBOARD_EXTRA_STATIC", "DASHBOARD_EXTENSION")
_MODULE = re.compile(r"[A-Za-z_][A-Za-z0-9_]*(\.[A-Za-z_][A-Za-z0-9_]*)*")
_METHODS = ("GET", "POST", "DELETE")


class SettingsError(ValueError):
    """A dashboard setting that cannot be used. The server does not start with it."""


@dataclass(frozen=True)
class DashboardSettings:
    mode: str = "live"
    vehicle_kind: str = "differential"
    provider: str | None = None
    extra_static: Path | None = None
    extension: str | None = None
    ignored: tuple = field(default=())

    @property
    def simulation(self) -> bool:
        return self.mode == "simulation"

    @classmethod
    def from_env(cls, env=None) -> "DashboardSettings":
        env = os.environ if env is None else env
        mode = (env.get("DASHBOARD_MODE") or "live").strip()
        if mode not in MODES:
            raise SettingsError(f"DASHBOARD_MODE {mode!r}: expected one of {MODES}")
        vehicle = (env.get("DASHBOARD_VEHICLE") or "differential").strip()
        if vehicle not in VEHICLE_KINDS:
            raise SettingsError(f"DASHBOARD_VEHICLE {vehicle!r}: expected one of {VEHICLE_KINDS}")
        given = {name: (env.get(name) or "").strip() for name in SIM_ONLY}
        if mode != "simulation":
            return cls(mode=mode, vehicle_kind=vehicle, ignored=tuple(name for name in SIM_ONLY if given[name]))
        provider = given["DASHBOARD_PROVIDER"] or None
        if provider is not None:
            module, sep, attribute = provider.partition(":")
            if not sep or not _MODULE.fullmatch(module) or not attribute.isidentifier():
                raise SettingsError(f"DASHBOARD_PROVIDER {provider!r}: expected 'package.module:Class'")
        extension = given["DASHBOARD_EXTENSION"] or None
        if extension is not None and not _MODULE.fullmatch(extension):
            raise SettingsError(f"DASHBOARD_EXTENSION {extension!r}: expected 'package.module'")
        extra = given["DASHBOARD_EXTRA_STATIC"] or None
        extra_path = None
        if extra is not None:
            extra_path = Path(extra).resolve()
            if not extra_path.is_dir():
                raise SettingsError(f"DASHBOARD_EXTRA_STATIC {extra!r} is not a folder")
        return cls(mode=mode, vehicle_kind=vehicle, provider=provider, extra_static=extra_path, extension=extension)

    def public(self) -> dict:
        """What the page may know (GET /api/dashboard/config): names only, no paths or secrets."""
        return {"runtime_mode": self.mode, "vehicle_kind": self.vehicle_kind,
                "provider": self.provider.rpartition(":")[2] if self.provider else None,
                "extension": self.extension, "extra_static": self.extra_static is not None}


def load_provider(settings: DashboardSettings, config):
    """The configured provider instance, or None for the default TelemetryProvider."""
    if not settings.provider:
        return None
    module_name, _, attribute = settings.provider.partition(":")
    cls = getattr(importlib.import_module(module_name), attribute)
    return cls(config)


def load_extension(settings: DashboardSettings):
    """The extension module, or None."""
    return importlib.import_module(settings.extension) if settings.extension else None


class RouteTable:
    """Extra routes of an extension: exact paths and prefixes ("/api/sim/map/*")."""

    def __init__(self, routes=()):
        self.exact: dict[tuple[str, str], object] = {}
        self.prefix: list[tuple[str, str, object]] = []
        for entry in routes:
            method, pattern, handler = entry
            method = str(method).upper()
            if method not in _METHODS or not isinstance(pattern, str) or not pattern.startswith("/") or not callable(handler):
                raise SettingsError(f"extension route {entry!r}: expected (GET|POST|DELETE, '/path', handler)")
            if pattern.endswith("*"):
                self.prefix.append((method, pattern[:-1], handler))
            else:
                self.exact[(method, pattern)] = handler
        self.prefix.sort(key=lambda item: -len(item[1]))

    def __len__(self):
        return len(self.exact) + len(self.prefix)

    def find(self, method: str, path: str):
        method = "GET" if method == "HEAD" else method
        handler = self.exact.get((method, path))
        if handler is not None:
            return handler
        for route_method, start, route_handler in self.prefix:
            if route_method == method and path.startswith(start):
                return route_handler
        return None


def routes_of(extension) -> RouteTable:
    return RouteTable(getattr(extension, "ROUTES", ()) if extension is not None else ())


def page_filter(settings: DashboardSettings, extension=None):
    """A function that changes the page text for these settings, or None when the page stays as it is."""
    steps = []
    if settings.simulation:
        steps.append(lambda text: text.replace('data-runtime-mode="live"', 'data-runtime-mode="simulation"', 1)
                     .replace('<div hidden="" class="simulation-banner"', '<div class="simulation-banner"', 1))
    if settings.vehicle_kind != "differential":
        kind = settings.vehicle_kind
        steps.append(lambda text: text.replace("<html ", f'<html data-vehicle-kind="{kind}" ', 1))
    extra = getattr(extension, "page_filter", None) if settings.simulation else None
    if callable(extra):
        steps.append(extra)
    if not steps:
        return None

    def apply(text: str) -> str:
        for step in steps:
            text = step(text)
        return text
    return apply
