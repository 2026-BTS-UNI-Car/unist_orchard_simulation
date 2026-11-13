"""Extension module of the simulation dashboard (DASHBOARD_EXTENSION, simulation mode only).

ROUTES (the dashboard tries them only for paths it does not serve itself, so STOP, control and status keep their code):
  GET /api/sim/status   the public part of the stack status: scene, simulation time, clock, real-time factor,
                        sensors_alive, result kind (provider.SimulationProvider.sim_status; no GPS origin, no truth)
page_filter: adds /ext-static/sim-info.js (DASHBOARD_EXTRA_STATIC) to the page; the script writes the scene and the
real-time factor into the dashboard's simulation banner.
"""
from __future__ import annotations

SCRIPT_TAG = '<script defer src="/ext-static/sim-info.js"></script>'


def sim_status(handler, parsed):
    provider = handler.provider
    status = getattr(provider, 'sim_status', None)
    if status is None:
        handler._send_json({'ok': False, 'error': 'simulation_provider_required'}, status=503)
        return
    handler._send_json(status())


ROUTES = [('GET', '/api/sim/status', sim_status)]


def page_filter(text: str) -> str:
    if SCRIPT_TAG in text or '</body>' not in text:
        return text
    return text.replace('</body>', SCRIPT_TAG + '\n</body>', 1)
