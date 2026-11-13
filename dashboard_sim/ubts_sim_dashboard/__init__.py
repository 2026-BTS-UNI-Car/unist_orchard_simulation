"""Simulation additions of the dashboard, plugged into its extension points (docs/steering/SIM_DASHBOARD_KO.md).

The dashboard itself (dashboard/: server and page) is a byte copy of the dashboard repository at one commit
(tools/steer_sim/sync_dashboard.py). This package holds what only the simulation needs and is selected with

  DASHBOARD_MODE=simulation
  DASHBOARD_PROVIDER=ubts_sim_dashboard.provider:SimulationProvider    status of the Gazebo sensors, no hardware actions
  DASHBOARD_EXTENSION=ubts_sim_dashboard.extension                     GET /api/sim/status and the page script tag
  DASHBOARD_EXTRA_STATIC=<this folder>/../static                       /ext-static/sim-info.js

The dashboard ignores the three variables in live mode, so nothing here can reach the vehicle dashboard.
"""
