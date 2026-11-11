"""UBTS row executor (field weekend 2026-10-24): supervised, forward-only row following.

Pure-Python core shared by the live ROS node (later) and the offline shadow replay:
  config.py         ExecutorConfig (frozen, strict JSON, validate(), feasibility rules)
  guard.py          OutputGuard (last check before any duty leaves the executor), Backstop
  core.py           ScanAdapter, ImuYawRate, Runner (vendored SIM estimator + controller)
  bridge_client.py  BridgeClient for the existing ESP bridge socket (/run/ubts-rover/control.sock)
  vendor/           SIM estimator/controller/single_row/scan_corridor, byte-identical (vendor/SOURCE_MANIFEST.json)
  single_row_sim.py SIM runner of the single_row mode (real executor in the loop, raster corridors; SINGLE_ROW_MODE.txt)
  sim_paths.py      external locations of the SIM-side tools (environment variable or config file; steering branch)

Nothing in this package sends a motor command by itself: the Runner only returns Decisions; a
caller decides whether a BridgeClient is used at all (ExecutorConfig.motor_output_enabled, default
False). No module here imports ROS. See OPERATIONS.txt for the protocol and recovery rules.

Steering branch copy (2026-11-09): imported from the single-row work 546fa8c without the turn / replay tools; the real S2
scan format lives in sim/ubts/steering/s2_format.py. Changes: docs/steering/EXECUTOR_IMPORT_KO.md.
"""
