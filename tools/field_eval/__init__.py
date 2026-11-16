"""UBTS field evaluation tool (offline): the lateral error of one row run against a reference centre line.

Built for the real vehicle (docs/steering/FIELD_EVAL_METHOD_KO.md): the reference is made with a method that is
independent of the executor's own estimate (the LiDAR corridor band), namely a map of the row's poles or trunks
measured with a tape (or a survey) and an offline map-based LiDAR localisation.

  refmap.py      reference map (JSON / CSV, row-local metres; no geographic coordinates), centre line, row end
  exec_log.py    executor JSONL reader (ticks, events, config, config_digest, wall-clock mapping)
  sources.py     scan + IMU sources: F-SIM capture, recorder archive sidecar (mapping_source/sensors.jsonl.gz), rosbag2
  bag_reader.py  rosbag2 sqlite3 / MCAP reader with a CDR decoder for LaserScan and Imu (standard library)
  localise.py    map-based localisation: clusters -> circle centres -> association -> SE(2) least squares, IMU prediction
  metrics.py     the SIM metrics (README 7, SYSTEM_REFERENCE 8) on the reference trajectory; executor estimate error
  svg_plot.py    SVG figures without matplotlib
  evaluate.py    one run -> REPORT.json, TRAJECTORY.csv, fig_*.svg
  cli.py         command line (python -B tools/field_eval/cli.py evaluate ...)
  sim_capture.py PC only: F-SIM runs of the follow matrices with the executor's scan / IMU inputs saved
  sim_verify.py  PC only: the tool on an F-SIM run compared with the SIM truth

Dependencies: Python >= 3.10 and numpy (the Jetson's). sim_capture.py and sim_verify.py need the SIM tree.
Not part of the executor package (tools/row_executor) and never imported by it. Truth is read only by sim_verify.py.
"""
SCHEMA_VERSION = 1
