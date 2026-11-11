"""UBTS field calibration analyser, drive parts (imported from 546fa8c for the front-steer branch).

Reads calibration recordings straight from a recording archive (no ROS) and measures the drive response with LiDAR
scan matching and the raw IMU (no wheel encoders, no GNSS, PWM is only the command):

  records.py                  readers: recorder archive layout, 10/3 full extract, 10/10 compact extract
  odometry.py                 offline point-to-line ICP odometry (the 10/10 lidar_motion.py method), windows, workers
  timeline.py                 motor status timeline (requested / MCU-reported duty, settled, ramp back-calculation)
  steps.py                    C1 standstill gyro, C2 breakaway ladder, C3 v(duty), C4 straight drift, C7 stop
  drive_calib.py              command line entry of the drive analysis (C1-C4, C7; JSON result)
  validate_drive_evidence.py  PC-only check of the drive numbers against drive-evidence.txt (old drivetrain)

Not imported from 546fa8c: the differential C5 arc gain and the C6 pivot ladder (left / right duty differences; the
front-steer vehicle has no such command), the differential config recommendation (recommend.py, report_ko.py,
field_quick_calib.py, make_field_config.py, run_field_calib.sh). C3b (rolldown) and C7 executor runs are read by
tools/row_executor/calib_analysis.py. The steering identification (B0-B7, F1-F5) is sim/ubts/steering/ident.

Dependencies: Python >= 3.10, numpy >= 1.21, scipy >= 1.8 (cKDTree, odometry.py only; imported when the ICP runs).
Nothing here opens a device, a socket or ROS; the archive is only read.
"""
