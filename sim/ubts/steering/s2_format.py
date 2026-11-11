"""Real RPLIDAR S2 LaserScan format of the UBTS rover, and the SIM scan -> S2 conversion (design 2.3, 3.7; A5).

One module for both SIMs: the F-SIM executor-in-the-loop (tools/row_executor/sim_in_loop.py, which re-exports the
names below) and the G-SIM S2 format node. Imports only numpy and the standard library (no harness, no estimator,
no scipy); runs on Python 3.10, 3.12 and 3.13.

Real format (the 10/10 recordings; the executor's ScanAdapter undoes it):
  3240 beams in raw driver order, angle_min = float32(-pi) = RAW_AMIN, angle_increment = RAW_INC (about 0.111 deg),
  frame 'laser', 10 Hz, range_min 0.05 m, range_max 30 m, 0.0 = no return.
  Raw beam j looks at the body angle wrap(RAW_AMIN + j * RAW_INC + lidar_yaw_offset_deg). The default offset is
  179.08 deg (the 10/10 mount estimate, design 3.7; travel axis at -179.08 deg in the scan frame). The array is not
  reordered.

RealFormat (moved unchanged from sim_in_loop.py at 546fa8c, design 8.3 P4):
  SIM scan = SIM_BEAMS (3200) ranges over 2 pi in the BODY frame, beam k at body angle -pi + k * 2 pi / 3200.
  Each real beam takes the nearest SIM beam of its body angle (np.rint of the bin, index modulo 3200).
  A range that is finite and below 30 m is rounded to 1 mm (np.round, half to even, of r * 1000, then / 1000);
  everything else (NaN, +inf, -inf, >= 30 m) becomes 0.0 (no return). Values below 30 m pass otherwise unchanged,
  also 0 and negative numbers (there is no range_min clip). The output is float32 in raw driver order.
"""
import math

import numpy as np

RAW_AMIN = -3.1415927410125732              # float32(-pi): angle_min of the real driver
RAW_INC = .0019398535368964076              # angle_increment of the real driver (about 0.1111 deg; 3240 beams span
                                            # slightly more than 2 pi, about 360.11 deg)
BEAMS = 3240                                # beams per real scan
SIM_BEAMS = 3200                            # beams per SIM scan (body frame, 2 pi)
FRAME_ID = 'laser'                          # LaserScan header.frame_id of the real driver
SCAN_HZ = 10.                               # real scan rate
RANGE_MIN_M = .05                           # LaserScan range_min of the real driver
RANGE_MAX_M = 30.                           # LaserScan range_max of the real driver
DEFAULT_YAW_OFFSET_DEG = 179.08             # lidar_yaw_offset_deg default (config.ExecutorConfig, design 3.7)


class RealFormat:
    """SIM 3200-beam body-frame scan -> real S2 LaserScan ranges (raw order, 0 = no return)."""

    def __init__(self, offset_deg=179.08):
        body = RAW_AMIN + np.arange(BEAMS) * RAW_INC + math.radians(offset_deg)
        body = np.arctan2(np.sin(body), np.cos(body))
        self.index = np.rint((body + math.pi) / (2 * math.pi / SIM_BEAMS)).astype(np.int64) % SIM_BEAMS

    def convert(self, sim_ranges):
        r = np.asarray(sim_ranges, dtype=np.float64)[self.index]
        r = np.where(np.isfinite(r) & (r < 30.), np.round(r * 1000.) / 1000., 0.)
        return r.astype(np.float32)
