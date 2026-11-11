"""SIM stand-ins of the operator's measuring instruments (plant side; design 3.9 B1, B2, B3, B6).

In the field a person measures with a tape (B1) and reads a digital angle gauge on each front wheel (B2 centre,
B3 table, B6 wheel-side reference). In the SIM the instruments read the rig's true geometry and true wheel angles and
add their own errors; only the readings go into the record. The instrument errors are SIM assumptions (placeholders),
written into every reading so that the analysis knows them:

  tape           sd 3 mm per length (SIM_ASSUMED), reading rounded to 1 mm
  angle gauge    sd 0.05 deg per wheel (SIM_ASSUMED), reading rounded to 0.1 deg (a common digital gauge)

The random draws come from one numpy generator per instrument, seeded from the run seed (deterministic, A8).
"""
from __future__ import annotations

import math
import zlib

import numpy as np

TAPE_SD_M = .003
TAPE_RESOLUTION_M = .001
GAUGE_SD_DEG = .05
GAUGE_RESOLUTION_DEG = .1
TAPE_PURPOSE = 301
GAUGE_PURPOSE = 302
# B1 lengths the analysis needs (profile names of the true values in the rig)
B1_NAMES = ('wheelbase_m', 'kingpin_width_m', 'track_front_m', 'rear_axle_x_in_base_m', 'lidar_x_in_base_m',
            'lidar_y_in_base_m', 'front_bumper_x_in_base_m')


def _rng(seed, key, purpose):
    return np.random.default_rng(np.random.SeedSequence([int(seed), zlib.crc32(key.encode('utf-8')), int(purpose)]))


def _round(value, step):
    return round(round(value / step) * step, 6)


class Tape:
    """B1 tape measurements of the true geometry (profile values of the rig's profile view)."""

    def __init__(self, seed, key):
        self.rng = _rng(seed, key, TAPE_PURPOSE)

    def measure(self, profile):
        out = {}
        for name in B1_NAMES:
            true = float(profile.value(name))
            out[name] = _round(true + self.rng.normal(0., TAPE_SD_M), TAPE_RESOLUTION_M)
        return dict(values=out, instrument='tape', sd_m=TAPE_SD_M, resolution_m=TAPE_RESOLUTION_M,
                    note='SIM: true length + N(0, sd), rounded; field: tape measure (B1)')


class AngleGauge:
    """Digital angle gauge on each front wheel: reads the true wheel angles (deg, + left) with its own error."""

    def __init__(self, seed, key):
        self.rng = _rng(seed, key, GAUGE_PURPOSE)

    def read(self, true_left_rad, true_right_rad):
        left = math.degrees(true_left_rad) + self.rng.normal(0., GAUGE_SD_DEG)
        right = math.degrees(true_right_rad) + self.rng.normal(0., GAUGE_SD_DEG)
        return dict(left_deg=_round(left, GAUGE_RESOLUTION_DEG), right_deg=_round(right, GAUGE_RESOLUTION_DEG),
                    instrument='angle_gauge', sd_deg=GAUGE_SD_DEG, resolution_deg=GAUGE_RESOLUTION_DEG)
