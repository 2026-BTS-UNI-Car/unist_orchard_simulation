"""The SIM LiDAR bridge (dashboard/scripts/lidar_ws_bridge.py, simulation form) sends the original LaserScan for
profile=mapping, as the dashboard repository's bridge does, so the dashboard's own live map
(dashboard_server/source_live_mapping.py) can use SIM scans (SIM_DASHBOARD_KO.md section 8). No ROS needed."""
import json
import math
from pathlib import Path
import sys
import types
import unittest

DASHBOARD = Path(__file__).resolve().parents[2] / 'dashboard'
sys.path.insert(0, str(DASHBOARD))

from scripts import lidar_ws_bridge as L  # noqa: E402


def fake_scan(n=3240, frame='laser'):
    ranges = [0.0] * n
    ranges[10], ranges[20], ranges[30] = 2.5, float('inf'), float('nan')
    stamp = types.SimpleNamespace(sec=12, nanosec=500000000)
    return types.SimpleNamespace(header=types.SimpleNamespace(stamp=stamp, frame_id=frame), ranges=ranges,
                                 intensities=[], angle_min=-math.pi, angle_increment=2 * math.pi / n,
                                 range_min=.05, range_max=30., scan_time=.1, time_increment=.1 / n)


class MappingProfileTest(unittest.TestCase):
    def test_raw_scan_keeps_every_range_and_the_geometry(self):
        raw = L.raw_scan_of(fake_scan())
        self.assertEqual(len(raw['ranges']), 3240)
        self.assertEqual(raw['ranges'][10], 2.5)
        self.assertIsNone(raw['ranges'][20])          # non-finite -> None (strict JSON)
        self.assertIsNone(raw['ranges'][30])
        self.assertEqual(raw['ranges'][0], 0.0)       # 0.0 = no return stays
        self.assertEqual(raw['header'], dict(stamp=dict(sec=12, nanosec=500000000), frame_id='laser'))
        self.assertAlmostEqual(raw['angle_min'], -math.pi)
        for key in ('angle_increment', 'range_min', 'range_max', 'scan_time', 'time_increment'):
            self.assertIn(key, raw)

    def test_mapping_message_is_what_the_live_map_reads(self):
        msg = fake_scan()
        state = L.ScanState()
        points = L.scan_to_points(msg, 900)
        state.update('laser', 12.5, points, 'ros2', L.raw_scan_of(msg))
        snap = state.snapshot()
        payload = L.mapping_payload(snap, points, 7)
        text = json.dumps(payload, allow_nan=False)
        back = json.loads(text)
        # source_live_mapping: packet type == kind, a stamp, and frame.get('raw_scan') not empty
        self.assertEqual((back['type'], back['stamp'], back['sequence']), ('scan', 12.5, 7))
        self.assertEqual(back['points'], [])
        self.assertEqual(back['source'], 'source_native_LaserScan')
        self.assertEqual(len(back['raw_scan']['ranges']), 3240)
        # the preview message of the same snapshot carries no raw scan
        preview = L.compact_scan_payload(snap, points, 8)
        self.assertNotIn('raw_scan', preview)
        self.assertTrue(preview['points'])

    def test_state_without_a_raw_scan(self):
        state = L.ScanState()
        state.update('laser', 1.0, [], 'ros2')
        self.assertEqual(state.snapshot()['raw_scan'], {})


if __name__ == '__main__':
    unittest.main()
