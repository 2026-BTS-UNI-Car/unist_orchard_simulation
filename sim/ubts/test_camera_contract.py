"""The camera has one ROS publisher and a valid optical frame at its source."""
import ast
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT.parent.parent / 'dashboard'))
from generate_world import generate


class CameraContractTest(unittest.TestCase):
    def test_source_camera_declares_ros_optical_frame(self):
        config = yaml.safe_load((ROOT / 'rover.yaml').read_text())
        tree = ET.fromstring(generate(config))
        sensor = tree.find(".//sensor[@type='camera']")
        self.assertEqual(sensor.findtext('camera/optical_frame_id'), 'camera_link_optical')
        self.assertEqual(sensor.findtext('topic'), '/sim/camera')
        self.assertEqual(float(sensor.findtext('update_rate')), config['camera']['hz'])
        self.assertEqual(int(sensor.findtext('camera/image/width')), config['camera']['width'])

    def test_single_direct_bridge_for_each_camera_topic(self):
        bridges = yaml.safe_load((ROOT / 'bridge.yaml').read_text())
        expected = {'sensor_msgs/msg/Image': ('/sim/camera', '/camera/image_raw'),
                    'sensor_msgs/msg/CameraInfo': ('/sim/camera_info', '/camera/camera_info')}
        for msg_type, (gz_topic, ros_topic) in expected.items():
            entries = [entry for entry in bridges if entry['ros_type_name'] == msg_type]
            self.assertEqual(len(entries), 1)
            self.assertEqual(entries[0]['gz_topic_name'], gz_topic)
            self.assertEqual(entries[0]['ros_topic_name'], ros_topic)
            self.assertEqual(entries[0]['direction'], 'GZ_TO_ROS')
        adapter = ast.parse((ROOT / 'adapter.py').read_text())
        publisher_topics = [call.args[1].value for call in ast.walk(adapter)
                            if isinstance(call, ast.Call) and isinstance(call.func, ast.Attribute)
                            and call.func.attr == 'create_publisher' and len(call.args) >= 2
                            and isinstance(call.args[1], ast.Constant)]
        self.assertNotIn('/camera/image_raw', publisher_topics)
        self.assertNotIn('/camera/camera_info', publisher_topics)

    def test_adapter_does_not_add_a_camera_payload_consumer_for_liveness(self):
        adapter = ast.parse((ROOT / 'adapter.py').read_text())
        subscribed_loop_topics = [value.elts[1].value for value in ast.walk(adapter)
            if isinstance(value,ast.Tuple) and len(value.elts)==3
            and isinstance(value.elts[1],ast.Constant)]
        self.assertNotIn('/camera/image_raw',subscribed_loop_topics)
        self.assertNotIn('/camera/camera_info',subscribed_loop_topics)
        self.assertIn('/sim/imu',subscribed_loop_topics)
        self.assertIn('/sim/wheel_odom',subscribed_loop_topics)


if __name__ == '__main__':
    unittest.main()
