"""Generated link children satisfy SDFormat's shared name namespace."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest
import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from assemble_world import assemble
from generate_world import generate


def assert_unique_link_children(root):
    for link in root.findall('.//link'):
        names = [child.get('name') for child in link if child.get('name') is not None]
        assert len(names) == len(set(names)), (link.get('name'), names)


def test_direct_generator_has_no_cross_type_link_name_collisions():
    config = yaml.safe_load((HERE/'rover.yaml').read_text())
    root = ET.fromstring(generate(config))
    assert_unique_link_children(root)
    collisions = {node.get('name') for node in root.findall('.//collision')}
    assert collisions
    # Renaming shape entities must not break a contact sensor or plugin reference.
    assert not root.findall('.//sensor/contact')
    for plugin in root.findall('.//plugin'):
        for parameter in plugin.iter():
            assert (parameter.text or '').strip().rsplit('::', 1)[-1] not in collisions


@pytest.mark.parametrize('backend', ['gazebo', 'embree'])
def test_assembled_sensor_fixture_keeps_unique_names_for_each_lidar_backend(backend, monkeypatch, tmp_path):
    config = yaml.safe_load((HERE/'rover.yaml').read_text())
    config['world']['preset'] = 'sensor_test'
    monkeypatch.setenv('UBTS_RENDER_ENGINE', 'ogre2')
    monkeypatch.setenv('UBTS_HEADLESS_RENDERING', '1')
    monkeypatch.setenv('UBTS_LIDAR_BACKEND', backend)
    root = ET.parse(assemble(config, tmp_path/'fixture.sdf')).getroot()
    assert_unique_link_children(root)
