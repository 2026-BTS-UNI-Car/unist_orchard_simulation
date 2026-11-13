"""Status provider of the simulation dashboard (DASHBOARD_PROVIDER, simulation mode only).

A TelemetryProvider of the copied dashboard that reads the Gazebo stack's status files instead of the Jetson's
devices: the sensor contract's /runtime/simulation_status.json (sensors_alive, clock, scene, real-time factor) and
camera_driver_status.json. Hardware actions (fan, network mode, Docker, camera settings, IMU zero) answer
'simulation_only'. Recording goes only to the stack's own recorder (http://sim:8790 or http://127.0.0.1:8790 in the sim
network namespace): a copied environment with a vehicle recorder URL is refused.

Ported from the earlier simulation copy of the dashboard (dashboard_server/simulation.py, main 09cc932). Left out on
purpose: the earlier SIM occupancy map (sim_mapping, raw_sim_mapping, sensor_mapping, map history) and the old native
mapping proxies. The steering stack never fed them (no mapping sample, scan_count 0); the new page has no screen for
them. The steering stack's map is the dashboard's own live map.
"""
from __future__ import annotations

import os
import time

from dashboard_server.telemetry import TelemetryProvider, _load_json

SIM_STATUS_FILE = '/runtime/simulation_status.json'           # the sensor contract's file (SIM_STATUS_FILE overrides)
SIM_RECORDER_URLS = ('http://sim:8790', 'http://127.0.0.1:8790')
STATUS_FRESH_S = 3.0
# Fields of simulation_status.json the page may see (GET /api/sim/status). The file also has the virtual GPS origin,
# the IMU model and the callback timing: none of them leaves the server.
PUBLIC_SIM_FIELDS = ('schema', 'scene', 'sim_time', 'clock_alive', 'rtf_5s', 'sensors_alive', 'result_kind',
                     'field_validated')


class SimulationProvider(TelemetryProvider):
    def __init__(self, config_path=None):
        super().__init__(config_path)
        self.mode = 'simulation'
        self.sim_status_file = os.environ.get('SIM_STATUS_FILE') or SIM_STATUS_FILE

    # --- the Gazebo stack -----------------------------------------------------------------------------------------
    def _sim(self):
        data = _load_json(self.sim_status_file)
        updated = data.get('updated_at')
        fresh = isinstance(updated, (int, float)) and not isinstance(updated, bool) and \
            0 <= time.time() - updated <= STATUS_FRESH_S
        if not fresh:
            data['sensors_alive'] = {}
            data['clock_alive'] = False
        data['fresh'] = fresh
        return data

    def sim_status(self):
        """The public part of the stack status (GET /api/sim/status of ubts_sim_dashboard.extension)."""
        data = self._sim()
        out = {key: data.get(key) for key in PUBLIC_SIM_FIELDS}
        alive = out.get('sensors_alive')
        out['sensors_alive'] = {str(k): v is True for k, v in alive.items()} if isinstance(alive, dict) else {}
        updated = data.get('updated_at')
        out['age_s'] = round(time.time() - updated, 2) if isinstance(updated, (int, float)) and \
            not isinstance(updated, bool) else None
        out['fresh'] = data['fresh']
        out['ok'] = True
        out['simulated'] = True
        return out

    def _alive(self, key):
        alive = self._sim().get('sensors_alive')
        return isinstance(alive, dict) and alive.get(key) is True

    def _hostname(self):
        return 'UBTS local Gazebo simulation'

    def rover_status(self):
        return {**super().rover_status(), 'mode': 'simulation', 'simulated': True}

    def _memory(self):
        result = super()._memory()
        result.update(display_total=result.get('total'), display_percent=result.get('percent'),
                      display_label='Docker Desktop VM')
        return result

    def lidar_status(self):
        alive = self._alive('scan')
        return {'present': alive, 'state': 'simulated' if alive else 'waiting_for_simulation',
                'device': 'Gazebo RPLIDAR S2 (S2 format node)', 'test_supported': False, 'holders': [],
                'test_mode': 'ros2_subscribe_only'}

    def camera_status(self):
        alive = self._alive('camera')
        settings = self.camera_settings()['settings']
        modes = [{'resolution': settings['resolution'], 'width': settings['width'], 'height': settings['height'],
                  'fps': [settings['fps']], 'label': 'Gazebo render preset'}]
        return {'present': alive, 'state': 'simulated' if alive else 'waiting_for_simulation',
                'device': 'Gazebo RGB camera', 'holders': [], 'driver': self._camera_driver_status(),
                'settings': settings, 'supported_view_modes': modes, 'supported_view_modes_source': 'simulation'}

    def camera_settings(self):
        driver = self._camera_driver_status()
        width, height, fps = driver.get('width', 640), driver.get('height', 360), driver.get('fps', 15)
        return {'ok': True, 'settings': {'width': width, 'height': height, 'fps': fps,
                                         'resolution': f'{width}x{height}', 'rotate_180': False,
                                         'source': 'simulation_preset'}}

    # --- no hardware in the simulation ----------------------------------------------------------------------------
    def _disabled(self, *_, **__):
        return {'ok': False, 'state': 'simulation_only', 'error': 'hardware_action_unavailable_in_simulation'}

    set_camera_settings = _disabled
    set_network_mode = _disabled
    docker_container_action = _disabled
    set_fan_mode = _disabled
    set_fan_pwm = _disabled
    set_fan_preset = _disabled
    zero_imu = _disabled

    def fan_status(self):
        return {'supported': False, 'state': 'not_applicable', 'simulated': True}

    def network_control_status(self):
        return {'enabled': False, 'state': 'simulation_only'}

    def docker(self):
        return {'available': False, 'containers': [], 'state': 'isolated_simulation', 'socket_mounted': False}

    # --- recording: the stack's recorder only ---------------------------------------------------------------------
    def _sim_recorder(self):
        return self.recorder_api_url.rstrip('/') in SIM_RECORDER_URLS

    def recording_status(self):
        if not self._sim_recorder():
            return {'ok': False, 'active': None, 'state': 'unknown', 'simulated': True, 'status_fresh': False,
                    'status_source': 'unavailable', 'error': 'simulation_recorder_endpoint_required',
                    'archive_count': self._recording_archive_count()}
        return {**super().recording_status(), 'simulated': True}

    def start_recording(self, payload):
        if not self._sim_recorder():
            return {'ok': False, 'error': 'simulation_recorder_endpoint_required'}
        return super().start_recording(payload)

    def stop_recording(self):
        if not self._sim_recorder():
            return {'ok': False, 'error': 'simulation_recorder_endpoint_required'}
        return super().stop_recording()

    # --- status page ----------------------------------------------------------------------------------------------
    def _services(self, *args, **kwargs):
        services = super()._services(*args, **kwargs)
        services.pop('cloudflared', None)
        sim = self._sim()
        domain = os.environ.get('ROS_DOMAIN_ID')
        services['simulation'] = {'ok': sim.get('clock_alive') is True, 'state': 'gazebo',
                                  'domain': int(domain) if domain and domain.isdigit() else None,
                                  'scene': sim.get('scene'), 'rtf_5s': sim.get('rtf_5s')}
        return services

    def _default_sensors(self):
        result = super()._default_sensors()
        result.pop('mavlink', None)
        result.pop('encoder', None)
        return result
