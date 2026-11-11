"""Live rclpy smoke test of executor_main (SHADOW). Not a unit test: run inside a ROS 2 container, e.g.

    docker run --rm --network none --name ubts-rowexec-rossmoke -v <worktree>:/work:ro -w /work \
        --entrypoint /bin/bash ubts-native-test:py310-scipy -c \
        '. /opt/ros/iron/setup.bash && python3 -B tools/row_executor/tests/live_ros_smoke.py'

It starts the REAL entry point (python3 -m row_executor.executor_main --shadow-auto, source SHA check ON) as a
subprocess, publishes synthetic sensor data like the real drivers (/scan: RELIABLE KeepLast(10) like
sllidar_ros2, 3240 beams, 10 Hz, 0 = no return; /imu/data_raw: sensor-data QoS, 33 Hz; /rover/control/requested:
String JSON 2 Hz), reads the operator page status with the token, presses STOP through the page, sends SIGTERM and
checks a clean exit and the JSONL log. No bridge socket exists (SHADOW). Prints one JSON result; exit 0 = pass.
"""
import json
import os
import signal
import subprocess
import sys
import tempfile
import time
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
sys.path.insert(0, TOOLS)
sys.dont_write_bytecode = True

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import Imu, LaserScan  # noqa: E402
from std_msgs.msg import String  # noqa: E402

import rowexec_testlib as L  # noqa: E402
from row_executor import config as C  # noqa: E402

PORT = 18097
RUN_S = 14.


def main():
    tmp = tempfile.mkdtemp(prefix='rowexec-smoke-')
    logs = os.path.join(tmp, 'logs')
    cfg = C.ExecutorConfig(bind_host='127.0.0.1', bind_port=PORT)
    cfg_path = os.path.join(tmp, 'config.json')
    with open(cfg_path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write(C.dumps(cfg))
    env = dict(os.environ, PYTHONPATH=TOOLS + os.pathsep + os.environ.get('PYTHONPATH', ''),
               PYTHONDONTWRITEBYTECODE='1', PYTHONUNBUFFERED='1')
    proc = subprocess.Popen([sys.executable, '-B', '-s', '-m', 'row_executor.executor_main', '--config', cfg_path,
                             '--log-dir', logs, '--shadow-auto'], env=env, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True)
    token_path = os.path.join(logs, 'operator_token.txt')
    deadline = time.monotonic() + 90.
    while not os.path.exists(token_path) and time.monotonic() < deadline and proc.poll() is None:
        time.sleep(.2)                        # executor startup (source SHA check, imports) on a loaded host
    startup_s = round(90. - (deadline - time.monotonic()), 1)
    time.sleep(1.)
    rclpy.init()
    node = rclpy.create_node('rowexec_smoke_publisher')
    scan_pub = node.create_publisher(LaserScan, '/scan', QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                                                                    reliability=ReliabilityPolicy.RELIABLE))
    imu_pub = node.create_publisher(Imu, '/imu/data_raw', qos_profile_sensor_data)
    human_pub = node.create_publisher(String, '/rover/control/requested', 1)
    ranges = L.raw_scan(L.orchard_trunks()).astype(np.float32).tolist()
    started = time.monotonic()
    next_scan = next_imu = next_human = started
    sent = dict(scan=0, imu=0, human=0)
    status = None
    token = open(token_path, encoding='utf-8').read().strip() if os.path.exists(token_path) else None
    base = 'http://127.0.0.1:%d' % PORT

    def get_status():
        req = urllib.request.Request(base + '/api/status', headers={'X-Row-Token': token})
        return json.loads(urllib.request.urlopen(req, timeout=3).read())

    while time.monotonic() - started < RUN_S:
        now = time.monotonic()
        if status is None and token and now - started >= RUN_S - 1.:
            status = get_status()               # while the sensors are still publishing
        stamp = node.get_clock().now().to_msg()
        if now >= next_imu:
            m = Imu()
            m.header.stamp, m.header.frame_id = stamp, 'imu_link'
            m.linear_acceleration.z = 9.81
            imu_pub.publish(m)
            sent['imu'] += 1
            next_imu += 1 / 33.
        if now >= next_scan:
            m = LaserScan()
            m.header.stamp, m.header.frame_id = stamp, 'laser'
            m.angle_min, m.angle_increment = L.RAW_AMIN, L.RAW_INC
            m.angle_max = L.RAW_AMIN + (L.BEAMS - 1) * L.RAW_INC
            m.range_min, m.range_max, m.scan_time = .05, 30., .1
            m.ranges = ranges
            scan_pub.publish(m)
            sent['scan'] += 1
            next_scan += .1
        if now >= next_human:
            human_pub.publish(String(data=json.dumps({'left': 0, 'right': 0})))
            sent['human'] += 1
            next_human += .5
        time.sleep(.002)
    stop_result = None
    if token:
        req = urllib.request.Request(base + '/api/stop', data=b'{}', method='POST', headers={
            'X-Row-Token': token, 'Origin': base, 'Content-Type': 'application/json'})
        stop_result = json.loads(urllib.request.urlopen(req, timeout=3).read())
    node.destroy_node()
    rclpy.shutdown()
    proc.send_signal(signal.SIGTERM)
    try:
        out, _ = proc.communicate(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()
        out, _ = proc.communicate()
    events, ticks, states = [], 0, set()
    for name in sorted(os.listdir(logs)) if os.path.isdir(logs) else []:   # absent: the executor refused to start
        if name.endswith('.jsonl'):
            for line in open(os.path.join(logs, name), encoding='utf-8'):
                rec = json.loads(line)
                if rec['k'] == 'event':
                    events.append(rec['event'])
                else:
                    ticks += 1
                    states.add(rec['state'])
    counts = (status or {}).get('counts', {})
    checks = dict(
        exit_code_0=proc.returncode == 0,
        token_written=token is not None,
        scans_processed=counts.get('scans_processed', 0) > 50,
        imu_fed=counts.get('imu_fed', 0) > 200,
        lidar_fresh=(status or {}).get('lidar_age') is not None and status['lidar_age'] <= cfg.lidar_max_age_s,
        mode_shadow=(status or {}).get('mode') == 'SHADOW',
        reached_active='ACTIVE' in states,
        human_seen=((status or {}).get('scan') or {}).get('human') == [0, 0],
        stop_shadow=bool(stop_result and stop_result.get('shadow') and stop_result.get('stop_sent') is None),
        log_ticks=ticks > 100,
        shutdown_event='shutdown' in events and 'config' in events,
        no_bridge=(status or {}).get('bridge') is None)
    result = dict(ok=all(checks.values()), checks=checks, startup_s=startup_s, sent=sent, counts=counts,
                  stamper=(status or {}).get('stamper'), states=sorted(states), events=sorted(set(events)),
                  log_ticks=ticks, returncode=proc.returncode, stdout_tail=out.splitlines()[-8:])
    print(json.dumps(result, indent=1))
    return 0 if result['ok'] else 1


if __name__ == '__main__':
    sys.exit(main())
