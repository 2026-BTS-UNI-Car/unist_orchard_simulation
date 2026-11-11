"""ROS graph capture and truth-isolation rules of the steering G-SIM (A13 G graph part; design 2.6, 4.3, 8.3 P6a).

capture_graph(node) lists every topic with its publishers and subscribers (node names) through the rclpy graph API.
check_truth_isolation(graph) applies the rules of SIM_ACCEPTANCE A13 and design 2.6 / 4.3 to a captured graph:

  R1  no /odom topic publisher, no /tf publisher (only /tf_static), no /joint_states publisher
  R2  truth topics (/ground_truth/odom, /sim/truth/steering, /sim/truth/joint_states) are read only by the allowed
      plant-side and evaluation nodes (TRUTH_CONSUMERS: the Embree LiDAR, the sensor contract = virtual GPS and IMU
      model, the ESP emulator, the recorder, the evaluation / check node, the gz bridge that publishes them)
  R3  /sim/scan (the 3200-beam SIM scan) is read only by the clip node and the S2 format node; /sim/scan_clipped
      (the clip node's output, field_world scenes) only by the S2 format node
  R4  /sim/act/* are written only by the ESP emulator (and read by the gz bridge)
  R5  no old differential command path: no /sim/cmd_vel, /sim/wheel_odom; no SimMotor control socket topics
  R6  (next phase, prepared) the executor node subscribes only /scan, /imu/data_raw, /rover/control/requested
      (check_executor_subscriptions). Judged as registered (SIM_ACCEPTANCE A13): any other topic fails, also the
      ROS infrastructure topics /clock, /parameter_events and /rosout. Those are listed apart (infrastructure) with
      the verdict an amendment that allows them would give (passed_if_infrastructure_allowed; a SIM clock mode
      executor needs /clock, which carries time only); no amendment is registered (2026-11-09 review RG-07).

Node names are matched without the namespace. The rules are pure functions on a dict, so they are tested without
ROS. Placeholder-based SIM results, not field validation.
"""
from __future__ import annotations

TRUTH_TOPICS = ('/ground_truth/odom', '/sim/truth/steering', '/sim/truth/joint_states')
CLIP_NODE = 'ubts_steer_scan_clip'               # steering/scan_unobserved_clip.py (plant side, reads the truth pose)
S2_NODE = 'ubts_steer_s2_format'                 # the S2 format node of the next phase
TRUTH_CONSUMERS = ('ubts_cpu_lidar', 'ubts_steer_sensor_contract', 'ubts_steer_esp_emulator', 'ubts_data_recorder',
                   'ubts_steer_p6a_check', 'ros_gz_bridge', CLIP_NODE)
SIM_SCAN_CONSUMERS = (CLIP_NODE, S2_NODE, 'ros_gz_bridge')
CLIPPED_SCAN_CONSUMERS = (S2_NODE,)
ACT_PREFIX = '/sim/act/'
ACT_PUBLISHERS = ('ubts_steer_esp_emulator',)
FORBIDDEN_TOPICS = ('/sim/cmd_vel', '/sim/wheel_odom', '/sim/wheel_tf')
EXECUTOR_NODE = 'ubts_row_executor'
EXECUTOR_ALLOWED = ('/scan', '/imu/data_raw', '/rover/control/requested')
INFRASTRUCTURE_TOPICS = ('/parameter_events', '/clock', '/rosout')
SCHEMA = 'ubts.steer_ros_graph.v1'


def capture_graph(node):
    """{topic: {'types': [...], 'publishers': [names], 'subscribers': [names]}} and the node list."""
    topics = {}
    for name, types in node.get_topic_names_and_types():
        pubs = sorted({info.node_name for info in node.get_publishers_info_by_topic(name)})
        subs = sorted({info.node_name for info in node.get_subscriptions_info_by_topic(name)})
        topics[name] = dict(types=list(types), publishers=pubs, subscribers=subs)
    nodes = sorted({n for n, _ in node.get_node_names_and_namespaces()})
    return dict(schema=SCHEMA, topics=topics, nodes=nodes)


def _base(name):
    return name.rsplit('/', 1)[-1]


def _allowed(name, allowed):
    """Node name allowed: exact, or the ros_gz_bridge helper names (parameter_bridge launches 'ros_gz_bridge')."""
    base = _base(name)
    return base in allowed or any(base.startswith(a + '_') for a in allowed if a == 'ros_gz_bridge')


def check_truth_isolation(graph, observer=None):
    """Findings of rules R1-R5 (module docstring) on a captured graph. observer: the capturing node's own name
    (ignored as a subscriber only if it is in TRUTH_CONSUMERS anyway). Returns dict(passed, findings, details)."""
    topics = graph['topics']
    findings = []

    def pubs(topic):
        return topics.get(topic, {}).get('publishers', [])

    def subs(topic):
        return topics.get(topic, {}).get('subscribers', [])
    if pubs('/odom'):
        findings.append(dict(rule='R1', topic='/odom', publishers=pubs('/odom')))
    if pubs('/tf'):
        findings.append(dict(rule='R1', topic='/tf', publishers=pubs('/tf')))
    if pubs('/joint_states'):
        findings.append(dict(rule='R1', topic='/joint_states', publishers=pubs('/joint_states')))
    for topic in TRUTH_TOPICS:
        bad = [n for n in subs(topic) if not _allowed(n, TRUTH_CONSUMERS)]
        if bad:
            findings.append(dict(rule='R2', topic=topic, subscribers=bad))
    for topic, allowed in (('/sim/scan', SIM_SCAN_CONSUMERS), ('/sim/scan_clipped', CLIPPED_SCAN_CONSUMERS)):
        bad = [n for n in subs(topic) if not _allowed(n, allowed)]
        if bad:
            findings.append(dict(rule='R3', topic=topic, subscribers=bad))
    for topic, info in topics.items():
        if topic.startswith(ACT_PREFIX):
            bad = [n for n in info['publishers'] if not _allowed(n, ACT_PUBLISHERS)]
            if bad:
                findings.append(dict(rule='R4', topic=topic, publishers=bad))
    for topic in FORBIDDEN_TOPICS:
        if pubs(topic) or subs(topic):
            findings.append(dict(rule='R5', topic=topic, publishers=pubs(topic), subscribers=subs(topic)))
    details = dict(truth_topics={t: dict(publishers=pubs(t), subscribers=subs(t)) for t in TRUTH_TOPICS},
                   sim_scan=dict(publishers=pubs('/sim/scan'), subscribers=subs('/sim/scan')),
                   sim_scan_clipped=dict(publishers=pubs('/sim/scan_clipped'), subscribers=subs('/sim/scan_clipped')),
                   scan=dict(publishers=pubs('/scan'), subscribers=subs('/scan')),
                   tf=dict(publishers=pubs('/tf'), subscribers=subs('/tf')),
                   tf_static=dict(publishers=pubs('/tf_static')),
                   odom=dict(publishers=pubs('/odom'), subscribers=subs('/odom')),
                   act={t: i['publishers'] for t, i in topics.items() if t.startswith(ACT_PREFIX)})
    return dict(passed=not findings, findings=findings, details=details, rules='R1-R5 (p6a_graph docstring)')


def check_executor_subscriptions(graph, node_name=EXECUTOR_NODE, allowed=EXECUTOR_ALLOWED,
                                 infrastructure=INFRASTRUCTURE_TOPICS):
    """R6 (next phase): topics the executor node subscribes. passed None when the node is not in the graph.

    passed is the registered rule (SIM_ACCEPTANCE A13): every subscribed topic must be in the allow list; the
    infrastructure topics (/clock, /parameter_events, /rosout) are not exempt. They are listed apart, with the
    verdict an amendment allowing them would give (passed_if_infrastructure_allowed, report only)."""
    subscribed = sorted(t for t, info in graph['topics'].items() if any(_base(n) == node_name
                                                                         for n in info['subscribers']))
    if node_name not in [_base(n) for n in graph.get('nodes', [])] and not subscribed:
        return dict(passed=None, node=node_name, note='executor node not running (next phase)', subscribed=[])
    infra = [t for t in subscribed if t in infrastructure]
    outside = [t for t in subscribed if t not in allowed]
    other = [t for t in outside if t not in infrastructure]
    return dict(passed=not outside, node=node_name, subscribed=subscribed, not_allowed=outside,
                infrastructure=infra, not_allowed_without_infrastructure=other,
                passed_if_infrastructure_allowed=not other, allowed=list(allowed),
                rule='registered A13 allow list; infrastructure topics are not exempt (no amendment registered)')
