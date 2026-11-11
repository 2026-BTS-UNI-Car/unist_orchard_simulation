"""Readable ROS views of the actual single-plane scan and planned 2-D path."""
import math
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


class SensorVisualization:
    def __init__(self, node, *, scan=True, mission=True):
        self.node = node
        self.publisher = node.create_publisher(MarkerArray, '/sim/visualization', 2) if scan else None
        # A plan changes infrequently. Keep its latest value for RViz clients
        # that connect or restart after the plan was published.
        path_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_publisher = node.create_publisher(Path, '/sim/mission_path', path_qos) if mission else None
        self.last_path = None

    def scan(self, msg):
        if self.publisher is None: return
        rays = Marker(); rays.header = msg.header; rays.ns = 'scan_plane'; rays.id = 0
        rays.type = Marker.LINE_LIST; rays.action = Marker.ADD
        rays.pose.orientation.w = 1.; rays.scale.x = .012
        rays.color.r = 1.; rays.color.g = .08; rays.color.b = .06; rays.color.a = .25
        rays.lifetime.sec = 1
        # Visualization is decimated; /scan still carries all 3,200 measurements.
        for i in range(0, len(msg.ranges), max(1, len(msg.ranges)//120)):
            r = msg.ranges[i]
            if not math.isfinite(r) or not msg.range_min <= r < msg.range_max: continue
            angle = msg.angle_min+i*msg.angle_increment
            rays.points.extend([Point(), Point(x=float(r*math.cos(angle)), y=float(r*math.sin(angle)), z=0.)])
        self.publisher.publish(MarkerArray(markers=[rays]))

    def mission(self, status, *, borrow_path=False):
        if self.path_publisher is None: return
        points = status.get('path', [])
        if borrow_path:
            # Internal path snapshots are tuples of numeric tuples. Validate a
            # newly supplied object once; subsequent captures retain it safely.
            if self.last_path is None or self.last_path[1] is not points:
                if type(points) is not tuple or any(type(point) is not tuple or len(point)!=2
                        or any(type(value) not in (int,float) for value in point) for point in points):
                    raise TypeError('borrowed_path_must_be_immutable_xy')
            signature = (status.get('state'), points)
        else:
            # Ordinary callers may mutate an existing list between calls.
            signature = (status.get('state'), tuple(tuple(point) for point in points))
        if signature == self.last_path: return
        self.last_path = signature
        path = Path(); path.header.frame_id = 'odom'; path.header.stamp = self.node.clock_stamp
        for x, y in points:
            pose = PoseStamped(); pose.header = path.header
            pose.pose.position.x = float(x); pose.pose.position.y = float(y); pose.pose.position.z = .08
            pose.pose.orientation.w = 1.; path.poses.append(pose)
        self.path_publisher.publish(path)
