import math
import time
from dataclasses import dataclass, field
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Quaternion, Point
from std_msgs.msg import Header


@dataclass
class ParkingSpot:
    name: str
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    timestamp: float = field(default_factory=time.time)
    frame_id: str = 'map'

    @property
    def yaw(self) -> float:
        siny_cosp = 2.0 * (self.qw * self.qz + self.qx * self.qy)
        cosy_cosp = 1.0 - 2.0 * (self.qy * self.qy + self.qz * self.qz)
        return math.atan2(siny_cosp, cosy_cosp)

    def to_dict(self) -> dict:
        return {
            'name': self.name,
            'x': self.x,
            'y': self.y,
            'z': self.z,
            'qx': self.qx,
            'qy': self.qy,
            'qz': self.qz,
            'qw': self.qw,
            'timestamp': self.timestamp,
            'frame_id': self.frame_id,
        }

    @classmethod
    def from_dict(cls, data: dict) -> 'ParkingSpot':
        return cls(
            name=data['name'],
            x=data['x'],
            y=data['y'],
            z=data.get('z', 0.0),
            qx=data.get('qx', 0.0),
            qy=data.get('qy', 0.0),
            qz=data.get('qz', 0.0),
            qw=data.get('qw', 1.0),
            timestamp=data.get('timestamp', 0.0),
            frame_id=data.get('frame_id', 'map'),
        )

    @classmethod
    def from_yaw(cls, name: str, x: float, y: float, yaw: float, z: float = 0.0) -> 'ParkingSpot':
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return cls(name=name, x=x, y=y, z=z, qx=0.0, qy=0.0, qz=qz, qw=qw)

    def to_pose_msg(self) -> Pose:
        pose = Pose()
        pose.position = Point(x=self.x, y=self.y, z=self.z)
        pose.orientation = Quaternion(x=self.qx, y=self.qy, z=self.qz, w=self.qw)
        return pose

    def to_initial_pose_msg(self, stamp, covariance=None) -> PoseWithCovarianceStamped:
        msg = PoseWithCovarianceStamped()
        msg.header = Header()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = stamp
        msg.pose.pose = self.to_pose_msg()

        if covariance is None:
            cov = [0.0] * 36
            cov[0] = 0.25   # x variance (0.5m std dev)
            cov[7] = 0.25   # y variance (0.5m std dev)
            cov[35] = 0.068  # yaw variance (~15 deg std dev)
        else:
            cov = covariance

        msg.pose.covariance = cov
        return msg

    def distance_to(self, x: float, y: float) -> float:
        return math.sqrt((self.x - x) ** 2 + (self.y - y) ** 2)

    def angle_diff_to(self, yaw: float) -> float:
        diff = self.yaw - yaw
        while diff > math.pi:
            diff -= 2.0 * math.pi
        while diff < -math.pi:
            diff += 2.0 * math.pi
        return diff
