from typing import List, Optional
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3
from builtin_interfaces.msg import Duration

from .parking_spot import ParkingSpot


COLOR_AVAILABLE = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
COLOR_NEAR = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
COLOR_SELECTED = ColorRGBA(r=0.2, g=0.4, b=1.0, a=1.0)
COLOR_FAR = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)


class MarkerPublisher:

    def __init__(self, node: Node, topic: str = '/parking_spot_markers'):
        self.node = node
        self.publisher = node.create_publisher(MarkerArray, topic, 10)
        self.namespace = 'parking_spots'

    def publish_all_spots(
        self,
        spots: List[ParkingSpot],
        nearest_id: Optional[int] = None,
        selected_id: Optional[int] = None,
        robot_x: Optional[float] = None,
        robot_y: Optional[float] = None,
    ):
        marker_array = MarkerArray()

        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        for i, spot in enumerate(spots):
            color = self._get_color(i, spot, nearest_id, selected_id, robot_x, robot_y)

            arrow = self._create_arrow_marker(i, spot, color)
            marker_array.markers.append(arrow)

            text = self._create_text_marker(i, spot, color)
            marker_array.markers.append(text)

        self.publisher.publish(marker_array)

    def _get_color(
        self,
        idx: int,
        spot: ParkingSpot,
        nearest_id: Optional[int],
        selected_id: Optional[int],
        robot_x: Optional[float],
        robot_y: Optional[float],
    ) -> ColorRGBA:
        if selected_id is not None and idx == selected_id:
            return COLOR_SELECTED
        if nearest_id is not None and idx == nearest_id:
            return COLOR_NEAR
        if robot_x is not None and robot_y is not None:
            dist = spot.distance_to(robot_x, robot_y)
            if dist > 1.0:
                return COLOR_FAR
        return COLOR_AVAILABLE

    def _create_arrow_marker(self, idx: int, spot: ParkingSpot, color: ColorRGBA) -> Marker:
        marker = Marker()
        marker.header.frame_id = spot.frame_id
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = self.namespace
        marker.id = idx * 2
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = spot.to_pose_msg()
        marker.scale = Vector3(x=0.5, y=0.08, z=0.08)
        marker.color = color
        marker.lifetime = Duration(sec=0, nanosec=0)
        return marker

    def _create_text_marker(self, idx: int, spot: ParkingSpot, color: ColorRGBA) -> Marker:
        marker = Marker()
        marker.header.frame_id = spot.frame_id
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = self.namespace + '_labels'
        marker.id = idx * 2 + 1
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose = spot.to_pose_msg()
        marker.pose.position.z += 0.4
        marker.scale = Vector3(x=0.0, y=0.0, z=0.15)
        marker.color = color
        marker.text = f"[{idx}] {spot.name}"
        marker.lifetime = Duration(sec=0, nanosec=0)
        return marker

    def clear_markers(self):
        marker_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        self.publisher.publish(marker_array)
