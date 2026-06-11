import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException, ConnectivityException
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from go2_interfaces.srv import SetPoseFromSpot, GetPoseError, ListParkingSpots

from ..utils.parking_spot import ParkingSpot
from ..utils.storage import ParkingSpotStorage
from ..utils.marker_publisher import MarkerPublisher


class ParkingSpotLoaderNode(Node):

    def __init__(self):
        super().__init__('parking_spot_loader')

        self.declare_parameter('map_name', 'my_map')
        self.declare_parameter('base_directory', '')
        self.declare_parameter('auto_highlight_nearest', True)
        self.declare_parameter('error_feedback_rate', 5.0)
        self.declare_parameter('covariance_x', 0.25)
        self.declare_parameter('covariance_y', 0.25)
        self.declare_parameter('covariance_yaw', 0.068)

        self.map_name = self.get_parameter('map_name').get_parameter_value().string_value
        base_dir = self.get_parameter('base_directory').get_parameter_value().string_value
        self.auto_highlight = self.get_parameter('auto_highlight_nearest').get_parameter_value().bool_value
        feedback_rate = self.get_parameter('error_feedback_rate').get_parameter_value().double_value
        self.cov_x = self.get_parameter('covariance_x').get_parameter_value().double_value
        self.cov_y = self.get_parameter('covariance_y').get_parameter_value().double_value
        self.cov_yaw = self.get_parameter('covariance_yaw').get_parameter_value().double_value

        self.storage = ParkingSpotStorage(base_directory=base_dir)
        self.spots = self.storage.load_spots(self.map_name)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.feedback_pub = self.create_publisher(
            String, '/parking_spot_status', 10)

        self.marker_pub = MarkerPublisher(self, '/parking_spot_markers')

        self.set_pose_srv = self.create_service(
            SetPoseFromSpot, '/set_pose_from_spot', self.set_pose_callback)
        self.set_nearest_srv = self.create_service(
            Trigger, '/set_pose_from_nearest', self.set_nearest_callback)
        self.get_error_srv = self.create_service(
            GetPoseError, '/get_pose_error', self.get_error_callback)
        self.list_spots_srv = self.create_service(
            ListParkingSpots, '/list_parking_spots', self.list_spots_callback)

        self.current_odom_x = 0.0
        self.current_odom_y = 0.0
        self.current_odom_yaw = 0.0
        self.nearest_id = None
        self.selected_id = None

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        if feedback_rate > 0.0:
            period = 1.0 / feedback_rate
            self.feedback_timer = self.create_timer(period, self.feedback_loop)

        self.marker_timer = self.create_timer(2.0, self.publish_markers)

        self.get_logger().info(
            f'ParkingSpotLoader started. Map: {self.map_name}, '
            f'Loaded {len(self.spots)} spots.')
        if self.spots:
            names = [f"  [{i}] {s.name} ({s.x:.2f}, {s.y:.2f})" for i, s in enumerate(self.spots)]
            self.get_logger().info('Available spots:\n' + '\n'.join(names))

    def odom_callback(self, msg: Odometry):
        self.current_odom_x = msg.pose.pose.position.x
        self.current_odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_odom_yaw = math.atan2(siny_cosp, cosy_cosp)

    def set_pose_callback(self, request, response):
        spot_id = request.spot_id
        if spot_id < 0 or spot_id >= len(self.spots):
            response.success = False
            response.message = f'Invalid spot_id {spot_id}. Available: 0-{len(self.spots)-1}'
            return response

        spot = self.spots[spot_id]
        self._publish_initial_pose(spot)
        self.selected_id = spot_id

        response.success = True
        response.message = (
            f'Set initial pose from "{spot.name}" at '
            f'({spot.x:.2f}, {spot.y:.2f}, yaw={math.degrees(spot.yaw):.1f}deg)')
        response.x = spot.x
        response.y = spot.y
        response.theta = spot.yaw
        self.get_logger().info(response.message)
        return response

    def set_nearest_callback(self, request, response):
        if not self.spots:
            response.success = False
            response.message = 'No parking spots loaded'
            return response

        nearest_id = self._find_nearest_spot()
        if nearest_id is None:
            response.success = False
            response.message = 'Cannot determine nearest spot'
            return response

        spot = self.spots[nearest_id]
        self._publish_initial_pose(spot)
        self.selected_id = nearest_id

        response.success = True
        response.message = (
            f'Set initial pose from nearest: "{spot.name}" '
            f'({spot.x:.2f}, {spot.y:.2f})')
        self.get_logger().info(response.message)
        return response

    def get_error_callback(self, request, response):
        spot_id = request.spot_id
        if spot_id < 0 or spot_id >= len(self.spots):
            response.success = False
            response.message = f'Invalid spot_id {spot_id}'
            return response

        spot = self.spots[spot_id]
        response.success = True
        response.error_x = self.current_odom_x - spot.x
        response.error_y = self.current_odom_y - spot.y
        response.error_theta = spot.angle_diff_to(self.current_odom_yaw)
        response.distance = spot.distance_to(self.current_odom_x, self.current_odom_y)
        response.message = (
            f'Error to "{spot.name}": dist={response.distance:.2f}m, '
            f'angle={math.degrees(response.error_theta):.1f}deg')
        return response

    def list_spots_callback(self, request, response):
        map_name = request.map_name if request.map_name else self.map_name
        spots = self.storage.load_spots(map_name)

        response.spot_names = [s.name for s in spots]
        response.spot_ids = list(range(len(spots)))
        response.x_positions = [s.x for s in spots]
        response.y_positions = [s.y for s in spots]
        response.thetas = [s.yaw for s in spots]
        return response

    def feedback_loop(self):
        if not self.spots:
            return

        nearest_id = self._find_nearest_spot()
        self.nearest_id = nearest_id

        if nearest_id is not None:
            spot = self.spots[nearest_id]
            dist = spot.distance_to(self.current_odom_x, self.current_odom_y)
            angle = math.degrees(spot.angle_diff_to(self.current_odom_yaw))

            msg = String()
            msg.data = (
                f'Nearest: [{nearest_id}] "{spot.name}" | '
                f'dist: {dist:.2f}m | angle: {angle:.1f}deg')
            self.feedback_pub.publish(msg)

    def publish_markers(self):
        if self.spots:
            self.marker_pub.publish_all_spots(
                self.spots,
                nearest_id=self.nearest_id if self.auto_highlight else None,
                selected_id=self.selected_id,
                robot_x=self.current_odom_x,
                robot_y=self.current_odom_y,
            )

    def _publish_initial_pose(self, spot: ParkingSpot):
        covariance = [0.0] * 36
        covariance[0] = self.cov_x
        covariance[7] = self.cov_y
        covariance[35] = self.cov_yaw

        msg = spot.to_initial_pose_msg(
            stamp=self.get_clock().now().to_msg(),
            covariance=covariance,
        )
        self.initial_pose_pub.publish(msg)
        self.get_logger().info(
            f'Published /initialpose: ({spot.x:.2f}, {spot.y:.2f}, '
            f'yaw={math.degrees(spot.yaw):.1f}deg)')

    def _find_nearest_spot(self):
        if not self.spots:
            return None

        min_dist = float('inf')
        nearest_id = 0
        for i, spot in enumerate(self.spots):
            dist = spot.distance_to(self.current_odom_x, self.current_odom_y)
            if dist < min_dist:
                min_dist = dist
                nearest_id = i
        return nearest_id


def main(args=None):
    rclpy.init(args=args)
    node = ParkingSpotLoaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
