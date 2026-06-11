import time
import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException, ConnectivityException
from std_srvs.srv import Trigger
from go2_interfaces.srv import SaveNamedPose, ListParkingSpots

from ..utils.parking_spot import ParkingSpot
from ..utils.storage import ParkingSpotStorage
from ..utils.marker_publisher import MarkerPublisher


class ParkingSpotSaverNode(Node):

    def __init__(self):
        super().__init__('parking_spot_saver')

        self.declare_parameter('map_name', 'my_map')
        self.declare_parameter('base_directory', '')
        self.declare_parameter('auto_save_distance', 0.0)

        self.map_name = self.get_parameter('map_name').get_parameter_value().string_value
        base_dir = self.get_parameter('base_directory').get_parameter_value().string_value
        self.auto_save_distance = self.get_parameter('auto_save_distance').get_parameter_value().double_value

        self.storage = ParkingSpotStorage(base_directory=base_dir)
        self.spots = self.storage.load_spots(self.map_name)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.marker_pub = MarkerPublisher(self, '/parking_spot_markers')

        self.save_spot_srv = self.create_service(
            Trigger, '/save_parking_spot', self.save_spot_callback)
        self.save_named_srv = self.create_service(
            SaveNamedPose, '/save_named_parking_spot', self.save_named_callback)
        self.list_spots_srv = self.create_service(
            ListParkingSpots, '/list_saved_spots', self.list_spots_callback)

        self.marker_timer = self.create_timer(2.0, self.publish_markers)

        if self.auto_save_distance > 0.0:
            self.auto_save_timer = self.create_timer(1.0, self.auto_save_check)
            self.last_auto_save_x = None
            self.last_auto_save_y = None

        self.get_logger().info(
            f'ParkingSpotSaver started. Map: {self.map_name}, '
            f'Loaded {len(self.spots)} existing spots.')

    def get_current_map_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(), timeout=Duration(seconds=1.0))
            return transform
        except (LookupException, ExtrapolationException, ConnectivityException) as e:
            self.get_logger().warning(f'Cannot get map->base_link transform: {e}')
            return None

    def save_spot_callback(self, request, response):
        transform = self.get_current_map_pose()
        if transform is None:
            response.success = False
            response.message = 'Cannot read map->base_link transform. Is SLAM running?'
            return response

        spot_name = f"spot_{len(self.spots)}"
        spot = self._transform_to_spot(transform, spot_name)
        self.spots.append(spot)

        if self.storage.save_spots(self.map_name, self.spots):
            response.success = True
            response.message = (
                f'Saved "{spot_name}" at ({spot.x:.2f}, {spot.y:.2f}, '
                f'yaw={math.degrees(spot.yaw):.1f}deg)')
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = 'Failed to write parking spots file'
            self.spots.pop()

        return response

    def save_named_callback(self, request, response):
        transform = self.get_current_map_pose()
        if transform is None:
            response.success = False
            response.message = 'Cannot read map->base_link transform. Is SLAM running?'
            return response

        name = request.name if request.name else f"spot_{len(self.spots)}"
        map_name = request.map_name if request.map_name else self.map_name

        spot = self._transform_to_spot(transform, name)
        self.spots.append(spot)

        if self.storage.save_spots(map_name, self.spots):
            response.success = True
            response.message = f'Saved "{name}" at ({spot.x:.2f}, {spot.y:.2f})'
            response.x = spot.x
            response.y = spot.y
            response.theta = spot.yaw
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = 'Failed to write parking spots file'
            self.spots.pop()

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

    def auto_save_check(self):
        if self.auto_save_distance <= 0.0:
            return

        transform = self.get_current_map_pose()
        if transform is None:
            return

        x = transform.transform.translation.x
        y = transform.transform.translation.y

        if self.last_auto_save_x is None:
            self.last_auto_save_x = x
            self.last_auto_save_y = y
            return

        dist = math.sqrt(
            (x - self.last_auto_save_x) ** 2 + (y - self.last_auto_save_y) ** 2)

        if dist >= self.auto_save_distance:
            spot_name = f"auto_{len(self.spots)}"
            spot = self._transform_to_spot(transform, spot_name)
            self.spots.append(spot)
            self.storage.save_spots(self.map_name, self.spots)
            self.last_auto_save_x = x
            self.last_auto_save_y = y
            self.get_logger().info(
                f'Auto-saved "{spot_name}" at ({spot.x:.2f}, {spot.y:.2f})')

    def publish_markers(self):
        if self.spots:
            self.marker_pub.publish_all_spots(self.spots)

    def _transform_to_spot(self, transform, name: str) -> ParkingSpot:
        t = transform.transform
        return ParkingSpot(
            name=name,
            x=t.translation.x,
            y=t.translation.y,
            z=t.translation.z,
            qx=t.rotation.x,
            qy=t.rotation.y,
            qz=t.rotation.z,
            qw=t.rotation.w,
            timestamp=time.time(),
            frame_id='map',
        )


def main(args=None):
    rclpy.init(args=args)
    node = ParkingSpotSaverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
