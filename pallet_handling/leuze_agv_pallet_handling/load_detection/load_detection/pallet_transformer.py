import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PolygonStamped
from tf2_ros import TransformListener, Buffer, TransformException, TransformBroadcaster
from tf2_geometry_msgs import do_transform_point, do_transform_polygon_stamped
from visualization_msgs.msg import Marker, MarkerArray


class PalletTransformer(Node):

    def __init__(self):
        """
        Class which transform pallet rectangle and pallet center point from lidar fram to the fixed map frame
            subscribe:  pallet rectangle in lidar frame (PolygonStamped)
                        pallet center in lidar frame (PointStamped)
            publish:    pallet rectangle in map frame (PolygonStamped)
                        pallet center in map frame (PointStamped)
            timer:      with 0.05s period publish last computed pallet data
        """
        
        super().__init__('transformer_node')
        self.point: PointStamped = None
        self.pallet: PolygonStamped = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)

        # Subscribers and publishers + timer
        self.timer = self.create_timer(0.05, self.lidar_data_callback)
        self.pallet_center_subscriber = self.create_subscription(
            PointStamped,
            'detected_center',
            self.last_data,
            10
        )
        self.pallet_subscriber = self.create_subscription(
            PolygonStamped,
            'pallet_help',
            self.last_data_polygon,
            10
        )
        self.pallet_center_publisher = self.create_publisher(PointStamped, 'pallet_center', 10)
        self.pallet_publisher = self.create_publisher(PolygonStamped, 'pallet', 10)
        

    def last_data(self, msg: PointStamped):
        """
        Transform subscribed data of pallet center to the map frame and save it
            msg = point of the center
        """
        try:
            # Get the transform from lidar_link to map
            transform = self.buffer.lookup_transform(
                'map', 'lidar_link', rclpy.time.Time()
            )
            # Transform the lidar data from lidar_link to map
            self.point = do_transform_point(msg, transform)
        except:
            pass

    def last_data_polygon(self, msg: PolygonStamped):
        """
        Transform subscribed data of pallet rectangle to the map frame and save it
            msg = pallet rectangle
        """
        try:
            # Get the transform from lidar_link to map
            transform = self.buffer.lookup_transform(
                'map', 'lidar_link', rclpy.time.Time()
            )
            # Transform the lidar data from lidar_link to map
            self.pallet = do_transform_polygon_stamped(msg, transform)
        except:
            pass




    def lidar_data_callback(self):
        """
        Callback function called by timer
        Publish periodically last transformed data about pallet
        """

        if self.point:
            self.pallet_center_publisher.publish(self.point)
        if self.pallet:
            self.pallet_publisher.publish(self.pallet)


def main(args=None):
    """
    Main starting method
    """
    
    rclpy.init(args=args)
    transformer_node = PalletTransformer()
    rclpy.spin(transformer_node)
    transformer_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
