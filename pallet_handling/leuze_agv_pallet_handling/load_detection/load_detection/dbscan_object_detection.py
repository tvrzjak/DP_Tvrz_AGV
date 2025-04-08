import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from sklearn.cluster import DBSCAN
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSLivelinessPolicy

class ScanProcessor(Node):
    def __init__(self):

        """
        Class which make clustering on the laser scan data
            subscribe:  scan data - a lot of points (LaserScan)
            publish:    array of clusters which are smaller than setted (MarkerArray)
        """

        super().__init__('scan_processor')
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            liveliness=QoSLivelinessPolicy.SYSTEM_DEFAULT)        
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile)
        self.publisher = self.create_publisher(MarkerArray, 'dbscan_groups', 10)
        self.eps = 0.15 # Specify the epsylon radius for grouping points


    def scan_callback(self, msg):
        """
        Scan callback method - for every scan is made clustering and clusters are published like MarkerArray
            msg = LaserScan message
        """

        # Process the LaserScan data and identify clusters
        clusters: list = self.detect_clusters(msg)

        # Publish the detected groups
        marker_array_msg = MarkerArray()
        marker_id = 0
        for cluster in clusters:
            center_point, scale = self.calculate_center_and_scale(cluster)
            if abs(scale/2)<0.15:
                marker = Marker()
                marker.header.frame_id = msg.header.frame_id
                marker.header.stamp = msg.header.stamp
                marker.id = marker_id
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = center_point[0]
                marker.pose.position.y = center_point[1]
                # marker.pose.position.z = center_point[2]
                marker.scale.x = scale
                marker.scale.y = scale
                marker.scale.z = scale
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                marker_array_msg.markers.append(marker)
                marker_id += 1
        self.publisher.publish(marker_array_msg)        


    def detect_clusters(self, scan_msg) -> list:
        """
        Clustering via DBSCAN algorithm
            msg = LaserScan message
        """

        # Convert LaserScan data to numpy array for easier manipulation
        ranges = np.array(scan_msg.ranges)

        # Remove invalid values (NaNs, infs) from the ranges
        valid_ranges = ranges[~np.isnan(ranges) & ~np.isinf(ranges)]

        # Convert valid_ranges to cartesian coordinates
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(valid_ranges))
        x = valid_ranges * np.cos(angles)
        y = valid_ranges * np.sin(angles)

        # Stack x, y coordinates to form (N, 2) array
        points = np.column_stack((x, y))

        # Perform DBSCAN clustering
        dbscan = DBSCAN(eps=self.eps, min_samples=3)
        dbscan.fit(points)

        # Retrieve unique cluster labels (excluding noise points)
        unique_labels = np.unique(dbscan.labels_)[1:]

        # Extract points for each cluster
        clusters: list = []
        for label in unique_labels:
            cluster_points = points[dbscan.labels_ == label]
            clusters.append(cluster_points)
        return clusters


    def calculate_center_and_scale(self, cluster):
        """
        Calculating of center and radius of the detected cluster
            cluster - points which are included in this cluster
        """

        # Calculate the centroid as the mean of all points in the cluster
        centroid = np.mean(cluster, axis=0)
    
        # Calculate pairwise distances between points
        pairwise_distances = np.linalg.norm(cluster[:, None] - cluster, axis=2)
        # Calculate the scale as the maximum distance between any two points (diameter)
        scale = np.max(pairwise_distances)
    
        return centroid, scale
    

def main(args=None):
    """
    Main starting method
    """

    rclpy.init(args=args)
    scan_processor = ScanProcessor()
    rclpy.spin(scan_processor)
    scan_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
