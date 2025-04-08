import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PolygonStamped, Point, Point32, PoseStamped, Quaternion, PointStamped, Twist
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose 
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Bool
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
import numpy as np
import cv2
import math
from itertools import combinations
import subprocess
import time
import os
import yaml
import json
from rcl_interfaces.msg import ParameterValue, ParameterType, Parameter
from rcl_interfaces.srv import SetParameters, GetParameters, ListParameters
from rcl_interfaces.msg import ParameterDescriptor, ParameterValue


class DetectAndDock(Node):
    
    def __init__(self):
        """
        Class which solve detecting of the pallet and after that docking under it
            subscribe:  detected cluster (MarkerArray)
                        rectangle representing pallet in map frame (PolygonStamped)
                        global costmap data (OccupancyGrid)
                        user decision if dock or not under the detected pallet (Bool)
            publish:    detected pallet in lidar frame (PolygonStamped)
                        detected center of the pallet in lidar frame (PointStamped)
                        potential goals to come near to the pallet (MarkerArray)
                        goal pose which starts path planning and following after publish (PoseStamped)                    
        """
        
        super().__init__('rectangle_detector')
        self.node = rclpy.create_node('detect_and_dock')

        # Load parameters from config file
        config_file = os.path.join(get_package_share_directory('load_detection'), 'config', 'config.yaml')

        if config_file:
            with open(config_file, 'r') as f:
                config_dict = yaml.load(f, Loader=yaml.SafeLoader)
        else:
            config_dict = []

        configuration = config_dict['load_detection']
        self.config = configuration['ros_parameters']

        # Subscription and publishers
        self.costmap_subscriber = self.create_subscription(
            OccupancyGrid,
            'global_costmap/costmap',
            self.costmap_sub,
            10)
        self.dbscan = self.create_subscription(
            MarkerArray,
            'dbscan_groups',
            self.detection,
            10)
        self.user = self.create_subscription(
            Bool,
            'user_response',
            self.docking,
            10)
        self.pallet_subscriber = self.create_subscription(
            PolygonStamped,
            'pallet',
            self.pallet_sub,
            10)
        self.footprint_subscriber = self.create_subscription(
            PolygonStamped,
            'global_costmap/published_footprint',
            self.footprint_sub,
            10)

        self.detect: Bool = False
        self.pallet: PolygonStamped = None
        self.detected_load_rect: tuple = None
        self.global_costmap: OccupancyGrid = None
        self.counter = 0
        self.last_goal = None
        self.robot_pose = None
        self.pallet_centers_list_x = []
        self.pallet_centers_list_y = []

        self.potential_goals = self.create_publisher(MarkerArray, 'potential_goals', 10)
        self.center_publisher = self.create_publisher(PointStamped, 'detected_center', 10)
        self.pallet_publisher = self.create_publisher(PolygonStamped, 'pallet_help', 10)
        self.goal_pose_publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)    
        self.spin_publisher = self.create_publisher(Twist, 'cmd_vel', 10)


    def costmap_sub(self, msg: OccupancyGrid):
        """
        Subscribe global costmap message and save it
            msg = costmap topic
        """
        self.global_costmap = msg


    def pallet_sub(self, msg: PolygonStamped):
        """
        Subscribe pallet rectangle message and save it
            msg = PolzgonStamped pallet topic
        """
        self.pallet = msg


    def footprint_sub(self, msg: PolygonStamped):
        """
        Subscribe pallet rectangle message and save it
            msg = PolzgonStamped pallet topic
        """
        self.robot_pose = self.polygon_stamped_center(msg)


    
    def detection(self, msg: MarkerArray):
        """
        Schedule controll method for detection pallet and its specifications (points needed for navigation)
            msg = MarkerArray topic with clusters detected by dbscan
        """
        
        # Centers of dbscan clusters
        centers: list = self.clusters_to_centers(msg)

        # Detect if there are four centers forming a rectangle
        if len(centers) >= 4:
            detected_load: list = self.detect_load(centers, self.config['shorter_side'], self.config['longer_side'], self.config['tolerance'])

            if detected_load:

                # Convert the points to a numpy array of shape (4, 1, 2) as expected by cv2.minAreaRect()
                detected_load_array: np.array = np.array(detected_load[0:4], dtype=np.float32).reshape((-1, 1, 2))
                
                # Fit a rotated rectangle to the points
                self.detected_load_rect = cv2.minAreaRect(detected_load_array)
                
                if self.detected_load_rect:
                    self.publish_pallet(self.detected_load_rect)
                    self.publish_center(self.detected_load_rect[0])
                    
                    if self.detect:
                        self.counter += 1
                        print(self.counter)
                        
                        # Calculate the center of the polygon
                        pallet_center: list = self.polygon_stamped_center(self.pallet)
                        self.pallet_centers_list_x.append(pallet_center[0])
                        self.pallet_centers_list_y.append(pallet_center[1])

                        # Average center coordinates
                        if self.counter == 10:
                            pallet_center[0] = np.mean(self.pallet_centers_list_x)
                            pallet_center[1] = np.mean(self.pallet_centers_list_y)
                            print(pallet_center)

                            self.counter = 0
                            self.pallet_centers_list_x = []
                            self.pallet_centers_list_y = []

                            # Find potential points for comming near to the pallet
                            potential_goals: list = self.find_longer_side_orthogonal_points(self.pallet, self.config['distance_from_pallet'])

                            # try if the points are on free space
                            goal: list = self.find_free_points(potential_goals, self.global_costmap)
                            self.publish_centers(goal)

                            if goal:
                                # go near to the pallet
                                self.detect = False
                                nearest_point = self.find_nearest_point_index(self.robot_pose, goal)
                                if self.publish_goal_pose(goal[nearest_point], pallet_center):
                                    print("Goal succeeded")

                                    # Stright line trajectory to the center of the pallet
                                    self.stright_line_trajectory(goal[nearest_point], pallet_center)
                                    time.sleep(10)
                                    self.set_nav2_params(self.config['robot_radius2'], self.config['global_inflation_radius2'], self.config['local_inflation_radius2'], self.config['footprint2'], self.config['collision_monitor2'])

                        

    def docking(self, msg: Bool):
        """
        Schedule control method for counting possible places to come near to the palllet
            msg = bool topic wit user command - if user want to dock under the actually detected pallet
        """

        user_input: Bool = msg.data
        if self.pallet and user_input:
            # Calculate the center of the polygon
            pallet_center: list = self.polygon_stamped_center(self.pallet)

            # Find potential points for comming near to the pallet
            potential_goals: list = self.find_longer_side_orthogonal_points(self.pallet, self.config['distance_from_pallet2'])

            # try if the points are on free space
            goal: list = self.find_free_points(potential_goals, self.global_costmap)
            self.publish_centers(goal)

            if goal:
                # go near to the pallet by standard planner
                nearest_point = self.find_nearest_point_index(self.robot_pose, goal)
                if self.publish_goal_pose(goal[nearest_point], pallet_center):
                    self.last_goal = goal[0]
                    print("Goal succeeded")
                    self.set_nav2_params(self.config['robot_radius'], self.config['global_inflation_radius'], self.config['local_inflation_radius'], self.config['footprint'], self.config['collision_monitor'])
                    self.detect = True

                    # print("zacinam")
                    # spin_msg = Twist()
                    # spin_msg.angular.z = -0.3
                    # self.spin_publisher.publish(spin_msg)
                    # time.sleep(4)
                    # spin_msg.angular.z = 0.0
                    # self.spin_publisher.publish(spin_msg)
                    # print("hotovo")
                    self.stright_line_trajectory(goal[nearest_point], pallet_center)
                    time.sleep(10)
                    self.set_nav2_params(self.config['robot_radius2'], self.config['global_inflation_radius2'], self.config['local_inflation_radius2'], self.config['footprint2'], self.config['collision_monitor2'])


    def polygon_stamped_center(self, polygon: PolygonStamped) -> list:
        """
        Compute center of rectangle with type PolygonStamped
            polygon = PolygonStamped rectangle
        """

        center: list = [0.0, 0.0, 0.0]
        num_points: int = len(polygon.polygon.points)
        for point in polygon.polygon.points:
            center[0] += point.x
            center[1] += point.y
            center[2] += point.z
        center[0] /= num_points
        center[1] /= num_points
        center[2] /= num_points
        return center



    def clusters_to_centers(self, msg: MarkerArray) -> list:
        """
        Compute centers of detected clusters
            msg = MarkerArray of clusters found by dbscan
        """

        centers: list = []
        marker: Marker
        for marker in msg.markers:
            # Assuming markers are of type SPHERE, use marker.pose.position as center
            center = (marker.pose.position.x, marker.pose.position.y)
            centers.append(center)

        return centers



    
    def publish_centers(self, centers: list):
        """
        Publish list of points into MarkerArray
            centers = list of cv2 Point objects
        """
        
        marker_array: MarkerArray = MarkerArray()
        marker_id: int = 0

        for center in centers:
            # Create marker for each center
            marker: Marker = Marker()
            marker.header.frame_id = "map"
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale.x = 0.4
            marker.scale.y = 0.4
            marker.scale.z = 0.4
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.pose.position.x = float(center[0])
            marker.pose.position.y = float(center[1])
            marker.pose.position.z = 0.0

            marker_array.markers.append(marker)
            marker_id += 1

        self.potential_goals.publish(marker_array)



    def publish_center(self, center):
        """
        Publish one point to rviz - now used for center of the pallet
            center = point with x,y coordinates
        """
        point_msg = PointStamped()
        point_msg.header.frame_id = 'lidar_link'
        point_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        point_msg.point.x = center[0]
        point_msg.point.y = center[1]
        point_msg.point.z = 0.0
        self.center_publisher.publish(point_msg)




    
    def detect_load(self, centers: list, shorter_side: float, longer_side: float, toler: float) -> list:
        """
        Find if in the list of points (centers) exist 4 points which create rectangle in a space with specified size
            centers = list of cv2 Point objects
            shorter_side = shorter size of rectangle (pallet)
            longer_side = longer size of rectangle (pallet)
            toler = tolerance from specified size
        """
        
        candidates: list = []
    
        for combo in combinations(centers, 4):
            # Compute the distances between centers
            dist12: float = self.euclidean_distance(combo[0], combo[1])
            dist13: float = self.euclidean_distance(combo[0], combo[2])
            dist14: float = self.euclidean_distance(combo[0], combo[3])
            dist23: float = self.euclidean_distance(combo[1], combo[2])
            dist24: float = self.euclidean_distance(combo[1], combo[3])
            dist34: float = self.euclidean_distance(combo[2], combo[3])
            
            # Store all distances in a list
            distances: list = [dist12, dist13, dist14, dist23, dist24, dist34]
            distances = sorted(distances)
            
            length1: float = shorter_side
            length2: float = longer_side
            length3: float = math.sqrt(length1*length1 + length2*length2)
            tolerance: float = toler
            
            if abs(distances[0] - distances[1]) <= tolerance and abs(distances[2] - distances[3]) <= tolerance:
                if abs(distances[0] - length1)  <= tolerance and abs(distances[1] - length1)  <= tolerance and abs(distances[2] - length2)  <= tolerance and abs(distances[3] - length2)  <= tolerance:
                    if abs(distances[4] - length3)  <= tolerance and abs(distances[5] - length3)  <= tolerance:
                        candidates.extend(combo)

        return candidates


    
    def euclidean_distance(self, point1: list, point2: list) -> float:
        """
        Compute euclidean distance between 2 points
            point1 = point with x,y coordinates
            point2 = point with x,y coordinates
        """
        
        return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)


    
    def publish_pallet(self, rotated_rect: tuple):
        """
        Publish detected pallet rectangle
            rotated_rect = rectangle
        """
        
        # Get the vertices of the rotated rectangle
        vertices = cv2.boxPoints(rotated_rect)
        
        # Convert vertices to ROS format
        polygon_msg = PolygonStamped()
        polygon_msg.header.frame_id = "lidar_link"

        for vertex in vertices:
            point = Point32()
            point.x = float(vertex[0])
            point.y = float(vertex[1])
            polygon_msg.polygon.points.append(point)
        
        # Publish the polygon message
        self.pallet_publisher.publish(polygon_msg)


    
    def find_longer_side_orthogonal_points(self, polygon: PolygonStamped, distance: float =0.5) -> list:
        """
        Find points, which are 'distance' far from the center of 2 longer sides of the rectangle in ortogonal direction
            rotated_rect = cv2 Rectangle object
            distance = how far should be points from sides of rectangle
        """
        
        # Get the vertices of the rotated rectangle
        vertices = polygon.polygon.points
        
        # Extract coordinates of the vertices
        vertex_coords = [(point.x, point.y) for point in vertices]
        
        # Compute distances between opposite vertices
        distance_01 = np.linalg.norm(np.array((vertex_coords[0][0], vertex_coords[0][1])) - np.array((vertex_coords[1][0], vertex_coords[1][1])))
        distance_12 = np.linalg.norm(np.array((vertex_coords[1][0], vertex_coords[1][1])) - np.array((vertex_coords[2][0], vertex_coords[2][1])))
        
        # Determine the longer sides
        longer_side_indices = [0, 2] if distance_01 > distance_12 else [1, 3]
        
        # Compute center points of longer sides
        center_points: list = []
        for i in longer_side_indices:
            next_index = (i + 1) % 4
            center_point = (
                (vertex_coords[i][0] + vertex_coords[next_index][0]) / 2,
                (vertex_coords[i][1] + vertex_coords[next_index][1]) / 2
            )
            center_points.append(center_point)
        
        # Compute unit vectors perpendicular to longer sides
        unit_vectors: list = []
        for i in longer_side_indices:
            next_index = (i + 1) % 4
            dx = vertex_coords[next_index][0] - vertex_coords[i][0]
            dy = vertex_coords[next_index][1] - vertex_coords[i][1]
            length = np.sqrt(dx ** 2 + dy ** 2)
            unit_vector = (dy / length, -dx / length)  # Perpendicular unit vector
            unit_vectors.append(unit_vector)
        
        # Compute offset vectors
        offset_vectors = [(unit[0] * distance, unit[1] * distance) for unit in unit_vectors]
        
        # Compute orthogonal points
        orthogonal_points: list = []
        for i in range(len(longer_side_indices)):
            orthogonal_point = (
                center_points[i][0] + offset_vectors[i][0],
                center_points[i][1] + offset_vectors[i][1]
            )
            orthogonal_points.append(orthogonal_point)

        return orthogonal_points


    
    def find_free_points(self, orthogonal_points: list, costmap: OccupancyGrid) -> list:
        """
        Decide if given list of points is on the free space in the costmap
            orthogonal_points = list of points
            map_data = unfiltered costmap data
            map_info = msg.info from costmap
        """
            
        feasible_points: list = []
        for point in orthogonal_points:
            if self.is_point_on_free_space(point, costmap):
                feasible_points.append(point)
        return feasible_points


    
    def is_point_on_free_space(self, point: list, costmap: OccupancyGrid) -> Bool:
        """
        Decide if given point is on the free space in the costmap
            point = poind with x,y coordinates
            map_data = unfiltered costmap data
            info = msg.info from costmap
        """
        
        costmap_data = np.array(costmap.data, dtype=np.uint8).reshape((costmap.info.height, costmap.info.width))

        # Convert the point to map coordinates
        map_resolution = costmap.info.resolution
        map_origin_x = costmap.info.origin.position.x
        map_origin_y = costmap.info.origin.position.y
        map_x:int = int((point[0] - map_origin_x) / map_resolution)
        map_y:int = int((point[1] - map_origin_y) / map_resolution)
        
        # Check if the point is within the map boundaries
        if map_x < 0 or map_x >= costmap.info.width or map_y < 0 or map_y >= costmap.info.height:
            return False
        
        # Check if the point is in free space (not occupied by obstacles)
        cell_value = costmap_data[map_y, map_x]
        if cell_value > 0:  # Occupied space (cell value > 0)
            return False
        
        return True


    def find_nearest_point_index(self, target_point: list, points: list) -> int:
        """
        Find the index of the nearest point from a list of points to a given target point.
        """
        min_distance: float = float('inf')
        nearest_point_index: int = None
        
        for i, point in enumerate(points):
            distance: float = math.sqrt((point[0] - target_point[0])**2 + (point[1] - target_point[1])**2)
            if distance < min_distance:
                min_distance = distance
                nearest_point_index = i
        
        return nearest_point_index    

    
    def publish_goal_pose(self, final_point: list, pose: list) -> Bool:
        """
        Publish given point like a goal pose for robot -> robot start navigating to this pose
            final_point = goal pose point
            pose = center of the pallet (it is next point for computing orinetation)
        """

        goal_pose: PoseStamped = PoseStamped()
        goal_pose.header.frame_id = "map"  # Assuming frame_id is 'map'

        goal_pose.pose.position.x = float(final_point[0]) 
        goal_pose.pose.position.y = float(final_point[1])

        # Compute orientation quaternion
        delta_x: float = pose[0] - final_point[0]
        delta_y: float = pose[1] - final_point[1]

        yaw_angle: float = math.atan2(delta_y, delta_x)
        quaternion: Quaternion = self.quaternion_from_yaw(yaw_angle)
        goal_pose.pose.orientation = quaternion

        action_client: ActionClient = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')

        while not action_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().info('Action server not available, waiting...')

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        future = action_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is None:
            self.node.get_logger().warn('Failed to get result')
            return False

        result = future.result()

        while result.status != GoalStatus.STATUS_SUCCEEDED:
            future = action_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self.node, future)
            result = future.result()
            
        return True
    


    def stright_line_trajectory(self, start_point: list, final_point: list):
        """
        Publish start line trajectory and start its folowing for docking under the pallet
            final_point = goal pose point
            pose = center of the pallet (it is next point for computing orinetation)
        """

        # Compute orientation quaternion
        delta_x: float = final_point[0] - start_point[0]
        delta_y: float = final_point[1] - start_point[1]
        yaw_angle: float = math.atan2(delta_y, delta_x)
        quaternion: Quaternion = self.quaternion_from_yaw(yaw_angle)

        # Prepare trajectory
        path_msg = Path()
        path_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        num_points = 60
        for i in range(num_points):
            t = i / (num_points - 1)

            pose: PoseStamped = PoseStamped()
            pose.pose.position.x = start_point[0] + t*delta_x
            pose.pose.position.y = start_point[1] + t*delta_y
            pose.pose.position.z = 0.0
            pose.pose.orientation = quaternion
            pose.header.stamp = path_msg.header.stamp
            pose.header.frame_id = path_msg.header.frame_id
            path_msg.poses.append(pose)    
    
        # FollowPath client
        follow_path_client = ActionClient(rclpy.create_node('follow_path_action_client'), FollowPath, '/follow_path')
        publisher = self.create_publisher(Path, '/plan', 10)
        publisher.publish(path_msg)

        while not follow_path_client.wait_for_server(timeout_sec=1.0):
            print('Waiting for action server...')

        goal_msg = FollowPath.Goal()
        goal_msg.path = path_msg

        future = follow_path_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(follow_path_client._node, future)

        if future.result() is not None:
            print('Path following started successfully!')
        else:
            print('Failed to start path following')


    
    def quaternion_from_yaw(self, yaw: float) -> Quaternion:
        """
        Compute quaternion from yaw
            yaw = yaw angle
        """

        quaternion: Quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = float(math.sin(yaw / 2))
        quaternion.w = float(math.cos(yaw / 2))
        return quaternion

   
    
    def set_nav2_params(self, robot_radius: float, global_inflation_radius, local_inflation_radius, footprint, collision_monitor):
        """
        Change parameters of the navigation stack
        """

        # command = ['ros2', 'param', 'set', '/global_costmap/global_costmap', 'robot_radius', str(robot_radius)]
        # subprocess.run(command)
        # command = ['ros2', 'param', 'set', '/global_costmap/global_costmap', 'inflation_layer.inflation_radius', str(global_inflation_radius)]
        # subprocess.run(command)
        # command = ['ros2', 'param', 'set', '/local_costmap/local_costmap', 'inflation_layer.inflation_radius', str(local_inflation_radius)]
        # subprocess.run(command)
        # command = ['ros2', 'param', 'set', '/local_costmap/local_costmap', 'footprint', footprint]
        # subprocess.run(command)
        
        cli = self.create_client(SetParameters, '/global_costmap/global_costmap/set_parameters')
        while not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        req = SetParameters.Request()

        new_param_value = ParameterValue(double_value=robot_radius, type=ParameterType.PARAMETER_DOUBLE)  # Create ParameterValue with double value
        req.parameters = [Parameter(name='robot_radius', value=new_param_value)]
        self.future = cli.call_async(req)
        new_param_value = ParameterValue(double_value=global_inflation_radius, type=ParameterType.PARAMETER_DOUBLE)  # Create ParameterValue with double value
        req.parameters = [Parameter(name='inflation_layer.inflation_radius', value=new_param_value)]
        self.future = cli.call_async(req)

        cli = self.create_client(SetParameters, '/local_costmap/local_costmap/set_parameters')
        while not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        req = SetParameters.Request()

        new_param_value = ParameterValue(string_value=footprint, type=ParameterType.PARAMETER_STRING)  # Create ParameterValue with double value
        req.parameters = [Parameter(name='footprint', value=new_param_value)]
        self.future = cli.call_async(req)
        new_param_value = ParameterValue(double_value=local_inflation_radius, type=ParameterType.PARAMETER_DOUBLE)  # Create ParameterValue with double value
        req.parameters = [Parameter(name='inflation_layer.inflation_radius', value=new_param_value)]
        self.future = cli.call_async(req)

        cli = self.create_client(SetParameters, '/safety_zones/set_parameters')
        while not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        req = SetParameters.Request()

        new_param_value = ParameterValue(bool_value=collision_monitor, type=ParameterType.PARAMETER_BOOL)  # Create ParameterValue with double value
        req.parameters = [Parameter(name='switch_collision_monitor', value=new_param_value)]
        self.future = cli.call_async(req)

        # par_robot_radius = Parameter("robot_radius", Parameter.Type.DOUBLE, 0.2)
        # par_client = ParameterClient("/global_costmap/global_costmap")
        # result = par_client.set_parameters([par_robot_radius])
        # self._set_parameters([par_robot_radius])




def main(args=None):
    """
    Main starting method
    """

    rclpy.init(args=args)
    node = DetectAndDock()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
