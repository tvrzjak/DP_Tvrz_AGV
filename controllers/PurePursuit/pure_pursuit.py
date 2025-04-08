
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, PointStamped
from std_msgs.msg import String, Bool
from nav_msgs.msg import Path
from nav_msgs.srv import GetMap
from svg.path import parse_path
import tf2_ros
import numpy as np
import math

def sgn(val):
    return -1 if val < 0 else 1

def pt_to_pt_distance(pt1, pt2):
    return math.sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2)

def convert_to_array(path, offset, resolution):
        """Converts the path (list of lists) to a NumPy array and multiplies each point by 0.05.
        """
        # Convert the path to a NumPy array
        # TODO maybe set resolution from yaml file - better
        path_array = np.array(path) * resolution
        path_array *= np.array([1, -1])
        path_array += np.array(offset)
    
        # Remove duplicate points
        unique_path = []
        for point in path_array:
            if not any(np.array_equal(point, existing_point) for existing_point in unique_path):
                unique_path.append(point)
        
        return np.array(unique_path)

def find_stop_index(trajectory, distance):
    for i in range(1, len(trajectory)):
        pt_to_start_distance = pt_to_pt_distance(trajectory[-i], trajectory[0])
        if pt_to_start_distance >= distance:
            print(i)
            return i

def path_to_points(d):
    # Parse the path
    path = parse_path(d)

    all_points = []
    num_points_per_segment = 50

    for segment in path:
        ts = np.linspace(0, 1, num_points_per_segment)
        for t in ts:
            point = segment.point(t)
            all_points.append((float(point.real), float(point.imag)))
            
    return all_points

class TrajectoryFollower(Node):
    def __init__(self):
        super().__init__('trajectory_follower')
        
        self.trajectory = None
        self.original_trajectory_length = 0
        self.stop_index = 0
        self.current_index = 0
        self.driving = False
        
        self.path_sub = self.create_subscription(String, '/path', self.load_path, 10)
        self.path_sub  # prevent unused variable warning
        self.driving_sub = self.create_subscription(Bool, '/driving', self.driving_toggle, 10)
        self.driving_sub = self.create_subscription(Bool, '/fake_path', self.load_fake_path, 10)
        self.driving_sub  # prevent unused variable warning
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.plan_pub = self.create_publisher(Path, '/plan', 10)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.map_client = self.create_client(GetMap, '/map_server/map')
        
        self.timer = self.create_timer(0.05, self.control_loop)
        self.first = False

        self.goal_point_pub = self.create_publisher(PointStamped, '/goal_point', 10)  # Nový publisher pro goalPt
        
        

    def load_path(self, msg):
        all_points = path_to_points(msg.data)
        self.get_logger().warn('nacitam')
        
        if not self.map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for /map_server/map service...')
            return  # Pokud služba není dostupná, neblokujeme
            
        self.get_logger().warn('mam clienta')
        request = GetMap.Request()
        future = self.map_client.call_async(request)
        future.add_done_callback(lambda future: self.process_map_response(future, all_points))

    def process_map_response(self, future, all_points):
        """Callback pro zpracování odpovědi od služby GetMap."""
        self.get_logger().warn('spinuju')
        try:
            result = future.result()
            if result is not None:
                map_msg = result.map
                map_height = map_msg.info.height
                map_originX = map_msg.info.origin.position.x
                map_originY = map_msg.info.origin.position.y
                map_resolution = map_msg.info.resolution
                self.get_logger().warn('Mam vsechny parametry')
                
                self.trajectory = convert_to_array(all_points, [map_originX, map_originY + (map_height * map_resolution)], map_resolution)
                self.get_logger().warn('spocital jsem to')
                
                self.original_trajectory_length = len(self.trajectory)
                self.stop_index = find_stop_index(self.trajectory, 1.0)
                self.publish_plan()
            else:
                self.get_logger().error('Failed to obtain map height, using default value.')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')

    def load_fake_path(self, msg):

        all_points = [(38.544602, 34.45137), (38.544602, 34.45137), (38.544602, 34.45137), (38.544602, 34.45137), (38.544602, 34.45137), (38.544602, 34.45137), (38.544602, 34.45137), (38.544602, 34.45137), (38.544602, 34.45137), (38.544602, 34.45137), (38.544602, 34.45137), (38.544602, 34.45137), (38.544602, 34.45137), (38.544602, 34.45137), (38.544602, 34.45137), (38.544602, 34.45137), (38.544602, 34.45137), (38.544602, 34.45137), (38.544602, 34.45137), (38.544602, 34.45137), (38.544602, 34.45137), (38.544602, 34.45137)]

        self.trajectory = convert_to_array(all_points, [-2.54, -2.35])

        self.original_trajectory_length = len(self.trajectory)

        self.stop_index = find_stop_index(self.trajectory, 1.0)

        self.publish_plan()

    def driving_toggle(self, msg):
        self.driving = msg.data

    
    def publish_plan(self):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        for point in self.trajectory:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.plan_pub.publish(path_msg)
    
    def get_robot_position(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            return np.array([transform.transform.translation.x, transform.transform.translation.y]), math.degrees(2 * math.atan2(transform.transform.rotation.z, transform.transform.rotation.w))
        except tf2_ros.LookupException:
            self.get_logger().warning('Could not get transform from map to base_link')
            return None, None
    
    def pure_pursuit_step(self, path, currentPos, currentHeading, lookAheadDis, LFindex):

        currentX = currentPos[0]
        currentY = currentPos[1]
        lastFoundIndex = LFindex
        
        for i in range(lastFoundIndex, len(path) - 1):
            x1, y1 = path[i][0] - currentX, path[i][1] - currentY
            x2, y2 = path[i+1][0] - currentX, path[i+1][1] - currentY
            dx, dy = x2 - x1, y2 - y1
            dr = math.sqrt(dx**2 + dy**2)
            D = x1 * y2 - x2 * y1
            discriminant = (lookAheadDis**2) * (dr**2) - D**2
            
            if discriminant >= 0:
                sol_x1 = (D * dy + sgn(dy) * dx * math.sqrt(discriminant)) / dr**2 + currentX
                sol_y1 = (-D * dx + abs(dy) * math.sqrt(discriminant)) / dr**2 + currentY
                sol_x2 = (D * dy - sgn(dy) * dx * math.sqrt(discriminant)) / dr**2 + currentX
                sol_y2 = (-D * dx - abs(dy) * math.sqrt(discriminant)) / dr**2 + currentY
                
                minX, maxX = min(path[i][0], path[i+1][0]), max(path[i][0], path[i+1][0])
                minY, maxY = min(path[i][1], path[i+1][1]), max(path[i][1], path[i+1][1])
                
                if (minX <= sol_x1 <= maxX and minY <= sol_y1 <= maxY) or (minX <= sol_x2 <= maxX and minY <= sol_y2 <= maxY):
                    if (minX <= sol_x1 <= maxX and minY <= sol_y1 <= maxY) and (minX <= sol_x2 <= maxX and minY <= sol_y2 <= maxY):
                        goalPt = [sol_x1, sol_y1] if pt_to_pt_distance([sol_x1, sol_y1], path[i+1]) < pt_to_pt_distance([sol_x2, sol_y2], path[i+1]) else [sol_x2, sol_y2]
                    else:
                        goalPt = [sol_x1, sol_y1] if (minX <= sol_x1 <= maxX and minY <= sol_y1 <= maxY) else [sol_x2, sol_y2]
                    
                    if pt_to_pt_distance(goalPt, path[i+1]) < pt_to_pt_distance([currentX, currentY], path[i+1]):
                        lastFoundIndex = i
                        break
                    else:
                        lastFoundIndex = i+1
                else:
                    goalPt = [path[lastFoundIndex][0], path[lastFoundIndex][1]]
        
        Kp = 0.55
        absTargetAngle = math.degrees(math.atan2(goalPt[1] - currentY, goalPt[0] - currentX))
        absTargetAngle = absTargetAngle if absTargetAngle >= 0 else absTargetAngle + 360
        turnError = absTargetAngle - currentHeading
        turnError = turnError - 360 if turnError > 180 else (turnError + 360 if turnError < -180 else turnError)
        turnVel = Kp * turnError
        
        return goalPt, lastFoundIndex, turnVel, turnError
    
    def control_loop(self):
        if self.driving is False:
            if self.first is False:
                cmd_vel = Twist()
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd_vel)
                self.first = True
            return
        else:
            self.first = False

        position, heading = self.get_robot_position()
        print(position)
        if position is None or self.trajectory is None:
            print("None")
            return
        
        if self.current_index >= len(self.trajectory)-self.stop_index:
            self.trajectory = np.vstack((self.trajectory, self.trajectory[:self.original_trajectory_length]))  # Přidání další kopie trajektorie
            
            # Pokud je trajektorie více než dvojnásobná, odstraníme první kopii
            if len(self.trajectory) > 2 * self.original_trajectory_length:
                self.trajectory = self.trajectory[self.original_trajectory_length:]
                self.current_index -= self.original_trajectory_length

        goalPt, self.current_index, turnVel, turn_angle = self.pure_pursuit_step(self.trajectory, position, heading, 0.4, self.current_index)

        goal_point_msg = PointStamped()
        goal_point_msg.header.stamp = self.get_clock().now().to_msg()
        goal_point_msg.header.frame_id = 'map'  # Stejný rámec jako poloha robota
        goal_point_msg.point.x = float(goalPt[0])
        goal_point_msg.point.y = float(goalPt[1])
        goal_point_msg.point.z = 0.0
        self.goal_point_pub.publish(goal_point_msg)
        
        max_speed = 0.3
        min_speed = 0.01
        max_turn_vel = 0.5
        angle_limit = 60
        if abs(turn_angle) >= angle_limit:
            linear_speed = min_speed
        else:
            linear_speed = max_speed * (1 - (abs(turn_angle) / angle_limit))

        if abs(turnVel*math.pi/180) > max_turn_vel:
            turn_vel = max_turn_vel * (turnVel/abs(turnVel))
        else:
            turn_vel = turnVel*math.pi/180

        cmd_vel = Twist()
        cmd_vel.linear.x = linear_speed
        cmd_vel.angular.z = turn_vel
        
        self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()