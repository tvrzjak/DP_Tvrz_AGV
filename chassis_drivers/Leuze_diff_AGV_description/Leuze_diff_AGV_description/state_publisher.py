from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion, Twist
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
from std_msgs.msg import String, Float32MultiArray
from nav_msgs.msg import Odometry
import json

class StatePublisher(Node):

    def __init__(self):
        # rclpy.init()
        super().__init__('state_publisher')

        # Opravit na config potom !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        self.r = 0.085  # Poloměr kola v metrech
        self.L = 0.32 # Rozvor kol v metrech 

        qos_profile = QoSProfile(depth=10)
        self.wheel_vel_publisher = self.create_publisher(Float32MultiArray, 'wheel_vel', qos_profile)
        self.odom_pub = self.create_publisher(Odometry, 'odom', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        self.wheel_sub = self.create_subscription(Float32MultiArray, 'wheel_vel_fb', self.wheels_info, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.apply_velocity, 10)

        self.right_wheel_pos = 0.0
        self.left_wheel_pos = 0.0

        self.x = 0
        self.y = 0
        self.theta = 0
        self.act_time = self.get_clock().now().nanoseconds
        self.prev_time = self.get_clock().now().nanoseconds
        self.dt = 0

        degree = pi / 180.0
        loop_rate = self.create_rate(30)


    def wheels_info(self, msg):
        data = msg.data

        now = self.get_clock().now()
        joint_states = JointState()
        joint_states.header.stamp = now.to_msg()
        joint_states.header.frame_id = "odom"
        joint_states.name = ["left_wheel_joint", "right_wheel_joint"]
        joint_states.position = [data[0]*self.dt, data[1]*self.dt]
        joint_states.velocity = [data[0], data[1]]
        joint_states.effort = [100000.0, 100000.0]
        self.joint_pub.publish(joint_states)

        v, omega = self.forward_kinematics(data[0], data[1])
        # v, omega = self.forward_kinematics(-1, -1)
        self.act_time = self.get_clock().now().nanoseconds
        self.dt = (self.act_time - self.prev_time)*1e-9
        self.prev_time = self.act_time
        self.theta += omega * self.dt
        self.x += v * cos(self.theta) * self.dt
        self.y += v * sin(self.theta) * self.dt
        
        self.get_logger().info(f'v: {v}, omega: {omega}')

        # self.get_logger().info(f'joint states velocity {joint_states.velocity}')

        odom_trans = TransformStamped()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'base_footprint'
        odom_trans.header.stamp = now.to_msg()
        odom_trans.transform.rotation = euler_to_quaternion(0, 0, self.theta)
        odom_trans.transform.translation.x = self.x
        odom_trans.transform.translation.y = self.y
        odom_trans.transform.translation.z = 0.1
        # odom_trans.transform.rotation = \
        #     euler_to_quaternion(0, 0, omega + pi/2) # roll,pitch,yaw
        self.broadcaster.sendTransform(odom_trans)

        # odom_trans.header.frame_id = 'base_link'
        # odom_trans.child_frame_id = 'lidar_link'
        # odom_trans.transform.rotation = euler_to_quaternion(0, 0, -2*self.theta)
        # odom_trans.transform.translation.x = 0.0
        # odom_trans.transform.translation.y = 0.0
        # odom_trans.transform.translation.z = 0.0
        # self.broadcaster.senself.dtransform(odom_trans)

        odom = Odometry()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.header.stamp = now.to_msg()
        odom.pose.pose.orientation = euler_to_quaternion(0, 0, self.theta)
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.1
        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = omega
        self.odom_pub.publish(odom)



    def apply_velocity(self, msg:Twist):
        v = msg.linear.x
        omega = msg.angular.z
        self.get_logger().info(f'v_pozad: {v}')
        omega_left, omega_right = self.inverse_kinematics(v, omega)
        send_msg = Float32MultiArray()
        send_msg.data = [omega_left, omega_right]
        self.wheel_vel_publisher.publish(send_msg)

    def forward_kinematics(self, omega_left, omega_right):
        v = (self.r / 2) * (omega_right + omega_left)
        omega = (self.r / self.L) * (omega_right - omega_left)
        if abs(omega)<0.04:
            omega=0.0
        return v, omega

    def inverse_kinematics(self, v, omega):
        omega_right = (2 * v + omega * self.L) / (2 * self.r)
        omega_left = (2 * v - omega * self.L) / (2 * self.r)
        return omega_left, omega_right

def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
    rclpy.init()
    node = StatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()