from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion, Twist
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import json

class StatePublisher(Node):

    def __init__(self):
        # rclpy.init()
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))


        self.currentVector_subscriber = self.create_subscription(Twist, 'VectorCurrent', self.odometry_publisher, 10)
        self.currentWheels_subscriber = self.create_subscription(Float32MultiArray, 'WheelsCurrent', self.joint_state_publisher, 10)
        self.setpointVector_publisher = self.create_publisher(Twist, 'VectorSetpoint', qos_profile)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)


        self.x = 0
        self.y = 0
        self.theta = 0
        self.act_time = self.get_clock().now().nanoseconds
        self.prev_time = self.get_clock().now().nanoseconds
        self.dt = 0





    def joint_state_publisher(self, msg:Float32MultiArray):

        encoders = msg.data

        now = self.get_clock().now()
        joint_states = JointState()
        joint_states.header.stamp = now.to_msg()
        joint_states.header.frame_id = "odom"
        joint_states.name = ["left_front_wheel_joint", "right_front_wheel_joint", "left_rear_wheel_joint", "right_rear_wheel_joint"]
        joint_states.position = [encoders[0]*self.dt, encoders[1]*self.dt, encoders[2]*self.dt, encoders[3]*self.dt]
        joint_states.velocity = [encoders[0],encoders[1],encoders[2],encoders[3]]
        joint_states.effort = [100000.0, 100000.0]
        self.joint_pub.publish(joint_states)



    def odometry_publisher(self, msg:Twist):

        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z

        now = self.get_clock().now()
        self.act_time = self.get_clock().now().nanoseconds
        self.dt = (self.act_time - self.prev_time)*1e-9
        self.prev_time = self.act_time
        self.theta += omega * self.dt
        self.x += (vx * cos(self.theta) - vy * sin(self.theta)) * self.dt
        self.y += (vx * sin(self.theta) + vy * cos(self.theta)) * self.dt

        # self.get_logger().info(f'joint states velocity {joint_states.velocity}')

        odom_trans = TransformStamped()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'base_footprint'
        odom_trans.header.stamp = now.to_msg()
        odom_trans.transform.rotation = euler_to_quaternion(0, 0, self.theta)
        odom_trans.transform.translation.x = self.x
        odom_trans.transform.translation.y = self.y
        odom_trans.transform.translation.z = 0.142
        # odom_trans.transform.rotation = \
        #     euler_to_quaternion(0, 0, omega + pi/2) # roll,pitch,yaw
        self.broadcaster.sendTransform(odom_trans)

        # odom_trans.header.frame_id = 'base_footprint'
        # odom_trans.child_frame_id = 'lidar_link'
        # odom_trans.transform.rotation = euler_to_quaternion(0, 0, -2*self.theta)
        # odom_trans.transform.translation.x = 0.0
        # odom_trans.transform.translation.y = 0.0
        # odom_trans.transform.translation.z = 0.0
        # self.broadcaster.sendTransform(odom_trans)

        odom = Odometry()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.header.stamp = now.to_msg()
        odom.pose.pose.orientation = euler_to_quaternion(0, 0, self.theta)
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.142
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = omega
        self.odom_pub.publish(odom)



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
