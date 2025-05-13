from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion, Twist
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
from std_msgs.msg import Float32MultiArray

class DiffKinematics(Node):

    def __init__(self):
        # rclpy.init()
        super().__init__('state_publisher')
        # Opravit na config potom !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        self.r = 0.0975  # Poloměr kola v metrech
        self.L = 0.5953 # Rozvor kol v metrech 

        qos_profile = QoSProfile(depth=10)
        self.wheel_vel_publisher = self.create_publisher(Float32MultiArray, 'wheel_vel', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.line_center = self.create_subscription(Twist, 'cmd_vel', self.apply_velocity, 10)

    def apply_velocity(self, msg:Twist):
        v = msg.linear.x
        omega = msg.angular.z
        omega_left, omega_right = self.inverse_kinematics(v, omega)
        send_msg = Float32MultiArray()
        send_msg.data = [omega_left, omega_right]
        self.wheel_vel_publisher.publish(send_msg)


    def forward_kinematics(self, omega_left, omega_right):
        v = (self.r / 2) * (omega_right + omega_left)
        omega = (self.r / self.L) * (omega_right - omega_left)
        return v, omega

    def inverse_kinematics(self, v, omega):
        omega_left = (2 * v - omega * self.L) / (2 * self.r)
        omega_right = (2 * v + omega * self.L) / (2 * self.r)
        return omega_left, omega_right
        

def main():
    rclpy.init()
    node = DiffKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()