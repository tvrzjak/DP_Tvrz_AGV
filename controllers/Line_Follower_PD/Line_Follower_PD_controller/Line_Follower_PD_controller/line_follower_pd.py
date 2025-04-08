#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String, ColorRGBA
from collections import deque

class LineFollowController(Node):
    def __init__(self):
        super().__init__("line_follower")
        self.get_logger().info("Initializing Line Follower PD Controller")

        # Subscriptions
        self.create_subscription(Float32, "line_center", self.measure_callback, 10)
        self.create_subscription(String, "qr_code", self.qr_code_callback, 10)

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.led_publisher = self.create_publisher(ColorRGBA, "leds/color", 10)

        # Constants
        self.ROBOT_CENTER = 118
        self.Kp = 0.015
        self.Ki = 0.000  # Ki je nastaveno na 0, takže není použito
        self.Kd = 0.00   # Kd je nastaveno na 0, takže není použito
        self.max_vel_z = 0.6
        self.min_vel_z = -0.6

        # State variables
        self.error_sum = 0.0
        self.last_error = 0.0
        self.last_time = self.get_clock().now().nanoseconds / 1e9  # Time in seconds
        self.speed = 0.7
        self.last_center = 0.0

    def qr_code_callback(self, msg):
        code = msg.data
        self.get_logger().info(f"Received QR code: {code}")

        if code == "C01G":
            self.speed = 0.7
            self.get_logger().info("Speed set to 0.7 due to C01G")
        elif code == "C01C":
            color = ColorRGBA(r=0.0, g=0.0, b=255.0, a=1.0)
            self.led_publisher.publish(color)
            self.get_logger().info("LED set to blue due to C01C")
        elif code == "C01X":
            self.speed = 0.3
            self.get_logger().info("Speed set to 0.3 due to C01X")
        elif code == "C01Y":
            color = ColorRGBA(r=255.0, g=0.0, b=0.0, a=1.0)
            self.led_publisher.publish(color)
            self.get_logger().info("LED set to red due to C01Y")

    def measure_callback(self, msg):
        center = msg.data

        # Filtrování velkých skoků
        if abs(self.last_center - center) > 50:
            self.get_logger().warn(f"Ignoring large jump in center: {center} (last: {self.last_center})")
            return
        self.last_center = center

        # Časový rozdíl
        current_time = self.get_clock().now().nanoseconds / 1e9
        delta_time = current_time - self.last_time
        if delta_time <= 0:
            delta_time = 0.001  # Zabránění dělení nulou, minimální hodnota

        # PID výpočet (pouze P, protože Ki a Kd jsou 0)
        error = center  # Chyba je relativní vůči nule, nikoli ROBOT_CENTER (dle tvého kódu)
        self.error_sum += error * delta_time
        delta_error = (error - self.last_error) / delta_time

        pid_output = (self.Kp * error) + (self.Ki * self.error_sum) + (self.Kd * delta_error)

        # Omezení úhlové rychlosti
        angular_z = max(self.min_vel_z, min(self.max_vel_z, pid_output))

        # Publikování příkazu
        msg = Twist()
        msg.linear.x = self.speed
        msg.angular.z = angular_z
        self.cmd_vel_publisher.publish(msg)
        self.get_logger().debug(f"Published cmd_vel: linear.x={msg.linear.x}, angular.z={msg.angular.z}")

        # Aktualizace stavových proměnných
        self.last_error = error
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down due to keyboard interrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()