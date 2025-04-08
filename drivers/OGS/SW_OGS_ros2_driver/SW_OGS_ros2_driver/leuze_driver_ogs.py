#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial


class LeuzeDriverOGS(Node):
    def __init__(self):
        super().__init__("leuze_driver_ogs")
        self.get_logger().info("Initializing Leuze OGS driver")

        self.ROBOT_CENTER = 118
        self.serialPort = None

        # Pokus o připojení k sériovému portu
        try:
            self.serialPort = serial.Serial(
                port="/dev/ttyUSB0",
                baudrate=115200,
                bytesize=8,
                timeout=0.05,
                parity=serial.PARITY_ODD,
                stopbits=serial.STOPBITS_ONE
            )
            self.get_logger().info("Serial port connected successfully")
        except serial.SerialException as e:
            self.get_logger().warn(f"Failed to connect to serial port: {str(e)}. Running without device.")

        # Vytvoření publisheru a timeru bez ohledu na připojení
        self.center_publisher = self.create_publisher(Float32, "line_center", 10)
        self.timer = self.create_timer(0.05, self.measurement)

    def measurement(self):
        if self.serialPort is None:
            self.get_logger().warn("No serial device connected. Skipping measurement.")
            return

        try:
            self.serialPort.write(b'\x11\x00\xCF\x00\x00\xDE')
            serialString = self.serialPort.readline()

            if len(serialString) < 9:
                self.get_logger().warn("Incomplete data received from serial port")
                return

            left = int.from_bytes(serialString[5:7], "little") / 10 - self.ROBOT_CENTER
            right = int.from_bytes(serialString[7:9], "little") / 10 - self.ROBOT_CENTER
            width = right - left
            center = left + width / 2

            msg = Float32()
            msg.data = float(center)
            self.center_publisher.publish(msg)
            self.get_logger().info(f"Published line center: {msg.data}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {str(e)}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in measurement: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = LeuzeDriverOGS()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down due to keyboard interrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()