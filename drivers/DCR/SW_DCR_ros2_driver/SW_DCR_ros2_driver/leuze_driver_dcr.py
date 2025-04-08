#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class DCRScan(Node):
    def __init__(self):
        super().__init__("line_follower")
        self.get_logger().info("Initializing DCR Scan driver")

        self.serialPort = None

        # Pokus o připojení k sériovému portu
        try:
            self.serialPort = serial.Serial(
                port="/dev/ttyUSB0",
                baudrate=9600,
                bytesize=8,
                timeout=0.05,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            self.get_logger().info("Serial port connected successfully")
        except serial.SerialException as e:
            self.get_logger().warn(f"Failed to connect to serial port: {str(e)}. Running without device.")

        # Vytvoření publisheru a timeru
        self.code_publisher = self.create_publisher(String, "qr_code", 10)
        self.timer = self.create_timer(0.05, self.measurement)

    def measurement(self):
        if self.serialPort is None:
            self.get_logger().warn("No serial device connected. Skipping measurement.")
            return

        try:
            serialString = self.serialPort.readline()
            if not serialString:
                self.get_logger().debug("No data received from serial port")
                return

            # Dekódování a extrakce kódu
            code = serialString.decode("ascii")[1:5]
            valid_codes = {"C01G", "C01m", "C01C", "C01X", "C01Y", "C01Z"}

            if code in valid_codes:
                self.get_logger().info(f"Detected valid QR code: {code}")
                msg = String()
                msg.data = code
                self.code_publisher.publish(msg)
        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {str(e)}")
        except UnicodeDecodeError:
            self.get_logger().warn("Failed to decode serial data as ASCII")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in measurement: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = DCRScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down due to keyboard interrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()