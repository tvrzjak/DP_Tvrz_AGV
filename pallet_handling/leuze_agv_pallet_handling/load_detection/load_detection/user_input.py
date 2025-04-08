import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class UserInputNode(Node):
    def __init__(self):
        """
        Class which wait for user input and publish it when user agree
            publish:    user response (Bool)
        """

        super().__init__('user_input_node')
        self.publisher = self.create_publisher(Bool, 'user_response', 10)
        self.subscription = None

    def get_user_input(self) -> Bool:
        """
        Wait for user input
        """

        response: str = input("Do you want to dock under the pallet? (y/n): ")
        if response.lower() == "y":
            return True
        else:
            return False

    def publish_response(self, response: Bool):
        """
        Publishing Bool topic based on user response
        """

        msg = Bool()
        msg.data = response
        self.publisher.publish(msg)
        self.get_logger().info('Published response: %s' % response)

    def main_loop(self):
        """
        Loop for repetitive asking for user input
        """

        while rclpy.ok():
            user_response = self.get_user_input()
            self.publish_response(user_response)


def main(args=None):
    """
    Main starting method
    """

    rclpy.init(args=args)
    node = UserInputNode()
    node.main_loop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
