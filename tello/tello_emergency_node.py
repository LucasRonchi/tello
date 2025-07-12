import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty

from socket import socket


class TelloControlNode(Node):
    def __init__(self):
        super().__init__('tello_control_node')

        # Connect to tello to send commands
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', 8889))
        self._send_command('command')

        # Set subscription
        self.sub_command = self.create_subscription(Empty, 'tello/emergency', self.callback_emergency, 10)


    def _send_command(self, command: str):
        self.sock.sendto(command.encode())


    def callback_emergency(self, msg):
        self.get_logger().info(f'Send "EMERGENCY" command.')
        self._send_command('emergency')


def main(args=None):
    rclpy.init(args=args)
    node = TelloControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
