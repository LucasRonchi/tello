import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty, String

from socket import socket


class TelloControlNode(Node):
    def __init__(self):
        super().__init__('tello_control_node')

        # Connect to tello to send commands
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', 8889))
        self._send_command('command')

        # Start list of commands and set possible commands
        self.list_commands = []
        self.possible_commands = (
            'command', 'takeoff', 'land', 'emergency', 'streamon', 'streamoff', 'up', 'down', 'left', 
            'right', 'forward', 'back', 'ccw', 'cw', 'flip', 'go', 'curve', 'stop', 'speed', 'rc'
        )

        # Set subscription
        self.sub_command = self.create_subscription(Empty, 'tello/emergency', self.callback_emergency, 1)
        self.sub_command = self.create_subscription(String, 'tello/command', self.callback_command, 10)

        # Set timer
        self.timer = self.create_timer(0.1, self.send_list_command)


    def _send_command(self, command: str):
        self.sock.sendto(command.encode())


    def callback_emergency(self, msg):
        self.get_logger().info(f'Send "EMERGENCY" command.')
        self._send_command('emergency')


    def callback_command(self, msg: String):
        if msg.data:
            if msg.data == 'emergency':
                self.get_logger().info(f'Send "EMERGENCY" command.')
                self._send_command('emergency')

            elif msg.data.startswith(self.possible_commands):
                self.list_commands.append(msg.data)


    def send_list_command(self):
        if not self.list_commands:
            return

        elif 'command' in self.list_commands:
            self.get_logger().info('Send "COMMAND" command.')
            self._send_command('command')
            self.list_commands.remove('command')

        elif 'emergency' in self.list_commands:
            self.get_logger().info('Send "EMERGENCY" command.')
            self._send_command('emergency')
            self.list_commands.remove('emergency')

        elif 'stop' in self.list_commands:
            self.get_logger().info('Send "STOP" command.')
            self._send_command('stop')
            self.list_commands.remove('stop')

        elif 'land' in self.list_commands:
            self.get_logger().info('Send "LAND" command.')
            self._send_command('land')
            self.list_commands.remove('land')

        else:
            last_command = self.list_commands.pop(0)
            self.get_logger().info(f'Send "{last_command.upper()}" command')
            self._send_command(last_command)


def main(args=None):
    rclpy.init(args=args)
    node = TelloControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
