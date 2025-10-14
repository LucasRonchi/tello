import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty, Int8MultiArray, UInt8, UInt16, String

from socket import socket


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        # Connect to tello to send commands
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', 8889))

        # Start tello sdk
        self._send_command('command')

        # Create control commands
        self._init_state_commands_subscription()
        self._init_direction_commands_subscription()
        self._init_rotate_commands_subscription()
        self._init_set_commands_subscription()

    def _init_state_commands_subscription(self):
        self.sub_takeoff = self.create_subscription(
            Empty,
            'tello/takeoff',
            lambda _: self._send_command('takeoff'),
        )
        self.sub_land = self.create_subscription(
            Empty,
            'tello/land',
            lambda _: self._send_command('land'),
        )
        self.sub_streamon = self.create_subscription(
            Empty,
            'tello/streamon',
            lambda _: self._send_command('streamon'),
        )
        self.sub_streamoff = self.create_subscription(
            Empty,
            'tello/streamoff',
            lambda _: self._send_command('streamoff'),
        )
        self.sub_emergency = self.create_subscription(
            Empty,
            'tello/emergency',
            lambda _: self._send_command('emergency'),
        )
        self.sub_flip = self.create_subscription(
            String,
            'tello/flip',
            lambda msg: self._send_flip_command(msg.data),
        )

    def _init_direction_commands_subscription(self):
        self.sub_up = self.create_subscription(
            UInt16,
            'tello/up',
            lambda msg: self._send_direction_command('up', msg.data),
        )
        self.sub_down = self.create_subscription(
            UInt16,
            'tello/down',
            lambda msg: self._send_direction_command('down', msg.data),
        )
        self.sub_left = self.create_subscription(
            UInt16,
            'tello/left',
            lambda msg: self._send_direction_command('left', msg.data),
        )
        self.sub_right = self.create_subscription(
            UInt16,
            'tello/right',
            lambda msg: self._send_direction_command('right', msg.data),
        )
        self.sub_forward = self.create_subscription(
            UInt16,
            'tello/forward',
            lambda msg: self._send_direction_command('forward', msg.data),
        )
        self.sub_back = self.create_subscription(
            UInt16,
            'tello/back',
            lambda msg: self._send_direction_command('back', msg.data),
        )
        self.sub_go = self.create_subscription(
            None,
            'tello/go',
            lambda _: self._send_command('go'),
        )
        self.sub_curve = self.create_subscription(
            None,
            'tello/curve',
            lambda _: self._send_command('curve'),
        )

    def _init_rotate_commands_subscription(self):
        self.sub_cw = self.create_subscription(
            UInt16,
            'tello/cw',
            lambda msg: self._send_rotate_command('cw', msg.data),
        )
        self.sub_ccw = self.create_subscription(
            UInt16,
            'tello/ccw',
            lambda msg: self._send_rotate_command('ccw', msg.data),
        )

    def _init_set_commands_subscription(self):
        self.sub_speed = self.create_subscription(
            UInt8,
            'tello/speed',
            lambda msg: self._send_speed_command(msg.data),
        )
        self.sub_rc = self.create_subscription(
            Int8MultiArray,
            'tello/rc',
            lambda msg: self._send_rc_command(msg.data),
        )
        self.sub_wifi = self.create_subscription(
            String,
            'tello/wifi',
            lambda msg: self._send_wifi_command(msg.data)
        )

    def _send_command(self, command: str):
        self.get_logger().info(f'Send "{command.upper()}" command.')
        self.sock.sendto(command.encode())

    def _send_flip_command(self, direction):
        if direction in ['left', 'right', 'forward', 'back', 'l', 'r', 'f', 'b']:
            self._send_command(f'flip {direction[0]}')

    def _send_direction_command(self, command, value):
        if 20 <= value <= 500:
            self._send_command(f'{command} {value}')

    def _send_rotate_command(self, command, value):
        if 1 <= value <= 3600:
            self._send_command(f'{command} {value}')

    def _send_speed_command(self, value):
        if 10 <= value <= 100:
            self._send_command(f'speed {value}')

    def _send_rc_command(self, commands):
        if len(commands) == 4 and all(-100 <= x <= 100 for x in commands):
            roll, pitch, throttle, yaw = commands
            self._send_command(f"rc {roll} {pitch} {throttle} {yaw}")

    def _send_wifi_command(self, ssid_password):
        if ssid_password.count(' ') == 1 and not ssid_password.startswith(' ') and not ssid_password.endswith(' '):
            self._send_command(f'wifi {ssid_password}')


def main(args=None):
    rclpy.init(args=args)
    control_node = ControlNode()
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
