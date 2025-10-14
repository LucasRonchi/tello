import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Empty, Int8MultiArray, UInt8, UInt16, String
from socket import socket

# Se criar mensagens customizadas, descomente
# from my_drone_interfaces.msg import GoTo, Curve

class ControllerNode(Node):
    def __init__(self):
        super().__init__('control_node')

        # Conexão com Tello
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', 8889))

        # Iniciar SDK
        self._send_command('command')

        # Subscriptions
        self._setup_flight_commands()        # takeoff, land, emergency, stream, flip
        self._setup_movement_commands()      # up, down, left, right, forward, back, go, curve
        self._setup_rotation_commands()      # cw, ccw
        self._setup_configuration_commands() # speed, rc, wifi

    def _setup_flight_commands(self):
        self.sub_takeoff = self.create_subscription(
            Empty,
            'tello/takeoff',
            lambda _: self._send_command('takeoff'),
            qos_profile_sensor_data,
        )
        self.sub_land = self.create_subscription(
            Empty,
            'tello/land',
            lambda _: self._send_command('land'),
            qos_profile_sensor_data,
        )
        self.sub_streamon = self.create_subscription(
            Empty,
            'tello/streamon',
            lambda _: self._send_command('streamon'),
            qos_profile_sensor_data,
        )
        self.sub_streamoff = self.create_subscription(
            Empty,
            'tello/streamoff',
            lambda _: self._send_command('streamoff'),
            qos_profile_sensor_data,
        )
        self.sub_emergency = self.create_subscription(
            Empty,
            'tello/emergency',
            lambda _: self._send_command('emergency'),
            qos_profile_sensor_data,
        )
        self.sub_flip = self.create_subscription(
            String,
            'tello/flip',
            lambda msg: self._send_flip_command(msg.data),
            qos_profile_sensor_data,
        )

    def _setup_movement_commands(self):
        directions = ['up', 'down', 'left', 'right', 'forward', 'back']
        for dir_name in directions:
            self.create_subscription(
                UInt16,
                f'tello/{dir_name}',
                lambda msg, d=dir_name: self._send_direction_command(d, msg.data),
                qos_profile_sensor_data,
            )

        self.sub_go = self.create_subscription(
            String,  # trocar para GoTo quando criar mensagem customizada
            'tello/go',
            lambda msg: self._send_command(f'go {msg.data}'),
            qos_profile_sensor_data,
        )
        self.sub_curve = self.create_subscription(
            String,  # trocar para Curve quando criar mensagem customizada
            'tello/curve',
            lambda msg: self._send_command(f'curve {msg.data}'),
            qos_profile_sensor_data,
        )

    def _setup_rotation_commands(self):
        self.sub_cw = self.create_subscription(
            UInt16,
            'tello/cw',
            lambda msg: self._send_rotate_command('cw',
            msg.data),
            qos_profile_sensor_data,
        )
        self.sub_ccw = self.create_subscription(
            UInt16,
            'tello/ccw',
            lambda msg: self._send_rotate_command('ccw',
            msg.data),
            qos_profile_sensor_data,
        )

    def _init_set_commands_subscription(self):
        self.sub_speed = self.create_subscription(
            UInt8,
            'tello/speed',
            lambda msg: self._send_speed_command(msg.data),
            qos_profile_sensor_data,
        )
        self.sub_rc = self.create_subscription(
            Int8MultiArray,
            'tello/rc',
            lambda msg: self._send_rc_command(msg.data),
            qos_profile_sensor_data,
        )
        self.sub_wifi = self.create_subscription(
            String,
            'tello/wifi',
            lambda msg: self._send_wifi_command(msg.data),
            qos_profile_sensor_data,
        )

    def _send_command(self, command: str):
        self.get_logger().info(f'Send "{command.upper()}" command.')
        self.sock.sendto(command.encode(), ('192.168.10.1', 8889))  # IP padrão do Tello

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
