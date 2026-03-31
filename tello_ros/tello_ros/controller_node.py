import rclpy
from rclpy.node import Node

from tello_ros_interfaces.srv import CommandEmpty, CommandFlip, CommandDirection, CommandRotation, CommandGoTo, CommandCurve, SetSpeed, SetRC, SetWifi
import socket


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # Conexão com Tello
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', 8889))

        # Iniciar SDK
        self._send_command('command')

        # Control commands
        self.srv_takeoff = self.create_service(
            CommandEmpty,
            'takeoff',
            lambda request, response: self._send_command('takeoff', response),
        )
        self.srv_land = self.create_service(
            CommandEmpty,
            'land',
            lambda request, response: self._send_command('land', response),
        )
        self.srv_streamon = self.create_service(
            CommandEmpty,
            'streamon',
            lambda request, response: self._send_command('streamon', response),
        )
        self.srv_streamoff = self.create_service(
            CommandEmpty,
            'streamoff',
            lambda request, response: self._send_command('streamoff', response),
        )
        self.srv_emergency = self.create_service(
            CommandEmpty,
            'emergency',
            lambda request, response: self._send_command('emergency', response),
        )
        self.srv_flip = self.create_service(
            CommandFlip,
            'flip',
            lambda request, response: self._send_command(f'flip {request.direction}', response),
        )
        self.srv_up = self.create_service(
            CommandDirection,
            'up',
            lambda request, response: self._send_command(f'up {request.distance}', response),
        )
        self.srv_down = self.create_service(
            CommandDirection,
            'down',
            lambda request, response: self._send_command(f'down {request.distance}', response),
        )
        self.srv_left = self.create_service(
            CommandDirection,
            'left',
            lambda request, response: self._send_command(f'left {request.distance}', response),
        )
        self.srv_right = self.create_service(
            CommandDirection,
            'right',
            lambda request, response: self._send_command(f'right {request.distance}', response),
        )
        self.srv_forward = self.create_service(
            CommandDirection,
            'forward',
            lambda request, response: self._send_command(f'forward {request.distance}', response),
        )
        self.srv_back = self.create_service(
            CommandDirection,
            'back',
            lambda request, response: self._send_command(f'back {request.distance}', response),
        )
        self.srv_cw = self.create_service(
            CommandRotation,
            'cw',
            lambda request, response: self._send_command(f'cw {request.angle}', response),
        )
        self.srv_ccw = self.create_service(
            CommandRotation,
            'ccw',
            lambda request, response: self._send_command(f'ccw {request.angle}', response),
        )
        self.srv_go = self.create_service(
            CommandGoTo,
            'go',
            lambda request, response: self._send_command(f'go {request.x} {request.y} {request.z} {request.speed}', response),
        )
        self.srv_curve = self.create_service(
            CommandCurve,
            'curve',
            lambda request, response: self._send_command(f'curve {request.x1} {request.y1} {request.z1} {request.x2} {request.y2} {request.z2} {request.speed}', response),
        )

        # Set Commands
        self.srv_speed = self.create_service(
            SetSpeed,
            'speed',
            lambda request, response: self._send_command(f'speed {request.speed}', response),
        )
        self.srv_rc = self.create_service(
            SetRC,
            'rc',
            lambda request, response: self._send_command(f'rc {request.roll} {request.pitch} {request.throttle} {request.yaw}', response),
        )
        self.srv_wifi = self.create_service(
            SetWifi,
            'wifi',
            lambda request, response: self._send_command(f'wifi {request.ssid} {request.password}', response),
        )


    def _send_command(self, command: str, response=None):
        self.get_logger().info(f'Send "{command}" command.')
        self.sock.sendto(command.encode(), ('192.168.10.1', 8889))  # IP padrão do Tello

        try:
            self.sock.settimeout(6)
            data, _ = self.sock.recvfrom(1024)
            string = data.decode('utf-8')
            if response:
                response.success = False
                response.message = string

            if string == 'ok':
                self.get_logger().info(f'Response for "{command}": "{string}".')
                if response:
                    response.success = True

            else:
                self.get_logger().error(f'Response for "{command}": "{string}".')

        except socket.timeout:
            self.get_logger().error(f'Timeout for "{command}".')
            if response:
                response.success = False
                response.message = 'timeout'
        
        if response:
            return response


    def destroy_node(self):
        self.get_logger().info('Encerrando conexão com Tello...')
        self.sock.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    controller_node = ControllerNode()
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
