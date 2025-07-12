import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty, Int8MultiArray, UInt8, UInt16

from socket import socket


class TelloControlNode(Node):
    def __init__(self):
        super().__init__('tello_control_node')

        # Connect to tello to send commands
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', 8889))
        self._send_command('command')

        # All command for tello:
        # 'command', 'takeoff', 'land', 'emergency', 'streamon', 'streamoff', 'up', 'down', 'left', 
        # 'right', 'forward', 'back', 'ccw', 'cw', 'flip', 'go', 'curve', 'stop', 'speed', 'rc'

        # Set subscription
        self.sub_emergency = self.create_subscription(Empty,  'tello/emergency',   self.callback_emergency)
        self.sub_takeoff   = self.create_subscription(Empty,  'tello/takeoff',     self.callback_takeoff)
        self.sub_land      = self.create_subscription(Empty,  'tello/land',        self.callback_land)
        self.sub_stop      = self.create_subscription(Empty,  'tello/stop',        self.callback_stop)
        self.sub_rc        = self.create_subscription(Int8MultiArray, 'tello/rc',  self.callback_rc)
        self.sub_streamon  = self.create_subscription(Empty,  'tello/streamon',    self.callback_streamon)
        self.sub_streamoff = self.create_subscription(Empty,  'tello/streamoff',   self.callback_streamoff)
        self.sub_flip      = self.create_subscription(UInt8,  'tello/flip',        self.callback_flip)
        self.sub_speed     = self.create_subscription(UInt8,  'tello/speed',       self.callback_speed)
        self.sub_up        = self.create_subscription(UInt16, 'tello/up',          self.callback_up)
        self.sub_down      = self.create_subscription(UInt16, 'tello/down',        self.callback_down)
        self.sub_left      = self.create_subscription(UInt16, 'tello/left',        self.callback_left)
        self.sub_right     = self.create_subscription(UInt16, 'tello/right',       self.callback_right)
        self.sub_forward   = self.create_subscription(UInt16, 'tello/forward',     self.callback_forward)
        self.sub_back      = self.create_subscription(UInt16, 'tello/back',        self.callback_back)
        self.sub_cw        = self.create_subscription(UInt16, 'tello/cw',          self.callback_cw)
        self.sub_ccw       = self.create_subscription(UInt16, 'tello/ccw',         self.callback_ccw)
        # self.sub_go        = self.create_subscription(None, 'tello/go', self.callback_go)
        # self.sub_curve     = self.create_subscription(None, 'tello/curve', self.callback_curve)


    def _send_command(self, command: str):
        self.sock.sendto(command.encode())


    def callback_emergency(self, msg: Empty):
        self.get_logger().info('Send "EMERGENCY" command.')
        self._send_command('emergency')


    def callback_takeoff(self, msg: Empty):
        self.get_logger().info('Send "TAKEOFF" command.')
        self._send_command('takeoff')


    def callback_land(self, msg: Empty):
        self.get_logger().info('Send "LAND" command.')
        self._send_command('land')

    
    def callback_stop(self, msg: Empty):
        self.get_logger().info('Send "STOP" command')


    def callback_rc(self, msg: Int8MultiArray):
        command = f'rc {" ".join(msg.data)}'
        self.get_logger().info(f'Send "{command.upper()}" command.')
        self._send_command(command)

    
    def callback_streamon(self, msg: Empty):
        self.get_logger().info('Send "STREAMON" command.')
        self._send_command('streamon')


    def callback_streamoff(self, msg: Empty):
        self.get_logger().info('Send "STREAMOFF" command.')
        self._send_command('streamoff')


    def callback_flip(self, msg: UInt16):
        # 0: Front, 1: Back, 2: Right, 3: Left
        directions = ['f', 'b', 'r', 'l']
        command = f'flip {directions[msg.data]}'
        self.get_logger().info(f'Send "{command.upper()}" command.')
        self._send_command(command)


    def callback_speed(self, msg: UInt16):
        self.get_logger().info(f'Send "STOP {msg.data}" command.')
        self._send_command(f'stop {msg.data}')


    def callback_up(self, msg: UInt16):
        self.get_logger().info('Send "UP" command.')
        self._send_command(f'up {msg.data}')

        
    def callback_down(self, msg: UInt16):
        self.get_logger().info('Send "DOWN" command.')
        self._send_command(f'down {msg.data}')

        
    def callback_left(self, msg: UInt16):
        self.get_logger().info('Send "LEFT" command.')
        self._send_command(f'left {msg.data}')

        
    def callback_right(self, msg: UInt16):
        self.get_logger().info('Send "RIGHT" command.')
        self._send_command(f'right {msg.data}')

        
    def callback_forward(self, msg: UInt16):
        self.get_logger().info('Send "FORWARD" command.')
        self._send_command(f'forward {msg.data}')

        
    def callback_back(self, msg: UInt16):
        self.get_logger().info('Send "BACK" command.')
        self._send_command(f'back {msg.data}')

        
    def callback_cw(self, msg: UInt16):
        self.get_logger().info('Send "CW" command.')
        self._send_command(f'cw {msg.data}')

        
    def callback_ccw(self, msg: UInt16):
        self.get_logger().info('Send "CCW" command.')
        self._send_command(f'ccw {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = TelloControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
