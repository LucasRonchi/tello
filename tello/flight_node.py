import rclpy
from rclpy.node import Node

from std_msgs.msg import Int8, UInt8, UInt16, Float32, String, Empty, Int8MultiArray
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge


class FlightNode(Node):
    def __init__(self, node: Node):
        self.node = node
        super().__init__('flight_node')

        # To use Image interface
        self.br = CvBridge()

        # Set subscriptions
        self.sub_frame        = self.create_subscription(Image,   'tello/frame',              self.callback_frame, 1)
        self.sub_attitude     = self.create_subscription(Vector3, 'tello/state/attitude',     self.callback_state_attitude, 1)
        self.sub_speed        = self.create_subscription(Vector3, 'tello/state/speed',        self.callback_state_speed, 1)
        self.sub_templ        = self.create_subscription(UInt8,   'tello/state/templ',        self.callback_state_templ, 1)
        self.sub_temph        = self.create_subscription(UInt8,   'tello/state/temph',        self.callback_state_temph, 1)
        self.sub_tof          = self.create_subscription(UInt8,   'tello/state/tof',          self.callback_state_tof, 1)
        self.sub_h            = self.create_subscription(Int8,    'tello/state/h',            self.callback_state_h, 1)
        self.sub_bat          = self.create_subscription(UInt8,   'tello/state/bat',          self.callback_state_bat, 1)
        self.sub_baro         = self.create_subscription(Float32, 'tello/state/baro',         self.callback_state_baro, 1)
        self.sub_time         = self.create_subscription(UInt16,  'tello/state/time',         self.callback_state_time, 1)
        self.sub_acceleration = self.create_subscription(Vector3, 'tello/state/acceleration', self.callback_state_acceleration, 1)

        # Set publisher
        self.pub_emergency = self.create_publisher(Empty,          'tello/emergency')
        self.pub_takeoff   = self.create_publisher(Empty,          'tello/takeoff')
        self.pub_land      = self.create_publisher(Empty,          'tello/land')
        self.pub_stop      = self.create_publisher(Empty,          'tello/stop')
        self.pub_rc        = self.create_publisher(Int8MultiArray, 'tello/rc')
        self.pub_streamon  = self.create_publisher(Empty,          'tello/streamon')
        self.pub_streamoff = self.create_publisher(Empty,          'tello/streamoff')
        self.pub_flip      = self.create_publisher(UInt8,          'tello/flip')
        self.pub_speed     = self.create_publisher(UInt8,          'tello/speed')
        self.pub_up        = self.create_publisher(UInt16,         'tello/up')
        self.pub_down      = self.create_publisher(UInt16,         'tello/down')
        self.pub_left      = self.create_publisher(UInt16,         'tello/left')
        self.pub_right     = self.create_publisher(UInt16,         'tello/right')
        self.pub_forward   = self.create_publisher(UInt16,         'tello/forward')
        self.pub_back      = self.create_publisher(UInt16,         'tello/back')
        self.pub_cw        = self.create_publisher(UInt16,         'tello/cw')
        self.pub_ccw       = self.create_publisher(UInt16,         'tello/ccw')
        # self.pub_go        = self.create_publisher(None,         'tello/go')
        # self.pub_curve     = self.create_publisher(None,         'tello/curve')


    def callback_frame(self, msg):
        self.frame = self.br.imgmsg_to_cv2(msg.data, encoding='rgb8')


    def callback_state_attitude(self, msg: Vector3):
        self.state_attitude = [
            msg.x,
            msg.y,
            msg.z,
        ]


    def callback_state_speed(self, msg: Vector3):
        self.state_speed = [
            msg.x,
            msg.y,
            msg.z,
        ]


    def callback_state_templ(self, msg: UInt8):
        self.state_templ = msg.data


    def callback_state_temph(self, msg: UInt8):
        self.state_temph = msg.data


    def callback_state_tof(self, msg: UInt8):
        self.state_tof = msg.data


    def callback_state_h(self, msg: Int8):
        self.state_h = msg.data


    def callback_state_bat(self, msg: UInt8):
        self.state_bat = msg.data


    def callback_state_baro(self, msg: Float32):
        self.state_baro = msg.data


    def callback_state_time(self, msg: UInt16):
        self.state_time = msg.data


    def callback_state_acceleration(self, msg: Vector3):
        self.state_acceleration = [
            msg.x,
            msg.y,
            msg.z,
        ]


def main(args=None):
    rclpy.init(args=args)
    node = FlightNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
