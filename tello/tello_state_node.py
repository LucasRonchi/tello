import rclpy
from rclpy.node import Node

from std_msgs.msg import Int8, UInt8, UInt16, Float32
from geometry_msgs.msg import Vector3

from socket import socket

import re


class TelloStateNode(Node):
    def __init__(self):
        super().__init__('tello_state_node')

        # Connect to tello to receive state
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', 8890))

        # State regex
        self.regex = re.compile(
            r"pitch:(-?\d+);"
            r"roll:(-?\d+);"
            r"yaw:(-?\d+);"
            r"vgx:(-?\d+);"
            r"vgy:(-?\d+);"
            r"vgz:(-?\d+);"
            r"templ:(-?\d+);"
            r"temph:(-?\d+);"
            r"tof:(-?\d+);"
            r"h:(-?\d+);"
            r"bat:(-?\d+);"
            r"baro:(-?\d+\.\d+);"
            r"time:(-?\d+);"
            r"agx:(-?\d+\.\d+);"
            r"agy:(-?\d+\.\d+);"
            r"agz:(-?\d+\.\d+);\r\n"
        )

        # Set publisher
        self.pub_attitude     = self.create_publisher(Vector3, 'tello/state/attitude')
        self.pub_speed        = self.create_publisher(Vector3, 'tello/state/speed')
        self.pub_templ        = self.create_publisher(UInt8,   'tello/state/templ')
        self.pub_temph        = self.create_publisher(UInt8,   'tello/state/temph')
        self.pub_tof          = self.create_publisher(UInt8,   'tello/state/tof')
        self.pub_h            = self.create_publisher(Int8,    'tello/state/h')
        self.pub_bat          = self.create_publisher(UInt8,   'tello/state/bat')
        self.pub_baro         = self.create_publisher(Float32, 'tello/state/baro')
        self.pub_time         = self.create_publisher(UInt16,  'tello/state/time')
        self.pub_acceleration = self.create_publisher(Vector3, 'tello/state/acceleration')

        # Set timer
        self.frequency = 10 # Hz
        self.timer = self.create_timer(1/self.frequency, self.timer_callback)


    def timer_callback(self):
        state, server = self.sock.recvfrom(1024)

        if not server:
            return

        match = self.regex.match(state.decode('utf-8'))

        if not match:
            return

        values = match.groups()
    
        self.pub_attitude.publish(
            Vector3(
                x=int(values[0]), 
                y=int(values[1]), 
                z=int(values[2])
            )
        )
        self.pub_speed.publish(
            Vector3(
                x=int(values[3]), 
                y=int(values[4]), 
                z=int(values[5])
            )
        )
        self.pub_templ.publish(UInt8(msg=int(values[6])))
        self.pub_temph.publish(UInt8(msg=int(values[7])))
        self.pub_tof.publish(UInt8(msg=int(values[8])))
        self.pub_h.publish(Int8(msg=int(values[9])))
        self.pub_bat.publish(UInt8(msg=int(values[10])))
        self.pub_baro.publish(Float32(msg=float(values[11])))
        self.pub_time.publish(UInt16(msg=int(values[12])))
        self.pub_acceleration.publish(
            Vector3(
                x=float(values[13]), 
                y=float(values[14]), 
                z=float(values[15])
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = TelloStateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
