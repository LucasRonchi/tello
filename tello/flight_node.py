import rclpy
from rclpy.node import Node

from std_msgs.msg import Int8, UInt8, UInt16, Float32, String
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image



class FlightNode(Node):
    def __init__(self):
        super().__init__('flight_node')

        # Set subscriptions
        self.sub_frame = self.create_subscription(Image, 'tello/frame', 10)
        self.sub_state_time = self.create_subscription(Image, 'tello/state/time', self.callback_state_time, 1)
        self.sub_ = self.create_subscription(Image, 'tello/', 10)

        # Set publisher
        self.pub_command = self.create_publisher(String, 'tello/command')

        # Ser timer
        self.timer_flight = self.create_timer(0.5, self.flight)


    def callback_state_time(self, msg: UInt16):
        pass



def main(args=None):
    rclpy.init(args=args)
    node = FlightNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
