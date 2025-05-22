import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge


class TelloCameraNode(Node):
    def __init__(self):
        super().__init__('tello_camera_node')

        # Connect to tello to receive frame and convert image
        self.frame_reader = cv2.VideoCapture('udp://@0.0.0.0:11111')
        self.br = CvBridge()

        # Set publisher
        self.pub_command = self.create_publisher(String, 'tello/command')
        self.pub_frame = self.create_publisher(Image, 'tello/frame')

        # Set timer
        self.frequency = 30 # Hz
        self.timer = self.create_timer(1/self.frequency, self.timer_callback)


    def timer_callback(self):
        try:
            ret, frame = self.frame_reader.read()
            
            if ret:
                img_msg = self.br.cv2_to_imgmsg(frame, encoding='rgb8')
                self.pub_frame.publish(img_msg)
            
            else:
                # Enable camera
                self.pub_command.publish(String(data='streamon'))

        except:
            # Enable camera
            self.pub_command.publish(String(data='streamon'))


def main(args=None):
    rclpy.init(args=args)
    node = TelloCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
