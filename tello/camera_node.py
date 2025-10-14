import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Empty
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Connect to tello to receive frame and convert image
        self.frame_reader = cv2.VideoCapture('udp://@0.0.0.0:11111')
        self.br = CvBridge()

        # Set publisher
        self.pub_command = self.create_publisher(Empty, 'tello/streamon', qos_profile_sensor_data)
        self.pub_frame = self.create_publisher(Image, 'tello/frame', qos_profile_sensor_data)

        # Set timer
        self.timer = self.create_timer(1/30, self.timer_callback)


    def timer_callback(self):
        try:
            while self.frame_reader.grab():
                ret, frame = self.frame_reader.retrieve()
            
            if ret:
                img_msg = self.br.cv2_to_imgmsg(frame, encoding='rgb8')
                self.pub_frame.publish(img_msg)
            
            else:
                # Enable camera
                self.pub_command.publish(Empty())

        except:
            # Enable camera
            self.pub_command.publish(Empty())

    def destroy_node(self):
        self.frame_reader.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
