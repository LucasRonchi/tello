import time

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

        # Set publisher
        self.pub_streamon = self.create_publisher(Empty, 'tello/streamon', qos_profile_sensor_data)
        self.pub_frame = self.create_publisher(Image, 'tello/frame', qos_profile_sensor_data)

        # Connect to tello to receive frame and convert image
        self.frame_reader = cv2.VideoCapture('udp://@0.0.0.0:11111', cv2.CAP_FFMPEG)
        self.br = CvBridge()

        # Set timer
        self.fps = 30
        self.timer = self.create_timer(1 / self.fps, self.timer_callback)



    def timer_callback(self):
        ret, frame = self.frame_reader.read()

        if not ret:
            now = self.get_clock().now()
            if (now - self._last_streamon).nanoseconds > 2e9:
                self._last_streamon = now
                self.get_logger().warn('Sem frame — reenviando streamon')
                self.pub_streamon.publish(Empty())
            return

        img_msg = self.br.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_frame.publish(img_msg)

    def destroy_node(self):
        self.get_logger().info('Encerrando câmera...')
        if self.frame_reader.isOpened():
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
