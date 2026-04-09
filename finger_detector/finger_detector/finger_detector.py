import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class WebcamSubscriber(Node):

    def __init__(self):
        super().__init__('webcam_subscriber')

        self.subscription = self.create_subscription(
            Image,
            'webcam/image_raw',  # mesmo tópico do publisher
            self.listener_callback,
            10
        )

        self.bridge = CvBridge()

    def listener_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Aqui você pode processar o frame
            cv2.imshow("Webcam Subscriber", frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Erro ao converter imagem: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = WebcamSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()