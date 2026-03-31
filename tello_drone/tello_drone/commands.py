import rclpy
from rclpy.node import Node
from tello_ros_interfaces.srv import CommandEmpty, CommandFlip, CommandDirection, CommandRotation, CommandGoTo, CommandCurve, SetSpeed, SetRC, SetWifi


class ForwardClient(Node):

    def __init__(self):
        super().__init__('forward_client')

        self.cli = self.create_client(CommandEmpty, 'takeoff')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def send_request(self):
        req = CommandEmpty.Request()
        req.distance = 1

        self.future = self.cli.call_async(req)


def main():
    rclpy.init()

    client = ForwardClient()
    client.send_request()

    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            try:
                response = client.future.result()
                client.get_logger().info(f'Response: {response.success}, {response.message}')
            except Exception as e:
                client.get_logger().error(f'Service call failed: {e}')
            break

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()