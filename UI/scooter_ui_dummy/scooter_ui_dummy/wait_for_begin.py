from scooter_interfaces.srv import WaitForBegin

import rclpy
from rclpy.node import Node


class WaitForBeginServiceServer(Node):
    """WaitForBegin service server node"""
    def __init__(self):
        super().__init__('wait_for_begin_service_server')
        self.srv = self.create_service(WaitForBegin, 'wait_for_begin', self.wait_for_begin_callback)

    def wait_for_begin_callback(self, request, response):
        """
        Awaits the user to press the begin button

        :param request: None
        :param response: None
        :return: response
        :rtype: None
        """
        self.get_logger().info('Awaiting begin button...')
        # TODO: implement begin button logic
        self.get_logger().info('Begin button pushed!')
        return response


def main():
    rclpy.init()

    wait_for_begin_service_server = WaitForBeginServiceServer()

    rclpy.spin(wait_for_begin_service_server)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

