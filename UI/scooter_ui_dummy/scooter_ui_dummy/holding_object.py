from scooter_interfaces.srv import HoldingObject

import rclpy
from rclpy.node import Node


class HoldingObjectServiceServer(Node):
    """HoldingObject service server node"""
    def __init__(self):
        super().__init__('holding_object_service_server')
        self.srv = self.create_service(HoldingObject, 'holding_object', self.holding_object_callback)

    def holding_object_callback(self, request, response):
        """
        Awaits the user to press the BASKET button or PLACE button

        :param request: None
        :param response: returns button transition
        :return: response
        :rtype: `HoldingObject.Response`
        """
        # TODO: implement basket/place button logic
        response.transition = HoldingObject.Response.BASKET
        self.get_logger().info('BASKET button pushed!')
        # response.transition = HoldingObject.Response.PLACE  # if place is pushed

        return response


def main():
    rclpy.init()

    holding_object_service_server = HoldingObjectServiceServer()

    rclpy.spin(holding_object_service_server)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

