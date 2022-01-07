from scooter_interfaces.srv import PickSelection

import rclpy
from rclpy.node import Node


class PickSelectionServiceServer(Node):
    """PickSelection service server node"""
    def __init__(self):
        super().__init__('pick_selection_service_server')
        self.srv = self.create_service(PickSelection, 'pick_selection', self.pick_selection_callback)

    def pick_selection_callback(self, request, response):
        """
        Awaits the user to press the PICK button or BACK button

        :param request: None
        :param response: returns button pick and center, and stitched cloud
        :return: response
        :rtype: `PickSelection.Response`
        """
        # TODO: First call GetCloud action to acquire stitched_cloud
        # response.stitched_cloud = cloud future

        # TODO: implement pick/back button logic
        response.transition = PickSelection.Response.PICK
        self.get_logger().info('Pick button pushed!')
        # response.transition = PickSelection.Response.BACK  # if back is pushed

        # wait here until the GetCloud action is actually complete
        # (spinning loading wheel or something in UI)
        return response


def main():
    rclpy.init()

    pick_selection_service_server = PickSelectionServiceServer()

    rclpy.spin(pick_selection_service_server)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

