from scooter_interfaces.srv import PickSelectionConfirm

import rclpy
from rclpy.node import Node


class PickSelectionConfirmServiceServer(Node):
    """PickSelectionConfrim service server node"""
    def __init__(self):
        super().__init__('pick_selection_confirm_service_server')
        self.srv = self.create_service(PickSelectionConfirm, 'pick_selection_confirm', self.pick_selection_confirm_callback)

    def pick_selection_confirm_callback(self, request, response):
        """
        Awaits the user to press the YES button or NO button
        Also segments selected objected in background

        :param request: Contains pc2 stitched cloud and int32[] center
        :param response: returns button transition, center, and stitched cloud, and sample points
        :return: response
        :rtype: `PickSelectionConfirm.Response`
        """
        # Pass through unchanging info
        response.center = request.center
        response.stitched_cloud = request.stitched_cloud

        # TODO: First call Segmentation action to acquire stitched_cloud
        # response.sample_points = cloud future

        # TODO: implement yes/no button logic
        response.transition = PickSelectionConfirm.Response.YES
        self.get_logger().info('Yes button pushed!')
        # response.transition = PickSelectionConfirm.Response.NO  # if no is pushed

        # wait here until the Segmentation action is actually complete
        # (spinning loading wheel or something in UI)
        return response


def main():
    rclpy.init()

    pick_selection_confirm_service_server = PickSelectionConfirmServiceServer()

    rclpy.spin(pick_selection_confirm_service_server)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

