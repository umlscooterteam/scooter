from scooter_interfaces.srv import PickSelectionConfirm
from scooter_interfaces.action import Segmentation

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class PickSelectionConfirmServiceServer(Node):
    """PickSelectionConfrim service server node"""
    def __init__(self):
        super().__init__('pick_selection_confirm_service_server')
        self.srv = self.create_service(PickSelectionConfirm, 'pick_selection_confirm', self.pick_selection_confirm_callback)
        self.segmentation_client = SegmentationActionClient()
        self.segmentation_result = None

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

        # Call Segmentation action to acquire stitched_cloud
        self.get_logger().info('Sending Segmentation action goal')
        self.segmentation_client.send_goal(request.stitched_cloud, request.center)

        # TODO: allow user to press yes/no button
        response.transition = PickSelectionConfirm.Response.YES
        self.get_logger().info('Yes button pushed!')
        # response.transition = PickSelectionConfirm.Response.NO  # if no is pushed
        # self.get_logger().info('No button pushed!')

        # wait here until the Segmentation action is actually complete
        # (spinning loading wheel or something in UI)
        while self.segmentation_result is None:
            rclpy.spin_once(self.segmentation_client)
            self.segmentation_result = self.segmentation_client.result
        self.get_logger().info('Segmentation Complete')

        # assign result from Segmentation to PickSelectionConfirm response msg
        response.sample_points = self.segmentation_result.sample_points
        return response


class SegmentationActionClient(Node):
    """SegmentationActionClient for calling GetCloud action"""
    def __init__(self):
        super().__init__('segmentation_action_client')
        self._segmentation_client = ActionClient(self, Segmentation, 'segmentation')
        self.result = None

    def send_goal(self, stitched_cloud, center):
        """
        sends Segmentation goal to Segmentation action server


        :param stitched_cloud: pointcloud2 of environment
        :param center: 3d point in pointcloud of object we wish to segment
        :return: stitched cloud unchanged, and sample points (segmented object)
        :rtype: `Segmentation.Response`
        """
        goal_msg = Segmentation.Goal()
        goal_msg.stitched_cloud = stitched_cloud
        goal_msg.center = center

        self._segmentation_client.wait_for_server()  # wait for server to become available

        self._send_goal_future = self._segmentation_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.segmentation_response_callback)

    def segmentation_response_callback(self, future):
        """
        Callback when Segmentation action server has gotten the goal sent.

        :param future: goal_handle for our Segmentation action request
        :return: None
        :rtype: None
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.segmentation_result_callback)

    def segmentation_result_callback(self, future):
        """
        Callback when Segmentation action server returns a response.

        :param future: Response from Segmentation action server
        :return: None
        :rtype None:
        """
        self.result = future.result().result


def main():
    rclpy.init()

    pick_selection_confirm_service_server = PickSelectionConfirmServiceServer()

    rclpy.spin(pick_selection_confirm_service_server)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

