from scooter_interfaces.srv import PickSelection
from scooter_interfaces.action import GetCloud

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class PickSelectionServiceServer(Node):
    """PickSelection service server node"""
    def __init__(self):
        super().__init__('pick_selection_service_server')
        self.srv = self.create_service(PickSelection, 'pick_selection', self.pick_selection_callback)
        self.get_cloud_client = GetCloudActionClient()
        self.get_cloud_result = None

    def pick_selection_callback(self, request, response):
        """
        Awaits the user to press the PICK button or BACK button

        :param request: None
        :param response: returns button pick and center, and stitched cloud
        :return: response
        :rtype: `PickSelection.Response`
        """

        # Call GetCloud action to acquire stitched_cloud (async)
        self.get_logger().info('Sending GetCloud action goal')
        self.get_cloud_client.send_goal()

        # TODO: Allow the user to select a point in the pointcloud
        # This may be tapping on a photo, or moving a laser, etc
        # response.center = ...  # must assign this value!

        # TODO: allow user to push pick/back button
        response.transition = PickSelection.Response.PICK  # for now, we automatically push PICK
        self.get_logger().info('Pick button pushed!')
        # response.transition = PickSelection.Response.BACK  # if back is pushed
        # self.get_logger().info('Back button pushed!')

        # wait here until the GetCloud action is actually complete
        # (spinning loading wheel or something in UI while the while loop is not complete)
        while self.get_cloud_result is None:
            rclpy.spin_once(self.get_cloud_client)
            self.get_cloud_result = self.get_cloud_client.result
        self.get_logger().info('GetCloud Complete')

        if self.get_cloud_result.success:
            pass  # GetCloud action returns success = True

        # assign result from GetCloud to PickSelection response msg
        response.stitched_cloud = self.get_cloud_result.stitched_cloud
        return response


class GetCloudActionClient(Node):
    """GetCloudActionClient for calling GetCloud action"""
    def __init__(self):
        super().__init__('get_cloud_action_client')
        self._get_cloud_client = ActionClient(self, GetCloud, 'get_cloud')
        self.result = None

    def send_goal(self):
        """
        sends GetCloud goal to GetCloud action server

        :return: None
        :rtype: None
        """
        goal_msg = GetCloud.Goal()

        self._get_cloud_client.wait_for_server()  # wait for server to become available

        self._send_goal_future = self._get_cloud_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.get_cloud_response_callback)

    def get_cloud_response_callback(self, future):
        """
        Callback when GetCloud action server has gotten the goal sent.

        :param future: goal_handle for our GetCloud action request
        :return: None
        :rtype: None
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_cloud_result_callback)

    def get_cloud_result_callback(self, future):
        """
        Callback when GetCloud action server returns a response.

        :param future: Response from GetCloud action server
        :return: None
        :rtype None:
        """
        self.result = future.result().result


def main(args=None):
    rclpy.init(args=args)

    pick_selection_service_server = PickSelectionServiceServer()

    rclpy.spin(pick_selection_service_server)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

