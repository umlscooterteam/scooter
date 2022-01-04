from scooter_interfaces.action import GetCloud

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node


class GetCloudActionServer(Node):
    """GetCloud service server node"""
    def __init__(self):
        super().__init__('get_cloud_action_server')
        self._action_server = ActionServer(
            self,
            GetCloud,
            'get_cloud',
            self.get_cloud_callback
        )

    def get_cloud_callback(self, goal_handle):
        """
        Stitches point-cloud together from realsense cameras
        :param goal_handle: the handle to the current goal (access current action msg fields)
        :return: GetCloud action msg result (pc2 stitched_cloud and bool success)
        :rtype: GetCloud.Result
        """
        self.get_logger().info('Executing goal (GetCloud)...')
        feedback_msg = GetCloud.Feedback()
        feedback_msg.percentage_complete = 0

        # publish feedback
        self.get_logger().info('Feedback: {0}'.format(feedback_msg.percentage_complete))
        goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()  # shows goal was successful

        # setting result
        result = GetCloud.Result()
        # result.stitched_cloud = None  # TODO
        result.success = True
        return result


def main(args=None):
    rclpy.init(args=args)

    get_cloud_action_server = GetCloudActionServer()

    rclpy.spin(get_cloud_action_server)


if __name__ == '__main__':
    main()
