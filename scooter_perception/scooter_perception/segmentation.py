from scooter_interfaces.action import Segmentation

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node


class SegmentationActionServer(Node):
    """Segmentation action server node"""
    def __init__(self):
        super().__init__('segmentation_action_server')
        self._action_server = ActionServer(
            self,
            Segmentation,
            'segmentation',
            self.segmentation_callback
        )

    def segmentation_callback(self, goal_handle):
        """
        Segments out the object in the stitched pointcloud where the center is located

        :param: goal_handle: the handle to the current goal (access current action msg fields)
        :return: Segmentation action msg result (pc2 sample_points and pc2 stitched_cloud)
        :rtype: :class:`Segmentation.Result`
        """
        self.get_logger().info('Executing goal (Segmentation)...')

        # how to publish feedback
        feedback_msg = Segmentation.Feedback()
        feedback_msg.percentage_complete = 0
        self.get_logger().info('Feedback: {0}'.format(feedback_msg.percentage_complete))
        goal_handle.publish_feedback(feedback_msg)

        # how to get parameters of the goal
        request_msg = goal_handle.request
        stitched_cloud = request_msg.stitched_cloud
        center = request_msg.center

        # shows goal was successful (when complete with action)
        goal_handle.succeed()

        # how to set result to return
        result = Segmentation.Result()
        # result.sample_points = None  # TODO
        # result.stitched_cloud = None
        return result


def main(args=None):
    rclpy.init(args=args)

    get_cloud_action_server = SegmentationActionServer()

    rclpy.spin(get_cloud_action_server)


if __name__ == '__main__':
    main()
