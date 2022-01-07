import rclpy
from rclpy.node import Node

from scooter_interfaces.srv import Pick


class PickServiceServer(Node):
    """Pick service server node"""
    def __init__(self):
        super().__init__("pick_service")
        self.srv = self.create_service(Pick, 'pick', self.pick_callback)

    def pick_callback(self, request, response):
        """
        Move the robot arm to pick the selected object given ``sample_points``, ``stitched_cloud``, and ``center``.

        :param request: ``Pick`` request, contains ``sample_points``, ``stitched_cloud``, and ``center`` attributes
        :param response: definition of response for ``Pick`` request
        :return: response: result from ``Pick`` request, containing ``success`` boolean
        """

        # TODO: generate grasp pose with ROS2 Grasp Library (uses GPD)
        # TODO: add stitched_cloud to MoveIt collision environment w/ Octomap
        # TODO: generate trajectory with MoveIt
        # TODO: execute trajectory

        response.success = True

        return response
