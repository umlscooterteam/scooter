from scooter_interfaces.srv import GetCloud

import rclpy
from rclpy.node import Node


class GetCloudService(Node):
    """GetCloud service server node"""
    def __init__(self):
        super().__init__('get_cloud_service')
        self.srv = self.create_service(GetCloud, 'get_cloud', self.get_cloud_callback)

    def get_cloud_callback(self, request, response):
        """
        Stitches point-cloud together from realsense cameras

        :param request: no request msg for GetCloud
        :param response: The response to the service (pc2 stitched_cloud and success bool)
        :return: response
        :rtype: GetCloud.srv
        """
        response.success = True
        self.get_logger().info('Get Cloud Service Called')

        return response


def main():
    rclpy.init()

    get_cloud_service = GetCloudService()

    rclpy.spin(get_cloud_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
