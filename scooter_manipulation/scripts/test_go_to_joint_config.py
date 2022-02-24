#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from scooter_interfaces.action import GoToJointConfig


class JointConfigs:
    fold_config = [0.0263192318379879, 1.2939172983169556, -1.7775753180133265, -1.2353704611407679, -1.5866463820086878, 0.1328919529914856]
    travel_config = [1.585824966430664, 1.1334773302078247, -2.7559364477740687, -0.05789691606630498, -1.5676992575274866, 0.1261676698923111]


class TestGoToJointConfig(Node):
    def __init__(self):
        super().__init__("test_gtjc")
        self._action_client = ActionClient(self, GoToJointConfig, "go_to_joint_config")

    def send_goal(self, position):
        goal_msg = GoToJointConfig.Goal()
        goal_msg.position = position

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)

    def get_result(self):
        self._action_client.wait_for_server()

        return self._action_client.get_result_async()

def send_goal_and_get_result(action_client, goal):
    future = action_client.send_goal(goal)
    rclpy.spin_until_future_complete(action_client, future)
    goal_handle = future.result()

    get_result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(action_client, get_result_future)

    result = get_result_future.result().result
    status = get_result_future.result().status

    return result, status

def main(args=None):
    rclpy.init(args=args)
    action_client = TestGoToJointConfig()

    while rclpy.ok():
        action_client.get_logger().info(f"Going to fold config...")
        result, status = send_goal_and_get_result(action_client, JointConfigs.fold_config)
        if status == GoalStatus.STATUS_SUCCEEDED:
            action_client.get_logger().info(f"Goal succeded! Result: {result}")
        else:
            action_client.get_logger().info('Goal failed with status code: {0}'.format(status))

        action_client.get_logger().info(f"Going to travel config...")
        result, status = send_goal_and_get_result(action_client, JointConfigs.travel_config)
        if status == GoalStatus.STATUS_SUCCEEDED:
            action_client.get_logger().info(f"Goal succeded! Result: {result}")
        else:
            action_client.get_logger().info('Goal failed with status code: {0}'.format(status))

    action_client.destroy()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
