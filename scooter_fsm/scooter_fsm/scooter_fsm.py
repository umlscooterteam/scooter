import rclpy
from rclpy.node import Node

from scooter_fsm import states
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from scooter_interfaces.srv import WaitForBegin, PickSelection, PickSelectionConfirm, Pick, HoldingObject, Basket
from scooter_interfaces.srv import PickSelection
from scooter_interfaces.action import GoToJointConfig


class ScooterFSMNode(Node):
    """Node that handles state transitions"""
    def __init__(self):
        super().__init__('scooter_fsm')

        self.get_logger().info("Initializing...")

        # dict to match type of request to a client in send_request() function
        self._clients = {
            type(WaitForBegin.Request()): self.create_client(WaitForBegin, "wait_for_begin"),
            type(PickSelection.Request()): self.create_client(PickSelection, "pick_selection"),
            type(PickSelectionConfirm.Request()): self.create_client(PickSelectionConfirm, "pick_selection_confirm"),
            type(Pick.Request()): self.create_client(Pick, "pick"),
            type(HoldingObject.Request()): self.create_client(HoldingObject, "holding_object"),
            type(Basket.Request()): self.create_client(Basket, "basket")
        }

        self._joint_config_action_client = ActionClient(self, GoToJointConfig, "go_to_joint_config")

        self.get_logger().info("Initialized!")

        # loop through states defined in scooter_fsm.states
        state = states.DriveState()
        self.get_logger().info(f"Initial state: {state}")
        result = None
        while rclpy.ok():
            state, result = state.run(self, result)
            self.get_logger().info(f"Now in state: {state}")

    def go_to_joint_config(self, position):
        """
        Set joint configuration goal and move arm. Returns a goal handle so that you can wait for the goal to be
        reached with `get_joint_config_result(goal_handle)`.

        :param position: A list of joint positions ordered from the shoulder to the wrist.
        :return: goal handle
        """
        goal_msg = GoToJointConfig.Goal()
        goal_msg.position = position

        self._joint_config_action_client.wait_for_server()

        future = self._joint_config_action_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        return goal_handle

    def get_joint_config_result(self, goal_handle):
        """
        Wait for the joint configuration goal to finish, returning `True` if it succeeded and `False` if it failed.

        :param goal_handle: the goal handle returned by `go_to_joint_config()`
        :return: `True` if the arm successfully reached the goal configuration, otherwise `False`
        """
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)

        result = get_result_future.result().result
        return result

    def send_request(self, request):
        """
        Send a request given any service definition that is defined in :class:`scooter_fsm.scooter_fsm.ScooterFSMNode`.
        If a client is not already defined for given interface type, this function will log an error and no request will
        be made.

        :param request: The request to send
        :return: The result from the service, or `None` if a client is not defined for this interface type
        """
        try:
            # get client from type of message
            client = self._clients[type(request)]

            # wait for the client to come up
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("Service not available, trying again in 1s...")

            # call service, return result
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            return future.result()

        except KeyError as e:
            # if we do not have a client defined for this service, exit
            self.get_logger().error(f"send_request called for invalid service type: {e}")
            return None


def main(args=None):
    rclpy.init(args=args)
    scooter_fsm_node = ScooterFSMNode()

    rclpy.spin(scooter_fsm_node)

    scooter_fsm_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
