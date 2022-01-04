import rclpy
from rclpy.node import Node

from scooter_fsm.states import *
# from scooter_interfaces.srv import WaitForBegin, PickSelection, PickSelectionConfirm, Pick, HoldingObject, Basket


class ScooterFSMNode(Node):
    """Node that handles state transitions"""
    def __init__(self):
        super().__init__('scooter_fsm')

        self.clients = {
            WaitForBegin: self.create_client(WaitForBegin, "wait_for_begin"),
            PickSelection: self.create_client(PickSelection, "pick_selection"),
            PickSelectionConfirm: self.create_client(PickSelectionConfirm, "pick_selection_confirm"),
            Pick: self.create_client(Pick, "pick"),
            HoldingObject: self.create_client(HoldingObject, "holding_object"),
            Basket: self.create_client(Basket, "basket")
        }

        # loop through states defined in scooter_fsm.states
        state = DriveMode()
        while rclpy.ok():
            state = state.run(self)
            self.get_logger().info(f"Now in state: {state}")

    def send_request(self, request):
        """
        Send a request given any service definition that is defined in `ScooterFSMNode`. If a client is not already
        defined for given interface type, this function will log an error and no request will be made.

        :param request: The request to send
        :return: The result from the service, or `None` if a client is not defined for this interface type
        """
        try:
            # get client from type of message
            client = self.client[type(request)]

            # wait for the client to come up
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("Service not available, trying again in 1s...")

            # call service, return result
            future = self.cli.call_async(request)
            self.spin_until_future_complete(future)
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
