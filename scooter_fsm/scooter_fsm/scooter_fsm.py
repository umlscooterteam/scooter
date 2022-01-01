import rclpy
from rclpy.node import Node

from scooter_fsm.states import *


class ScooterFSMNode(Node):
    """Node that handles state transitions"""
    def __init__(self):
        super().__init__('scooter_fsm')
        self.test_string = "h"

        state = DriveMode()

        # loop through states defined in scooter_fsm.states
        while rclpy.ok():
            state = state.run(self)


def main(args=None):
    rclpy.init(args=args)
    scooter_fsm_node = ScooterFSMNode()

    rclpy.spin(scooter_fsm_node)

    scooter_fsm_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
