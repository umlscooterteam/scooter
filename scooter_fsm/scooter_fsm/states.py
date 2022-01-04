from scooter_fsm.state import State

# scooter services
# from scooter_interfaces.srv import GetCloud, Segmentation, Pick
# UI services
# from scooter_interfaces.srv import WaitForBegin, PickSelection, PickSelectionConfirm, HoldingObject, Basket


class DriveMode(State):
    """Waiting for user to begin pick action

    Calls service: `wait_for_begin`
    """
    @staticmethod
    def run(node):
        node.send_request(wait_for_begin_request)
        return DriveMode()
