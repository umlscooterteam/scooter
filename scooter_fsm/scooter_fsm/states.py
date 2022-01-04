from scooter_fsm.state import State
from scooter_interfaces.srv import WaitForBegin, PickSelection, PickSelectionConfirm, HoldingObject, Basket


class DriveMode(State):
    """Waiting for user to begin pick action

    Calls service: `wait_for_begin`
    """
    @staticmethod
    def run(node):
        return DriveMode()
