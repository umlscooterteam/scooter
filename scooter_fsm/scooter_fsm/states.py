from scooter_fsm.state import State
# from scooter_interfaces.srv import WaitForBegin, PickSelection, PickSelectionConfirm, HoldingObject, Basket


class DriveMode(State):
    @staticmethod
    def run(node):
        """Drive mode, calls ``WaitForBegin`` service

        :param: :class:`scooter_fsm.scooter_fsm.ScooterFSMNode` as context for this state
        :return: The next state
        :rtype: :class:`scooter_fsm.state.State`
        """
        return DriveMode()
