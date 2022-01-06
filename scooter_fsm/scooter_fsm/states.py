from scooter_fsm.state import State
# from scooter_interfaces.srv import WaitForBegin, PickSelection, PickSelectionConfirm, Pick, HoldingObject, Basket
from scooter_interfaces.srv import PickSelection


class DriveState(State):
    @staticmethod
    def run(node, result):
        """Drive mode, calls ``WaitForBegin`` service

        :param: :class:`scooter_fsm.scooter_fsm.ScooterFSMNode` as context for this state
        :return: The next state
        :rtype: :class:`scooter_fsm.state.State`
        """

        # request = WaitForBegin.Request()
        # result = node.send_request(request)

        return PickSelectionState(), result


class PickSelectionState(State):
    @staticmethod
    def run(node, result):
        """Drive mode, calls ``WaitForBegin`` service

        :param: :class:`scooter_fsm.scooter_fsm.ScooterFSMNode` as context for this state
        :return: The next state
        :rtype: :class:`scooter_fsm.state.State`
        """

        request = PickSelection.Request()
        result = node.send_request(request)

        if result.transition == result.PICK:
            return PickSelectionConfirmState(), result
        elif result.transition == result.BACK:
            return Drive(), result


class PickSelectionConfirmState(State):
    @staticmethod
    def run(node, result):
        """Drive mode, calls ``WaitForBegin`` service

        :param: :class:`scooter_fsm.scooter_fsm.ScooterFSMNode` as context for this state
        :return: The next state
        :rtype: :class:`scooter_fsm.state.State`
        """

        request = PickSelectionConfirm.Request()
        request.stitched_cloud = result.stitched_cloud
        request.center = result.center

        result = node.send_request(request)

        if result.transition == result.PICK:
            return Pick(), result
        elif result.transition == result.BACK:
            return Drive(), result
