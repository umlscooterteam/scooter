from scooter_fsm.state import State
from scooter_interfaces.srv import WaitForBegin, PickSelection, PickSelectionConfirm, Pick, HoldingObject, Basket


class DriveState(State):
    @staticmethod
    def run(node, result):
        """
        :param node: :class:`scooter_fsm.scooter_fsm.ScooterFSMNode` as context for this state
        :param result: The result from the previous state
        :return: The next state
        :rtype: :class:`scooter_fsm.state.State`
        """

        request = WaitForBegin.Request()
        result = node.send_request(request)

        return PickSelectionState(), result


class PickSelectionState(State):
    @staticmethod
    def run(node, result):
        """
        :param node: :class:`scooter_fsm.scooter_fsm.ScooterFSMNode` as context for this state
        :param result: The result from the previous state
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
        """
        :param node: :class:`scooter_fsm.scooter_fsm.ScooterFSMNode` as context for this state
        :param result: The result from the previous state
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


class PickState(State):
    @staticmethod
    def run(node, result):
        """
        :param node: :class:`scooter_fsm.scooter_fsm.ScooterFSMNode` as context for this state
        :param result: The result from the previous state
        :return: The next state
        :rtype: :class:`scooter_fsm.state.State`
        """

        request = Pick.Request()
        request.sample_points = result.sample_points
        request.stitched_cloud = result.stitched_cloud
        request.center = result.center

        result = node.send_request(request)

        if result.success:
            return HoldingObjectState(), result
        else:
            return DriveState(), result


class HoldingObjectState(State):
    @staticmethod
    def run(node, result):
        """
        :param node: :class:`scooter_fsm.scooter_fsm.ScooterFSMNode` as context for this state
        :param result: The result from the previous state
        :return: The next state
        :rtype: :class:`scooter_fsm.state.State`
        """

        request = HoldingObject.Request()
        result = node.send_request()

        if result.transition == result.BASKET:
            return BasketState(), result
        else:
            return DriveState(), result


class BasketState(State):
    @staticmethod
    def run(node, result):
        """
        :param node: :class:`scooter_fsm.scooter_fsm.ScooterFSMNode` as context for this state
        :param result: The result from the previous state
        :return: The next state
        :rtype: :class:`scooter_fsm.state.State`
        """

        request = Basket.Request()
        result = node.send_request()

        return DriveState(), result
