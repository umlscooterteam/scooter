from scooter_fsm.state import State
from scooter_interfaces.srv import WaitForBegin, PickSelection, PickSelectionConfirm, Pick, HoldingObject, Basket
from scooter_manipulation.joint_configs import JointConfigs


class DriveState(State):
    @staticmethod
    def run(node, result):
        """
        Drive mode, waiting for the begin button to be pressed. Calls the ``WaitForBegin`` service.

        :param node: :class:`scooter_fsm.scooter_fsm.ScooterFSMNode` as context for this state
        :param result: The result from the previous state
        :return: Tuple containing the next state :class:`scooter_fsm.state.State` and the result of this state
        :rtype: tuple
        """
        goal_handle = node.go_to_joint_config(JointConfigs.TRAVEL)

        request = WaitForBegin.Request()
        result = node.send_request(request)

        success = node.get_joint_config_result(goal_handle)

        return PickSelectionState(), result


class PickSelectionState(State):
    @staticmethod
    def run(node, result):
        """
        User is selecting an object to pick. Calls the ``PickSelection`` service.

        :param node: :class:`scooter_fsm.scooter_fsm.ScooterFSMNode` as context for this state
        :param result: The result from the previous state
        :return: Tuple containing the next state :class:`scooter_fsm.state.State` and the result of this state
        :rtype: tuple
        """

        request = PickSelection.Request()
        result = node.send_request(request)

        if result.transition == result.PICK:
            return PickSelectionConfirmState(), result
        elif result.transition == result.BACK:
            return DriveState(), result


class PickSelectionConfirmState(State):
    @staticmethod
    def run(node, result):
        """
        User is confirming the object that they have selected. Calls the ``PickSelectionConfirm`` service.

        :param node: :class:`scooter_fsm.scooter_fsm.ScooterFSMNode` as context for this state
        :param result: The result from the previous state
        :return: Tuple containing the next state :class:`scooter_fsm.state.State` and the result of this state
        :rtype: tuple
        """

        request = PickSelectionConfirm.Request()
        request.stitched_cloud = result.stitched_cloud
        request.center = result.center

        result = node.send_request(request)

        if result.transition == result.YES:
            return PickState(), result
        elif result.transition == result.NO:
            return DriveState(), result


class PickState(State):
    @staticmethod
    def run(node, result):
        """
        ``Robot arm is picking the object. Calls the ``Pick`` service.

        :param node: :class:`scooter_fsm.scooter_fsm.ScooterFSMNode` as context for this state
        :param result: The result from the previous state
        :return: Tuple containing the next state :class:`scooter_fsm.state.State` and the result of this state
        :rtype: tuple
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
        The robot arm is now holding an object. Calls the ``HoldingObject`` service.

        :param node: :class:`scooter_fsm.scooter_fsm.ScooterFSMNode` as context for this state
        :param result: The result from the previous state
        :return: Tuple containing the next state :class:`scooter_fsm.state.State` and the result of this state
        :rtype: tuple
        """

        request = HoldingObject.Request()
        result = node.send_request(request)

        if result.transition == result.BASKET:
            return BasketState(), result
        else:
            return DriveState(), result


class BasketState(State):
    @staticmethod
    def run(node, result):
        """
        The robot arm is placing the object in the basket. Calls the ``Basket`` service.

        :param node: :class:`scooter_fsm.scooter_fsm.ScooterFSMNode` as context for this state
        :param result: The result from the previous state
        :return: Tuple containing the next state :class:`scooter_fsm.state.State` and the result of this state
        :rtype: tuple
        """

        request = Basket.Request()
        result = node.send_request(request)

        return DriveState(), result
