from scooter_fsm.state import State


class DriveMode(State):
    """Waiting for user to begin pick action

    Calls service: `wait_for_begin`
    """
    @staticmethod
    def run(node):
        print(node.test_string)
        return DriveMode()
