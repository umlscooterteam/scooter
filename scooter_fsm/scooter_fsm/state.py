class State:
    """A state"""
    @staticmethod
    def run(*args, **kwargs):
        """
        Functionality for a state can be implemented by overriding this function

        :param: args: Placeholder for any context that needs to be passed to the state
        :param: kwargs: Placeholder for any context that needs to be passed to the state
        :return: The next state
        :rtype: :class:`scooter_fsm.state.State`
        """
        pass

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return self.__class__.__name__
