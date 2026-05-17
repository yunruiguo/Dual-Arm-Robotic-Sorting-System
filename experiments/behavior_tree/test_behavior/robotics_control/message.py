from typing import Callable, Optional, List

class Message:
    """Message class to store task-related data."""

    def __init__(self, index: int, action_func: Optional[Callable] = None,
        decision_logic: Optional[Callable] = None, **params):
        """
        :param index: (0=, 1=)
        :param action_func: 
        :param params: Parameters, list(kwargs.values())
        """
        self.index = index
        self.params: List = list(params.values())  # 
        self.action_func = action_func
        self.decision_logic = decision_logic
        self.pending_decision_logic = (decision_logic, action_func.__name__) if decision_logic else None

    def execute(self):
        """Execute the assigned action function with params."""
        if self.action_func:
            print(f"Executing {self.action_func.__name__} with params: {self.params}")
            self.action_func(self.index, *self.params)

    def __repr__(self):
        return f"Message(index={self.index}, params={self.params}, action_func={self.action_func.__name__ if self.action_func else None})"
