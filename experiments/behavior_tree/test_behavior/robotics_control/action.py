from enum import Enum
from typing import List
import sys
import os
sys.path.append(os.path.abspath(os.path.dirname(__file__)))  # 
from message import Message

class ActionStatus(Enum):
    PENDING = "pending"
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"
    FAILED = "failed"

class Action:
    """Action class that contains multiple messages (not just left/right)."""

    def __init__(self, messages: List[Message]):
        if len(messages) < 1:
            raise ValueError("A action must have at least one messages.")
        self.messages = messages
        self.status = ActionStatus.PENDING

    def execute(self):
        """Execute all messages in the Action."""
        self.status = ActionStatus.IN_PROGRESS
        try:
            for msg in self.messages:
                msg.execute()
            self.status = ActionStatus.COMPLETED
        except Exception as e:
            print(f"Action execution failed: {e}")
            self.status = ActionStatus.FAILED

    def __repr__(self):
        return f"Action(status={self.status}, messages={self.messages})"

