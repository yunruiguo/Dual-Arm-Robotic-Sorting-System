import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from action import Action
from typing import List, Optional


class ActionQueue:
    """Queue to store actions."""

    def __init__(self):
        self.queue: List[Task] = []

    def add_action(self, action: Action):
        """Add a action to the queue."""
        self.queue.append(action)
        print(f"Action added. Queue size: {len(self.queue)}")

    def get_next_action(self) -> Optional[Action]:
        """Retrieve and remove the next action in the queue."""
        if self.queue:
            return self.queue.pop(0)
        return None

    def peek_next_action(self) -> Optional[Action]:
        """Peek at the next action without removing it."""
        return self.queue[0] if self.queue else None

    def is_empty(self) -> bool:
        """Check if the queue is empty."""
        return len(self.queue) == 0

    def __repr__(self):
        return f"ActionQueue(size={len(self.queue)})"

