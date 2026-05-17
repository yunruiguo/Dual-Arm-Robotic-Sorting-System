# -----------------------------------------------------------------------------
# 
# -----------------------------------------------------------------------------
from grasp_msgs.msg import ArmCommand, GripperCommand  # ,

class ArmFeedbackMsg:
    def __init__(self):
        self.pose = ArmCommand()
        self.distance_remaining = 0.0
        self.is_running = False
        self.success = False

    def reset(self):
        """."""
        self.pose = ArmCommand()
        self.distance_remaining = 0.0
        self.is_running = False
        self.success = False


class GripperFeedbackMsg:
    def __init__(self):
        self.pose = GripperCommand()
        self.success = False
        self.distance_remaining = 0.0
        self.is_running = False

    def reset(self):
        """."""
        self.pose = GripperCommand()
        self.distance_remaining = 0.0
        self.is_running = False
        self.success = False

