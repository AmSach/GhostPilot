"""Mock nav_msgs for testing."""

class Odometry:
    def __init__(self):
        self.header = None
        self.child_frame_id = ''
        self.pose = None
        self.twist = None