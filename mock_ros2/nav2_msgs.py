"""Mock nav2_msgs for testing."""

class NavigateToPose:
    class Goal:
        def __init__(self):
            self.pose = None
    
    class Result:
        def __init__(self):
            self.status = 0