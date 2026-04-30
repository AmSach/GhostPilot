"""Mock geometry_msgs for testing."""

class PoseStamped:
    def __init__(self):
        self.header = Header()
        self.pose = Pose()

class Header:
    def __init__(self):
        self.stamp = TimeStamp()
        self.frame_id = ''

class TimeStamp:
    def __init__(self):
        self.sec = 0
        self.nanosec = 0

class Pose:
    def __init__(self):
        self.position = Point()
        self.orientation = Quaternion()

class Point:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

class Quaternion:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 1.0

class TransformStamped:
    pass

class Vector3:
    pass