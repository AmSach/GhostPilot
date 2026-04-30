"""Mock sensor_msgs for testing."""

class Image:
    def __init__(self):
        self.header = None
        self.height = 0
        self.width = 0
        self.data = b''

class Imu:
    def __init__(self):
        self.header = None
        self.orientation = None
        self.angular_velocity = None
        self.linear_acceleration = None