"""Mock ROS2 modules for testing without ROS2 installation."""

# Mock rclpy
class MockNode:
    def __init__(self, name):
        self.name = name
        self._publishers = []
        self._subscribers = []
        self._parameters = {}
    
    def create_publisher(self, msg_type, topic, qos):
        pub = MockPublisher(topic, msg_type)
        self._publishers.append(pub)
        return pub
    
    def create_subscription(self, msg_type, topic, callback, qos):
        sub = MockSubscription(topic, msg_type, callback)
        self._subscribers.append(sub)
        return sub
    
    def declare_parameter(self, name, default):
        self._parameters[name] = default
    
    def get_parameter(self, name):
        class Param:
            value = self._parameters.get(name)
        return Param()
    
    def get_logger(self):
        return MockLogger()
    
    def get_clock(self):
        class Clock:
            def now(self):
                class Time:
                    def to_msg(self):
                        return {'sec': 0, 'nanosec': 0}
                return Time()
        return Clock()
    
    def destroy_node(self):
        pass

class MockPublisher:
    def __init__(self, topic, msg_type):
        self.topic = topic
        self.msg_type = msg_type
        self.published = []
    
    def publish(self, msg):
        self.published.append(msg)

class MockSubscription:
    def __init__(self, topic, msg_type, callback):
        self.topic = topic
        self.msg_type = msg_type
        self.callback = callback

class MockLogger:
    def info(self, msg, **kwargs):
        pass
    def debug(self, msg, **kwargs):
        pass
    def warn(self, msg, **kwargs):
        pass
    def error(self, msg, **kwargs):
        pass

def init():
    pass

def shutdown():
    pass

def spin(node):
    pass

# Mock message types
class PoseStamped:
    def __init__(self):
        self.header = MockHeader()
        self.pose = MockPose()

class MockHeader:
    def __init__(self):
        self.stamp = {'sec': 0, 'nanosec': 0}
        self.frame_id = ''

class MockPose:
    def __init__(self):
        self.position = MockPoint()
        self.orientation = MockQuaternion()

class MockPoint:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

class MockQuaternion:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 1.0

class String:
    def __init__(self):
        self.data = ''

class Image:
    pass

class Imu:
    pass

class Odometry:
    def __init__(self):
        self.header = MockHeader()
        self.pose = type('obj', (object,), {'pose': MockPose()})()

# Mock action client
class ActionClient:
    def __init__(self, node, action_type, action_name):
        self.node = node
        self.action_type = action_type
        self.action_name = action_name
    
    def wait_for_server(self, timeout=None):
        return True
    
    def send_goal_async(self, goal):
        class Future:
            def result(self):
                class Result:
                    status = 4  # SUCCEEDED
                return Result()
        return Future()

# Mock Node base
class Node(MockNode):
    pass