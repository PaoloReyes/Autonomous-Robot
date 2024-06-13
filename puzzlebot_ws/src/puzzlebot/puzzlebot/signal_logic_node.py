import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from geometry_msgs.msg import Twist

from rclpy import qos
class SignalLogicNode(Node):
    def __init__(self):
        super().__init__('signal_logic_node')
        
        # Subscribers
        self.behaviour_sign = self.create_subscription(String, '/behaviour', self.behaviour_callback, 10)
        self.light_sign = self.create_subscription(String, '/light', self.light_callback, 10)
        self.direction_sign = self.create_subscription(String, '/direction', self.direction_callback, 10)
        self.sub_vel = self.create_subscription(Twist, '/ctrl_vel', self.cmd_vel_callback, qos.qos_profile_sensor_data)

        # Publisher
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Timers
        self.timer_period = 0.08
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # Variables
        self.beh = ""
        self.light = ""
        self.direction = ""
        
        self.vel = Twist()

        self.last = []

        self.has_beh = False
        self.has_light = False
        self.has_dir = False

        self.stop_timer = None
        self.slow_down_timer = None
        self.direction_timer = None

    def behaviour_callback(self, msg):
        self.beh = msg.data
        self.has_beh = True

    def light_callback(self, msg):
        self.light = msg.data
        self.has_light = True

    def direction_callback(self, msg):
        self.direction = msg.data
        self.has_dir = True

    def cmd_vel_callback(self, msg):
        self.vel = msg

    def timer_callback(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = SignalLogicNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()