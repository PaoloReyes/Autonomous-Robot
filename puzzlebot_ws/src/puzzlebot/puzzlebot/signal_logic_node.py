import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class SignalLogicNode(Node):
    def __init__(self):
        super().__init__('signal_logic_node')
        
        # Subscribers
        self.sub_sign = self.create_subscription(String, '/sign', self.sign_callback, 10)
        self.sub_vel = self.create_subscription(Twist, '/crt_vel', self.cmd_vel_callback, 10)

        # Publisher
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Timers
        self.timer_period = 0.08
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # Variables
        self.sign = ""
        self.vel = Twist()
        self.last_sign = ""
        self.has_sign = False
        self.stop_timer = None
        self.slow_down_timer = None
        self.direction_timer = None

    def sign_callback(self, msg):
        self.sign = msg.data
        self.has_sign = True

    def cmd_vel_callback(self, msg):
        self.vel = msg

    def go(self):
        self.pub.publish(self.vel)

    def stop(self):
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0
        self.pub.publish(self.vel)

    def slow_down(self):
        self.vel.linear.x = self.vel.linear.x / 2
        self.pub.publish(self.vel)

    def timer_callback(self):
        if self.has_sign:
            if self.sign != self.last_sign:
                if self.sign == "stop":
                    self.stop()
                    self.stop_timer = self.create_timer(5.0, self.go_after_stop)
                elif self.sign == "workers":
                    self.slow_down()
                    self.slow_down_timer = self.create_timer(5.0, self.go_after_slow_down)
                elif self.sign == "give_way":
                    self.slow_down()
                    self.slow_down_timer = self.create_timer(2.0, self.go_after_slow_down)
                elif self.sign == "forward":
                    self.go()
                elif self.sign == "left":
                    self.go()
                    self.direction_timer = self.create_timer(2.0, self.turn_left)
                elif self.sign == "right":
                    self.go()
                    self.direction_timer = self.create_timer(2.0, self.turn_right)
                elif self.sign == "green":
                    self.go()
                elif self.sign == "yellow":
                    self.slow_down()
                elif self.sign == "red":
                    self.stop()
                else:
                    self.go()
                self.last_sign = self.sign
        else:
            self.go()

    def go_after_stop(self):
        self.go()
        self.stop_timer.cancel()

    def go_after_slow_down(self):
        self.go()
        self.slow_down_timer.cancel()

    def turn_left(self):
        self.vel.angular.z = 0.5
        self.pub.publish(self.vel)
        self.direction_timer.cancel()

    def turn_right(self):
        self.vel.angular.z = -0.5
        self.pub.publish(self.vel)
        self.direction_timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = SignalLogicNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()