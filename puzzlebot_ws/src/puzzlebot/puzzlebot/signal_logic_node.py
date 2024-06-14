import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

from directions_msgs.msg import Signal

from rclpy import qos

from time import sleep

class SignalLogicNode(Node):
    def __init__(self):
        super().__init__('signal_logic_node')
        
        # Subscribers
        self.direction_sign = self.create_subscription(Signal, '/direction', self.direction_callback, qos.qos_profile_sensor_data)
        self.sub_vel = self.create_subscription(Twist, '/ctrl_vel', self.cmd_vel_callback, qos.qos_profile_sensor_data)

        # Publisher
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.01, self.timer_callback)

        self.last_direction = 3
        self.last_behavior = 3
        self.last_light = 3
        self.vel_inc = Twist()
        self.speed_factor = 1.0
        self.initial_time = 0
        self.delay = 0
    
        self.behaviour = False

        self.get_logger().info('Signal Logic Node Started')

    def direction_callback(self, msg):
        """
            self.dir = 0 -> Go straight
            self.dir = 1 -> Turn Left
            self.dir = 2 -> Turn Right
            self.dir = 3 -> Not signal
            
            self.beh = 0 -> Give way
            self.beh = 1 -> Stop
            self.beh = 2 -> Slow down
            self.beh = 3 -> Not signal
            
            self.light = 0 -> Green
            self.light = 1 -> Yellow
            self.light = 2 -> Red
            self.light = 3 -> Not signal
        """
        self.get_logger().warning(f'Direction: {msg.direction}, Behavior: {msg.behavior}, Light: {msg.light}')
        self.last_direction = msg.direction
        self.last_behavior = msg.behavior
        self.last_light = msg.light

        output_vel = Twist()
        sleep_time = 0
        if self.last_behavior != 3:
            output_vel.angular.z = self.vel_inc.angular.z
            if self.last_behavior == 0:
                output_vel.linear.x = 0.08
                sleep_time = 2
            elif self.last_behavior == 1:
                output_vel.linear.x = 0.0
                sleep_time = 10
            elif self.last_behavior == 2:
                output_vel.linear.x = 0.08
                sleep_time = 5
        self.behaviour = True
        self.pub.publish(output_vel)
        sleep(sleep_time)
        self.behaviour = False

    def timer_callback(self):   
        if not self.behaviour:
            self.pub.publish(self.vel_inc)  

    def cmd_vel_callback(self, msg):
        self.vel_inc = msg
        
    #     # Timers
    #     self.timer_period = 0.08
    #     self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
    #     self.beh = 0
    #     self.light = 0
    #     self.dir = 0

    #     self.signal = Signal()
    #     self.vel = Twist()
    #     self.vel_inc = Twist()

    #     self.last = []

    #     self.has_sign = False

    # def direction_callback(self, msg):
    #     self.signal = msg
    #     self.has_sign = True

    # def cmd_vel_callback(self, msg):
    #     self.vel_inc = msg

    # def timer_callback(self):
    #     if self.has_sign:
    #         self.dir = self.signal.direction
    #         self.beh = self.signal.behavior
    #         self.light = self.signal.light

    #         """
    #         self.dir = 0 -> Go straight
    #         self.dir = 1 -> Turn Left
    #         self.dir = 2 -> Turn Right
    #         self.dir = 3 -> Not signal
            
    #         self.beh = 0 -> Give way
    #         self.beh = 1 -> Stop
    #         self.beh = 2 -> Slow down
    #         self.beh = 3 -> Not signal
            
    #         self.light = 0 -> Green
    #         self.light = 1 -> Yellow
    #         self.light = 2 -> Red
    #         self.light = 3 -> Not signal
    #         """
    #         if self.dir != self.dir_last:
    #             if self.light != 3:
    #                 if self.dir == 0:
    #                     if self.light == 0:
    #                         self.vel.linear.x = self.vel_inc.linear.x
    #                         self.vel.angular.z = self.vel_inc.angular.z
    #                     elif self.light == 1:
    #                         self.vel.linear.x = 0.5 * self.vel_inc.linear.x
    #                         self.vel.angular.z = 0.5 * self.vel_inc.angular.z
    #                     elif self.light == 2:
    #                         self.vel.linear.x = 0
    #                         self.vel.angular.z = 0
    #                     else:
    #                         self.vel.linear.x = self.vel_inc.linear.x
    #                         self.vel.angular.z = self.vel_inc.angular.z
    #                     self.pub.publish(self.vel)

    #                 elif self.dir == 1:
    #                     self.right_turn()
    #                 elif self.dir == 2:
    #                     self.left_turn()
    #             else:
    #                 if self.beh == 0:
    #                     self.vel.linear.x = 0.5 * self.vel_inc.linear.x
    #                     self.vel.angular.z = 0.5 * self.vel_inc.angular.z
    #                     self.pub.publish(self.vel)
    #                     sleep(2)
    #                 elif self.beh == 1:
    #                     self.vel.linear.x = 0
    #                     self.vel.angular.z = 0
    #                     self.pub.publish(self.vel)
    #                     sleep(5)
    #                 elif self.beh == 2:
    #                     self.vel.linear.x = 0.5 * self.vel_inc.linear.x
    #                     self.vel.angular.z = 0.5 * self.vel_inc.angular.z
    #                     self.pub.publish(self.vel)
    #                     sleep(5)
    #         else:
    #             self.vel.linear.x = self.vel_inc.linear.x
    #             self.vel.angular.z = self.vel_inc.angular.z
    #             self.pub.publish(self.vel)

    #         self.dir_last = self.dir
    #         self.beh_last = self.beh
    #         self.light_last = self.light
    #         self.has_dir = False

    # def right_turn(self):
    #     self.vel.linear.x = 0.5 * self.vel_inc.linear.x
    #     self.vel.angular.z = -0.5 * self.vel_inc.angular.z
    #     self.pub.publish(self.vel)
    #     sleep(2)

    # def left_turn(self):
    #     self.vel.linear.x = 0.5 * self.vel_inc.linear.x
    #     self.vel.angular.z = 0.5 * self.vel_inc.angular.z
    #     self.pub.publish(self.vel)
    #     sleep(2)

def main(args=None):
    rclpy.init(args=args)
    node = SignalLogicNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()