"""
This node is responsible for controlling the robot based on the signal received from the signal_node.
The signal can be one of the following: 'stop', 'foward', 'turn_left', 'turn_right', 'working', 'give_way'.
The robot should stop for 5 seconds when the signal is 'stop'.
The robot should move forward when the signal is 'foward'.
The robot should turn left when the signal is 'turn_left'.
The robot should turn right when the signal is 'turn_right'.
The robot should slow down for 5 seconds when the signal is 'working'.
The robot should slow down for 2 seconds when the signal is 'give_way'.

The node should publish the velocity to the 'cmd_vel' topic.
The node should publish the point to the 'points' topic.

The node should subscribe to the 'Signal' topic to receive the signal.
The node should subscribe to the 'crt_vel' topic to receive the velocity.

Author: Tony Miranda
    
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time

class SignalLogicNode(Node):
    def __init__(self):
        
        super().__init__('signal_logic_node')
        
        # Variable to store the signal
        self.signal = None
        # Variable to store the velocity
        self.velocity = Twist()
        
        # Subscription
        self.signal_sub = self.create_subscription(
            String,
            'Signal',
            self.signal_callback,
            10
        )
        self.vel_sub = self.create_subscription(
            Twist,
            'ctr_vel',
            self.vel_callback,
            10
        )
        # Publisher
        self.vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        self.point_pub = self.create_publisher(
            String,
            'points',
            10
        )
        # Timer
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Logger
        self._logger = self.get_logger('SignalLogicNode initailized')
        

    def signal_callback(self, msg):
        self.signal = msg.data
        
    def vel_callback(self, msg):
        self.velocity = msg
    
    def timer_callback(self):
        if self.signal == 'stop':
            self.stop_robot()
        elif self.signal == 'foward':
            self.move_forward()
        elif self.signal == 'turn_left':
            self.turn_left()
        elif self.signal == 'turn_right':
            self.turn_right()
        elif self.signal == 'working':
            self.working()
        elif self.signal == 'give_way':
            self.give_way()
        else:
            self.move_forward()
            self._logger.info('No signal received')

    def stop_robot(self):
        # Code to stop the robot
        # make the robot stop for 5 seconds
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0
        self.vel_pub.publish(self.velocity)
        time.sleep(5)
        self.move_forward()
        
    def move_forward(self):
        # Code to move the robot forward
        # quit any rotation
        self.velocity.linear.z = 0.0 # No rotation
        self.pub.publish(self.velocity)
        
    def turn_left(self):
        # Send a Message to "points_node" to turn the robot left
        self.point_pub.publish('left')
        

    def turn_right(self):
        # Send a Message to "points_node" to turn the robot right
        self.point_pub.publish('right')
        
        
    def working(self):
        # Code to make the robot work
        # slow down the robot for 
        self.velocity.linear.x = self.velocity.linear.x/2
        self.vel_pub.publish(self.velocity)
        time.sleep(5)
        self.move_forward()
        
    def give_way(self):
        # Code to give way to other vehicles
        # slow down the robot for 2 seconds
        self.velocity.linear.x = self.velocity.linear.x/2
        self.vel_pub.publish(self.velocity)
        time.sleep(2)
        self.move_forward()

    

def main(args=None):
    rclpy.init(args=args)
    signal_logic_node = SignalLogicNode()
    rclpy.spin(signal_logic_node)
    signal_logic_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()