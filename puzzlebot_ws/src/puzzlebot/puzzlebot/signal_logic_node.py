import numpy as np
import os
import rclpy
from rclpy.node import Node
import time

from std_msgs.msg import String, twist

class SignalLogicNode(Node):
    def __init__(self):
        super().__init__('signal_logic_node')
        
        # Subscribers
        self.sub = self.create_subscription(String, 'sign', self.sign_callback, 10)
        self.sub = self.create_subscription(twist, 'ctr_vel', self.cmd_vel_callback, 10)

        #Publisher
        self.pub = self.create_publisher(twist, 'cmd_vel', 10)
        
        # Timers
        timer_period = 0.08
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Variables
        self.sign = String()
        self.vel = twist()
        
        self.has_sign = False
        
    def sign_callback(self, msg):
        self.sign = msg.data
        self.has_sign = True
    
    def cmd_vel_callback(self, msg):
        self.vel = msg.data
    
    def go (self):
        self.pub.publish(self.vel)
    def stop (self):
        self.vel.linear.x = 0
        self.vel.angular.z = 0
        self.pub.publish(self.vel) 
    def slow_down (self):
        self.vel.linear.x = self.vel.linear.x / 2
        self.pub.publish(self.vel)
    
    def timer_callback(self):
        
        if self.has_sign:    
            # Behaviors
            if self.sign == "stop":
                self.stop()
                time.sleep(5)
                self.go()
            elif self.sign == "working":
                self.slow_down()
                time.sleep(5)
                self.go()
            elif self.sign == "give_way":
                self.slow_down()
                time.sleep(2)
                self.go()
            
            # Directions
            elif self.sign == "forward":
                self.go()
            elif self.sign == "left":
                self.vel.angular.z = 0.5
                self.pub.publish(self.vel)
            elif self.sign == "right":
                self.vel.angular.z = -0.5
                self.pub.publish(self.vel)
                
            # Traffic lights
            elif self.sign == "green":
                self.go()
            elif self.sign == "yellow":
                self.slow_down()
            elif self.sign == "red":
                self.stop()
            else:
                self.has_sign = False
        else: 
                self.go()