import rclpy
from rclpy.node import Node
from rclpy import qos

from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist

from .submodules.fuzzy_controller import FuzzyController
class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.controller = FuzzyController()

        self.create_subscription(Int32MultiArray, '/CoM', self.com_callback, qos.qos_profile_sensor_data)

        self.cmd_vel_pub = self.create_publisher(Twist, 'ctrl_vel', qos.qos_profile_sensor_data)

        self.get_logger().info('Puzzlebot Controller Node Started')

    # def com_callback(self, msg):
    #     x, y = msg.data
    #     cmd_vel = Twist()
    #     cmd_vel.linear.x = y * self.KPLinear
    #     cmd_vel.angular.z = x * self.KPAngular
    #     self.cmd_vel_pub.publish(cmd_vel)

    def com_callback(self, msg):
        x, y = msg.data
        if y != 0:
            v, w = self.controller.compute(x, y)
        else:
            v = 0.0
            if x > 0:
                w = 0.7
            else:
                w = -0.7
        cmd_vel = Twist()
        cmd_vel.linear.x = v
        cmd_vel.angular.z = w
        self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()