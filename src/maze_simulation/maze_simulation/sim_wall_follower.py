import rclpy
import math
import time
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan



class TurtlePub(Node):
		def __init__(self):
				super().__init__('TurtlePub')
				self.get_logger().info(f'Hello World')





def main(args=None):
                rclpy.init(args=args)
                turtle_pub = TurtlePub()
                rclpy.spin(turtle_pub)
                turtle_pub.destroy_node()
                rclpy.shutdown()

if __name__ == '__main__':
        main()
