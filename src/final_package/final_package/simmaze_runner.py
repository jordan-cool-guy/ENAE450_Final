import rclpy
import math
from math import pi
import time
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class TurtlePub(Node):
	def __init__(self):
		super().__init__('TurtlePub')
		self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
		self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)	

		#minimum distance
		self.minthreshold = 0.6
		#maximum distance
		self.maxthreshold = 6.0

		self.seen_wall = False
		self.aligned = False
		self.following_wall = False

	def scan_callback(self, msg):

		Left_Point = msg.ranges[90]
		#print("Left Point 90", Left_Point)
		Right_Point = msg.ranges[270]
		#print("Right Point 270", Right_Point)
		Front_Point = msg.ranges[0]
		#print("Front Point 0", Front_Point)


		#Average distances L - Left, R - Right, F - Front
		L = msg.ranges[60:90]
		L = sum(L)/len(L)
		print("Left Avg: ", L)

		R = msg.ranges[270:300]
		R = sum(R)/len(R)
		print("Right Avg: ", R)


		F = msg.ranges[359:] + msg.ranges[0:5]
		F = sum(F)/len(F)
		print("Front Avg: ", F)

		if (not self.seen_wall):
			self.get_logger().info(f'Moving to wall')
			self.move_to_wall(msg.ranges)
		elif (self.seen_wall and not self.aligned):
			self.align_to_wall(msg.ranges)
			self.get_logger().info(f'aligning')
		elif (self.seen_wall and self.aligned and self.following_wall):
			self.get_logger().info(f'following wall')
			self.follow_wall(msg.ranges)
	
	#
	def move_forward(self):
		twist = Twist()

		twist.linear.x = 0.5
		twist.angular.z = 0.0

		self.publisher.publish(twist)
		self.get_logger().info('Moving Forward')

	def turn_turtle(self,x,theta):
		twist = Twist()

		twist.linear.x = x
		twist.angular.z = theta

		self.publisher.publish(twist)
		self.get_logger().info('Turning Turtle')


	def stop_turtle(self):
		twist = Twist()

		twist.linear.x = 0.0
		twist.angular.z = 0.0

		self.publisher.publish(twist)
		self.get_logger().info('Stopping Turtle')
		

	#Prevents turtle from moving past minimum distance
	def move_to_wall(self, ranges):
		dist = ranges[0]
		if dist <= self.minthreshold:
			self.seen_wall = True
			self.stop_turtle()
		else:
			self.move_forward()
	
	#Finds wall (dependent)
	def find_wall(self, x, theta):
		twist = Twist()

		twist.linear.x = x
		twist.angular.z = theta

		self.publisher.publish(twist)

	#Moves robot until no longer aligned
	def follow_wall(self, ranges):

		self.move_forward()

		F = ranges[359:] +ranges[0:5]
		F = sum(F)/len(F)
		#If wall is detected in front
		if F < self.minthreshold:
			self.get_logger().info(f'WALL DETECTED')
			self.get_logger().info(f'is aligned: {self.aligned}')
			self.stop_turtle()
			self.aligned = False

	def align_to_wall(self,ranges):
		L = ranges[60:90]
		L = sum(L)/len(L)

		R = ranges[270:300]
		R = sum(R)/len(R)

		F = ranges[359:] +ranges[0:5]
		F = sum(F)/len(F)
		
		# for f in F:
		if F< self.minthreshold:

			while (L > self.maxthreshold) and (R > self.maxthreshold): #If left and right point are inf
				self.find_wall(0.2, 0.4)
				self.find_wall(0.5,0.0)

			if L > R:
				#If Left dist > Right dist, turtlebot knows more space that way, will turn left
				print("looping left")
				self.turn_turtle(0.0, 0.3)
				time.sleep(0.5)
				self.stop_turtle()

			elif R > L :
				#If Right dist > Left dist, turtlebot knows more space that way, will turn right
				print("looping right")
				self.turn_turtle(0.0, -0.3)
				time.sleep(0.5)
				self.stop_turtle()

			else:
				print("ERROR")

		else:
			self.aligned = True
			self.following_wall = True

		
def main(args=None):
		rclpy.init(args=args)
		turtle_pub = TurtlePub()
		rclpy.spin(turtle_pub)	
		turtle_pub.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()
