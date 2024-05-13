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
		self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
		self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)		
		self.scan_sub
		self.seen_wall = False
		self.aligned = False
		self.following_wall = False
		self.threshold = 0.4
		#If not sim, set to false
		self.is_sim = True
		self.aligned_following = False
		self.damping = False
		self.following_correction = 0
		self.test_count = 0
		self.last_error = 0
		
	def scan_callback(self, msg):	
		if (not self.seen_wall):
			self.get_logger().info(f'moving to wall')
			self.move_to_wall(msg.ranges, self.is_sim)
		elif (self.seen_wall and not self.aligned):
			#time.sleep(1)
			self.get_logger().info(f'aligning')
			self.align_with_wall(msg.ranges)
			#time.sleep(1)
		#elif (self.seen_wall and self.aligned and self.following_wall):
			self.get_logger().info(f'following wall')
			#This is done to give the /scan topic a chance to populate with
			#most current readings
			#if self.test_count > 10:
				#self.follow_wall(msg.ranges)
			#else:
				#self.test_count += 1


	def move_turtle(self, dist=0.3):
		msg = Twist()

		msg.linear.x = dist
		self.publisher.publish(msg)

	def rotate_turtle(self, angle):
		msg = Twist()

		msg.angular.z = angle
		self.publisher.publish(msg)
		time.sleep(1.0)
	
	def move_and_rotate(self, x, z):
		msg = Twist()

		msg.linear.x = x
		msg.angular.z = z
		self.publisher.publish(msg)
		

	def stop_turtle(self):
		msg = Twist()

		msg.linear.x = 0.0
		msg.angular.z = 0.0
		self.publisher.publish(msg)
		self.get_logger().info('stopping turtle')
		time.sleep(0.5)


	def move_to_wall(self, ranges, simulation=True):
		if (simulation):
			dist = ranges[0]

			if dist <= self.threshold:
				self.seen_wall = True
				self.stop_turtle()
			else:
				self.move_turtle()
		else:
			dist = ranges[360]
			if dist <= self.threshold:
				self.seen_wall = True
				self.stop_turtle()
			else:
				self.move_turtle()		



	def compute_angular_correction(self, closest_index, simulation=True):
		if (not simulation):
			closest_angle = abs(closest_index / 2.0 - 180)
			if (closest_index > 360):
				angular_z = -1 * (90 - closest_angle)
			else:
				angular_z = -1 * (90 + closest_angle)
		else:

			if closest_index > 180:
				closest_angle = 360 - closest_index
			else:
				closest_angle = closest_index
				
			if (closest_index > 180):
				angular_z = -1 * (90 - closest_angle)
			else:
				angular_z = -1 * (90 - closest_angle)
		self.get_logger().info(f'closest angle {closest_angle}')
		self.get_logger().info(f'correction angle {angular_z}')

		return angular_z * math.pi / 180 



	def align_with_wall(self, ranges):


		if self.following_wall:
			self.get_logger().info(f'seen wall anf aligning with wall')

			#already following wall so we want to align with next wall
			#rotate 90 degrees
			# self.stop_turtle()
			self.rotate_turtle(-math.pi/2)
			time.sleep(1)
			self.stop_turtle()
			self.aligned = True	
			self.following_wall = True

		else:
			closest_index = ranges.index(min(ranges))
			angular_correction = self.compute_angular_correction(closest_index, self.is_sim)
	
			if abs(angular_correction * 180 / math.pi) < 2:
				self.following_wall = True
				self.aligned = True
				self.stop_turtle()	
			else:			
				self.rotate_turtle(angular_correction)	
				self.stop_turtle()	

	def follow_wall(self, ranges):

		if self.is_sim:
			front_ranges = ranges[358:] + ranges[0:2]
			left_side_ranges = ranges[88:92]
		else:
			left_side_ranges = ranges[538:542]
			front_ranges = ranges[358:362]

		front_avg = sum(front_ranges) / len(front_ranges)
		left_side_avg = sum(left_side_ranges) / len(left_side_ranges)
		


		#If wall is detected in front
		if(front_avg < self.threshold):
			self.get_logger().info(f'WALL DETECTED')
			self.get_logger().info(f'is aligned: {self.aligned}')

			self.stop_turtle()
			self.aligned = False
			return


		#Attempting to mimic PD Control
		Kp = 0.3  
		Kd = 0.1  

		distance_error = left_side_avg - self.threshold

		# proportional term
		proportional_correction = Kp * distance_error

		# deriv term
		error_change = distance_error - self.last_error

		derivative_correction = Kd * error_change

		angular_velocity = proportional_correction + derivative_correction

		# Store current error
		self.last_error = distance_error

		self.get_logger().info(f'correction: {angular_velocity}')
		self.get_logger().info(f'dist: {left_side_avg}')

		linear_velocity = 0.2
		self.following_correction = angular_velocity
		self.move_and_rotate(linear_velocity, angular_velocity)

		
def main(args=None):
		rclpy.init(args=args)
		turtle_pub = TurtlePub()
		rclpy.spin(turtle_pub)	
		turtle_pub.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()

		






