#! /usr/bin/python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import tf
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Quaternion
from pid import PID
import copy

class moveTbot3:
	def __init__(self):
		rospy.init_node('move_turtle',anonymous = True)
		self.actions = String()
		self.pose = Pose()
		self.vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
		self.action_subscriber = rospy.Subscriber('/actions',String,self.callback_actions)
		self.pid_subscriber = rospy.Subscriber("/Controller_Status",String,self.callback_pid)
		self.pose_subscriber = rospy.Subscriber('/odom',Odometry,self.pose_callback)
		self.status_publisher = rospy.Publisher("/status",String,queue_size = 10)
		self.free = String(data = "Idle")
		self.rate = rospy.Rate(30)
		print "Ready!"
		rospy.spin()

	def callback_pid(self,data):
		if data.data == "Done":
			if len(self.actions)>0:
				self.execute_next()

	def callback_actions(self,data):
		self.actions = data.data.split("_")
		self.rate.sleep()
		self.execute_next()
		# self.move()

	def execute_next(self):
		action = self.actions.pop(0)
		direction = None
		if action == "MoveF" or action == "MoveB":
			current_pose = self.pose
			quat = (current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w)
			euler = tf.transformations.euler_from_quaternion(quat)
			current_yaw = euler[2]
			if current_yaw > (-math.pi /4.0) and current_yaw < (math.pi / 4.0):
				print "Case 1"
				#raw_input()
				target_pose = copy.deepcopy(current_pose)
				target_pose.position.x += 0.5
				#direction = 'x'
				#incr y co-ordinate
			elif current_yaw > (math.pi / 4.0 ) and current_yaw < (3.0 * math.pi / 4.0):
				print "Case 2"
				#raw_input()
				target_pose = copy.deepcopy(current_pose)
				target_pose.position.y += 0.5
				#direction = 'y'
				#decr x co
			elif current_yaw > (-3.0*math.pi /4.0) and current_yaw < (-math.pi /4.0):
				print "Case 3"
				#raw_input()
				target_pose = copy.deepcopy(current_pose)
				target_pose.position.y -= 0.5
				#direction = '-y'
			else:
				print "Case 4"
				#raw_input()
				target_pose = copy.deepcopy(current_pose)
				target_pose.position.x -= 0.5
				#direction = '-x'
			PID(target_pose,"linear").publish_velocity()
			
		elif action == "TurnCW" or action == "TurnCCW":
			current_pose = self.pose
			quat = (current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w)
			euler = tf.transformations.euler_from_quaternion(quat)
			yaw = euler[2]
			if action == "TurnCW":
				target_yaw = yaw - ( math.pi / 2.0)
				if target_yaw < -math.pi:
					target_yaw += (math.pi * 2)
			else:
				target_yaw = yaw + ( math.pi / 2.0)
				if target_yaw >= (math.pi ):
					target_yaw -= (math.pi * 2 )
			target_pose = Pose()
			target_pose.position = current_pose.position
			target_quat = Quaternion(*tf.transformations.quaternion_from_euler(euler[0],euler[1],target_yaw))
			target_pose.orientation = target_quat
			print target_pose.orientation
			PID(target_pose,"rotational").publish_velocity()

		else:
			print "Invalid action"
			exit(-1)
		if len(self.actions) == 0:
			self.status_publisher.publish(self.free)


	def pose_callback(self,data):
		self.pose = data.pose.pose

	

if __name__ == "__main__":
	try:
		moveTbot3()
	except rospy.ROSInterruptException:
		pass
