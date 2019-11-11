#! /usr/bin/python

import rospy
import math
import numpy as np
import time
import tf

from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry

class PID(object):

	def __init__(self, target_pose, mode):
		self.target_pose = target_pose
		self.mode = mode
		self.current_pose = Pose()
		self.init_pose = Pose()
		self.init_val_flag = 0
		self.vel = Twist()
		
		self.KP = 1.0
		self.KD = 1.5
		self.KP_rot = 5.0
		self.KD_rot = 100.0
		self.KP_rot_angular = 5.0
		self.KD_rot_angular = 100.0

		self.max_vel = 0.4
		self.max_rot = 3.0
		self.max_rot_angular = 0.3
		
		self.p_error_x = 0.0
		self.p_error_last_x = 0.0
		self.d_error_x = 0.0

		self.p_error_angular_z_linear = 0.0
		self.p_error_angular_z_linear_last = 0.0
		self.d_error_angular_z_linear  = 0.0
		
		self.p_error_angular_z_rot = 0.0
		self.p_error_angular_z_rot_last = 0.0
		self.d_error_angular_z_rot  = 0.0

		self.last_time = None

		rospy.Subscriber('/odom', Odometry, self.pose_callback, queue_size=1)
		self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		self.controller_status_publisher = rospy.Publisher('/Controller_Status', String, queue_size=1)

	def set_current_pose(self, current_pose):
		if self.init_val_flag == 0:
			self.init_pose = current_pose
			self.init_val_flag = 1
		else:
			current_rot_matrix = tf.transformations.quaternion_matrix([current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w])
			current_trans_matrix = tf.transformations.translation_matrix([current_pose.position.x, current_pose.position.y, current_pose.position.z])
			current_matrix = np.matmul(current_trans_matrix, current_rot_matrix)

			init_rot_matrix = tf.transformations.quaternion_matrix([self.init_pose.orientation.x, self.init_pose.orientation.y, self.init_pose.orientation.z, self.init_pose.orientation.w])
			init_trans_matrix = tf.transformations.translation_matrix([self.init_pose.position.x, self.init_pose.position.y, self.init_pose.position.z])
			init_matrix = np.matmul(init_trans_matrix, init_rot_matrix)

			current_pose_wrt_init = np.matmul(np.linalg.inv(init_matrix), current_matrix)
			current_quat = tf.transformations.quaternion_from_matrix(current_pose_wrt_init)

			self.current_pose.position.x = current_pose_wrt_init[0][3]
			self.current_pose.position.y = current_pose_wrt_init[1][3]
			self.current_pose.position.z = current_pose_wrt_init[2][3]
			self.current_pose.orientation.x = current_quat[0]
			self.current_pose.orientation.y = current_quat[1]
			self.current_pose.orientation.z = current_quat[2]
			self.current_pose.orientation.w = current_quat[3]

	def euler_from_pose(self, pose):
		quat = (pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quat)
		return euler

	def get_error(self):
		error_position_x = self.target_pose.position.x - self.current_pose.position.x
		error_position_y = self.target_pose.position.y - self.current_pose.position.y

		euler_target = self.euler_from_pose(self.target_pose)
		euler_current = self.euler_from_pose(self.current_pose)
		
		error_orientation_z = euler_target[2] - euler_current[2]

		return error_position_x, error_position_y, error_orientation_z

	def linear_error_conditions(self):
		error_dist_x,error_dist_y,_ = self.get_error()

		return error_dist_x


	def linear_vel(self, dt=None):
		if dt == None:
			cur_time = time.time()
			if self.last_time is None:
				self.last_time = cur_time
			dt = cur_time - self.last_time
			self.last_time = cur_time
		
		
		self.p_error_x = self.linear_error_conditions()
		
		if dt == 0.0:
			return self.KP * self.max_vel * self.p_error_x

		self.d_error_x = (self.p_error_last_x - self.p_error_x) / dt
		
		self.p_error_last_x = self.p_error_x
		
		return (self.KP * self.max_vel * self.p_error_x) + (self.KD * self.max_vel * self.d_error_x)

	def get_steering_angle(self):
		target_euler = self.euler_from_pose(self.target_pose)
		current_euler = self.euler_from_pose(self.current_pose)
		
		if target_euler[2]>3 and current_euler[2]<0:
			steer_orientation = -(target_euler[2] - abs(current_euler[2]))
		else:
			steer_orientation = target_euler[2] - current_euler[2]

		steer_positional_diff = math.atan2(self.target_pose.position.y - self.current_pose.position.y, self.target_pose.position.x - self.current_pose.position.x)

		if abs(abs(self.current_pose.position.x) - abs(self.init_pose.position.x))<0.09 or abs(abs(self.target_pose.position.x) - abs(self.current_pose.position.x))<0.09:
			return steer_orientation, 0.0, current_euler
		return steer_orientation, steer_positional_diff, current_euler
		

	def angular_vel(self, dt=None):
		if dt == None:
			cur_time = time.time()
			if self.last_time is None:
				self.last_time = cur_time
			dt = cur_time - self.last_time
			self.last_time = cur_time

		error_angular_z, differntial_error, current_euler = self.get_steering_angle()
		if dt == 0.0:
			if self.mode=='linear':
				self.p_error_angular_z_linear = error_angular_z + differntial_error
				self.p_error_angular_z_linear_last = self.p_error_angular_z_linear
				return (self.max_rot * self.KP_rot * self.p_error_angular_z_linear)
			elif self.mode=='rotational':
				self.p_error_angular_z_rot = error_angular_z
				self.p_error_angular_z_rot_last = self.p_error_angular_z_rot
				return (self.max_rot_angular * self.KP_rot_angular * self.p_error_angular_z_rot)
			
		
		
		if self.mode=='linear':
			self.p_error_angular_z_linear = error_angular_z + differntial_error
			self.d_error_angular_z_linear = (self.p_error_angular_z_linear - self.p_error_angular_z_linear_last) / dt
			self.p_error_angular_z_linear_last = self.p_error_angular_z_linear
			return ((self.max_rot *self.KP_rot * self.p_error_angular_z_linear) + (self.max_rot*self.KD_rot*self.d_error_angular_z_linear))
		
		elif self.mode=='rotational':
			self.p_error_angular_z_rot = error_angular_z
			self.d_error_angular_z_rot = (self.p_error_angular_z_rot - self.p_error_angular_z_rot_last) / dt
			self.p_error_angular_z_rot_last = self.p_error_angular_z_rot

			return (self.max_rot_angular * self.KP_rot_angular * self.p_error_angular_z_rot) + (self.KD_rot_angular * self.max_rot_angular * self.d_error_angular_z_rot)

		
			

		

	def publish_velocity(self):

		rospy.sleep(1)
		
		target_rot_matrix = tf.transformations.quaternion_matrix([self.target_pose.orientation.x, self.target_pose.orientation.y, self.target_pose.orientation.z, self.target_pose.orientation.w])
		target_trans_matrix = tf.transformations.translation_matrix([self.target_pose.position.x, self.target_pose.position.y, self.target_pose.position.z])
		target_matrix = np.matmul(target_trans_matrix, target_rot_matrix)

		current_rot_matrix = tf.transformations.quaternion_matrix([self.init_pose.orientation.x, self.init_pose.orientation.y, self.init_pose.orientation.z, self.init_pose.orientation.w])
		current_trans_matrix = tf.transformations.translation_matrix([self.init_pose.position.x, self.init_pose.position.y, self.init_pose.position.z])
		current_matrix = np.matmul(current_trans_matrix, current_rot_matrix)

		target_pose = np.matmul(np.linalg.inv(current_matrix), target_matrix)
		target_quat = tf.transformations.quaternion_from_matrix(target_pose)
		
		self.target_pose.position.x = target_pose[0][3]
		self.target_pose.position.y = target_pose[1][3]
		self.target_pose.position.z = target_pose[2][3]
		self.target_pose.orientation.x = target_quat[0]
		self.target_pose.orientation.y = target_quat[1]
		self.target_pose.orientation.z = target_quat[2]
		self.target_pose.orientation.w = target_quat[3]

		error_x = abs(abs(self.target_pose.position.x) - abs(self.current_pose.position.x))
		error_y = abs(abs(self.target_pose.position.y) - abs(self.current_pose.position.y))
		if self.target_pose.orientation.z==1.0:
			current_euler = self.euler_from_pose(self.current_pose)
			target_euler = self.euler_from_pose(self.target_pose)
			angular_error = abs(target_euler[2] - abs(current_euler[2]))
		else:
			current_euler = self.euler_from_pose(self.current_pose)
			target_euler = self.euler_from_pose(self.target_pose)
			angular_error = abs(target_euler[2] - current_euler[2])
		
		if self.mode == 'linear':
			while error_x>0.001:
				linear_velocity = self.linear_vel()
				self.vel.linear.x = linear_velocity
				self.vel.linear.y = 0.0
				self.vel.linear.z = 0.0
				self.vel.angular.x = 0.0
				self.vel.angular.y = 0.0
				self.vel.angular.z = self.angular_vel()

				self.velocity_publisher.publish(self.vel)
				error_x = abs(abs(self.target_pose.position.x) - abs(self.current_pose.position.x))
				error_y = abs(abs(self.target_pose.position.y) - abs(self.current_pose.position.y))
			
			self.vel.linear.x = 0.0
			self.vel.linear.y = 0.0
			self.vel.linear.z = 0.0
			self.vel.angular.x = 0.0
			self.vel.angular.y = 0.0
			self.vel.angular.z = 0.0

			self.velocity_publisher.publish(self.vel)
			
			self.mode = 'rotational'
			
			if self.target_pose.orientation.z==1.0:
				current_euler = self.euler_from_pose(self.current_pose)
				target_euler = self.euler_from_pose(self.target_pose)
				angular_error = abs(target_euler[2] - abs(current_euler[2]))
			else:
				current_euler = self.euler_from_pose(self.current_pose)
				target_euler = self.euler_from_pose(self.target_pose)
				angular_error = abs(target_euler[2] - current_euler[2])
			
			while angular_error>0.001:
				angular_velocity = self.angular_vel()
				self.vel.linear.x = 0.0
				self.vel.linear.y = 0.0
				self.vel.linear.z = 0.0
				self.vel.angular.x = 0.0
				self.vel.angular.y = 0.0
				self.vel.angular.z = angular_velocity

				self.velocity_publisher.publish(self.vel)
				if self.target_pose.orientation.z==1.0:
					current_euler = self.euler_from_pose(self.current_pose)
					target_euler = self.euler_from_pose(self.target_pose)
					angular_error = abs(target_euler[2] - abs(current_euler[2]))
				else:
					current_euler = self.euler_from_pose(self.current_pose)
					target_euler = self.euler_from_pose(self.target_pose)
					angular_error = abs(target_euler[2] - current_euler[2])

			self.mode= 'linear'

		

		elif self.mode == 'rotational':
			
			
			while angular_error>0.001:
				angular_velocity = self.angular_vel()
				self.vel.linear.x = 0.0
				self.vel.linear.y = 0.0
				self.vel.linear.z = 0.0
				self.vel.angular.x = 0.0
				self.vel.angular.y = 0.0
				self.vel.angular.z = angular_velocity

				self.velocity_publisher.publish(self.vel)
				if self.target_pose.orientation.z==1.0:
					current_euler = self.euler_from_pose(self.current_pose)
					target_euler = self.euler_from_pose(self.target_pose)
					angular_error = abs(target_euler[2] - abs(current_euler[2]))
				else:
					current_euler = self.euler_from_pose(self.current_pose)
					target_euler = self.euler_from_pose(self.target_pose)
					angular_error = abs(target_euler[2] - current_euler[2])
		
		self.vel.linear.x = 0.0
		self.vel.linear.y = 0.0
		self.vel.linear.z = 0.0
		self.vel.angular.x = 0.0
		self.vel.angular.y = 0.0
		self.vel.angular.z = 0.0

		self.velocity_publisher.publish(self.vel)

		self.controller_status_publisher.publish('Done')
		
	def pose_callback(self, msg):
		self.set_current_pose(msg.pose.pose)


if __name__ == "__main__":
	rospy.init_node('PID_Node')