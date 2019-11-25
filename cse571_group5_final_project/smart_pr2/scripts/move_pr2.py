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
from pid_fp import PID
import copy

import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import PointStamped
from math import pi
from gazebo_msgs.srv import GetModelState
from pr2_controllers_msgs.msg import Pr2GripperCommand
from moveit_commander.conversions import pose_to_list

import time, os
import numpy as np
import pickle

from scipy.ndimage import filters
import roslib

# ROS Messages
from sensor_msgs.msg import CompressedImage, Image
from pr2_controllers_msgs.msg import PointHeadActionGoal, PointHeadGoal, PointHeadAction, PointHeadActionResult 
from actionlib_msgs.msg import GoalStatusArray


# Required to change the format of the sensor image
from cv_bridge import CvBridge, CvBridgeError
import cv2

# Required to train a CNN image classifier.
from keras.models import Sequential
from keras.layers import Conv2D
from keras.layers import MaxPooling2D
from keras.layers import Flatten
from keras.layers import Dense
from keras.preprocessing.image import ImageDataGenerator
from keras.preprocessing import image

# ### fix for value = True issue
# import keras.backend.tensorflow_backend as tb
# tb._SYMBOLIC_SCOPE.value = True

ROOT_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir))
VERBOSE = True

class movePR2:
	def __init__(self):
		rospy.init_node('move_pr2',anonymous = True)
		self.actions = String()
		self.pose = Pose()
		self.vel_pub = rospy.Publisher('/base_controller/command',Twist,queue_size=10)
		self.action_subscriber = rospy.Subscriber('/actions',String,self.callback_actions)
		self.pid_subscriber = rospy.Subscriber("/Controller_Status",String,self.callback_pid)
		self.pose_subscriber = rospy.Subscriber('/base_odometry/odom',Odometry,self.pose_callback)
		self.status_publisher = rospy.Publisher("/status",String,queue_size = 10)
		self.sense_publisher = rospy.Publisher("/sense_report", String, queue_size = 10)
		self.free = String(data = "Idle")
		self.rate = rospy.Rate(30)


		moveit_commander.roscpp_initialize(sys.argv)
		self.robot = moveit_commander.RobotCommander()

		self.scene = moveit_commander.PlanningSceneInterface()

		self.group_name = "right_arm"
		self.group = moveit_commander.MoveGroupCommander(self.group_name)

		self.standing_joints = [-0.00020377586883224552, \
							0.03213652755729335, \
							-0.006529160981967763, \
							-0.1942282176898953, \
							-0.0024028167806875445, \
							-0.20476869150849364, \
							0.0002324956593149352]

		self.gripper_pub = rospy.Publisher("r_gripper_controller/command", Pr2GripperCommand, queue_size=10)
		while self.gripper_pub.get_num_connections() == 0:
			rospy.loginfo("Waiting for gripper publisher to connect")
			rospy.sleep(1)


		self.headpub = rospy.Publisher("/head_traj_controller/point_head_action/goal", PointHeadActionGoal, queue_size = 10)
		
		while self.headpub.get_num_connections() == 0:
			rospy.loginfo("Waiting for head publisher to connect")
			rospy.sleep(1)

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

		elif action == "pick":
			self.pick()
		elif action == "place":
			self.place()
		elif "sense" in action:
			obj_name = action.split("|")[1]
			obj_name = obj_name.replace("@","_")
			print("Object: " + obj_name)
			self.sense(obj_name)

		else:
			print "Invalid action"
			exit(-1)
		if len(self.actions) == 0:
			self.status_publisher.publish(self.free)



	def place(self):
		print("Performing Place Operation")
		print("Step 1: Opening Gripper!")
		self.gripper("open")
		print("Step 2: Closing Gripper!")
		self.gripper("close")

	def pick(self):
		print("Performing Pick Operation...")
		print("Step 1: Opening Gripper")
		self.gripper("open")

		print("Step 2: Moving To Approach")
		approach_joints = [-0.3039317534057506,\
							0.854805322462882,\
							-1.5700910376892132,\
							-0.16689570952151023,\
							-0.14461307158717318,\
							-1.3447341945775477,\
							0.7118105483210035]

		joint_goal = approach_joints
		self.group.go(joint_goal, wait=True)
		self.group.stop()
		rospy.sleep(1)

		print("Step 3: Completing Grasp")
		grasp_joints = [-0.03260850653005498,\
		0.8679200984865636,\
		-1.5700975812614626,\
		-0.18182010402644977,\
		-0.017845654757860707,\
		-1.0650192634362217,\
		0.7383857717301172]

		joint_goal = grasp_joints
		self.group.go(joint_goal, wait=True)
		self.group.stop()
		rospy.sleep(1)

		print("Step 4: Closing Gripper")
		self.gripper("close")

		print("Step 5: Returning to Standing")
		self.reset_joints()

	def gripper(self,cmd):
		if(cmd == "open"):
			print("Opening Gripper")
			data = Pr2GripperCommand()
			data.position = 0.1
			data.max_effort= 100.0
			self.gripper_pub.publish(data)
			rospy.sleep(1)
		elif(cmd == "close"):
			print("Closing Gripper")
			data = Pr2GripperCommand()
			data.position = 0.0
			data.max_effort= 100.0
			self.gripper_pub.publish(data)
			rospy.sleep(1)

	def reset_joints(self):
		print("Going to Standing Stance")


		joint_goal = self.standing_joints
		self.group.go(joint_goal, wait=True)
		self.group.stop()

		rospy.sleep(1)

		current_joints = self.group.get_current_joint_values()
		return current_joints

	def sense(self, obj_name):
		self.look_at(obj_name)

	def pose_callback(self,data):
		self.pose = data.pose.pose
	

	def look_at(self, obj_name, frame_id='base_link'):
		
		"""
		This method will publish coke_location coordinates to PR2's head. This will rotate 
		PR2's head to point to those coordinates in the environment.
		
		:param: frame_id: str
		:param: x: float
		:param: y: float
		:param: z: float

		"""

		print("Trying to look at {}".format(obj_name))
		gms_func = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
		can_pose = gms_func(obj_name, "pr2::base_footprint").pose.position

		data = PointHeadActionGoal()
		point = PointStamped()

		point.header.frame_id = frame_id
		point.point.x = can_pose.x
		point.point.y = can_pose.y
		point.point.z = can_pose.z

		data.goal = PointHeadGoal()
		data.goal.target = point

		data.goal.pointing_frame = "high_def_frame"
		data.goal.pointing_axis.x = 1
		data.goal.pointing_axis.y = 0
		data.goal.pointing_axis.z = 0

		# Publish goal to PR2's head
		self.headpub.publish(data)
	
		print ('Moving head to coordinates: ')
		print (can_pose)


		# The callback method for this subscriber will only be activated when the head is pointing to the goal.
		self.headsub = rospy.Subscriber("/head_traj_controller/point_head_action/result", PointHeadActionResult, self.callback_sense)

	def callback_sense(self, status):

		print ('Callback action')

		point = PointHeadActionResult()

		
		if point.status.status == 0:
			self.headsub.unregister()
			time.sleep(2)

			classifier = ObjectClassifier()
			classifier.sense()


			return 

class ObjectClassifier:

	"""

	Identifies the object scanned by the camera. Initiates pick action if the identified object
	is a coke_can.
	
	"""

	def __init__(self):
		
		self.prediction = 2
		self.sense_publisher = rospy.Publisher("/sense_report", String, queue_size = 10)
	

	def train_classifier(self):

		"""

		Train a Convolutional Neural Network to classify the type of object present
		in the image received from camera sensor. This classifier is able to classify 
		between a coke can and a plastic cup.

		"""

		classifier = Sequential()

		classifier.add(Conv2D(32, (3,3), input_shape = (64, 64, 3), activation='relu'))
		classifier.add(MaxPooling2D(pool_size=(2, 2)))
		classifier.add(Flatten())
		classifier.add(Dense(units=128, activation = 'relu'))
		classifier.add(Dense(units=1, activation = 'sigmoid'))
		
		classifier.compile(optimizer = 'adam', loss = 'binary_crossentropy', metrics = ['accuracy'])

		train_datagen = ImageDataGenerator(rescale = 1./255,
	                                       shear_range = 0.2,
	                                       zoom_range = 0.5,
	                                       horizontal_flip = True)

		test_datagen = ImageDataGenerator(rescale = 1./255,
										  shear_range = 0.2,
										  zoom_range = 0.5,
										  horizontal_flip = True)

		training_set = train_datagen.flow_from_directory(ROOT_PATH + '/pr2_image_data/training_set',
	                                target_size = (64, 64),
	                                batch_size = 16,
	                                class_mode = 'binary')
		
		test_set = test_datagen.flow_from_directory(ROOT_PATH + '/pr2_image_data/test_set',
	                                target_size = (64, 64),
	                                batch_size = 2,
	                                class_mode = 'binary')


		print 'Training set gathered...\n'

		classifier.fit_generator(training_set,
	                             steps_per_epoch = 193,
	                             epochs = 20,
	                             validation_data = test_set)


		with open(ROOT_PATH + '/ObjectClassifier.pkl', 'wb') as file:
			pickle.dump(classifier, file)
			print ('Saved object classifier.')
	    


	def load_classifier(self):


		print(ROOT_PATH + '/ObjectClassifier.pkl')
		with open(ROOT_PATH + '/ObjectClassifier.pkl', 'rb') as file:
			classifier = pickle.load(file)

		return classifier



	def sense(self):

		'''
		Subscribe to the camera and prepare to capture an image.

		:return: 0 if coke_can, 1 otherwise
		'''	

		self.subscriber = rospy.Subscriber('/wide_stereo/right/image_raw', Image, self.callback)		

		if VERBOSE:
			print 'Subscribed to PR2 camera topic'
			

		
	def predict_class(self, sensor_image):

		"""

		Predict the type of object using the trained classifier.
		:param: image: Image
		:returns: float prediction

		"""

		classifier = self.load_classifier()
		
		print ('Trying to predict...\n')

		# Making new predictions based off the trained CNN
		camera_image = image.load_img(sensor_image, target_size=(64, 64))
		camera_image = image.img_to_array(camera_image)
		camera_image = np.expand_dims(camera_image, axis=-0)
		prediction = classifier.predict(camera_image)

		return prediction[0][0]



	def callback(self, ros_data):

		'''
		Callback function for the subscribed topic. This function predicts the type of 
		object found in the image and then publishes it to the arm_joint topic.
		'''

		bridge = CvBridge()
		
		try:
			cv2_img = bridge.imgmsg_to_cv2(ros_data, "bgr8")
			#cv2_img = cv2.resize(cv2_img, (600, 400), interpolation = cv2.INTER_AREA)
			image_path = ROOT_PATH + '/sensor_image.png'
			cv2.imwrite(image_path, cv2_img)

		except CvBridgeError as e:
			print (e)

		self.prediction = self.predict_class(image_path)
		self.subscriber.unregister()
		self.sense_publisher.publish(String(data="prediction|{}".format(self.prediction)))




if __name__ == "__main__":
	try:
		movePR2()
	except rospy.ROSInterruptException:
		pass
