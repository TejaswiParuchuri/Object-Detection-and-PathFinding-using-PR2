#!/usr/bin/env python

"""

To get the image from PR2's camera, we first subscribe to a ros topic containing sensor_msgs/Image. 
:topic: /wide_stereo/right/image_raw
:message: sensor_msgs/Image

"""

"""
CNN Classifier implementation reference: 
https://becominghuman.ai/building-an-image-classifier-using-deep-learning-in-python-totally-from-a-beginners-perspective-be8dbaf22dd8
"""


import sys, time, os
import numpy as np
import pickle

from scipy.ndimage import filters
import cv2
import roslib
import rospy

# ROS Messages
import geometry_msgs.msg
from gazebo_msgs.srv import GetModelState
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import PointStamped
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


# Used in the ObjectDetection Class. Prints certain information about connections, if set to True.
VERBOSE = True

ROOT_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir))


class MoveRobotHead:

	"""
	Used to point the robot's head in the direction of the object to be picked up.
	"""

	def __init__(self):
		
		
	        rospy.init_node('MoveRobotHead', anonymous = True)

		self.headpub = rospy.Publisher("/head_traj_controller/point_head_action/goal", PointHeadActionGoal, queue_size = 10)
		
		while self.headpub.get_num_connections() == 0:
		 	rospy.loginfo("Waiting for head publisher to connect")
		 	rospy.sleep(1)

		

	def look_at(self, x, y, z, frame_id='base_link'):
		
		"""
		This method will publish coke_location coordinates to PR2's head. This will rotate 
		PR2's head to point to those coordinates in the environment.
		
		:param: frame_id: str
		:param: x: float
		:param: y: float
		:param: z: float

		"""

		print("Starting head rotation")

		data = PointHeadActionGoal()
		point = PointStamped()

		point.header.frame_id = frame_id
		point.point.x = x
		point.point.y = y
		point.point.z = z

		data.goal = PointHeadGoal()
		data.goal.target = point

		data.goal.pointing_frame = "high_def_frame"
		data.goal.pointing_axis.x = 1
		data.goal.pointing_axis.y = 0
		data.goal.pointing_axis.z = 0

		# Publish goal to PR2's head
		self.headpub.publish(data)
	

		# The callback method for this subscriber will only be activated when the head is pointing to the goal.
		self.headsub = rospy.Subscriber("/head_traj_controller/point_head_action/result", PointHeadActionResult, self.callback)

		return 1


	def callback(self, status):

		print ('Callback action')

		point = PointHeadActionResult()
		
		if point.status.status == 0:
			self.headsub.unregister()
			rospy.sleep(2)
			obj = ObjectClassifier()
			obj.sense() 


class ObjectClassifier:

	"""

	Identifies the object scanned by the camera. Initiates pick action if the identified object
	is a coke_can.
	
	"""

	def __init__(self):
		
		print("Starting the camera sensor")	


	def load_classifier(self):

		print (ROOT_PATH)

		with open(ROOT_PATH + '/ObjectClassifier.pkl', 'rb') as file:
			classifier = pickle.load(file)

		return classifier



	def sense(self):

		'''
		Subscribe to the camera and prepare to capture an image.

		:return: 0 if coke_can, 1 otherwise
		'''	

		self.subscriber = rospy.Subscriber('/wide_stereo/right/image_raw', Image, self.callback, queue_size=1)		

		if VERBOSE:
			print 'Subscribed to PR2\'s head camera.'
			

		
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
		camera_image = np.expand_dims(camera_image, axis=0)
		prediction = classifier.predict(camera_image)

		return prediction[0][0]



	def callback(self, ros_data):

		'''
		Callback function for the subscribed topic. This function predicts the type of 
		object found in the image and then publishes it to the arm_joint topic.
		'''
	
		bridge = CvBridge()
		
		try:
			print ('Trying to fetch image from the sensor.')
			cv2_img = bridge.imgmsg_to_cv2(ros_data, "bgr8")
			image_path = ROOT_PATH + '/sensor_image.png'
			rospy.sleep(3)
			cv2.imwrite(image_path, cv2_img)

		except CvBridgeError as e:
			print (e)

		self.prediction = self.predict_class(image_path)
		
		if self.prediction == 0:
			print ('Saw a coke can')
		else:
			print ('Saw a plastic cup')

		self.subscriber.unregister()

	
