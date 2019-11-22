#!/usr/bin/env python

"""
To get the image from PR2's camera, we first subscribe to a ros topic containing sensor_msgs
CompressedImage. For PR2, this topic is /wide_stereo/right/image_color/compressed.
The message type for this topic is sensor_msgs/CompressedImage.
"""

"""
CNN Classifier Code Availability: 
https://becominghuman.ai/building-an-image-classifier-using-deep-learning-in-python-totally-from-a-beginners-perspective-be8dbaf22dd8
"""

import sys, time, os
import numpy as np
import pickle

from scipy.ndimage import filters
import cv2
import roslib
import rospy

import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list

from gazebo_msgs.srv import GetModelState
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PointStamped
from pr2_controllers_msgs.msg import PointHeadActionGoal
from pr2_controllers_msgs.msg import PointHeadGoal

from keras.models import Sequential
from keras.layers import Conv2D
from keras.layers import MaxPooling2D
from keras.layers import Flatten
from keras.layers import Dense
from keras.preprocessing import image
from keras.preprocessing.image import ImageDataGenerator


VERBOSE = True


class RobotHead:

	"""
	Used to point the robot's head in the direction of the object to be picked up.
	"""

	def __init__(self):

		# Initialize the publisher to the required topic
		self.headpub = rospy.Publisher("/head_traj_controller/point_head_action/goal", PointHeadActionGoal, queue_size=10)


	def look_at(self, frame_id, x, y, z):
		
		"""
		Here, the PR2 head will be rotated towards the object that is to be picked up.
		This is done to get the desired image of the object, which will then try to classify
		the object.
		"""

		gms_func = rospy.ServiceProxy( '/gazebo/get_model_state', GetModelState)
		can_pose = gms_func("coke_can2","base_link").pose.position

		data = PointHeadActionGoal()
		point = PointStamped()

		point.header.frame_id = "base_link"
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

		print ('Moving head')

		

class ObjectClassifier:

	"""
	Identifies the object scanned by the camera. Initiates pick action if the identified object
	is a coke_can.
	"""

	def __init__(self):

		self.root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir))

		
		# Initialize rostopic: /wide_stereo/right/image_color/compressed, where we will publish the results
		self.image_pub = rospy.Publisher('/wide_stereo/right/image/compressed', CompressedImage, queue_size=1)

		# Intialize subscriber 
		self.subscriber = rospy.Subscriber('/wide_stereo/right/image/compressed', CompressedImage, self.callback, queue_size=1)

		if VERBOSE:
			print 'Subscribed to PR2 /wide_stereo/right/image/compressed topic'
		

	def train_classifier(self):

		"""
		Train a Convolutional Neural Network to classify the types of object present
		in the environment. This classifier is able to classify between a coke can
		and a plastic cup.

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
	                                       zoom_range = 0.2,
	                                       horizontal_flip = True)

		test_datagen = ImageDataGenerator(rescale = 1./255)

		training_set = train_datagen.flow_from_directory(self.root_path + '/pr2_image_data/training_set',
	                                target_size = (64, 64),
	                                batch_size = 16,
	                                class_mode = 'binary')
		test_set = test_datagen.flow_from_directory(self.root_path + '/pr2_image_data/test_set',
	                                target_size = (64, 64),
	                                batch_size = 1,
	                                class_mode = 'binary')


		print 'Training set gathered...\n'

		classifier.fit_generator(training_set,
	                             steps_per_epoch = 64,
	                             epochs = 5,
	                             validation_data = test_set)


		with open(self.root_path + '/ObjectClassifier.pkl', 'wb') as file:
			pickle.dump(classifier, file)
			print ('Saved object classifier.')
	    


	def predict_class(self, sensor_image):

		"""
		Predict the type of object using the trained classifier.
		:param: image: path to image
		:returns: int prediction
		"""

		with open(self.root_path + '/ObjectClassifier.pkl', 'rb') as file:
			classifier = pickle.load(file)

		print ('Trying to predict...\n')

		# Making new predictions based off the trained CNN
		camera_image = image.load_img(self.root_path + '/pr2_image_data/test_set/plastic_cup/plastic_cup_4.png', target_size=(64, 64))
		camera_image = image.img_to_array(camera_image)
		camera_image = np.expand_dims(camera_image, axis=0)
		prediction = classifier.predict(camera_image)

		if prediction[0] == 0:
			print 'Detected coke can.'
		else:
			print 'Detected a plastic cup.'

		return prediction[0]


	def callback(self, ros_data):

		'''
		Callback function for the subscribed topic.
		'''
		print ('Callback function...')

		if VERBOSE:
			print 'Received image of type: "%s"' % ros_data.format

		# Convert compressed image to cv2 image
		np_arr = np.fromstring(ros_data.data, np.uint8)
		image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)


		# Perform the image classification part here


		# Publish the result to arm_joint topic to indicate whether or not to pick up this object


if __name__=='__main__':

	#obj_classifier = ObjectClassifier()
	#rospy.init_node('ObjectClassifier', anonymous=True)


	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('move_group_python',
                anonymous=True)
	robot = moveit_commander.RobotCommander()

	scene = moveit_commander.PlanningSceneInterface()

	obj = ObjectClassifier()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print 'Keyboard Interrupt'