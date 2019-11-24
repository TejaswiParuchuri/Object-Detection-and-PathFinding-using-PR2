#!/usr/bin/env python

"""

To get the image from PR2's camera, we first subscribe to a ros topic containing sensor_msgs/Image. 
:topic: /wide_stereo/right/image_raw
:message type: sensor_msgs/Image

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

import geometry_msgs.msg
from gazebo_msgs.srv import GetModelState
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import PointStamped
from pr2_controllers_msgs.msg import PointHeadActionGoal
from pr2_controllers_msgs.msg import PointHeadGoal

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


class RobotHead:

	"""
	Used to point the robot's head in the direction of the object to be picked up.
	"""

	def __init__(self):
		pass
		# Initialize the publisher to the required topic
		

	def look_at(self, frame_id, x, y, z):
		
		"""
		This method will publish coke_location coordinates to PR2's head. This will rotate 
		PR2's head to point to those coordinates in the environment.
		
		:param: frame_id: str
		:param: x: float
		:param: y: float
		:param: z: float

		"""

		rospy.init_node('RobotHead', anonymous = True)
		
		headpub = rospy.Publisher("/head_traj_controller/point_head_action/goal", PointHeadActionGoal, queue_size=10)

		while headpub.get_num_connections() == 0:
			rospy.loginfo("Waiting for head publisher to connect")
			rospy.sleep(1)
		
		gms_func = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
		can_pose = gms_func("coke_can1","pr2::base_footprint").pose.position
	
		print (can_pose)

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
		headpub.publish(data)
		
		'''
		data = PointHeadActionGoal()
		data.header.frame_id = "pr2::base_link"
		goal = PointHeadGoal()
		goal.point.x = 1.0
		goal.point.y = 2.0
		goal.point.z = 3.0
		'''

		print ('Moving head')



class ObjectClassifier:

	"""

	Identifies the object scanned by the camera. Initiates pick action if the identified object
	is a coke_can.
	
	"""

	def __init__(self):

		rospy.init_node('ObjectClassifier', anonymous=True)
		

		self.publisher = None 
		self.subscriber = rospy.Subscriber('/wide_stereo/right/image_raw', Image, self.callback)		

		if VERBOSE:
			print 'Subscribed to PR2 camera topic'
		

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
	                                       zoom_range = 0.2,
	                                       horizontal_flip = True)

		test_datagen = ImageDataGenerator(rescale = 1./255)

		training_set = train_datagen.flow_from_directory(ROOT_PATH + '/pr2_image_data/training_set',
	                                target_size = (64, 64),
	                                batch_size = 16,
	                                class_mode = 'binary')
		test_set = test_datagen.flow_from_directory(ROOT_PATH + '/pr2_image_data/test_set',
	                                target_size = (64, 64),
	                                batch_size = 1,
	                                class_mode = 'binary')


		print 'Training set gathered...\n'

		classifier.fit_generator(training_set,
	                             steps_per_epoch = 64,
	                             epochs = 5,
	                             validation_data = test_set)


		with open(ROOT_PATH + '/ObjectClassifier.pkl', 'wb') as file:
			pickle.dump(classifier, file)
			print ('Saved object classifier.')
	    


	def load_classifier(self):

		with open(ROOT_PATH + '/ObjectClassifier.pkl', 'rb') as file:
			classifier = pickle.load(file)

		return classifier



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
			image_path = ROOT_PATH + '/sensor_image.png'
			cv2.imwrite(image_path, cv2_img)

		except CvBridgeError as e:
			print (e)

		prediction = self.predict_class(image_path)
		self.subscriber.unregister()

		# Trigger the pick action if a coke_can if found
		if prediction == 0:
			# self.publisher.publish()
			pass
		else:
			pass



if __name__=='__main__':

	#obj = ObjectClassifier()

	robot_head = RobotHead()
	robot_head.look_at('base_link', -10.0, 10.0, 5.0)

	try:
		rospy.spin()

	except KeyboardInterrupt:
		print 'Keyboard Interrupt'