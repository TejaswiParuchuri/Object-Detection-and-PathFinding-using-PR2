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


from object_detection import ObjectClassifier, MoveRobotHead
import sys, time, os
import numpy as np
import pickle

from scipy.ndimage import filters
import cv2
import roslib
import rospy
import json
import random

# ROS Messages
import geometry_msgs.msg
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from sensor_msgs.msg import CompressedImage, Image

# Used in the ObjectDetection Class. Prints certain information about connections, if set to True.
VERBOSE = True

ROOT_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir))


if __name__ == '__main__':

	with open(ROOT_PATH+"/objects.json",'r') as json_file:
		try:
			objects = json.load(json_file)
		except (ValueError, KeyError, TypeError):
			print "JSON error"

	cans_cups=objects["cans"]
	cans_cups.update(objects["cups"])

	print("Moving PR2 to pick location...")
	pr2 = ModelState()
	pr2.model_name="pr2"
	
	choice=random.choice(cans_cups.values())
	

	pr2.pose.position.x = choice["load_loc"][0][0]
	pr2.pose.position.y = choice["load_loc"][0][1]
	rospy.wait_for_service("/gazebo/set_model_state")
	
	modelSetter = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
	modelSetter(pr2)
	
	robot_head = MoveRobotHead()
	robot_head.look_at(choice["loc"][0] - pr2.pose.position.x, choice["loc"][1] - pr2.pose.position.y, 0.185067)
	

	rospy.spin()
