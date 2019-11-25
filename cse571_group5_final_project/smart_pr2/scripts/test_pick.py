#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import PointStamped
from pr2_controllers_msgs.msg import PointHeadActionGoal
from pr2_controllers_msgs.msg import PointHeadGoal
from math import pi
from std_msgs.msg import String
from gazebo_msgs.srv import GetModelState
from pr2_controllers_msgs.msg import Pr2GripperCommand
from moveit_commander.conversions import pose_to_list


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python',
                anonymous=True)
robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

group_name = "right_arm"
group = moveit_commander.MoveGroupCommander(group_name)

standing_joints = [-0.00020377586883224552, \
					0.03213652755729335, \
					-0.006529160981967763, \
					-0.1942282176898953, \
					-0.0024028167806875445, \
					-0.20476869150849364, \
					0.0002324956593149352]

gripper_pub = rospy.Publisher("r_gripper_controller/command", Pr2GripperCommand, queue_size=10)
while gripper_pub.get_num_connections() == 0:
	rospy.loginfo("Waiting for gripper publisher to connect")
	rospy.sleep(1)

def gripper(cmd):
	if(cmd == "open"):
		print("Opening Gripper")
		data = Pr2GripperCommand()
		data.position = 0.1
		data.max_effort= 100.0
		gripper_pub.publish(data)
		rospy.sleep(3)
	elif(cmd == "close"):
		print("Closing Gripper")
		data = Pr2GripperCommand()
		data.position = 0.0
		data.max_effort= 100.0
		gripper_pub.publish(data)
		rospy.sleep(3)

def reset_joints():
	print("Going to Standing Stance")


	joint_goal = standing_joints
	group.go(joint_goal, wait=True)
	group.stop()

	rospy.sleep(3)

	current_joints = group.get_current_joint_values()
	return current_joints


def pick():

	print("Performing Pick Operation...")
	print("Step 1: Opening Gripper")
	gripper("open")

	print("Step 2: Moving To Approach")
	approach_joints = [-0.3039317534057506,\
						0.854805322462882,\
						-1.5700910376892132,\
						-0.16689570952151023,\
						-0.14461307158717318,\
						-1.3447341945775477,\
						0.7118105483210035]

	joint_goal = approach_joints
	group.go(joint_goal, wait=True)
	group.stop()
	rospy.sleep(3)

	print("Step 3: Completing Grasp")
	grasp_joints = [-0.03260850653005498,\
	0.8679200984865636,\
	-1.5700975812614626,\
	-0.18182010402644977,\
	-0.017845654757860707,\
	-1.0650192634362217,\
	0.7383857717301172]

	joint_goal = grasp_joints
	group.go(joint_goal, wait=True)
	group.stop()
	rospy.sleep(3)

	print("Step 4: Closing Gripper")
	gripper("close")

	print("Step 5: Returning to Standing")
	reset_joints()

def print_joints():
	current_joints = group.get_current_joint_values()
	print("Current Joints: {}".format(current_joints))

pick()