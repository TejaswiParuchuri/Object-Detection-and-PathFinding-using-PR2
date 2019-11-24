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
#group = moveit_commander.MoveGroupCommander(group_name)

standing_joints = [-0.00020377586883224552, \
					0.03213652755729335, \
					-0.006529160981967763, \
					-0.1942282176898953, \
					-0.0024028167806875445, \
					-0.20476869150849364, \
					0.0002324956593149352]

pick_joints = [0.26677748681820557, \
				0.7100922466133728, \
				-3.2354144833557843, \
				-0.45251067713982973, \
				1.4677722914652653, \
				-0.4585965724554333, \
				-0.9799488614156298]

pub = rospy.Publisher("r_gripper_controller/command", Pr2GripperCommand, queue_size=10)
while pub.get_num_connections() == 0:
	rospy.loginfo("Waiting for gripper publisher to connect")
	rospy.sleep(1)

headpub = rospy.Publisher("/head_traj_controller/point_head_action/goal", PointHeadActionGoal, queue_size=10)
while headpub.get_num_connections() == 0:
	rospy.loginfo("Waiting for head publisher to connect")
	rospy.sleep(1)

def reset_joints():
	print("Going to Standing Stance")

	print("Closing Gripper")
	data = Pr2GripperCommand()
	data.position = 0.0
	data.max_effort= 100.0
	pub.publish(data)


	joint_goal = standing_joints
	group.go(joint_goal, wait=True)

	# Calling ``stop()`` ensures that there is no residual movement
	group.stop()

	## END_SUB_TUTORIAL

	# For testing:
	# Note that since this section of code will not be included in the tutorials
	# we use the class variable rather than the copied state variable
	current_joints = group.get_current_joint_values()


def pick():
	print("Going to Picking Stance")

	print("Opening Gripper")
	data = Pr2GripperCommand()
	data.position = 0.1
	data.max_effort= 100.0
	pub.publish(data)

	joint_goal = pick_joints
	group.go(joint_goal, wait=True)

	# Calling ``stop()`` ensures that there is no residual movement
	group.stop()

	## END_SUB_TUTORIAL

	# For testing:
	# Note that since this section of code will not be included in the tutorials
	# we use the class variable rather than the copied state variable
	current_joints = group.get_current_joint_values()

def point_head_at_can():

	print("Pointing Head...")

	gms_func = rospy.ServiceProxy( '/gazebo/get_model_state', GetModelState)
	can_pose = gms_func("coke_can6","pr2::base_footprint").pose.position

	print("can_pose:")
	print(can_pose)

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

	headpub.publish(data)

# # reset_joints()
# pick()
# reset_joints()

point_head_at_can()