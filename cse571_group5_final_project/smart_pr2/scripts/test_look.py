#!/usr/bin/env python

import sys
import copy
import rospy
import geometry_msgs.msg
from geometry_msgs.msg import PointStamped
from pr2_controllers_msgs.msg import PointHeadActionGoal
from pr2_controllers_msgs.msg import PointHeadGoal
from math import pi
from std_msgs.msg import String
from gazebo_msgs.srv import GetModelState

rospy.init_node('move_group_python',
                anonymous=True)


headpub = rospy.Publisher("/head_traj_controller/point_head_action/goal", PointHeadActionGoal, queue_size=10)
while headpub.get_num_connections() == 0:
	rospy.loginfo("Waiting for head publisher to connect")
	rospy.sleep(1)

def point_head_at_can():
	print("Pointing Head...")

	gms_func = rospy.ServiceProxy( '/gazebo/get_model_state', GetModelState)
	can_pose = gms_func("coke_can1","pr2::base_footprint").pose.position

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

point_head_at_can()