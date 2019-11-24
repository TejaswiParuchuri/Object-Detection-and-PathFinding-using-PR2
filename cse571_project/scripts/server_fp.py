#!/usr/bin/env python

from cse571_project.srv import *
import rospy
from mazeGenerator_fp import *
import sys
import argparse
import time
from action_server_fp import RobotActionsServer 
import pickle
import copy
import os

root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir))
books = None
mazeInfo = None
mazeInfoCopy = None
parser = argparse.ArgumentParser()
parser.add_argument('-cans', help='for providing no. of subjects', metavar='5', action='store', dest='n_cans', default=8, type=int)
parser.add_argument('-cups', help='for providing no. of books for each subject', metavar='5', action='store', dest='n_cups', default=8, type=int)
parser.add_argument('-s', help='for providing random seed', metavar='32', action='store', dest='seed', default=int(time.time()), type=int)
parser.add_argument('-action_seed', help='for providing action selection random seed', metavar='32', action='store', dest='action_seed', default=int(time.time()), type=int)
robot_action_server = None


def check_is_edge(req):
	"""
	This function checks if two points are connected via edge or not.
	"""
	global mazeInfo
	edge = (req.x1*1.0,req.y1*1.0,req.x2*1.0,req.y2*1.0)
	for edge_point in edge:
		if edge_point < -mazeInfo.grid_dimension * 0.5 or edge_point > mazeInfo.grid_dimension * 0.5:
			return 0
	if edge in mazeInfo.blocked_edges or (edge[2],edge[3],edge[0],edge[1]) in mazeInfo.blocked_edges:
		return 0
	else:
		return 1


def handle_reset_world(req):
	global mazeInfo
	mazeInfo = copy.deepcopy(mazeInfoCopy)
	robot_action_server.current_state = robot_action_server.generate_init_state()
	return 1
def handle_get_successor(req):
    """
        This function returns successor of a given state. 
                
        parameters:	x_cord - current x-cordinate of turtlebot           output:   x_cord - new x-cordinate of turtlebot
                    y_cord - current y-cordinate of turtlebot					  y_cord - new y-cordinate of turtlebot
                    direction - current orientation								  direction - new orientation
                    action - current action										  g_cost - Manhatan distance from initial state to new state
                                                                                    hurestic_value - Manhatan distance from goal state to new state
    """
    global mazeInfo
    directionList = ["NORTH", "EAST","SOUTH","WEST"]
    x_cord, y_cord, direction, action = req.x, req.y, req.direction, req.action
    #print "I am here"
    #Checking requested action and making changes in states
    if action == 'TurnCW':
        index = directionList.index(req.direction)
        direction = directionList[(index+1)%4]
        g_cost = 1

    elif action == 'TurnCCW':
        index = directionList.index(req.direction)
        direction = directionList[(index-1)%4]
        g_cost = 1

    elif action == 'moveF':
        if direction == "NORTH":
            y_cord += 0.5
        elif direction == "EAST":
            x_cord += 0.5
        elif direction == "SOUTH":
            y_cord -= 0.5
        elif direction == "WEST":
            x_cord -= 0.5
        g_cost = 1

    elif action == 'MoveB':
        if direction == "NORTH":
            y_cord -= 0.5
        elif direction == "EAST":
            x_cord -= 0.5
        elif direction == "SOUTH":
            y_cord += 0.5
        elif direction == "WEST":
            x_cord += 0.5
        g_cost = 1
    
    if req.x <= x_cord and req.y <= y_cord:
	rospy.wait_for_service('check_is_edge')
	check_is_edge = rospy.ServiceProxy('check_is_edge',CheckEdge)
	x1=req.x
	y1=req.y
	x2=x_cord
	y2=y_cord
        isValidEdge = check_is_edge(x1,y1,x2,y2)
    else:
	rospy.wait_for_service('check_is_edge')
	check_is_edge = rospy.ServiceProxy('check_is_edge',CheckEdge)
	x2=req.x
	y2=req.y
	x1=x_cord
	y1=y_cord
        isValidEdge = check_is_edge(x1,y1,x2,y2)
    if not isValidEdge.value:
        return GetSuccessorResponse(-1, -1, direction, -1)

    return GetSuccessorResponse(x_cord, y_cord, direction, g_cost)

def remove_blocked_edge(req):
	bookname = req.bookname
	global books
	global mazeInfo
	location_of_blocked_edge_list = books["books"][bookname]["load_loc"]
	if location_of_blocked_edge_list[0][0] <= location_of_blocked_edge_list[1][0] and location_of_blocked_edge_list[0][1] <= location_of_blocked_edge_list[1][1]:
		blocked_edge = (location_of_blocked_edge_list[0][0], location_of_blocked_edge_list[0][1], location_of_blocked_edge_list[1][0], location_of_blocked_edge_list[1][1])
	else:
		blocked_edge = (location_of_blocked_edge_list[1][0], location_of_blocked_edge_list[1][1], location_of_blocked_edge_list[0][0], location_of_blocked_edge_list[0][1])
	mazeInfo.blocked_edges.remove(blocked_edge)
	return "1"


def server():
	rospy.Service('remove_blocked_edge', RemoveBlockedEdgeMsg,remove_blocked_edge)
	rospy.Service('check_is_edge',CheckEdge,check_is_edge)
	rospy.Service('reset_world',ResetWorldMsg,handle_reset_world)
	rospy.Service('get_successor', GetSuccessor, handle_get_successor)
	print "Ready!"
	rospy.spin()


if __name__ == "__main__":
	args = parser.parse_args()
	n_cans = args.n_cans
	n_cups = args.n_cups
	seed = args.seed
	print "n_cans: ", n_cans
	print "n_caps:", n_cups
	if n_cans > 20 or n_cans <4:
		print('Maximum no. of cans available are: 20 and Minimum number available are:4')
		exit()
	if n_cups>n_cans:
		prints('Max no.of cups should be less than no.of cans')
		exit()
	grid_size = 8 * (n_cans//4)
	mazeInfo = Maze(grid_size, 0.5)
	cans = mazeInfo.generate_blocked_edges(n_cans,n_cups, seed, root_path)
	mazeInfoCopy = copy.deepcopy(mazeInfo)
	#print "blocked_edges: ", mazeInfo.blocked_edges
	rospy.init_node('server')
	robot_action_server = RobotActionsServer(cans, root_path, args.action_seed)
	server()
