#!/usr/bin/env python
import sys
import rospy
from cse571_project.srv import *
from std_msgs.msg import String
import json

def get_current_state():
    """
    This function calls get_initial_state service to recive the initial state of the turtlebot.

    return:  x_cord - initial x-cordinate of turtlebot           
             y_cord - initial y-cordinate of turtlebot
             direction - initial orientation
    """
    rospy.wait_for_service('get_current_state')
    try:
        get_initial_state = rospy.ServiceProxy('get_current_state', GetInitialState)
        response = get_initial_state()
        return json.loads(response.state)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def is_terminal_state(state):
    rospy.wait_for_service('is_terminal_state')
    try:
        is_term_state = rospy.ServiceProxy('is_terminal_state', IsTerminalState)
        response = is_term_state(json.dumps(state))
        return response.value == 1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def reset_world():
    rospy.wait_for_service('reset_world')
    try:
        handle = rospy.ServiceProxy('reset_world', ResetWorldMsg)
        response = handle()
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def get_all_actions():
    """
    This function calls is_goal_state service to check if the current state is the goal state or not.

    parameters:  x_cord - current x-cordinate of turtlebot           return:   1 : if current state is the goal state
                 y_cord - current y-cordinate of turtlebot                     0 : if current state is not the goal state
    """
    rospy.wait_for_service('get_all_actions')
    try:
        all_actions = rospy.ServiceProxy('get_all_actions', GetActions)
        response = all_actions()
        return response.actions.split(",")
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def get_possible_actions(state):
    rospy.wait_for_service('get_possible_actions')
    try:
        possible_actions = rospy.ServiceProxy('get_possible_actions', GetPossibleActions)
        state = json.dumps(state)
        response = possible_actions(state)
        return response.actions.split(",")
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def get_possible_states(state, action, action_params):
    rospy.wait_for_service('get_possible_actions')
    try:
        possible_states = rospy.ServiceProxy('get_possible_states', GetPossibleStates)
        state = json.dumps(state)
        action_params = json.dumps(action_params)
        response = possible_states(state, action, action_params)
        return json.loads(response.states)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def get_reward(state, action, next_state):
    rospy.wait_for_service('get_reward')
    try:
        get_reward = rospy.ServiceProxy('get_reward', GetReward)
        state = json.dumps(state)
        next_state = json.dumps(next_state)
        response = get_reward(state, action, next_state)
        return response.reward
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def execute_action(action, action_params):
    rospy.wait_for_service('execute_action')
    try:
        execute_action = rospy.ServiceProxy('execute_action', ActionMsg)
        action_params = json.dumps(action_params)
        response = execute_action(action, action_params)
        return response.success, json.loads(response.next_state)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

