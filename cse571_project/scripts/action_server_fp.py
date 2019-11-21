#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelState,ModelStates
from std_msgs.msg import String
from cse571_project.srv import *
import numpy as np
import tf
import math
import copy
import json
from collections import namedtuple

class RobotActionsServer:

    def __init__(self, object_dict, root_path, random_seed=10):
        self.object_dict = object_dict
        self.failure = -1
        self.success = 1
        self.status = String(data='Idle')
        self.model_state_publisher = rospy.Publisher("/gazebo/set_model_state",ModelState,queue_size = 10)
        self.action_publisher = rospy.Publisher("/actions", String, queue_size=10)
        self.status_publisher = rospy.Publisher("/status", String, queue_size=10)
        self.random_seed = random_seed
        self.current_state = self.generate_init_state()
        self.action_config = self.load_action_config(root_path + '/action_config_fp.json')
        self.direction_list = ["NORTH","EAST","SOUTH","WEST"]
        np.random.seed(self.random_seed)
        rospy.Service("execute_action", ActionMsg,self.execute_action)
        rospy.Service('get_all_actions', GetActions, self.get_all_actions)
        rospy.Service('get_possible_actions', GetPossibleActions, self.get_possible_actions)
        rospy.Service('get_possible_states', GetPossibleStates, self.get_possible_states)
        rospy.Service('get_reward', GetReward, self.get_reward)
        rospy.Service('is_terminal_state', IsTerminalState, self.is_terminal_state_handler)
        rospy.Service('get_current_state', GetInitialState, self.get_current_state)
        print "Action Server Initiated"


    def generate_init_state(self):
        state = {"holding":None}
        state['robot'] = {'x': 0.0, 'y': 0.0, 'orientation': 'EAST'}
        for can in self.object_dict["cans"]:
            state[can] = {
                            'x': float(self.object_dict["cans"][can]["loc"][0]), 
                            'y': float(self.object_dict["cans"][can]["loc"][1]), 
                            'placed': False
                        }
        for cup in self.object_dict["cups"]:
            state[cup] = {
                            'x': float(self.object_dict["cups"][cup]["loc"][0]),
                            'y': float(self.object_dict["cups"][cup]["loc"][1]),
                        }
        state["bin"]={'x': float(self.object_dict["bins"]["bin"]["loc"][0]),
                  'y': float(self.object_dict["bins"]["bin"]["loc"][1]),}
        return state


    def get_current_state(self, req):
        return json.dumps(self.current_state)


    def load_action_config(self, action_config_file):
        f = open(action_config_file)
        action_config = json.load(f)
        f.close()
        return action_config


    def get_turtlebot_location(self,state):
        return state['robot']['x'], state['robot']['y'], state['robot']['orientation']


    def release_object(self, object_name):
        print("TODO: Release Object {}".format(object_name))

    def grasp_can(self, can_name):
        print("TODO: Grasp Can {}".format(can_name))

    def remove_edge(self, book_name):
        rospy.wait_for_service('remove_blocked_edge')
        try:
            remove_edge = rospy.ServiceProxy('remove_blocked_edge',RemoveBlockedEdgeMsg)
            _ = remove_edge(book_name)
        except rospy.ServiceException,e:
            print "Sevice call failed: %s"%e


    def check_edge(self, x1, y1, x2, y2):
        rospy.wait_for_service('check_is_edge')
        try:
            check_is_edge = rospy.ServiceProxy('check_is_edge',CheckEdge)
            if x1 <= x2 and y1 <= y2:
                result = check_is_edge(x1,y1,x2,y2)
            else:
                result = check_is_edge(x2,y2,x1,y1)
            return result.value == 1
        except rospy.ServiceException,e:
            print "Sevice call failed: %s"%e


    def is_terminal_state_handler(self, req):
        state = json.loads(req.state)
        return self.is_terminal_state(state)


    def is_terminal_state(self, state):
        # Terminal state is reached when all books are placed
        cnt = 0
        for key in state.keys():
            if key.startswith('cans'):
                if state[key]['placed']:
                    cnt += 1

        if cnt == len(self.object_dict["cans"].keys()):
            return 1
        else:
            return 0


    def get_all_actions(self, req):
        return ','.join(self.action_config.keys())


    def get_possible_actions(self, req):
        state = req.state
        
        # These actions are executable anywhere in the environment
        action_list = ['sense', 'pick', 'place', 'TurnCW', 'TurnCCW']

        # Check if we can execute moveF
        success, next_state = self.execute_moveF(state)
        if success == 1:
            action_list.append('moveF')
        return ','.join(action_list)


    def get_possible_states(self, req):
        state = json.loads(req.state)
        action = req.action
        action_params = json.loads(req.action_params)

        next_states = {}
        i = 1
        for possible_action in self.action_config[action]['possibilities']:
            
            state_key = 'state_{}'.format(i)
            i += 1

            if possible_action == "noaction":
                next_states[state_key] = (state, self.action_config[action]['possibilities'][possible_action])
                continue
            
            # generate calling function
            calling_params = []
            for param in self.action_config[possible_action]['params']:
                calling_params.append("'" + action_params[param] + "'")
            calling_params.append("'" + json.dumps(state) + "'")
            calling_function = "self.{}({})".format(self.action_config[possible_action]['function'], ','.join(calling_params))
            success, next_state = eval(calling_function)

            next_states[state_key] = (next_state, self.action_config[action]['possibilities'][possible_action])

        return json.dumps(next_states)


    def get_reward(self, req):
        state = json.loads(req.state)
        action = req.action
        next_state = json.loads(req.next_state)

        if state == next_state:
            return self.action_config[action]['fail_reward']
        else:
            return self.action_config[action]['success_reward']
    

    def execute_action(self, req):
        action = req.action_name
        params = json.loads(req.action_params)

        # No operations in terminal state
        if self.is_terminal_state(self.current_state):
            return -1, json.dumps(self.current_state)

        # Choose an action based on probabilities in action config
        chosen_action = np.random.choice(self.action_config[action]['possibilities'].keys(), 
                                         p=self.action_config[action]['possibilities'].values())

        if chosen_action == "noaction":
            return self.failure, json.dumps(self.current_state)

        # generate calling function
        calling_params = []
        for param in self.action_config[chosen_action]['params']:
            calling_params.append("'" + params[param] + "'")
        calling_params.append("'" + json.dumps(self.current_state) + "'")
        calling_params.append('True')
        calling_function = "self.{}({})".format(self.action_config[chosen_action]['function'], ','.join(calling_params))
        success, next_state = eval(calling_function)
        
        # Update state
        self.current_state = copy.deepcopy(next_state)
        return success, json.dumps(next_state)

    def execute_sense(self, current_state, simulation=False):
        print("TODO: execute_sense function")
        current_state = json.loads(current_state)
        next_state = copy.deepcopy(current_state)
        return self.success, next_state

    def execute_place(self, can_name, bin_name, current_state, simulation=False):
        current_state = json.loads(current_state)
        robot_state = self.get_turtlebot_location(current_state)
        next_state = copy.deepcopy(current_state)

        ### Kirtus: Check if we are holding the can
        if can_name != current_state["holding"]:
            self.status_publisher.publish(self.status)
            return self.failure, current_state

        ### Kirtus: Validate CAN and BIN
        if can_name in self.object_dict["cans"] and bin_name in self.object_dict["bins"]:
            ### Kirtus: Make sure we are at the right location
            if (robot_state[0],robot_state[1]) in self.object_dict["bins"][bin_name]["load_loc"]:
                # Kirtus: simulation update
                if simulation:
                    goal_loc = list(self.object_dict["bins"][bin_name]["loc"])
                    goal_loc[0] = goal_loc[0] + 0.5
                    goal_loc[1] = goal_loc[1] + 0.5
                    self.release_object(book_name)
                    rospy.Rate(1).sleep()
                    
                self.status_publisher.publish(self.status)

                # Update state
                next_state[can_name]['x'] = -1
                next_state[can_name]['y'] = -1
                next_state[can_name]['placed'] = True
                next_state['holding'] = None
                    
        return self.success, next_state
        
        self.status_publisher.publish(self.status)
        return self.failure, next_state

    def execute_pick(self, can_name, current_state, simulation=False):
        current_state = json.loads(current_state)
        robot_state = self.get_turtlebot_location(current_state)
        next_state = copy.deepcopy(current_state)

        # Kirtus: validation
        if can_name in self.object_dict["cans"] and not current_state[can_name]['placed']:
            # Robot is at the load location for the book
            if (robot_state[0],robot_state[1]) in self.object_dict["cans"][can_name]["load_loc"]:
                # Basket is empty
                if current_state['holding'] is None:
                    
                    #Kirtus: Simulation update
                    if simulation:
                        self.grasp_can(can_name)
                        rospy.Rate(1).sleep()

                    self.status_publisher.publish(self.status)

                    #Kirtus: pick up the can
                    next_state['holding'] = can_name
                    next_state[can_name]['x'] = -1
                    next_state[can_name]['y'] = -1

                    return self.success, next_state

        self.status_publisher.publish(self.status)
        return self.failure, next_state

    def execute_moveF(self, current_state, simulation=False):
        current_state = json.loads(current_state)
        robot_state = self.get_turtlebot_location(current_state)
        next_state = copy.deepcopy(current_state)
        x1 = robot_state[0]
        y1 = robot_state[1]

        # Get new location
        if "EAST" in robot_state[2]:
            x2 = x1 + 0.5
            y2 = y1
        elif "WEST" in robot_state[2]:
            x2 = x1 - 0.5
            y2 = y1
        elif "NORTH" in robot_state[2]:
            x2 = x1
            y2 = y1 + 0.5
        else:
            x2 = x1
            y2 = y1 - 0.5

        # Check if that edge isn't blocked
        if self.check_edge(x1,y1,x2,y2):
            action_str = "MoveF"

            # Make bot move if simulating in gazebo
            if simulation:
                self.action_publisher.publish(String(data=action_str))
                rospy.wait_for_message("/status",String)
            
            # Update State
            next_state['robot']['x'] = x2
            next_state['robot']['y'] = y2

            return self.success, next_state
        else:
            return self.failure, next_state


    def execute_TurnCW(self, current_state, simulation=False):
        current_state = json.loads(current_state)
        next_state = copy.deepcopy(current_state)

        # Make bot move if simulating in gazebo
        if simulation:
            action_str = "TurnCW"
            self.action_publisher.publish(String(data=action_str))
            rospy.wait_for_message("/status",String)

        # Update state
        current_orientation = current_state['robot']['orientation']
        new_orientation = self.direction_list[(self.direction_list.index(current_orientation) + 1)%4]
        next_state['robot']['orientation'] = new_orientation

        return self.success, next_state


    def execute_TurnCCW(self, current_state, simulation=False):
        current_state = json.loads(current_state)
        next_state = copy.deepcopy(current_state)
        
        # Make bot move if simulating in gazebo
        if simulation:
            action_str = "TurnCCW"
            self.action_publisher.publish(String(data=action_str))
            rospy.wait_for_message("/status",String)
        
        # Update state
        current_orientation = current_state['robot']['orientation']
        new_orientation = self.direction_list[(self.direction_list.index(current_orientation) - 1)%4]
        next_state['robot']['orientation'] = new_orientation

        return self.success, next_state


if __name__ == "__main__":
    object_dict = None
    RobotActionsServer(object_dict)
