#!/usr/bin/env python
# encoding: utf-8

import heapq
import environment_api as api 
import rospy
from std_msgs.msg import String
import numpy as np
import random
import os
import json
import time

ROOT_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir))


class PathGeneration:

    def __init__(self):
        self.current_state = api.get_current_state()
        print "Current State:"
        print self.current_state
        self.action_list = api.get_all_actions()
	#self.state=api.get_initial_state(self.current_state)
	init_state=(api.get_current_state()['robot']['x'], api.get_current_state()['robot']['y'], api.get_current_state()['robot']['orientation'])
	print(init_state)
	self.path_generation()

    def path_generation(self):

        print("Robot can perform following actions: {}".format(self.action_list))
	#f=open(plan_file, "r")
	#contents =f.readlines()
	with open(ROOT_PATH+"/objects.json",'r') as json_file:
        	try:
                	objects = json.load(json_file)
	        except (ValueError, KeyError, TypeError):
        	        print "JSON error"
	#print objects
	cans_cups=objects["cans"]
	cans_cups.update(objects["cups"])
	choice=random.choice(cans_cups.values())
	print choice["load_loc"]
	'''location=[]
	for i in cans_cups.values():
		location+=i["load_loc"]
	print location'''
	#init_state=(api.get_current_state()['robot']['x'], api.get_current_state()['robot']['y'], api.get_current_state()['robot']['orientation'])
	from_location=api.State(0.0,0.0,'EAST')
	actions,dummy_from_location,result=self.get_path(from_location,choice["load_loc"])
	print actions,dummy_from_location,result
	'''can_name=''
	api.reset_world()
	while not api.is_terminal_state(api.get_current_state()):
			init_state=json.dumps(api.get_current_state())
			robot_state=[api.get_current_state()["robot"]["x"],api.get_current_state()["robot"]["y"]]
			if robot_state in location:
				possible_actions = api.get_possible_actions(self.current_state)
			else:
				possible_actions=['TurnCW', 'TurnCCW','moveF']
			#print possible_actions
			time.sleep(1)
			action=random.choice(possible_actions)
			#print action
			action_params={}
			if('pick' in action):
				can_name=random.choice(cans_cups.keys())
				action_params["can_name"]=can_name
			if('place' in action):
				action_params["can_name"]=can_name
				action_params["bin_name"]="bin"
			return_value,next_state=api.execute_action(action,action_params)
			if action=='pick':
				print 'pick'
				if return_value:
					print retrun_value,'pick successful'
					print next_state
			print return_value,(api.get_current_state()['robot']['x'], api.get_current_state()['robot']['y'], api.get_current_state()['robot']['orientation'])'''
			
    def is_goal_state(self, current_state, goal_state):
        if(current_state.x == goal_state.x and current_state.y == goal_state.y and current_state.orientation == goal_state.orientation):
            return True
        return False		
    def get_manhattan_distance(self, from_state, to_state):
        return abs(from_state.x - to_state.x) + abs(from_state.y - to_state.y)


    def build_goal_states(self, locations):
        states = []
        for location in locations:
            states.append(api.State(location[0], location[1], "EAST"))
            states.append(api.State(location[0], location[1], "WEST"))
            states.append(api.State(location[0], location[1], "NORTH"))
            states.append(api.State(location[0], location[1], "SOUTH"))
        return states    


    def get_path(self, init_state, goal_locations):
        final_state = None
        goal_states = self.build_goal_states(goal_locations)
        goal_reached = False
        for goal_state in goal_states: #search for any of the load locations
            possible_actions = api.get_all_actions()
            action_list = []
            
            state_queue = []
            heapq.heappush(state_queue, (self.get_manhattan_distance(init_state, goal_state), 0,(init_state, [])))
            visited = []
            state_cost = {}
            insert_order = 0
            while len(state_queue) > 0:
                top_item = heapq.heappop(state_queue)
                current_cost = top_item[0]
                current_state = top_item[2][0]
                current_actions = top_item[2][1]
                
                if(current_state in visited):
                    continue
                
                if(self.is_goal_state(current_state, goal_state)):
                    goal_reached = True
                    break

                visited.append(current_state)
                for action in possible_actions:
                    nextstate, cost = api.get_successor(current_state, action)
		    print nextstate,cost
                    cost = self.get_manhattan_distance(nextstate, goal_state) # manhattan distance heuristc
                    key = (nextstate.x, nextstate.y, nextstate.orientation)
                    if(nextstate.x == -1 and nextstate.y == -1):
                        continue
                    # if a new state is found then add to queue
                    if(nextstate not in visited and key not in state_cost.keys()):
                        heapq.heappush(state_queue, (cost, insert_order, (nextstate, current_actions + [action])))
                        insert_order += 1
                        state_cost[key] = cost
                    
            if(self.is_goal_state(current_state, goal_state)):
                action_list = current_actions
                final_state = current_state
                goal_reached = True
                break

        return action_list, final_state, goal_reached

if __name__ == "__main__":
    path_generator = PathGeneration()
