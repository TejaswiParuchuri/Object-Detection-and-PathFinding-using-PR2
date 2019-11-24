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
#from object_detection import MoveRobotHead, ObjectClassifier

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
	from_location=api.State(0.0,0.0,'EAST')
	f=open(ROOT_PATH + '/state_action.txt', 'w+')
	init_state=(api.get_current_state()['robot']['x'], api.get_current_state()['robot']['y'], api.get_current_state()['robot']['orientation'])
	f.write("%s\t"%str(init_state))
	while cans_cups:
		choice=random.choice(cans_cups.values())
		print choice["load_loc"]
		key=cans_cups.keys()[cans_cups.values().index(choice)]
		actions,dummy_from_location,result=self.get_path(from_location,choice["load_loc"],choice["loc"])
		print actions,dummy_from_location,result
		for action in actions:
			action_params={}
			#print "Executing actions:",action
			success, next_state = api.execute_action(action, action_params)
			if not success:
				break
			f.write("%s \n"%action)
			init_state=(api.get_current_state()['robot']['x'], api.get_current_state()['robot']['y'], api.get_current_state()['robot']['orientation'])
			f.write("%s \t"%str(init_state))
		print "Successfully reached object:",key
		del cans_cups[key]
		# At this point the robot has reached the load location of an object, but it doesn't know if it's a coke can or a plastic cup.
		# Hence, execute the sense action.
		#robot_head = MoveRobotHead()
		#sense = robot_head.look_at(x, y, z)
		'''
		if sense == 0:
			# Execute pick action
			pass
		else:
			# Do not pick
			pass'''
		if 'can' in key:
			success,next_state=api.execute_action('pick',{"can_name":key})
			f.write("pick_%s \t"%str(key))
			print "I have executed pick action:",next_state
			actions,dummy_from_location,result=self.get_path(dummy_from_location,objects["bins"]["bin"]["load_loc"],objects["bins"]["bin"]["loc"])
			print actions,dummy_from_location,result
			for action in actions:
				action_params={}
				#print "Executing actions:",action
				success, next_state = api.execute_action(action, action_params)
				if not success:
					break
				f.write("%s \n"%action)
				init_state=(api.get_current_state()['robot']['x'], api.get_current_state()['robot']['y'], api.get_current_state()['robot']['orientation'])
				f.write("%s \t"%str(init_state))
			print "Successfully reached bin"
			success,next_state=api.execute_action('place',{"can_name":key,"bin_name":"bin"})
			print "I have executed place action:",next_state
			f.write("place_%s \t"%str(key))
		from_location=dummy_from_location
	f.close()
	print "Successfully Completed"
	
			
    def is_goal_state(self, current_state, goal_state):
        if(current_state.x == goal_state.x and current_state.y == goal_state.y and current_state.orientation == goal_state.orientation):
            return True
        return False		
    def get_manhattan_distance(self, from_state, to_state):
        return abs(from_state.x - to_state.x) + abs(from_state.y - to_state.y)


    def build_goal_states(self, locations,original_location):
        states = []
	#print "Original_location:",original_location
        for location in locations:
	    if(location[0]<original_location[0] and location[1]<original_location[1]):
            	states.append(api.State(location[0], location[1], "NORTH"))
            	states.append(api.State(location[0], location[1], "EAST"))
	    elif(location[0]>original_location[0] and location[1]<original_location[1]):
            	states.append(api.State(location[0], location[1], "NORTH"))
		states.append(api.State(location[0], location[1], "WEST"))
	    elif(location[0]<original_location[0] and location[1]>original_location[1]):
            	states.append(api.State(location[0], location[1], "SOUTH"))
            	states.append(api.State(location[0], location[1], "EAST"))
	    else:
            	states.append(api.State(location[0], location[1], "SOUTH"))
		states.append(api.State(location[0], location[1], "WEST"))
            '''states.append(api.State(location[0], location[1], "EAST"))
            states.append(api.State(location[0], location[1], "WEST"))
            states.append(api.State(location[0], location[1], "NORTH"))
            states.append(api.State(location[0], location[1], "SOUTH"))'''
        return states    


    def get_path(self, init_state, goal_locations,original_location):
        final_state = None
        goal_states = self.build_goal_states(goal_locations,original_location)
        goal_reached = False
        for goal_state in goal_states: #search for any of the load locations
            possible_actions = ['moveF','TurnCW','TurnCCW']
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
		#print current_state        
                if(current_state in visited):
                    continue
                
                if(self.is_goal_state(current_state, goal_state)):
                    goal_reached = True
                    break

                visited.append(current_state)
                for action in possible_actions:
		    #print action
                    nextstate, cost = api.get_successor(current_state, action)
		    #print nextstate,cost
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
