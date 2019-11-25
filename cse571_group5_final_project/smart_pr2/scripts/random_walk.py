#!/usr/bin/env python
# encoding: utf-8

import heapq
import environment_api as api 
import rospy
from std_msgs.msg import String
import numpy as np
import random

class RandomWalker:

    def __init__(self):
        self.current_state = api.get_current_state()
        print "Current State:"
        print self.current_state
        self.action_list = api.get_all_actions()
        self.random_walk()

    def random_walk(self):

        print("Robot can perform following actions: {}".format(self.action_list))
        while True:

            if api.is_terminal_state(self.current_state):
                print "Goal Reached"
                break

            possible_actions = api.get_possible_actions(self.current_state)
            print("Possible actions in current state: {}".format(possible_actions))
            
            for action in possible_actions:
                print "Action {}".format(action)
                if action == "pick": # try to pick book 1
                    action_params = {"book_name":"book_1"}
                elif action == "place":
                    action_params = {"book_name":"book_1", "bin_name":"trolly_2"}
                else:
                    action_params = {}

                states = api.get_possible_states(self.current_state, action, action_params)
                print "Possible states are:"
                for state in states:
                    next_state = states[state][0]
                    probability = states[state][1]
                    print state
                    print "State: ", next_state
                    print "Probability: ", probability
                    print "Reward: ", api.get_reward(self.current_state, action, next_state)
                    print ""


            idx = random.randint(0, len(possible_actions) - 1)
            chosen_action = possible_actions[idx]
            if chosen_action == "pick": # try to pick book 1
                action_params = {"book_name":"book_1"}
            elif chosen_action == "place":
                action_params = {"book_name":"book_1", "bin_name":"trolly_2"}
            else:
                action_params = {}

            print "Executing action: {} with params: {}".format(chosen_action, action_params)

            success, next_state = api.execute_action(chosen_action, action_params)
            if success == 1:
                print "Successfully executed"
            else:
                print "Action failed"
            
            self.current_state = next_state
            print "updated current state:"
            print self.current_state

            raw_input("\nPress Enter to continue execution...")


if __name__ == "__main__":
    random_walker = RandomWalker()