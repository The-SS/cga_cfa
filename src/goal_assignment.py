#!/usr/bin/env python
'''
We import the current location of robots (2D position in a plane) and the desired locations (2D positions) from two csv files.
Then, we use this data to match each agent with the closest goal position to it based on travel time using the Hungarian Algorithm. This results in a one-to-one mapping between each agent and a desired destination that ensures minimum travel time.

Author:
Sleiman Safaoui:
GitHub:
The-SS
Email:
snsafaoui@gmail.com
sleiman.safaoui@utdallas.edu

Date:
May 24, 2018

Theory:
Benjamin Gravell
Email:
bjgravell@gmail.com
Tyler Summers
Email:
tyler.summers@utdallas.edu
'''
from __future__ import print_function
import numpy as np
import csv
import math
import matplotlib.pyplot as plt
from scipy.optimize import linear_sum_assignment
import rospy
import roslib

# num_agents = 10
# density = 0.5
# radius_agents = 1 #0.1
# vel_agents = 1 #0.3
# agent_locations_file = "agents_locations.csv"
# goals_file = "desired_goals.csv"

# load ros parameters
num_agents = rospy.get_param("/goal_assignment/num_agents")
density = rospy.get_param("/goal_assignment/density")
radius_agents = rospy.get_param("/goal_assignment/radius_agents")
vel_agents = rospy.get_param("/goal_assignment/vel_agents")
agent_locations_file = rospy.get_param("/goal_assignment/agent_locations_file")
goals_file = rospy.get_param("/goal_assignment/goals_file")


def import_CSV(file_name): # returns content of a csv file
    data = np.loadtxt(file_name, delimiter=',', usecols=range(2), skiprows = 1)
    return(data)

def find_duration(pos1, pos2, vel): # finds duration for each agent to go to all goal locations
    r = len(pos1)
    c = len(pos2)
    dist = np.zeros([r, c])
    duration = np.zeros([r, c])
    for i in range(r):
        for j in range(c):
            dist[i][j] = ( (pos2[j][0] - pos1[i][0])**2 + (pos2[j][1] - pos1[i][1])**2 )**0.5
            duration[i][j] = dist[i][j] / vel
    return duration

def assign_goal(current_pos, goal_pos, vel): # rearranges goals based on shortest distance
    duration = find_duration(current_pos, goal_pos, vel)
    agents, order = linear_sum_assignment(duration) # solve the linear assignment problem that matches each agent with a goal
    print(order)
    assigned_goals = np.array(goal_pos[order]) # rearrange the goal locations
    return assigned_goals

def main():
    goal_pos = import_CSV(goals_file)
    current_pos = import_CSV(agent_locations_file)
    len_cur = len(current_pos)
    len_goal = len(goal_pos)
    if (len_cur != num_agents |  len_goal != num_agents): # check if number of current and goal positions is correct
        print('Number of current positions or goal positions is different than number of agents')
        return
    assigned_goals = assign_goal(current_pos, goal_pos, vel_agents)
    print(assigned_goals)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
