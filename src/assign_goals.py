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
# import matplotlib.pyplot as plt
from scipy.optimize import linear_sum_assignment

########### FUNCTIONS FOR assign_goals
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

########### assign_goals
def assign_goals(current_pos, goal_pos, vel): # rearranges goals based on shortest distance
    duration = find_duration(current_pos, goal_pos, vel)
    agents, order = linear_sum_assignment(duration) # solve the linear assignment problem that matches each agent with a goal
    print(order)
    assigned_goals = np.array(goal_pos[order]) # rearrange the goal locations
    return assigned_goals
