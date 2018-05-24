#!/usr/bin/env python
'''
We import the current location of aerial robots (3D position) and the desired locations (3D positions) from two csv files.
Then, we use this data to match each agent with the closest goal position to it using the Hungarian Algorithm. This results in a one-to-one mapping between each agent and a desired destination.

Author:
Sleiman Safaouigit:
GitHub:
The-SS
Email:
snsafaoui@gmail.com
sleiman.safaoui@utdallas.edu

Date:
May 24, 2018
'''
from __future__ import print_function
import numpy as np
import csv
import math
import matplotlib.pyplot as plt
import rospy
import roslib

# load ros parameters
num_agents = rospy.get_param("/goal_assignment/num_agents")
density = rospy.get_param("/goal_assignment/density")
radius_agents = rospy.get_param("/goal_assignment/radius_agents")
vel_agents = rospy.get_param("/goal_assignment/vel_agents")
agent_locations_file = rospy.get_param("/goal_assignment/agent_locations_file")
goals_file = rospy.get_param("/goal_assignment/goals_file")


def ImportCSV(file_name): # returns content of a csv file
    data = np.loadtxt(file_name, delimiter=',', usecols=range(3))
    rb = csv.reader(open(file_name, "rb"), delimiter=',', skiprows = 1)
    print(data)
    return(data)

def main():
    current_pos = ImportCSV(agent_locations_file)
    goal_pos = ImportCSV(goals_file) 

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
