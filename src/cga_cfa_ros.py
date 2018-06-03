#!/usr/bin/env python
'''
Use the other scripts to find collision free goal assignments.
This script is ROS compatible. It it called through the launch file.
!!!!!!!!!!! DEPENDS ON ROSPARAM !!!!!!!!!!!

Author:
Sleiman Safaoui:
GitHub:
The-SS
Email:
snsafaoui@gmail.com
sleiman.safaoui@utdallas.edu

Date:
May 31, 2018

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
# import matplotlib.pyplot as plt
import rospy
import roslib
from ccda import *
from assign_goals import *

# load ros parameters
num_agents = rospy.get_param("/cga_cfa/num_agents")
density = rospy.get_param("/cga_cfa/density")
radius_agents = rospy.get_param("/cga_cfa/radius_agents")
agent_locations_file = rospy.get_param("/cga_cfa/agent_locations_file")
goals_file = rospy.get_param("/cga_cfa/goals_file")
speed_agents = rospy.get_param("/cga_cfa/speed_agents")
t0 = rospy.get_param("/cga_cfa/t0")


def main():
    # find goal position
    goal_pos = import_CSV(goals_file)
    if goal_pos == []:
        print('Goal positions not defined')
        return
    # find current position
    current_pos = import_CSV(agent_locations_file)
    if current_pos == []:
        print('Current positions not defined')
        return
    # check that the two positions match in number (1-1 map)
    len_cur = len(current_pos)
    len_goal = len(goal_pos)
    if (len_cur != num_agents |  len_goal != num_agents):
        print('Number of current positions or goal positions is different than number of agents')
        return
    # assign goals
    assigned_goals = assign_goals(current_pos, goal_pos, speed_agents)
    print(assigned_goals)
    # check for collisions
    flags = CCDA(current_pos, assigned_goals, speed_agents, radius_agents, t0)
    print(flags)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass