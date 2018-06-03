#!/usr/bin/env python
'''
Use the other scripts to find collision free goal assignments

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
    # find goal pos
    goal_pos = import_CSV(goals_file)
    current_pos = import_CSV(agent_locations_file)
    len_cur = len(current_pos)
    len_goal = len(goal_pos)
    if (len_cur != num_agents |  len_goal != num_agents): # check if number of current and goal positions is correct
        print('Number of current positions or goal positions is different than number of agents')
        return
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
