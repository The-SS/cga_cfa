#!/usr/bin/env python
'''
We resolve collisions using a randomly prioritized sequential algorithm for altitude assignment.

Author:
Sleiman Safaoui:
GitHub:
The-SS
Email:
snsafaoui@gmail.com
sleiman.safaoui@utdallas.edu

Date:
June 3, 2018

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
import copy
from ccda import *

# from assign_goals import *


########### altitude_assignment
def altitude_assignment(current_pos, goal_pos, speed_agents, radius_agents, t0): # assigns colliding agents to different altitudes
    num_agents = len(current_pos)
    num_altitudes = 1
    alt_agent_assignment = [[]] # list of lists where each would contain the agents at an altitude
    alt_goal_assignment = [[]] # list of lists where each would contain the goals at an altitude
    alt_assignment_flags = [[]] # list of lists of flags indicating whether agent belongs to an altitude

    # set the first agent and its goal to the first altitutde
    alt_agent_assignment[0].append(current_pos[0])
    alt_goal_assignment[0].append(goal_pos[0])
    alt_assignment_flags[0] = np.zeros(num_agents)
    alt_assignment_flags[0][0] = 1

    # print('5')
    # print('alt_agent_assignment', alt_agent_assignment)
    # print('alt_goal_assignment', alt_goal_assignment)
    # print('alt_assignment_flags', alt_assignment_flags)

    # assign the rest of the agents to collision free altitudes
    for agent in range(1, num_agents): # loop through the agents
        assigned = False # check to test agent is assigned
        for alt in range(num_altitudes): # for each agent, start at the lowest altitude and check if it could be assigned there. If not, move up a level and check with the agents of that level. Repeat until agent is assigned
            agents_at_altitude = copy.copy(alt_agent_assignment[alt]) # agents at current altitude
            goals_at_altitude = copy.copy(alt_goal_assignment[alt]) # goals at current altitude
            agents_at_altitude.append(current_pos[agent]) # add current agent to agents at current altitude
            goals_at_altitude.append(goal_pos[agent]) # do the same for the current goal

            flags = CCDA(agents_at_altitude, goals_at_altitude, speed_agents, radius_agents, t0) # test to see if adding the agent will cause collisions
            check = not np.any(flags)
            if check: # if no collision is detected
                alt_agent_assignment[alt] = agents_at_altitude # add agent to list of agents at this altitude
                alt_goal_assignment[alt] = goals_at_altitude # and add its goal position to the goals at current altitude
                alt_assignment_flags[alt][agent] = 1 # raise the flag indicating that the agents belongs to the current altitude
                assigned = True
                break

        if not assigned: # if collisions were detected at all altitudes, add the agent to a new altitude
            num_altitudes += 1 # increase number of altitudes
            alt_agent_assignment.append([current_pos[agent]]) # add a new list (new height) with the current agent
            alt_goal_assignment.append([goal_pos[agent]]) # add a new list (new height) with the current goal
            alt_assignment_flags.append(np.zeros(num_agents)) # add a new list (new height) of flags
            alt_assignment_flags[num_altitudes-1][agent] = 1 # flag the current agent at the current height

    print(num_altitudes)
    return alt_assignment_flags, alt_agent_assignment, alt_goal_assignment

#
# def tb():
#     '''
#     testbench for altitude_assignment
#     '''
#     current_pos = import_CSV('/Users/TheSS/cga_cfa/cga_cfa/config/agents_locations.csv')
#     goal_pos = import_CSV('/Users/TheSS/cga_cfa/cga_cfa/config/desired_goals.csv')
#     speed_agents = 1
#     radius_agents = 0.1
#     t0 = 0
#
#     flags, alt_ag, alt_goal = altitude_assignment(current_pos, goal_pos, speed_agents, radius_agents, t0)
#     print('flags', flags)
#     print('alt_ag', alt_ag)
#     print('alt_goal', alt_goal)
#     print('num levels', len(alt_ag))
#     for i in range(len(alt_ag)):
#         print('level ', i)
#         print('alt_ag at current level',alt_ag[i])
#         print('alt_goal at current level',alt_goal[i])
#
# if __name__ == '__main__':
#     tb()







#
