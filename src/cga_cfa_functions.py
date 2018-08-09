#!/usr/bin/env python
'''
Functions/Modules to be used by the cga_cfa script

Author:
Sleiman Safaoui:
GitHub:
The-SS
Email:
snsafaoui@gmail.com
sleiman.safaoui@utdallas.edu

Date:
Aug 6, 2018

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
from scipy.optimize import linear_sum_assignment


# Useful function
def import_CSV(file_name): # returns content of a csv file
    '''
    Imports data from a CSV file. First row is skipped and only two columns are read
    '''
    data = np.loadtxt(file_name, delimiter=',', usecols=range(2), skiprows = 1)
    return(data)

def euclidean_distance(point_a, point_b):
    return ((float(point_a[0]) - float(point_b[0]))**2. + (float(point_a[1]) - float(point_b[1]))**2.)**0.5

def euclidean_norm(vector):
    return (vector[0]**2 + vector[1]**2)**0.5


# Helper function
def find_t_crit(t0, tf_i, tf_j, t_cpa): # finds t_crit
    '''
    finds the critical time: the time at which the distance between two trajectories is at its minimum value by comparing start time, end time, and time of closest distance.
    '''
    tf = min(tf_i, tf_j)
    if (t0 >= t_cpa):
        t_crit = t0
    elif (t_cpa >= tf):
        t_crit = tf
    else:
        t_crit = t_cpa
    return t_crit

def find_duration(pos1, pos2, vel): # finds duration for each agent to go to all goal locations
    '''
    Finds the duration of all journies between a set of planar start points and a set of planar end points given the velocity
    '''
    if vel == 0:
        return []

    r = len(pos1)
    c = len(pos2)
    dist = np.zeros([r, c])
    duration = np.zeros([r, c])
    for i in range(r):
        for j in range(c):
            dist[i][j] = ( (pos2[j][0] - pos1[i][0])**2 + (pos2[j][1] - pos1[i][1])**2 )**0.5
            duration[i][j] = dist[i][j] / vel
    return duration

def CCDA(current_pos, goal_pos, speed, radius, t0): # CCDA collision detection algorithm
    '''
    Constraint Collision Detection Algorithm
    Detects collisions between agents following a linear trajectory in a 2D plane
    '''
    # Notes:
    # goal_pos is arranged so that the first goal position corresponds to the first agents and so on
    # speed cannot be zero

    len_cur = len(current_pos) # number of agents
    len_goal = len(goal_pos) # number of goal locations
    e_min = np.zeros([len_cur, len_goal]) # minimum error in clearance between agents
    start_pos_rel = np.zeros([len_cur, len_goal, 2]) # relative start position vector between agents
    heading_vector = np.zeros([len_cur, 2]) # heading vector of each agent (goal - cur_pos)
    unit_heading_vector = np.zeros([len_cur, 2]) # normalized heading direction of each agent (heading / norm(heading))
    vel = np.zeros([len_cur, 2]) # velocity vector of each agent
    vel_rel = np.zeros([len_cur, len_goal, 2]) # relative velocity vector between agents
    tf = np.zeros([len_cur]) # time to complete trajectory
    t_cpa = np.zeros([len_cur, len_goal]) # time at closest point of approach (cpa) between agents (in global time frame)
    t_crit = np.zeros([len_cur, len_goal]) # time at cpa bound to motion time
    flag = np.zeros([len_cur, len_goal]) # flag array indicating collisions if element is 1 (0 <--> no collision)


    for i in range(len_cur):
        heading_vector[i] = goal_pos[i] - current_pos[i]
        norm_heading = euclidean_norm(heading_vector[i])
        if norm_heading == 0:
            unit_heading_vector[i] = 0.*heading_vector[i] #float('inf')
        else:
            unit_heading_vector[i] = heading_vector[i] / euclidean_norm(heading_vector[i])
        vel[i] = speed*unit_heading_vector[i]
        tf[i] = euclidean_distance(goal_pos[i], current_pos[i])/speed

    for i in range(len_cur):
        for j in range(len_goal):
            start_pos_rel[i,j] = list( np.array(current_pos[j]) - np.array(current_pos[i]))
            vel_rel[i,j] = list(np.array(vel[j]) - np.array(vel[i]))
            dot_vel = np.dot(vel_rel[i,j], vel_rel[i,j])
            if dot_vel == 0:
                t_cpa[i,j] = float('inf')
            else:
                t_cpa[i,j] = - (np.dot(start_pos_rel[i,j], vel_rel[i,j]))/(dot_vel) + t0
            t_crit[i,j] = find_t_crit(t0, tf[i], tf[j], t_cpa[i,j])
            e_min[i,j] = euclidean_norm((t_crit[i,j]*vel_rel[i,j]+start_pos_rel[i,j])) - 2*radius
            if ((i!=j) & (e_min[i,j] < 0)):
                flag[i,j] = 1
            else:
                pass
    return flag


# Main functions
def assign_goals(current_pos, goal_pos, vel): # rearranges goals based on shortest distance
    '''
    arranges goals based on a linear sum assignmnet problem
    '''
    duration = find_duration(current_pos, goal_pos, vel)
    agents, order = linear_sum_assignment(duration) # solve the linear assignment problem that matches each agent with a goal
    #print('order', order)
    goal_pos = np.array(goal_pos)
    assigned_goals = np.array(goal_pos[order]) # rearrange the goal locations
    return assigned_goals, order

def altitude_assignment(current_pos, goal_pos, speed_agents, radius_agents, t0): # assigns colliding agents to different altitudes
    '''
    Uses the curent position and the desired (ordered) position of the agents to detect possible collisions between agents as they travel at a constant speed along straight planar lines. Collisions are resolved by an altitude assignment.
    '''


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

    # assign the rest of the agents to collision free altitudes
    for agent in range(1, num_agents): # loop through the agents
        assigned = False # check to test agent is assigned
        for alt in range(num_altitudes): # for each agent, start at the lowest altitude and check if it could be assigned there. If not, move up a level and check with the agents of that level. Repeat until agent is assigned
            agents_at_altitude = copy.copy(alt_agent_assignment[alt]) # agents at current altitude
            goals_at_altitude = copy.copy(alt_goal_assignment[alt]) # goals at current altitude
            agents_at_altitude.append(current_pos[agent]) # add current agent to agents at current altitude
            goals_at_altitude.append(goal_pos[agent]) # do the same for the current goal

            flags = CCDA(agents_at_altitude, goals_at_altitude, speed_agents, radius_agents, t0) # test to see if adding the agent will cause collisions
            check = not np.any(flags) # check for collisions
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
            alt_assignment_flags[-1][agent] = 1 # flag the current agent at the current height

    #print('number of altitudes', num_altitudes)
    return alt_assignment_flags, alt_agent_assignment, alt_goal_assignment


#
