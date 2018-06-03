#!/usr/bin/env python
'''
We use current position and goal position data to find possible collisions. We then resolve these collisions.

Author:
Sleiman Safaoui:
GitHub:
The-SS
Email:
snsafaoui@gmail.com
sleiman.safaoui@utdallas.edu

Date:
May 29, 2018

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
# import csv
import math
# import matplotlib.pyplot as plt
# from scipy.optimize import linear_sum_assignment
# import rospy
# import roslib

current_pos = np.array([[1.7351, 1.0643],[-2.1862, -2.5050],[1.6837, -0.9659],[-2.9210, 1.0299],[-0.4582, 0.1292],[-0.0988, -2.1177],[-0.5646, 2.3109],[2.9597, -2.9703],[2.9639, 2.8435],[-2.4635, 2.9821]])
goal_pos = np.array([[-2.0113, 1.9968],[-2.4400, -1.9845],[0.6231, 2.3760],[1.0993, -1.4096],[2.3509, 1.3046],[-0.7279, -0.3862],[2.5815, -2.8898],[-0.4143, -2.7870],[-2.9354, 0.0158],[2.9761, -0.6480]])
speed = 2
radius = 1
t0 = 0

def euclidean_distance(point_a, point_b):
    return ((point_a[0] - point_b[0])**2 + (point_a[1] - point_b[1])**2)**0.5

def euclidean_norm(vector):
    return (vector[0]**2 + vector[1]**2)**0.5

def find_t_crit(t0, tf_i, tf_j, t_cpa): # finds t_crit
    tf = min(tf_i, tf_j)
    if (t0 >= t_cpa):
        t_crit = t0
    elif (t_cpa >= tf):
        t_crit = tf
    else:
        t_crit = t_cpa
    return t_crit


def CCDA(current_pos, goal_pos, speed, radius, t0):
    '''
    Constraint Collision Detection Algorithm
    Detects collisions between agents following a linear trajectory in a 2D plane
    '''
    # Notes:
    # - here, goal_pos is arranged so that the first goal position corresponds to the first agents and so on
    # - here speed cannot be zero

    len_cur = len(current_pos) # number of agents
    len_goal = len(goal_pos) # number of goal locations
    # for now, len_cur = len_goal
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
            unit_heading_vector[i] = float('inf')
        else:
            unit_heading_vector[i] = heading_vector[i] / euclidean_norm(heading_vector[i])
        vel[i] = speed*unit_heading_vector[i]
        tf[i] = euclidean_distance(goal_pos[i], current_pos[i])/speed #TODO: fill in

    for i in range(len_cur):
        for j in range(len_goal):
            start_pos_rel[i,j] = current_pos[j] - current_pos[i]
            vel_rel[i,j] = vel[j] - vel[i]
            dot_vel = np.dot(vel_rel[i,j], vel_rel[i,j])
            if dot_vel == 0:
                t_cpa[i,j] = float('inf')
            else:
                t_cpa[i,j] = - (np.dot(start_pos_rel[i,j], vel_rel[i,j]))/(np.dot(vel_rel[i,j], vel_rel[i,j])) + t0
            t_crit[i,j] = find_t_crit(t0, tf[i], tf[j], t_cpa[i,j])
            e_min[i,j] = euclidean_norm((t_crit[i,j]*vel_rel[i,j]+start_pos_rel[i,j])) - 2*radius
            if ((i!=j) & (e_min[i,j] < 0)):
                flag[i,j] = 1
            else:
                pass
    return flag

def main():
    flag = CCDA(current_pos, goal_pos, speed, radius, t0)
    print(flag)


if __name__ == '__main__':
    main()

#
