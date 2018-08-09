#!/usr/bin/env python
'''
Concurrent goal assignment with collision free avoidance implementation

Author:
Sleiman Safaoui
Github:
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
from cga_cfa_functions import import_CSV, assign_goals, altitude_assignment
import os


class CGA_CFA_ALT:
    '''
    class that handles concurrent goal assignments with collision free avoidance via altitude resolution
    '''
    def __init__ (self, safe_radius = 0.1, agent_speed = 0.3, t_init = 0.0):
        self.agent_speed = agent_speed # speed of agents (max and constant)
        self.safe_radius = safe_radius # minimum distance allowed between agents
        self.init_time = t_init # time at which the trajectories will start (0. is good enough)
        self.num_agents = 0 # number of agents
        self.current_pos = [] # current locations
        self.goals = [] # goal locations
        # variables for results
        self.assigned_goals = [] # rearranged goals
        self.num_altitudes = 0 # number of altitudes
        self.altitude_assignment_flags = [] # flags for altitude assignment (1 --> agent is at that altitude. 0--> not at that altitude)
        self.alt_agent_assignment = [] # list of agents at each altitude (their initial position)
        self.alt_goal_assignment = [] # list of goals at each altitude
        '''
        altitude_assignment_flags, alt_agent_assignment, and alt_goal_assignment:
        altitude_assignment_flags is a number_of_altitudes x number_of_agents list. At each altitude, the index corresponds to the agent. The value at the index is a boolean to specify whether or not the agents will be at that height
        alt_agent_assignment is list with number_of_altitudes lists. Each internal list contains the current position of the agents that will be at the altitude specified by the index value.
        alt_goal_assignment is list containing number_of_altitudes lists. Each internal list contains the goal position of the agents that will be at the altitude specified by the index value.
        The alt_agent_assignment and alt_goal_assignment are coupled.
        '''

    def update_agent_speed(self, speed):
        '''
        Updates the speed of the agents
        '''
        if speed > 0.0:
            self.agent_speed = speed
            return True
        else:
            print('Could not update agent speed')
            return False

    def update_safe_radius(self, r):
        '''
        Updates the safety radius
        '''
        if r >= 0.0:
            self.safe_radius = r
            return True
        else:
            print('Could not update safe radius')
            return False

    def update_init_time(self, t0):
        '''
        Updates the inital time
        '''
        if t0 >= 0.0:
            self.init_time = t0
            return True
        else:
            print('Could not update initial time')
            return False

    def update_current_position(self, pos):
        '''
        Updates the current position of the agents
        '''
        if list(pos) == []:
            print('Could not update current position')
            return False
        else:
            self.current_pos = pos
            return True

    def update_goal_position(self, pos):
        '''
        Updates the goal positions for the agents
        '''
        if list(pos) == []:
            print('Could not update goal position')
            return False
        else:
            self.goals = pos
            return True

    def update_static_params(self, speed, radius, t):
        '''
        updates the speed, safe radius, and initial time
        '''
        c1 = self.update_agent_speed(float(speed))
        c2 = self.update_safe_radius(float(radius))
        c3 = self.update_init_time(float(t))

        if c1:
            print('Speed updated')
        if c2:
            print('Safety radius updated')
        if c3:
            print('Initial time updated')

        if c1 & c2 & c3:
            print('Static parameters updated successfully!')
            return True # all parameters updated
        else:
            print('Not all parameters were updated')
            return False

    def update_positions(self, current, goal):
        '''
        updates the current position and the goal position of the robots
        '''

        cc = self.update_current_position(current)
        cg = self.update_goal_position(goal)

        if cc & cg: # both parameters were updated
            lc = len(current)
            lg = len(goal)

            if lc == lg: # both parameters have same length --> data updated correctly
                self.num_agents = lc
                print('Positions updated successfully!')
                return True
            else:
                self.num_agents = 0
                print('Current and goal position sizes do not match. Setting number of agents to zero')
                return False

        else: # either parameter was not updated
            self.num_agents = 0
            print('Could not update both positions. Setting number of agents to zero')
            return False

    def goal_assignment(self):
        '''
        Assign agents to goals to minimize total flight time
        '''
        if self.num_agents == 0:
            print('Cannot assign goals. Update parameters correctly first.')
            return False

        # All data in parameters is correct
        self.assigned_goals, _ = assign_goals(self.current_pos, self.goals, self.agent_speed)
        print('Goal assignment complete!')
        return True

    def collision_avoidance(self):
        '''
        collision avoidance
        '''
        if self.num_agents == 0:
            print('Cannot apply collision avoidance yet. Update parameters correctly first.')
            self.altitude_assignment_flags = []
            self.num_altitudes = 0
            return False

        if self.assigned_goals == []:
            print('Cannot apply collision avoidance yet. Assign goals first.')
            self.altitude_assignment_flags = []
            self.num_altitudes = 0
            return False

        flags, alt_agents, alt_goals = altitude_assignment(self.current_pos, self.assigned_goals, self.agent_speed, self.safe_radius, self.init_time) # apply collision avoidance strategy
        self.altitude_assignment_flags = map(list, zip(*map(list,zip(*flags))))
        self.alt_agent_assignment = alt_agents
        self.alt_goal_assignment  = alt_goals
        self.num_altitudes, _ = np.array(self.altitude_assignment_flags).shape # find number of altitudes to be used
        print('Collision avoidance via altitude assignments complete!')
        return True

    # API for retrieving data
    def get_number_agents(self):
        '''
        returns the number of agents
        '''
        return self.num_agents

    def get_assigned_goals(self):
        '''
        assigned goals by index (e.g: row i is for agent i)
        num_of_agents x 2 list
        '''
        return self.assigned_goals

    def get_number_altitudes(self):
        '''
        total number of altitudes that are required (integer)
        '''
        return self.num_altitudes

    def get_altitude_assignment_flags(self):
        '''
        altitude assignment flags
        num_altitudes x num_agents list
        '''
        return self.altitude_assignment_flags

    def get_agents_at_alt(self, alt):
        '''
        list of agents at a specific altitude (altitude's number 1-based indexing).
        Indexes start from 0 (0-based indexing)
        Initial and final postions are paired with the index
        '''
        alt = alt-1 # change it to 0 based indexing
        if alt < 0:
            return [],[],[]
        if alt > self.num_altitudes -1 :
            return [],[],[]

        idx = list(np.nonzero(self.altitude_assignment_flags[alt])[0] + 1)
        pos_c = list(self.alt_agent_assignment[alt])
        pos_g = list(self.alt_goal_assignment[alt])
        return idx, pos_c, pos_g

    def get_alt_agent(self, agent):
        '''
        returns the altitude that the agent is assigned to.
        agent numbre uses 0-based indexing (0, 1, 2, ...)
        returned altitude uses 1-based indexing
        '''
        if ((agent < 0) | (agent > self.num_agents-1)):
            print("Agent index is invalid")
            return -1

        for i in range(self.num_altitudes):
            if self.altitude_assignment_flags[i][agent] == 1:
                return i+1

        print("Could not find altitude of agent")
        return -1

    def get_goal_agent(self, agent):
        '''
        returns the planar goal of the agent.
        agent numbre uses 0-based indexing (0, 1, 2, ...)
        returned goal is for the planar value (independent of the altitude)
        '''
        if ((agent < 0) | (agent > self.num_agents-1)):
            print("Agent index is invalid")
            return []

        return self.assigned_goals[agent]


# Sample usage
def main():
    cca = CGA_CFA_ALT(safe_radius = .3) # Creating an object of the class
    # current = import_CSV('../config/agents_locations.csv') # importing current positions
    current = [[ 0.  , 0. ],[ 0.  ,-0.5],[ 0.  ,-1. ],[ 0.5 , 0. ],[ 0.5 ,-0.5],[ 0.5 ,-1. ],[-0.5,  0. ],[-0.5, -0.5],[-0.5, -1. ]] # Or defining current postions manually
    goal = import_CSV('../config/desired_goals.csv') # importing goal positions
    # print('current\n', current)
    # print('goal\n', goal)

    # Running the cga cfa algorithms
    cca.update_static_params(speed=.3, radius=.3, t=0.) # updating static parameters (optional)
    print('speed',cca.agent_speed)
    print('radius', cca.safe_radius)
    print('t', cca.init_time)
    cca.update_positions(current, goal) # updating positions (required)

    cca.goal_assignment() # running goal assignment algorithm

    cca.collision_avoidance() # running collision avoidance algorithm

    # Printing out results using provided API functions
    print('assigned_goals\n', cca.get_assigned_goals()) # assigned goals
    print('num_altitudes ', cca.get_number_altitudes()) # number of altitudes
    print('altitude_assignment_flags\n', cca.get_altitude_assignment_flags()) # altitude flags

    # getting agents of a certain altitude (index, current position, goal position)
    print('Agents at altitude 1:\n', cca.get_agents_at_alt(1))
    print('Agents at altitude 2:\n', cca.get_agents_at_alt(2))

    # getting altitude and goal of a certian agent by index
    for i in range(cca.get_number_agents()):
        print('Alt of agent %d is %f' %(i,cca.get_alt_agent(i)))
        print('Goal of agent %d is ' %i, cca.get_goal_agent(i))

if __name__ == "__main__":
    main()
