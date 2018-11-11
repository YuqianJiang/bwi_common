#!/usr/bin/env python

import numpy as np
import os
import sys
import math

#somehow necessary for importing local modules
if not hasattr(sys, 'argv'):
    sys.argv  = ['']


nA = 1000
nS = 1000

# Parameters
LEARNING_RATE_Q = 1
LEARNING_RATE_R = 1
DISCOUNT = 0.001
EPSILON = 0.01
BETA = 0.3

class CostLearner():
	def __init__(self):
		self.q_table = np.zeros((nS, nA))
		self.ro_table = np.zeros((nS, nA))
		self.cost_table = np.zeros((nS, nA))
		self.state_list = []
		self.action_list = []
		self.sa_set = set()
		self.state = []
		self.tables = {'cost_table' : self.cost_table,
									 'q_table' : self.q_table,
									 'ro_table' : self.ro_table}

	def encode_state(self, state_factored):

		if state_factored in self.state_list:
			return self.state_list.index(state_factored)
		else:
			self.state_list.append(state_factored)

		return self.state_list.index(state_factored)

	def encode_action(self, action):
		if action in self.action_list:
			return self.action_list.index(action)
		else:
			self.action_list.append(action)

		return self.action_list.index(action)

	def table_to_asp(self, table_name):
		cost_file = open("/tmp/costs.asp","w")
		table = self.tables[table_name]

		rule = "#program base.\n"
		for (state,action) in self.sa_set:
			actionname = self.action_list[action]
			symbolicstate = self.state_list[state]
			rule += "c(("+','.join(str(e) for e in symbolicstate) +"),"+actionname+","+str(-int(round(table[state,action])))+").\n"

		#print rule
		cost_file.write(rule)
		cost_file.close()

	def learn(self, state, state_next, action, reward):

		reward = float(reward)

		state_idx = self.encode_state(state)
		state_next_idx = self.encode_state(state_next)
		action_idx = self.encode_action(action)

		if (state_idx, action_idx) not in self.sa_set:
			self.sa_set.add((state_idx,action_idx))

		self.cost_table[state_idx, action_idx] = reward

		q_update = 0.1 * (reward - self.ro_table[state_idx,action_idx] + max(self.q_table[state_next_idx, :]) - self.q_table[state_idx, action_idx])
		self.q_table[state_idx, action_idx] += q_update

		ro_update = 0.5 * (reward + max(self.q_table[state_next_idx, :]) - max(self.q_table[state_idx, :])- self.ro_table[state_idx, action_idx])
		self.ro_table[state_idx, action_idx] += ro_update

		#print "q value:", self.q_table[state_idx, action_idx]
		#print "ro value", self.ro_table[state_idx, action_idx]
		#print "sa_set", self.sa_set
		#print "state_list", self.state_list
		#print "action_list", self.action_list

	def constrain_plan_quality(self, path):

		plan_quality = 0
		for (state,action) in path:
			state_idx = self.encode_state(state)
			action_idx = self.encode_action(action)
			plan_quality += int(round(self.ro_table[state_idx, action_idx]))

		constraint_file = open("/tmp/constraint.asp","w")

		#rule = "#program check(n).\n" + "#external query(n).\n"
		rule = "#program check(n).\n#external query(n).\n"
		rule += ":- C >= " + str(-plan_quality) + ", total_cost(C, n).\n"
		print(rule)
		constraint_file.write(rule)
		constraint_file.close()

	def clear_constraint(self):

		constraint_file = open("/tmp/constraint.asp","w")
		constraint_file.close()