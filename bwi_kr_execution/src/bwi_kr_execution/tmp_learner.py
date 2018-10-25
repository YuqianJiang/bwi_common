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

class TMPLearner():
	def __init__(self):
		self.q_table = np.zeros((nS, nA))
		self.ro_table = np.zeros((nS, nA))
		self.state_list = []
		self.action_list = []
		self.sa_set = set()
		self.current_action = None
		self.start_time = None
		self.state = []

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

	def ro_table_to_asp(self):
		cost_file = open("/tmp/cost.asp","w")

		qrule = "#program base.\n"
		for (state,action) in self.sa_set:
			actionname = self.action_list[action]
			symbolicstate = self.state_list[state]
			qrule += "ro(("+','.join(str(e) for e in symbolicstate) +"),"+actionname+","+str(int(math.floor(self.ro_table[state,action])))+").\n"

		print qrule
		cost_file.write(qrule)
		cost_file.close()

	def learn(self, state, state_next, action, reward):

		state_idx = self.encode_state(state)
		state_next_idx = self.encode_state(state_next)
		action_idx = self.encode_action(action)

		if (state_idx, action_idx) not in self.sa_set:
			self.sa_set.add((state_idx,action_idx))

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
			plan_quality += int(math.floor(self.ro_table[state, action]))

		constraint_file = open("/tmp/constraint.asp","w")

		rule = "#program check(n).\n" + "#external query(n).\n"
		rule += ":- C <= " + str(plan_quality) + ", cost(C,n), query(n).\n"
		constraint_file.write(rule)
		constraint_file.close()
