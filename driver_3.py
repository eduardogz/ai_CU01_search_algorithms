"""

ColumbiaX: CSMM.101x Artificial Intelligence (AI) 2017
Project 1: search algorithms
Author: Eduardo Cruz, eduardo.gz@gmail.com

These classes implement various search algorithms to solve an n-puzzle.

Run Test Case 1 on the CLI:
python driver_3.py bfs 3,1,2,0,4,5,6,7,8 

Test Case 2  on the CLI:
python driver_3.py dfs 1,2,5,3,4,0,6,7,8

"""

import sys
import math
import os # to detect windows and omit memory measurement as ru_maxrss is unix only
if os.name != 'nt':
	import resource
from collections import deque
from time import time

class Node(object):
	""" Node object used to build and traverse the tree """
	def __init__(self, state: list, parent, action: str, cost: int):
		super(Node, self).__init__()
		self.state = state
		self.parent = parent
		self.action = action
		self.cost = cost

	def toString(self):
		return '(' + str(self.state) + ', ' + str(type(self.parent)) + ', ' + self.action + ',' + str(self.cost) + ')'

class Solver(object):
	""" Main class to select method and hold common data structures """
	def __init__(self, method: str, initState: list):
		super(Solver, self).__init__()
		self.method = method
		self.initState = initState

		self.nodeDB = dict() # our little node database
		
		# explored and fringe are deque() lists that can be used as a stack or as a queue
		# use .append() and .popleft() to behave as queue (FIFO)
		# use .append() and .pop() to behave as stack (LIFO)
		self.explored = deque()
		self.fringe = deque() 

		self.profiler = Profiler()
		self.set_goal()

		while True:
			if self.method == 'bfs' or self.method == 'dfs':
				self.bfs_dfs(self.method)
				break
			break

	def set_goal(self): 
		"""  Generates the goal state, an ordered list of integers from 0 to the size of provided initState; and gets its ID """
		stateGoal = list()
		length = int(len(self.initState))
		i = 0
		for i in range(length):
			stateGoal.append(i)
		self.goalId = self.create_node_id(stateGoal)
		##print('goal: ', self.goalId)

	def path_to_goal(self, nodeId):
		""" Calculates the path to root and reverses it to provide the path to goal, also prepares profiler stats """
		solution = list()
		while nodeId != 0:
			solution.append(self.nodeDB[nodeId].action)
			nodeId = self.nodeDB[nodeId].parent
			self.profiler.costOfPath = self.profiler.costOfPath + self.nodeDB[nodeId].cost
			self.profiler.searchDepth += 1
		solution.reverse()
		self.profiler.pathToGoal = solution
		self.profiler.fringeSize = len(self.fringe)
		self.profiler.nodesExpanded = len(self.explored)

	def calculate_depth(self, nodeId):
		depth = 0
		while nodeId != 0:
			depth += 1
			nodeId = self.nodeDB[nodeId].parent
		return depth

	def create_node_id(self, state):
		""" Creates an ID based on the state by converting each element to string, joining them, and casting to integer """
		return int(''.join(map(str, state)))

	def bfs_dfs(self, method):
		""" Breadth First Search (BFS) and Depth First Search (DFS) """
		self.profiler.runningTimeStart = time()
		initNode = Node(self.initState, 0, '', 1) # root node
		self.nodeDB[0] = initNode # save initNode in nodeDB
		self.fringe.append(0) # set id of root node to 0 and add to fringe
		
		while self.fringe: # while fringe not empty
			#print('*** START ***')
			if method == 'bfs':
				nodeIdFromFringe = self.fringe.popleft() # queue
			if method == 'dfs':
				nodeIdFromFringe = self.fringe.pop() # stack
			#print('exploring node:', nodeIdFromFringe, 'created from (', self.nodeDB[nodeIdFromFringe].action,')')

			if nodeIdFromFringe == self.goalId:
				self.path_to_goal(nodeIdFromFringe)
				self.profiler.runningTimeEnd = time()
				self.profiler.write_file() 
				return True # we found a solution

			else:
				board = Board(self.nodeDB[nodeIdFromFringe].state, self.nodeDB[nodeIdFromFringe].action)
				#print(board.pretty_out())
				for action in board.get_possible_actions():
					##print('opening action: ', action)
					newState = board.do_action(action)
					childId = self.create_node_id(newState)

					# check if childId is in the fringe and in explored deques
					childIdInFringe = False
					childIdInExplored = False
					try:
						self.fringe.index(childId)
						childIdInFringe = True
						try:
							self.explored.index(childId)
							childIdInExplored = True
						except:
							pass
					except:
						pass

					if not childIdInFringe and not childIdInExplored:

						newNode = Node(newState, nodeIdFromFringe, action, 1)
						self.nodeDB[childId] = newNode

						self.fringe.append(childId)
						#print('created new node', childId, action, '; added to fringe, len:', len(self.fringe))

						self.profiler.update_max_fringe_size(len(self.fringe))
						self.profiler.update_search_depth(self.calculate_depth(childId))
						
				self.explored.append(nodeIdFromFringe)
				#print('added node', nodeIdFromFringe, 'to explored, len: ', len(self.explored))
				#print('*** END ***\n\n')

		self.profiler.runningTimeEnd = time()
		return False

class Profiler(object):
	""" Class to save and output statistics """
	def __init__(self):
		super(Profiler, self).__init__()
		self.pathToGoal = 0
		self.costOfPath = 0
		self.nodesExpanded = 0
		self.fringeSize = 0
		self.maxFringeSize = 0
		self.searchDepth = 0
		self.maxSearchDepth = 0
		self.runningTimeStart = 0
		self.runningTimeEnd = 0

	def update_max_fringe_size(self, fringeLen):
		if self.maxFringeSize < fringeLen:
			self.maxFringeSize = fringeLen

	def update_search_depth(self, depth):
		if self.maxSearchDepth < depth:
			self.maxSearchDepth = depth


	def write_file(self):

		runningTime = self.runningTimeEnd - self.runningTimeStart
		output = 'path_to_goal: ' + str(self.pathToGoal) + '\n'
		output += 'cost_of_path: ' + str(self.costOfPath) + '\n'
		output += 'nodes_expanded: ' + str(self.nodesExpanded) + '\n'
		output += 'fringe_size: ' + str(self.fringeSize) + '\n'
		output += 'max_fringe_size: ' + str(self.maxFringeSize) + '\n'
		output += 'search_depth: ' + str(self.searchDepth) + '\n'
		output += 'max_search_depth: ' + str(self.maxSearchDepth) + '\n'
		output += 'running_time: ' + str(runningTime) + '\n'
		
		if(os.name == 'nt'):
			output += 'max_ram_usage = 0\n'
		else:
			output += 'max_ram_usage: ' + str(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024) + '\n'

		file = open('output.txt', 'w')
		file.write(output)
		print(output)

class Board(object):
	""" Low-level methods for the board """
	def __init__(self, state: list, action):
		super(Board, self).__init__()
		self.state = state
		self.dim = int(math.sqrt(len(self.state))) # find and store the board dimension 
		self.emptyAtIndex = 0
		self.possibleActions = []
		self.action = action # the action that created this board

		self.find_possible_actions()

	def get_state(self):
		return self.state

	def get_possible_actions(self):
		return self.possibleActions

	def find_possible_actions(self):
		""" returns a list with the legal actions for the current state """
		self.possibleActions = []
		i = 0
		col = 0
		row = 0
		while i < len(self.state): # finds row and col where the zero value (empty tile) is at:
			row = math.floor(i / self.dim)
			col = i % self.dim
			if self.state[i] == 0:
				###print('Found empty slot on: (' + str(row) + ',' + str(col) + ')')
				self.emptyAtIndex = i
				break
			i = i+1

		# determine possible actions based on row and col
		# visit child nodes in the "UDLR" order
		# AND EXCLUDE the inverse action that created the board
		if row > 0 and self.action != 'Down':
			self.possibleActions.append('Up')
		if row < self.dim - 1 and self.action != 'Up':
			self.possibleActions.append('Down')
		if col > 0 and self.action != 'Right':
			self.possibleActions.append('Left')
		if col < self.dim - 1 and self.action != 'Left':
			self.possibleActions.append('Right')

	def do_action(self, action):
		""" Execute an action to current state and return the resulting new state """
		try:
			self.possibleActions.index(action)
		except:
			#print('Board.do_action: requested action "'+ action + '" NOT possible, ignoring request.')
			return None

		newState = list(self.state)

		if action == 'Up':
			targetCellIndex = self.emptyAtIndex - self.dim
			targetCellValue = self.state[targetCellIndex]
			newState[self.emptyAtIndex] = targetCellValue
			newState[targetCellIndex] = 0
			return newState
		if action == 'Down':
			targetCellIndex = self.emptyAtIndex + self.dim
			targetCellValue = self.state[targetCellIndex]
			newState[self.emptyAtIndex] = targetCellValue
			newState[targetCellIndex] = 0
			return newState
		if action == 'Left':
			targetCellIndex = self.emptyAtIndex - 1
			targetCellValue = self.state[targetCellIndex]
			newState[self.emptyAtIndex] = targetCellValue
			newState[targetCellIndex] = 0
			return newState
		if action == 'Right':
			targetCellIndex = self.emptyAtIndex + 1
			targetCellValue = self.state[targetCellIndex]
			newState[self.emptyAtIndex] = targetCellValue
			newState[targetCellIndex] = 0
			return newState
	def pretty_out(self):
		""" A prettier output for debugging """
		stateLen = len(self.state)
		prettyRow = ''
		prettyOut = '\n'
		for i in range(stateLen):
			prettyRow += str(self.state[i]) + ' '
			if (i + 1) % self.dim == 0:
				prettyOut += prettyRow + '\n'
				prettyRow = ''
		return prettyOut

# Entry point
cliMethod = sys.argv[1]
cliState = list ( map ( int, list(sys.argv[2].split(',') ) ) )
#print('running: ' + cliMethod + ' on :', cliState)
solver = Solver(cliMethod, cliState)
