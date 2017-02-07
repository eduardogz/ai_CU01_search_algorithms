"""

ColumbiaX: CSMM.101x Artificial Intelligence (AI) 2017
Project 1: search algorithms
Author: Eduardo Cruz, eduardo.gz@gmail.com

These classes implement various search algorithms to solve an n-puzzle.

Run Test Case 1 on the CLI:
python driver_3.py method 3,1,2,0,4,5,6,7,8 

Test Case 2  on the CLI:
python driver_3.py method 1,2,5,3,4,0,6,7,8

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
		
		# visited and frontier can be used as a stack or queue
		# use .append() and .popleft() to behave as queue (FIFO)
		# use .append() and .pop() to behave as stack (LIFO)
		self.visited = deque()
		self.frontier = deque() 

		self.profiler = Profiler()
		self.set_goal()

		while True:
			if self.method == 'bfs':
				self.bfs()
			break

	def set_goal(self): 
		"""  Generates the goal state, an ordered list of integers from 0 to the size of provided initState """
		self.stateGoal = []
		length = int(len(self.initState))
		i = 0
		for i in range(length):
			self.stateGoal.append(i)
		#print('goal: ', self.stateGoal)

	def check_goal(self, state: list):
		""" Looks if the provided state matches the goal state """
		i = 0
		for item in state:
			if item != self.stateGoal[i]:
				return False
			i += 1
		return True

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
		self.profiler.fringeSize = len(self.frontier)
		self.profiler.nodesExpanded = len(self.visited)

	def calculate_depth(self, nodeId):
		depth = 0
		while nodeId != 0:
			depth += 1
			nodeId = self.nodeDB[nodeId].parent
		return depth

	def bfs(self):
		self.profiler.runningTimeStart = time()
		#print('solving: ', self.initState)
		initNode = Node(self.initState, 0, '', 1) # root node
		self.frontier.append(0) # set id of root node to 0
		self.nodeDB[0] = initNode # save initNode in nodeDB

		while self.frontier: # while frontier not empty
			visitedNodeId = self.frontier.popleft()
			board = Board(self.nodeDB[visitedNodeId].state, self.nodeDB[visitedNodeId].action)
			if self.check_goal(self.nodeDB[visitedNodeId].state):
				self.path_to_goal(visitedNodeId)
				self.profiler.runningTimeEnd = time()
				self.profiler.write_file()
				return True

			else:
				#print('exploring: ', visitedNodeId, ' :')
				#print(board.prettyPrint())
				parentId = visitedNodeId
				childNumber = 1
				for action in board.get_possible_actions():
					#print('opening action: ', action)
					childId = hash(str(parentId) + '-' + str(childNumber)) # use hash to speed things up a little

					# see if childId is in the frontier and in visited
					if self.frontier.count(childId) < 1 and self.visited.count(childId) < 1:
						newNode = Node(board.doAction(action), parentId, action, 1)
						self.nodeDB[childId] = newNode
						self.frontier.append(childId)

						currentFringeLen = len(self.frontier)
						if self.profiler.maxFringeSize < currentFringeLen:
							self.profiler.maxFringeSize = currentFringeLen

						currentNodeDepth = self.calculate_depth(childId)
						if self.profiler.maxSearchDepth < currentNodeDepth:
							self.profiler.maxSearchDepth = currentNodeDepth

						#print('frontier: ', self.frontier)
						#print('visited: ', self.visited,'\n')

					childNumber += 1
				self.visited.append(visitedNodeId)
		self.profiler.runningTimeEnd = time()
		return False

class Profiler(object):
	"""docstring for Profiler"""
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
			output += 'max_ram_usage: ' + str(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss) + '\n'

		file = open('output.txt', 'w')
		file.write(output)
		#print(output)

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
				##print('Found empty slot on: (' + str(row) + ',' + str(col) + ')')
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

	def doAction(self, action):
		""" Execute an action to current state and return the resulting new state """
		try:
			self.possibleActions.index(action)
		except:
			print('Board.doAction: requested action "'+ action + '" NOT possible, ignoring request.')
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
	def prettyPrint(self):
		""" A prettier output for debugging """
		stateLen = len(self.state)
		prettyOut = '\n'
		for i in range(stateLen):
			prettyOut += str(self.state[i]) + ' '
			if (i + 1) % self.dim == 0:
				print(prettyOut)
				prettyOut = ''
		prettyOut = '\n'
		return prettyOut

# Entry point
cliMethod = sys.argv[1]
cliState = list ( map ( int, list(sys.argv[2].split(',') ) ) )
#print('running: ' + cliMethod + ' on :', cliState)
solver = Solver(cliMethod, cliState)
