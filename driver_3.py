import math
from collections import deque

# Eduardo Cruz

class Node(object):
	"""docstring for Node"""
	def __init__(self, uniqueId, state: list, parent, action: str, cost: int):
		super(Node, self).__init__()
		self.uniqueId = uniqueId # must be in the form: hash('parent.uniqueId + childNumber')
		self.state = state
		self.parent = parent
		self.action = action
		self.cost = cost

	def toString(self):
		return '(' + str(self.state) + ', ' + str(type(self.parent)) + ', ' + self.action + ',' + str(self.cost) + ')'

class Solver(object):
	"""docstring for Solver"""
	def __init__(self, method: str, initState: list):
		super(Solver, self).__init__()
		self.method = method
		self.initState = initState

		self.visited = deque()
		self.frontier = deque() 
		# visited and frontier can be used as a stack or queue
		# use .append() and .popleft() to behave as queue (FIFO)
		# use .append() and .pop() to behave as stack (LIFO)

		self.Profiler = Profiler()

		self.set_goal()

	def set_goal(self): # run once
		self.stateGoal = []
		length = int(len(self.initState))
		i = 0
		for i in range(length):
			self.stateGoal.append(i)

	def check_goal(self, state: list):
		i = 0
		for item in state:
			if item != self.stateGoal[i]:
				return False
			i += 1
		return True

	# recursive, use solution = [] (empty list) for first call
	def path_to_goal(self, node, solution: list):
		if node.parent == 0 :
			return solution
		else:
			solution.append(node.action)
			self.path_to_goal(node.parent, solution)

	def node_in_deque(self, nodeId, dequeHaystack):
		for node in dequeHaystack:
			if nodeId == node.uniqueId:
				return True
		return False

	def bfs(self):
		initNode = Node(0, self.initState, 0, '', 0) # root node
		self.frontier.append(initNode)

		while self.frontier: # while frontier not empty
			print(self.frontier)

			visitedNode = self.frontier.popleft()
			self.visited.append(visitedNode)

			if self.check_goal(visitedNode.state):
				print('found solution!!!: ', self.path_to_goal(visitedNode, []))
				return True

			else:
				board = Board(visitedNode.state)
				parentId = visitedNode.uniqueId
				childNumber = 1
				for action in board.get_possible_actions():

					childId = hash(str(parentId) + str(childNumber)) # a repeatable id

					if not self.node_in_deque(childId, self.frontier) and not self.node_in_deque(childId, self.visited):
						newNode = Node(childId, board.doAction(action), parentId, action, 1)
						self.frontier.append(newNode)

					childNumber += 1
		return False

class Profiler(object):
	"""docstring for Profiler"""
	def __init__(self):
		super(Profiler, self).__init__()
		self.path_to_goal = 0
		self.cost_of_path = 0
		self.nodes_expanded = 0
		self.fringe_size = 0
		self.max_fringe_size = 0
		self.search_depth = 0
		self.max_search_depth = 0
		self.running_time = 0
		self.max_ram_usage = 0

class Board(object):
	"""docstring for Board"""
	def __init__(self, state: list):
		super(Board, self).__init__()
		self.state = state
		self.dim = int(math.sqrt(len(self.state))) # find and store the board dimension 
		self.emptyAtIndex = 0
		self.possibleActions = []

		self.find_possible_actions()

	def get_state(self):
		return self.state

	def get_possible_actions(self):
		return self.possibleActions

	def find_possible_actions(self):
		self.possibleActions = []
		i = 0
		col = 0
		row = 0
		while i < len(self.state): # finds row and col where the zero value (empty tile) is at:
			row = math.floor(i / self.dim)
			col = i % self.dim
			if self.state[i] == 0:
				#print('Found empty slot on: (' + str(row) + ',' + str(col) + ')')
				self.emptyAtIndex = i
				break
			i = i+1

		# determine possible actions based on row and col
		# visit child nodes in the "UDLR" order
		if row > 0 :
			self.possibleActions.append('Up')
		if row < self.dim - 1 :
			self.possibleActions.append('Down')
		if col > 0 :
			self.possibleActions.append('Left')
		if col < self.dim - 1 :
			self.possibleActions.append('Right')

	# returns a new state
	def doAction(self, action):
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

testCase1 = Board([3,1,2,0,4,5,6,7,8])
testCase2 = Board([1,2,5,3,4,0,6,7,8])

solverBFS = Solver('bfs', [3,1,2,0,4,5,6,7,8])
solverBFS.bfs()
