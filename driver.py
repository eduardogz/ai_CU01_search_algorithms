import math

class Node(object):
	"""docstring for Node"""
	def __init__(self, parent, action):
		super(Node, self).__init__()
		self.parent = parent
		self.action = action

class State(object):
	"""docstring for State"""
	def __init__(self, arg):
		super(State, self).__init__()
		self.arg = arg

class Solver(object):
	"""docstring for Solver"""
	def __init__(self, arg):
		super(Solver, self).__init__()
		self.arg = arg

class Board(object):
	"""docstring for Board"""
	def __init__(self, state: list):
		super(Board, self).__init__()
		self.state = state
		self.dim = int(math.sqrt(len(self.state))) # find and store the board dimension 
		self.emptyAtIndex = 0
		self.possibleActions = []

		self.findPossibleActions()

	def findPossibleActions(self):
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

	def doAction(self, action):
		try:
			self.possibleActions.index(action)
		except:
			print('Board.doAction: requested action "'+ action + '" NOT possible, ignoring request.')
			return None

		while (True):
			if action == 'Up':
				targetCellIndex = self.emptyAtIndex - self.dim
				targetCellValue = self.state[targetCellIndex]
				self.state[self.emptyAtIndex] = targetCellValue
				self.state[targetCellIndex] = 0
				break
			if action == 'Down':
				targetCellIndex = self.emptyAtIndex + self.dim
				targetCellValue = self.state[targetCellIndex]
				self.state[self.emptyAtIndex] = targetCellValue
				self.state[targetCellIndex] = 0
				break
			if action == 'Left':
				targetCellIndex = self.emptyAtIndex - 1
				targetCellValue = self.state[targetCellIndex]
				self.state[self.emptyAtIndex] = targetCellValue
				self.state[targetCellIndex] = 0
				break
			if action == 'Right':
				targetCellIndex = self.emptyAtIndex + 1
				targetCellValue = self.state[targetCellIndex]
				self.state[self.emptyAtIndex] = targetCellValue
				self.state[targetCellIndex] = 0
				break
			break # should never get here but protect from infinite loop
		self.findPossibleActions() # update possibleActions
		

testCase1 = Board([3,1,2,0,4,5,6,7,8])
testCase2 = Board([1,2,5,3,4,0,6,7,8])

print(testCase1.state)
testCase1.doAction('Left')
print(testCase1.state)
testCase1.doAction('Right')
print(testCase1.state)
testCase1.doAction('Right')
print(testCase1.state)