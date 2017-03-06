"""

ColumbiaX: CSMM.101x Artificial Intelligence (AI) 2017
Project 1: search algorithms
Author: Eduardo Cruz, eduardo.gz@gmail.com

These classes implement various search algorithms to solve an n-puzzle.

Initial state case 1: 3,1,2,0,4,5,6,7,8 
Initial state case 2: 1,2,5,3,4,0,6,7,8

Path to python 3.6:
"C:\Program Files (x86)\Python36-32\python.exe"

Run:
python driver_3.py METHOD STATE

Example:
"C:\Program Files (x86)\Python36-32\python.exe" driver_3.py bfs 3,1,2,0,4,5,6,7,8
"C:\Program Files (x86)\Python36-32\python.exe" driver_3.py bfs 1,2,5,3,4,0,6,7,8

"""

import sys
import math
import os  # to detect windows and omit memory measurement as ru_maxrss doesn't work on Windows
if os.name != 'nt':
    import resource
from collections import deque
from time import time


class Solver:
    """ Main class to select method and hold common data structures """

    def __init__(self, method: str, initState: tuple):
        super(Solver, self).__init__()
        self.initState = initState

        # saves states tuples, uses a double ended queue, can behave both as a
        # FIFO and as a LIFO
        self.fringe = deque()
        # saves states tuples; uses a set for speed when checking if a state
        # has been explored
        self.explored = set()
        # saves nodes details for every node state: nodeState -> nodeParent,
        # action, cost, depth
        self.nodeDB = dict()

        self.profiler = Profiler()
        self.set_goal()

        while True:
            if method == 'bfs':
                self.bfs()
                break
            if method == 'dfs':
                self.dfs()
                break
            if method == 'ast':
                self.ast()
                break
            break

    def create_node(self, stateParent: tuple, action: str, cost: int, depth: int):
        """ Returns a tuple with the node information to be saved in nodeDB with the node state as key"""
        return tuple(list((stateParent, action, cost, depth)))

    def set_goal(self):
        """  Generates the goal state, a tuple  from 0 to the size of provided initState; and gets its ID """
        stateGoal = list()
        length = int(len(self.initState))
        i = 0
        for i in range(length):
            stateGoal.append(i)
        self.stateGoal = tuple(stateGoal)  # save the list as a tuple
        #print('stateGoal: ', self.stateGoal)

    def path_to_goal(self, state):
        """ Calculates the path to root and reverses it to provide the path to goal, also prepares profiler stats """
        solution = list()
        while self.nodeDB[state][0] != None:  # root node has None as parent
            solution.append(self.nodeDB[state][1])
            # start with the node parent, because when solution is found
            # node[0] has the goal state
            state = self.nodeDB[state][0]
            self.profiler.costOfPath = self.profiler.costOfPath + \
                self.nodeDB[state][2]
            self.profiler.searchDepth += 1
        solution.reverse()
        self.profiler.pathToGoal = solution
        self.profiler.fringeSize = len(self.fringe)

    def bfs(self):
        """ Breadth First Search (BFS) """
        self.profiler.runningTimeStart = time()
        initNode = self.create_node(None, '', 1, 0)  # root node
        self.nodeDB[self.initState] = initNode
        self.fringe.append(self.initState)

        while self.fringe:  # while fringe not empty
            #print('*** START ***')
            stateFromFringe = self.fringe.popleft()  # queue

            self.explored.add(stateFromFringe)
            #print('add:', str(nodeFromFringe), 'len(explored): ', len(self.explored))

            if stateFromFringe == self.stateGoal:
                #print('found it! ')
                self.path_to_goal(stateFromFringe)
                self.profiler.runningTimeEnd = time()
                self.profiler.write_file()
                return True

            else:
                #print('exploring node:', str(nodeFromFringe))
                self.profiler.nodesExpanded += 1
                board = Board(stateFromFringe, self.nodeDB[stateFromFringe][1])
                # print(board.pretty_out())
                possibleActions = board.get_possible_actions()
                for action in possibleActions:
                    #print('opening action: ', action)
                    newState = board.do_action(action)
                    #print('new node', str(newState), ', add to fringe: ', len(self.fringe))

                    if (newState not in self.fringe) and (newState not in self.explored):

                        self.nodeDB[newState] = self.create_node(
                            stateFromFringe, action, 1, self.nodeDB[stateFromFringe][3] + 1)
                        self.fringe.append(newState)

                        self.profiler.update_max_fringe_size(len(self.fringe))
                        self.profiler.update_max_search_depth(
                            self.nodeDB[newState][3])

            #print('*** END ***\n\n')
            # print(str(self.profiler.nodesExpanded))

        self.profiler.runningTimeEnd = time()
        return False

    def dfs(self):
        """ Depth First Search (DFS) """
        self.profiler.runningTimeStart = time()
        initNode = self.create_node(None, '', 1, 0)  # root node
        self.nodeDB[self.initState] = initNode
        self.fringe.append(self.initState)

        while self.fringe:  # while fringe not empty
            #print('*** START ***')
            stateFromFringe = self.fringe.pop()  # stack

            self.explored.add(stateFromFringe)
            #print('add:', str(nodeFromFringe), 'len(explored): ', len(self.explored))

            if stateFromFringe == self.stateGoal:
                #print('found it! ')
                self.path_to_goal(stateFromFringe)
                self.profiler.runningTimeEnd = time()
                self.profiler.write_file()
                return True

            else:
                #print('exploring node:', str(nodeFromFringe))
                self.profiler.nodesExpanded += 1
                board = Board(stateFromFringe, self.nodeDB[stateFromFringe][1])
                # print(board.pretty_out())
                possibleActions = board.get_possible_actions()
                possibleActions.reverse()  # reverse-UDLR order
                for action in possibleActions:
                    #print('opening action: ', action)
                    newState = board.do_action(action)
                    #print('new node', str(newState), ', add to fringe: ', len(self.fringe))

                    if (newState not in self.fringe) and (newState not in self.explored):

                        self.nodeDB[newState] = self.create_node(
                            stateFromFringe, action, 1, self.nodeDB[stateFromFringe][3] + 1)
                        self.fringe.append(newState)

                        self.profiler.update_max_fringe_size(len(self.fringe))
                        self.profiler.update_max_search_depth(
                            self.nodeDB[newState][3])

            #print('*** END ***\n\n')
            # print(str(self.profiler.nodesExpanded))

        self.profiler.runningTimeEnd = time()
        return False

    def ast(self):
        """ A-Star (A*) """
        self.profiler.runningTimeStart = time()
        initNode = self.create_node(None, '', 1, 0)  # root node
        self.nodeDB[self.initState] = initNode
        self.fringe.append(self.initState)

        while self.fringe:  # while fringe not empty
            #print('*** START ***')
            stateFromFringe = self.fringe.popleft()  # queue

            self.explored.add(stateFromFringe)
            #print('add:', str(nodeFromFringe), 'len(explored): ', len(self.explored))

            if stateFromFringe == self.stateGoal:
                #print('found it! ')
                self.path_to_goal(stateFromFringe)
                self.profiler.runningTimeEnd = time()
                self.profiler.write_file()
                return True

            else:
                #print('exploring node:', str(nodeFromFringe))
                self.profiler.nodesExpanded += 1
                board = Board(stateFromFringe, self.nodeDB[stateFromFringe][1])
                # print(board.pretty_out())
                possibleActions = board.get_possible_actions()
                for action in possibleActions:
                    #print('opening action: ', action)
                    newState = board.do_action(action)
                    #print('new node', str(newState), ', add to fringe: ', len(self.fringe))

                    if (newState not in self.fringe) and (newState not in self.explored):

                        self.nodeDB[newState] = self.create_node(
                            stateFromFringe, action, 1, self.nodeDB[stateFromFringe][3] + 1)
                        self.fringe.append(newState)

                        self.profiler.update_max_fringe_size(len(self.fringe))
                        self.profiler.update_max_search_depth(
                            self.nodeDB[newState][3])

            #print('*** END ***\n\n')
            # print(str(self.profiler.nodesExpanded))

        self.profiler.runningTimeEnd = time()
        return False


class Profiler:
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

    def update_max_search_depth(self, depth):
        if self.maxSearchDepth < depth:
            self.maxSearchDepth = depth

    def write_file(self):

        runningTime = str(self.runningTimeEnd - self.runningTimeStart)
        roundedRunningTime = '{:.10}'.format(runningTime)
        output = 'path_to_goal: ' + str(self.pathToGoal) + '\n'
        output += 'cost_of_path: ' + str(self.costOfPath) + '\n'
        output += 'nodes_expanded: ' + str(self.nodesExpanded) + '\n'
        output += 'fringe_size: ' + str(self.fringeSize) + '\n'
        output += 'max_fringe_size: ' + str(self.maxFringeSize) + '\n'
        output += 'search_depth: ' + str(self.searchDepth) + '\n'
        output += 'max_search_depth: ' + str(self.maxSearchDepth) + '\n'
        output += 'running_time: ' + roundedRunningTime + '\n'

        if(os.name == 'nt'):
            output += 'max_ram_usage = 0\n'
        else:
            output += 'max_ram_usage: ' + \
                str(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024) + '\n'

        file = open('output.txt', 'w')
        file.write(output)
        print(output)


class Board:
    """ Low-level methods for the board """

    def __init__(self, state: list, action: str):
        super(Board, self).__init__()
        self.state = state
        # find and store the board dimension
        self.dim = int(math.sqrt(len(self.state)))
        self.emptyAtIndex = 0
        self.possibleActions = []
        self.action = action  # the action that created this board

        self.find_possible_actions()

    def get_possible_actions(self):
        return self.possibleActions

    def find_possible_actions(self):
        """ returns a list with the legal actions for the current state """
        self.possibleActions = []
        i = 0
        col = 0
        row = 0
        # finds row and col where the zero value (empty tile) is at:
        while i < len(self.state):
            row = math.floor(i / self.dim)
            col = i % self.dim
            if self.state[i] == 0:
                #print('Found empty slot on: (' + str(row) + ',' + str(col) + ')')
                self.emptyAtIndex = i
                break
            i += 1

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
            return tuple(newState)
        if action == 'Down':
            targetCellIndex = self.emptyAtIndex + self.dim
            targetCellValue = self.state[targetCellIndex]
            newState[self.emptyAtIndex] = targetCellValue
            newState[targetCellIndex] = 0
            return tuple(newState)
        if action == 'Left':
            targetCellIndex = self.emptyAtIndex - 1
            targetCellValue = self.state[targetCellIndex]
            newState[self.emptyAtIndex] = targetCellValue
            newState[targetCellIndex] = 0
            return tuple(newState)
        if action == 'Right':
            targetCellIndex = self.emptyAtIndex + 1
            targetCellValue = self.state[targetCellIndex]
            newState[self.emptyAtIndex] = targetCellValue
            newState[targetCellIndex] = 0
            return tuple(newState)

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
cliState = tuple(map(int, list(sys.argv[2].split(','))))
print('running: ' + cliMethod + ' on :', cliState)
solver = Solver(cliMethod, cliState)
