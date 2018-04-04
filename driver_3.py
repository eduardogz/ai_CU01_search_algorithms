""" ColumbiaX: CSMM.101x Artificial Intelligence (AI) 2017
Project 1: search algorithms
Author: Eduardo Cruz, eduardo.gz@gmail.com """

import sys
import math
import os  # to detect windows and omit memory measurement as ru_maxrss doesn't work on Windows
from collections import deque
from time import time

if os.name != 'nt':
    import resource


class Solver:
    """ Main class to select method and hold common data structures """

    def __init__(self, method: str, init_state: tuple):
        super(Solver, self).__init__()
        self.init_state = init_state

        # saves states tuples, uses a double ended queue, can behave both as a
        # FIFO and as a LIFO
        self.fringe = deque()
        # saves states tuples; uses a set for speed when checking if a state
        # has been explored
        self.explored = set()
        # saves nodes details for every node state: nodeState -> nodeParent,
        # action, cost, depth
        self.node_db = dict()

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

    def create_node(self, state_parent: tuple, action: str, cost: int, depth: int):
        """ Returns a tuple with the node information to be saved in node_db with the node state as key"""
        return tuple(list((state_parent, action, cost, depth)))

    def set_goal(self):
        """  Generates the goal state, a tuple  from 0 to the size of provided init_state; and gets its ID """
        state_goal = list()
        length = int(len(self.init_state))
        i = 0
        for i in range(length):
            state_goal.append(i)
        self.state_goal = tuple(state_goal)  # save the list as a tuple
        # print('state_goal: ', self.state_goal)

    def path_to_goal(self, state):
        """ Calculates the path to root and reverses it to provide the path to goal, also prepares profiler stats """
        solution = list()
        while self.node_db[state][0] is not None:  # root node has None as parent
            solution.append(self.node_db[state][1])
            # start with the node parent, because when solution is found
            # node[0] has the goal state
            state = self.node_db[state][0]
            self.profiler.cost_of_path = self.profiler.cost_of_path + \
                self.node_db[state][2]
            self.profiler.search_depth += 1
        solution.reverse()
        self.profiler.path_to_goal = solution
        self.profiler.fringe_size = len(self.fringe)

    def bfs(self):
        """ Breadth First Search (BFS) """
        self.profiler.running_time_start = time()
        init_node = self.create_node(None, '', 1, 0)  # root node
        self.node_db[self.init_state] = init_node
        self.fringe.append(self.init_state)

        while self.fringe:  # while fringe not empty
            # print('*** START ***')
            state_from_fringe = self.fringe.popleft()  # queue

            self.explored.add(state_from_fringe)
            # print('add:', str(nodeFromFringe), 'len(explored): ', len(self.explored))

            if state_from_fringe == self.state_goal:
                # print('found it! ')
                self.path_to_goal(state_from_fringe)
                self.profiler.running_time_end = time()
                self.profiler.write_file()
                return True

            else:
                # print('exploring node:', str(nodeFromFringe))
                self.profiler.nodes_expanded += 1
                board = Board(state_from_fringe, self.node_db[state_from_fringe][1])
                # print(board.pretty_out())
                possible_actions = board.get_possible_actions()
                for action in possible_actions:
                    # print('opening action: ', action)
                    new_state = board.do_action(action)
                    # print('new node', str(new_state), ', add to fringe: ', len(self.fringe))

                    if (new_state not in self.fringe) and (new_state not in self.explored):

                        self.node_db[new_state] = self.create_node(
                            state_from_fringe, action, 1, self.node_db[state_from_fringe][3] + 1)
                        self.fringe.append(new_state)

                        self.profiler.update_max_fringe_size(len(self.fringe))
                        self.profiler.update_max_search_depth(
                            self.node_db[new_state][3])

            # print('*** END ***\n\n')
            # print(str(self.profiler.nodes_expanded))

        self.profiler.running_time_end = time()
        return False

    def dfs(self):
        """ Depth First Search (DFS) """
        self.profiler.running_time_start = time()
        init_node = self.create_node(None, '', 1, 0)  # root node
        self.node_db[self.init_state] = init_node
        self.fringe.append(self.init_state)

        while self.fringe:  # while fringe not empty
            # print('*** START ***')
            state_from_fringe = self.fringe.pop()  # stack

            self.explored.add(state_from_fringe)
            # print('add:', str(nodeFromFringe), 'len(explored): ', len(self.explored))

            if state_from_fringe == self.state_goal:
                # print('found it! ')
                self.path_to_goal(state_from_fringe)
                self.profiler.running_time_end = time()
                self.profiler.write_file()
                return True

            else:
                # print('exploring node:', str(nodeFromFringe))
                self.profiler.nodes_expanded += 1
                board = Board(state_from_fringe, self.node_db[state_from_fringe][1])
                # print(board.pretty_out())
                possible_actions = board.get_possible_actions()
                possible_actions.reverse()  # reverse-UDLR order
                for action in possible_actions:
                    # print('opening action: ', action)
                    new_state = board.do_action(action)
                    # print('new node', str(new_state), ', add to fringe: ', len(self.fringe))

                    if (new_state not in self.fringe) and (new_state not in self.explored):

                        self.node_db[new_state] = self.create_node(
                            state_from_fringe, action, 1, self.node_db[state_from_fringe][3] + 1)
                        self.fringe.append(new_state)

                        self.profiler.update_max_fringe_size(len(self.fringe))
                        self.profiler.update_max_search_depth(
                            self.node_db[new_state][3])

            # print('*** END ***\n\n')
            # print(str(self.profiler.nodes_expanded))

        self.profiler.running_time_end = time()
        return False

    def ast(self):
        """ A-Star (A*) """
        self.profiler.running_time_start = time()
        init_node = self.create_node(None, '', 1, 0)  # root node
        self.node_db[self.init_state] = init_node
        self.fringe.append(self.init_state)

        while self.fringe:  # while fringe not empty
            # print('*** START ***')
            state_from_fringe = self.fringe.popleft()  # queue

            self.explored.add(state_from_fringe)
            # print('add:', str(nodeFromFringe), 'len(explored): ', len(self.explored))

            if state_from_fringe == self.state_goal:
                # print('found it! ')
                self.path_to_goal(state_from_fringe)
                self.profiler.running_time_end = time()
                self.profiler.write_file()
                return True

            else:
                # print('exploring node:', str(nodeFromFringe))
                self.profiler.nodes_expanded += 1
                board = Board(state_from_fringe, self.node_db[state_from_fringe][1])
                # print(board.pretty_out())
                possible_actions = board.get_possible_actions()
                for action in possible_actions:
                    # print('opening action: ', action)
                    new_state = board.do_action(action)
                    # print('new node', str(new_state), ', add to fringe: ', len(self.fringe))

                    if (new_state not in self.fringe) and (new_state not in self.explored):

                        self.node_db[new_state] = self.create_node(
                            state_from_fringe, action, 1, self.node_db[state_from_fringe][3] + 1)
                        self.fringe.append(new_state)

                        self.profiler.update_max_fringe_size(len(self.fringe))
                        self.profiler.update_max_search_depth(
                            self.node_db[new_state][3])

            # print('*** END ***\n\n')
            # print(str(self.profiler.nodes_expanded))

        self.profiler.running_time_end = time()
        return False


class Profiler:
    """ Class to save and output statistics """

    def __init__(self):
        super(Profiler, self).__init__()
        self.path_to_goal = 0
        self.cost_of_path = 0
        self.nodes_expanded = 0
        self.fringe_size = 0
        self.max_fringe_size = 0
        self.search_depth = 0
        self.max_search_depth = 0
        self.running_time_start = 0
        self.running_time_end = 0

    def update_max_fringe_size(self, fringe_len):
        """ Updates max_fringe_size if necessary"""
        if self.max_fringe_size < fringe_len:
            self.max_fringe_size = fringe_len

    def update_max_search_depth(self, depth):
        """ Updates max_search_depth if necessary"""
        if self.max_search_depth < depth:
            self.max_search_depth = depth

    def write_file(self):
        """ Writes the required output.txt file"""

        running_time = str(self.running_time_end - self.running_time_start)
        rounded_running_time = '{:.10}'.format(running_time)
        output = 'path_to_goal: ' + str(self.path_to_goal) + '\n'
        output += 'cost_of_path: ' + str(self.cost_of_path) + '\n'
        output += 'nodes_expanded: ' + str(self.nodes_expanded) + '\n'
        output += 'fringe_size: ' + str(self.fringe_size) + '\n'
        output += 'max_fringe_size: ' + str(self.max_fringe_size) + '\n'
        output += 'search_depth: ' + str(self.search_depth) + '\n'
        output += 'max_search_depth: ' + str(self.max_search_depth) + '\n'
        output += 'running_time: ' + rounded_running_time + '\n'

        if os.name == 'nt':
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
        self.empty_at_index = 0
        self.possible_actions = []
        self.action = action  # the action that created this board

        self.find_possible_actions()

    def get_possible_actions(self):
        """ Simple getter"""
        return self.possible_actions

    def find_possible_actions(self):
        """ returns a list with the legal actions for the current state """
        self.possible_actions = []
        i = 0
        col = 0
        row = 0
        # finds row and col where the zero value (empty tile) is at:
        while i < len(self.state):
            row = math.floor(i / self.dim)
            col = i % self.dim
            if self.state[i] == 0:
                # print('Found empty slot on: (' + str(row) + ',' + str(col) + ')')
                self.empty_at_index = i
                break
            i += 1

        # determine possible actions based on row and col
        # visit child nodes in the "UDLR" order
        # AND EXCLUDE the inverse action that created the board
        if row > 0 and self.action != 'Down':
            self.possible_actions.append('Up')
        if row < self.dim - 1 and self.action != 'Up':
            self.possible_actions.append('Down')
        if col > 0 and self.action != 'Right':
            self.possible_actions.append('Left')
        if col < self.dim - 1 and self.action != 'Left':
            self.possible_actions.append('Right')

    def do_action(self, action):
        """ Execute an action to current state and return the resulting new state """
        try:
            self.possible_actions.index(action)
        except ValueError:
            print('Board.do_action: requested action "' + action + '" NOT possible, ignoring request.')
            return None

        new_state = list(self.state)

        if action == 'Up':
            target_cell_index = self.empty_at_index - self.dim
            target_cell_value = self.state[target_cell_index]
            new_state[self.empty_at_index] = target_cell_value
            new_state[target_cell_index] = 0
            return tuple(new_state)
        elif action == 'Down':
            target_cell_index = self.empty_at_index + self.dim
            target_cell_value = self.state[target_cell_index]
            new_state[self.empty_at_index] = target_cell_value
            new_state[target_cell_index] = 0
            return tuple(new_state)
        elif action == 'Left':
            target_cell_index = self.empty_at_index - 1
            target_cell_value = self.state[target_cell_index]
            new_state[self.empty_at_index] = target_cell_value
            new_state[target_cell_index] = 0
            return tuple(new_state)
        elif action == 'Right':
            target_cell_index = self.empty_at_index + 1
            target_cell_value = self.state[target_cell_index]
            new_state[self.empty_at_index] = target_cell_value
            new_state[target_cell_index] = 0
            return tuple(new_state)

    def pretty_out(self):
        """ A prettier output for debugging """
        state_len = len(self.state)
        pretty_row = ''
        pretty_out = '\n'
        for i in range(state_len):
            pretty_row += str(self.state[i]) + ' '
            if (i + 1) % self.dim == 0:
                pretty_out += pretty_row + '\n'
                pretty_row = ''
        return pretty_out

# Entry point
CLI_METHOD = sys.argv[1]
CLI_STATE = tuple(map(int, list(sys.argv[2].split(','))))
print('running: ' + CLI_METHOD + ' on :', CLI_STATE)
SOLVER = Solver(CLI_METHOD, CLI_STATE)
