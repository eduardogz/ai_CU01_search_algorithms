""" ColumbiaX: CSMM.101x Artificial Intelligence (AI) 2017
Project 1: search algorithms
Author: Eduardo Cruz, eduardo.gz@gmail.com """

import sys
from collections import deque
from time import time
from board import Board
from profiler import Profiler


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
        init_node = create_node(None, '', 1, 0)  # root node
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

                        self.node_db[new_state] = create_node(
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
        init_node = create_node(None, '', 1, 0)  # root node
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

                        self.node_db[new_state] = create_node(
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
        init_node = create_node(None, '', 1, 0)  # root node
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

                        self.node_db[new_state] = create_node(
                            state_from_fringe, action, 1, self.node_db[state_from_fringe][3] + 1)
                        self.fringe.append(new_state)

                        self.profiler.update_max_fringe_size(len(self.fringe))
                        self.profiler.update_max_search_depth(
                            self.node_db[new_state][3])

            # print('*** END ***\n\n')
            # print(str(self.profiler.nodes_expanded))

        self.profiler.running_time_end = time()
        return False


def create_node(state_parent: tuple, action: str, cost: int, depth: int):
    """ Returns a tuple with the node information to be saved in node_db with the node state as key"""
    return tuple(list((state_parent, action, cost, depth)))

# Entry point
CLI_METHOD = sys.argv[1]
CLI_STATE = tuple(map(int, list(sys.argv[2].split(','))))
print('running: ' + CLI_METHOD + ' on :', CLI_STATE)
SOLVER = Solver(CLI_METHOD, CLI_STATE)
