""" ColumbiaX: CSMM.101x Artificial Intelligence (AI) 2017
Project 1: search algorithms
Author: Eduardo Cruz, eduardo.gz@gmail.com """

import sys
from solver import Solver


# Entry point
CLI_METHOD = sys.argv[1]
CLI_STATE = tuple(map(int, list(sys.argv[2].split(','))))
print('running: ' + CLI_METHOD + ' on :', CLI_STATE)
SOLVER = Solver(CLI_METHOD, CLI_STATE)
