""" ColumbiaX: CSMM.101x Artificial Intelligence (AI) 2017
Project 1: search algorithms
Author: Eduardo Cruz, eduardo.gz@gmail.com """

import sys
from solver import Solver


METHOD = sys.argv[1]
INITIAL_STATE = tuple(map(int, list(sys.argv[2].split(','))))

SOLVER = Solver(INITIAL_STATE)
SOLVER.run(METHOD)
