""" A borad representing an n-puzzle"""
from math import sqrt, floor


class Board:
    """ Low-level methods for the board """

    def __init__(self, state: list, action: str):
        self.state = state
        # find and store the board dimension
        self.dim = int(sqrt(len(self.state)))
        self.empty_at_index = 0
        self.possible_actions = []
        self.action = action  # the action that created this board

        self.find_possible_actions()

    def get_possible_actions(self):
        """ Simple getter"""
        return self.possible_actions

    def find_possible_actions(self):
        """ Returns a list with the legal actions for the current state """
        self.possible_actions = []
        i = 0
        col = 0
        row = 0
        # finds row and col where the zero value (empty tile) is at:
        while i < len(self.state):
            row = floor(i / self.dim)
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
        if action not in self.possible_actions:
            print('Board.do_action: requested action "' + action + '" NOT possible, ignoring.')
            return tuple(list(self.state))

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

        print('Board.do_action: UNKNOWN action "' + action + '"!, aborting.')
        raise ValueError

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
