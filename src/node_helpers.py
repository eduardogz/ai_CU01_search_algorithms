""" helper functions to work with nodes."""


def create_node(state_parent: tuple, action: str, cost: int, depth: int):
    """ Returns a tuple with the node information to be saved in node_db with the node state as key"""
    return tuple(list((state_parent, action, cost, depth)))
