""" Keeps track of program execution."""
import os  # to detect windows and omit memory measurement as ru_maxrss doesn't work on Windows

if os.name != 'nt':
    # pylint: disable=E0401
    import resource


class Profiler:
    """ Class to save and output statistics """
    # pylint: disable=too-many-instance-attributes

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
