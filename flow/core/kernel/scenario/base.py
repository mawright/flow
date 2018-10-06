"""

"""


class KernelScenario(object):
    """

    """

    def __init__(self):
        """
        
        """
        pass

    def update(self):
        """

        :return:
        """
        raise NotImplementedError

    def edge_length(self, edge_id):
        """Return the length of a given edge/junction.

        Return -1001 if edge not found.
        """
        raise NotImplementedError

    def speed_limit(self, edge_id):
        """Return the speed limit of a given edge/junction.

        Return -1001 if edge not found.
        """
        raise NotImplementedError

    def num_lanes(self, edge_id):
        """Return the number of lanes of a given edge/junction.

        Return -1001 if edge not found.
        """
        raise NotImplementedError

    def get_edge_list(self):
        """Return the names of all edges in the network."""
        raise NotImplementedError

    def get_junction_list(self):
        """Return the names of all junctions in the network."""
        raise NotImplementedError

    def max_speed(self):
        """

        :return:
        """
        raise NotImplementedError
