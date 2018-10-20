

class KernelTrafficLight(object):
    """

    """

    def __init__(self, master_kernel, kernel_api):
        """

        """
        self.master_kernel = master_kernel
        self.kernel_api = kernel_api

    def update(self):
        """Update the states and phases of the traffic lights.

        This ensures that the traffic light variables match current traffic
        light data.
        """
        raise NotImplementedError

    def get_ids(self):
        """Return the names of all nodes with traffic lights."""
        raise NotImplementedError

    def set_state(self, node_id, state, link_index="all"):
        """Set the state of the traffic lights on a specific node.

        Parameters
        ----------
        node_id : str
            name of the node with the controlled traffic lights
        state : str
            desired state(s) for the traffic light
        link_index : int, optional
            index of the link whose traffic light state is meant to be changed.
            If no value is provided, the lights on all links are updated.
        """
        raise NotImplementedError

    def get_state(self, node_id):
        """Return the state of the traffic light(s) at the specified node.

        Parameters
        ----------
        node_id: str
            name of the node

        Returns
        -------
        state : str
            Index = lane index
            Element = state of the traffic light at that node/lane
        """
        raise NotImplementedError
