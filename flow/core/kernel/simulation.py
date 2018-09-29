

class KernelSimulation(object):
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

    def start_simulation(self):
        """Start a simulation instance.

        :return:
        """
        raise NotImplementedError

    def load_simulation(self, file_path):
        """Load a saved network configuration into the simulation instance.

        This includes the starting position and speeds of vehicles on a
        network, as well the geometry of the network and the location of
        traffic lights.

        Parameters
        ----------
        file_path : str
            location of the store simulation parameters

        Returns
        -------
        nothing
        """
        raise NotImplementedError

    def save_simulation(self, file_path):
        """Save the network configuration of a simulation.

        This can later be loaded (see ``load_simulation``).

        Parameters
        ----------
        file_path : str
            location to store simulation parameters

        Returns
        -------
        nothing
        """
        raise NotImplementedError


class TraCISimulation(KernelSimulation):
    """

    """

    def __init__(self):
        """

        """
        super(KernelSimulation, self).__init__()
