

class KernelSimulation(object):
    """

    """

    def __init__(self, master_kernel, kernel_api):
        """Initialize the simulation kernel.

        Parameters
        ----------
        master_kernel : flow.core.kernel.Kernel
            the higher level kernel (used to call methods from other
            sub-kernels)
        kernel_api : any
            an API that may be used to interact with the simulator
        """
        self.master_kernel = master_kernel
        self.kernel_api = kernel_api

    def simulation_step(self):
        """Advance the simulation by one step.

        This is done in most cases by calling a relevant simulator API method.
        """
        raise NotImplementedError

    def update(self, reset):
        """Update the internal attributes of the simulation kernel.

        Any update operations are meant to support ease of simulation in
        current and future steps.

        Parameters
        ----------
        reset : bool
            specifies whether the simulator was reset in the last simulation
            step
        """
        raise NotImplementedError

    def start_simulation(self, network, sim_params):
        """Start a simulation instance.

        network : any
            an object or variable that is meant to symbolize the network that
            is used during the simulation. For example, in the case of sumo
            simulations, this is (string) the path to the .sumo.cfg file.
        sim_params : flow.core.params.SumoParams  # FIXME: make ambiguous
            simulation-specific parameters
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
        """
        raise NotImplementedError

    def save_simulation(self, file_path):
        """Save the network configuration of a simulation.

        This can later be loaded (see ``load_simulation``).

        Parameters
        ----------
        file_path : str
            location to store simulation parameters
        """
        raise NotImplementedError
