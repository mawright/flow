from flow.core.kernel.simulation import KernelSimulation
import traci.constants as tc


class TraCISimulation(KernelSimulation):
    """

    """

    def __init__(self, master_kernel, kernel_api):
        """

        """
        KernelSimulation.__init__(self, master_kernel, kernel_api)

        # subscribe some simulation parameters needed to check for entering,
        # exiting, and colliding vehicles
        self.kernel_api.simulation.subscribe([
            tc.VAR_DEPARTED_VEHICLES_IDS, tc.VAR_ARRIVED_VEHICLES_IDS,
            tc.VAR_TELEPORT_STARTING_VEHICLES_IDS
        ])

    def simulation_step(self):
        """See parent class."""
        self.kernel_api.simulationStep()

    def update(self):
        """See parent class."""
        pass
