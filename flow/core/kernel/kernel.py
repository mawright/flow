"""Script containing the Flow kernel object for interacting with simulators."""
from flow.core.kernel.simulation import TraCISimulation
# from flow.core.kernel.scenario import TraCIScenario
from flow.core.kernel.vehicle import TraCIVehicle
# from flow.core.kernel.traffic_light import TraCITrafficLight


class Kernel(object):
    """Kernel for abstract function calling across traffic simulator APIs.

    The kernel contains four different subclasses for distinguishing between
    the various components of a traffic simulator.

    - simulation: controls starting, loading, saving, advancing, and resetting
      a simulation in Flow (see flow/core/kernel/simulation/base.py)
    - scenario: stores network-specific information (see
      flow/core/kernel/scenario/base.py)
    - vehicle: stores and regularly updates vehicle-specific information. At
      times, this class is optimized to efficiently collect information from
      the simulator (see flow/core/kernel/vehicle/base.py).
    - traffic_light: stores and regularly updates traffic light-specific
      information (see flow/core/kernel/traffic_light/base.py).

    The above kernel subclasses are designed specifically to support
    simulator-agnostic state information calling. For example, if you would
    like to collect the vehicle speed of a specific vehicle, then simply type:

    >>> k = Kernel(simulator="...")  # a kernel for some simulator type
    >>> veh_id = "..."  # some vehicle ID
    >>> speed = k.vehicle.get_speed(veh_id)

    In addition, these subclasses support sending commands to the simulator via
    its API. For example, in order to assign a specific vehicle a target
    acceleration, type:

    >>> k = Kernel(simulator="...")  # a kernel for some simulator type
    >>> veh_id = "..."  # some vehicle ID
    >>> k.vehicle.apply_acceleration(veh_id)

    These subclasses can be modified and recycled to support various different
    traffic simulators, e.g. SUMO, AIMSUN, TruckSim, etc...
    """

    def __init__(self,
                 simulator,
                 kernel_api,
                 sim_params,
                 scenario,
                 vehicles,
                 traffic_lights):
        """Instantiate a Flow kernel object.

        Parameters
        ----------
        simulator : str
            simulator type, must be one of {"traci"}

        Raises
        ------
        ValueError
            if the specified input simulator is not a valid type
        """
        if simulator == "traci":
            self.simulation = TraCISimulation(self, kernel_api)
            # self.scenario = TraCIScenario(self, kernel_api, scenario)
            self.scenario = scenario
            self.vehicle = TraCIVehicle(self, kernel_api, sim_params, vehicles)
            # self.traffic_light = TraCITrafficLight(self, kernel_api,
            #                                        traffic_lights)
            self.traffic_light = traffic_lights
        else:
            raise ValueError('Simulator type "{}" is not valid.'.
                             format(simulator))

    def update(self, reset):
        """Update the kernel subclasses with after a simulation step.

        This is meant to support optimizations in the performance of some
        simulators. For example, this step allows the vehicle subclass in the
        "traci" simulator uses the ``update`` method to collect and store
        subscription information.
        """
        # self.scenario.update()
        # self.simulation.update()
        self.vehicle.update(reset)
        # self.traffic_light.update()
