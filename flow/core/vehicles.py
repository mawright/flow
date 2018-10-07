"""Contains the vehicles class."""

from flow.controllers.car_following_models import SumoCarFollowingController
from flow.controllers.rlcontroller import RLController
from flow.controllers.lane_change_controllers import SumoLaneChangeController
from flow.core.params import SumoCarFollowingParams, SumoLaneChangeParams
import collections


class Vehicles:
    """Base vehicle class.

    This is used to describe the state of all vehicles in the network.
    State information on the vehicles for a given time step can be set or
    retrieved from this class.
    """

    def __init__(self):
        """Instantiate the base vehicle class."""
        self.ids = []  # ids of all vehicles
        self.__human_ids = []  # ids of human-driven vehicles
        self.__controlled_ids = []  # ids of flow-controlled vehicles
        self.__controlled_lc_ids = []  # ids of flow lc-controlled vehicles
        self.__rl_ids = []  # ids of rl-controlled vehicles
        self.__observed_ids = []  # ids of the observed vehicles

        # vehicles: Key = Vehicle ID, Value = Dictionary describing the vehicle
        # Ordered dictionary used to keep neural net inputs in order
        self.__vehicles = collections.OrderedDict()

        # create a sumo_observations variable that will carry all information
        # on the state of the vehicles for a given time step
        self.__sumo_obs = None

        self.num_vehicles = 0  # total number of vehicles in the network
        self.num_rl_vehicles = 0  # number of rl vehicles in the network
        self.num_types = 0  # number of unique types of vehicles in the network
        self.types = []  # types of vehicles in the network

        # contains the parameters associated with each type of vehicle
        self.type_parameters = dict()

        # contain the minGap attribute of each type of vehicle
        self.minGap = dict()

        # list of vehicle ids located in each edge in the network
        self._ids_by_edge = dict()

        # number of vehicles that entered the network for every time-step
        self._num_departed = []

        # number of vehicles to exit the network for every time-step
        self._num_arrived = []

        # simulation step size
        self.sim_step = 0

        # initial state of the vehicles class, used for serialization purposes
        self.initial = []

    def add(self,
            veh_id,
            acceleration_controller=(SumoCarFollowingController, {}),
            lane_change_controller=(SumoLaneChangeController, {}),
            routing_controller=None,
            num_vehicles=1,
            sumo_car_following_params=None,
            sumo_lc_params=None):
        """Add a sequence of vehicles to the list of vehicles in the network.

        Parameters
        ----------
        veh_id : str
            base vehicle ID for the vehicles (will be appended by a number)
        acceleration_controller : tup, optional
            1st element: flow-specified acceleration controller
            2nd element: controller parameters (may be set to None to maintain
            default parameters)
        lane_change_controller : tup, optional
            1st element: flow-specified lane-changer controller
            2nd element: controller parameters (may be set to None to maintain
            default parameters)
        routing_controller : tup, optional
            1st element: flow-specified routing controller
            2nd element: controller parameters (may be set to None to maintain
            default parameters)
        num_vehicles : int, optional
            number of vehicles of this type to be added to the network
        sumo_car_following_params : flow.core.params.SumoCarFollowingParams
            Params object specifying attributes for Sumo car following model.
        sumo_lc_params : flow.core.params.SumoLaneChangeParams
            Params object specifying attributes for Sumo lane changing model.
        """
        if sumo_car_following_params is None:
            sumo_car_following_params = SumoCarFollowingParams()

        if sumo_lc_params is None:
            sumo_lc_params = SumoLaneChangeParams()

        type_params = {}
        type_params.update(sumo_car_following_params.controller_params)
        type_params.update(sumo_lc_params.controller_params)

        # If a vehicle is not sumo or RL, let the minGap be zero so that it
        # does not tamper with the dynamics of the controller
        if acceleration_controller[0] != SumoCarFollowingController \
                and acceleration_controller[0] != RLController:
            type_params["minGap"] = 0.0

        # this dict will be used when trying to introduce new vehicles into
        # the network via a flow
        self.type_parameters[veh_id] = \
            {"acceleration_controller": acceleration_controller,
             "lane_change_controller": lane_change_controller,
             "routing_controller": routing_controller,
             "sumo_car_following_params": sumo_car_following_params,
             "sumo_lc_params": sumo_lc_params}

        self.initial.append({
            "veh_id":
                veh_id,
            "acceleration_controller":
                acceleration_controller,
            "lane_change_controller":
                lane_change_controller,
            "routing_controller":
                routing_controller,
            "num_vehicles":
                num_vehicles,
            "sumo_car_following_params":
                sumo_car_following_params,
            "sumo_lc_params":
                sumo_lc_params
        })

        # this is used to return the actual headways from the vehicles class
        self.minGap[veh_id] = type_params["minGap"]

        for i in range(num_vehicles):
            v_id = veh_id + '_%d' % i

            # add the vehicle to the list of vehicle ids
            self.ids.append(v_id)

            self.__vehicles[v_id] = dict()

            # specify the type
            self.__vehicles[v_id]["type"] = veh_id

            # specify the acceleration controller class
            self.__vehicles[v_id]["acc_controller"] = \
                acceleration_controller[0](
                    v_id,
                    sumo_cf_params=sumo_car_following_params,
                    **acceleration_controller[1])

            # specify the lane-changing controller class
            self.__vehicles[v_id]["lane_changer"] = \
                lane_change_controller[0](veh_id=v_id,
                                          **lane_change_controller[1])

            # specify the routing controller class
            if routing_controller is not None:
                self.__vehicles[v_id]["router"] = \
                    routing_controller[0](veh_id=v_id,
                                          router_params=routing_controller[1])
            else:
                self.__vehicles[v_id]["router"] = None

            # check if the vehicle is human-driven or autonomous
            if acceleration_controller[0] == RLController:
                self.__rl_ids.append(v_id)
            else:
                self.__human_ids.append(v_id)

                # check if the vehicle's lane-changing / acceleration actions
                # are controlled by sumo or not.
                if acceleration_controller[0] != SumoCarFollowingController:
                    self.__controlled_ids.append(v_id)
                if lane_change_controller[0] != SumoLaneChangeController:
                    self.__controlled_lc_ids.append(v_id)

        # update the variables for the number of vehicles in the network
        self.num_vehicles = len(self.ids)
        self.num_rl_vehicles = len(self.__rl_ids)

        # increase the number of unique types of vehicles in the network, and
        # add the type to the list of types
        self.num_types += 1
        self.types.append({"veh_id": veh_id, "type_params": type_params})

    def get_type(self, veh_id):
        return self.__vehicles[veh_id]["type"]
