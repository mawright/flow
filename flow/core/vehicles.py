"""Contains the vehicles class."""

from flow.controllers.car_following_models import SumoCarFollowingController
from flow.controllers.rlcontroller import RLController
from flow.controllers.lane_change_controllers import SumoLaneChangeController
import collections
import logging

from flow.core.params import SumoCarFollowingParams, SumoLaneChangeParams

SPEED_MODES = {
    "aggressive": 0,
    "no_collide": 1,
    "right_of_way": 25,
    "all_checks": 31
}
LC_MODES = {"aggressive": 0, "no_lat_collide": 512, "strategic": 1621}


class Vehicles:
    """Base vehicle class.

    This is used to describe the state of all vehicles in the network.
    State information on the vehicles for a given time step can be set or
    retrieved from this class.
    """

    def __init__(self):
        """Instantiate the base vehicle class."""
        self.__ids = []  # ids of all vehicles
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
        self.initial_speeds = []  # speed of vehicles at the start of a rollout

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
            initial_speed=0,
            num_vehicles=1,
            speed_mode='right_of_way',
            lane_change_mode="no_lat_collide",
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
        initial_speed : float, optional  # TODO: move to gen_start_pos
            initial speed of the vehicles being added (in m/s)
        num_vehicles : int, optional
            number of vehicles of this type to be added to the network
        speed_mode : str or int, optional  # TODO: move to sumo_cf_params
            may be one of the following:

             * "right_of_way" (default): respect safe speed, right of way and
               brake hard at red lights if needed. DOES NOT respect
               max accel and decel which enables emergency stopping.
               Necessary to prevent custom models from crashing
             * "no_collide": Human and RL cars are preventing from reaching
               speeds that may cause crashes (also serves as a failsafe).
             * "aggressive": Human and RL cars are not limited by sumo with
               regard to their accelerations, and can crash longitudinally
             * "all_checks": all sumo safety checks are activated
             * int values may be used to define custom speed mode for the given
               vehicles, specified at:
               http://sumo.dlr.de/wiki/TraCI/Change_Vehicle_State#speed_mode_.280xb3.29

        lane_change_mode : str or int, optional  # TODO: move to sumo_lc_params
            may be one of the following:

            * "no_lat_collide" (default): Human cars will not make lane
              changes, RL cars can lane change into any space, no matter how
              likely it is to crash
            * "strategic": Human cars make lane changes in accordance with SUMO
              to provide speed boosts
            * "aggressive": RL cars are not limited by sumo with regard to
              their lane-change actions, and can crash longitudinally
            * int values may be used to define custom lane change modes for the
              given vehicles, specified at:
              http://sumo.dlr.de/wiki/TraCI/Change_Vehicle_State#lane_change_mode_.280xb6.29

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

        # adjust the speed mode value
        if isinstance(speed_mode, str) and speed_mode in SPEED_MODES:
            speed_mode = SPEED_MODES[speed_mode]
        elif not (isinstance(speed_mode, int)
                  or isinstance(speed_mode, float)):
            logging.error("Setting speed mode of {0} to "
                          "default.".format(veh_id))
            speed_mode = SPEED_MODES["no_collide"]

        # adjust the lane change mode value
        if isinstance(lane_change_mode, str) and lane_change_mode in LC_MODES:
            lane_change_mode = LC_MODES[lane_change_mode]
        elif not (isinstance(lane_change_mode, int)
                  or isinstance(lane_change_mode, float)):
            logging.error("Setting lane change mode of {0} to "
                          "default.".format(veh_id))
            lane_change_mode = LC_MODES["no_lat_collide"]

        # this dict will be used when trying to introduce new vehicles into
        # the network via a flow
        self.type_parameters[veh_id] = \
            {"acceleration_controller": acceleration_controller,
             "lane_change_controller": lane_change_controller,
             "routing_controller": routing_controller,
             "initial_speed": initial_speed,
             "speed_mode": speed_mode,
             "lane_change_mode": lane_change_mode,
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
            "initial_speed":
                initial_speed,
            "num_vehicles":
                num_vehicles,
            "speed_mode":
                speed_mode,
            "lane_change_mode":
                lane_change_mode,
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
            self.__ids.append(v_id)

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

            # specify the speed of vehicles at the start of a rollout
            self.__vehicles[v_id]["initial_speed"] = initial_speed

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

            # specify the speed and lane change mode for the vehicle
            self.__vehicles[v_id]["speed_mode"] = speed_mode
            self.__vehicles[v_id]["lane_change_mode"] = lane_change_mode

        # update the variables for the number of vehicles in the network
        self.num_vehicles = len(self.__ids)
        self.num_rl_vehicles = len(self.__rl_ids)

        # increase the number of unique types of vehicles in the network, and
        # add the type to the list of types
        self.num_types += 1
        self.types.append({"veh_id": veh_id, "type_params": type_params})
