"""Modified merge env for attention test"""
import numpy as np
from gym.spaces import Box, Dict, Discrete

from flow.core import rewards
from flow.envs.base_env import Env
from flow.envs.merge import WaveAttenuationMergePOEnv

ADDITIONAL_ENV_PARAMS = {
    # maximum acceleration for autonomous vehicles, in m/s^2
    "max_accel": 3,
    # maximum deceleration for autonomous vehicles, in m/s^2
    "max_decel": 3,
    # desired velocity for all vehicles in the network, in m/s
    "target_velocity": 25,
    # maximum number of cars to control
    # "max_rl": 50
}


class NoneBox(Box):
    """Kludgy wrapper for gym `Box` space"""
    def __init__(self, low=None, high=None, shape=None, dtype=None):
        broadcast_shape = [s if s is not None else 1 for s in shape]
        super().__init__(low, high, broadcast_shape, dtype)

        self.shape = None if shape is None else tuple(shape)

    def sample(self):
        raise NotImplementedError

    def __repr__(self):
        return "NoneBox" + str(self.shape)


class VariableNumberVehicleMerge(WaveAttenuationMergePOEnv):
    """Modification of the built-in `WaveAttenuationMergePOEnv` class that uses
    a variable-size state and observation space rather than a fixed-size one.

    """
    def __init__(self, env_params, sim_params, scenario, simulator='traci'):
        for p in ADDITIONAL_ENV_PARAMS:
            if p not in env_params.additional_params:
                raise KeyError(
                    'Environment parameter "{}" not supplied'.format(p))

        # used for visualization
        self.leader = []
        self.follower = []

        # maximum number of cars
        # self._max_rl = env_params.additional_params['max_rl']

        Env.__init__(self, env_params, sim_params, scenario, simulator)

    @property
    def rl_veh(self):
        """List of ids of all rl vehicles in the network"""
        return self.k.vehicle.get_rl_ids()

    @property
    def num_rl(self):
        """Number of controlled vehicles currently in the network"""
        return self.k.vehicle.num_rl_vehicles

    @property
    def observation_space(self):
        """The observations for this problem are the speed and bumper-to-bumper
        headway (i.e., distance) for the preceding and subsequent vehicle, plus
        the controlled vehicle's speed, for each autonomous vehicle.

        Shape: (None, 5)
        """
        obs_space = NoneBox(low=0., high=1., shape=(None, 5),
                            dtype=np.float32)
        return obs_space
        # return Dict({"obs": obs_space,
        #              'num_rl': Discrete(self._max_rl)})

    @property
    def action_space(self):
        """The action space consists of a vector of bounded accelerations for each
        autonomous vehicle $i$. In order to ensure safety, these actions are
        bounded by failsafes provided by the simulator at every time step.

        Shape: (1,) - Placeholder dimension has size of one entry per
        conrolled car. 1 is for broadcasting.
        """
        return NoneBox(low=-abs(self.env_params.additional_params["max_decel"]),
                   high=self.env_params.additional_params["max_accel"],
                   shape=(None,),
                   dtype=np.float32)

    def _apply_rl_actions(self, rl_actions=None):
        for a, veh_id in zip(rl_actions, self.rl_veh):
            self.k.vehicle.apply_acceleration([veh_id], [a])

    def get_state(self):
        """State is two items.
        1) List of lists, outer list is of variable size (one element
        per controlled vehicle) and inner lists have five elements.
        2) Number of controlled vehicles (i.e., number of inner lists), used
        for masking out padded entries when observations are batched.
        """
        self.leader = []
        self.follower = []

        # normalizing constants
        max_speed = self.k.scenario.max_speed()
        max_length = self.k.scenario.length()

        observation = np.zeros((self.num_rl, 5))
        for rl_id, obs in zip(self.rl_veh, observation):
            this_speed = self.k.vehicle.get_speed(rl_id)
            lead_id = self.k.vehicle.get_leader(rl_id)
            follower = self.k.vehicle.get_follower(rl_id)

            if lead_id in ["", None]:
                # in case leader is not visible
                lead_speed = max_speed
                lead_head = max_length
            else:
                self.leader.append(lead_id)
                lead_speed = self.k.vehicle.get_speed(lead_id)
                lead_head = self.k.vehicle.get_x_by_id(lead_id) \
                    - self.k.vehicle.get_x_by_id(rl_id) \
                    - self.k.vehicle.get_length(rl_id)

            if follower in ["", None]:
                # in case follower is not visible
                follow_speed = 0
                follow_head = max_length
            else:
                self.follower.append(follower)
                follow_speed = self.k.vehicle.get_speed(follower)
                follow_head = self.k.vehicle.get_headway(follower)

            obs1 = this_speed / max_speed
            obs2 = (lead_speed - this_speed) / max_speed
            obs3 = lead_head / max_length
            obs4 = (this_speed - follow_speed) / max_speed
            obs5 = follow_head / max_length

            obs[:] = [obs1, obs2, obs3, obs4, obs5]

        return observation
        # return observation, self.num_rl

    # def compute_reward(self, rl_actions, **kwargs):
    #     """See class definition."""
    #     if self.env_params.evaluate:
    #         return np.mean(self.vehicles.get_speed(self.vehicles.get_ids()))
    #     else:
    #         # return a reward of 0 if a collision occurred
    #         if kwargs["fail"]:
    #             return 0

    #         # reward high system-level velocities
    #         cost1 = rewards.desired_velocity(self, fail=kwargs["fail"])

    #         # penalize small time headways
    #         cost2 = 0
    #         t_min = 1  # smallest acceptable time headway
    #         for rl_id in self.rl_veh:
    #             lead_id = self.k.vehicle.get_leader(rl_id)
    #             if lead_id not in ["", None] \
    #                     and self.k.vehicle.get_speed(rl_id) > 0:
    #                 t_headway = max(
    #                     self.k.vehicle.get_headway(rl_id) /
    #                     self.k.vehicle.get_speed(rl_id), 0)
    #                 cost2 += min((t_headway - t_min) / t_min, 0)

    #         # weights for cost1 and cost2 respectively
    #         eta1, eta2 = 1.00, 0.10

    #         return max(eta1 * cost1 + eta2 * cost2, 0)

    def sort_by_position(self):
        """See parent class.

        Sorting occurs by the ``get_x_by_id`` method instead of
        ``get_absolute_position``.
        """
        # vehicles are sorted by their get_x_by_id value
        sorted_ids = sorted(self.vehicles.get_ids(), key=self.get_x_by_id)
        return sorted_ids, None

    def clip_actions(self, rl_actions=None):
        """Clip the actions passed from the RL agent.

        Parameters
        ----------
        rl_actions : list or numpy ndarray
            list of actions provided by the RL algorithm

        Returns
        -------
        numpy ndarray (float)
            The rl_actions clipped according to the box
        """
        # ignore if no actions are issued
        if rl_actions is None:
            return None

        # clip according to the action space requirements
        if not isinstance(self.action_space, NoneBox):
            return super().clip_actions(rl_actions)

        rl_actions = np.clip(
            rl_actions,
            a_min=self.action_space.low,
            a_max=self.action_space.high)
        return rl_actions

    def additional_command(self):
        """See parent class.

        This method performs the auxiliary task:

        * Define which vehicles are observed for visualization purposes.
        * (rl_veh and buffer are not used in this version)
        """
        for veh_id in self.leader + self.follower:
            self.k.vehicle.set_observed(veh_id)
