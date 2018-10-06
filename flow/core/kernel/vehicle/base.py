

class KernelVehicle(object):
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

    def get_ids(self):
        """Return the names of all vehicles currently in the network."""
        raise NotImplementedError

    def get_human_ids(self):
        """Return the names of all non-rl vehicles currently in the network."""
        raise NotImplementedError

    def get_controlled_ids(self):
        """Return the names of all flow acceleration-controlled vehicles."""
        raise NotImplementedError

    def get_controlled_lc_ids(self):
        """Return the names of all flow lane change-controlled vehicles."""
        raise NotImplementedError

    def get_rl_ids(self):
        """Return the names of all rl-controlled vehicles in the network."""
        raise NotImplementedError

    def get_ids_by_edge(self, edges):
        """Return the names of all vehicles in the specified edge.

        If no vehicles are currently in the edge, then returns an empty list.
        """
        raise NotImplementedError

    def get_inflow_rate(self, time_span):
        """Return the inflow rate (in veh/hr) of vehicles from the network.

        This value is computed over the specified **time_span** seconds.
        """
        raise NotImplementedError

    def get_outflow_rate(self, time_span):
        """Return the outflow rate (in veh/hr) of vehicles from the network.

        This value is computed over the specified **time_span** seconds.
        """
        raise NotImplementedError

    def get_num_arrived(self):
        """Return the number of vehicles that arrived in the last time step."""
        raise NotImplementedError

    def get_speed(self, veh_id, error=-1001):
        """Return the speed of the specified vehicle.

        Parameters
        ----------
        veh_id : str or list<str>
            vehicle id, or list of vehicle ids
        error : any, optional
            value that is returned if the vehicle is not found

        Returns
        -------
        float
        """
        raise NotImplementedError

    def get_absolute_position(self, veh_id, error=-1001):
        """Return the absolute position of the specified vehicle.

        Parameters
        ----------
        veh_id : str or list<str>
            vehicle id, or list of vehicle ids
        error : any, optional
            value that is returned if the vehicle is not found

        Returns
        -------
        float
        """
        raise NotImplementedError

    def get_position(self, veh_id, error=-1001):
        """Return the position of the vehicle relative to its current edge.

        Parameters
        ----------
        veh_id : str or list<str>
            vehicle id, or list of vehicle ids
        error : any, optional
            value that is returned if the vehicle is not found

        Returns
        -------
        float
        """
        raise NotImplementedError

    def get_edge(self, veh_id, error=""):
        """Return the edge the specified vehicle is currently on.

        Parameters
        ----------
        veh_id : str or list<str>
            vehicle id, or list of vehicle ids
        error : any, optional
            value that is returned if the vehicle is not found

        Returns
        -------
        str
        """
        raise NotImplementedError

    def get_lane(self, veh_id, error=-1001):
        """Return the lane index of the specified vehicle.

        Parameters
        ----------
        veh_id : str or list<str>
            vehicle id, or list of vehicle ids
        error : any, optional
            value that is returned if the vehicle is not found

        Returns
        -------
        int
        """
        raise NotImplementedError

    def get_length(self, veh_id, error=-1001):
        """Return the length of the specified vehicle.

        Parameters
        ----------
        veh_id : str or list<str>
            vehicle id, or list of vehicle ids
        error : any, optional
            value that is returned if the vehicle is not found

        Returns
        -------
        float
        """
        raise NotImplementedError

    def get_route(self, veh_id, error=list()):
        """Return the route of the specified vehicle.

        Parameters
        ----------
        veh_id : str or list<str>
            vehicle id, or list of vehicle ids
        error : any, optional
            value that is returned if the vehicle is not found

        Returns
        -------
        list<str>
        """
        raise NotImplementedError

    def get_leader(self, veh_id, error=""):
        """Return the leader of the specified vehicle.

        Parameters
        ----------
        veh_id : str or list<str>
            vehicle id, or list of vehicle ids
        error : any, optional
            value that is returned if the vehicle is not found

        Returns
        -------
        str
        """
        raise NotImplementedError

    def get_follower(self, veh_id, error=""):
        """Return the follower of the specified vehicle.

        Parameters
        ----------
        veh_id : str or list<str>
            vehicle id, or list of vehicle ids
        error : any, optional
            value that is returned if the vehicle is not found

        Returns
        -------
        str
        """
        raise NotImplementedError

    def get_headway(self, veh_id, error=-1001):
        """Return the headway of the specified vehicle(s).

        Parameters
        ----------
        veh_id : str or list<str>
            vehicle id, or list of vehicle ids
        error : any, optional
            value that is returned if the vehicle is not found

        Returns
        -------
        float
        """
        raise NotImplementedError

    def get_lane_headways(self, veh_id, error=list()):
        """Return the lane headways of the specified vehicles.

        This includes the headways between the specified vehicle and the
        vehicle immediately ahead of it in all lanes.

        Parameters
        ----------
        veh_id : str or list<str>
            vehicle id, or list of vehicle ids
        error : any, optional
            value that is returned if the vehicle is not found

        Returns
        -------
        list<float>
        """
        raise NotImplementedError

    def get_lane_leaders(self, veh_id, error=list()):
        """Return the leaders for the specified vehicle in all lanes.

        Parameters
        ----------
        veh_id : str or list<str>
            vehicle id, or list of vehicle ids
        error : any, optional
            value that is returned if the vehicle is not found

        Returns
        -------
        list<float>
        """
        raise NotImplementedError

    def get_lane_tailways(self, veh_id, error=list()):
        """Return the lane tailways of the specified vehicle.

        This includes the headways between the specified vehicle and the
        vehicle immediately behind it in all lanes.

        Parameters
        ----------
        veh_id : str or list<str>
            vehicle id, or list of vehicle ids
        error : any, optional
            value that is returned if the vehicle is not found

        Returns
        -------
        list<float>
        """
        raise NotImplementedError

    def get_lane_followers(self, veh_id, error=list()):
        """Return the followers for the specified vehicle in all lanes.

        Parameters
        ----------
        veh_id : str or list<str>
            vehicle id, or list of vehicle ids
        error : list, optional
            value that is returned if the vehicle is not found

        Returns
        -------
        list<str>
        """
        raise NotImplementedError
