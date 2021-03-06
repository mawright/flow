"""Script containing the base scenario kernel class."""
from copy import deepcopy
import flow.config as config
import json
import subprocess
import os.path as osp
import os
import time
from flow.core.kernel.scenario.base import KernelScenario

# length of vehicles in the network, in meters
VEHICLE_LENGTH = 5


class AimsunKernelScenario(KernelScenario):
    """Scenario kernel for Aimsun-based simulations.

    This class is responsible for passing features to and calling the
    "generate.py" file within flow/utils/aimsun/. All other features are
    designed to extend KernelScenario.
    """

    def __init__(self, master_kernel, sim_params):
        """See parent class."""
        KernelScenario.__init__(self, master_kernel, sim_params)

        self.kernel_api = None
        self.network = None
        self._edges = None
        self._edge_list = None
        self._junction_list = None
        self.__max_speed = None
        self.__length = None
        self.rts = None
        self._edge_flow2aimsun = {}
        self._edge_aimsun2flow = {}
        self.aimsun_proc = None

    def generate_network(self, scenario):
        self.network = scenario

        output = {
            "edges": scenario.edges,
            "nodes": scenario.nodes,
            "types": scenario.types,
            "connections": scenario.connections,
            "inflows": None,
            "vehicle_types": scenario.vehicles.types,
            "osm_path": scenario.net_params.osm_path,
            'render': self.sim_params.render,
            "sim_step": self.sim_params.sim_step,
            "traffic_lights": None
        }

        if scenario.net_params.inflows is not None:
            output["inflows"] = scenario.net_params.inflows.__dict__

        if scenario.traffic_lights is not None:
            output["traffic_lights"] = scenario.traffic_lights.__dict__

        cur_dir = os.path.join(config.PROJECT_PATH,
                               'flow/core/kernel/scenario')
        # TODO: add current time
        with open(os.path.join(cur_dir, 'data.json'), 'w') as outfile:
            json.dump(output, outfile, sort_keys=True, indent=4)

        # path to the Aimsun_Next binary
        aimsun_path = osp.join(osp.expanduser(config.AIMSUN_NEXT_PATH),
                               'Aimsun_Next')

        # path to the supplementary file that is used to generate an aimsun
        # network from a template
        script_path = osp.join(config.PROJECT_PATH,
                               'flow/utils/aimsun/generate.py')

        # start the aimsun process
        aimsun_call = [aimsun_path, "-script", script_path]
        self.aimsun_proc = subprocess.Popen(aimsun_call)

        # merge types into edges
        if scenario.net_params.osm_path is None:
            for i in range(len(scenario.edges)):
                if 'type' in scenario.edges[i]:
                    for typ in scenario.types:
                        if typ['id'] == scenario.edges[i]['type']:
                            new_dict = deepcopy(typ)
                            new_dict.pop("id")
                            scenario.edges[i].update(new_dict)
                            break

            self._edges = {}
            for edge in deepcopy(scenario.edges):
                edge_name = edge['id']
                self._edges[edge_name] = {}
                del edge['id']
                self._edges[edge_name] = edge

            # list of edges and internal links (junctions)
            self._edge_list = [
                edge_id for edge_id in self._edges.keys() if edge_id[0] != ':'
            ]
            self._junction_list = list(
                set(self._edges.keys()) - set(self._edge_list))
        else:
            data_file = 'flow/utils/aimsun/osm_edges.json'
            filepath = os.path.join(config.PROJECT_PATH, data_file)

            while not os.path.exists(filepath):
                time.sleep(0.5)

            with open(filepath) as f:
                self._edges = json.load(f)

            # delete the file
            os.remove(filepath)

        # maximum achievable speed on any edge in the network
        self.__max_speed = max(
            self.speed_limit(edge) for edge in self.get_edge_list())

        # length of the network, or the portion of the network in
        # which cars are meant to be distributed
        self.__length = sum(
            self.edge_length(edge_id) for edge_id in self.get_edge_list()
        )

        # parameters to be specified under each unique subclass's
        # __init__() function
        self.edgestarts = self.network.edge_starts

        # if no edge_starts are specified, generate default values to be used
        # by the "get_x" method
        if self.edgestarts is None:
            length = 0
            self.edgestarts = []
            for edge_id in sorted(self._edge_list):
                # the current edge starts where the last edge ended
                self.edgestarts.append((edge_id, length))
                # increment the total length of the network with the length of
                # the current edge
                length += self._edges[edge_id]['length']

        # these optional parameters need only be used if "no-internal-links"
        # is set to "false" while calling sumo's netconvert function
        self.internal_edgestarts = self.network.internal_edge_starts
        self.intersection_edgestarts = self.network.intersection_edge_starts

        # in case the user did not write the intersection edge-starts in
        # internal edge-starts as well (because of redundancy), merge the two
        # together
        self.internal_edgestarts += self.intersection_edgestarts
        seen = set()
        self.internal_edgestarts = \
            [item for item in self.internal_edgestarts
             if item[1] not in seen and not seen.add(item[1])]
        self.internal_edgestarts_dict = dict(self.internal_edgestarts)

        # total_edgestarts and total_edgestarts_dict contain all of the above
        # edges, with the former being ordered by position
        self.total_edgestarts = self.edgestarts + self.internal_edgestarts
        self.total_edgestarts.sort(key=lambda tup: tup[1])

        self.total_edgestarts_dict = dict(self.total_edgestarts)

        # specify routes vehicles can take  # TODO: move into a method
        self.rts = self.network.routes

    def pass_api(self, kernel_api):
        """See parent class."""
        self.kernel_api = kernel_api

        # create the edge mapping from edges names in Flow to Aimsun and vice
        # versa
        self._edge_flow2aimsun = {}
        self._edge_aimsun2flow = {}
        for edge in self.get_edge_list():
            aimsun_edge = self.kernel_api.get_edge_name(edge)
            self._edge_flow2aimsun[edge] = aimsun_edge
            self._edge_aimsun2flow[aimsun_edge] = edge

    def update(self, reset):
        """See parent class."""
        pass

    def close(self):
        """See parent class."""
        # delete the json file that was used to read the scenario data
        cur_dir = os.path.join(config.PROJECT_PATH,
                               'flow/core/kernel/scenario')
        os.remove(os.path.join(cur_dir, 'data.json'))

    ###########################################################################
    #                        State acquisition methods                        #
    ###########################################################################

    def edge_length(self, edge_id):
        """See parent class."""
        try:
            return self._edges[edge_id]["length"]
        except KeyError:
            print('Error in edge length with key', edge_id)
            return -1001

    def length(self):
        """See parent class."""
        return sum(self.edge_length(edge_id)
                   for edge_id in self.get_edge_list())

    def speed_limit(self, edge_id):
        """See parent class."""
        try:
            return self._edges[edge_id]["speed"]
        except KeyError:
            print('Error in speed limit with key', edge_id)
            return -1001

    def max_speed(self):
        """See parent class."""
        return max(
            self.speed_limit(edge) for edge in self.get_edge_list())

    def num_lanes(self, edge_id):
        """See parent class."""
        try:
            return self._edges[edge_id]["numLanes"]
        except KeyError:
            print('Error in num lanes with key', edge_id)
            return -1001

    def get_edge_list(self):
        """See parent class."""
        return [
            edge_id for edge_id in self._edges.keys() if edge_id[0] != ":"
        ]

    def get_junction_list(self):
        """See parent class."""
        return list(
            set(self._edges.keys()) - set(self._edge_list))

    def get_edge(self, x):  # TODO: maybe remove
        """See parent class."""
        for (edge, start_pos) in reversed(self.total_edgestarts):
            if x >= start_pos:
                return edge, x - start_pos

    def get_x(self, edge, position):  # TODO: maybe remove
        """See parent class."""
        # if there was a collision which caused the vehicle to disappear,
        # return an x value of -1001
        if len(edge) == 0:
            return -1001

        if edge[0] == ":" or '_to_' in edge:
            try:
                return self.internal_edgestarts_dict[edge] + position
            except KeyError:
                # in case several internal links are being generalized for
                # by a single element (for backwards compatibility)
                edge_name = edge.rsplit("_", 1)[0]
                return self.total_edgestarts_dict.get(edge_name, -1001)
        else:
            return self.total_edgestarts_dict[edge] + position

    def next_edge(self, edge, lane):
        """See parent class."""
        try:
            return self._connections["next"][edge][lane]
        except KeyError:
            return []

    def prev_edge(self, edge, lane):
        """See parent class."""
        try:
            return self._connections["prev"][edge][lane]
        except KeyError:
            return []

    def aimsun_edge_name(self, edge):
        """Returns the edge name in Aimsun."""
        return self._edge_flow2aimsun[edge]

    def flow_edge_name(self, edge):
        """Returns the edge name in Aimsun."""
        if edge not in self._edge_aimsun2flow:
            # print("aimsun edge unknown: {}".format(edge))
            return ''
        else:
            return self._edge_aimsun2flow[edge]
