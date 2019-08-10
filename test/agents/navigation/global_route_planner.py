# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides GlobalRoutePlanner implementation.
"""

import math

import numpy as np
import networkx as nx

import carla
from agents.navigation.local_planner import RoadOption
from agents.tools.misc import vector


class GlobalRoutePlanner(object):
    """
    This class provides a very high level route plan.
    Instantiate the class by passing a reference to
    A GlobalRoutePlannerDAO object.
    """

    def __init__(self, dao):
        """
        Constructor
        """
        self._dao = dao
        self._topology = None
        self._graph = None
        self._id_map = None
        self._road_id_to_edge = None

    def setup(self):
        """
        Performs initial server data lookup for detailed topology
        and builds graph representation of the world map.
        """
        self._topology = self._dao.get_topology()
        self._graph, self._id_map, self._road_id_to_edge = self._build_graph()
        self._lane_change_link()

    def _build_graph(self):
        """
        This function builds a networkx  graph representation of topology.
        The topology is read from self._topology.
        graph node properties:
            vertex   -   (x,y,z) position in world map
        graph edge properties:
            entry_vector    -   unit vector along tangent at entry point
            exit_vector     -   unit vector along tangent at exit point
            net_vector      -   unit vector of the chord from entry to exit
            intersection    -   boolean indicating if the edge belongs to an
                                intersection
        return      :   graph -> networkx graph representing the world map,
                        id_map-> mapping from (x,y,z) to node id
                        road_id_to_edge-> map from road id to edge in the graph
        """
        graph = nx.DiGraph()
        id_map = dict() # Map with structure {(x,y,z): id, ... }
        road_id_to_edge = dict() # Map with structure {road_id: {lane_id: edge, ... }, ... }

        for segment in self._topology:

            entry_xyz, exit_xyz = segment['entryxyz'], segment['exitxyz']
            path = segment['path']
            entry_wp, exit_wp = segment['entry'], segment['exit']
            intersection = entry_wp.is_intersection
            road_id, lane_id = entry_wp.road_id, entry_wp.lane_id

            for vertex in entry_xyz, exit_xyz:
                # Adding unique nodes and populating id_map
                if vertex not in id_map:
                    new_id = len(id_map)
                    id_map[vertex] = new_id
                    graph.add_node(new_id, vertex=vertex)
            n1 = id_map[entry_xyz]
            n2 = id_map[exit_xyz]
            if road_id not in road_id_to_edge:
                road_id_to_edge[road_id] = dict()
            road_id_to_edge[road_id][lane_id] = (n1, n2)

            # Adding edge with attributes
            graph.add_edge(
                n1, n2,
                length=len(path) + 1, path=path,
                entry_waypoint=entry_wp, exit_waypoint=exit_wp,
                entry_vector=vector(
                    entry_wp.transform.location,
                    path[0].transform.location if len(path) > 0 else exit_wp.transform.location),
                exit_vector=vector(
                    path[-1].transform.location if len(path) > 0 else entry_wp.transform.location,
                    exit_wp.transform.location),
                net_vector=vector(entry_wp.transform.location, exit_wp.transform.location),
                intersection=intersection, type=RoadOption.LANEFOLLOW)

        return graph, id_map, road_id_to_edge

    def _localize(self, location):
        """
        This function finds the road segment closest to given location
        location        :   carla.Location to be localized in the graph
        return          :   pair node ids representing an edge in the graph
        """
        waypoint = self._dao.get_waypoint(location)
        return self._road_id_to_edge[waypoint.road_id][waypoint.lane_id]

    def _lane_change_link(self):
        """
        This method places zero cost links in the topology graph
        representing availability of lane changes.
        """

        for segment in self._topology:
            left_found, right_found = False, False

            for waypoint in segment['path']:
                if not segment['entry'].is_intersection:
                    next_waypoint, next_road_option, next_segment = None, None, None

                    if bool(waypoint.lane_change & carla.LaneChange.Right) and not right_found:
                        next_waypoint = waypoint.get_right_lane()
                        if next_waypoint is not None and next_waypoint.lane_type == carla.LaneType.Driving and \
                            waypoint.road_id == next_waypoint.road_id:
                            next_road_option = RoadOption.CHANGELANERIGHT
                            try:
                                next_segment = self._localize(next_waypoint.transform.location)
                            except KeyError:
                                print("Failed to localize! : ", next_waypoint.road_id, next_waypoint.lane_id)
                            if next_segment is not None:
                                self._graph.add_edge(
                                    self._id_map[segment['entryxyz']], next_segment[0], entry_waypoint=segment['entry'],
                                    exit_waypoint=self._graph.edges[next_segment[0], next_segment[1]]['entry_waypoint'],
                                    path=[], length=0, type=next_road_option, change_waypoint = waypoint)
                                right_found = True

                    if bool(waypoint.lane_change & carla.LaneChange.Left) and not left_found:
                        next_waypoint = waypoint.get_left_lane()
                        if next_waypoint is not None and next_waypoint.lane_type == carla.LaneType.Driving and \
                            waypoint.road_id == next_waypoint.road_id:
                            next_road_option = RoadOption.CHANGELANELEFT
                            try:
                                next_segment = self._localize(next_waypoint.transform.location)
                            except KeyError:
                                print("Failed to localize! : ", next_waypoint.road_id, next_waypoint.lane_id)
                            if next_segment is not None:
                                self._graph.add_edge(
                                    self._id_map[segment['entryxyz']], next_segment[0], entry_waypoint=segment['entry'],
                                    exit_waypoint=self._graph.edges[next_segment[0], next_segment[1]]['entry_waypoint'],
                                    path=[], length=0, type=next_road_option, change_waypoint = waypoint)
                                left_found = True

                if left_found and right_found:
                    break

    def _distance_heuristic(self, n1, n2):
        """
        Distance heuristic calculator for path searching
        in self._graph
        """
        l1 = np.array(self._graph.nodes[n1]['vertex'])
        l2 = np.array(self._graph.nodes[n2]['vertex'])
        return np.linalg.norm(l1-l2)

    def _path_search(self, origin, destination):
        """
        This function finds the shortest path connecting origin and destination
        using A* search with distance heuristic.
        origin      :   carla.Location object of start position
        destination :   carla.Location object of of end position
        return      :   path as list of node ids (as int) of the graph self._graph
        connecting origin and destination
        """

        start, end = self._localize(origin), self._localize(destination)

        route = nx.astar_path(
            self._graph, source=start[0], target=end[0],
            heuristic=self._distance_heuristic, weight='length')
        route.append(end[1])
        return route

    def _turn_decision(self, index, route, threshold=math.radians(5)):
        """
        This method returns the turn decision (RoadOption) for pair of edges
        around current index of route list
        """

        decision = None
        previous_node = route[index-1]
        current_node = route[index]
        next_node = route[index+1]
        next_edge = self._graph.edges[current_node, next_node]
        if index > 0:
            current_edge = self._graph.edges[previous_node, current_node]
            calculate_turn = current_edge['type'].value == RoadOption.LANEFOLLOW.value and \
                not current_edge['intersection'] and \
                    next_edge['type'].value == RoadOption.LANEFOLLOW.value and \
                        next_edge['intersection']
            if calculate_turn:
                cv, nv = current_edge['exit_vector'], next_edge['net_vector']
                cross_list = []
                for neighbor in self._graph.successors(current_node):
                    select_edge = self._graph.edges[current_node, neighbor]
                    if select_edge['type'].value == RoadOption.LANEFOLLOW.value:
                        if neighbor != route[index+1]:
                            sv = select_edge['net_vector']
                            cross_list.append(np.cross(cv, sv)[2])
                next_cross = np.cross(cv, nv)[2]
                deviation = math.acos(np.dot(cv, nv) /\
                    (np.linalg.norm(cv)*np.linalg.norm(nv)))
                if not cross_list:
                    cross_list.append(0)
                if deviation < threshold:
                    decision = RoadOption.STRAIGHT
                elif cross_list and next_cross < min(cross_list):
                    decision = RoadOption.LEFT
                elif cross_list and next_cross > max(cross_list):
                    decision = RoadOption.RIGHT
            else:
                decision = next_edge['type']
        else:
            decision = next_edge['type']

        return decision

    def abstract_route_plan(self, origin, destination):
        """
        The following function generates the route plan based on
        origin      : carla.Location object of the route's start position
        destination : carla.Location object of the route's end position
        return      : list of turn by turn navigation decisions as
        agents.navigation.local_planner.RoadOption elements
        Possible values are STRAIGHT, LEFT, RIGHT, LANEFOLLOW, VOID
        CHANGELANELEFT, CHANGELANERIGHT
        """

        route = self._path_search(origin, destination)
        plan = []

        for i in range(len(route) - 1):
            road_option = self._turn_decision(i, route)
            plan.append(road_option)

        return plan

    def _find_closest_in_list(self, current_waypoint, waypoint_list):
        min_distance = float('inf')
        closest_index = -1
        for i, waypoint in enumerate(waypoint_list):
            distance = waypoint.transform.location.distance(
                current_waypoint.transform.location)
            if distance < min_distance:
                min_distance = distance
                closest_index = i

        return closest_index

    def trace_route(self, origin, destination):
        """
        This method returns list of (carla.Waypoint, RoadOption) from origin to destination
        """

        route_trace = []
        route = self._path_search(origin, destination)
        current_waypoint = self._dao.get_waypoint(origin)
        resolution = self._dao.get_resolution()

        for i in range(len(route) - 1):
            road_option = self._turn_decision(i, route)
            edge = self._graph.edges[route[i], route[i+1]]
            path = []

            if edge['type'].value != RoadOption.LANEFOLLOW.value and edge['type'].value != RoadOption.VOID.value:
                n1, n2 = self._road_id_to_edge[edge['exit_waypoint'].road_id][edge['exit_waypoint'].lane_id]
                next_edge = self._graph.edges[n1, n2]
                if next_edge['path']:
                    closest_index = self._find_closest_in_list(current_waypoint, next_edge['path'])
                    closest_index = min(len(next_edge['path'])-1, closest_index+5)
                    current_waypoint = next_edge['path'][closest_index]
                else:
                    current_waypoint = next_edge['exit_waypoint']
                route_trace.append((current_waypoint, road_option))

            else:
                path = path + [edge['entry_waypoint']] + edge['path'] + [edge['exit_waypoint']]
                closest_index = self._find_closest_in_list(current_waypoint, path)
                for waypoint in path[closest_index:]:
                    current_waypoint = waypoint
                    route_trace.append((current_waypoint, road_option))
                    if len(route)-i <= 2 and waypoint.transform.location.distance(destination) < 2*resolution:
                        break

        return route_trace
