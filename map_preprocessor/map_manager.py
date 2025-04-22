#!/usr/bin/python3

"""
This module defines a function that manages the map and global path
"""
import os
from autoware_planning_msgs.msg import LaneletRoute
from geometry_msgs.msg import Pose
from queue import Queue

import lanelet2
from autoware_lanelet2_extension_python.utility import load_info_from_yaml, MapProjectorInfo
from autoware_lanelet2_extension_python.projection import MGRSProjector, TransverseMercatorProjector
from autoware_perception_msgs.msg import TrafficLightGroupArray

def automatic_find_projector_yaml(map_path):
    """in the same directory with the name projector_info.yaml"""
    # get the directory of the map file path
    map_dir = os.path.dirname(map_path)
    # get the name of the map file without the extension
    projector_info = os.path.join(map_dir, "map_projector_info.yaml")
    if os.path.exists(projector_info):
        return projector_info
    else:
        return None

def get_lanelet2_projector(projector_info: MapProjectorInfo):
    """
    プロジェクタ情報に基づいて、適切なlanelet2のプロジェクタを返します。
    
    引数:
      projector_info: プロジェクタ情報を保持するオブジェクト
      
    戻り値:
      lanelet2のプロジェクタオブジェクト
      
    例外:
      ValueError: サポートされていないプロジェクタタイプが指定された場合
    """
    # LOCAL_CARTESIAN_UTM の場合
    if projector_info.projector_type == "LOCAL_CARTESIAN_UTM":
        position = lanelet2.GPSPoint(
            projector_info.map_origin.latitude,
            projector_info.map_origin.longitude,
            projector_info.map_origin.altitude
        )
        origin = lanelet2.io.Origin(position)
        return lanelet2.projection.UtmProjector(origin)
    
    # MGRS の場合
    elif projector_info.projector_type == "MGRS":
        projector = MGRSProjector(lanelet2.io.Origin(0, 0))
        projector.setMGRSCode(projector_info.mgrs_grid)
        return projector
    
    # TRANSVERSE_MERCATOR の場合
    elif projector_info.projector_type == "TRANSVERSE_MERCATOR":
        position = lanelet2.core.GPSPoint(
            projector_info.map_origin.latitude,
            projector_info.map_origin.longitude,
            projector_info.map_origin.altitude
        )
        origin = lanelet2.Origin(position)
        return TransverseMercatorProjector(origin)
    
    # LOCAL_CARTESIAN の場合
    elif projector_info.projector_type == "LOCAL_CARTESIAN":
        position = lanelet2.core.GPSPoint(
            projector_info.map_origin.latitude,
            projector_info.map_origin.longitude,
            projector_info.map_origin.altitude
        )
        origin = lanelet2.ioOrigin(position)
        return lanelet2.projection.LocalCartesianProjector(origin)

class MapManager:
    def __init__(self, map_path, local_map_range=80.0, look_ahead_step=30):
        self.map_path = map_path
        self.look_ahead_step = look_ahead_step

        self.projector_yaml = automatic_find_projector_yaml(map_path)
        if self.projector_yaml is None:
            self.projector = MGRSProjector(lanelet2.io.Origin(0.0, 0.0))
        else:
            self.projector = get_lanelet2_projector(load_info_from_yaml(self.projector_yaml))
        self.map_object = lanelet2.io.load(self.map_path, self.projector)

        # ## Also load the connection first
        # traffic_rules = lanelet2.traffic_rules.create(
        #     lanelet2.traffic_rules.Locations.Germany,
        #     lanelet2.traffic_rules.Participants.Vehicle,
        # )
        # self.routing_graph = lanelet2.routing.RoutingGraph(self.map_object, traffic_rules)

        self.global_path = []
        self.drivable_paths = []
        self.local_map_range = local_map_range

        self.traffic_light_messages = Queue(look_ahead_step)
        self.latest_access = -1

        self.latest_traffic_light_messages = {}

    def step_traffic_light_message(self, msg:TrafficLightGroupArray):
        data = dict()
        data["stamp"] = msg.stamp
        data["elements"] = {}
        for traffic_light_group in msg.traffic_light_groups:
            group_id = traffic_light_group.traffic_light_group_id
            elements = traffic_light_group.elements
            data["elements"][group_id] = []
            for i, element in enumerate(elements):
                data["elements"][group_id].append({
                    "color": element.color,
                    "shape": element.shape,
                    "status": element.status,
                    "confidence": element.confidence
                })
        if self.traffic_light_messages.full():
            self.latest_traffic_light_messages = self.traffic_light_messages.get()

        self.traffic_light_messages.put(data["elements"])

    
    def _get_traffic_light(self, lanelet):
        lights = lanelet.trafficLights()
        if len(lights) == 0:
            return -1
        return lights[0].id

    def set_global_path(self, msgs:LaneletRoute):
        self.global_prefered_path = []
        self.drivable_paths = []
        for segment in msgs.segments:
            self.global_path.append(segment.preferred_primitive.id)
            for primitive in segment.primitives:
                if primitive.id in self.drivable_paths:
                    print(f"{primitive.id} reappear")
                self.drivable_paths.append(primitive.id)


    def fetch_local_map(self, position:Pose):
        """This methods should return the map elements near the position, and also return nearby global routes"""
        x = position.position.x
        y = position.position.y
        z = position.position.z
        # get the map elements near the position
        search_bounding_box = lanelet2.core.BoundingBox2d(
            lanelet2.core.BasicPoint2d(float(x - self.local_map_range), float(y - self.local_map_range)),
            lanelet2.core.BasicPoint2d(float(x + self.local_map_range), float(y + self.local_map_range))
        )
        nearby_lanelets =  self.map_object.laneletLayer.search(search_bounding_box)
        nearby_lanelets_ids = [lanelet.id for lanelet in nearby_lanelets]
        associated_traffic_light_ids = [self._get_traffic_light(lanelet) for lanelet in nearby_lanelets]
        current_traffic_light_status = self.latest_traffic_light_messages # latest for the target frame

        nearby_global_path = []
        for lanelet_id in self.global_path:
            if lanelet_id in nearby_lanelets_ids:
                nearby_global_path.append(lanelet_id)

        nearby_drivable_path = []
        for lanelet_id in self.drivable_paths:
            if lanelet_id in nearby_lanelets_ids:
                nearby_drivable_path.append(lanelet_id)

        return {"map_elements": nearby_lanelets, "routes": nearby_global_path, "nearby_drivable_path": nearby_drivable_path, "nearby_lanelets_ids": nearby_lanelets_ids, 
                "associated_traffic_light_ids": associated_traffic_light_ids, "current_traffic_light_status": current_traffic_light_status}
