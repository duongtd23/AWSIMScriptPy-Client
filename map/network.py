import json
from typing import List
from traffic_lane import *
import numpy as np

class LaneOffset:
    def __init__(self, lane_str, offset: float=0.0):
        self.lane_str = lane_str
        self.offset = offset

class Network:
    coordinate_type:bool = False
    traffic_lanes: List[TrafficLane]
    def __init__(self, json_str):
        obj = json.loads(json_str)
        self.coordinate_type = obj['coordinate_type']
        self.traffic_lanes = [TrafficLane.from_dict(lane) for lane in obj['traffic_lanes']]

    def parse_lane_offset(self, lane_offset):
        """
        Derive the 3D point from the given lane offset
        :param lane_offset:
        :return: {id, point}, where id is the waypoint index of the start of
        the waypoint segment on which the point is located
        """
        lane = next((l for l in self.traffic_lanes if l.id == lane_offset.lane_str), None)
        if not lane:
            full_name = "TrafficLane." + lane_offset.lane_str
            lane = next((l for l in self.traffic_lanes if l.id == full_name), None)
            if not lane:
                raise Exception(f"Lane {lane_offset.lane_str} not found")

        remaining_dis = lane_offset.offset
        for i in range(len(lane.way_points) - 1):
            start = lane.way_points[i]
            end = lane.way_points[i + 1]
            segment = end - start
            if remaining_dis - np.linalg.norm(segment) < 0:
                point = start + remaining_dis/np.linalg.norm(segment)*segment
                yaw_angle = math.atan2(end[1] - point[1], end[0] - point[0])
                return i, point, utils.quaternion_from_yaw(yaw_angle)

            remaining_dis = remaining_dis - np.linalg.norm(segment)

        raise Exception(f"Offset {lane_offset.offset} exceeds length of lane {lane_offset.lane_str}")
