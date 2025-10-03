import json
import typing
from traffic_lane import *

class Network:
    # def __init__(self, traffic_lanes, use_Unity_coordinate=False):
    #     self.traffic_lanes = traffic_lanes
    #     self.use_Unity_coordinate = use_Unity_coordinate
    coordinate_type:bool = False
    traffic_lanes:Tuple[TrafficLane]
    def __init__(self, j):
        self.__dict__ = json.loads(j)
