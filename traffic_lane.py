from enum import Enum
import numpy as np
from typing import Optional, Tuple
import math

class TurnDirection(Enum):
    UNDEFINED = 0
    STRAIGHT = 1
    LEFT = 2
    RIGHT = 3

# Represent a traffic lane
class TrafficLane:
    def __init__(self, id: str, turn_direction:TurnDirection, speed_limit:float,
                 way_points, next_lanes: Tuple[str], prev_lanes: Tuple[str]):
        self.id = id
        self.turn_direction = turn_direction
        self.speed_limit = speed_limit

        self.way_points = way_points
        self.next_lanes = next_lanes
        self.prev_lanes = prev_lanes

        # self.stop_line = None   TODO: parse this info
        # self.right_of_way_lanes = []   TODO: parse this info

    def is_intersection_lane(self):
        return self.turn_direction != TurnDirection.UNDEFINED
    
    def get_2D_waypoints(self):
        return [(x,y) for (x,y,_) in self.way_points]
    
    def __str__(self):
        next_ids = [entry.id for entry in self.next_lanes]
        prev_ids = [entry.id for entry in self.prev_lanes]
        return f'Lane #{self.id}, next: {next_ids}, prev: {prev_ids}'