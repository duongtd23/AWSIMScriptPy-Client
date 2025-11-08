import enum
import numpy as np

from core.client_ros_node import ClientNode

class Actor:
    def __init__(self, actor_id):
        # static properties
        self.actor_id = actor_id
        self.actions = []

    def add_action(self, action):
        self.actions.append(action)

    def tick(self, global_state, client_node):
        ids_to_remove = []
        for (i,action) in enumerate(self.actions):
            completed = action.execute(self, global_state, client_node)
            if completed and action.one_shot:
                ids_to_remove.insert(0, i)

        [self.actions.pop(i) for i in ids_to_remove]


class VehicleActor(Actor):
    def __init__(self, actor_id, size=(5,2,1.4), center=(0.0,0.0,0.0)):
        """
        :param size: (length, width, height)
        :param center:
        """
        # static properties
        super().__init__(actor_id)
        self.size = np.array(size)
        self.center = np.array(center)

    def get_front_center(self, position, heading_deg):
        """
        :return: the front center point of the vehicle
        """
        front_center_local = self.center[:2] + np.array((self.size[0]/2, 0))
        theta = np.deg2rad(heading_deg)
        rot = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta),  np.cos(theta)]
        ])
        _2dpoint = position[:2] + rot @ front_center_local
        return np.append(_2dpoint, position[2])

class EgoVehicle(VehicleActor):
    def __init__(self, size=(4.886,2.186,1.421), center=(1.424,0.0,0.973)):
        VehicleActor.__init__(self, "ego", size)
        self.center = np.array(center)

class BodyStyle(enum.Enum):
    TAXI="taxi"
    HATCHBACK="hatchback"
    SMALL_CAR="small-car"
    VAN="van"
    TRUCK="truck"

class NPCVehicle(VehicleActor):
    def __init__(self, actor_id, body_style:BodyStyle, size=None, center=None):
        if size is None:
            match body_style:
                case BodyStyle.TAXI:
                    size = (4.64, 1.94, 1.64)
                case BodyStyle.HATCHBACK:
                    size = (4.02,1.94,1.64)
                case BodyStyle.SMALL_CAR:
                    size = (3.48, 1.82, 1.74)
                case BodyStyle.VAN:
                    size = (4.64, 1.94, 1.82)
                case BodyStyle.TRUCK:
                    size = (9.2, 3.08, 4.14)
        if center is None:
            match body_style:
                case BodyStyle.TAXI:
                    center = (1.12, 0.0, 0.85)
                case BodyStyle.HATCHBACK:
                    center = (1.43, 0.0, 0.85)
                case BodyStyle.SMALL_CAR:
                    center = (1.12, 0.0, 0.9)
                case BodyStyle.VAN:
                    center = (1.12, 0.0, 1.18)
                case BodyStyle.TRUCK:
                    center = (0.0, 0.0, 2.13)

        VehicleActor.__init__(self, actor_id, size)
        self.body_style = body_style
        self.center = np.array(center)
