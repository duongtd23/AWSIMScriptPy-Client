import enum
import numpy as np

from core.client_ros_node import ClientNode

class Actor:
    def __init__(self, actor_id):
        # static properties
        self.actor_id = actor_id
        self.state = {}
        self.actions = []

    # TODO
    # def update_state(self):
    #     self.state = self.sim_api.get_actor_state(self.id)

    def add_action(self, action):
        self.actions.append(action)

    def tick(self, global_state, client_node):
        for action in self.actions:
            if action.execute(self, global_state, client_node):
                if action.one_shot:
                    self.actions.remove(action)

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
                case HATCHBACK:
                    size = (4.02,1.94,1.64)
        if not center:
            match body_style:
                case HATCHBACK:
                    center = (1.43,0.85,0.0)

        VehicleActor.__init__(self, actor_id, size)
        self.body_style = body_style
        self.center = np.array(center)
