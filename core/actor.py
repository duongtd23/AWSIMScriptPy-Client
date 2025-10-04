import enum
import numpy as np

from core.client_ros_node import ClientNode

class Actor:
    def __init__(self, actor_id, client_node:ClientNode):
        # static properties
        self.actor_id = actor_id
        self.client_node = client_node
        self.state = {}
        self.actions = []

    # TODO
    # def update_state(self):
    #     self.state = self.sim_api.get_actor_state(self.id)

    def add_action(self, action):
        self.actions.append(action)

    def tick(self, global_state):
        for action in self.actions:
            if action.execute(self, global_state):
                if action.one_shot:
                    self.actions.remove(action)

class VehicleActor(Actor):
    def __init__(self, actor_id, client_node, size=(5,2,1.4), center=(0.0,0.0,0.0)):
        """
        :param size: (length, width, height)
        :param center:
        """
        # static properties
        super().__init__(actor_id, client_node)
        self.size = np.array(size)
        self.center = np.array(center)

class EgoVehicle(VehicleActor):
    def __init__(self, client_node, size=(4.886,2.186,1.421), center=(1.424,0.0,0.973)):
        VehicleActor.__init__(self, "ego", client_node, size)
        self.center = np.array(center)

class BodyStyle(enum.Enum):
    TAXI="taxi"
    HATCHBACK="hatchback"
    SMALL_CAR="small-car"
    VAN="van"
    TRUCK="truck"

class NPCVehicle(VehicleActor):
    def __init__(self, actor_id, client_node, body_style:BodyStyle, size=None, center=None):
        if not size:
            match body_style:
                case HATCHBACK:
                    size = (4.02,1.94,1.64)
        if not center:
            match body_style:
                case HATCHBACK:
                    center = (1.43,0.85,0.0)

        VehicleActor.__init__(self, actor_id, client_node, size)
        self.body_style = body_style
        self.center = np.array(center)
