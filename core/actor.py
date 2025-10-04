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
    def __init__(self, actor_id, client_node, size=(5,2)):
        """
        :param size: (length, width)
        """
        # static properties
        super().__init__(actor_id, client_node)
        self.size = np.array(size)


class EgoVehicle(VehicleActor):
    def __init__(self, actor_id, client_node, size=(4.88,2.00)):
        VehicleActor.__init__(self, actor_id, client_node, size)

class BodyStyle(enum.Enum):
    TAXI="taxi"
    HATCHBACK="hatchback"
    SMALL_CAR="small-car"
    VAN="van"
    TRUCK="truck"

class NPCVehicle(VehicleActor):
    def __init__(self, actor_id, client_node, body_style:BodyStyle, size=(5,2)):
        VehicleActor.__init__(self, actor_id, client_node, size)
        self.body_style = body_style