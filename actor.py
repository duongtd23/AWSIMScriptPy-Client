import numpy as np

class Vehicle:
    def __init__(self, name, size=(5,2)):
        """
        :param name:
        :param size: (length, width)
        """
        # static properties
        self.name = name
        self.size = np.array(size)

        # dynamic properties
        self.properties = {}

class EgoVehicleSetting:
    def __init__(self, max_velocity):
        self.max_velocity = max_velocity

class EgoVehicle(Vehicle):
    def __init__(self, name, init_pose, goal, custom_setting, size=(5,2)):
        Vehicle.__init__(self, name, size)
        self.init_pose = init_pose
        self.goal = goal
        self.custom_setting = custom_setting

class NPCVehicle(Vehicle):
    def __init__(self, name, size):
        Vehicle.__init__(self, name, size)