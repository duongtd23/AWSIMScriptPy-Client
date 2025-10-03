from core.client_ros_node import *
from core.action import *
from core.actor import *
from actions.ego_actions import *
from actions.npc_actions import *
from map.network import *

class ScenarioManager:
    def __init__(self, client_node, network, actors):
        self.client_node = client_node
        self.network = network
        self.actors = actors
        self.running = True

    def run(self):
        while self.running:
            global_state = {a.id: a for a in self.actors}
            for actor in self.actors:
                actor.update_state()
                actor.tick(global_state)

            # if self.client_node.should_terminate():
            #     self.running = False
