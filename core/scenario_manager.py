from core.client_ros_node import *
from core.action import *
from core.actor import *
from actions.ego_actions import *
from actions.npc_actions import *

class ScenarioManager:
    def __init__(self, client_node:ClientNode, network:Network, actors):
        self.client_node = client_node
        self.network = network
        self.actors = actors
        self.running = True
        self.global_state = {
            "ads_internal_status": AdsInternalStatus.UNINITIALIZED.value,
            "ego_motion_state": MOTION_STATE_STOPPED,
            "actors": {a.actor_id: {} for a in self.actors}
        }
        self.logger = client_node.get_logger()

    def run(self):
        while self.running:
            self.update_global_state()
            for actor in self.actors:
                # actor.update_state()
                actor.tick(self.global_state)

            # if self.client_node.should_terminate():
            #     self.running = False

    def update_global_state(self):
        ads_exec_state = self.client_node.query_execution_state()
        self.global_state["ego_motion_state"] = ads_exec_state.motion_state
        if ads_exec_state.is_autonomous_mode_available and \
                self.global_state["ads_internal_status"] < AdsInternalStatus.AUTONOMOUS_MODE_READY.value:
            self.logger.info("Autonomous operation mode is ready")
            self.global_state["ads_internal_status"] = AdsInternalStatus.AUTONOMOUS_MODE_READY.value

        if (ads_exec_state.routing_state == ROUTING_STATE_ARRIVED and
                self.global_state["ads_internal_status"] == AdsInternalStatus.AUTONOMOUS_IN_PROGRESS.value):
            self.logger.info("Arrived destination")
            self.global_state["ads_internal_status"] = AdsInternalStatus.GOAL_ARRIVED.value

        # TODO: update vehicle kinematics