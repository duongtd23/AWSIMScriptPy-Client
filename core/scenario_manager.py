from core.client_ros_node import *
from core.action import *
from core.actor import *
from actions.ego_actions import *
from actions.npc_actions import *

class ScenarioManager:
    def __init__(self, client_node:ClientNode, network:Network, actors):
        self.client_node = client_node
        self.network = network
        self.logger = client_node.get_logger()

        # validate actor ids
        if not any(a.actor_id=="ego" for a in actors):
            self.logger.warn("No ego actor found.")
        for i in range(len(actors)):
            for j in range(i + 1, len(actors)):
                if actors[i].actor_id == actors[j].actor_id:
                    raise Exception(f"Two actors have same id {actors[i].actor_id}")

        self.actors = actors
        self.running = True
        self.global_state = {
            "ads_internal_status": AdsInternalStatus.UNINITIALIZED.value,
            "ego_motion_state": MOTION_STATE_STOPPED,
            "actor-sizes": {  # hard code. TODO: fix this
                "ego": {
                    "size": [4.886,2.186,1.421],
                    "center": [1.424,0.0,0.973]
                },
                "npc1": {
                    "size": [4.02,1.94,1.64],
                    "center": [1.43,0.85,0.0]
                }
            },
            # "actor-kinematics": {}
        }

    def run(self):
        while self.running:
            self.update_global_state()
            for actor in self.actors:
                # actor.update_state()
                actor.tick(self.global_state)

            if self.global_state["ads_internal_status"] == AdsInternalStatus.GOAL_ARRIVED.value:
                self.terminate()

            time.sleep(0.1)

    def terminate(self):
        self.running = False

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

        kinematics_msg = self.client_node.query_groundtruth_kinematics()
        self.global_state["actor-kinematics"] = msg_to_dict(kinematics_msg)

def msg_to_dict(msg):
    def veh_obj_to_dict(vehicle):
        return {
            # "name": vehicle.name,
            "pose": {
                "position": utils.object_to_point_arr(vehicle.pose.position),
                "rotation": utils.object_to_point_arr(vehicle.pose.rotation)
            },
            "twist": {
                "linear": utils.object_to_point_arr(vehicle.twist.linear),
                "angular": utils.object_to_point_arr(vehicle.twist.angular)
            },
            "accel": vehicle.accel,
            # "bounding_box": {
            #     "x": (vehicle.bounding_box.x),
            #     "y": (vehicle.bounding_box.y),
            #     "width": (vehicle.bounding_box.width),
            #     "height": (vehicle.bounding_box.height),
            # },
        }

    def pedes_obj_to_dict(pedestrian):
        return {
            # "name": pedestrian.name,
            "pose": {
                "position": utils.object_to_point_arr(pedestrian.pose.position),
                "rotation": utils.object_to_point_arr(pedestrian.pose.rotation),
            },
            "speed": (pedestrian.speed),
            # "bounding_box": {
            #     "x": (pedestrian.bounding_box.x),
            #     "y": (pedestrian.bounding_box.y),
            #     "width": (pedestrian.bounding_box.width),
            #     "height": (pedestrian.bounding_box.height),
            # }
        }

    result = {
        "ego": {
            "pose": {
                "position": utils.object_to_point_arr(msg.groundtruth_ego.pose.position),
                "rotation": utils.object_to_point_arr(msg.groundtruth_ego.pose.rotation),
            },
            "twist": {
                "linear": utils.object_to_point_arr(msg.groundtruth_ego.twist.linear),
                "angular": utils.object_to_point_arr(msg.groundtruth_ego.twist.angular),
            },
            "acceleration": {
                "linear": utils.object_to_point_arr(msg.groundtruth_ego.accel.linear),
                "angular": utils.object_to_point_arr(msg.groundtruth_ego.accel.angular),
            }
        },
        "vehicles": {v.name: veh_obj_to_dict(v) for v in msg.groundtruth_vehicles},
        "pedestrians": {p.name: pedes_obj_to_dict(p) for p in msg.groundtruth_pedestrians}
    }
    return result
