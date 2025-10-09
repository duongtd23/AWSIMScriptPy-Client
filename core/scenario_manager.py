from core.client_ros_node import *
from core.action import *
from core.actor import *
from actions.ego_actions import *
from actions.npc_actions import *

class Scenario:
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
        }
        self.has_active_action = True

    def run(self):
        while self.running:
            self.update_global_state()
            for actor in self.actors:
                # actor.update_state()
                actor.tick(self.global_state, self.client_node)

            if self.global_state["ads_internal_status"] == AdsInternalStatus.GOAL_ARRIVED.value:
                print("Goal arrived. Scenario terminating...")
                self.terminate()

            if not any(actor.actor_id != "ego" and actor.actions for actor in self.actors):
                # print("No remain actions")
                time.sleep(0.1)

    def terminate(self):
        self.running = False

    def update_global_state(self):
        ads_exec_state = self.client_node.query_execution_state()
        self.global_state["ego_motion_state"] = ads_exec_state.motion_state
        if ads_exec_state.is_autonomous_mode_available and \
                ads_exec_state.routing_state == ROUTING_STATE_SET and \
                self.global_state["ads_internal_status"] < AdsInternalStatus.AUTONOMOUS_MODE_READY.value:
            # print(ads_exec_state)
            self.logger.info("Autonomous operation mode is ready")
            self.global_state["ads_internal_status"] = AdsInternalStatus.AUTONOMOUS_MODE_READY.value

        if ads_exec_state.routing_state == ROUTING_STATE_ARRIVED:
            # motion_state = 1, routing_state = 3, operation_state = 2, is_autonomous_mode_available = True
            self.logger.info("Arrived destination")
            self.global_state["ads_internal_status"] = AdsInternalStatus.GOAL_ARRIVED.value

        kinematics_msg = self.client_node.query_groundtruth_kinematics()
        self.global_state["actor-kinematics"] = kinematic_msg_to_dict(kinematics_msg)

        gt_size_msg = self.client_node.query_groundtruth_size()
        self.global_state["actor-sizes"] = gt_size_msg_to_dict(gt_size_msg)

class ScenarioManager:
    def __init__(self, wait_writing_trace=False,):
        rclpy.init()
        self.client_node = ClientNode()
        self.network = self.client_node.send_map_network_req()
        self.wait_writing_trace = wait_writing_trace

    def reset(self):
        self.client_node.destroy_node()
        rclpy.shutdown()
        rclpy.init()
        self.client_node = ClientNode()

    def run(self, scenarios):
        while scenarios:
            current_scenario = scenarios.pop(0)
            # current_scenario.client_node = self.client_node
            # current_scenario.logger = current_scenario.client_node.get_logger()
            print("\n Starting scenario")
            current_scenario.run()
            self.scenario_finish()
            # self.reset()

        self.terminate()

    def scenario_finish(self):
        self.client_node.publish_finish_signal()
        if self.wait_writing_trace:
            # wait for trace writing completes
            while True:
                res = self.client_node.query_recording_state()
                if (res.state is not MonitorRecordingState.Response.WRITING_DATA and
                        res.state is not MonitorRecordingState.Response.RECORDING):
                    break
                time.sleep(1)

        self.client_node.remove_npcs()
        self.client_node.clear_route()
        self.client_node.published_finish_signal = False
        time.sleep(15)

    def terminate(self):
        self.client_node.destroy_node()
        rclpy.shutdown()

def kinematic_msg_to_dict(msg):
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

def gt_size_msg_to_dict(msg):
    def size_obj_to_dict(size_obj):
        return {
            "size": utils.object_to_point_arr(size_obj.size),
            "center": utils.object_to_point_arr(size_obj.center),
        }

    return {v.name: size_obj_to_dict(v) for v in msg.vehicle_sizes}