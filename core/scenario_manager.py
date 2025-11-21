from core.client_ros_node import *
from core.action import *
from core.actor import *
from actions.ego_actions import *
from actions.npc_actions import *
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class Scenario:
    def __init__(self, network: Network, actors, client_node: ClientNode=None):
        self.network = network
        self.client_node = client_node
        if self.client_node is not None:
            self.logger = client_node.get_logger()

        # validate actor ids
        if not any(a.actor_id=="ego" for a in actors):
            print("[WARN] No ego actor found.")
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
        self.lock = False

    def set_client(self, client_node):
        self.client_node = client_node
        self.logger = client_node.get_logger()

    def run(self):
        while self.running:
            self.update_global_state()
            for actor in self.actors:
                actor.tick(self.global_state, self.client_node)

            if self.global_state["ads_internal_status"] == AdsInternalStatus.AUTONOMOUS_IN_PROGRESS.value:
                self.running = False
                self.subscribe_kinematics()
                return

            # depending on the PC performance, this value can be decreased or
            # it should be increased
            time.sleep(0.1)

    def terminate(self):
        self.running = False

    def update_global_state(self):
        ads_exec_state = self.client_node.query_execution_state()
        self.global_state["ego_motion_state"] = ads_exec_state.motion_state
        if ads_exec_state.is_autonomous_mode_available and \
                ads_exec_state.routing_state == ROUTING_STATE_SET and \
                self.global_state["ads_internal_status"] < AdsInternalStatus.AUTONOMOUS_MODE_READY.value:
            self.logger.info("Autonomous operation mode is ready.")
            self.global_state["ads_internal_status"] = AdsInternalStatus.AUTONOMOUS_MODE_READY.value

        if ads_exec_state.motion_state == MOTION_STATE_MOVING and \
            self.global_state["ads_internal_status"] == AdsInternalStatus.AUTONOMOUS_MODE_READY.value:
            self.global_state["ads_internal_status"] = AdsInternalStatus.AUTONOMOUS_IN_PROGRESS.value

        kinematics_msg = self.client_node.query_groundtruth_kinematics()
        self.global_state["actor-kinematics"] = kinematic_msg_to_dict(kinematics_msg)

        gt_size_msg = self.client_node.query_groundtruth_size()
        self.global_state["actor-sizes"] = gt_size_msg_to_dict(gt_size_msg)

    def subscribe_kinematics(self):
        self.client_node.create_subscription(
            aw_monitor.msg.GroundtruthKinematic,
            '/simulation/gt/kinematic',
            self.kinematics_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
        )
        self.client_node.create_subscription(
            RouteState,
            '/api/routing/state',
            self.routing_state_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                durability=DurabilityPolicy.TRANSIENT_LOCAL
            )
        )
        self.my_spin()

    def kinematics_callback(self, msg):
        if self.lock:
            print(f"ignore kinematics {msg}")
            return
        self.lock = True
        self.global_state["actor-kinematics"] = kinematic_msg_to_dict(msg)
        for actor in self.actors:
            actor.tick(self.global_state, self.client_node)
        self.lock = False

    def routing_state_callback(self, msg):
        if msg.state == ROUTING_STATE_ARRIVED:
            self.logger.info("Goal arrived")
            self.global_state["ads_internal_status"] = AdsInternalStatus.GOAL_ARRIVED.value

    def my_spin(self):
        while self.global_state["ads_internal_status"] < AdsInternalStatus.GOAL_ARRIVED.value:
            rclpy.spin_once(self.client_node)

        self.logger.info("Scenario terminated")

class ScenarioManager:
    def __init__(self, wait_writing_trace=False,):
        rclpy.init()
        self.client_node = ClientNode()
        self.network = self.client_node.send_map_network_req()
        self.wait_writing_trace = wait_writing_trace

    def reset(self, init_new=False):
        self.client_node.destroy_node()
        rclpy.shutdown()
        if init_new:
            rclpy.init()
            self.client_node = ClientNode()

    def run(self, scenarios):
        while scenarios:
            current_scenario = scenarios.pop(0)
            current_scenario.set_client(self.client_node)
            print("\nStarting scenario")
            current_scenario.run()
            self.scenario_finish()
            self.reset(init_new=len(scenarios)>0)

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