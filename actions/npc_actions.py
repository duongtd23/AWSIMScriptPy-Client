from core.action import Action
import std_msgs, rclpy
import json, time
import utils
from core.actor import NPCVehicle
from core.client_ros_node import *

class SpawnNPCVehicle(Action):
    def __init__(self, position, orientation, condition=None):
        """
        :param position: np array
        :param orientation: np array
        :return:
        """
        super().__init__(condition=condition, one_shot=True)
        self.position = position
        self.orientation = orientation

    def _do(self, actor:NPCVehicle, client_node):
        my_dict = {
            "name": actor.actor_id,
            "body_style": actor.body_style.value,
            "position": utils.array_to_dict_pos(self.position),
            "orientation": utils.array_to_dict_orient(self.orientation)
        }

        msg = std_msgs.msg.String()
        msg.data = json.dumps(my_dict)
        client_node.dynamic_npc_spawning_publisher.publish(msg)

        # do a service request to confirm the spawning
        req = DynamicControl.Request()
        req.json_request = msg.data
        retry = 0
        while retry < 10:
            future = client_node.dynamic_npc_spawning_client.call_async(req)
            rclpy.spin_until_future_complete(client_node, future)
            response = future.result()
            if response.status.success:
                print(f"Spawned NPC vehicle {actor.actor_id}")
                break

            time.sleep(0.5)
            retry += 1

        if retry >= 10:
            client_node.get_logger().error(f"Failed to spawn NPC vehicle, "
                                                 f"error message: {response.status.message}")

class FollowLane(Action):
    def __init__(self, condition=None, target_speed=None, acceleration=None, deceleration=None):
        super().__init__(condition=condition, one_shot=True)

        # if target_speed is None, it follows the speed limit of the current lane
        # if acceleation/deceleration is None, the default values will be used
        self.target_speed = target_speed
        self.acceleration = acceleration
        self.deceleration = deceleration

    def _do(self, actor:NPCVehicle, client_node):
        is_speed_defined = self.target_speed is not None
        is_acceleration_defined = self.acceleration is not None
        is_deceleration_defined = self.deceleration is not None
        my_dict = {
            "target": actor.actor_id,
            "speed": self.target_speed if is_speed_defined else 0,
            "acceleration": self.acceleration if is_acceleration_defined else 0,
            "deceleration": self.deceleration if is_deceleration_defined else 0,
            "is_speed_defined": is_speed_defined,
            "is_acceleration_defined": is_acceleration_defined,
            "is_deceleration_defined": is_deceleration_defined
        }
        msg = std_msgs.msg.String()
        msg.data = json.dumps(my_dict)
        client_node.follow_lane_publisher.publish(msg)

        # do a service request to confirm the command was sent and processed properly
        retry = 0
        req = DynamicControl.Request()
        req.json_request = msg.data
        while retry < 10:
            future = client_node.follow_lane_client.call_async(req)
            rclpy.spin_until_future_complete(client_node, future)
            response = future.result()
            if response.status.success:
                print(f"Sent follow lane command to {actor.actor_id} successfully.")
                break

            time.sleep(0.5)
            retry += 1

        if retry == 10:
            client_node.get_logger().error(f"[ERROR] AWSIM failed to "
                                                 f"process follow lane action, "
                                                 f"error message: {response.status.message}.")

class FollowWaypoints(Action):
    def __init__(self, waypoints, condition=None,
                 target_speed=None, acceleration=None, deceleration=None):
        super().__init__(condition=condition, one_shot=True)
        self.waypoints = waypoints

        # if target_speed is None, it follows the speed limit of the current lane
        # if acceleation/deceleration is None, the default values will be used
        self.target_speed = target_speed
        self.acceleration = acceleration
        self.deceleration = deceleration

    def _do(self, actor:NPCVehicle, client_node):
        is_speed_defined = self.target_speed is not None
        is_acceleration_defined = self.acceleration is not None
        is_deceleration_defined = self.deceleration is not None

        my_dict = {
            "target": actor.actor_id,
            "waypoints": self.waypoints,
            "speed": self.target_speed if is_speed_defined else 0,
            "acceleration": self.acceleration if is_acceleration_defined else 0,
            "deceleration": self.deceleration if is_deceleration_defined else 0,
            "is_speed_defined": is_speed_defined,
            "is_acceleration_defined": is_acceleration_defined,
            "is_deceleration_defined": is_deceleration_defined
        }
        msg = std_msgs.msg.String()
        msg.data = json.dumps(my_dict)
        client_node.follow_waypoints_publisher.publish(msg)

        # do a service request to confirm the command was sent and processed properly
        retry = 0
        req = DynamicControl.Request()
        req.json_request = msg.data
        while retry < 10:
            future = client_node.follow_waypoints_client.call_async(req)
            rclpy.spin_until_future_complete(client_node, future)
            response = future.result()
            if response.status.success:
                print(f"Sent follow waypoints command to {actor.actor_id} successfully.")
                break
            time.sleep(0.5)
            retry += 1

        if retry == 10:
            client_node.get_logger().error(f"[ERROR] AWSIM failed to process follow waypoints action, "
                  f"error message: {response.status.message}.")
