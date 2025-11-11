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

    def _do(self, actor:NPCVehicle, client_node, global_state):
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
    def __init__(self, lane=None, condition=None, target_speed=None, acceleration=None, deceleration=None):
        super().__init__(condition=condition, one_shot=True)

        # if lane is None, following the lane on which the actor currently located.
        # Once finish the lane, randomly follow one of the next lanes
        # If specified, it must be a string
        self.lane = lane

        # if target_speed is None, it follows the speed limit of the current lane
        # if acceleation/deceleration is None, the default values will be used
        self.target_speed = target_speed
        self.acceleration = acceleration
        self.deceleration = deceleration

    def _do(self, actor:NPCVehicle, client_node, global_state):
        is_speed_defined = self.target_speed is not None
        is_acceleration_defined = self.acceleration is not None
        is_deceleration_defined = self.deceleration is not None
        my_dict = {
            "target": actor.actor_id,
            "lane": "" if self.lane is None else self.lane,
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
        print(f"Sent follow lane command to {actor.actor_id} successfully.")

class FollowWaypoints(Action):
    def __init__(self, waypoints=None, waypoints_calculation_callback=None,
                 condition=None, target_speed=None, acceleration=None, deceleration=None):
        """
        Either waypoints or waypoints_calculation_callback must be defined.
        :param waypoints:
        :param waypoints_calculation_callback:
        :param condition:
        :param target_speed:
        :param acceleration:
        :param deceleration:
        """
        if waypoints is None and waypoints_calculation_callback is None:
            raise ValueError("waypoints or waypoints_calculation_callback must be defined")
        super().__init__(condition=condition, one_shot=True)
        self.waypoints = waypoints
        self.waypoints_calculation_callback = waypoints_calculation_callback

        # if target_speed is None, it follows the speed limit of the current lane
        # if acceleation/deceleration is None, the default values will be used
        self.target_speed = target_speed
        self.acceleration = acceleration
        self.deceleration = deceleration

    def _do(self, actor:NPCVehicle, client_node, global_state):
        waypoints = self.waypoints
        if waypoints is None:
            waypoints = self.waypoints_calculation_callback(actor, global_state)
            print(waypoints)
        is_speed_defined = self.target_speed is not None
        is_acceleration_defined = self.acceleration is not None
        is_deceleration_defined = self.deceleration is not None

        my_dict = {
            "target": actor.actor_id,
            "waypoints": waypoints,
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
        print(f"Sent follow waypoints command to {actor.actor_id} successfully.")

class SetTargetSpeed(Action):
    def __init__(self, target_speed, condition=None, acceleration=None, deceleration=None):
        super().__init__(condition=condition, one_shot=True)

        # if acceleation/deceleration is None, the default values will be used
        self.target_speed = target_speed
        self.acceleration = acceleration
        self.deceleration = deceleration

    def _do(self, actor:NPCVehicle, client_node, global_state):
        is_acceleration_defined = self.acceleration is not None
        is_deceleration_defined = self.deceleration is not None
        my_dict = {
            "target": actor.actor_id,
            "speed": self.target_speed,
            "acceleration": self.acceleration if is_acceleration_defined else 0,
            "deceleration": self.deceleration if is_deceleration_defined else 0,
            "is_acceleration_defined": is_acceleration_defined,
            "is_deceleration_defined": is_deceleration_defined
        }
        msg = std_msgs.msg.String()
        msg.data = json.dumps(my_dict)
        client_node.set_target_speed_publisher.publish(msg)
        print(f"Sent set target speed to {actor.actor_id} successfully.")

class ChangeLane(Action):
    def __init__(self, next_lane, lateral_velocity=1.0,
                 condition=None):
        super().__init__(condition=condition, one_shot=True)
        self.next_lane = next_lane
        self.lateral_velocity = lateral_velocity

    def _do(self, actor:NPCVehicle, client_node, global_state):
        kinematic = global_state['actor-kinematics']['vehicles'].get(actor.actor_id)
        if not kinematic:
            print(f'[ERROR] NPC {actor.actor_id} not found in the world state')
            return
        current_pos = np.array(kinematic['pose']['position'])
        current_speed = np.linalg.norm(np.array(kinematic['twist']['linear']))
        front_center_point = actor.get_front_center(current_pos, kinematic['pose']['rotation'][2])

        projection = None
        next_lane_wp_id = -1
        for i in range(len(self.next_lane.way_points) - 1):
            proj, projection_inside_segment = utils.project_point_to_segment_2d(
                front_center_point[:2],
                self.next_lane.way_points[i][:2],
                self.next_lane.way_points[i + 1][:2])
            if projection_inside_segment:
                projection = proj
                next_lane_wp_id = i

        if projection is None:
            print(f'[ERROR] Invalid next lane for the lane change')
            return

        # calculate waypoints
        lateral_dis = np.linalg.norm(front_center_point[:2] - projection)
        long_speed = np.sqrt(current_speed ** 2 - self.lateral_velocity ** 2)
        long_dis = lateral_dis / self.lateral_velocity * long_speed
        direction = projection - self.next_lane.way_points[next_lane_wp_id][:2]
        direction = direction / np.linalg.norm(direction)
        wp0 = projection + direction * long_dis
        waypoints = [front_center_point, np.append(wp0, front_center_point[2])]

        # add one more waypoint for smooth steering back
        wp1 = wp0 + direction * current_speed * 0.52 + 4
        waypoints.append(np.append(wp1, front_center_point[2]))

        next_wp_id = next_lane_wp_id + 1
        while next_wp_id < len(self.next_lane.way_points):
            next_wp = self.next_lane.way_points[next_wp_id][:2]
            if np.dot(direction, next_wp - wp1) > 0:
                break
            next_wp_id += 1

        if next_wp_id < len(self.next_lane.way_points):
            waypoints += self.next_lane.way_points[next_wp_id:]

        my_dict = {
            "target": actor.actor_id,
            "waypoints": [utils.array_to_dict_pos(p) for p in waypoints]
        }
        msg = std_msgs.msg.String()
        msg.data = json.dumps(my_dict)
        client_node.follow_waypoints_publisher.publish(msg)
        print(f"Sent lane change command to {actor.actor_id}.")
