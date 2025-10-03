import enum, json
from client import *
import rclpy
import utils
from geometry_msgs.msg import Pose, PoseWithCovariance, PoseWithCovarianceStamped
import std_msgs

from network import LaneOffset

class BodyStyle(enum.Enum):
    TAXI="taxi"
    HATCHBACK="hatchback"
    SMALL_CAR="small-car"
    VAN="van"
    TRUCK="truck"

class Scenario:
    def __init__(self, node, network):
        self.node = node
        self.network = network
        self.timestep=0.1

    def spawn_npc_vehicle(self, name, body_style:BodyStyle, position, orientation):
        """
        :param name:
        :param body_style:
        :param position: np array
        :param orientation: np array
        :return:
        """
        my_dict = {
            "name": name,
            "body_style": body_style.value,
            "position": utils.array_to_dict_pos(position),
            "orientation": utils.array_to_dict_orient(orientation)
        }

        msg = std_msgs.msg.String()
        msg.data = json.dumps(my_dict)
        self.node.dynamic_npc_spawning_publisher.publish(msg)

        # do a service request to confirm the spawning
        req = DynamicControl.Request()
        req.json_request = msg.data
        retry = 0
        while retry < 10:
            future = self.node.dynamic_npc_spawning_client.call_async(req)
            rclpy.spin_until_future_complete(self.node, future)
            response = future.result()
            if response.status.success:
                print(f"Spawned NPC vehicle {name}")
                break

            time.sleep(self.timestep)
            retry += 1

        if retry >= 10:
            self.node.get_logger().error(f"Failed to spawn NPC vehicle, error message: {response.status.message}")

    def spawn_ego(self, position, orientation):
        """
        Spawn the ego vehicle and request (re-)localization
        :param position:
        :param orientation:
        :return:
        """
        ros_pose = utils.obj_to_ros_pose(position, orientation)
        cov = [0.0] * 36
        cov[0] = 1e-4  # x
        cov[7] = 1e-4  # y
        cov[35] = 1e-4  # yaw

        # publish a pose message
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose = ros_pose
        msg.pose.covariance = cov

        self.node.ego_pose_publisher.publish(msg)

        ok = self.node.re_localization(ros_pose, cov)
        if ok:
            self.node.get_logger().info("Spawned ego and re-localized successfully.")
            # re-localization succeeded
            # self.ads_internal_status = AdsInternalStatus.GOAL_SET
            # self.loop()
        else:
            self.node.get_logger().error("Failed to localize ego.")

    def set_ad_task(self, goal_position, goal_orientation, velocity_limit=None):
        """
        Set Autonomous driving task, including goal, max velocity, activate autonoums mode
        :param goal_position:
        :param goal_orientation:
        :return:
        """
        ros_goal = utils.obj_to_ros_pose(goal_position, goal_orientation)
        self.node.set_goal(ros_goal)



    def simulate_npcs(self, npc_vehicles):
        pass

    def upd_kinematics(self):
        pass

if __name__ == '__main__':
    rclpy.init()
    node = ClientNode()
    network = node.send_map_network_req()

    print(network)

    _, init_pos, init_orient = network.parse_lane_offset(LaneOffset('355', 30))
    _, goal_pos, goal_orient = network.parse_lane_offset(LaneOffset('214', 21))

    _, npc_init_pos, npc_init_orient = network.parse_lane_offset(LaneOffset('205', 50))

    print(npc_init_pos)

    scenario = Scenario(node, network)
    scenario.spawn_ego(init_pos, init_orient)
    scenario.spawn_npc_vehicle("npc1", BodyStyle.VAN, npc_init_pos, npc_init_orient)

    node.destroy_node()
    rclpy.shutdown()