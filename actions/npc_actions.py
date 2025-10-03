from core.action import Action
import std_msgs, rclpy
import json, time
import utils
from core.actor import NPCVehicle

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

    def _do(self, actor:NPCVehicle):
        my_dict = {
            "name": actor.actor_id,
            "body_style": actor.body_style.value,
            "position": utils.array_to_dict_pos(self.position),
            "orientation": utils.array_to_dict_orient(self.orientation)
        }

        msg = std_msgs.msg.String()
        msg.data = json.dumps(my_dict)
        actor.client_node.dynamic_npc_spawning_publisher.publish(msg)

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


