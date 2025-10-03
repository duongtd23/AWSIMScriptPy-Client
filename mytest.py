from core.scenario_manager import *

def make_scenario(network):
    _, init_pos, init_orient = network.parse_lane_offset(LaneOffset('355', 30))
    _, goal_pos, goal_orient = network.parse_lane_offset(LaneOffset('214', 21))

    _, npc_init_pos, npc_init_orient = network.parse_lane_offset(LaneOffset('205', 50))

    print(npc_init_pos)

    ego = EgoVehicle("ego", node)
    npc1 = NPCVehicle("npc1", node, BodyStyle.VAN)
    ego.add_action(SpawnEgo(position=init_pos, orientation=init_orient))
    ego.add_action(SetGoalPose(position=goal_pos, orientation=goal_orient))
    # ego.add_action(ActivateAutonomousMode())

    # npc sequence: spawn only when ego comes closer than 20m
    npc1.add_action(SpawnNPCVehicle(position=npc_init_pos, orientation=npc_init_orient))
    # condition=distance_to_ego_less_than(20)))
    # npc1.add_action(SetTargetSpeed(speed=5.0))

    return ScenarioManager(node, network,[ego, npc1])

if __name__ == '__main__':
    rclpy.init()
    node = ClientNode()
    network = node.send_map_network_req()
    print(network)

    scenario = make_scenario(network)
    scenario.run()

    node.destroy_node()
    rclpy.shutdown()