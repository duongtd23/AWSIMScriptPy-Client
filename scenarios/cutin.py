import numpy as np

from core.scenario_manager import *
from core.trigger_condition import *

def make_cutin_scenario(network,
                     ego_init_laneoffset,
                     ego_goal_laneoffset,
                     npc_init_laneoffset,
                     cutin_start_laneoffset,
                     ego_speed=20/3.6,
                     npc_speed=10/3.6,
                     cutin_vy = 1.2,
                     dx0=10,
                     npc_root_to_frontcenter=2.01+1.43,
                     acceleration=7,
                     delay_time=0.01,
                     cutin_right=False):

    _, _, init_pos, init_orient = network.parse_lane_offset(ego_init_laneoffset)
    _, _, goal_pos, goal_orient = network.parse_lane_offset(ego_goal_laneoffset)

    # ego specification
    ego = EgoVehicle(node)
    npc1 = NPCVehicle("npc1", node, BodyStyle.HATCHBACK)
    ego.add_action(SpawnEgo(position=init_pos, orientation=init_orient))
    ego.add_action(SetGoalPose(position=goal_pos, orientation=goal_orient))
    ego.add_action(ActivateAutonomousMode(condition=autonomous_mode_ready()))
    ego.add_action(SetVelocityLimit(ego_speed))

    _, _, npc_init_pos, npc_init_orient = network.parse_lane_offset(npc_init_laneoffset)
    _id, source_lane, wp1, _ = network.parse_lane_offset(cutin_start_laneoffset)

    # cutin specification
    # 1st step: compute the waypoints
    waypoints = [npc_init_pos, wp1]
    angle = np.asin(cutin_vy / npc_speed)
    diagonal = npc_speed * source_lane.width / cutin_vy
    direction = (source_lane.way_points[_id + 1] - wp1)[:2]
    direction_normalized = direction / np.linalg.norm(direction)
    wp1_2D = wp1[:2]
    wp2 = utils.rotate_point(wp1_2D + direction_normalized * diagonal,
                             wp1_2D,
                             angle if not cutin_right else -angle)

    waypoints.append(np.append(wp2, wp1[2]))

    # 2nd step: calculate distance to trigger npc movement
    speedup_time = npc_speed / acceleration
    speedup_dis = npc_speed ** 2 / 2 / acceleration
    npc_travel_dis = np.linalg.norm(wp1 - npc_init_pos) - npc_root_to_frontcenter
    remaining_dis_to_wp1 = npc_travel_dis - speedup_dis
    time_to_wp1 = speedup_time + remaining_dis_to_wp1 / npc_speed + delay_time
    ego_travel_dis = ego_speed * time_to_wp1
    dis_threshold = dx0 + ego_travel_dis - npc_travel_dis
    print(dis_threshold)

    # npc and follow waypoints (cutin waypoints) specification
    npc1.add_action(SpawnNPCVehicle(position=npc_init_pos, orientation=npc_init_orient))
    npc1.add_action(FollowWaypoints(waypoints=[utils.array_to_dict_pos(p) for p in waypoints],
                                    target_speed=npc_speed,
                                    acceleration=8,
                                    condition=longitudinal_distance_to_ego_less_than(dis_threshold)))

    return ScenarioManager(node, network,[ego, npc1])

if __name__ == '__main__':
    rclpy.init()
    node = ClientNode()
    network = node.send_map_network_req()
    print(network)

    scenario = make_cutin_scenario(network,
                                    ego_init_laneoffset=LaneOffset('111', 0),
                                    ego_goal_laneoffset=LaneOffset('111', 130),
                                    npc_init_laneoffset=LaneOffset('112', 50),
                                    cutin_start_laneoffset=LaneOffset('112', 58)
                                    )
    scenario.run()

    node.destroy_node()
    rclpy.shutdown()