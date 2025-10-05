import numpy as np

from core.scenario_manager import *
from core.trigger_condition import *

def make_uturn_scenario(network,
                         ego_init_laneoffset,
                         ego_goal_laneoffset,
                         npc_init_laneoffset,
                         uturn_start_laneoffset,
                         uturn_next_lane,
                         ego_speed=20/3.6,
                         npc_speed=10/3.6,
                         dx0=40,
                         npc_root_to_frontcenter=2.01+1.43,
                         acceleration=7,
                         delay_time=0.01):
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
    _id, source_lane, wp1, _ = network.parse_lane_offset(uturn_start_laneoffset)

    # uturn specification
    if isinstance(uturn_next_lane, str):
        next_lane = next((l for l in network.traffic_lanes if l.id == uturn_next_lane), None)
    elif isinstance(uturn_next_lane, TrafficLane):
        next_lane = uturn_next_lane

    # 1st step: compute the waypoints
    waypoints = [npc_init_pos, wp1]
    direction = (source_lane.way_points[_id + 1] - wp1)[:2]
    direction_normalized = direction / np.linalg.norm(direction)
    uturn_backwheel_pos = -npc_root_to_frontcenter*direction_normalized + wp1[:2]
    turning_center = (next_lane.project_point_onto_lane(uturn_backwheel_pos) + uturn_backwheel_pos)/2

    turning_side = utils.get_point_side(uturn_backwheel_pos, wp1[:2], turning_center)
    for i in range(1, 5):
        wp = utils.rotate_point(wp1[:2], turning_center, turning_side*i*np.pi/4)
        waypoints.append(np.append(wp, wp1[2]))

    # 2nd step: calculate distance to trigger npc movement
    speedup_time = npc_speed / acceleration
    speedup_dis = npc_speed ** 2 / 2 / acceleration
    npc_travel_dis = np.linalg.norm(wp1 - npc_init_pos) - npc_root_to_frontcenter
    remaining_dis_to_wp1 = npc_travel_dis - speedup_dis
    time_to_wp1 = speedup_time + remaining_dis_to_wp1 / npc_speed + delay_time
    ego_travel_dis = ego_speed * time_to_wp1
    dis_threshold = dx0 + ego_travel_dis + npc_travel_dis
    print(dis_threshold)

    # npc and follow waypoints (swerve waypoints) specification
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

    scenario = make_uturn_scenario(network,
                                    ego_init_laneoffset=LaneOffset('511', 40),
                                    ego_goal_laneoffset=LaneOffset('513', 15),
                                    npc_init_laneoffset=LaneOffset('521', 35),
                                    uturn_start_laneoffset=LaneOffset('511', 43),
                                    uturn_next_lane='513'
                                    )
    scenario.run()

    node.destroy_node()
    rclpy.shutdown()