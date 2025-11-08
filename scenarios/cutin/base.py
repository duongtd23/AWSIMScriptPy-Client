from core.scenario_manager import *
from core.trigger_condition import *

def cutin_waypoints(wp1, npc_speed, cutin_vy, source_lane, current_wp_id, next_lane):
    proj, _ = next_lane.project_point2D_onto_lane(wp1[:2])
    lateral_dis = np.linalg.norm(proj - wp1[:2])

    angle = np.asin(cutin_vy / npc_speed)
    diagonal = npc_speed * lateral_dis / cutin_vy
    current_follow_wp = source_lane.way_points[current_wp_id] # the next waypoint ahead of the vehicle
    direction = (current_follow_wp - wp1)[:2]
    direction_normalized = direction / np.linalg.norm(direction)

    turning_side = utils.get_point_side(wp1[:2], current_follow_wp[:2], proj)
    wp2 = utils.rotate_point(wp1[:2] + direction_normalized * diagonal,
                             wp1[:2],
                             turning_side * angle)
    wp3 = wp2 + (0.4 * npc_speed + 5) * direction_normalized
    return [wp1, np.append(wp2, wp1[2]), np.append(wp3, wp1[2])]

def make_cutin_scenario(network,
                     ego_init_laneoffset,
                     ego_goal_laneoffset,
                     npc_init_laneoffset,
                     cutin_start_laneoffset,
                     cutin_next_lane,
                     ego_speed,
                     npc_speed,
                     cutin_vy,
                     dx0,
                     acceleration=8,
                     body_style=BodyStyle.HATCHBACK,
                     delay_time=0.05):

    _, _, init_pos, init_orient = network.parse_lane_offset(ego_init_laneoffset)
    _, _, goal_pos, goal_orient = network.parse_lane_offset(ego_goal_laneoffset)

    # ego specification
    ego = EgoVehicle()
    ego.add_action(SpawnEgo(position=init_pos, orientation=init_orient))
    ego.add_action(SetGoalPose(position=goal_pos, orientation=goal_orient))
    ego.add_action(ActivateAutonomousMode(condition=autonomous_mode_ready()))
    ego.add_action(SetVelocityLimit(ego_speed,one_shot=True))

    _, _, npc_init_pos, npc_init_orient = network.parse_lane_offset(npc_init_laneoffset)
    _id, source_lane, wp1, _ = network.parse_lane_offset(cutin_start_laneoffset)

    npc1 = NPCVehicle("npc1", body_style)
    npc_root_to_frontcenter = npc1.size[0]/2 + npc1.center[0]

    # cutin specification
    next_lane = network.parse_lane(cutin_next_lane)

    # 1st step: compute the waypoints
    waypoints = ([npc_init_pos] +
                 cutin_waypoints(wp1, npc_speed, cutin_vy, source_lane, _id + 1, next_lane))

    # 2nd step: calculate distance to trigger npc movement
    speedup_time = npc_speed / acceleration
    speedup_dis = npc_speed ** 2 / 2 / acceleration
    npc_travel_dis = np.linalg.norm(wp1 - npc_init_pos) - npc_root_to_frontcenter
    remaining_dis_to_wp1 = npc_travel_dis - speedup_dis
    time_to_wp1 = speedup_time + remaining_dis_to_wp1 / npc_speed + delay_time
    ego_travel_dis = ego_speed * time_to_wp1
    dis_threshold = dx0 + ego_travel_dis - npc_travel_dis

    # npc and follow waypoints (cutin waypoints) specification
    npc1.add_action(SpawnNPCVehicle(position=npc_init_pos, orientation=npc_init_orient))
    npc1.add_action(FollowWaypoints(waypoints=[utils.array_to_dict_pos(p) for p in waypoints],
                                    target_speed=npc_speed,
                                    acceleration=acceleration,
                                    condition=longitudinal_distance_to_ego <= dis_threshold))

    # to show non-conservative
    # npc1.add_action(FollowLane(target_speed=npc_speed,
    #                         acceleration=acceleration,
    #                         condition=longitudinal_distance_to_ego_less_than(dis_threshold)))
    return Scenario(network, [ego, npc1])

if __name__ == '__main__':
    scenario_manager = ScenarioManager()
    scenario =  make_cutin_scenario(scenario_manager.network,
                                    ego_init_laneoffset=LaneOffset('111', 0),
                                    ego_goal_laneoffset=LaneOffset('111', 130),
                                    npc_init_laneoffset=LaneOffset('112', 88),
                                    cutin_start_laneoffset=LaneOffset('112', 95),
                                    cutin_next_lane='111'
                                    )
    scenario_manager.run([scenario])