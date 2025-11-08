from core.scenario_manager import *
from core.trigger_condition import *

def  make_uturn_scenario(network,
                         ego_init_laneoffset,
                         ego_goal_laneoffset,
                         npc_init_laneoffset,
                         uturn_start_laneoffset,
                         uturn_next_lane,
                         ego_speed,
                         npc_speed,
                         dx0,
                         acceleration=8,
                         body_style=BodyStyle.TAXI,
                         delay_time=0.03):
    _, _, init_pos, init_orient = network.parse_lane_offset(ego_init_laneoffset)
    _, _, goal_pos, goal_orient = network.parse_lane_offset(ego_goal_laneoffset)

    # ego specification
    ego = EgoVehicle()
    ego.add_action(SpawnEgo(position=init_pos, orientation=init_orient))
    ego.add_action(SetGoalPose(position=goal_pos, orientation=goal_orient))
    ego.add_action(ActivateAutonomousMode(condition=autonomous_mode_ready()))
    ego.add_action(SetVelocityLimit(ego_speed))

    _, _, npc_init_pos, npc_init_orient = network.parse_lane_offset(npc_init_laneoffset)
    _id, source_lane, wp1, _ = network.parse_lane_offset(uturn_start_laneoffset)

    npc1 = NPCVehicle("npc1", body_style)
    npc_root_to_frontcenter = npc1.size[0]/2 + npc1.center[0]
    next_lane = network.parse_lane(uturn_next_lane)

    # uturn specification
    # 1st step: compute the waypoints
    waypoints = [npc_init_pos, wp1]
    direction = (source_lane.way_points[_id + 1] - wp1)[:2]
    direction_normalized = direction / np.linalg.norm(direction)
    uturn_backwheel_pos = -npc_root_to_frontcenter*direction_normalized + wp1[:2]
    proj,_ = next_lane.project_point2D_onto_lane(uturn_backwheel_pos)
    turning_center = (proj + uturn_backwheel_pos)/2

    turning_side = utils.get_point_side(uturn_backwheel_pos, wp1[:2], turning_center)
    for i in range(1, 4):
        wp = utils.rotate_point(wp1[:2], turning_center, turning_side*i*np.pi/3)
        waypoints.append(np.append(wp, wp1[2]))

    # 2nd step: calculate distance to trigger npc movement
    speedup_time = npc_speed / acceleration
    speedup_dis = npc_speed ** 2 / 2 / acceleration
    npc_travel_dis = np.linalg.norm(wp1 - npc_init_pos) - npc_root_to_frontcenter
    remaining_dis_to_wp1 = npc_travel_dis - speedup_dis
    time_to_wp1 = speedup_time + remaining_dis_to_wp1 / npc_speed + delay_time
    ego_travel_dis = ego_speed * time_to_wp1
    dis_threshold = dx0 + ego_travel_dis + npc_travel_dis

    # npc and follow waypoints (swerve waypoints) specification
    npc1.add_action(SpawnNPCVehicle(position=npc_init_pos, orientation=npc_init_orient))
    npc1.add_action(FollowWaypoints(waypoints=[utils.array_to_dict_pos(p) for p in waypoints],
                                    target_speed=npc_speed,
                                    acceleration=acceleration,
                                    condition=longitudinal_distance_to_ego <= dis_threshold))

    # to show non-conservative
    # npc1.add_action(FollowLane(target_speed=npc_speed + 3,
    #                         acceleration=acceleration,
    #                         condition=distance_to_ego_less_than(70)))

    return Scenario(network, [ego, npc1])