"""
Another specification for the cutin scenarios
"""
from core.trigger_condition import *
from core.scenario_manager import *

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

def make_scenario(network,
                 ego_init_laneoffset,
                 ego_goal_laneoffset,
                 npc_init_laneoffset,
                 cutin_next_lane,
                 ego_speed,
                 npc_speed,
                 cutin_vy,
                 dx0,
                 acceleration=8,
                 body_style=BodyStyle.SMALL_CAR):

    _, _, init_pos, init_orient = network.parse_lane_offset(ego_init_laneoffset)
    _, _, goal_pos, goal_orient = network.parse_lane_offset(ego_goal_laneoffset)

    # ego specification
    ego = EgoVehicle()
    ego.add_action(SpawnEgo(position=init_pos, orientation=init_orient))
    ego.add_action(SetGoalPose(position=goal_pos, orientation=goal_orient))
    ego.add_action(ActivateAutonomousMode(condition=autonomous_mode_ready()))
    ego.add_action(SetVelocityLimit(ego_speed,one_shot=True))

    _, source_lane, npc_init_pos, npc_init_orient = network.parse_lane_offset(npc_init_laneoffset)
    npc1 = NPCVehicle("npc1", body_style)

    # cutin specification
    next_lane = network.parse_lane(cutin_next_lane)

    def cal_cutin_waypoints(actor, global_state):
        npc_kin = global_state['actor-kinematics']['vehicles'].get(actor.actor_id)
        npc_pos = np.array(npc_kin['pose']['position'])
        npc_front_center = actor.get_front_center(npc_pos, npc_kin['pose']['rotation'][2])
        wp_id = source_lane.parse_wp_segment(npc_front_center)
        waypoints = cutin_waypoints(npc_front_center, npc_speed, cutin_vy, source_lane, wp_id + 1, next_lane)
        return [utils.array_to_dict_pos(p) for p in waypoints]

    # npc and follow waypoints (cutin waypoints) specification
    npc1.add_action(SpawnNPCVehicle(position=npc_init_pos, orientation=npc_init_orient))
    npc1.add_action(FollowLane(target_speed=npc_speed,
                               acceleration=acceleration,
                               condition=av_speed >= ego_speed - 1))
    npc1.add_action(FollowWaypoints(waypoints_calculation_callback=cal_cutin_waypoints,
                                    condition=longitudinal_distance_to_ego <= dx0))
    return Scenario(network, [ego, npc1])

if __name__ == '__main__':
    scenario_manager = ScenarioManager()
    scenario =  make_scenario(scenario_manager.network,
                              ego_init_laneoffset=LaneOffset('111', 0),
                              ego_goal_laneoffset=LaneOffset('111', 130),
                              npc_init_laneoffset=LaneOffset('112', 70),
                              cutin_next_lane='111',
                              ego_speed=30/3.6,
                              npc_speed=10/3.6,
                              cutin_vy=1.2,
                              dx0=10
                            )
    scenario_manager.run([scenario])