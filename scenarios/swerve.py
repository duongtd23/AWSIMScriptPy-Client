import numpy as np

from core.scenario_manager import *
from core.trigger_condition import *

def make_swerve_scenario(network,
                         ego_init_laneoffset,
                         ego_goal_laneoffset,
                         npc_init_laneoffset,
                         swerve_start_laneoffset,
                         ego_speed=30/3.6,
                         npc_speed=20/3.6,
                         swerve_vy = 1.2,
                         swerve_ny=1.8,
                         swerve_dis=2.0,
                         dx0=30,
                         swerve_right=True):
    _, _, init_pos, init_orient = network.parse_lane_offset(ego_init_laneoffset)
    _, _, goal_pos, goal_orient = network.parse_lane_offset(ego_goal_laneoffset)

    ego = EgoVehicle(node)
    npc1 = NPCVehicle("npc1", node, BodyStyle.VAN)
    ego.add_action(SpawnEgo(position=init_pos, orientation=init_orient))
    ego.add_action(SetGoalPose(position=goal_pos, orientation=goal_orient))
    ego.add_action(ActivateAutonomousMode(condition=autonomous_mode_ready()))
    ego.add_action(SetVelocityLimit(ego_speed))

    _, _, npc_init_pos, npc_init_orient = network.parse_lane_offset(npc_init_laneoffset)
    _id, source_lane, wp1, _ = network.parse_lane_offset(swerve_start_laneoffset)

    waypoints = [npc_init_pos, wp1]

    angle = np.asin(swerve_vy / npc_speed)
    diagonal = npc_speed * swerve_ny / swerve_vy  # ny / sin(angle)
    direction = (source_lane.way_points[_id+1] - npc_init_pos)[:2]
    direction_normalized = direction/np.linalg.norm(direction)
    npc_init_pos2d = npc_init_pos[:2]
    wp2 = utils.rotate_point(npc_init_pos2d + direction_normalized*diagonal,
                             npc_init_pos2d,
                             angle if not swerve_right else -angle)

    wp3 = wp2 + direction_normalized * swerve_dis
    wp4 = npc_init_pos2d + direction_normalized * (swerve_dis + 2*np.cos(angle)*diagonal)

    waypoints.append(np.append(wp2, npc_init_pos[2]))
    waypoints.append(np.append(wp3, npc_init_pos[2]))
    waypoints.append(np.append(wp4, npc_init_pos[2]))

    # TODO: parse next lane

    npc1.add_action(SpawnNPCVehicle(position=npc_init_pos, orientation=npc_init_orient))
    npc1.add_action(FollowWaypoints(waypoints=waypoints,
                                    condition=longitudinal_distance_to_ego_less_than(50),
                                    target_speed=npc_speed,
                                    acceleration=7))

    return ScenarioManager(node, network,[ego, npc1])

if __name__ == '__main__':
    rclpy.init()
    node = ClientNode()
    network = node.send_map_network_req()
    print(network)

    scenario = make_swerve_scenario(network,
                                    ego_init_laneoffset=LaneOffset('355', 20),
                                    ego_goal_laneoffset=LaneOffset('214', 21),
                                    npc_init_laneoffset=LaneOffset('205', 50),
                                    swerve_start_laneoffset=LaneOffset('205', 58)
                                    )
    scenario.run()

    node.destroy_node()
    rclpy.shutdown()