import numpy as np

from core.scenario_manager import *
from core.trigger_condition import *

def make_swerve_scenario(network,
                         npc_init_laneoffset,
                         swerve_start_laneoffset,
                         npc_speed=10/3.6,
                         swerve_vy = 1.2,
                         swerve_ny=1.8,
                         swerve_dis=2.0,
                         swerve_right=True):
    npc1 = NPCVehicle("npc1", node, BodyStyle.HATCHBACK)

    _, _, npc_init_pos, npc_init_orient = network.parse_lane_offset(npc_init_laneoffset)
    _id, source_lane, wp1, _ = network.parse_lane_offset(swerve_start_laneoffset)

    waypoints = [npc_init_pos, wp1]

    angle = np.asin(swerve_vy / npc_speed)
    print(f"angle: {angle}")
    diagonal = npc_speed * swerve_ny / swerve_vy  # ny / sin(angle)
    print(f"diagonal: {diagonal}")
    direction = (source_lane.way_points[_id+1] - wp1)[:2]
    direction_normalized = direction/np.linalg.norm(direction)
    wp1_2D = wp1[:2]
    wp2 = utils.rotate_point(wp1_2D + direction_normalized*diagonal,
                             wp1_2D,
                             angle if not swerve_right else -angle)

    wp3 = wp2 + direction_normalized * swerve_dis
    wp4 = wp1_2D + direction_normalized * (swerve_dis + 2*np.cos(angle)*diagonal)

    waypoints.append(np.append(wp2, wp1[2]))
    waypoints.append(np.append(wp3, wp1[2]))
    waypoints.append(np.append(wp4, wp1[2]))

    print(waypoints)

    # TODO: parse next lane

    npc1.add_action(SpawnNPCVehicle(position=npc_init_pos, orientation=npc_init_orient))
    npc1.add_action(FollowWaypoints(waypoints=[utils.array_to_dict_pos(p) for p in waypoints],
                                    target_speed=npc_speed,
                                    acceleration=7))

    return ScenarioManager(node, network,[npc1])

if __name__ == '__main__':
    rclpy.init()
    node = ClientNode()
    network = node.send_map_network_req()
    print(network)

    scenario = make_swerve_scenario(network,
                                    npc_init_laneoffset=LaneOffset('205', 50),
                                    swerve_start_laneoffset=LaneOffset('205', 58)
                                    )
    scenario.run()

    node.destroy_node()
    rclpy.shutdown()