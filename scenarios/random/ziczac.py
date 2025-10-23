import copy

from core.scenario_manager import *
from core.trigger_condition import autonomous_mode_ready
from scenarios.utils import *

def make_scenario(network,
                 npc_init_laneoffset,
                 start_laneoffset,
                 npc_speed,
                 acceleration=7,
                 body_style=BodyStyle.SMALL_CAR):

    _, _, npc_init_pos, npc_init_orient = network.parse_lane_offset(npc_init_laneoffset)
    _id, source_lane, wp1, _ = network.parse_lane_offset(start_laneoffset)

    npc1 = NPCVehicle("npc1", body_style)
    npc_root_to_frontcenter = npc1.size[0]/2 + npc1.center[0]

    # ziczac specification
    waypoints = [npc_init_pos, wp1]

    direction = (source_lane.way_points[_id + 1] - wp1)[:2]
    direction_normalized = direction / np.linalg.norm(direction)

    long_shift = 8
    lat_shift = 2.7
    wp2 = point_forward(wp1[:2], direction_normalized, long_shift, lat_shift)
    waypoints.append(np.append(wp2, wp1[2]))

    right_point = wp2
    for i in range(1, 4):
        wp3 = right_point + direction_normalized * 2
        wp4 = point_forward(wp3, direction_normalized, 6, -2.4)
        wp5 = wp4 + direction_normalized * 2
        wp6 = point_forward(wp5, direction_normalized, 6, 2.4)
        for p in [wp3, wp4, wp5, wp6]:
            waypoints.append(np.append(p, wp1[2]))
        right_point = copy.deepcopy(wp6)

    npc1.add_action(SpawnNPCVehicle(position=npc_init_pos, orientation=npc_init_orient))
    npc1.add_action(FollowWaypoints(waypoints=[utils.array_to_dict_pos(p) for p in waypoints],
                                    target_speed=npc_speed,
                                    acceleration=acceleration))

    return Scenario(network, [npc1])

if __name__ == '__main__':
    scenario_manager = ScenarioManager()
    scenario =  make_scenario(scenario_manager.network,
                    npc_init_laneoffset=LaneOffset('111', 28),
                    start_laneoffset=LaneOffset('111', 35),
                    npc_speed=20/3.6)
    scenario_manager.run([scenario])