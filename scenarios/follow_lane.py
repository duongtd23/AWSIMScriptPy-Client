from core.scenario_manager import *

def follow_lane_scenario(network,
                     npc_init_laneoffset,
                     npc_speed=30/3.6,
                     acceleration=5,
                     body_style=BodyStyle.HATCHBACK):

    _, _, npc_init_pos, npc_init_orient = network.parse_lane_offset(npc_init_laneoffset)
    npc1 = NPCVehicle("npc1", body_style)

    # npc and follow waypoints (cutin waypoints) specification
    npc1.add_action(SpawnNPCVehicle(position=npc_init_pos, orientation=npc_init_orient))
    npc1.add_action(FollowLane(target_speed=npc_speed,
                               acceleration=acceleration))

    return Scenario(network, [npc1])

if __name__ == '__main__':
    scenario_manager = ScenarioManager()
    scenario =  follow_lane_scenario(scenario_manager.network,
                                     npc_init_laneoffset=LaneOffset('112', 88),
                                     npc_speed=30 / 3.6,
                                     acceleration=3)
    scenario_manager.run([scenario])