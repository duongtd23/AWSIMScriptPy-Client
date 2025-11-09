from core.scenario_manager import *
from core.trigger_condition import *

def follow_lane_scenario(network,
                     npc_init_laneoffset,
                     npc_speed=30/3.6,
                     acceleration=5,
                     body_style=BodyStyle.HATCHBACK):

    _, _, npc_init_pos, npc_init_orient = network.parse_lane_offset(npc_init_laneoffset)
    npc1 = NPCVehicle("npc1", body_style)

    npc1.add_action(SpawnNPCVehicle(position=npc_init_pos, orientation=npc_init_orient))
    npc1.add_action(FollowLane(target_speed=npc_speed,
                               acceleration=acceleration))
    # npc1.add_action(FollowLane('TrafficLane.335',
    #                            target_speed=3,
    #                            condition=end_lane(npc_init_laneoffset.lane_str, network)))
    npc1.add_action(SetTargetSpeed(target_speed=3,
                                   condition=actor_speed < 8))

    return Scenario(network, [npc1])

if __name__ == '__main__':
    scenario_manager = ScenarioManager()
    scenario =  follow_lane_scenario(scenario_manager.network,
                                     npc_init_laneoffset=LaneOffset('124', 2),
                                     npc_speed=30 / 3.6,
                                     acceleration=3)
    scenario_manager.run([scenario])