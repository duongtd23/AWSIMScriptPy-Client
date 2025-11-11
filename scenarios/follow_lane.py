from core.scenario_manager import *
from core.trigger_condition import *

scenario_manager = ScenarioManager()
network = scenario_manager.network
_, _, init_pos, init_orient = network.parse_lane_offset(LaneOffset('123'))
npc1 = NPCVehicle("npc1", BodyStyle.SMALL_CAR)
npc1.add_action(SpawnNPCVehicle(position=init_pos, orientation=init_orient))
npc1.add_action(FollowLane(target_speed=30/3.6, acceleration=5))
npc1.add_action(ChangeLane(next_lane=network.parse_lane('124'), lateral_velocity=1.0,
                           condition=reach_point(LaneOffset('123', 12), network)))
scenario = Scenario(network, [npc1])
scenario_manager.run([scenario])