from scenarios.cutin.base import cutin_waypoints
from core.trigger_condition import *
from core.scenario_manager import *

def make_scenario(network,
                 ego_init_laneoffset,
                 ego_goal_laneoffset,
                 npc_init_laneoffset,
                 cutin_next_lane,
                 _ego_speed,
                 _npc_speed,
                 _cutin_vy,
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
    ego.add_action(SetVelocityLimit(_ego_speed,one_shot=True))

    # NPC specification
    _, source_lane, npc_init_pos, npc_init_orient = network.parse_lane_offset(npc_init_laneoffset)
    npc1 = NPCVehicle("npc1", body_style)

    next_lane = network.parse_lane(cutin_next_lane)
    npc1.add_action(SpawnNPCVehicle(position=npc_init_pos, orientation=npc_init_orient))
    npc1.add_action(FollowLane(target_speed=_npc_speed,
                               acceleration=acceleration,
                               condition=av_speed >= _ego_speed-0.5))
    npc1.add_action(ChangeLane(next_lane=next_lane,
                               lateral_velocity=_cutin_vy,
                               condition=longitudinal_distance_to_ego <= dx0))
    return Scenario(network, [ego, npc1])

if __name__ == '__main__':
    scenario_manager = ScenarioManager()
    scenario =  make_scenario(scenario_manager.network,
                              ego_init_laneoffset=LaneOffset('111', 0),
                              ego_goal_laneoffset=LaneOffset('111', 130),
                              npc_init_laneoffset=LaneOffset('112', 70),
                              cutin_next_lane='111',
                              _ego_speed=30/3.6,
                              _npc_speed=10/3.6,
                              _cutin_vy=1.2,
                              dx0=10
                            )
    scenario_manager.run([scenario])