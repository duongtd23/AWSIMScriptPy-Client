from core.scenario_manager import *
from scenarios.cutin.base import make_cutin_scenario

if __name__ == '__main__':
    scenario_manager = ScenarioManager()
    scenario =  make_cutin_scenario(scenario_manager.network,
                              ego_init_laneoffset=LaneOffset('111', 0),
                              ego_goal_laneoffset=LaneOffset('111', 130),
                              npc_init_laneoffset=LaneOffset('112', 70),
                              cutin_next_lane='111',
                              _ego_speed=30/3.6,
                              _npc_speed=10/3.6,
                              _cutin_vy=1.0,
                              dx0=15
                            )
    scenario_manager.run([scenario])