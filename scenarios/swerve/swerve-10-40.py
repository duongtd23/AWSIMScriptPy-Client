from core.scenario_manager import *
from scenarios.swerve.base import make_swerve_scenario

if __name__ == '__main__':
    # fixed params
    ve = 40/3.6
    vo = 10/3.6

    # dynamic params
    vys = [1.0, 1.2, 1.4]
    dx0s = [38, 33, 29]
    offsets = [65, 70, 74]

    # scenarios
    scenario_manager = ScenarioManager()
    scenarios = []
    for (vy,dx0,offset) in zip(vys, dx0s,offsets):
        scenarios.append(
            make_swerve_scenario(scenario_manager.network,
                                 ego_init_laneoffset=LaneOffset('268'),
                                 ego_goal_laneoffset=LaneOffset('214', 26),
                                 npc_init_laneoffset=LaneOffset('205', offset),
                                 swerve_start_laneoffset=LaneOffset('205', offset+5),
                                 ego_speed=ve,
                                 npc_speed=vo,
                                 swerve_vy=vy,
                                 dx0=dx0
                                ))
    scenario_manager.run(scenarios)