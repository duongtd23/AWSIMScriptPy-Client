from core.scenario_manager import *
from scenarios.swerve import make_swerve_scenario

if __name__ == '__main__':
    scenario_manager = ScenarioManager()
    scenarios = []
    ve = 30
    vo = 15
    vys = [1.0, 1.2, 1.4]
    dx0s = [33, 28, 24]
    for (vy,dx0) in zip(vys, dx0s):
        scenarios.append(
            make_swerve_scenario(scenario_manager.network,
                                 ego_init_laneoffset=LaneOffset('355', 20),
                                 ego_goal_laneoffset=LaneOffset('214', 21),
                                 npc_init_laneoffset=LaneOffset('205', 40),
                                 swerve_start_laneoffset=LaneOffset('205', 48),
                                 ego_speed=ve/3.6,
                                 npc_speed=vo/3.6,
                                 swerve_vy=vy,
                                 dx0=dx0
                                ))
    scenario_manager.run(scenarios)