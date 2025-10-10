from core.scenario_manager import *
from scenarios.swerve import make_swerve_scenario

if __name__ == '__main__':
    scenario_manager = ScenarioManager()
    scenarios = []
    ve = 40
    vo = 15
    vys = [1.0, 1.2, 1.4]
    dx0s = [41, 34, 30]
    offsets = [47, 52, 57]
    # offsets = [55, 60, 65]
    for (vy, dx0, offset) in zip(vys, dx0s, offsets):
        scenarios.append(
            make_swerve_scenario(scenario_manager.network,
                                 ego_init_laneoffset=LaneOffset('268'),
                                 ego_goal_laneoffset=LaneOffset('214', 26),
                                 npc_init_laneoffset=LaneOffset('205', 58),
                                 swerve_start_laneoffset=LaneOffset('205', 65),
                                 ego_speed=ve/3.6,
                                 npc_speed=vo/3.6,
                                 swerve_vy=vy,
                                 dx0=dx0
                                ))
    scenario_manager.run(scenarios)