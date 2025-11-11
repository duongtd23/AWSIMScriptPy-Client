from core.scenario_manager import *
from scenarios.swerve.base import make_swerve_scenario

def make_scenarios(network):
    # fixed params
    ve = 30/3.6
    vo = 10/3.6

    # dynamic params
    vys = [1.0, 1.2, 1.4]
    dx0s = [31, 27, 24]

    # scenarios
    scenarios = []
    for (vy,dx0) in zip(vys, dx0s):
        scenarios.append(
            make_swerve_scenario(network,
                                 ego_init_laneoffset=LaneOffset('355', 10),
                                 ego_goal_laneoffset=LaneOffset('214', 10),
                                 npc_init_laneoffset=LaneOffset('205', 60),
                                 _ego_speed=ve,
                                 _npc_speed=vo,
                                 swerve_vy=vy,
                                 body_style=BodyStyle.SMALL_CAR,
                                 dx0=dx0
                                ))
    return scenarios

if __name__ == '__main__':
    scenario_manager = ScenarioManager()
    scenario_manager.run(make_scenarios(scenario_manager.network))