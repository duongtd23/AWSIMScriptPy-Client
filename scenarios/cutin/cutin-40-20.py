from core.scenario_manager import *
from scenarios.cutin.base import make_cutin_scenario

if __name__ == '__main__':
    # fixed params
    ego_speed = 40 / 3.6
    npc_speed = 20 / 3.6

    # dynamic params
    vys = [1.2, 1.4, 1.6]
    dx0s = [13, 12, 12]

    # scenarios
    scenario_manager = ScenarioManager()
    scenarios = []
    for (vy,dx0) in zip(vys, dx0s):
        scenarios.append(
            make_cutin_scenario(scenario_manager.client_node, scenario_manager.network,
                                ego_init_laneoffset=LaneOffset('111', 0),
                                ego_goal_laneoffset=LaneOffset('111', 210),
                                npc_init_laneoffset=LaneOffset('112', 117),
                                cutin_start_laneoffset=LaneOffset('112', 123),
                                cutin_next_lane='111',
                                ego_speed=ego_speed,
                                npc_speed=npc_speed,
                                cutin_vy=vy,
                                dx0=dx0,
                                body_style=BodyStyle.SMALL_CAR,
                                ))
    scenario_manager.run(scenarios)