from core.scenario_manager import *
from scenarios.cutin import make_cutin_scenario

if __name__ == '__main__':
    scenario_manager = ScenarioManager()
    scenarios = []
    vys = [1.2, 1.4, 1.6]
    dx0s = [13, 12, 12]
    for (vy,dx0) in zip(vys, dx0s):
        scenarios.append(
            make_cutin_scenario(scenario_manager.client_node, scenario_manager.network,
                                ego_init_laneoffset=LaneOffset('111', 0),
                                ego_goal_laneoffset=LaneOffset('111', 210),
                                npc_init_laneoffset=LaneOffset('112', 117),
                                cutin_start_laneoffset=LaneOffset('112', 123),
                                cutin_next_lane='111',
                                ego_speed=40 / 3.6,
                                npc_speed=20 / 3.6,
                                cutin_vy=vy,
                                dx0=dx0,
                                npc_root_to_frontcenter=1.74 + 1.12,
                                body_style=BodyStyle.SMALL_CAR,
                                acceleration=8,
                                delay_time=0.05
                                ))
    scenario_manager.run(scenarios)