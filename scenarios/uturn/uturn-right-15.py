from core.scenario_manager import *
from scenarios.uturn.base import make_uturn_scenario

if __name__ == '__main__':
    # fixed params
    vo = 15/3.6

    # dynamic params
    ves = [30, 35, 40]
    dx0s = [21, 25, 28]
    ego_init_laneoffsets = [LaneOffset('514', 38),
                            LaneOffset('514', 17),
                            LaneOffset('282', 4)]
    ego_goal_laneoffsets = [LaneOffset('516', 20),
                            LaneOffset('516', 20),
                            LaneOffset('124', 18)]

    # scenarios
    scenario_manager = ScenarioManager()
    scenarios = []
    for (ve, dx0, ego_init_laneoffset, ego_goal_laneoffset) in (
            zip(ves, dx0s, ego_init_laneoffsets, ego_goal_laneoffsets)):
        scenarios.append(
            make_uturn_scenario(scenario_manager.network,
                                ego_init_laneoffset=ego_init_laneoffset,
                                ego_goal_laneoffset=ego_goal_laneoffset,
                                npc_init_laneoffset=LaneOffset('521', 37),
                                uturn_start_laneoffset=LaneOffset('521', 43),
                                uturn_next_lane='511',
                                ego_speed=ve/3.6,
                                npc_speed=vo,
                                dx0=dx0
                                ))
    scenario_manager.run(scenarios)