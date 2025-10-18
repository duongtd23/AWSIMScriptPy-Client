import scenarios.uturn.uturn_left_10 as uturn_left_10
import scenarios.uturn.uturn_left_15 as uturn_left_15
import scenarios.uturn.uturn_right_10 as uturn_right_10
import scenarios.uturn.uturn_right_15 as uturn_right_15

from core.scenario_manager import ScenarioManager

if __name__ == '__main__':
    scenario_manager = ScenarioManager()
    scenarios = (
        uturn_left_10.make_scenarios(scenario_manager.network) +
        uturn_left_15.make_scenarios(scenario_manager.network) +
        uturn_right_10.make_scenarios(scenario_manager.network) +
        uturn_right_15.make_scenarios(scenario_manager.network)
    )
    scenario_manager.run(scenarios)