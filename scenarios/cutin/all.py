import scenarios.cutin.cutin_30_10 as cutin_30_10
import scenarios.cutin.cutin_40_20 as cutin_40_20

from core.scenario_manager import ScenarioManager

if __name__ == '__main__':
    scenario_manager = ScenarioManager()
    scenarios = (
        cutin_30_10.make_scenarios(scenario_manager.network) +
        cutin_40_20.make_scenarios(scenario_manager.network)
    )
    scenario_manager.run(scenarios)