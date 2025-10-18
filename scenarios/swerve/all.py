import scenarios.swerve.swerve_10_30 as swerve_10_30
import scenarios.swerve.swerve_10_40 as swerve_10_40
import scenarios.swerve.swerve_15_30 as swerve_15_30
import scenarios.swerve.swerve_15_40 as swerve_15_40

from core.scenario_manager import ScenarioManager

if __name__ == '__main__':
    scenario_manager = ScenarioManager()
    scenarios = (
        swerve_10_30.make_scenarios(scenario_manager.network) +
        swerve_10_40.make_scenarios(scenario_manager.network) +
        swerve_15_30.make_scenarios(scenario_manager.network) +
        swerve_15_40.make_scenarios(scenario_manager.network)
    )
    scenario_manager.run(scenarios)