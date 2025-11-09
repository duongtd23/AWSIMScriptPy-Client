## AWSIM-ScriptPy - A Flexible Interface for Scenario Specification in AWSIM-Labs and Autoware

This is the client library of AWSIM-ScriptPy, a flexible interface for scenario specification in [AWSIM-Labs simulator](https://github.com/duongtd23/AWSIM-Labs/tree/dev) for [Autoware](https://github.com/autowarefoundation/autoware).

The implementation of simulator server interface is available at the extended [AWSIM-Labs](https://github.com/duongtd23/AWSIM-Labs/tree/dev).

### Usage
#### Python Scenario Specification

We recommend specifying your scenarios using Python. Some example scenarios are available in [scenarios](scenarios) folder.
For instance, to execute a ziczac scenario, use the following command:
```bash
python -m scenarios.random.ziczac2
```
Make sure to source the folder where Autoware was installed first.
Note that multiple scenarios can be passed and run sequentially, for example, see file [scenarios/cutin/all.py](scenarios/cutin/all.py).

See [Predefined Actions](#predefined-actions) and [Predefined Conditions](#predefined-conditions) sections below for more details on how to specify scenarios using the Python interface.

#### Using .script Files

To simulate a scenario specified in a `.script` file (original AWSIM-Script), you can use the `script_file_manager.py` utility as follows:
```bash
python script_file_manager.py <path-to-input-script-or-folder>
```

See `python script_file_manager.py -h` for more details.
If a folder is given, each script file inside will be processed one by one.
If using with [AW-RuntimeMonitor](https://github.com/duongtd23/AW-RuntimeMonitor/tree/awsimclient), a trace (text data and video) will be saved for each simulation (script file).

### Predefined Actions
Some predefined actions for common tasks are available in [actions](actions) folder.
#### Ego Vehicle Actions
- `Spawning`: Spawn the ego vehicle at a specified position and orientation (precisely, reset initial pose), and perform ADS re-localization. Note that precise 3D coordinates can be obtained from a lane offset expression with our helper functions likes
  ```python
  _, _, position, orientation = network.parse_lane_offset(LaneOffset('111', 10))
  ```
- `SetGoalPose`: Set the goal pose for the ego vehicle at a specified position and orientation. A goal is required for Autoware to perform autonomous driving.
- `ActivateAutonomousMode`: Activate the autonomous driving mode in Autoware.
This action should be specified with the condition that the autonomous driving mode is ready (at least after setting initial pose and goal pose).
- `SetVelocityLimit`: Set the speed limit for the ego vehicle. Note that the real speed at each moment depends on Autoware's planning and control modules.

![Alt text](assets/network.png "Intersection with lane IDs")

#### Other Vehicle Actions
- `SpawnNPCVehicle`: Spawn a non-player character (NPC) vehicle at a specified position and orientation.
- `FollowLane`: Make the NPC vehicle follow the corresponding lane at the current position (without specifying the lane explicitly). 
  - If the lane later splits into multiple lanes (e.g., near an intersection), one next lane to follow is chosen randomly. For instance, in the figure above, if the NPC vehicle is on lane `124` and approaches the intersection, it may randomly choose to follow lane `441` or `442` or `335` (to turn right) once finishing lane `124`.
  - Instead of letting the NPC vehicle choose next lanes randomly like above, we can also explicitly specify the desired next lane to follow by passing `lane` parameter to the `FollowLane` action. For instance, the following specification makes the NPC vehicle follow lane `335` after finishing lane `124`:
  ```python
  npc1.add_action(FollowLane(
       lane='TrafficLane.335', 
       condition=end_lane('TrafficLane.124', network)))
  ```

  - We can also specify a desired speed profile, by passing `target_speed` (m/s), `acceleration` (m/s^2), and `deceleration` (m/s^2). If `target_speed` is not specified, the NPC vehicle will follow the speed limit of the lane. If `acceleration` or `deceleration` is not specified, a default comfortable value will be used.

- `SetTargetSpeed`: Set the target speed for the NPC vehicle. The vehicle will accelerate or decelerate to reach the target speed. Again, we can also specify desired `acceleration` and `deceleration`. By passing the `target_speed=0`, we can make the NPC vehicle stop.

- `ChangeLane`: Make the NPC vehicle change to the given `next_lane` argument. The lane must be adjacent to the current lane of the vehicle.
To control how fast the lane change is performed, we can also specify `lateral_velocity` (in m/s). For instance, if `npc1` is currently on lane `123`, which is on the left of lane `124` in the figure above, the following code makes it change to lane `124` with a lateral velocity of `1.3` m/s when it reaches the point at offset 10m on lane `123`:
  ```python
  lane_124 = network.parse_lane(`124`)
  npc1.add_action(ChangeLane(
      next_lane=lane_124, 
      lateral_velocity=1.3, 
      condition=reach_point(LaneOffset('123', 10), network)))
  ```
- `FollowWaypoints`: Make the NPC vehicle follow a sequence of specified waypoints (3D positions). There are two ways to use this action:
  - Specify a waypoint sequence directly at the time of action initialization:
  ```python
  waypoints = [
      {"x": 10, "y": 5, "z": 0},
      {"x": 20, "y": 15, "z": 0},
      {"x": 30, "y": 25, "z": 0},
  ]
  npc1.add_action(FollowWaypoints(waypoints=waypoints))
  ```
  - There exists a case such that the waypoints cannot be determined at the time of action initialization, for example, when the waypoints depend on the current state (like position and velocity) of the (NPC) actor as well as the ego vehicle. In this case, we can provide a callback function that calculates the waypoints based on the current global state and the given actor. For example:
  ```python
  def cal_waypoints(actor, global_state):
      # Calculate waypoints based on the current state
      waypoints = []
      # ... populate waypoints ...
      return waypoints

  npc1.add_action(SpawnNPCVehicle(...))
  npc1.add_action(FollowLane(...))
  npc1.add_action(FollowWaypoints(
    waypoints_calculation_callback=cal_waypoints,
    condition=longitudinal_distance_to_ego <= dx0))
  ```

  This is a powerful action, and we argue that we can specify any desired trajectory for NPC vehicles using this action.

### Predefined Conditions
Some predefined conditions for action triggering are available in [core/trigger_condition.py](core/trigger_condition.py) file.
- `autonomous_mode_ready()`: Condition that checks whether the autonomous driving mode is ready in Autoware. It should be used for the `ActivateAutonomousMode` action as follows:
  ```python
  ego.add_action(ActivateAutonomousMode(
      condition=autonomous_mode_ready()))
  ```

- `end_lane(lane: str, network: Network)`: Condition that checks whether the actor has reached the end of the specified lane. An example usage has been shown in the `FollowLane` action description above.
- `reach_point(lane_offset: LaneOffset, network: Network)`: Condition that checks whether the actor has reached the specified point (given as lane offset). An example usage has been shown in the `ChangeLane` action description above.

- `distance_to_ego <comparison-operator> threshold`, where `comparison-operator` is either `<`, `<=`, `>`, or `>=`: Condition that checks the distance from the concern actor to the ego vehicle satisfies the given comparison with the specified threshold (in meters). For example, to trigger an action for the concern NPC actor when the ego vehicle is within `20` meters, we can use:
  ```python
  distance_to_ego <= 20
  ```
  Note that `distance_to_ego` does not account for the actor and ego vehicle's shape.

![Alt text](assets/distance_visual.png "Distance to Ego vs Longitudinal Distance to Ego")

- `longitudinal_distance_to_ego <comparison-operator> threshold`, where `comparison-operator` is either `<`, `<=`, `>`, or `>=`: Condition that checks the longitudinal distance from the concern actor to the ego vehicle satisfies the given comparison with the specified threshold (in meters). The longitudinal distance is calculated along the lane centerline of the concern actor, taking into account the actor and ego vehicle's length. The figure above illustrates the difference between `longitudinal_distance_to_ego` (left and middle figures) and `distance_to_ego` (right figure).

- `actor_speed <comparison-operator> threshold`: Condition that checks the speed of the concern actor satisfies the given comparison with the specified threshold (in m/s).
- `av_speed <comparison-operator> threshold`: Condition that checks the speed of the ego vehicle satisfies the given comparison with the specified threshold (in m/s).

### Define New Actions and Conditions
You can define new actions by extending the `Action` class (defined in [core/action.py](core/action.py) file), and implementing the `_do(actor, global_state)` method.  This method is called when the action is triggered for the given `actor`, with the current `global_state`. See the existing actions in [actions](actions) folder for reference.

Similarly, you can define new trigger conditions. If the condition involves numeric comparisons (like distance or speed comparisons), we recommend to use the predefined wrapper class `Measurement` (defined in [core/trigger_condition.py](core/trigger_condition.py) file) to implement the condition. See the existing conditions like `distance_to_ego` and `actor_speed` for reference.