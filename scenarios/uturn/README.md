## U-turn Scenarios

This folder contains the specification in AWSIM-ScriptPy for U-turn scenarios from the [JAMA Standard](https://www.jama.or.jp/english/reports/docs/Automated_Driving_Safety_Evaluation_Framework_Ver3.0.pdf).
The following figure (from the JAMA Standard) illustrates an example of a U-turn scenario.

<img src="../../assets/fig-uturn.png" alt="drawing" width="240"/>

The ego vehicle (in blue) is traveling straight in its lane at a constant speed of $ve$.
An oncoming NPC vehicle (in green) approaches from the opposite direction at a constant speed of $vo$.
At the moment when the longitudinal distance between the two vehicles is $dx_0$, the NPC vehicle begins to make a U-turn maneuver.
The relevant parameters for this scenario are as follows:
- $ve$: Ego vehicle speed (m/s)
- $vo$: Oncoming vehicle speed (m/s)
- $dx_0$: Longitudinal distance between the two vehicles (m)
- $dy_0$: Lateral displacement between two vehicles before the U-turn (m). However, in the scenario implementation, we consider only two concrete values for $dy_0$ corresponding to two cases: (1) when the ego vehicle is on the rightmost lane and (2) when it is on the adjacent lane to the rightmost lane.

For each parameter setting of $ve$, $vo$, $dx_0$, and $dy_0$, based on the JAMA's good driver model, we can determine whether a collision can be avoided through braking alone or if a collision is unavoidable for an ideal ADS.
Readers are referred to the JAMA standard (Section 2.3.3.1) for more details.

### Scenario Specification in AWSIM-ScriptPy

Here, we provide AWSIM-ScriptPy implementations for several U-turn scenarios from the JAMA standard, such that they can be simulated in the Autoware-AWSIM-Labs environment to check the performance of Autoware.
The base implementation is in the [base.py](base.py) file, and specific parameter settings are in separate files such as [uturn_left_10.py](uturn_left_10.py).

The following code snippet (from [base.py](base.py)) shows the main idea of how to implement the JAMA's U-turn maneuver in AWSIM-ScriptPy:
```python
# function to dynamically calculate U-turn waypoints based on the world state
def cal_waypoints(actor, global_state):
    # extract the current position of the NPC vehicle and its front-center point
    npc_kin = global_state['actor-kinematics']['vehicles'].get(actor.actor_id)
    npc_pos = np.array(npc_kin['pose']['position'])
    npc_front_center = actor.get_front_center(npc_pos, npc_kin['pose']['rotation'][2])

    # The current front center point npc_front_center will be the first waypoint
    waypoints = [npc_front_center]

    # Calculate the other three waypoints to make the U-turn maneuver
    next_lane = network.parse_lane(uturn_next_lane)
    proj,_ = next_lane.project_point2D_onto_lane(npc_pos[:2])
    turning_center = (proj + npc_pos[:2])/2
    turning_side = utils.get_point_side(npc_pos[:2], npc_front_center[:2], turning_center)
    for i in range(1, 4):
        # these are three waypoints during the U-turn
        # the rotate_point is a helper function that rotates a point (npc_front_center) around a center point (turning_center) by a given angle (turning_side*i*np.pi/3)
        wp = utils.rotate_point(npc_front_center[:2], turning_center, turning_side*i*np.pi/3)
        waypoints.append(np.append(wp, npc_front_center[2]))

    # To make the NPC continue straight after the U-turn and make it steer back smoothly, we add one more waypoint
    direction = (npc_front_center - npc_pos)[:2]
    direction_normalized = direction / np.linalg.norm(direction)
    extended_wp = waypoints[-1][:2] - direction_normalized * utils.extended_point_scale(_npc_speed)
    waypoints.append(np.append(extended_wp, npc_front_center[2]))
    return [utils.array_to_dict_pos(p) for p in waypoints]

    # Convert waypoints to dict format required by FollowWaypoints action
    return [utils.array_to_dict_pos(p) for p in waypoints]

# npc specification
_, _, npc_init_pos, npc_init_orient = network.parse_lane_offset(npc_init_laneoffset)
npc1 = NPCVehicle("npc1", body_style)
npc1.add_action(SpawnNPCVehicle(position=npc_init_pos, orientation=npc_init_orient))
npc1.add_action(FollowLane(target_speed=_npc_speed,
                            acceleration=acceleration,
                            condition=av_speed >= _ego_speed - 0.2))
npc1.add_action(FollowWaypoints(waypoints_calculation_callback=cal_waypoints,
                                condition=longitudinal_distance_to_ego <= dx0))
```

In this code, after spawning the NPC vehicle, we let it follow its lane at a constant speed of `_npc_speed` until the ego vehicle's speed almost reaches its target speed `_ego_speed`.
Once the longitudinal distance between the NPC vehicle and the ego vehicle becomes less than or equal to `dx0`, the `FollowWaypoints` action is triggered, which makes the NPC vehicle perform the U-turn maneuver by following the dynamically calculated waypoints.
The key part of this specification is the `cal_waypoints` function, which computes the four waypoints dynamically for the U-turn maneuver based on the current position of the NPC vehicle.

The example above demonstrates that AWSIM-ScriptPy's flexible waypoint-based action system allows us to implement the U-turn maneuver easily.
Similar to the swerve scenarios, the only technical task is to implement the waypoint calculation logic encapsulated in the `cal_waypoints` function.

### Scenario Execution

This is a recorded video when the ego is in the rightmost lane


https://github.com/user-attachments/assets/b10bed76-2022-43d4-b584-6be8decae5c2



Below is a recorded video of  when the ego is in the adjacent lane to the rightmost lane



https://github.com/user-attachments/assets/9b19b8bc-27a8-4591-a0c1-3c948e20a5d6

