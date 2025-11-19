from core.scenario_manager import *
from core.trigger_condition import *

def make_deceleration_scenario(network,
                     ego_init_laneoffset,
                     ego_goal_laneoffset,
                     npc_init_laneoffset,
                     _speed,
                     body_style=BodyStyle.HATCHBACK):

    _, _, init_pos, init_orient = network.parse_lane_offset(ego_init_laneoffset)
    _, _, goal_pos, goal_orient = network.parse_lane_offset(ego_goal_laneoffset)

    # ego specification
    ego = EgoVehicle()
    ego.add_action(SpawnEgo(position=init_pos, orientation=init_orient))
    ego.add_action(SetGoalPose(position=goal_pos, orientation=goal_orient))
    ego.add_action(ActivateAutonomousMode(condition=autonomous_mode_ready()))
    ego.add_action(SetVelocityLimit(_speed,one_shot=True))

    # NPC specification
    _, _, npc_init_pos, npc_init_orient = network.parse_lane_offset(npc_init_laneoffset)
    npc1 = NPCVehicle("npc1", body_style)
    npc_root_to_back = npc1.size[0]/2 - npc1.center[0]

    # function to calculate the spawn pose of the NPC based on the AV's global state
    def pose_cal(actor, global_state):
        ego_kin = global_state['actor-kinematics']['ego']
        ego_pos = np.array(ego_kin['pose']['position'])
        ego_front_center = actor.get_front_center(ego_pos, ego_kin['pose']['rotation'][2])
        forward = (ego_front_center - ego_pos)[:2]
        forward = forward / np.linalg.norm(forward)
        npc_pos = ego_front_center[:2] + (2 * _speed + npc_root_to_back) * forward
        orient = utils.quaternion_from_yaw(ego_kin['pose']['rotation'][2]/180*np.pi)
        return np.append(npc_pos, ego_front_center[2]), orient

    # dynamically spawn the NPC when AV speed >= _speed
    npc1.add_action(SpawnNPCVehicle(pose_callback=pose_cal,
                                    condition=av_speed >= _speed))
    # Set the moving speed without acceleration step 
    npc1.add_action(FollowLane(target_speed=_speed,
                               acceleration=500))
    npc1.add_action(SetTargetSpeed(target_speed=0,
                                   acceleration=-9.8,
                                   condition=actor_speed >= _speed))

    return Scenario(network, [ego, npc1])

if __name__ == '__main__':
    scenario_manager = ScenarioManager()
    scenario =  make_deceleration_scenario(scenario_manager.network,
                                    ego_init_laneoffset=LaneOffset('111', 0),
                                    ego_goal_laneoffset=LaneOffset('111', 210),
                                    npc_init_laneoffset=LaneOffset('111', 70),
                                    _speed=30 / 3.6
                                    )
    scenario_manager.run([scenario])