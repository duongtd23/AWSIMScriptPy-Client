from core.scenario_manager import *
from core.trigger_condition import *

def make_cutout_scenario(network,
                     ego_init_laneoffset,
                     ego_goal_laneoffset,
                     cutout_next_lane,
                     _speed,
                     vy,
                     dx_f,
                     body_style=BodyStyle.SMALL_CAR):

    _, _, init_pos, init_orient = network.parse_lane_offset(ego_init_laneoffset)
    _, _, goal_pos, goal_orient = network.parse_lane_offset(ego_goal_laneoffset)

    # ego specification
    ego = EgoVehicle()
    ego.add_action(SpawnEgo(position=init_pos, orientation=init_orient))
    ego.add_action(SetGoalPose(position=goal_pos, orientation=goal_orient))
    ego.add_action(ActivateAutonomousMode(condition=autonomous_mode_ready()))
    ego.add_action(SetVelocityLimit(_speed))

    # cut-out NPC
    npc1 = NPCVehicle("npc1", BodyStyle.SMALL_CAR)
    npc1_root_to_rearcenter = npc1.size[0]/2 - npc1.center[0]

    # function to calculate the spawn pose of the NPC based on the AV's global state
    def npc1_pose_cal(actor, global_state):
        ego_kin = global_state['actor-kinematics']['ego']
        ego_pos = np.array(ego_kin['pose']['position'])
        ego_front_center = actor.get_front_center(ego_pos, ego_kin['pose']['rotation'][2])
        forward = (ego_front_center - ego_pos)[:2]
        forward = forward / np.linalg.norm(forward)
        npc_pos = ego_front_center[:2] + (2 * _speed + npc1_root_to_rearcenter) * forward
        orient = utils.quaternion_from_yaw(ego_kin['pose']['rotation'][2]/180*np.pi)
        return np.append(npc_pos, ego_front_center[2]), orient

    def spawned_cond(actor, global_state):
        if not global_state['actor-kinematics']:
            return False
        npc = global_state['actor-kinematics']['vehicles'].get(actor.actor_id)
        if not npc:
            return False
        return True

    # dynamically spawn the cut-out vehicle when AV speed >= _speed
    npc1.add_action(SpawnNPCVehicle(pose_callback=npc1_pose_cal,
                                    condition=av_speed >= _speed))
    # Set the moving speed without acceleration step 
    npc1.add_action(FollowLane(target_speed=_speed,
                               acceleration=500,
                               condition=spawned_cond))
    next_lane = network.parse_lane(cutout_next_lane)
    npc1.add_action(ChangeLane(next_lane=next_lane,
                                lateral_velocity=vy,
                                condition=actor_speed >= _speed))
    
    # stopped vehicle (challenging vehicle)
    npc2 = NPCVehicle("npc2", body_style)
    npc2_root_to_rearcenter = npc2.size[0]/2 - npc2.center[0]

    def npc2_pose_cal(actor, global_state):
        ego_kin = global_state['actor-kinematics']['ego']
        ego_pos = np.array(ego_kin['pose']['position'])
        ego_front_center = actor.get_front_center(ego_pos, ego_kin['pose']['rotation'][2])
        forward = (ego_front_center - ego_pos)[:2]
        forward = forward / np.linalg.norm(forward)
        npc_pos = ego_front_center[:2] + (2 * _speed + npc1.size[0] + dx_f + npc2_root_to_rearcenter) * forward
        orient = utils.quaternion_from_yaw(ego_kin['pose']['rotation'][2]/180*np.pi)
        return np.append(npc_pos, ego_front_center[2]), orient

    npc2.add_action(SpawnNPCVehicle(pose_callback=npc2_pose_cal,
                                    condition=av_speed >= _speed))

    return Scenario(network, [ego, npc1, npc2])

if __name__ == '__main__':
    scenario_manager = ScenarioManager()
    scenario =  make_cutout_scenario(scenario_manager.network,
                                    ego_init_laneoffset=LaneOffset('111', 0),
                                    ego_goal_laneoffset=LaneOffset('111', 210),
                                    _speed=30 / 3.6,
                                    cutout_next_lane='112',
                                    vy=1.5,
                                    dx_f=10.0
                                    )
    scenario_manager.run([scenario])