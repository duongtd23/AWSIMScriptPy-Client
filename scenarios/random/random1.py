from core.scenario_manager import *

def complete_lane(lane):
    last_wp = lane.way_points[-1]
    def _cond(actor, global_state):
        if not global_state['actor-kinematics']:
            return False
        npc = global_state['actor-kinematics']['vehicles'].get(actor.actor_id)
        if not npc:
            print(f'[ERROR] NPC {actor.actor_id} not found')
            return False
        pos = np.array(npc['pose']['position'])
        front_center_point = actor.get_front_center(pos, npc['pose']['rotation'][2])
        return np.linalg.norm(front_center_point[:2] - last_wp[:2]) < 0.1
    return _cond

def make_scenario(network):
    def reach_point(point):
        if isinstance(point, LaneOffset):
            _, _, nppoint, _ = network.parse_lane_offset(point)
        elif isinstance(point, np.ndarray):
            nppoint = point
        else:
            raise TypeError('Unsupported point type')

        def _cond(actor, global_state):
            if not global_state['actor-kinematics']:
                return False

            npc = global_state['actor-kinematics']['vehicles'].get(actor.actor_id)
            if not npc:
                print(f'[ERROR] NPC {actor.actor_id} not found')
                return False
            pos = np.array(npc['pose']['position'])
            front_center_point = actor.get_front_center(pos, npc['pose']['rotation'][2])
            return np.linalg.norm(front_center_point[:2] - nppoint[:2]) < 0.1
        return _cond

    _, _, npc_init_pos, npc_init_orient = network.parse_lane_offset(LaneOffset('123'))
    npc1 = NPCVehicle("npc1", BodyStyle.SMALL_CAR)

    lane_124 = network.parse_lane('124')

    npc1.add_action(SpawnNPCVehicle(position=npc_init_pos, orientation=npc_init_orient))
    npc1.add_action(FollowLane(target_speed=20/3.6,
                               acceleration=7))
    npc1.add_action(ChangeLane(next_lane=lane_124, lateral_velocity=1.3,
                               condition=reach_point(LaneOffset('123', 12))))
    npc1.add_action(FollowLane('TrafficLane.335',
                               target_speed=3,
                               condition=complete_lane(lane_124)))
    # npc1.add_action(SetTargetSpeed(target_speed=1,
    #                                condition=npc_speed_gt(npc_speed-0.1)))

    return Scenario(network, [npc1])

if __name__ == '__main__':
    scenario_manager = ScenarioManager()
    scenario =  make_scenario(scenario_manager.network)
    scenario_manager.run([scenario])