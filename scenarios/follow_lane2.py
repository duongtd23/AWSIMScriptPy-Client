from core.scenario_manager import *
from core.trigger_condition import *

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

def follow_lane_scenario(network,
                     npc_init_laneoffset,
                     npc_speed=30/3.6,
                     acceleration=5,
                     body_style=BodyStyle.HATCHBACK):

    _, _, npc_init_pos, npc_init_orient = network.parse_lane_offset(npc_init_laneoffset)
    npc1 = NPCVehicle("npc1", body_style)

    init_lane = network.parse_lane(npc_init_laneoffset.lane_str)
    # npc and follow waypoints (cutin waypoints) specification
    npc1.add_action(SpawnNPCVehicle(position=npc_init_pos, orientation=npc_init_orient))
    npc1.add_action(FollowLane(target_speed=npc_speed,
                               acceleration=acceleration))
    # npc1.add_action(FollowLane('TrafficLane.335',
    #                            target_speed=3,
    #                            condition=complete_lane(init_lane)))
    npc1.add_action(SetTargetSpeed(target_speed=3,
                                   condition=speed < 8))

    return Scenario(network, [npc1])

if __name__ == '__main__':
    scenario_manager = ScenarioManager()
    scenario =  follow_lane_scenario(scenario_manager.network,
                                     npc_init_laneoffset=LaneOffset('124', 2),
                                     npc_speed=30 / 3.6,
                                     acceleration=3)
    scenario_manager.run([scenario])