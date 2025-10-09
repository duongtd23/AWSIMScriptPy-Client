from core.client_ros_node import AdsInternalStatus
import utils
import numpy as np

def longitudinal_distance_to_ego_less_than(threshold):
    def _cond(actor, global_state):
        if not global_state['actor-kinematics'] or not global_state['actor-sizes']:
            return False
        ego = global_state['actor-kinematics']['ego']
        ego_pos = np.array(ego['pose']['position'])
        ego_euler_angles = np.array(ego['pose']['rotation'])/180*np.pi

        npc = global_state['actor-kinematics']['vehicles'].get(actor.actor_id)
        if not npc:
            print(global_state['actor-kinematics'])
            print(f'[ERROR] NPC {actor.actor_id} not found')
            return False
        npc_pos = np.array(npc['pose']['position'])

        dis = utils.longitudinal_distance(ego_pos, npc_pos, ego_euler_angles)

        ego_root_to_front = global_state["actor-sizes"]["ego"]["size"][0]/2 + \
                            global_state["actor-sizes"]["ego"]["center"][0]
        npc_root_to_front = global_state["actor-sizes"][actor.actor_id]["size"][0]/2 + \
                            global_state["actor-sizes"][actor.actor_id]["center"][0]
        return dis - ego_root_to_front - npc_root_to_front <= threshold

    return _cond

def autonomous_mode_ready():
    def _cond(actor, global_state):
        return global_state['ads_internal_status'] >= AdsInternalStatus.AUTONOMOUS_MODE_READY.value
    return _cond