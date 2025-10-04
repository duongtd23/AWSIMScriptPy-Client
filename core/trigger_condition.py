from core.client_ros_node import AdsInternalStatus

def distance_to_ego_less_than(threshold):
    def _cond(actor, global_state):
        ego = global_state['ego']
        if not ego["position"] or not actor.state.get("position"):
            return False
        ex, ey, _ = ego.state['position']
        ax, ay, _ = actor.state['position']
        dx, dy = ex - ax, ey - ay
        return (dx*dx + dy*dy) ** 0.5 < threshold
    return _cond

def autonomous_mode_ready():
    def _cond(actor, global_state):
        return global_state['ads_internal_status'] >= AdsInternalStatus.AUTONOMOUS_MODE_READY
    return _cond