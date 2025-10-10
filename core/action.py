from abc import ABC, abstractmethod

class Action(ABC):
    def __init__(self, condition=None, one_shot=True, callback=None):
        self.condition = condition
        self.one_shot = one_shot
        self.callback = callback

    def should_trigger(self, actor, global_state):
        return self.condition is None or self.condition(actor, global_state)

    def execute(self, actor, global_state, client_node):
        if self.should_trigger(actor, global_state):
            self._do(actor, client_node)
            if self.callback is not None:
                self.callback(self, actor)
            return True
        return False

    @abstractmethod
    def _do(self, actor, client_node):
        pass
