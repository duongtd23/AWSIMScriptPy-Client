from abc import ABC, abstractmethod

class Action(ABC):
    def __init__(self, condition=None, one_shot=True):
        self.condition = condition
        self.one_shot = one_shot

    def should_trigger(self, actor, global_state):
        return self.condition is None or self.condition(actor, global_state)

    def execute(self, actor, global_state):
        if self.should_trigger(actor, global_state):
            self._do(actor)
            return True
        return False

    @abstractmethod
    def _do(self, actor):
        pass
