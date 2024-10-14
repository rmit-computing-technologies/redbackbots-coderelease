from BehaviourTask import BehaviourTask
from body.roles.Default import Default

class PositionSelector(BehaviourTask):
    """
        Descriptions:
        All robots have a player number assigned to them and a list of role priorities are
        defined.
        Appropriate python role are then specified as the subtask
    """

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "Default": Default(self)
        }

    def _reset(self):
        self._current_sub_task = "Default"

    def _transition(self):
        self._current_sub_task = "Default"

    def _tick(self):
        self._tick_sub_task()