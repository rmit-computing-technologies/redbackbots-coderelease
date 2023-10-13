from BehaviourTask import BehaviourTask
from body.test.WalkAround import WalkAround


class Default(BehaviourTask):

    """ 
        Descriptions:
        A basic skill that is making the robot walk around the field
    """

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "WalkAround": WalkAround(self)
        }

    def _reset(self):
        self._current_sub_task = "WalkAround"
