from BehaviourTask import BehaviourTask
from body.skills.Walk import Walk
from util.Global import ball_rel_pos


class WalkSideways(BehaviourTask):

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "Walk": Walk(self)
        }

    def _reset(self):
        self._current_sub_task = "Walk"

    def _transition(self):
        self._current_sub_task = "Walk"

    def _tick(self):
        if self._current_sub_task == "Walk":
            if ball_rel_pos()[1] < 0:
                # side step right
                self._tick_sub_task(0, -100, 0)
            else:
                # side step left
                self._tick_sub_task(0, 100, 0)

            