from BehaviourTask import BehaviourTask
from body.skills.LineupKick import LineupKick
from util.GameStatus import enemy_goal
from util.Constants import GOAL_WIDTH

class PenaltyKicker(BehaviourTask):


    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "LineupKick": LineupKick(self)
        }

    def _transition(self):
        pass

    def _reset(self):
        self._current_sub_task = "LineupKick"

    def _tick(self):
        target = enemy_goal()
        target.y = GOAL_WIDTH / 4
        self._tick_sub_task(target = target)
