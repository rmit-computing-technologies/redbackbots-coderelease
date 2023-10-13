import util.actioncommand as ac
from BehaviourTask import BehaviourTask
from body.skills.Turn import Turn
from body.skills.Walk import Walk
from head.HeadAware import HeadAware
from util.Global import canSeeBall, ballHeading, ballDistance
from math import radians


class TurnToBall(BehaviourTask):
    HEADING_ERROR_TO_ADJUST = radians(15)
    TURN_RATE = 1.5

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "Walk": Walk(self),
            "Turn": Turn(self)
        }

    def _reset(self):
        self._current_sub_task = "Walk"

    def _transition(self):
        if HeadAware._ball_lost.is_max():
            self._current_sub_task = "Turn"
        else:
            self._current_sub_task = "Walk"

    def _tick(self):
        if self._current_sub_task == "Walk":
            if abs(ballHeading()) >= self.HEADING_ERROR_TO_ADJUST:
                self._tick_sub_task(0, 0, turn=Turn.TURN_RATE if ballHeading() > 0 else -Turn.TURN_RATE)
        else:
            self._tick_sub_task()