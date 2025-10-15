from BehaviourTask import BehaviourTask
from body.skills.Stand import Stand
from body.skills.Walk import Walk
from util.Timer import Timer
import random

class Localise(BehaviourTask):
    """
    Localise behaviour that alternates between standing and turning.
    """
    TURN_TIME = 2
    TURN_SPEED = 0.8
    STAND_TIME = 1
    stand_timer = Timer(target_seconds=STAND_TIME)
    turn_timer = Timer(target_seconds=TURN_TIME)

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "Stand": Stand(self),
            "Turn": Walk(self)
        }

    def _transition(self):
        if self.stand_timer.finished() and self._current_sub_task == "Stand":
            self._current_sub_task = "Turn"
            self.turn_timer.restart().start()
            self.stand_timer.restart().stop()
        elif self.turn_timer.finished() and self._current_sub_task == "Turn":
            self._current_sub_task = "Stand"
            self.stand_timer.restart().start()
            self.turn_timer.restart().stop()

    def _reset(self):
        self._current_sub_task = "Stand"
        self.stand_timer.restart().start()

    def _tick(self):
        if self._current_sub_task == "Turn":
            self._tick_sub_task(turn=self.TURN_SPEED)
        else:
            self._tick_sub_task()
