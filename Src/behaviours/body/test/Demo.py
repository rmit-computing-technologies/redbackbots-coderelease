import robot

from BehaviourTask import BehaviourTask
from body.skills.Kick import Kick
from body.skills.Stand import Stand
from body.skills.TurnToBall import TurnToBall
from body.skills.WalkToPoint import WalkToPoint
from head.HeadAware import HeadAware
from util.Global import ballWorldPos, ballDistance, ballRelPos


class Demo(BehaviourTask):
    """
    Go to the ball and kick it.
    """

    def _initialise_sub_tasks(self):

        # Turn localisation off
        HeadAware.localise = False
        
        # Set sub-tasks
        self._sub_tasks = {
            "WalkToBall" : WalkToPoint(self),
            "Stand" : Stand(self),
            "TurnToBall": TurnToBall(self),
            "Kick": Kick(self)
        }


    def _transition(self):
            if self._current_sub_task == "Kick" and not self._sub_tasks[self._current_sub_task].is_finished:
                self._current_sub_task = "Kick"

            if HeadAware._ball_lost.is_max():
                self._current_sub_task = "TurnToBall"

            elif ballDistance() > 200:
                self._current_sub_task = "WalkToBall"

            else:
                self._current_sub_task = "Kick"

    def _reset(self):
        self._current_sub_task = "TurnToBall"

    def _tick(self):
        if self._current_sub_task == "WalkToBall":
            self._tick_sub_task(final_pos=ballWorldPos(), speed=0.2, prevent_leaving_field=False)
        elif self._current_sub_task == "Kick" and ballRelPos().y < 0:
            self._tick_sub_task(kicking_foot = robot.Foot.RIGHT)
        else:
            self._tick_sub_task()