from BehaviourTask import BehaviourTask
from body.skills.Turn import Turn
from body.skills.WalkToPoint import WalkToPoint
from head.HeadAware import HeadAware
from util.Global import myPos
from util.Vector2D import Vector2D
from util.Constants import NOT_CLOSE_TO_POSITION_DISTANCE


# A simple task making the robot walk along the center line.
# This isn't designed to be practical, but more of a test of our localisation system
class WalkAlongCentre(BehaviourTask):
    _y = -1000

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "WalkToPoint": WalkToPoint(self),
            "Localise": Turn(self),
        }

    def _reset(self):
        self._current_sub_task = "WalkToPoint"

    def _transition(self):
        if HeadAware.high_uncertainty.is_max():
            self._current_sub_task = "Localise"
        else:
            self._current_sub_task = "WalkToPoint"

    def _tick(self):
        target = Vector2D(0, self._y)
        vector = target.minus(myPos())
        dist_to_target_sq = vector.length2()
        if self._y > 1300:
            # Celebrate
            self._y=-1000
            return
        if self._current_sub_task == "WalkToPoint":
            self._tick_sub_task(final_pos=target)
        else:
            self._tick_sub_task()
            return
        # if distance to target is small, update target
        if dist_to_target_sq < NOT_CLOSE_TO_POSITION_DISTANCE ** 2:
            self._y += 100
            print("new target:", self._y)
