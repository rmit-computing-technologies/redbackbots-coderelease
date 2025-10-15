from BehaviourTask import BehaviourTask
from body.skills.WalkToPoint import WalkToPoint
from util.GameStatus import own_goal
from util.Global import ball_world_pos


class WalkBetween(BehaviourTask):

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "WalkBetween": WalkToPoint(self),
        }

    
    def _reset(self):
        self._current_sub_task = "WalkBetween"

    
    def _transition(self):
        self._current_sub_task = "WalkBetween"


    def _tick(self):
        if self._current_sub_task == "WalkBetween":
            goal = own_goal()
            ball = ball_world_pos()
            add = goal.plus(ball)
            div = add.multiply(0.5)
            self._tick_sub_task(final_pos=div)