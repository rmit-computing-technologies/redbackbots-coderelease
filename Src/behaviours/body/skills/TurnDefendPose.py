import util.actioncommand as ac
from BehaviourTask import BehaviourTask
from body.skills.TurnToBall import TurnToBall
from body.skills.Stand import Stand
from head.HeadAware import HeadAware
from util.Global import ball_heading, ball_distance, ego_see_ball
from math import radians
from util.Timer import Timer


class TurnDefendPose(BehaviourTask):
    HEADING_ERROR_TO_ADJUST = radians(15)

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "TurnToBall": TurnToBall(self),
            "Stand": Stand(self)
        }

    def _reset(self):
        self._current_sub_task = "TurnToBall"
        self._timer = Timer(1000000)
        self._timer2 = Timer(1000000)
        self._timer_running = False
        self._timer2_running = False
        self._aligned_to_ball = False
        self._first = True


    def _tick(self):
        if abs(ball_heading()) < self.HEADING_ERROR_TO_ADJUST and ball_distance() < 1000 and ego_see_ball() and not self._timer_running:
            self._timer.restart().start()
            self._timer_running = True

        if self._timer_running and not self._timer.finished():
            self._current_sub_task = "Stand"
            self._tick_sub_task()
        elif self._timer_running and self._timer.finished():
            self._timer_running = False
            self.world.b_request.actions.body = ac.defenderCentre()
            self._timer2.restart().start()
            self._timer2_running = True
        elif not self._timer2_running:
            self._current_sub_task = "TurnToBall"
            self._tick_sub_task()
            