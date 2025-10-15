from BehaviourTask import BehaviourTask
from body.skills.Turn import Turn
from body.skills.Walk import Walk
from util.Global import ball_heading, is_ball_lost, ball_distance
from math import radians


class TurnToBall(BehaviourTask):
    HEADING_ERROR_TO_ADJUST = radians(20)
    TURN_RATE = 0.5


    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "Walk": Walk(self),
            "Turn": Turn(self)
        }


    def _reset(self):
        self._current_sub_task = "Walk"


    def _transition(self):
        if is_ball_lost():
            self._current_sub_task = "Turn"
        else:
            self._current_sub_task = "Walk"


    def _tick(self):
        if self._current_sub_task == "Walk":
            turn_rate = TurnToBall.get_turn_rate()

            if abs(ball_heading()) > self.HEADING_ERROR_TO_ADJUST:
                self._tick_sub_task(0, 0, turn=turn_rate if ball_heading() > 0 else -turn_rate)
        else:
            self._tick_sub_task()


    @staticmethod
    def get_turn_rate(normal=False):
        if normal:
            return TurnToBall.TURN_RATE
        
        ball_dist = ball_distance()
        # Fine-tune the turn rate if the ball is closer
        if ball_dist < 1500: # 1500/3000 = 0.5 so reverts back to standard
            return ball_dist/3000
        else:
            return TurnToBall.TURN_RATE
