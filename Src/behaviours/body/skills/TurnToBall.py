from BehaviourTask import BehaviourTask
from body.skills.Turn import Turn
from body.skills.Walk import Walk
from util.Global import ballHeading, isBallLost, ballDistance, canSeeBall
from math import radians
from util import Log


class TurnToBall(BehaviourTask):
    HEADING_ERROR_TO_ADJUST = radians(20)
    TURN_RATE = 0.5
    BALL_LOST_TIME = 3
    FRAMES_SEEN_BALL_FOR_SEEN = 2

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "Walk": Walk(self),
            "Turn": Turn(self)
        }

    def _reset(self):
        self._current_sub_task = "Walk"

    def _transition(self):
        # Turn until can see ball for 2 frames
        if isBallLost(self.BALL_LOST_TIME) and not canSeeBall(self.FRAMES_SEEN_BALL_FOR_SEEN):
            self._current_sub_task = "Turn"
        else:
            self._current_sub_task = "Walk"

    def _tick(self):
        if self._current_sub_task == "Walk":
            turn_rate = TurnToBall.get_turn_rate()

            if abs(ballHeading()) > self.HEADING_ERROR_TO_ADJUST:
                self._tick_sub_task(0, 0, turn=turn_rate if ballHeading() > 0 else -turn_rate)
        else:
            self._tick_sub_task()
    
    @staticmethod
    def get_turn_rate(normal=False):
        if normal:
            return TurnToBall.TURN_RATE
        
        ball_distance = ballDistance()
        # Fine-tune the turn rate if the ball is closer
        if ball_distance < 1500: # 1500/3000 = 0.5 so reverts back to standard
            return ball_distance/3000
        else:
            return TurnToBall.TURN_RATE
