from BehaviourTask import BehaviourTask
from body.skills.Walk import Walk


class Turn(BehaviourTask):
    """
    Basic behaviour which makes the robot spin along the y-axis.
    Example uses:
    - Localising
    - Finding the ball
    """

    TURN_RATE = 1.5 

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "Walk": Walk(self)
        }

    def _reset(self):
        self._current_sub_task = "Walk"

    def _tick(self, left=False):
        self._tick_sub_task(0, 0, Turn.TURN_RATE if left else -Turn.TURN_RATE)
