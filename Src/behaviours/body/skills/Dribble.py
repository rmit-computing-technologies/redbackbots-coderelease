from BehaviourTask import BehaviourTask
from body.skills.Lineup import Lineup, AlignmentVectors
from body.skills.Walk import Walk
from util.Global import ball_world_pos, myPos, myHeading
from util.Hysteresis import Hysteresis
from util.MathUtil import normalisedTheta
from util import log

class Dribble(BehaviourTask):

    """ 
        Description:
        This skill is associated with dribbling the ball acorss the field. 
        it includes lining upto the ball as well as detecting if the ball is moving. 
    """
    MAX_TICKS_OUTSIDE_OF_IDEAL_LINEUP = 50
    SPEED = 0.2
    MAX_RADIANS_OUTSIDE_OF_IDEAL_LINEUP = 0.1

    _outside_lineup_path = Hysteresis(min_value=0, max_value=MAX_TICKS_OUTSIDE_OF_IDEAL_LINEUP)

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "Dribble": Walk(self),
            "Lineup": Lineup(self),
        }

    def _transition(self):
        lineup_error = AlignmentVectors(dribble=True).get_lineup_error()

        if (abs(lineup_error) > self.MAX_RADIANS_OUTSIDE_OF_IDEAL_LINEUP):
            self._outside_lineup_path.up()
        else:
            self._outside_lineup_path.down()

        # If the robot isn't aligned nicely on it's y axis (ie: ball in front), or if the ball has moved too far outside, we can't dribble since our position is likely off
        if (abs(lineup_error) > self.MAX_RADIANS_OUTSIDE_OF_IDEAL_LINEUP and self._current_sub_task=="Lineup") or self._outside_lineup_path.is_max():
            self._current_sub_task = "Lineup"
        else:
            self._current_sub_task = "Dribble"

    def _reset(self):
        self._current_sub_task = "Lineup"
        self._outside_lineup_path.reset()

    def _tick(self):
        alignment_vectors = AlignmentVectors(dribble=True)
        move_vec = alignment_vectors.get_move_vector()
        lineup_error = alignment_vectors.get_lineup_error()

        if self._current_sub_task == "Dribble":
            # move_vec = ballWorldPos().minus(myPos()).rotate(-myHeading()).multiply(1.5)
            self._tick_sub_task(move_vec.x, move_vec.y, lineup_error, speed=self.SPEED)
        else:
            self._tick_sub_task(dribble=True)