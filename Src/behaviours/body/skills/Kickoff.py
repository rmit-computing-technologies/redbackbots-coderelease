from BehaviourTask import BehaviourTask
from body.skills.Dribble import Dribble
from body.skills.LineupKick import LineupKick
from body.skills.TurnToBall import TurnToBall
from util.Global import is_ball_lost

class Kickoff(BehaviourTask):

    """ 
        Description:
        This class contains the actions that would be carried out during kickoff.
        The kickoff robot will dribble the ball towards its own team while the attack kick off timer is running.
    """

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "Dribble": Dribble(self),
            "TurnToBall": TurnToBall(self)
        }


    def _transition(self):
        if not is_ball_lost():
            # Dribble the ball towards its own team
            self._current_sub_task = "Dribble"
                
        # If the ball is lost.
        else:
            self._current_sub_task = "TurnToBall"


    def _reset(self):
        self._current_sub_task = "TurnToBall"
        self._field_position = ("Superstar", None)


    def _tick(self, field_position = ("Superstar", None)):
        self.field_position = field_position
        self._tick_sub_task()