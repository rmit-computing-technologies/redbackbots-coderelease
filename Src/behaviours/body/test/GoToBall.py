from BehaviourTask import BehaviourTask
from body.skills.Stand import Stand
from body.skills.TurnToBall import TurnToBall
from body.skills.WalkToPoint import WalkToPoint
from head.HeadAware import HeadAware
from util.Global import ball_world_pos, ball_distance, is_ball_lost


class GoToBall(BehaviourTask):
    """
    Go to the ball and stand behind it.
    """

    """
        Description:
        This skill is associated with walking towards the ball, including 
        turning towards the ball if it is not in direct line form the robot.
    """

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "WalkToBall" : WalkToPoint(self),
            "Stand" : Stand(self),
            "TurnToBall": TurnToBall(self)
        }


    def _transition(self):
            if ball_distance() > 200:
                self._current_sub_task = "WalkToBall"

            elif is_ball_lost():
                self._current_sub_task = "TurnToBall"

            else:
                self._current_sub_task = "Stand"
  
    def _reset(self):
        self._current_sub_task = "Stand"


    def _tick(self):
        if self._current_sub_task == "WalkToBall":
            self._tick_sub_task(final_pos=ball_world_pos(), speed=0.2)

        else:
            self._tick_sub_task()