import robot

from BehaviourTask import BehaviourTask
from body.skills.Kick import Kick
from body.skills.Stand import Stand
from body.skills.TurnToBall import TurnToBall
from body.skills.WalkToPoint import WalkToPoint
from body.test.WalkAround import WalkAround
from head.HeadAware import HeadAware
from util.Global import ball_world_pos, ball_distance, ball_rel_pos, ego_ball_lost # <-- util.Global has some interesting functions
from util.Vector2D import Vector2D


class Demo(BehaviourTask):
    """
    Go to the ball and kick it.
    """

    def _initialise_sub_tasks(self): # <-- this function defines which child skills this skill will call
        # Set sub-tasks
        # Note: the name of the substask CAN be different to the skill it is calling
        self._sub_tasks = {
            "WalkToCentre" : WalkToPoint(self),
            "WalkAround" : WalkAround(self),
            "Stand" : Stand(self),
            "TurnToBall": TurnToBall(self),
            "Kick": Kick(self)
        }


    def _transition(self): # <-- this function determines which of the above child skill we want to run, this runs every tick
            if not ego_ball_lost():
                self._current_sub_task = "Stand"
            else:
                self._current_sub_task = "WalkAround"

    def _reset(self): # <-- this function runs when the skill is first called
        self._current_sub_task = "WalkAround"

    def _tick(self): # <--  this function runs every tick and is where you can pass params to child skills
                     #      make sure self._tick_sub_task() is always called, if it doesn't get called the
                     #      robot might fall over!
        if self._current_sub_task == "WalkToCentre": # <-- We never change to this WalkToCentre OR Kick in _transition, I wonder what will happen if we do
            self._tick_sub_task(final_pos=Vector2D(0,0), speed=0.2)
        elif self._current_sub_task == "Kick" and ball_rel_pos().y < 0:
            self._tick_sub_task(kicking_foot = robot.Foot.RIGHT)
        else:
            self._tick_sub_task()
