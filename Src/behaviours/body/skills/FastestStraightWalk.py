from BehaviourTask import BehaviourTask
from body.skills.WalkToPoint import WalkToPoint
from body.skills.Stand import Stand
from util.Constants import HALF_FIELD_WIDTH
from util.Global import myPos
from util import log

class FastestStraightWalk(BehaviourTask):
    """
    This skill if for the fastest walk Obsticle avoidance leaderboard challenge.
    
    Please note that you'll need to change the redbackbots cfg file to have the following state estimistation settings:
                [stateestimation]
                # initial_pose_type=GAME
                initial_pose_type=SPECIFIED
                # HALF FIELD
                # specified_initial_x=2004
                # specified_initial_y=-2360
                # FULL FIELD
                specified_initial_x=3200
                specified_initial_y=-3500
                # initial_pose_type=UNPENALISED
                # specified_initial_x=-2850
                # specified_initial_y=0
                specified_initial_theta=90
    """
    
    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "WalkToPoint": WalkToPoint(self),
            "Stand": Stand(self)
        }

    def _transition(self):
        if myPos().y >= HALF_FIELD_WIDTH + 500:
            self._current_sub_task = "Stand"
        else:
            self._target_pos.y = myPos().y + 1000
            self._current_sub_task = "WalkToPoint"

    def _reset(self):
        self._current_sub_task = "Stand"
        self._inital_pos = myPos()
        self._target_pos = myPos()

    def _tick(self):
        if self._current_sub_task == "WalkToPoint":
            self._tick_sub_task(final_pos=self._target_pos, speed=1, prevent_leaving_field=True)
