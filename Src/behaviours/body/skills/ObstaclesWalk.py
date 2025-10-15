from BehaviourTask import BehaviourTask
from util.Constants import (
    PENALTY_CROSS_ABS_X, 
    HALF_FIELD_WIDTH, 
    HALF_FIELD_LENGTH, 
    PENALTY_BOX_LENGTH, 
    PENALTY_BOX_WIDTH, 
    GOAL_BOX_LENGTH, 
    GOAL_BOX_WIDTH
)
from util.Global import close_to_position, set_look_target
from util.Vector2D import Vector2D
from body.skills.WalkToPoint import WalkToPoint
from body.skills.Stand import Stand
from util import log


class ObstaclesWalk(BehaviourTask):
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
    
    OFFSET_DISTANCE = 700 # This is the distance we use to offset points
    _starting_position = Vector2D(PENALTY_CROSS_ABS_X, -HALF_FIELD_WIDTH - 500) # Starting posistion
                
    TARGET_POINTS = [
        Vector2D(PENALTY_CROSS_ABS_X, HALF_FIELD_WIDTH + 500),
        Vector2D(HALF_FIELD_LENGTH - (GOAL_BOX_LENGTH/2), GOAL_BOX_WIDTH/2),
        Vector2D(PENALTY_CROSS_ABS_X - OFFSET_DISTANCE, 0),
        Vector2D((HALF_FIELD_LENGTH - GOAL_BOX_LENGTH/2), -GOAL_BOX_WIDTH/2),
        Vector2D((HALF_FIELD_LENGTH - PENALTY_BOX_LENGTH - OFFSET_DISTANCE), -(PENALTY_BOX_WIDTH/2)),
    ]

    _walk_to_posistion = TARGET_POINTS[-1]
    
    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "WalkToPoint": WalkToPoint(self),
            "Stand": Stand(self)
        }

    def _reset(self):
        self._points_queue = self.TARGET_POINTS.copy()
        self._current_sub_task = "Stand"
        self._walk_to_posistion = self._points_queue.pop()

    #TODO: When a robot is detected, move to the side that we need to go around, in order to complete the leaderboard successfully
    def _transition(self):
        if close_to_position(self._walk_to_posistion):
            if len(self._points_queue) == 0:
                self._current_sub_task = "Stand"
                return
            else:
                self._walk_to_posistion = self._points_queue.pop()
        
        self._current_sub_task = "WalkToPoint"
        set_look_target(self._walk_to_posistion)
        
    def _tick(self):
        if self._current_sub_task == "WalkToPoint":
            self._tick_sub_task(final_pos = self._walk_to_posistion, prevent_leaving_field=False)
        else:
            self._tick_sub_task()
