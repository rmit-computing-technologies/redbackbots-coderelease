from math import radians
from BehaviourTask import BehaviourTask
from body.skills.Stand import Stand
from body.skills.Walk import Walk
from body.skills.WalkToPoint import WalkToPoint
from util.Global import myPos, close_to_position, not_close_to_position, is_robot_in_box, ego_ball_lost, ball_world_pos
from util.Constants import GOAL_BOX_LENGTH
from util.GameStatus import own_goal, enemy_goal
from util.FieldGeometry import heading_error
from util.Vector2D import Vector2D


class WalkToGoalBox(BehaviourTask):
    speed = 0.2
    HEADING_ERROR = radians(30)
    _position_close = False

    TURN_RATE = 1.5

    HALF_GOAL_BOX_LENGTH = GOAL_BOX_LENGTH/2

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "WalkToPoint": WalkToPoint(self),
            "Walk": Walk(self),
            "Stand": Stand(self)
        }

    def _reset(self):
        self._is_finished = False
        self._position_close = False
        self._current_sub_task = "WalkToPoint"
        self._target_pos = own_goal()

        self.ENEMY_GOAL = enemy_goal()

    def _transition(self):
        self._target_pos = own_goal()
        
        if self._position_close:
            if abs(heading_error(self.ENEMY_GOAL)) > self.HEADING_ERROR:
                self._current_sub_task = "Walk"
            else:
                self._current_sub_task = "Stand"
                self._is_finished = True

    def _tick(self, final_pos=None):
        if final_pos:
            self._target_pos = final_pos

        if not self._position_close and close_to_position(self._target_pos):
            self._position_close = True
        elif self._position_close and not_close_to_position(self._target_pos):
            self._position_close = False

        target_heading_pos = Vector2D(0, myPos().y)

        if self._current_sub_task == "WalkToPoint":
            if is_robot_in_box()[1]:
                self._tick_sub_task(
                    final_pos=self._target_pos, 
                    speed=self.speed, 
                    always_look_at_final=True, 
                    final_heading=(target_heading_pos.minus(myPos())).heading()
                    )
            else:
                self._tick_sub_task(
                    final_pos=self._target_pos, speed=self.speed)
        elif self._current_sub_task == "Walk":
            self._tick_sub_task(turn=self.TURN_RATE if heading_error(self.ENEMY_GOAL) > 0
                                else -self.TURN_RATE)
        else:
            self._tick_sub_task()
