from BehaviourTask import BehaviourTask
from body.skills.TurnToBall import TurnToBall
from body.skills.WalkToPoint import WalkToPoint
from body.skills.Walk import Walk
from body.skills.Stand import Stand
from body.skills.Localise import Localise
from head.HeadAware import HeadAware
from util.Vector2D import Vector2D
from util.Global import ball_world_pos, myPos, we_see_ball, is_ball_lost, ball_distance, ball_heading, close_to_position, not_close_to_position
from util.Constants import FIELD_LENGTH, PENALTY_BOX_LENGTH
from util.GameStatus import own_goal, enemy_goal_heading, is_ball_in_attacking_half
from math import radians

class DefenceAngle(BehaviourTask):
    HEADING_ERROR = radians(30)

    _position_close = False
    SCALE_FACTOR = PENALTY_BOX_LENGTH / FIELD_LENGTH

    TURN_RATE = 1.5 

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "TurnToBall": TurnToBall(self),
            "WalkWithoutTurning": Walk(self),
            "Localise": Localise(self),
            "FaceForward": Walk(self),
            "Stand": Stand(self)
        }

    def _reset(self):
        self._current_sub_task = "WalkWithoutTurning"
        self._position_close = False
        self._target_pos = own_goal()
        self._ball_pos = ball_world_pos()
        self._tracking_ball = False

    def _transition(self):
        if HeadAware.high_uncertainty.is_max():
            self._current_sub_task = "Localise"
            return
        
        if is_ball_lost():
            self._tracking_ball = False
        
        if we_see_ball() or self._tracking_ball:
            self._tracking_ball = True

            if self._position_close:
                self._current_sub_task = "TurnToBall"
                return
            self._current_sub_task = "WalkWithoutTurning"
            return
        
        if abs(enemy_goal_heading()) < self.HEADING_ERROR:
            self._current_sub_task = "Stand"
            return
        
        self._current_sub_task = "FaceForward"

    def _tick(self, reference_point = own_goal(), default_x = None, dynamic = True):
        self._ball_pos = ball_world_pos()
        self._target_pos = reference_point

        # If ball is not lost, target_pos is between the ball and goal
        if self._tracking_ball:
            if is_ball_in_attacking_half() or not dynamic:
                pos_y = self.linear_interpolation(reference_point, self._ball_pos, default_x)
                self._target_pos = Vector2D(default_x, pos_y)
            else:
                # Revert to usual defence angle and move backward as ball moves closer to own goal
                self._target_pos = reference_point.minus((reference_point.minus(self._ball_pos)).scale(0.5))
        else:
            self._target_pos = Vector2D(default_x, reference_point.y)
        
        if not self._position_close and close_to_position(self._target_pos):
            self._position_close = True
        elif self._position_close and not_close_to_position(self._target_pos):
            self._position_close = False 

        if self._current_sub_task == "FaceForward":
            self._tick_sub_task(turn=self.TURN_RATE if enemy_goal_heading() > 0
                else -self.TURN_RATE)
        elif self._current_sub_task == "WalkWithoutTurning":
            x_diff = myPos().x - self._target_pos.x
            y_diff = myPos().y - self._target_pos.y
            
            ball_dist = ball_distance()
            if ball_dist< 1000:
                walk_modifier = 1
            else:
                walk_modifier = (ball_dist/500)-1

            if ball_dist < 4500: # 4500/3000 = 1.5 so reverts back to standard 
                turn_rate = ball_dist/3000
            else:
                turn_rate = self.TURN_RATE

            if abs(x_diff) < WalkToPoint.WALK_SPEED:
                walk_speed = abs(x_diff)
            else:
                walk_speed = WalkToPoint.WALK_SPEED / walk_modifier
            
            self._tick_sub_task(
                forward = -walk_speed if x_diff > 0 else walk_speed, 
                left = -WalkToPoint.WALK_SPEED/walk_modifier if y_diff > 0 else WalkToPoint.WALK_SPEED/walk_modifier, 
                speed = 1/walk_modifier, 
                turn=turn_rate if ball_heading() > 0 else -turn_rate
                )

        else:
            self._tick_sub_task()


    # Find the y coordinate along two points at the x value
    def linear_interpolation(self, pos1, pos2, default_x):
        new_pos_y = (pos1.y * (pos2.x - default_x) + pos2.y * (default_x - pos1.x)) / (pos2.x - pos1.x)
        return new_pos_y

            
