from math import radians
from BehaviourTask import BehaviourTask
from body.skills.Walk import Walk
from util.Global import myPos, myHeading
from util.Vector2D import Vector2D
from body.skills.WalkToPoint import WalkToPoint
from util.GameStatus import enemy_left_corner, enemy_right_corner,enemy_goal
from util.Global import ball_world_pos, close_to_position, not_close_to_position
from util.Constants import GOAL_BOX_LENGTH
from util.MathUtil import normalisedTheta

class WalkToCorner(BehaviourTask):
    """
    Description:
    A skill associated with walking to a corner point on the field.
    """
    WALK_SPEED = 300 # mm/s
    speed = 0.2
    TURN_RATE = 1.5 
    CORNER_BUFFER = 150
    
    TARGET_POINT = Vector2D(enemy_goal().x - GOAL_BOX_LENGTH, 0)

    _position_close = False
    
    ENEMY_LEFT_CORNER = Vector2D(enemy_left_corner().x - CORNER_BUFFER, enemy_left_corner().y - CORNER_BUFFER)
    ENEMY_RIGHT_CORNER = Vector2D(enemy_right_corner().x - CORNER_BUFFER, enemy_right_corner().y + CORNER_BUFFER)
    
    finalPos = Vector2D(0,0)
    
    _in_corner = False
    
    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "Walk": Walk(self),
            "WalkToPoint": WalkToPoint(self)        
        }
    
    def _reset(self):
        self._current_sub_task = "WalkToPoint"
        self._position_close = False

    def _transition(self):
        if self._position_close:
            self._current_sub_task = "Walk"

    def _tick(self):

        if not self._position_close and close_to_position(self.finalPos):
            self._position_close = True
        elif self._position_close and not_close_to_position(self.finalPos):
            self._position_close = False

        if self._current_sub_task == "WalkToPoint":
            # Goes to the Enemy Left Corner position
            if ball_world_pos().y > 0:
                self.finalPos = self.ENEMY_RIGHT_CORNER

            else:
                self.finalPos = self.ENEMY_LEFT_CORNER

            self._tick_sub_task(final_pos = self.finalPos, speed = self.speed)
        elif self._current_sub_task == "Walk":
            
            if self._heading_error() < radians(5):
                self._in_corner = True
            
            self._tick_sub_task(turn=self.TURN_RATE if self._heading_error() > 0
                else -self.TURN_RATE)

    def _heading_error(self):
        # Face the ball
        return normalisedTheta(
            (self.TARGET_POINT.minus(myPos())).heading() - myHeading())


        

    
